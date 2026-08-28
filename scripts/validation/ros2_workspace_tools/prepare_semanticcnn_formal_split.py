#!/usr/bin/env python3
"""Create a full, bag-disjoint SemanticCNN train/dev/test dataset view.

The source sessions must already be sealed with ``CNN_READY.json``.  Large
per-frame array directories are not copied: the derived sessions use absolute
directory symlinks to the immutable source sessions and own only their split
lists and provenance metadata.  The output is assembled in a temporary sibling
directory and atomically renamed into place after all internal checks pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import tempfile
from pathlib import Path

import numpy as np


SPLITS = ("train", "dev", "test")
SEQ_LEN = 10
REQUIRED_ARRAY_DIRS = (
    "scans_lidar",
    "intensities_lidar",
    "angles_lidar",
    "virtual_ranges_lidar",
    "virtual_angles_lidar",
    "range_valid_mask_lidar",
    "self_mask_lidar",
    "valid_mask_lidar",
    "semantic_label",
    "source_sensor",
    "raw_beam_index",
    "positions",
    "velocities",
    "cmd_velocities",
    "sub_goals_local",
)
OPTIONAL_SOURCE_FILES = (
    "CNN_READY.json",
    "quality_report.json",
    "source_QUALITY_PASS.json",
    "capture_contract.json",
)
EXACT_CONTRACT_KEYS = (
    "format",
    "samples_01",
    "samples_02",
    "total_slots",
    "slot_contract",
    "semantic_cnn_pool_mode",
    "pool_num_bins",
    "pool_range_normalization",
    "self_mask_mode",
    "forward_only",
    "reverse_recovery_frames",
    "label_names",
    "cmd_label_interface",
    "cmd_vel_angular_z_relay_scale",
    "subgoal_source",
    "subgoal_lookahead",
)
FLOAT_CONTRACT_KEYS = (
    "range_max_01",
    "range_max_02",
    "pool_range_max",
    "pool_angle_min",
    "pool_angle_max",
    "expected_frame_period_ms",
    "scan_02_expected_frame_period_ms",
    "frame_period_tolerance_ms",
    "reverse_linear_x_epsilon",
    "cmd_vel_max_age_ms",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--train-session", action="append", required=True)
    parser.add_argument("--dev-session", action="append", required=True)
    parser.add_argument("--test-session", action="append", required=True)
    return parser.parse_args()


def read_lines(path: Path) -> list[str]:
    if not path.is_file():
        raise FileNotFoundError(f"required list does not exist: {path}")
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def write_lines(path: Path, values: list[str]) -> None:
    path.write_text("".join(f"{value}\n" for value in values), encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_sample_names(source: Path) -> list[str]:
    names: list[str] = []
    for split in SPLITS:
        names.extend(read_lines(source / f"{split}.txt"))
    if len(names) != len(set(names)):
        raise ValueError(f"source split lists contain duplicate filenames: {source}")
    if not names:
        raise ValueError(f"source session contains no samples: {source}")
    return names


def source_bag_path(metadata: dict) -> str:
    source_bag = str(metadata.get("source_bag", metadata.get("bag", "")))
    source_npz = metadata.get("source_npz_session")
    if not source_bag and source_npz:
        source_npz_metadata = Path(str(source_npz)) / "metadata.json"
        if source_npz_metadata.is_file():
            source_bag = str(
                json.loads(source_npz_metadata.read_text(encoding="utf-8")).get("bag", "")
            )
    return source_bag


def contract_differences(reference: dict, candidate: dict) -> list[str]:
    differences = [
        key for key in EXACT_CONTRACT_KEYS if reference.get(key) != candidate.get(key)
    ]
    for key in FLOAT_CONTRACT_KEYS:
        left = reference.get(key)
        right = candidate.get(key)
        if left is None and right is None:
            continue
        try:
            matches = math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-6)
        except (TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(key)
    return differences


def valid_window_end_names(metadata: dict, selected_names: set[str]) -> list[str]:
    records = metadata.get("frames", [])
    expected_ms = float(metadata.get("expected_frame_period_ms", 65.0))
    tolerance_ms = float(metadata.get("frame_period_tolerance_ms", 20.0))
    valid: list[str] = []
    for end in range(SEQ_LEN - 1, len(records)):
        window = records[end - SEQ_LEN + 1 : end + 1]
        names = [str(record["name"]) for record in window]
        if any(name not in selected_names for name in names):
            continue
        episode_ids = {int(record.get("episode_id", -1)) for record in window}
        if len(episode_ids) != 1 or -1 in episode_ids:
            continue
        stamps = [int(record["scan_01_stamp_ns"]) for record in window]
        deltas_ms = [
            (right - left) / 1_000_000.0 for left, right in zip(stamps, stamps[1:])
        ]
        if all(abs(delta - expected_ms) <= tolerance_ms for delta in deltas_ms):
            valid.append(names[-1])
    return valid


def vector_stats(values: list[np.ndarray]) -> dict:
    if not values:
        raise ValueError("cannot compute statistics for an empty value list")
    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError("training normalization input contains non-finite values")
    return {
        "count": int(array.shape[0]),
        "mean": array.mean(axis=0).tolist(),
        "std_population": array.std(axis=0).tolist(),
        "min": array.min(axis=0).tolist(),
        "max": array.max(axis=0).tolist(),
    }


def validate_source(source: Path) -> tuple[dict, list[str], list[str]]:
    if not (source / "CNN_READY.json").is_file():
        raise FileNotFoundError(f"source is not sealed with CNN_READY.json: {source}")
    metadata_path = source / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("format") != "semantic2d-fixed-dual-native-v3":
        raise ValueError(f"unsupported source format: {source}")
    for directory in REQUIRED_ARRAY_DIRS:
        if not (source / directory).is_dir():
            raise FileNotFoundError(f"source session is missing {directory}/: {source}")
    names = source_sample_names(source)
    frame_names = {str(record["name"]) for record in metadata.get("frames", [])}
    if set(names) != frame_names:
        raise ValueError(f"split lists and metadata.frames differ: {source}")
    ready = json.loads((source / "CNN_READY.json").read_text(encoding="utf-8"))
    if int(ready.get("sample_count", -1)) != len(names):
        raise ValueError(f"CNN_READY sample_count differs from split lists: {source}")
    for directory in REQUIRED_ARRAY_DIRS:
        actual = len(list((source / directory).glob("*.npy")))
        if actual != len(names):
            raise ValueError(
                f"{source.name}/{directory} has {actual} arrays, expected {len(names)}"
            )
    windows = valid_window_end_names(metadata, set(names))
    if not windows:
        raise ValueError(f"source has no valid {SEQ_LEN}-frame SemanticCNN window: {source}")
    return metadata, names, windows


def materialize_session(
    source: Path,
    output_root: Path,
    split: str,
    metadata: dict,
    names: list[str],
    window_ends: list[str],
) -> dict:
    derived_name = f"{source.name}-formal-{split}"
    derived = output_root / derived_name
    derived.mkdir()
    for directory in REQUIRED_ARRAY_DIRS:
        os.symlink(source / directory, derived / directory, target_is_directory=True)
    for current_split in SPLITS:
        write_lines(derived / f"{current_split}.txt", names if current_split == split else [])
    shutil.copy2(source / "label_names.txt", derived / "label_names.txt")
    for filename in OPTIONAL_SOURCE_FILES:
        target = source / filename
        if target.is_file():
            os.symlink(target, derived / f"source_{filename}")

    source_bag = source_bag_path(metadata)
    derived_metadata = dict(metadata)
    derived_metadata.update(
        {
            "session_name": derived_name,
            "split_role": split,
            "formal_split_view": True,
            "formal_split_source_session": str(source),
            "formal_split_source_bag": source_bag,
            "formal_split_selected_samples": len(names),
            "formal_split_valid_windows": len(window_ends),
        }
    )
    (derived / "metadata.json").write_text(
        json.dumps(derived_metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return {
        "derived_session": derived_name,
        "source_session": source.name,
        "source_path": str(source),
        "source_bag": source_bag,
        "source_cnn_ready_sha256": sha256_file(source / "CNN_READY.json"),
        "split": split,
        "samples": len(names),
        "episodes": sorted({int(record["episode_id"]) for record in metadata["frames"]}),
        "valid_windows": len(window_ends),
    }


def main() -> None:
    args = parse_args()
    source_root = args.source_root.resolve(strict=True)
    output_root = args.output_root.resolve()
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite formal dataset: {output_root}")

    requested = {
        "train": args.train_session,
        "dev": args.dev_session,
        "test": args.test_session,
    }
    requested_names = [name for split in SPLITS for name in requested[split]]
    if len(requested_names) != len(set(requested_names)):
        raise ValueError("every source session must belong to exactly one split")

    validated: dict[str, tuple[Path, dict, list[str], list[str]]] = {}
    reference_metadata: dict | None = None
    reference_name = ""
    all_filenames: set[str] = set()
    resolved_sources: set[Path] = set()
    source_bags: set[str] = set()
    for name in requested_names:
        source = (source_root / name).resolve(strict=True)
        if source in resolved_sources:
            raise ValueError(f"multiple session names resolve to the same source: {source}")
        resolved_sources.add(source)
        metadata, names, window_ends = validate_source(source)
        source_bag = source_bag_path(metadata)
        if not source_bag:
            raise ValueError(f"cannot prove bag isolation because source_bag is missing: {source}")
        if source_bag in source_bags:
            raise ValueError(f"source bag is assigned more than once: {source_bag}")
        source_bags.add(source_bag)
        if reference_metadata is None:
            reference_metadata = metadata
            reference_name = name
        else:
            differences = contract_differences(reference_metadata, metadata)
            if differences:
                raise ValueError(
                    f"incompatible source contracts: {reference_name} vs {name}: "
                    + ", ".join(differences)
                )
        overlap = all_filenames.intersection(names)
        if overlap:
            raise ValueError(f"cross-session filename leakage detected: {sorted(overlap)[:3]}")
        all_filenames.update(names)
        validated[name] = (source, metadata, names, window_ends)

    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{output_root.name}.tmp-", dir=str(output_root.parent))
    )
    try:
        reports: list[dict] = []
        train_subgoals: list[np.ndarray] = []
        train_targets: list[np.ndarray] = []
        for split in SPLITS:
            for name in requested[split]:
                source, metadata, names, window_ends = validated[name]
                reports.append(
                    materialize_session(
                        source, temporary, split, metadata, names, window_ends
                    )
                )
                if split == "train":
                    for end_name in window_ends:
                        subgoal = np.asarray(
                            np.load(source / "sub_goals_local" / end_name), dtype=np.float64
                        ).reshape(-1)
                        command = np.asarray(
                            np.load(source / "cmd_velocities" / end_name), dtype=np.float64
                        ).reshape(-1)
                        if subgoal.shape != (2,):
                            raise ValueError(f"sub-goal shape must be (2,): {source}/{end_name}")
                        if command.shape != (3,):
                            raise ValueError(f"command shape must be (3,): {source}/{end_name}")
                        train_subgoals.append(subgoal)
                        train_targets.append(command[[0, 2]])

        write_lines(temporary / "dataset.txt", [report["derived_session"] for report in reports])
        first_source = validated[requested["train"][0]][0]
        shutil.copy2(first_source / "label_names.txt", temporary / "label_names.txt")

        split_samples = {
            split: sum(report["samples"] for report in reports if report["split"] == split)
            for split in SPLITS
        }
        split_windows = {
            split: sum(report["valid_windows"] for report in reports if report["split"] == split)
            for split in SPLITS
        }
        manifest = {
            "schema": "semanticcnn_formal_split/v1",
            "purpose": "formal SemanticCNN training, model selection, and held-out evaluation",
            "strategy": "whole source bag/session isolation",
            "trajectory_leakage_allowed": False,
            "array_storage": "absolute directory symlinks to immutable CNN_READY source sessions",
            "sequence_length": SEQ_LEN,
            "sessions": reports,
            "split_samples": split_samples,
            "split_windows": split_windows,
            "total_samples": sum(split_samples.values()),
            "total_windows": sum(split_windows.values()),
            "unique_filenames": len(all_filenames),
        }
        manifest_path = temporary / "formal_split_manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        normalization = {
            "schema": "semanticcnn_training_input_stats/v1",
            "scope": "valid train-window endpoints only; dev/test excluded",
            "sequence_length": SEQ_LEN,
            "pool_mode": reference_metadata.get("semantic_cnn_pool_mode"),
            "scan_map": {
                "normalization": "virtual range clipped to pool_range_max then divided by pool_range_max",
                "range": [0.0, 1.0],
                "dataset_mean_std_required": False,
            },
            "sub_goal_local_xy": vector_stats(train_subgoals),
            "target_linear_x_angular_z": vector_stats(train_targets),
        }
        stats_path = temporary / "train_normalization_stats.json"
        stats_path.write_text(
            json.dumps(normalization, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        ready = {
            "schema": "semanticcnn_formal_split_approval/v1",
            "status": "PASS",
            "checks": {
                "all_sources_cnn_ready": True,
                "fixed_dual_contracts_compatible": True,
                "whole_session_split_isolation": True,
                "filename_leakage": False,
                "valid_windows_all_splits": all(value > 0 for value in split_windows.values()),
                "training_stats_exclude_dev_test": True,
            },
            "formal_split_manifest_sha256": sha256_file(manifest_path),
            "train_normalization_stats_sha256": sha256_file(stats_path),
            "dataset_txt_sha256": sha256_file(temporary / "dataset.txt"),
        }
        (temporary / "FORMAL_SPLIT_READY.json").write_text(
            json.dumps(ready, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(temporary, output_root)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    print(json.dumps(manifest, indent=2, sort_keys=True))
    print(f"formal_dataset={output_root}")
    print(f"normalization_stats={output_root / 'train_normalization_stats.json'}")


if __name__ == "__main__":
    main()
