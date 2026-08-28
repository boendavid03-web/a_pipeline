#!/usr/bin/env python3
"""Create a small, bag-disjoint SemanticCNN dataset view from sealed sessions.

The generated dataset does not copy the large per-frame arrays.  Each derived
session links to one immutable source session and indexes a bounded number of
contiguous frames in exactly one of train/dev/test.  It is intended for smoke
training only, not for publishing formal model metrics.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from collections import defaultdict
from pathlib import Path


SPLITS = ("train", "dev", "test")
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
SEQ_LEN = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    for split in SPLITS:
        parser.add_argument(f"--{split}-session", required=True)
    parser.add_argument("--max-samples-per-split", type=int, default=512)
    parser.add_argument("--episodes-per-split", type=int, default=4)
    return parser.parse_args()


def read_lines(path: Path) -> list[str]:
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def contiguous_segments(metadata: dict, eligible: set[str]) -> list[list[dict]]:
    segments: list[list[dict]] = []
    current: list[dict] = []
    for record in metadata.get("frames", []):
        if record.get("name") not in eligible:
            if current:
                segments.append(current)
                current = []
            continue
        if current:
            same_episode = int(record["episode_id"]) == int(current[-1]["episode_id"])
            delta_ms = (int(record["scan_01_stamp_ns"]) - int(current[-1]["scan_01_stamp_ns"])) / 1e6
            expected_ms = float(metadata.get("expected_frame_period_ms", 65.0))
            tolerance_ms = float(metadata.get("frame_period_tolerance_ms", 20.0))
            if not same_episode or abs(delta_ms - expected_ms) > tolerance_ms:
                segments.append(current)
                current = []
        current.append(record)
    if current:
        segments.append(current)
    return [segment for segment in segments if len(segment) >= SEQ_LEN]


def choose_records(
    metadata: dict,
    eligible: set[str],
    maximum: int,
    episode_limit: int,
) -> list[dict]:
    segments = contiguous_segments(metadata, eligible)
    if not segments:
        raise ValueError("source split has no contiguous 10-frame SemanticCNN sequence")

    # Prefer long segments while keeping several episodes represented.  Taking a
    # prefix from a segment preserves causal history and deterministic replay.
    segments = sorted(segments, key=lambda segment: (-len(segment), int(segment[0]["episode_id"])))
    chosen_segments = segments[:episode_limit]
    per_segment = max(SEQ_LEN, math.ceil(maximum / len(chosen_segments)))
    selected: list[dict] = []
    for segment in chosen_segments:
        remaining = maximum - len(selected)
        if remaining <= 0:
            break
        take = min(len(segment), per_segment, remaining)
        if take >= SEQ_LEN:
            selected.extend(segment[:take])

    # Fill any remaining budget from other long segments without duplicating
    # records.  Each additional slice must still admit at least one window.
    selected_names = {record["name"] for record in selected}
    for segment in segments:
        remaining = maximum - len(selected)
        if remaining < SEQ_LEN:
            break
        available = [record for record in segment if record["name"] not in selected_names]
        take = min(len(available), remaining)
        if take >= SEQ_LEN:
            selected.extend(available[:take])
            selected_names.update(record["name"] for record in available[:take])

    selected.sort(key=lambda record: int(record["scan_01_stamp_ns"]))
    if len(selected) < SEQ_LEN:
        raise ValueError("bounded selection produced no SemanticCNN window")
    return selected


def valid_window_count(records: list[dict], selected_names: set[str]) -> int:
    count = 0
    for end in range(SEQ_LEN - 1, len(records)):
        window = records[end - SEQ_LEN + 1 : end + 1]
        if any(record["name"] not in selected_names for record in window):
            continue
        if len({int(record["episode_id"]) for record in window}) != 1:
            continue
        stamps = [int(record["scan_01_stamp_ns"]) for record in window]
        if all(45_000_000 <= right - left <= 85_000_000 for left, right in zip(stamps, stamps[1:])):
            count += 1
    return count


def source_candidates(source: Path) -> list[str]:
    names: list[str] = []
    for split in SPLITS:
        names.extend(read_lines(source / f"{split}.txt"))
    return list(dict.fromkeys(names))


def materialize_session(
    source_root: Path,
    output_root: Path,
    source_name: str,
    split: str,
    maximum: int,
    episode_limit: int,
) -> dict:
    source = (source_root / source_name).resolve(strict=True)
    if not (source / "CNN_READY.json").is_file():
        raise FileNotFoundError(f"source session is not sealed with CNN_READY.json: {source}")
    metadata_path = source / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    source_bag = str(metadata.get("source_bag", metadata.get("bag", "")))
    source_npz = metadata.get("source_npz_session")
    if not source_bag and source_npz:
        source_npz_metadata = Path(source_npz) / "metadata.json"
        if source_npz_metadata.is_file():
            source_bag = str(
                json.loads(source_npz_metadata.read_text(encoding="utf-8")).get("bag", "")
            )
    candidates = source_candidates(source)
    selected_records = choose_records(metadata, set(candidates), maximum, episode_limit)
    selected_names = [str(record["name"]) for record in selected_records]

    derived_name = f"{source.name}-smoke-{split}"
    derived = output_root / derived_name
    derived.mkdir()
    for directory in REQUIRED_ARRAY_DIRS:
        target = source / directory
        if not target.is_dir():
            raise FileNotFoundError(f"source session is missing {directory}/: {source}")
        os.symlink(target, derived / directory, target_is_directory=True)

    for current_split in SPLITS:
        values = selected_names if current_split == split else []
        (derived / f"{current_split}.txt").write_text(
            "".join(f"{name}\n" for name in values), encoding="utf-8"
        )
    (derived / "label_names.txt").write_text(
        (source / "label_names.txt").read_text(encoding="utf-8"), encoding="utf-8"
    )
    derived_metadata = dict(metadata)
    derived_metadata.update(
        {
            "session_name": derived_name,
            "split_role": split,
            "smoke_subset": True,
            "smoke_subset_source_session": str(source),
            "smoke_subset_selected_samples": len(selected_names),
            "smoke_subset_selected_episode_ids": sorted(
                {int(record["episode_id"]) for record in selected_records}
            ),
        }
    )
    (derived / "metadata.json").write_text(
        json.dumps(derived_metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    windows = valid_window_count(metadata.get("frames", []), set(selected_names))
    if windows <= 0:
        raise ValueError(f"derived {split} split has no valid windows")
    return {
        "derived_session": derived_name,
        "source_session": source.name,
        "source_path": str(source),
        "source_bag": source_bag,
        "split": split,
        "samples": len(selected_names),
        "episodes": sorted({int(record["episode_id"]) for record in selected_records}),
        "valid_windows": windows,
    }


def main() -> None:
    args = parse_args()
    if args.max_samples_per_split < SEQ_LEN:
        raise ValueError("--max-samples-per-split must be at least 10")
    if args.episodes_per_split < 1:
        raise ValueError("--episodes-per-split must be positive")
    source_root = args.source_root.resolve(strict=True)
    output_root = args.output_root.resolve()
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite smoke dataset: {output_root}")
    output_root.mkdir(parents=True)

    requested = {split: getattr(args, f"{split}_session") for split in SPLITS}
    if len(set(requested.values())) != len(SPLITS):
        raise ValueError("train/dev/test must use three distinct source sessions")

    reports = []
    for split in SPLITS:
        reports.append(
            materialize_session(
                source_root,
                output_root,
                requested[split],
                split,
                args.max_samples_per_split,
                args.episodes_per_split,
            )
        )
    (output_root / "dataset.txt").write_text(
        "".join(f"{report['derived_session']}\n" for report in reports), encoding="utf-8"
    )
    first_source = source_root / requested["train"] / "label_names.txt"
    (output_root / "label_names.txt").write_text(
        first_source.read_text(encoding="utf-8"), encoding="utf-8"
    )
    manifest = {
        "schema": "semanticcnn_smoke_subset/v1",
        "purpose": "bounded smoke training only; not a formal benchmark split",
        "trajectory_leakage_allowed": False,
        "array_storage": "absolute directory symlinks to immutable CNN_READY source sessions",
        "max_samples_per_split": args.max_samples_per_split,
        "episodes_per_split": args.episodes_per_split,
        "sessions": reports,
        "split_samples": {report["split"]: report["samples"] for report in reports},
        "split_windows": {report["split"]: report["valid_windows"] for report in reports},
    }
    manifest_path = output_root / "smoke_subset_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
