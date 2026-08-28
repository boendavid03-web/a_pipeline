#!/usr/bin/env python3
"""Atomically approve and hash a validated CNN supervision dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


SCHEMA = "semantic_nav_cnn_supervision_approval/v1"
CAPTURE_CONTRACT_SCHEMA = "semantic_nav_auto_capture_contract/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path, root: Path) -> dict:
    return {
        "file": str(path.relative_to(root)),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def split_entries(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--quality-report", required=True, type=Path)
    parser.add_argument("--capture-contract", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    session = args.session.expanduser().resolve()
    quality_path = args.quality_report.expanduser().resolve()
    contract_path = args.capture_contract.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not session.is_dir():
        raise NotADirectoryError(session)
    for path, label in (
        (quality_path, "quality report"),
        (contract_path, "capture contract"),
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
        if not path.is_relative_to(session):
            raise ValueError(f"{label} must be inside the supervision session")
    if not output.is_relative_to(session):
        raise ValueError("approval marker must be inside the supervision session")
    if output.exists():
        raise FileExistsError(output)

    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    if (
        quality.get("status") != "PASS"
        or quality.get("error_count") != 0
        or quality.get("errors")
        or quality.get("warnings")
    ):
        raise ValueError("quality report is not an unconditional PASS")
    report_session = quality.get("session")
    if not isinstance(report_session, str) or Path(report_session).resolve() != session:
        raise ValueError("quality report names a different supervision session")
    gates = quality.get("quality_gate_configuration")
    required_boolean_gates = (
        "require_online_subgoal",
        "require_successful_episodes_only",
        "require_ground_truth_person_labels",
        "require_person_observations",
        "require_forward_only",
        "require_pre_relay_command_labels",
        "fail_on_warnings",
    )
    if not isinstance(gates, dict) or any(
        gates.get(name) is not True for name in required_boolean_gates
    ):
        raise ValueError("quality report was not produced with every strict gate")
    numeric_gates = (
        "minimum_samples",
        "minimum_duration_sec",
        "minimum_unique_command_vectors",
        "minimum_nonzero_command_fraction",
        "minimum_effective_sample_rate_hz",
        "minimum_person_positive_sample_fraction",
        "maximum_subgoal_age_ms",
        "maximum_cmd_vel_age_ms",
    )
    if any(
        not isinstance(gates.get(name), (int, float))
        or not math.isfinite(float(gates[name]))
        or float(gates[name]) <= 0.0
        for name in numeric_gates
    ):
        raise ValueError("quality report has missing or non-positive numeric gates")
    if gates.get("maximum_person_truth_unmatched_samples") != 0:
        raise ValueError("quality report did not require zero unmatched person truth")
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    if contract.get("schema") != CAPTURE_CONTRACT_SCHEMA:
        raise ValueError("unsupported capture contract schema")

    metadata_path = session / "metadata.json"
    split_paths = [session / f"{name}.txt" for name in ("train", "dev", "test")]
    for path in (metadata_path, *split_paths):
        if not path.is_file():
            raise FileNotFoundError(path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata_bag = metadata.get("bag")
    contract_bag = contract.get("bag")
    if (
        not isinstance(metadata_bag, str)
        or not isinstance(contract_bag, str)
        or Path(metadata_bag).resolve() != Path(contract_bag).resolve()
    ):
        raise ValueError("capture contract and dataset metadata name different bags")
    verified_assets = contract.get("supervision_assets", {}).get(
        "verified_files"
    )
    if not isinstance(verified_assets, list):
        raise ValueError("capture contract has no verified asset files")
    asset_hashes = {
        item.get("kind"): item.get("sha256")
        for item in verified_assets
        if isinstance(item, dict)
    }
    for kind, metadata_key in (
        ("map_yaml", "map_yaml_sha256"),
        ("occupancy_image", "occupancy_image_sha256"),
        ("semantic_label", "semantic_label_sha256"),
        ("label_names", "label_names_sha256"),
    ):
        if metadata.get(metadata_key) != asset_hashes.get(kind):
            raise ValueError(
                f"dataset {metadata_key} differs from capture contract"
            )
    sample_names = set()
    for path in split_paths:
        entries = split_entries(path)
        if sample_names.intersection(entries):
            raise ValueError("train/dev/test split files overlap")
        sample_names.update(entries)
    sample_dir = session / "samples"
    actual_names = {path.name for path in sample_dir.glob("*.npz")}
    if sample_names != actual_names:
        raise ValueError("split union differs from supervision sample files")
    if int(metadata.get("samples", -1)) != len(actual_names):
        raise ValueError("metadata sample count differs from sample files")
    if not actual_names or int(quality.get("samples", -1)) != len(actual_names):
        raise ValueError("quality report sample count differs or is zero")
    if quality.get("subgoal_source") != "online":
        raise ValueError("quality report did not validate online subgoals")
    if quality.get("person_label_mode") != "ground-truth-legs":
        raise ValueError("quality report did not validate ground-truth legs")
    if int(quality.get("episode_count", 0)) <= 0:
        raise ValueError("quality report contains no successful episode")
    if quality.get("episode_filter", {}).get("mode") != "successful_only":
        raise ValueError("quality report did not validate successful-only filtering")
    if int(quality.get("negative_linear_x_samples", -1)) != 0:
        raise ValueError("quality report contains reverse-motion samples")
    if int(quality.get("person_ground_truth_unmatched_samples", -1)) != 0:
        raise ValueError("quality report contains unmatched person truth")
    if int(quality.get("person_label_count", 0)) <= 0:
        raise ValueError("quality report contains no Person-labeled slot")

    core_paths = [metadata_path, *split_paths, quality_path, contract_path]
    sample_paths = [sample_dir / name for name in sorted(actual_names)]
    approval = {
        "schema": SCHEMA,
        "session": str(session),
        "quality_status": "PASS",
        "sample_count": len(sample_paths),
        "core_files": [file_record(path, session) for path in core_paths],
        "sample_files": [file_record(path, session) for path in sample_paths],
    }
    temporary = output.with_name(f".{output.name}.tmp")
    if temporary.exists():
        raise FileExistsError(temporary)
    temporary.write_text(
        json.dumps(approval, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)
    print(json.dumps(approval, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
