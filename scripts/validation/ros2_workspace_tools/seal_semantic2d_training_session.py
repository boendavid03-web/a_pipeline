#!/usr/bin/env python3
"""Hash and approve a checked SemanticCNN-native training session."""

from __future__ import annotations

import argparse
import hashlib
import json
import string
from pathlib import Path


SCHEMA = "semantic_nav_semantic2d_training_approval/v1"
SOURCE_APPROVAL_SCHEMA = "semantic_nav_cnn_supervision_approval/v1"
CAPTURE_CONTRACT_SCHEMA = "semantic_nav_auto_capture_contract/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def record(path: Path, root: Path) -> dict:
    return {
        "file": str(path.relative_to(root)),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def verify_file_records(records, root: Path, label: str) -> set[str]:
    if not isinstance(records, list) or not records:
        raise ValueError(f"source approval has no {label} records")
    verified = set()
    for index, item in enumerate(records):
        if not isinstance(item, dict):
            raise ValueError(f"{label}[{index}] is not an object")
        raw_name = item.get("file")
        expected_size = item.get("size_bytes")
        expected_hash = item.get("sha256")
        if not isinstance(raw_name, str) or not raw_name:
            raise ValueError(f"{label}[{index}] has an invalid file name")
        relative = Path(raw_name)
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or str(relative) != raw_name
        ):
            raise ValueError(f"{label}[{index}] has an unsafe file name")
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f"{label}[{index}] file is missing or outside source session")
        if raw_name in verified:
            raise ValueError(f"{label} repeats file {raw_name}")
        if (
            isinstance(expected_size, bool)
            or not isinstance(expected_size, int)
            or expected_size < 0
            or path.stat().st_size != expected_size
        ):
            raise ValueError(f"{label} size mismatch for {raw_name}")
        if (
            not isinstance(expected_hash, str)
            or len(expected_hash) != 64
            or any(character not in string.hexdigits for character in expected_hash)
            or sha256(path) != expected_hash.lower()
        ):
            raise ValueError(f"{label} checksum mismatch for {raw_name}")
        verified.add(raw_name)
    return verified


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--source-session", required=True, type=Path)
    parser.add_argument("--quality-report", required=True, type=Path)
    parser.add_argument("--source-approval", required=True, type=Path)
    parser.add_argument("--capture-contract", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    session = args.session.expanduser().resolve()
    source = args.source_session.expanduser().resolve()
    quality_path = args.quality_report.expanduser().resolve()
    source_approval_path = args.source_approval.expanduser().resolve()
    contract_path = args.capture_contract.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not session.is_dir() or not source.is_dir():
        raise NotADirectoryError("training and source sessions must exist")
    for path in (quality_path, source_approval_path, contract_path, output):
        if not path.is_relative_to(session):
            raise ValueError("training approval inputs/output must be session-local")
    for path in (quality_path, source_approval_path, contract_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    if output.exists():
        raise FileExistsError(output)

    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    if (
        quality.get("status") != "PASS"
        or quality.get("error_count") != 0
        or quality.get("errors")
        or quality.get("warnings")
        or Path(str(quality.get("session", ""))).resolve() != session
        or Path(str(quality.get("source_session", ""))).resolve() != source
    ):
        raise ValueError("Semantic2D quality report is not an exact PASS")
    source_approval = json.loads(
        source_approval_path.read_text(encoding="utf-8")
    )
    if (
        source_approval.get("schema") != SOURCE_APPROVAL_SCHEMA
        or source_approval.get("quality_status") != "PASS"
        or Path(str(source_approval.get("session", ""))).resolve() != source
    ):
        raise ValueError("source fixed-slot approval is invalid")
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    if contract.get("schema") != CAPTURE_CONTRACT_SCHEMA:
        raise ValueError("capture contract is invalid")

    metadata_path = session / "metadata.json"
    label_path = session / "label_names.txt"
    split_paths = [session / f"{name}.txt" for name in ("train", "dev", "test")]
    for path in (metadata_path, label_path, *split_paths):
        if not path.is_file():
            raise FileNotFoundError(path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    sample_count = int(metadata.get("samples", -1))
    if sample_count <= 0 or int(quality.get("samples", -1)) != sample_count:
        raise ValueError("training sample count is invalid or differs from report")
    if Path(str(metadata.get("source_npz_session", ""))).resolve() != source:
        raise ValueError("training metadata names a different source session")
    if int(source_approval.get("sample_count", -1)) != sample_count:
        raise ValueError("source approval sample count differs from training export")
    approved_source_samples = verify_file_records(
        source_approval.get("sample_files"), source, "sample_files"
    )
    actual_source_samples = {
        str(path.relative_to(source))
        for path in (source / "samples").glob("*.npz")
        if path.is_file()
    }
    if (
        len(approved_source_samples) != sample_count
        or approved_source_samples != actual_source_samples
    ):
        raise ValueError(
            "source approval sample records differ from source NPZ files"
        )
    source_core = {
        item.get("file"): item.get("sha256")
        for item in source_approval.get("core_files", [])
        if isinstance(item, dict)
    }
    if metadata.get("source_metadata_sha256") != source_core.get(
        "metadata.json"
    ):
        raise ValueError("training metadata is not bound to source approval")
    if sha256(contract_path) != source_core.get("capture_contract.json"):
        raise ValueError("training capture contract differs from source approval")
    field_map = metadata.get("field_map")
    if not isinstance(field_map, dict) or not field_map:
        raise ValueError("training metadata field_map is missing")
    field_directories = set(field_map) | {"intensities_lidar"}
    npy_files = []
    for directory_name in sorted(field_directories):
        directory = session / directory_name
        files = sorted(directory.glob("*.npy")) if directory.is_dir() else []
        if len(files) != sample_count:
            raise ValueError(
                f"training field {directory_name} has {len(files)} files, "
                f"expected {sample_count}"
            )
        npy_files.extend(files)

    core_paths = [
        metadata_path,
        label_path,
        *split_paths,
        quality_path,
        source_approval_path,
        contract_path,
    ]
    approval = {
        "schema": SCHEMA,
        "session": str(session),
        "source_session": str(source),
        "sample_count": sample_count,
        "core_files": [record(path, session) for path in core_paths],
        "training_arrays": [record(path, session) for path in npy_files],
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
