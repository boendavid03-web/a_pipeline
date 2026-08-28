#!/usr/bin/env python3
"""Atomically register a verified Semantic2D session in a dataset root."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import string
from pathlib import Path


READY_SCHEMA = "semantic_nav_semantic2d_training_approval/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_file_records(records, root: Path, label: str) -> set[str]:
    if not isinstance(records, list) or not records:
        raise ValueError(f"CNN_READY.json has no {label} records")
    verified = set()
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise ValueError(f"{label}[{index}] is not an object")
        raw_name = record.get("file")
        expected_size = record.get("size_bytes")
        expected_hash = record.get("sha256")
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
            raise ValueError(f"{label}[{index}] file is missing or outside session")
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


def validate_ready_session(session: Path) -> tuple[dict, dict]:
    metadata_path = session / "metadata.json"
    ready_path = session / "CNN_READY.json"
    if not metadata_path.is_file() or not ready_path.is_file():
        raise ValueError(f"training session is not approved: {session.name}")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    ready = json.loads(ready_path.read_text(encoding="utf-8"))
    if (
        ready.get("schema") != READY_SCHEMA
        or Path(str(ready.get("session", ""))).resolve() != session
        or int(ready.get("sample_count", -1)) != int(metadata.get("samples", -2))
    ):
        raise ValueError(f"CNN_READY.json does not approve {session.name}")
    core_files = verify_file_records(
        ready.get("core_files"), session, f"{session.name} core_files"
    )
    training_arrays = verify_file_records(
        ready.get("training_arrays"),
        session,
        f"{session.name} training_arrays",
    )
    if core_files & training_arrays:
        raise ValueError(f"CNN_READY.json repeats files across record groups: {session.name}")
    return metadata, ready


def nonempty_lines(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def atomic_text(path: Path, content: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    if temporary.exists():
        raise FileExistsError(temporary)
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--session", required=True, type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    root = args.dataset_root.expanduser().resolve()
    session = args.session.expanduser().resolve()
    if not session.is_dir() or session.parent != root:
        raise ValueError("session must be a direct child of the dataset root")
    if session.name.startswith(".") or "-" not in session.name:
        raise ValueError(
            "training session name must be visible and contain '-' for the loader"
        )
    label_path = session / "label_names.txt"
    for path in (session / "metadata.json", label_path, session / "CNN_READY.json"):
        if not path.is_file():
            raise FileNotFoundError(path)
    metadata, _ready = validate_ready_session(session)
    if metadata.get("session_name") != session.name:
        raise ValueError("training metadata session_name differs from directory")
    names = nonempty_lines(label_path)
    if len(names) < 2 or names[0] != "_background_":
        raise ValueError("training label_names.txt is invalid")

    root.mkdir(parents=True, exist_ok=True)
    lock_path = root / ".dataset.lock"
    with lock_path.open("a+", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        # Re-read and hash under the dataset lock so registration never relies
        # only on the unverified manifest fields stored in CNN_READY.json.
        metadata, _ready = validate_ready_session(session)
        if metadata.get("session_name") != session.name:
            raise ValueError("training metadata session_name differs from directory")
        root_labels = root / "label_names.txt"
        if root_labels.exists():
            if nonempty_lines(root_labels) != names:
                raise ValueError(
                    "dataset-root label_names.txt differs from the new session"
                )
        else:
            atomic_text(root_labels, "\n".join(names) + "\n")
        index = root / "dataset.txt"
        sessions = nonempty_lines(index) if index.exists() else []
        if len(set(sessions)) != len(sessions):
            sessions = list(dict.fromkeys(sessions))
        for existing in sessions:
            if (
                Path(existing).name != existing
                or existing.startswith(".")
                or "-" not in existing
            ):
                raise ValueError(
                    f"dataset index contains an unsafe session name: {existing}"
                )
            existing_dir = root / existing
            existing_metadata, _existing_ready = validate_ready_session(
                existing_dir.resolve()
            )
            if existing_metadata.get("session_name") != existing:
                raise ValueError(
                    f"dataset index session metadata name mismatch: {existing}"
                )
        if session.name not in sessions:
            sessions.append(session.name)
        atomic_text(index, "".join(f"{name}\n" for name in sessions))
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    print(
        json.dumps(
            {"dataset_root": str(root), "registered_session": session.name},
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
