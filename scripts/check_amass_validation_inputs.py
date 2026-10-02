#!/usr/bin/env python3
"""Check raw AMASS recordings required by the unmodified SMPL validation split."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
from zipfile import BadZipFile

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DATASETS = ("HumanEva", "SFU", "MPI_mosh", "MPI_HDM05")


def motion_name(path: Path) -> str:
    """Apply the basename normalization used by convert_amass_to_proto.py."""
    name = path.with_suffix(".motion").name
    for old in ("-", " ", "(", ")"):
        name = name.replace(old, "_")
    return (path.parent / name).as_posix()


def valid_npz(path: Path) -> tuple[bool, str]:
    try:
        with np.load(path, allow_pickle=False) as data:
            keys = set(data.files)
            fps_key = next((k for k in ("mocap_framerate", "mocap_frame_rate") if k in keys), None)
            if not {"poses", "trans"} <= keys or fps_key is None:
                return False, "missing poses, trans, or frame rate"
            poses, trans, fps = data["poses"], data["trans"], data[fps_key]
            if poses.ndim != 2 or poses.shape[0] < 2:
                return False, "invalid poses shape"
            if trans.shape != (poses.shape[0], 3):
                return False, "invalid trans shape"
            if not np.isfinite(fps).all() or float(np.asarray(fps).reshape(-1)[0]) <= 0:
                return False, "invalid frame rate"
    except (OSError, ValueError, KeyError, TypeError, EOFError, BadZipFile) as exc:
        return False, str(exc)
    return True, ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("amass_root", type=Path)
    parser.add_argument(
        "--yaml", type=Path, default=ROOT / "data/yaml_files/amass_smpl_validation.yaml"
    )
    args = parser.parse_args()
    raw_root = args.amass_root.resolve()
    motions = yaml.safe_load(args.yaml.read_text())["motions"]
    expected = {item["file"] for item in motions}
    if len(expected) != 345:
        raise ValueError(f"Expected 345 unique validation motions, got {len(expected)}")

    candidates: dict[str, list[Path]] = defaultdict(list)
    for dataset in DATASETS:
        for path in (raw_root / dataset).rglob("*.npz"):
            candidates[motion_name(path.relative_to(raw_root))].append(path)

    found: set[str] = set()
    problems: list[str] = []
    for name in sorted(expected):
        paths = candidates.get(name, [])
        if len(paths) != 1:
            problems.append(f"{name}: {len(paths)} matching raw files")
            continue
        ok, reason = valid_npz(paths[0])
        if ok:
            found.add(name)
        else:
            problems.append(f"{name}: {reason} ({paths[0]})")

    for dataset in DATASETS:
        required = {name for name in expected if name.startswith(dataset + "/")}
        print(f"{dataset}: {len(found & required)}/{len(required)} found")
    print(f"TOTAL: {len(found)}/{len(expected)}")
    for problem in problems:
        print(f"MISSING_OR_INVALID: {problem}")
    return 0 if len(found) == len(expected) else 1


if __name__ == "__main__":
    raise SystemExit(main())
