#!/usr/bin/env python3
"""Create an immutable, self-contained map/semantic asset snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path

import yaml
from PIL import Image


SCHEMA = "semantic_nav_supervision_asset_snapshot/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--label-names", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args()


def snapshot_assets(
    map_yaml: Path,
    semantic_label: Path,
    label_names: Path,
    output_dir: Path,
) -> dict:
    sources = [
        path.expanduser().resolve()
        for path in (map_yaml, semantic_label, label_names)
    ]
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)
    map_yaml, semantic_label, label_names = sources
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict) or "image" not in metadata:
        raise ValueError("map YAML must contain an image path")
    occupancy = Path(str(metadata["image"]))
    if not occupancy.is_absolute():
        occupancy = map_yaml.parent / occupancy
    occupancy = occupancy.resolve()
    if not occupancy.is_file():
        raise FileNotFoundError(occupancy)
    occupancy_size = Image.open(occupancy).size
    semantic_size = Image.open(semantic_label).size
    if occupancy_size != semantic_size:
        raise ValueError(
            "occupancy and semantic label dimensions differ: "
            f"{occupancy_size} != {semantic_size}"
        )
    names = [
        line.strip()
        for line in label_names.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not names or names[0] != "_background_":
        raise ValueError("label names must begin with _background_")
    if not any(name.casefold() == "person" for name in names):
        raise ValueError("label names must contain Person")
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(output_dir)
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_dir.with_name(
        f".{output_dir.name}.incomplete-{os.getpid()}"
    )
    temporary.mkdir()
    try:
        occupancy_suffix = occupancy.suffix.lower() or ".img"
        snapshot_occupancy = temporary / f"occupancy{occupancy_suffix}"
        snapshot_label = temporary / "label.png"
        snapshot_names = temporary / "label_names.txt"
        snapshot_map = temporary / "map.yaml"
        shutil.copy2(occupancy, snapshot_occupancy)
        shutil.copy2(semantic_label, snapshot_label)
        shutil.copy2(label_names, snapshot_names)
        snapshot_metadata = dict(metadata)
        snapshot_metadata["image"] = snapshot_occupancy.name
        snapshot_map.write_text(
            yaml.safe_dump(snapshot_metadata, sort_keys=False),
            encoding="utf-8",
        )
        source_paths = {
            "map_yaml": map_yaml,
            "occupancy_image": occupancy,
            "semantic_label": semantic_label,
            "label_names": label_names,
        }
        snapshot_paths = {
            "map_yaml": snapshot_map,
            "occupancy_image": snapshot_occupancy,
            "semantic_label": snapshot_label,
            "label_names": snapshot_names,
        }
        manifest = {
            "schema": SCHEMA,
            "image_size": list(occupancy_size),
            "label_names": names,
            "sources": {
                key: {"path": str(path), "sha256": sha256(path)}
                for key, path in source_paths.items()
            },
            "snapshot": {
                key: {"file": path.name, "sha256": sha256(path)}
                for key, path in snapshot_paths.items()
            },
        }
        (temporary / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, output_dir)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return manifest


def main():
    args = parse_args()
    manifest = snapshot_assets(
        args.map_yaml,
        args.semantic_label,
        args.label_names,
        args.output_dir,
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
