#!/usr/bin/env python3
"""Compare local Arena worlds with the revision pinned by Arena's .repos file.

This checks source identity and exact-name model availability. It does not
claim that a USD scene has loaded or that its physics is valid.
"""

import argparse
import json
import re
import subprocess
from pathlib import Path


WORLDS = ("factory", "generated", "hospital", "house17", "ignc", "map_empty")
PINNED_REVISION = "3f142b2"


def git(*args: str, cwd: Path, input_bytes: bytes | None = None) -> bytes:
    return subprocess.run(
        ["git", "-C", str(cwd), *args],
        input=input_bytes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    ).stdout.strip()


def verify(upstream: Path, local: Path) -> dict:
    revision = git("rev-parse", "HEAD", cwd=upstream).decode()
    if not revision.startswith(PINNED_REVISION):
        raise ValueError(f"upstream HEAD {revision} is not the pinned {PINNED_REVISION}")

    result = {"upstream_revision": revision, "worlds": {}}
    for world in WORLDS:
        entries = git("ls-tree", "-r", "-z", "HEAD", f"worlds/{world}", cwd=upstream)
        matches, missing, changed = 0, [], []
        for entry in entries.split(b"\0"):
            if not entry:
                continue
            metadata, path_bytes = entry.split(b"\t", 1)
            expected = metadata.split()[2].decode()
            relative = path_bytes.decode()
            path = local / relative
            if not path.is_file():
                missing.append(relative)
            elif git("hash-object", str(path), cwd=local).decode() == expected:
                matches += 1
            else:
                changed.append(relative)
        result["worlds"][world] = {
            "source_files_matching": matches,
            "source_files_missing": missing,
            "source_files_changed": changed,
            "source_exact": bool(entries) and not missing and not changed,
        }

    obstacle_file = local / "worlds/generated/map/obstacles.yaml"
    models = sorted(set(re.findall(r"^\s*- model:\s*([^\s#]+)", obstacle_file.read_text(), re.M)))
    available, absent = [], []
    for model in models:
        gaz = local / "gazebo_models" / model / "model.sdf"
        entity = local / "entities/obstacles/static" / model / "sdf" / f"{model}.sdf"
        (available if gaz.is_file() or entity.is_file() else absent).append(model)
    result["generated_model_names"] = {
        "available_exact_names": available,
        "missing_exact_names": absent,
    }
    result["all_source_exact"] = all(w["source_exact"] for w in result["worlds"].values())
    result["generated_asset_names_complete"] = not absent
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True, help="clean checkout of voshch/arena-simulation-setup at 3f142b2")
    parser.add_argument("--local", type=Path, required=True, help="local arena/simulation-setup directory")
    args = parser.parse_args()
    print(json.dumps(verify(args.upstream.resolve(), args.local.resolve()), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
