#!/usr/bin/env python3
"""Generate fixed-speed, obstacle-free IRA out-and-back pedestrian patrols."""

from __future__ import annotations

import argparse
import math
import os
import random
import tempfile
from pathlib import Path

import yaml


MAX_PEDESTRIANS = 50
DEFAULT_HALF_WIDTH_M = 11.0
DEFAULT_HALF_HEIGHT_M = 8.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--pedestrian-count", type=int, default=8)
    parser.add_argument("--seed", type=int, default=21)
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--half-width", type=float, default=DEFAULT_HALF_WIDTH_M)
    parser.add_argument("--half-height", type=float, default=DEFAULT_HALF_HEIGHT_M)
    return parser.parse_args()


def out_and_back_routes(
    count: int,
    seed: int,
    half_width_m: float = DEFAULT_HALF_WIDTH_M,
    half_height_m: float = DEFAULT_HALF_HEIGHT_M,
) -> list[list[list[float]]]:
    """Return radial two-way routes with distributed starts and headings."""
    if not 1 <= count <= MAX_PEDESTRIANS:
        raise ValueError(f"pedestrian_count must be between 1 and {MAX_PEDESTRIANS}")
    if not math.isfinite(half_width_m) or half_width_m <= 1.0:
        raise ValueError("half_width must be finite and greater than 1 m")
    if not math.isfinite(half_height_m) or half_height_m <= 1.0:
        raise ValueError("half_height must be finite and greater than 1 m")

    rng = random.Random(seed)
    angular_offset = rng.uniform(0.0, math.pi / max(count, 2))
    routes: list[list[list[float]]] = []
    for index in range(count):
        angle = angular_offset + math.pi * index / count
        endpoint = [
            round(half_width_m * math.cos(angle), 6),
            round(half_height_m * math.sin(angle), 6),
            0.0,
        ]
        opposite = [-endpoint[0], -endpoint[1], 0.0]
        # Alternate the initial direction so the first traversal already
        # contains headings around the full circle. The repeated first point
        # makes this a literal continuous out-and-back patrol.
        start, end = (opposite, endpoint) if index % 2 == 0 else (endpoint, opposite)
        routes.append([list(start), list(end), list(start)])
    return routes


def make_config(scene: Path, count: int, seed: int, speed: float) -> dict:
    if not scene.is_file():
        raise ValueError(f"empty scene does not exist: {scene}")
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("speed must be a positive finite number")
    groups = {}
    for index, route in enumerate(out_and_back_routes(count, seed)):
        groups[f"empty_field_{index + 1:03d}"] = {
            "num": 1,
            "asset_path": "Isaac/People/Characters/",
            "routines": [
                {
                    "patrol": {
                        "weight": 1,
                        "repeat": 1,
                        "speed_range": [round(speed, 6), round(speed, 6)],
                        "path_points": route,
                    }
                }
            ],
        }
    return {
        "isaacsim.replicator.agent": {
            "version": "1.6.0",
            "seed": seed,
            "simulation_duration": 86400.0,
            "environment": {"base_stage_asset_path": str(scene.resolve())},
            "character": {
                "motion_library_path": "Isaac/People/MotionLibrary/HumanMotionLibrary.usd",
                "groups": groups,
            },
        }
    }


def write_config(config: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main() -> int:
    args = parse_args()
    try:
        config = make_config(
            args.scene, args.pedestrian_count, args.seed, args.speed
        )
    except ValueError as error:
        raise SystemExit(f"ERROR: {error}") from error
    write_config(config, args.output)
    print(
        "EMPTY_PEOPLE_CONFIG=PASS "
        f"people={args.pedestrian_count} speed_mps={args.speed:.3f} "
        f"seed={args.seed} routes=out_and_back directions={args.pedestrian_count} "
        f"output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
