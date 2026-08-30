#!/usr/bin/env python3
"""Generate deterministic IRA routes for crowded detector/tracker stress tests.

The routes stay in the project engineering-lobby free space and contain only
the two or three named stress pedestrians.  They intentionally do not alter
detector or tracker settings; the configured spacing is merely a requested
scene geometry and the evaluator remains authoritative for the achieved GT
distance.
"""

from __future__ import annotations

import argparse
import json
import math
from copy import deepcopy
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_TEMPLATE = SCRIPT_DIR / "ira_people_demo/custom_eng_lobby_people.yaml"
DEFAULT_WORLD = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)
DEFAULT_SCENE = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
ROBOT_SPAWN_ROS_M = (13.5, 6.5, 0.01)

SCENARIO_COUNTS = {"A": 2, "B": 2, "C": 2, "D": 2, "E": 3}
DEFAULT_SPACING_M = {"A": 1.0, "B": 0.0, "C": 0.0, "D": 0.75, "E": 0.0}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIO_COUNTS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--spacing", type=float)
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--wall-clearance", type=float, default=0.55)
    return parser.parse_args()


def route_specs(scenario: str, spacing: float) -> list[tuple[str, list[list[float]]]]:
    """Return project-map x/y/z patrols for one requested stress geometry."""
    if scenario in {"A", "D"}:
        center_x, center_y, half = 14.5, 10.0, 2.0
        left_x = center_x - 0.5 * spacing
        right_x = center_x + 0.5 * spacing
        left = [[left_x, center_y - half, 0.0], [left_x, center_y + half, 0.0]]
        right_forward = [
            [right_x, center_y - half, 0.0],
            [right_x, center_y + half, 0.0],
        ]
        right_reverse = list(reversed(right_forward))
        return [
            ("stress_a", left),
            ("stress_b", right_forward if scenario == "A" else right_reverse),
        ]
    if scenario == "B":
        center_x, center_y, half = 14.5, 10.0, 2.0
        vertical = [
            [center_x, center_y - half, 0.0],
            [center_x, center_y + half, 0.0],
        ]
        return [("stress_a", vertical), ("stress_b", list(reversed(vertical)))]
    if scenario == "C":
        center_x, center_y, half = 13.5, 9.5, 2.0
        diagonal = half / math.sqrt(2.0)
        return [
            (
                "stress_a",
                [
                    [center_x - diagonal, center_y - diagonal, 0.0],
                    [center_x + diagonal, center_y + diagonal, 0.0],
                ],
            ),
            (
                "stress_b",
                [
                    [center_x - diagonal, center_y + diagonal, 0.0],
                    [center_x + diagonal, center_y - diagonal, 0.0],
                ],
            ),
        ]
    if scenario == "E":
        center_x, center_y, half = 13.5, 9.5, 2.0
        return [
            (
                "stress_a",
                [[center_x, center_y + half, 0.0], [center_x, center_y - half, 0.0]],
            ),
            (
                "stress_b",
                [[center_x - half, center_y, 0.0], [center_x + half, center_y, 0.0]],
            ),
            (
                "stress_c",
                [[center_x + half, center_y, 0.0], [center_x - half, center_y, 0.0]],
            ),
        ]
    raise ValueError(f"unsupported scenario: {scenario}")


def build_config(
    template: dict,
    scenario: str,
    spacing: float,
    speed: float,
    seed: int,
) -> tuple[dict, dict]:
    specs = route_specs(scenario, spacing)
    root = template["isaacsim.replicator.agent"]
    root["environment"]["base_stage_asset_path"] = str(DEFAULT_SCENE.resolve())
    template_groups = root["character"]["groups"]
    base_group = deepcopy(next(iter(template_groups.values())))
    groups = {}
    for identity, points in specs:
        group = deepcopy(base_group)
        group["num"] = 1
        patrols = [
            routine["patrol"]
            for routine in group.get("routines", [])
            if "patrol" in routine
        ]
        if len(patrols) != 1:
            raise ValueError("crowded stress template must contain one patrol routine")
        patrols[0]["speed_range"] = [speed, speed]
        patrols[0]["path_points"] = points
        groups[identity] = group
    root["seed"] = seed
    root["character"]["groups"] = groups
    metadata = {
        "schema": "isaac_crowded_tracking_scenario/v1",
        "scenario": scenario,
        "description": {
            "A": "two_people_parallel_same_direction",
            "B": "two_people_head_on",
            "C": "two_people_diagonal_crossing",
            "D": "two_people_parallel_opposite_direction",
            "E": "three_people_crossing",
        }[scenario],
        "expected_pedestrian_count": len(specs),
        # pedestrian_payload() exposes the authored IRA group instance, whose
        # stable ROS-side ID adds ``_0`` to each one-person group name.
        "stress_ids": [f"{identity}_0" for identity, _ in specs],
        "requested_spacing_m": spacing if scenario in {"A", "D"} else None,
        "speed_mps": speed,
        "seed": seed,
        "robot_spawn_ros_m": list(ROBOT_SPAWN_ROS_M),
        "routes_ros_m": {identity: points for identity, points in specs},
    }
    return template, metadata


def validate_routes(config: dict, world: Path, clearance: float) -> None:
    from convert_gazebo_boxes_to_usda import load_static_boxes
    from people_route_geometry import DEFAULT_LOBBY_BOUNDS, edge_is_continuously_safe

    boxes, _ = load_static_boxes(world)
    groups = config["isaacsim.replicator.agent"]["character"]["groups"]
    for identity, group in groups.items():
        patrol = next(routine["patrol"] for routine in group["routines"] if "patrol" in routine)
        planar = [(float(point[0]), float(point[1])) for point in patrol["path_points"]]
        for start, end in zip(planar, planar[1:] + planar[:1]):
            if not edge_is_continuously_safe(
                start, end, boxes, DEFAULT_LOBBY_BOUNDS, clearance
            ):
                raise ValueError(
                    f"{identity} route is not safe at {clearance:.2f} m clearance: "
                    f"{start} -> {end}"
                )
        for point in planar:
            if not edge_is_continuously_safe(
                ROBOT_SPAWN_ROS_M[:2],
                point,
                boxes,
                DEFAULT_LOBBY_BOUNDS,
                clearance,
            ):
                raise ValueError(
                    f"{identity} is not directly observable from the stationary "
                    f"robot spawn: {ROBOT_SPAWN_ROS_M[:2]} -> {point}"
                )


def main() -> int:
    args = parse_args()
    spacing = DEFAULT_SPACING_M[args.scenario] if args.spacing is None else args.spacing
    if not math.isfinite(spacing) or spacing < 0.0:
        raise SystemExit("ERROR: --spacing must be finite and non-negative")
    if args.scenario in {"A", "D"} and not 0.45 <= spacing <= 1.50:
        raise SystemExit("ERROR: Scenario A/D --spacing must be from 0.45 through 1.50 m")
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise SystemExit("ERROR: --speed must be a positive finite number")
    if not 0 <= args.seed <= 4_294_967_295:
        raise SystemExit("ERROR: --seed must be between 0 and 4294967295")
    template = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    config, metadata = build_config(
        template, args.scenario, spacing, args.speed, args.seed
    )
    validate_routes(config, args.world, args.wall_clearance)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    metadata_path = args.metadata_output or args.output.with_suffix(".metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(
        "CROWDED_TRACKING_SCENARIO_CONFIG=PASS "
        f"scenario={args.scenario} people={metadata['expected_pedestrian_count']} "
        f"spacing_m={metadata['requested_spacing_m']} output={args.output} "
        f"metadata={metadata_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
