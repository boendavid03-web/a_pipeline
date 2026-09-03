#!/usr/bin/env python3
"""Generate deterministic one-person IRA routes for motion evaluation."""

from __future__ import annotations

import argparse
import math
import sys
from copy import deepcopy
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
DEFAULT_TEMPLATE = SCRIPT_DIR / "ira_people_demo/custom_eng_lobby_people.yaml"
DEFAULT_MAP_YAML = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/"
    "gazebo_eng_lobby.yaml"
)
DEFAULT_WORLD = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)
DEFAULT_SCENE = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"

SCENARIOS = {
    "front_approach": {
        "robot_pose": (1.5, 9.0, 0.0),
        "local_route": ((10.0, 0.0), (2.0, 0.0)),
    },
    "front_leave": {
        "robot_pose": (1.5, 9.0, 0.0),
        "local_route": ((2.0, 0.0), (10.0, 0.0)),
    },
    "lateral": {
        "robot_pose": (18.0, 16.0, math.pi / 2.0),
        "local_route": ((4.0, 4.0), (4.0, -4.0)),
    },
    "diagonal": {
        "robot_pose": (1.5, 13.0, math.pi / 4.0),
        "local_route": ((8.0, -4.0), (2.3431457505, 1.6568542495)),
    },
}


def local_to_odom(point: tuple[float, float], pose: tuple[float, float, float]) -> tuple[float, float]:
    x, y = point
    origin_x, origin_y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        origin_x + cosine * x - sine * y,
        origin_y + sine * x + cosine * y,
    )


def scenario_spec(name: str, speed: float) -> dict:
    raw = SCENARIOS[name]
    start, end = raw["local_route"]
    pose = raw["robot_pose"]
    odom_start = local_to_odom(start, pose)
    odom_end = local_to_odom(end, pose)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    local_velocity = (speed * dx / length, speed * dy / length)
    yaw = pose[2]
    odom_velocity = (
        math.cos(yaw) * local_velocity[0] - math.sin(yaw) * local_velocity[1],
        math.sin(yaw) * local_velocity[0] + math.cos(yaw) * local_velocity[1],
    )
    return {
        "scenario": name,
        "robot_pose": pose,
        "local_route": (start, end),
        "odom_route": (odom_start, odom_end),
        "route_length": length,
        "target_local_velocity": local_velocity,
        "target_odom_velocity": odom_velocity,
    }


def build_config(template: dict, spec: dict, speed: float, seed: int) -> dict:
    root = template["isaacsim.replicator.agent"]
    root["environment"]["base_stage_asset_path"] = str(DEFAULT_SCENE.resolve())
    groups = root["character"]["groups"]
    if not groups:
        raise ValueError("single-motion template has no character group")
    group = deepcopy(next(iter(groups.values())))
    patrols = [
        routine["patrol"] for routine in group.get("routines", []) if "patrol" in routine
    ]
    if len(patrols) != 1:
        raise ValueError("single-motion template must contain exactly one patrol routine")
    group["num"] = 1
    patrols[0]["repeat"] = 1
    patrols[0]["speed_range"] = [speed, speed]
    patrols[0]["path_points"] = [
        [float(point[0]), float(point[1]), 0.0] for point in spec["odom_route"]
    ]
    root["seed"] = seed
    root["character"]["groups"] = {"benchmark_person": group}
    return template


def validate_geometry(config: dict, spec: dict, world: Path, clearance: float) -> None:
    from convert_gazebo_boxes_to_usda import load_static_boxes
    from people_route_geometry import DEFAULT_LOBBY_BOUNDS, edge_is_continuously_safe

    boxes, _ = load_static_boxes(world)
    start, end = spec["odom_route"]
    robot = spec["robot_pose"][:2]
    if not edge_is_continuously_safe(start, end, boxes, DEFAULT_LOBBY_BOUNDS, clearance):
        raise ValueError(
            f"{spec['scenario']} route is not safe at {clearance:.2f} m clearance: "
            f"{start} -> {end}"
        )
    samples = max(1, int(math.ceil(spec["route_length"] / 0.25)))
    for index in range(samples + 1):
        fraction = index / samples
        point = (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )
        if not edge_is_continuously_safe(
            robot, point, boxes, DEFAULT_LOBBY_BOUNDS, clearance
        ):
            raise ValueError(
                f"{spec['scenario']} route loses direct visibility at {point} "
                f"from robot {robot} with {clearance:.2f} m clearance"
            )


def metadata_for(
    spec: dict,
    *,
    speed: float,
    seed: int,
    clearance: float,
    template: Path,
    map_yaml: Path,
    world: Path,
) -> dict:
    vector = lambda values: [float(value) for value in values]
    return {
        "schema": "isaac_single_motion_scenario/v1",
        "scenario": spec["scenario"],
        "expected_pedestrian_count": 1,
        "ground_truth_id": "benchmark_person_0",
        "speed_mps": speed,
        "seed": seed,
        "clearance_m": clearance,
        "benchmark_frame": {
            "name": "benchmark_lidar",
            "origin": "stationary base_link and dual-LiDAR midpoint ground projection",
            "x_axis": "robot_forward",
            "y_axis": "robot_left",
        },
        "robot_pose_odom": {
            "x_m": spec["robot_pose"][0],
            "y_m": spec["robot_pose"][1],
            "yaw_rad": spec["robot_pose"][2],
        },
        "route": {
            "local_points_m": [vector(point) for point in spec["local_route"]],
            "odom_points_m": [vector(point) for point in spec["odom_route"]],
            "length_m": spec["route_length"],
            "target_local_velocity_mps": vector(spec["target_local_velocity"]),
            "target_odom_velocity_mps": vector(spec["target_odom_velocity"]),
        },
        "inputs": {
            "template": str(template.resolve()),
            "map_yaml": str(map_yaml.resolve()),
            "world": str(world.resolve()),
            "scene": str(DEFAULT_SCENE.resolve()),
        },
        "limitations": [
            "two-point IRA patrol repeats; only target-direction steady legs are primary",
            "robot global pose differs for some scenarios because of map clearance",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=tuple(SCENARIOS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path, required=True)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--clearance", type=float, default=0.55)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for path in (args.template, args.map_yaml, args.world, DEFAULT_SCENE):
        if not path.is_file():
            raise SystemExit(f"ERROR: required single-motion input is missing: {path}")
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise SystemExit("ERROR: --speed must be a positive finite number")
    if not math.isfinite(args.clearance) or args.clearance <= 0.0:
        raise SystemExit("ERROR: --clearance must be a positive finite number")
    if not 0 <= args.seed <= 4_294_967_295:
        raise SystemExit("ERROR: --seed must be between 0 and 4294967295")

    spec = scenario_spec(args.scenario, args.speed)
    if not math.isclose(spec["route_length"], 8.0, abs_tol=1.0e-9):
        raise SystemExit(
            f"ERROR: {args.scenario} route length is {spec['route_length']}, expected 8.0 m"
        )
    template = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    config = build_config(template, spec, args.speed, args.seed)
    validate_geometry(config, spec, args.world, args.clearance)
    metadata = metadata_for(
        spec,
        speed=args.speed,
        seed=args.seed,
        clearance=args.clearance,
        template=args.template,
        map_yaml=args.map_yaml,
        world=args.world,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.metadata_output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    args.metadata_output.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    print(
        "SINGLE_MOTION_SCENARIO_CONFIG=PASS "
        f"scenario={args.scenario} speed_mps={args.speed:.3f} "
        f"output={args.output} metadata={args.metadata_output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
