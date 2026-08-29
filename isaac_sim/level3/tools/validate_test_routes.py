#!/usr/bin/env python3
"""Validate deferred Level 3 goal geometry against the untouched raster map."""

from __future__ import annotations

import argparse
import heapq
import importlib.util
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import yaml

warnings.filterwarnings("ignore", category=UserWarning, module="scipy")
from scipy.ndimage import distance_transform_edt  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
LEVEL3_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = LEVEL3_ROOT.parents[1]
ROUTES_FILE = LEVEL3_ROOT / "config/test_routes.yaml"
MAP_YAML = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby"
    / "gazebo_eng_lobby.yaml"
)
CALIBRATOR = SCRIPT_DIR / "calibrate_map_alignment.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def load_calibrator():
    spec = importlib.util.spec_from_file_location("level3_route_calibrator", CALIBRATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {CALIBRATOR}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    args = parse_args()
    routes = yaml.safe_load(ROUTES_FILE.read_text(encoding="utf-8"))
    map_config = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
    image_path = (MAP_YAML.parent / map_config["image"]).resolve()
    image = load_calibrator().read_pgm(image_path)
    resolution = float(map_config["resolution"])
    origin_x, origin_y = (float(value) for value in map_config["origin"][:2])
    occupancy_probability = (255.0 - image.astype(float)) / 255.0
    occupied = occupancy_probability > float(map_config["occupied_thresh"])
    known_free = occupancy_probability < float(map_config["free_thresh"])
    clearance = distance_transform_edt(~occupied) * resolution
    height, width = image.shape

    def pose_xy(name: str) -> tuple[float, float]:
        return float(routes[name]["x_m"]), float(routes[name]["y_m"])

    def row_column(point: tuple[float, float]) -> tuple[int, int]:
        x_value, y_value = point
        row = int(round(height - 0.5 - (y_value - origin_y) / resolution))
        column = int(round((x_value - origin_x) / resolution - 0.5))
        if not (0 <= row < height and 0 <= column < width):
            raise ValueError(f"test point outside map: {point}")
        return row, column

    def segment_metrics(
        start: tuple[float, float], goal: tuple[float, float]
    ) -> tuple[float, bool]:
        count = max(2, int(math.ceil(math.dist(start, goal) / 0.01)) + 1)
        x_values = np.linspace(start[0], goal[0], count)
        y_values = np.linspace(start[1], goal[1], count)
        rows = np.clip(
            np.rint(height - 0.5 - (y_values - origin_y) / resolution).astype(int),
            0,
            height - 1,
        )
        columns = np.clip(
            np.rint((x_values - origin_x) / resolution - 0.5).astype(int),
            0,
            width - 1,
        )
        return float(clearance[rows, columns].min()), bool(known_free[rows, columns].all())

    # A 0.48 m binary clearance is slightly larger than the 0.448 m footprint
    # circumradius. Inflation costs remain Nav2's responsibility at runtime.
    conservative_clearance_m = 0.48
    traversable = known_free & (clearance >= conservative_clearance_m)

    def shortest_path_m(
        start: tuple[float, float], goal: tuple[float, float]
    ) -> float:
        start_cell, goal_cell = row_column(start), row_column(goal)
        if not traversable[start_cell] or not traversable[goal_cell]:
            return math.inf
        distances = {start_cell: 0.0}
        queue = [
            (
                math.hypot(
                    goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1]
                ),
                0.0,
                *start_cell,
            )
        ]
        moves = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        while queue:
            estimated, cost, row, column = heapq.heappop(queue)
            del estimated
            if cost != distances.get((row, column)):
                continue
            if (row, column) == goal_cell:
                return cost * resolution
            for delta_row, delta_column, step_cost in moves:
                next_row, next_column = row + delta_row, column + delta_column
                if not (
                    0 <= next_row < height
                    and 0 <= next_column < width
                    and traversable[next_row, next_column]
                ):
                    continue
                next_cost = cost + step_cost
                key = (next_row, next_column)
                if next_cost >= distances.get(key, math.inf):
                    continue
                distances[key] = next_cost
                heuristic = math.hypot(
                    goal_cell[0] - next_row, goal_cell[1] - next_column
                )
                heapq.heappush(
                    queue,
                    (next_cost + heuristic, next_cost, next_row, next_column),
                )
        return math.inf

    spawn = pose_xy("spawn_map")
    first = pose_xy("first_goal")
    obstacle = pose_xy("obstacle_goal")
    spawn_yaw = float(routes["spawn_map"]["yaw_rad"])
    omni_distance = float(routes["omni_follow_path"]["local_y_distance_m"])
    omni_target = (
        spawn[0] - math.sin(spawn_yaw) * omni_distance,
        spawn[1] + math.cos(spawn_yaw) * omni_distance,
    )
    first_clearance, first_known_free = segment_metrics(spawn, first)
    obstacle_clearance, obstacle_known_free = segment_metrics(spawn, obstacle)
    omni_clearance, omni_known_free = segment_metrics(spawn, omni_target)
    obstacle_path_length = shortest_path_m(spawn, obstacle)
    first_distance = math.dist(spawn, first)
    obstacle_distance = math.dist(spawn, obstacle)
    endpoint_clearances = {
        name: float(clearance[row_column(point)])
        for name, point in (
            ("spawn", spawn),
            ("first", first),
            ("obstacle", obstacle),
            ("omni", omni_target),
        )
    }
    checks = {
        "first_goal_is_1_to_2_m": 1.0 <= first_distance <= 2.0,
        "first_goal_segment_known_free": first_known_free,
        "first_goal_segment_clearance_ge_0_75_m": first_clearance >= 0.75,
        "obstacle_direct_segment_crosses_occupied": obstacle_clearance <= 0.05,
        "obstacle_goal_endpoint_clearance_ge_0_90_m": endpoint_clearances["obstacle"] >= 0.90,
        "obstacle_conservative_route_exists": math.isfinite(obstacle_path_length),
        "obstacle_route_has_material_detour": (
            math.isfinite(obstacle_path_length)
            and obstacle_path_length / obstacle_distance >= 1.10
        ),
        "omni_distance_is_0_75_to_1_m": 0.75 <= omni_distance <= 1.0,
        "omni_segment_known_free": omni_known_free,
        "omni_segment_clearance_ge_0_75_m": omni_clearance >= 0.75,
    }
    passed = all(checks.values())
    report = {
        "schema": "a_pipeline_level3_test_routes/v1",
        "status": "PASS" if passed else "FAIL",
        "checks": checks,
        "conservative_center_clearance_m": conservative_clearance_m,
        "spawn_map_xy_m": list(spawn),
        "first_goal": {
            "map_xy_m": list(first),
            "straight_distance_m": first_distance,
            "minimum_occupied_clearance_m": first_clearance,
            "segment_all_known_free": first_known_free,
        },
        "obstacle_goal": {
            "map_xy_m": list(obstacle),
            "straight_distance_m": obstacle_distance,
            "minimum_direct_segment_occupied_clearance_m": obstacle_clearance,
            "direct_segment_all_known_free": obstacle_known_free,
            "conservative_route_length_m": obstacle_path_length,
            "conservative_route_detour_ratio": obstacle_path_length / obstacle_distance,
        },
        "omni_follow_path": {
            "target_map_xy_m": list(omni_target),
            "local_y_distance_m": omni_distance,
            "minimum_occupied_clearance_m": omni_clearance,
            "segment_all_known_free": omni_known_free,
        },
        "endpoint_occupied_clearance_m": endpoint_clearances,
    }
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"STATIC_TEST_ROUTE_GEOMETRY={'PASS' if passed else 'FAIL'}")
    print(f"FIRST_GOAL_DISTANCE_M={first_distance:.9f}")
    print(f"FIRST_GOAL_MIN_CLEARANCE_M={first_clearance:.9f}")
    print(f"OBSTACLE_DIRECT_MIN_CLEARANCE_M={obstacle_clearance:.9f}")
    print(f"OBSTACLE_ROUTE_LENGTH_M={obstacle_path_length:.9f}")
    print(f"OBSTACLE_ROUTE_DETOUR_RATIO={obstacle_path_length / obstacle_distance:.9f}")
    print(f"OMNI_PATH_MIN_CLEARANCE_M={omni_clearance:.9f}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
