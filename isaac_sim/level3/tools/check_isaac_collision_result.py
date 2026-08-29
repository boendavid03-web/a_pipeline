#!/usr/bin/env python3
"""Combine a completed Isaac RESULT line with a Level 3 goal result."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


RESULT_PREFIX = "WAREHOUSE_PEOPLE_ROBOT_RESULT="
READY_PREFIX = "WAREHOUSE_PEOPLE_ROBOT_READY="
EXPECTED_SCENE_USD = (
    "/home/user/navigation_project/a_pipeline/isaac_sim/scenes/"
    "a_pipeline_eng_lobby.usda"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("isaac_log", type=Path)
    parser.add_argument("--goal-result", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = []
    ready_records = []
    for line in args.isaac_log.read_text(encoding="utf-8", errors="replace").splitlines():
        if READY_PREFIX in line:
            ready_records.append(json.loads(line.split(READY_PREFIX, 1)[1]))
        if RESULT_PREFIX in line:
            results.append(json.loads(line.split(RESULT_PREFIX, 1)[1]))
    if not ready_records:
        raise ValueError(f"no {READY_PREFIX} line in {args.isaac_log}")
    if not results:
        raise ValueError(f"no {RESULT_PREFIX} line in {args.isaac_log}")
    isaac_result = results[-1]
    ready = ready_records[-1]
    spawn = ready.get("robot_spawn", [])
    checks = {
        "custom_static_scene": (
            ready.get("scene") == "custom"
            and ready.get("scene_usd") == EXPECTED_SCENE_USD
            and ready.get("stage_up_axis") == "Z"
            and abs(float(ready.get("stage_meters_per_unit", math.nan)) - 1.0)
            <= 1.0e-9
            and len(spawn) == 3
            and all(
                abs(float(actual) - expected) <= 1.0e-6
                for actual, expected in zip(spawn, (2.0, 2.0, 0.30))
            )
        ),
        "static_sensor_contract": (
            ready.get("lidar_mode") == "rtx"
            and ready.get("lidar_profile") == "rplidar_s2e"
            and int(ready.get("lidar_samples", -1)) == 360
            and int(ready.get("lidar_rate_hz", -1)) == 10
            and ready.get("people_enabled") is False
            and ready.get("robot_collision_protection") is True
        ),
        "isaac_result_pass": isaac_result.get("status") == "PASS",
        "collision_blocked_count_zero": int(
            isaac_result.get("collision_blocked_count", -1)
        ) == 0,
        "robot_collision_protection_enabled": (
            isaac_result.get("robot_collision_protection") is True
        ),
        "people_disabled": int(isaac_result.get("people", -1)) == 0,
        "isaac_received_navigation_commands": int(
            isaac_result.get("ros_cmd_vel_messages_received", 0)
        ) > 0,
        "isaac_robot_moved": float(
            isaac_result.get("robot_planar_displacement_m", 0.0)
        ) >= 0.10,
    }
    goal_result = json.loads(args.goal_result.read_text(encoding="utf-8"))
    goal_displacement = float(goal_result.get("odom_displacement_m", math.nan))
    isaac_displacement = float(
        isaac_result.get("robot_planar_displacement_m", math.nan)
    )
    checks["goal_result_pass"] = goal_result.get("status") == "PASS"
    checks["goal_and_isaac_motion_agree"] = (
        math.isfinite(goal_displacement)
        and math.isfinite(isaac_displacement)
        and abs(goal_displacement - isaac_displacement) <= 0.20
    )
    passed = all(checks.values())
    for name, value in checks.items():
        print(f"TEST_{name.upper()}={'PASS' if value else 'FAIL'}")
    print(f"LEVEL3_COLLISION_RESULT={'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
