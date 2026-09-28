#!/usr/bin/env python3
"""Stage 4: validate reset/multiple-goal episode behavior in Isaac5."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
BACKEND_ROOT = HERE.parents[0]


def main() -> int:
    from isaacsim import SimulationApp

    app = world = robot = controller = None
    result: dict[str, object] = {"status": "FAIL", "stage": "4"}
    try:
        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from episode_environment import NavigationEpisodeEnvironment, ObstacleBox
        from navigation_interface import ManualGoalController

        app, world, robot, controller, _ = make_runtime(
            "navigation", ROBOT_USD, use_ros=False, dual_scan=False
        )
        obstacle_specs = (
            ObstacleBox("north", (0.0, 2.5), (2.5, 0.05)),
            ObstacleBox("south", (0.0, -2.5), (2.5, 0.05)),
            ObstacleBox("east", (2.5, 0.0), (0.05, 2.5)),
            ObstacleBox("west", (-2.5, 0.0), (0.05, 2.5)),
            ObstacleBox("box", (1.0, 0.7), (0.25, 0.25)),
        )
        goals = ((0.5, 0.0), (-0.5, 0.0), (0.0, 0.5))
        environment = NavigationEpisodeEnvironment(
            goals,
            position_tolerance=0.08,
            max_episode_time_s=5.0,
            obstacles=obstacle_specs,
        )
        episode_rows: list[dict[str, object]] = []
        reset_rows: list[dict[str, object]] = []
        start_pose = (0.0, 0.0, 0.02)
        dt = 1.0 / 60.0
        start_wall = time.monotonic()

        for goal in goals:
            record = environment.reset_episode(world, robot, start_pose)
            reset_position, _, reset_yaw = parse_pose(robot)
            reset_error = float(np.linalg.norm(reset_position[:2] - np.asarray(start_pose[:2])))
            reset_rows.append(
                {
                    "episode_id": record.episode_id,
                    "goal": list(goal),
                    "reset_position": reset_position.tolist(),
                    "reset_error_m": reset_error,
                    "reset_yaw": reset_yaw,
                }
            )
            policy = ManualGoalController(goal[0], goal[1], 0.0)
            sim_time = 0.0
            while not environment.done and sim_time < 5.0:
                position, _, yaw = parse_pose(robot)
                command = policy.command(float(position[0]), float(position[1]), yaw)
                controller.apply(*command)
                world.step(render=False, step_sim=True)
                sim_time = float(world.current_time)
                position, _, _ = parse_pose(robot)
                environment.observe(position[:2], sim_time)
            episode_rows.append(
                {
                    "episode_id": record.episode_id,
                    "goal": list(record.goal_xy),
                    "status": record.status,
                    "steps": record.steps,
                    "sim_time_s": record.sim_time_s,
                    "goal_distance_m": record.goal_distance_m,
                    "collision_obstacle": record.collision_obstacle,
                    "failure_reason": record.failure_reason,
                }
            )

        from isaacsim.core.utils import stage as stage_utils

        stage = stage_utils.get_current_stage()
        obstacle_prims = [
            str(path)
            for path in (
                "/World/LidarTest/north",
                "/World/LidarTest/south",
                "/World/LidarTest/east",
                "/World/LidarTest/west",
                "/World/LidarTest/box",
            )
            if stage.GetPrimAtPath(path).IsValid()
        ]
        checks = {
            "multiple_goals": len(goals) == 3,
            "world_obstacles_composed": len(obstacle_prims) == 5,
            "all_resets_within_tolerance": all(row["reset_error_m"] < 0.05 for row in reset_rows),
            "all_episodes_success": all(row["status"] == "SUCCESS" for row in episode_rows),
            "no_geometric_collision": all(row["collision_obstacle"] is None for row in episode_rows),
            "finite_episode_metrics": all(
                np.isfinite(float(row["goal_distance_m"])) for row in episode_rows
            ),
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "4",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "robot_usd": str(ROBOT_USD),
            "goals": [list(goal) for goal in goals],
            "obstacle_prims": obstacle_prims,
            "reset_rows": reset_rows,
            "episodes": episode_rows,
            "wall_time_s": time.monotonic() - start_wall,
            "checks": checks,
            "collision_semantics": "geometric_robot_footprint_vs_obstacle_aabb",
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "4", "error": repr(exc)}
        return 1
    finally:
        teardown = {"world_stop": False, "app_close": False, "errors": []}
        if world is not None:
            try:
                world.stop()
                teardown["world_stop"] = True
            except Exception as exc:
                teardown["errors"].append(f"world_stop: {exc!r}")
        if app is not None:
            try:
                app.close()
                teardown["app_close"] = True
            except Exception as exc:
                teardown["errors"].append(f"app_close: {exc!r}")
        teardown["status"] = (
            "PASS"
            if not teardown["errors"] and teardown["world_stop"] and teardown["app_close"]
            else "FAIL"
        )
        result["teardown"] = teardown
        print("ISAAC5_STAGE4_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
