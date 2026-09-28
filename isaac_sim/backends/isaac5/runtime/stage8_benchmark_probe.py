#!/usr/bin/env python3
"""Stage 8: validate the independent Isaac5 related benchmark harness."""

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
    result: dict[str, object] = {"status": "FAIL", "stage": "8"}
    try:
        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from benchmark_runner import Scenario, summarize_episode_metrics
        from episode_environment import NavigationEpisodeEnvironment, ObstacleBox
        from navigation_interface import ManualGoalController

        app, world, robot, controller, _ = make_runtime(
            "navigation", ROBOT_USD, use_ros=False, dual_scan=False
        )
        scenarios = (
            Scenario("goal_forward", (0.5, 0.0)),
            Scenario("goal_reverse", (-0.5, 0.0)),
            Scenario("goal_lateral", (0.0, 0.5)),
        )
        obstacle_specs = (
            ObstacleBox("north", (0.0, 2.5), (2.5, 0.05)),
            ObstacleBox("south", (0.0, -2.5), (2.5, 0.05)),
            ObstacleBox("east", (2.5, 0.0), (0.05, 2.5)),
            ObstacleBox("west", (-2.5, 0.0), (0.05, 2.5)),
            ObstacleBox("box", (1.0, 0.7), (0.25, 0.25)),
        )
        rows: list[dict[str, object]] = []
        reset_rows: list[dict[str, object]] = []
        start_wall = time.monotonic()

        for scenario in scenarios:
            environment = NavigationEpisodeEnvironment(
                [scenario.goal_xy],
                position_tolerance=0.08,
                max_episode_time_s=5.0,
                obstacles=obstacle_specs,
            )
            record = environment.reset_episode(world, robot, (0.0, 0.0, 0.02))
            reset_position, _, _ = parse_pose(robot)
            reset_error = float(np.linalg.norm(reset_position[:2]))
            reset_rows.append(
                {
                    "scenario_id": scenario.scenario_id,
                    "reset_error_m": reset_error,
                }
            )
            policy = ManualGoalController(*scenario.goal_xy, 0.0)
            dt = 1.0 / 60.0
            while not environment.done and float(world.current_time) < 5.0:
                position, _, yaw = parse_pose(robot)
                command = policy.command(float(position[0]), float(position[1]), yaw)
                controller.apply(*command)
                world.step(render=False, step_sim=True)
                position, _, _ = parse_pose(robot)
                environment.observe(position[:2], float(world.current_time))
            rows.append(
                {
                    "scenario_id": scenario.scenario_id,
                    "goal_xy": list(record.goal_xy),
                    "status": record.status,
                    "steps": record.steps,
                    "sim_time_s": record.sim_time_s,
                    "goal_distance_m": record.goal_distance_m,
                    "collision_obstacle": record.collision_obstacle,
                    "failure_reason": record.failure_reason,
                }
            )

        metrics = summarize_episode_metrics(rows)
        checks = {
            "scenario_count": len(scenarios) == 3,
            "reset_contract": all(row["reset_error_m"] < 0.05 for row in reset_rows),
            "success_rate_one": metrics["success_rate"] == 1.0,
            "no_collisions": metrics["collision_count"] == 0,
            "no_timeouts": metrics["timeout_count"] == 0,
            "finite_goal_metric": metrics["mean_goal_distance_m"] is not None
            and bool(np.isfinite(float(metrics["mean_goal_distance_m"]))),
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "8",
            "benchmark_backend": "isaac5_related_benchmark_no_arena",
            "arena_native_integration": False,
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "scenarios": [scenario.__dict__ for scenario in scenarios],
            "reset_rows": reset_rows,
            "episodes": rows,
            "metrics": metrics,
            "checks": checks,
            "wall_time_s": time.monotonic() - start_wall,
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "8", "error": repr(exc)}
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
        print("ISAAC5_STAGE8_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
