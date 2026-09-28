#!/usr/bin/env python3
"""Stage 6: validate minimal Social Force motion and robot avoidance."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
BACKEND_ROOT = HERE.parents[0]


def main() -> int:
    from isaacsim import SimulationApp

    app = world = robot = controller = human_body = None
    result: dict[str, object] = {"status": "FAIL", "stage": "6"}
    try:
        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from social_navigation import SocialForceHuman, SocialNavigationPolicy

        app, world, robot, controller, _ = make_runtime(
            "navigation", ROBOT_USD, use_ros=False, dual_scan=False
        )
        from isaacsim.core.api.objects import DynamicCapsule

        human = SocialForceHuman(
            position_xy=(0.85, -0.9),
            goal_xy=(0.85, 0.9),
            speed_mps=0.3,
            personal_space_m=0.8,
            repulsion_gain=0.45,
        )
        human_body = world.scene.add(
            DynamicCapsule(
                prim_path="/World/SocialHuman/person_01",
                name="social_human_01",
                position=np.asarray([human.position_xy[0], human.position_xy[1], 0.85]),
                radius=0.24,
                height=1.2,
                color=np.asarray([0.9, 0.45, 0.2]),
                mass=70.0,
            )
        )
        policy = SocialNavigationPolicy(
            (1.8, 0.0),
            max_linear_mps=0.35,
            influence_radius_m=1.1,
            personal_space_m=0.8,
            repulsion_gain=0.65,
        )
        robot_radius = 0.36
        human_radius = 0.24
        dt = 1.0 / 60.0
        trajectory: list[dict[str, object]] = []
        min_separation = float("inf")
        max_abs_y = 0.0
        start_wall = time.monotonic()

        for _step in range(480):
            robot_position, _, robot_yaw = parse_pose(robot)
            human.step(robot_position[:2], dt)
            human_body.set_world_pose(
                np.asarray([human.position_xy[0], human.position_xy[1], 0.85]),
                np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            command = policy.command(robot_position[:2], robot_yaw, [human.position_xy])
            controller.apply(*command)
            world.step(render=False, step_sim=True)
            robot_position, _, robot_yaw = parse_pose(robot)
            separation = float(np.linalg.norm(robot_position[:2] - human.position_xy))
            min_separation = min(min_separation, separation)
            max_abs_y = max(max_abs_y, abs(float(robot_position[1])))
            if _step % 30 == 0:
                trajectory.append(
                    {
                        "sim_time": float(world.current_time),
                        "robot_position": robot_position.tolist(),
                        "human_position": human.position_xy.tolist(),
                        "human_velocity": human.velocity_xy.tolist(),
                        "separation_m": separation,
                        "command": list(command),
                    }
                )

        robot_position, _, _ = parse_pose(robot)
        goal_distance = float(np.linalg.norm(robot_position[:2] - policy.goal_xy))
        human_displacement = float(np.linalg.norm(human.position_xy - np.asarray([0.85, -0.9])))
        checks = {
            "human_motion_finite": bool(
                np.isfinite(human.position_xy).all()
                and np.isfinite(human.velocity_xy).all()
            ),
            "human_moved": human_displacement > 0.3,
            "robot_progress": float(robot_position[0]) > 0.4,
            "no_direct_collision": min_separation > robot_radius + human_radius,
            "personal_space_maintained": min_separation > 0.65,
            "avoidance_lateral_motion": max_abs_y > 0.03,
            "finite_goal_distance": bool(np.isfinite(goal_distance)),
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "6",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "backend": "minimal_social_force_pair_no_ira",
            "sim_time_s": float(world.current_time),
            "human_start": [0.85, -0.9],
            "human_goal": [0.85, 0.9],
            "robot_goal": [1.8, 0.0],
            "human_displacement_m": human_displacement,
            "robot_final_position": robot_position.tolist(),
            "robot_goal_distance_m": goal_distance,
            "min_robot_human_separation_m": min_separation,
            "max_robot_abs_y_m": max_abs_y,
            "trajectory": trajectory,
            "checks": checks,
            "social_semantics": "minimal pairwise repulsion and goal-seeking; not full social-navigation acceptance",
            "wall_time_s": time.monotonic() - start_wall,
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "6", "error": repr(exc)}
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
        print("ISAAC5_STAGE6_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
