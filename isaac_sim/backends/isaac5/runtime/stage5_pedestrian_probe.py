#!/usr/bin/env python3
"""Stage 5: validate a minimal kinematic pedestrian in Isaac5."""

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
    result: dict[str, object] = {"status": "FAIL", "stage": "5"}
    try:
        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from lidar_sensor import PhysxRaycastLidar
        from minimal_human_backend import MinimalHumanBackend, MinimalHumanState

        app, world, robot, controller, _ = make_runtime(
            "lidar", ROBOT_USD, use_ros=False, dual_scan=False
        )
        from isaacsim.core.api.objects import DynamicCapsule

        human = MinimalHumanState(
            track_id=7,
            position_xy_map=np.asarray([-1.4, -0.5], dtype=np.float32),
            velocity_xy_map_absolute=np.zeros(2, dtype=np.float32),
            goal_xy_map=np.asarray([-1.4, 0.8], dtype=np.float32),
            speed_mps=0.35,
        )
        human_backend = MinimalHumanBackend([human])
        human_body = world.scene.add(
            DynamicCapsule(
                prim_path="/World/MinimalHuman/person_07",
                name="minimal_human_07",
                position=np.asarray([-1.4, -0.5, 0.85]),
                radius=0.24,
                height=1.2,
                color=np.asarray([0.2, 0.6, 0.9]),
                mass=70.0,
            )
        )
        sensor = PhysxRaycastLidar(
            sample_count=360,
            rate_hz=10.0,
            range_min=0.2,
            range_max=8.0,
            frame_id="base_scan",
        )
        samples: list[dict[str, object]] = []
        sim_time = 0.0
        start_wall = time.monotonic()
        for step in range(60):
            human_backend.step(1.0 / 60.0)
            human_body.set_world_pose(
                np.asarray([human.position_xy_map[0], human.position_xy_map[1], 0.85]),
                np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            controller.apply(0.0, 0.0, 0.0)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            if step % 6 != 0:
                continue
            position, orientation, yaw = parse_pose(robot)
            scan = sensor.sample(sim_time, position, orientation)
            observations = human_backend.relative_observations(position[:2], yaw)
            finite_ranges = np.asarray(
                [value for value in scan.ranges if np.isfinite(value)], dtype=float
            )
            samples.append(
                {
                    "sim_time": sim_time,
                    "human_position": human.position_xy_map.tolist(),
                    "human_velocity": human.velocity_xy_map_absolute.tolist(),
                    "human_goal": human.goal_xy_map.tolist(),
                    "relative_position": observations[0]["relative_position_xy_base"].tolist(),
                    "relative_velocity": observations[0]["relative_velocity_xy_base"].tolist(),
                    "scan_beams": len(scan.ranges),
                    "finite_scan_beams": int(finite_ranges.size),
                    "scan_min_range": float(finite_ranges.min()) if finite_ranges.size else None,
                }
            )

        stage = __import__("isaacsim.core.utils.stage", fromlist=["get_current_stage"])
        human_prim = stage.get_current_stage().GetPrimAtPath("/World/MinimalHuman/person_07")
        human_positions = np.asarray([row["human_position"] for row in samples], dtype=float)
        checks = {
            "human_prim_composed": human_prim.IsValid(),
            "position_velocity_goal_finite": all(
                np.isfinite(np.asarray(row[key], dtype=float)).all()
                for row in samples
                for key in ("human_position", "human_velocity", "human_goal")
            ),
            "human_motion_observed": len(samples) >= 2 and bool(
                np.linalg.norm(human_positions[-1] - human_positions[0]) > 0.05
            ),
            "relative_observation_finite": all(
                np.isfinite(np.asarray(row["relative_position"], dtype=float)).all()
                and np.isfinite(np.asarray(row["relative_velocity"], dtype=float)).all()
                for row in samples
            ),
            "scan_contract_present": all(
                row["scan_beams"] == 360 and row["finite_scan_beams"] > 0
                for row in samples
            ),
        }
        result = {
            "status": "PASS" if samples and all(checks.values()) else "FAIL",
            "stage": "5",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "backend": "minimal_kinematic_human_no_ira",
            "pedestrian_count": 1,
            "frames": len(samples),
            "samples": samples[:3],
            "last_sample": samples[-1] if samples else None,
            "wall_time_s": time.monotonic() - start_wall,
            "checks": checks,
            "perception_semantics": "ground_truth_relative_observation; no detector/tracker",
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "5", "error": repr(exc)}
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
        print("ISAAC5_STAGE5_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
