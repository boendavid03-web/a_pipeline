#!/usr/bin/env python3
"""Stage 2-B: validate real dual scans against the legacy DRL-VO adapter."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from basic_navigation import ROBOT_USD, make_runtime, parse_pose
from drlvo_observation_adapter import (
    DrlvoObservationAdapter,
    OBSERVATION_SIZE,
    PED_MAP_SHAPE,
)
from lidar_sensor import DualPhysxRaycastLidar


HERE = Path(__file__).resolve().parent
GENERATED_ROOT = HERE.parents[0] / "generated"
PHYSICS_DT = 1.0 / 60.0
SAMPLE_EVERY = 6
SAMPLE_COUNT = 2000
FRAME_COUNT = 12


def main() -> int:
    app = world = robot = controller = None
    result: dict[str, object] = {"status": "FAIL", "stage": "2-B"}
    try:
        app, world, robot, controller, _ros = make_runtime(
            "lidar", ROBOT_USD, use_ros=False
        )
        sensor = DualPhysxRaycastLidar(sample_count=SAMPLE_COUNT, rate_hz=10.0)
        adapter = DrlvoObservationAdapter()
        pedestrian_map = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
        # This is a contract fixture only. Stage 2-C replaces it with the
        # measured minimal human backend; it is not used as final inference.
        pedestrian_map[0, 4, 40] = 0.4
        observations = []
        sim_time = 0.0
        for step in range(FRAME_COUNT * SAMPLE_EVERY):
            controller.apply(0.0, 0.0, 0.0)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            if step % SAMPLE_EVERY != 0:
                continue
            position, orientation, yaw = parse_pose(robot)
            pair = sensor.sample(sim_time, position, orientation)
            adapted = adapter.adapt(
                pair,
                np.asarray([position[0], position[1], yaw], dtype=np.float32),
                np.asarray([0.5, 0.0], dtype=np.float32),
                pedestrian_map,
            )
            observations.append(
                {
                    "timestamp_ns": adapted.timestamp_ns,
                    "history_shape": list(adapted.scan_history.shape),
                    "front_scan_shape": list(adapted.front_scan.shape),
                    "observation_shape": list(adapted.observation.shape),
                    "local_goal_frame": adapted.local_goal.frame_id,
                    "final_goal_frame": adapted.final_goal.frame_id,
                }
            )
        checks = {
            "observation_size": all(
                row["observation_shape"] == [OBSERVATION_SIZE] for row in observations
            ),
            "history_shape": all(
                row["history_shape"] == [10, 720] for row in observations
            ),
            "front_scan_shape": all(
                row["front_scan_shape"] == [720] for row in observations
            ),
            "pedestrian_map_shape": pedestrian_map.shape == PED_MAP_SHAPE,
            "strict_timestamps": all(
                first["timestamp_ns"] < second["timestamp_ns"]
                for first, second in zip(observations, observations[1:])
            ),
            "goal_frames": all(
                row["local_goal_frame"] == "base_link"
                and row["final_goal_frame"] == "odom"
                for row in observations
            ),
            "action_conversion": DrlvoObservationAdapter.normalized_action_to_cmd_vel(
                np.asarray([0.0, -0.5], dtype=np.float32)
            )[0]
            == 0.25,
        }
        result = {
            "status": "PASS" if observations and all(checks.values()) else "FAIL",
            "stage": "2-B",
            "experience": str(GENERATED_ROOT / "minimal_core_physx_no_rtx.kit"),
            "legacy_adapter": str(
                Path(__file__).resolve().parents[3]
                / "sim_to_real/robot/comparison_models/runtime_code/methods/experiments"
                / "drl_vo_ros2_offline/observation_adapter.py"
            ),
            "observations": len(observations),
            "checks": checks,
            "sample": observations[:2],
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "2-B", "error": repr(exc)}
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
            "PASS" if not teardown["errors"] and teardown["app_close"] else "FAIL"
        )
        result["teardown"] = teardown
        print("ISAAC5_STAGE2B_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
