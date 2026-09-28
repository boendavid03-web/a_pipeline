#!/usr/bin/env python3
"""Stage 2-C: validate a measured minimal-human pedestrian-map contract."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

from basic_navigation import ROBOT_USD, make_runtime, parse_pose
from drlvo_observation_adapter import DrlvoObservationAdapter, PED_MAP_SHAPE
from lidar_sensor import DualPhysxRaycastLidar
from minimal_human_backend import MinimalHumanBackend, MinimalHumanState


HERE = Path(__file__).resolve().parent
WORKSPACE_ROOT = HERE.parents[3]
GENERATED_ROOT = HERE.parents[0] / "generated"
PHYSICS_DT = 1.0 / 60.0
SAMPLE_EVERY = 6
SAMPLE_COUNT = 2000
FRAME_COUNT = 8


def legacy_map_converter():
    path = (
        WORKSPACE_ROOT
        / "sim_to_real/robot/comparison_models/runtime_code/methods/experiments"
        / "drl_vo_ros2_offline/observation_adapter.py"
    ).resolve()
    spec = importlib.util.spec_from_file_location("stage2c_legacy_adapter", path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    app = world = robot = controller = human_body = None
    result: dict[str, object] = {"status": "FAIL", "stage": "2-C"}
    try:
        app, world, robot, controller, _ros = make_runtime(
            "lidar", ROBOT_USD, use_ros=False
        )
        from isaacsim.core.api.objects import DynamicCapsule

        human = MinimalHumanState(
            track_id=1,
            position_xy_map=np.asarray([2.0, 0.0], dtype=np.float32),
            velocity_xy_map_absolute=np.zeros(2, dtype=np.float32),
            goal_xy_map=np.asarray([2.0, 1.2], dtype=np.float32),
            speed_mps=0.6,
        )
        human_backend = MinimalHumanBackend([human])
        human_body = world.scene.add(
            DynamicCapsule(
                prim_path="/World/MinimalHuman/person_01",
                name="minimal_human_01",
                position=np.asarray([2.0, 0.0, 0.85]),
                radius=0.24,
                height=1.2,
                color=np.asarray([0.2, 0.6, 0.9]),
                mass=70.0,
            )
        )
        sensor = DualPhysxRaycastLidar(sample_count=SAMPLE_COUNT, rate_hz=10.0)
        adapter = DrlvoObservationAdapter()
        legacy = legacy_map_converter()
        samples = []
        sim_time = 0.0
        for step in range(FRAME_COUNT * SAMPLE_EVERY):
            human_backend.step(PHYSICS_DT)
            human_body.set_world_pose(
                np.asarray([human.position_xy_map[0], human.position_xy_map[1], 0.85]),
                np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            controller.apply(0.0, 0.0, 0.0)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            if step % SAMPLE_EVERY != 0:
                continue
            position, orientation, yaw = parse_pose(robot)
            pair = sensor.sample(sim_time, position, orientation)
            pose = np.asarray([position[0], position[1], yaw], dtype=np.float32)
            ped_map, diagnostics = legacy.tracks_to_drl_vo_ped_map_with_diagnostics(
                human_backend.tracks(), pose
            )
            adapted = adapter.adapt(
                pair,
                pose,
                np.asarray([0.5, 0.0], dtype=np.float32),
                ped_map,
            )
            samples.append(
                {
                    "sim_time": sim_time,
                    "human_position": human.position_xy_map.tolist(),
                    "human_velocity": human.velocity_xy_map_absolute.tolist(),
                    "human_goal": human.goal_xy_map.tolist(),
                    "map_shape": list(ped_map.shape),
                    "map_nonzero": int(np.count_nonzero(ped_map)),
                    "written_track_ids": diagnostics["written_track_ids"],
                    "observation_shape": list(adapted.observation.shape),
                }
            )
        checks = {
            "human_state_finite": all(
                np.isfinite(np.asarray(row[key], dtype=float)).all()
                for row in samples
                for key in ("human_position", "human_velocity", "human_goal")
            ),
            "human_goal_defined": all(
                np.linalg.norm(np.asarray(row["human_goal"]) - np.asarray(row["human_position"]))
                >= 0.0
                for row in samples
            ),
            "map_shape": all(row["map_shape"] == list(PED_MAP_SHAPE) for row in samples),
            "map_contains_measured_track": all(row["map_nonzero"] > 0 for row in samples),
            "track_written": all(1 in row["written_track_ids"] for row in samples),
            "observation_shape": all(row["observation_shape"] == [19202] for row in samples),
        }
        result = {
            "status": "PASS" if samples and all(checks.values()) else "FAIL",
            "stage": "2-C",
            "experience": str(GENERATED_ROOT / "minimal_core_physx_no_rtx.kit"),
            "backend": "minimal_kinematic_human_no_ira",
            "frames": len(samples),
            "checks": checks,
            "sample": samples[:2],
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "2-C", "error": repr(exc)}
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
        print("ISAAC5_STAGE2C_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
