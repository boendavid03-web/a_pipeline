#!/usr/bin/env python3
"""Isaac5 custom-experience basic navigation gates.

This entry point is deliberately separate from the historical Isaac5 runner.
It uses the no-RTX experience, creates only the Mecanum robot and a local
physics floor, and has no pedestrian, Arena, Nav2, or perception dependency.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from lidar_sensor import PhysxRaycastLidar
from navigation_interface import ManualGoalController, NavigationObservation
from policy_adapter import ManualGoalPolicy, PolicyAdapter
from robot_controller import MecanumController, clamp_twist, yaw_delta, yaw_from_quaternion


HERE = Path(__file__).resolve().parent
BACKEND_ROOT = HERE.parents[0]
WORKSPACE_ROOT = HERE.parents[4]
GENERATED_ROOT = BACKEND_ROOT / "generated"
EXPERIENCE = GENERATED_ROOT / "minimal_core_physx_no_rtx.kit"
ROBOT_USD = (
    WORKSPACE_ROOT
    / "robot_related"
    / "robots"
    / "chassis_arm"
    / "motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
).resolve()
ROBOT_PRIM = "/World/Robot"
PHYSICS_DT = 1.0 / 60.0
LIDAR_EVERY_STEPS = 6


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("control", "ros", "lidar", "navigation"), default="control")
    parser.add_argument("--duration", type=float, default=4.0, help="Simulation seconds for ros/lidar/navigation")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--fast", action="store_true", help="Do not pace the simulation to wall time")
    parser.add_argument("--ros", action="store_true", help="Enable Isaac5 ROS 2 bridge and publish/subscribe topics")
    parser.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW"), default=(1.0, 0.0, 0.0))
    parser.add_argument("--test-command", nargs=3, type=float, metavar=("VX", "VY", "WZ"))
    parser.add_argument("--robot-usd", type=Path, default=ROBOT_USD)
    return parser.parse_args()


def add_local_floor(world: object) -> None:
    from isaacsim.core.api.objects import FixedCuboid

    world.scene.add(
        FixedCuboid(
            prim_path="/World/LocalFloor",
            name="isaac5_local_floor",
            position=np.asarray([0.0, 0.0, -0.05]),
            scale=np.asarray([12.0, 12.0, 0.10]),
            color=np.asarray([0.25, 0.25, 0.25]),
        )
    )


def add_lidar_obstacles(world: object) -> None:
    """Provide deterministic non-robot geometry for scene-query validation."""

    from isaacsim.core.api.objects import FixedCuboid

    specs = (
        ("north", (0.0, 2.5, 0.75), (5.0, 0.10, 1.5)),
        ("south", (0.0, -2.5, 0.75), (5.0, 0.10, 1.5)),
        ("east", (2.5, 0.0, 0.75), (0.10, 5.0, 1.5)),
        ("west", (-2.5, 0.0, 0.75), (0.10, 5.0, 1.5)),
        ("box", (1.0, 0.7, 0.45), (0.50, 0.50, 0.90)),
    )
    for name, position, scale in specs:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/LidarTest/{name}",
                name=f"lidar_test_{name}",
                position=np.asarray(position),
                scale=np.asarray(scale),
                color=np.asarray([0.55, 0.58, 0.62]),
            )
        )


def parse_pose(robot: object) -> tuple[np.ndarray, np.ndarray, float]:
    position, orientation = robot.get_world_pose()
    position = np.asarray(position, dtype=float)
    orientation = np.asarray(orientation, dtype=float)
    return position, orientation, yaw_from_quaternion(orientation)


def make_runtime(
    phase: str,
    robot_usd: Path,
    use_ros: bool,
    dual_scan: bool = False,
):
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "multi_gpu": False,
            "renderer": "Wireframe",
            "create_new_stage": True,
            "fast_shutdown": False,
            "enable_crashreporter": False,
            "limit_cpu_threads": 8,
            "extra_args": ["--/exts/omni.kit.registry.nucleus/enable=false"],
        },
        experience=str(EXPERIENCE),
    )
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils import stage as stage_utils
    # Do not enable isaacsim.ros2.bridge here.  Its 5.1 manifest pulls the
    # RTX sensor/viewport dependency closure.  ros_bridge.py uses the bundled
    # Python 3.11 Humble packages directly, which is sufficient for the
    # explicit /cmd_vel, /clock, /odom and TF topic contract.
    stage_utils.create_new_stage()
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    add_local_floor(world)
    if phase in ("lidar", "navigation"):
        add_lidar_obstacles(world)
    stage_utils.add_reference_to_stage(str(robot_usd), ROBOT_PRIM)
    robot = world.scene.add(
        Robot(
            prim_path=ROBOT_PRIM,
            name="mecanum730_isaac5_basic",
            position=np.asarray([0.0, 0.0, 0.02]),
            orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
        )
    )
    world.reset()
    robot.set_world_pose(np.asarray([0.0, 0.0, 0.02]), np.asarray([1.0, 0.0, 0.0, 0.0]))
    controller = MecanumController(robot)
    ros = None
    if use_ros:
        from ros_bridge import RosControlBridge

        ros = RosControlBridge(
            use_scan=phase == "lidar" and not dual_scan,
            use_dual_scan=dual_scan,
        )
    return app, world, robot, controller, ros


def step_once(app: object, world: object, robot: object, controller: MecanumController, command: tuple[float, float, float], ros: object | None, lidar: PhysxRaycastLidar | None, sim_time: float, scan_records: list[dict]) -> float:
    controller.apply(*command)
    world.step(render=False, step_sim=True)
    sim_time = float(world.current_time)
    position, orientation, _ = parse_pose(robot)
    if ros is not None:
        ros.publish_clock(sim_time)
        ros.publish_state(sim_time, position, orientation, command)
    if lidar is not None and int(round(sim_time / PHYSICS_DT)) % LIDAR_EVERY_STEPS == 0:
        scan = lidar.sample(sim_time, position, orientation)
        if ros is not None:
            ros.publish_scan(scan)
        finite = [value for value in scan.ranges if math.isfinite(value)]
        scan_records.append(
            {
                "sim_time": scan.sim_time,
                "frame_id": scan.frame_id,
                "beam_count": len(scan.ranges),
                "finite_beams": len(finite),
                "min_range": min(finite) if finite else None,
                "max_range_observed": max(finite) if finite else None,
                "sample_ranges": scan.ranges[::60],
            }
        )
    return sim_time


def run_control(app: object, world: object, robot: object, controller: MecanumController, ros: object | None, fast: bool) -> dict:
    cases = (
        ("forward", (0.25, 0.0, 0.0)),
        ("reverse", (-0.25, 0.0, 0.0)),
        ("lateral_left", (0.0, 0.25, 0.0)),
        ("rotate_ccw", (0.0, 0.0, 0.5)),
    )
    results = []
    for name, command in cases:
        world.reset()
        robot.set_world_pose(np.asarray([0.0, 0.0, 0.02]), np.asarray([1.0, 0.0, 0.0, 0.0]))
        start_position, start_orientation, _ = parse_pose(robot)
        samples = []
        for step in range(45):
            sim_start = time.monotonic()
            step_once(app, world, robot, controller, command, ros, None, 0.0, [])
            if step in (0, 14, 44):
                position, orientation, yaw = parse_pose(robot)
                samples.append(
                    {
                        "step": step + 1,
                        "position": position.tolist(),
                        "yaw": yaw,
                        "wheel_state": controller.wheel_state(),
                    }
                )
            if not fast:
                time.sleep(max(0.0, PHYSICS_DT - (time.monotonic() - sim_start)))
        end_position, end_orientation, end_yaw = parse_pose(robot)
        results.append(
            {
                "name": name,
                "command": list(command),
                "wheel_command_rad_s": controller.last_wheel_command.tolist(),
                "start_position": start_position.tolist(),
                "end_position": end_position.tolist(),
                "delta_position": (end_position - start_position).tolist(),
                "yaw_delta": yaw_delta(start_orientation, end_orientation),
                "end_yaw": end_yaw,
                "wheel_state": controller.wheel_state(),
                "samples": samples,
            }
        )
    checks = {
        "wheel_contract": all(item["wheel_state"]["indices"] == [0, 1, 2, 3] for item in results),
        "forward_x_positive": results[0]["delta_position"][0] > 0.02,
        "reverse_x_negative": results[1]["delta_position"][0] < -0.02,
        "lateral_y_nonzero": abs(results[2]["delta_position"][1]) > 0.02,
        "rotation_yaw_nonzero": abs(results[3]["yaw_delta"]) > 0.05,
        "finite_state": all(np.all(np.isfinite(np.asarray(item["end_position"]))) for item in results),
    }
    return {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks, "cases": results}


def run_stream(app: object, world: object, robot: object, controller: MecanumController, ros: object | None, duration: float, fast: bool, phase: str, goal: tuple[float, float, float] | None, test_command: tuple[float, float, float] | None) -> dict:
    # The imported robot is approximately 0.7 m wide.  A 1 m near range is
    # intentional here: it is the established Isaac5 scene-query contract
    # and prevents the sensor mount/chassis from becoming every beam's hit.
    lidar = PhysxRaycastLidar(sample_count=360, rate_hz=10.0, range_min=1.0) if phase in ("lidar", "navigation") else None
    nav = ManualGoalController(*goal) if goal is not None else None
    policy = PolicyAdapter(ManualGoalPolicy()) if goal is not None else None
    scan_records: list[dict] = []
    pose_start, orientation_start, _ = parse_pose(robot)
    sim_time = 0.0
    steps = max(1, int(round(duration / PHYSICS_DT)))
    wall_start = time.monotonic()
    commands = []
    latest_scan = None
    for _ in range(steps):
        position, orientation, yaw = parse_pose(robot)
        if ros is not None:
            ros.spin_once()
            command = ros.current_command()
        elif nav is not None:
            observation = NavigationObservation(
                odom={"pose": (float(position[0]), float(position[1]), yaw)},
                tf={"parent": "odom", "child": "base_link"},
                laser_scan=latest_scan,
                goal=goal,
            )
            command = policy.cmd_vel(observation)
        elif test_command is not None:
            command = clamp_twist(*test_command)
        else:
            command = (0.0, 0.0, 0.0)
        sim_time = step_once(app, world, robot, controller, command, ros, lidar, sim_time, scan_records)
        if scan_records and lidar is not None:
            latest_scan = scan_records[-1]
        commands.append(list(command))
        if not fast:
            deadline = wall_start + sim_time
            time.sleep(max(0.0, deadline - time.monotonic()))
    pose_end, orientation_end, yaw_end = parse_pose(robot)
    result = {
        "status": "PASS",
        "duration_sim_s": sim_time,
        "steps": steps,
        "command_samples": commands[::max(1, len(commands) // 5)],
        "start_position": pose_start.tolist(),
        "end_position": pose_end.tolist(),
        "delta_position": (pose_end - pose_start).tolist(),
        "yaw_delta": yaw_delta(orientation_start, orientation_end),
        "end_yaw": yaw_end,
        "wheel_state": controller.wheel_state(),
        "scan_records": scan_records,
        "wall_duration_s": time.monotonic() - wall_start,
    }
    if ros is not None:
        result["ros_received_count"] = int(ros.received_count)
        result["ros_last_command"] = list(ros.current_command())
    if phase == "lidar":
        result["status"] = "PASS" if scan_records and all(item["beam_count"] == 360 for item in scan_records) else "FAIL"
    if nav is not None:
        result["goal"] = list(goal)
        result["goal_reached"] = nav.reached(float(pose_end[0]), float(pose_end[1]), yaw_end)
        result["navigation_inputs"] = {"odom": True, "tf": True, "laser_scan": bool(scan_records)}
        result["status"] = "PASS" if result["goal_reached"] else "FAIL"
    return result


def main() -> int:
    config = args()
    robot_usd = config.robot_usd.resolve()
    if not EXPERIENCE.is_file():
        raise FileNotFoundError(EXPERIENCE)
    if not robot_usd.is_file():
        raise FileNotFoundError(robot_usd)
    if config.ros is False and config.phase == "ros":
        config.ros = True
    app = world = robot = controller = ros = None
    result: dict = {"status": "FAIL", "phase": config.phase}
    try:
        app, world, robot, controller, ros = make_runtime(config.phase, robot_usd, config.ros)
        print(
            "ISAAC5_BASIC_RUNTIME_READY="
            + json.dumps(
                {
                    "experience": str(EXPERIENCE),
                    "robot_usd": str(robot_usd),
                    "phase": config.phase,
                    "ros": ros is not None,
                    "lidar": config.phase in ("lidar", "navigation"),
                    "pedestrians": False,
                    "arena": False,
                    "renderer": "custom_no_rtx",
                    "wheel_indices": controller.wheel_indices.tolist(),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        if config.phase == "control":
            result = run_control(app, world, robot, controller, ros, config.fast)
        else:
            goal = tuple(config.goal) if config.phase == "navigation" else None
            result = run_stream(
                app,
                world,
                robot,
                controller,
                ros,
                config.duration,
                config.fast,
                config.phase,
                goal,
                tuple(config.test_command) if config.test_command else None,
            )
        result.update({"phase": config.phase, "ros_enabled": ros is not None, "isaac5": True})
        print("ISAAC5_BASIC_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result.update({"status": "FAIL", "error": repr(exc), "phase": config.phase})
        print("ISAAC5_BASIC_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)
        return 1
    finally:
        teardown = {"ros_close": False, "world_stop": False, "app_close": False, "errors": []}
        if ros is not None:
            try:
                ros.close()
                teardown["ros_close"] = True
            except Exception as exc:  # pragma: no cover - Kit runtime dependent
                teardown["errors"].append(f"ros_close: {exc!r}")
        if world is not None:
            try:
                world.stop()
                teardown["world_stop"] = True
            except Exception as exc:  # pragma: no cover
                teardown["errors"].append(f"world_stop: {exc!r}")
        if app is not None:
            try:
                app.close()
                teardown["app_close"] = True
            except Exception as exc:  # pragma: no cover
                teardown["errors"].append(f"app_close: {exc!r}")
        teardown["status"] = "PASS" if not teardown["errors"] and teardown["app_close"] else "FAIL"
        print("ISAAC5_BASIC_TEARDOWN=" + json.dumps(teardown, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
