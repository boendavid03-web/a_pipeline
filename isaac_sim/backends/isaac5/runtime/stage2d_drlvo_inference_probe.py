#!/usr/bin/env python3
"""Stage 2-D: run the existing DRL-VO node against Isaac5 producers."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
BACKEND_ROOT = HERE.parents[0]
WORKSPACE_ROOT = HERE.parents[3]
COMPARISON_ROOT = WORKSPACE_ROOT / "sim_to_real/robot/comparison_models"
ISAAC_ROOT = Path(os.environ.get("ISAAC_SIM_5_ROOT", "/home/user/isaacsim/5.1.0"))
ROS_ROOT = ISAAC_ROOT / "exts/isaacsim.ros2.bridge/humble"
WS_INSTALL = Path(
    os.environ.get(
        "ISAAC5_ROS_CP311_OVERLAY_INSTALL",
        str(COMPARISON_ROOT / "ros2_ws/install"),
    )
)
DRL_NODE_PATH = (
    COMPARISON_ROOT
    / "ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py"
).resolve()
CHECKPOINT = COMPARISON_ROOT / "checkpoints/drl_vo/base_bc_best.pt"
RUNTIME_CODE = COMPARISON_ROOT / "runtime_code"


def add_ros_python_paths() -> None:
    if (WS_INSTALL / "semantic_nav_runtime/lib/python3.11/site-packages").is_dir():
        package_paths = (
            WS_INSTALL / "semantic_nav_runtime/lib/python3.11/site-packages",
            WS_INSTALL / "navigation_evaluation_msgs/lib/python3.11/site-packages",
        )
    else:
        package_paths = (
            WS_INSTALL / "semantic_nav_runtime/local/lib/python3.10/dist-packages",
            WS_INSTALL / "navigation_evaluation_msgs/local/lib/python3.10/dist-packages",
        )
    paths = (
        ROS_ROOT,
        ROS_ROOT / "rclpy",
        *package_paths,
        RUNTIME_CODE,
        DRL_NODE_PATH.parent,
    )
    for path in paths:
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def load_drlvo_node_class():
    if not DRL_NODE_PATH.is_file():
        raise FileNotFoundError(DRL_NODE_PATH)
    spec = importlib.util.spec_from_file_location("stage2d_existing_drlvo_node", DRL_NODE_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(DRL_NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.DrlVoFixedDualInference


def parameter_args() -> list[str]:
    return [
        "stage2d_drlvo_probe",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        "mode:=base",
        "-p",
        f"model:={CHECKPOINT}",
        "-p",
        "device:=cpu",
        "-p",
        "scan_01_topic:=/scan_01",
        "-p",
        "scan_02_topic:=/scan_02",
        "-p",
        "odom_topic:=/odom",
        "-p",
        "local_subgoal_topic:=/semantic_cnn/local_subgoal",
        "-p",
        "final_goal_topic:=/semantic_cnn/final_goal",
        "-p",
        # The existing hardware node currently accepts only the dr_spaam
        # source label, while consuming the same TrackedPedestrianArray
        # contract. This probe supplies the Stage 2-C deterministic track
        # producer; it does not claim DR-SPAAM perception was executed.
        "pedestrian_source:=dr_spaam",
        "-p",
        "pedestrian_tracks_topic:=/pedestrian_tracks",
        "-p",
        "cmd_vel_topic:=/cmd_vel",
        "-p",
        "map_frame:=odom",
        "-p",
        "odom_frame:=odom",
        "-p",
        "base_frame:=base_link",
        "-p",
        "pedestrian_frame:=odom",
        "-p",
        "pedestrian_track_frame:=odom",
        "-p",
        "enable_fixed_self_filter:=false",
        "-p",
        "require_full_history:=true",
        "-p",
        "scan_timeout:=0.8",
        "-p",
        "odom_timeout:=0.8",
        "-p",
        "subgoal_timeout:=0.8",
        "-p",
        "final_goal_timeout:=0.8",
        "-p",
        "pedestrian_track_timeout:=0.8",
    ]


def main() -> int:
    add_ros_python_paths()
    from isaacsim import SimulationApp

    import rclpy

    result: dict[str, object] = {"status": "FAIL", "stage": "2-D"}
    app = world = robot = controller = ros = inference = None
    try:
        if not CHECKPOINT.is_file():
            raise FileNotFoundError(CHECKPOINT)
        rclpy.init(args=parameter_args())
        DrlVoFixedDualInference = load_drlvo_node_class()
        inference = DrlVoFixedDualInference()

        from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3
        from semantic_nav_runtime.msg import TrackedPedestrian, TrackedPedestrianArray
        from std_msgs.msg import Header
        from navigation_evaluation_msgs.msg import InferenceMetrics
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data

        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from lidar_sensor import DualPhysxRaycastLidar
        from minimal_human_backend import MinimalHumanBackend, MinimalHumanState
        from ros_bridge import RosControlBridge

        app, world, robot, controller, ros = make_runtime(
            "lidar", ROBOT_USD, use_ros=True, dual_scan=True
        )
        # make_runtime creates a second node in the already initialized ROS
        # context; its bridge close path does not own global shutdown here.
        goal_local_pub = ros.node.create_publisher(
            PointStamped, "/semantic_cnn/local_subgoal", 10
        )
        goal_final_pub = ros.node.create_publisher(
            PointStamped, "/semantic_cnn/final_goal", 10
        )
        tracks_pub = ros.node.create_publisher(
            TrackedPedestrianArray, "/pedestrian_tracks", 20
        )
        metrics_rows: list[dict[str, object]] = []
        cmd_rows: list[tuple[float, float, float]] = []

        def on_metrics(message: InferenceMetrics) -> None:
            metrics_rows.append(
                {
                    "sequence_id": int(message.sequence_id),
                    "success": bool(message.success),
                    "policy_ms": float(message.policy_ms),
                    "total_ms": float(message.total_ms),
                    "action": list(message.action),
                }
            )

        ros.node.create_subscription(
            InferenceMetrics,
            "/navigation_evaluation/inference_metrics",
            on_metrics,
            30,
        )
        sensor = DualPhysxRaycastLidar(sample_count=2000, rate_hz=10.0)
        human_backend = MinimalHumanBackend(
            [
                MinimalHumanState(
                    track_id=1,
                    position_xy_map=np.asarray([2.0, 0.0], dtype=np.float32),
                    velocity_xy_map_absolute=np.zeros(2, dtype=np.float32),
                    goal_xy_map=np.asarray([2.0, 1.2], dtype=np.float32),
                    speed_mps=0.6,
                )
            ]
        )
        sim_time = 0.0
        start_wall = time.monotonic()
        start_position, _, _ = parse_pose(robot)
        duration_s = 6.0
        steps = int(round(duration_s / (1.0 / 60.0)))
        for step in range(steps):
            # Dispatch callbacks generated by the previous physics tick.
            rclpy.spin_once(inference, timeout_sec=0.001)
            ros.spin_once()
            command = ros.current_command()
            cmd_rows.append(command)
            controller.apply(*command)
            human_backend.step(1.0 / 60.0)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            position, orientation, yaw = parse_pose(robot)
            ros.publish_clock(sim_time)
            ros.publish_state(sim_time, position, orientation, command)

            stamp = ros.stamp(sim_time)
            local = PointStamped()
            local.header.stamp = stamp
            local.header.frame_id = "base_link"
            local.point.x = 0.5 - float(position[0])
            local.point.y = -float(position[1])
            final = PointStamped()
            final.header.stamp = stamp
            final.header.frame_id = "odom"
            final.point.x = 0.5
            final.point.y = 0.0
            goal_local_pub.publish(local)
            goal_final_pub.publish(final)

            track_message = TrackedPedestrianArray()
            track_message.header.stamp = stamp
            track_message.header.frame_id = "odom"
            track = TrackedPedestrian()
            track.track_id = human_backend.humans[0].track_id
            track.position = Point(
                x=float(human_backend.humans[0].position_xy_map[0]),
                y=float(human_backend.humans[0].position_xy_map[1]),
                z=0.0,
            )
            track.velocity = Vector3(
                x=float(human_backend.humans[0].velocity_xy_map_absolute[0]),
                y=float(human_backend.humans[0].velocity_xy_map_absolute[1]),
                z=0.0,
            )
            track.confidence = 1.0
            track.age = 1
            track.hits = 1
            track.misses = 0
            track.state = "CONFIRMED"
            track.time_since_update = 0.0
            track_message.tracks = [track]
            tracks_pub.publish(track_message)

            if step % 6 == 0:
                pair = sensor.sample(sim_time, position, orientation)
                ros.publish_dual_scan(pair)
            if not bool(os.environ.get("ISAAC5_STAGE2D_FAST")):
                time.sleep(max(0.0, (1.0 / 60.0) - 0.001))

        for _ in range(10):
            rclpy.spin_once(inference, timeout_sec=0.001)
            ros.spin_once()
        end_position, _, _ = parse_pose(robot)
        wall_time = time.monotonic() - start_wall
        success_rows = [row for row in metrics_rows if row["success"]]
        checks = {
            "checkpoint_loaded": True,
            "inference_metrics_received": bool(metrics_rows),
            "successful_inferences": bool(success_rows),
            "cmd_vel_received": ros.received_count > 0,
            "finite_commands": bool(cmd_rows)
            and all(np.isfinite(np.asarray(row, dtype=float)).all() for row in cmd_rows),
            "robot_pose_finite": bool(
                np.isfinite(np.asarray(end_position, dtype=float)).all()
            ),
            "physics_duration": sim_time >= duration_s - 1.0 / 60.0,
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "2-D",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "checkpoint": str(CHECKPOINT),
            "device": "cpu",
            "sim_time_s": sim_time,
            "wall_time_s": wall_time,
            "inference_count": len(metrics_rows),
            "successful_inference_count": len(success_rows),
            "command_count": len(cmd_rows),
            "ros_received_cmd_count": ros.received_count,
            "start_position": np.asarray(start_position).tolist(),
            "end_position": np.asarray(end_position).tolist(),
            "delta_position": (np.asarray(end_position) - np.asarray(start_position)).tolist(),
            "latency_ms": {
                "min": min((float(row["total_ms"]) for row in success_rows), default=None),
                "max": max((float(row["total_ms"]) for row in success_rows), default=None),
                "mean": (
                    float(np.mean([float(row["total_ms"]) for row in success_rows]))
                    if success_rows
                    else None
                ),
            },
            "checks": checks,
            "metrics_sample": metrics_rows[:3],
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "2-D", "error": repr(exc)}
        return 1
    finally:
        teardown = {"inference_destroy": False, "ros_close": False, "world_stop": False, "app_close": False, "ros_shutdown": False, "errors": []}
        if inference is not None:
            try:
                inference.destroy_node()
                teardown["inference_destroy"] = True
            except Exception as exc:
                teardown["errors"].append(f"inference_destroy: {exc!r}")
        if ros is not None:
            try:
                ros.close()
                teardown["ros_close"] = True
            except Exception as exc:
                teardown["errors"].append(f"ros_close: {exc!r}")
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
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
            teardown["ros_shutdown"] = True
        except Exception as exc:
            teardown["errors"].append(f"ros_shutdown: {exc!r}")
        teardown["status"] = (
            "PASS"
            if not teardown["errors"]
            and teardown["inference_destroy"]
            and teardown["app_close"]
            and teardown["ros_shutdown"]
            else "FAIL"
        )
        result["teardown"] = teardown
        print("ISAAC5_STAGE2D_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
