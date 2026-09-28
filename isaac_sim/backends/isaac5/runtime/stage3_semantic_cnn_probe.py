#!/usr/bin/env python3
"""Stage 3: run the existing SemanticCNN node against Isaac5 producers.

This probe keeps the trained SemanticCNN model and node unchanged.  Isaac5
provides the dual non-RTX scene-query scans, odometry, static TF, and a local
goal.  Semantic labels come from an existing exported static label map; no
S3-Net, pedestrian, Arena, or Isaac6 runtime is started.
"""

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
SEMANTIC_NODE_PATH = (
    COMPARISON_ROOT
    / "ros2_ws/src/semantic_nav_runtime/scripts/semantic_cnn_fixed_dual_inference_node.py"
).resolve()
MODEL = COMPARISON_ROOT / "checkpoints/semantic_cnn/semantic_cnn_native_cmd_best_dev.pth"
MODEL_CODE = COMPARISON_ROOT / "checkpoints/semantic_cnn"
MAP_YAML = WORKSPACE_ROOT / "runs/20260808_migration_export_smoke/maps/semantic_label/map.yaml"
SEMANTIC_LABEL = WORKSPACE_ROOT / "runs/20260808_migration_export_smoke/maps/semantic_label/label.png"


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
    for path in (ROS_ROOT, ROS_ROOT / "rclpy", *package_paths):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def load_semantic_node_class():
    if not SEMANTIC_NODE_PATH.is_file():
        raise FileNotFoundError(SEMANTIC_NODE_PATH)
    spec = importlib.util.spec_from_file_location(
        "stage3_existing_semantic_cnn_node", SEMANTIC_NODE_PATH
    )
    if spec is None or spec.loader is None:
        raise ImportError(SEMANTIC_NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.FixedDualSemanticCnnInference


def parameter_args() -> list[str]:
    return [
        "stage3_semantic_cnn_probe",
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        f"model:={MODEL}",
        "-p",
        f"model_code:={MODEL_CODE}",
        "-p",
        f"map_yaml:={MAP_YAML}",
        "-p",
        f"semantic_label:={SEMANTIC_LABEL}",
        "-p",
        "device:=cpu",
        "-p",
        "scan_01_topic:=/scan_01",
        "-p",
        "scan_02_topic:=/scan_02",
        "-p",
        "use_online_s3net:=false",
        "-p",
        "odom_topic:=/odom",
        "-p",
        "local_subgoal_topic:=/semantic_cnn/local_subgoal",
        "-p",
        "final_goal_topic:=/semantic_cnn/final_goal",
        "-p",
        "cmd_vel_topic:=/cmd_vel",
        "-p",
        "base_frame:=base_link",
        "-p",
        "map_frame:=odom",
        "-p",
        "visualize:=false",
        "-p",
        "publish_debug_images:=false",
        "-p",
        "enable_self_filter:=false",
        "-p",
        "scan_timeout:=0.8",
        "-p",
        "odom_timeout:=0.8",
        "-p",
        "subgoal_timeout:=0.8",
        "-p",
        "command_timeout:=0.8",
    ]


def main() -> int:
    add_ros_python_paths()
    from isaacsim import SimulationApp

    import rclpy

    result: dict[str, object] = {"status": "FAIL", "stage": "3"}
    app = world = robot = controller = ros = inference = None
    try:
        for required in (MODEL, MODEL_CODE / "model.py", MAP_YAML, SEMANTIC_LABEL):
            if not required.exists():
                raise FileNotFoundError(required)

        rclpy.init(args=parameter_args())
        FixedDualSemanticCnnInference = load_semantic_node_class()
        inference = FixedDualSemanticCnnInference()

        from geometry_msgs.msg import PointStamped
        from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics

        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from lidar_sensor import DualPhysxRaycastLidar
        from ros_bridge import RosControlBridge

        app, world, robot, controller, ros = make_runtime(
            "lidar", ROBOT_USD, use_ros=True, dual_scan=True
        )
        local_goal_pub = ros.node.create_publisher(
            PointStamped, "/semantic_cnn/local_subgoal", 10
        )
        final_goal_pub = ros.node.create_publisher(
            PointStamped, "/semantic_cnn/final_goal", 10
        )
        metrics_rows: list[dict[str, object]] = []
        decision_rows: list[dict[str, object]] = []
        command_rows: list[tuple[float, float, float]] = []

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

        def on_decision(message: ActuationDecision) -> None:
            decision_rows.append(
                {
                    "inference_sequence_id": int(message.inference_sequence_id),
                    "gated": bool(message.gated),
                    "final_linear_x": float(message.final_command.linear.x),
                    "final_angular_z": float(message.final_command.angular.z),
                }
            )

        ros.node.create_subscription(
            InferenceMetrics,
            "/navigation_evaluation/inference_metrics",
            on_metrics,
            30,
        )
        ros.node.create_subscription(
            ActuationDecision,
            "/semantic_cnn/actuation_decision",
            on_decision,
            30,
        )

        sensor = DualPhysxRaycastLidar(sample_count=2000, rate_hz=10.0)
        goal_x, goal_y = 0.5, 0.0
        sim_time = 0.0
        start_wall = time.monotonic()
        start_position, _, _ = parse_pose(robot)
        duration_s = 6.0
        steps = int(round(duration_s / (1.0 / 60.0)))

        for step in range(steps):
            rclpy.spin_once(inference, timeout_sec=0.001)
            ros.spin_once()
            command = ros.current_command()
            command_rows.append(command)
            controller.apply(*command)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            position, orientation, _ = parse_pose(robot)
            ros.publish_clock(sim_time)
            ros.publish_state(sim_time, position, orientation, command)

            stamp = ros.stamp(sim_time)
            local_goal = PointStamped()
            local_goal.header.stamp = stamp
            local_goal.header.frame_id = "base_link"
            local_goal.point.x = goal_x - float(position[0])
            local_goal.point.y = goal_y - float(position[1])
            final_goal = PointStamped()
            final_goal.header.stamp = stamp
            final_goal.header.frame_id = "odom"
            final_goal.point.x = goal_x
            final_goal.point.y = goal_y
            local_goal_pub.publish(local_goal)
            final_goal_pub.publish(final_goal)

            if step % 6 == 0:
                ros.publish_dual_scan(sensor.sample(sim_time, position, orientation))

        for _ in range(20):
            rclpy.spin_once(inference, timeout_sec=0.001)
            ros.spin_once()

        end_position, _, _ = parse_pose(robot)
        wall_time = time.monotonic() - start_wall
        successful = [row for row in metrics_rows if row["success"]]
        goal_distance = float(
            np.linalg.norm(
                np.asarray(end_position, dtype=float)[:2]
                - np.asarray([goal_x, goal_y], dtype=float)
            )
        )
        goal_reached = goal_distance <= 0.35
        finite_commands = bool(command_rows) and all(
            np.isfinite(np.asarray(row, dtype=float)).all() for row in command_rows
        )
        checks = {
            "checkpoint_loaded": True,
            "static_semantic_map_loaded": True,
            "inference_metrics_received": bool(metrics_rows),
            "successful_inferences": bool(successful),
            "actuation_decisions_received": bool(decision_rows),
            "cmd_vel_received": ros.received_count > 0,
            "finite_commands": finite_commands,
            "robot_moved": float(np.linalg.norm(np.asarray(end_position) - np.asarray(start_position))) > 1e-3,
            "goal_reached_within_node_tolerance": goal_reached,
            "robot_pose_finite": bool(np.isfinite(np.asarray(end_position, dtype=float)).all()),
            "physics_duration": sim_time >= duration_s - 1.0 / 60.0,
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "3",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "model": str(MODEL),
            "device": "cpu",
            "semantic_mode": "static_map",
            "sim_time_s": sim_time,
            "wall_time_s": wall_time,
            "inference_count": len(metrics_rows),
            "successful_inference_count": len(successful),
            "actuation_decision_count": len(decision_rows),
            "ros_received_cmd_count": ros.received_count,
            "start_position": np.asarray(start_position).tolist(),
            "end_position": np.asarray(end_position).tolist(),
            "delta_position": (np.asarray(end_position) - np.asarray(start_position)).tolist(),
            "goal": [goal_x, goal_y],
            "goal_distance_m": goal_distance,
            "latency_ms": {
                "min": min((float(row["total_ms"]) for row in successful), default=None),
                "max": max((float(row["total_ms"]) for row in successful), default=None),
                "mean": float(np.mean([float(row["total_ms"]) for row in successful])) if successful else None,
            },
            "checks": checks,
            "metrics_sample": metrics_rows[:3],
            "decision_sample": decision_rows[:3],
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "3", "error": repr(exc)}
        return 1
    finally:
        teardown = {
            "inference_destroy": False,
            "ros_close": False,
            "world_stop": False,
            "app_close": False,
            "ros_shutdown": False,
            "errors": [],
        }
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
        print("ISAAC5_STAGE3_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
