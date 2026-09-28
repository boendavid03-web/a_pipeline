#!/usr/bin/env python3
"""Stage 7: Isaac5 scan -> DR-SPAAM -> tracker -> existing DRL-VO."""

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
TRACKER_PATH = (
    COMPARISON_ROOT
    / "ros2_ws/src/semantic_nav_runtime/scripts/pedestrian_point_tracker_core.py"
).resolve()
DRSPAAM_ROOT = COMPARISON_ROOT / "third_party/dr_spaam"
DRSPAAM_CHECKPOINT = COMPARISON_ROOT / "checkpoints/dr_spaam/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
DRSPAAM_CONFIDENCE = 0.20


def add_python_paths() -> None:
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
    for path in (
        ROS_ROOT,
        ROS_ROOT / "rclpy",
        DRSPAAM_ROOT,
        DRL_NODE_PATH.parent,
        COMPARISON_ROOT / "runtime_code",
        *package_paths,
    ):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def load_class(path: Path, class_name: str, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return getattr(module, class_name)


def parameter_args() -> list[str]:
    return [
        "stage7_drspaam_probe",
        "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", "mode:=base",
        "-p", f"model:={COMPARISON_ROOT / 'checkpoints/drl_vo/base_bc_best.pt'}",
        "-p", "device:=cpu",
        "-p", "scan_01_topic:=/scan_01",
        "-p", "scan_02_topic:=/scan_02",
        "-p", "odom_topic:=/odom",
        "-p", "local_subgoal_topic:=/semantic_cnn/local_subgoal",
        "-p", "final_goal_topic:=/semantic_cnn/final_goal",
        "-p", "pedestrian_source:=dr_spaam",
        "-p", "pedestrian_tracks_topic:=/pedestrian_tracks",
        "-p", "cmd_vel_topic:=/cmd_vel",
        "-p", "map_frame:=odom",
        "-p", "odom_frame:=odom",
        "-p", "base_frame:=base_link",
        "-p", "pedestrian_frame:=odom",
        "-p", "pedestrian_track_frame:=odom",
        "-p", "enable_fixed_self_filter:=false",
        "-p", "require_full_history:=true",
        "-p", "scan_timeout:=0.8",
        "-p", "odom_timeout:=0.8",
        "-p", "subgoal_timeout:=0.8",
        "-p", "final_goal_timeout:=0.8",
        "-p", "pedestrian_track_timeout:=0.8",
    ]


def to_world(robot_position, robot_yaw, sensor_point_xy):
    tx, ty = 0.20, 0.13
    local = np.asarray([tx, ty], dtype=float) + np.asarray(sensor_point_xy, dtype=float)
    c, s = np.cos(float(robot_yaw)), np.sin(float(robot_yaw))
    return np.asarray(
        [robot_position[0] + c * local[0] - s * local[1], robot_position[1] + s * local[0] + c * local[1]],
        dtype=np.float32,
    )


def main() -> int:
    add_python_paths()
    from isaacsim import SimulationApp

    import rclpy
    from dr_spaam.detector import Detector

    result: dict[str, object] = {"status": "FAIL", "stage": "7"}
    app = world = robot = controller = ros = inference = human_body = None
    try:
        for required in (DRL_NODE_PATH, TRACKER_PATH, DRSPAAM_CHECKPOINT):
            if not required.is_file():
                raise FileNotFoundError(required)

        rclpy.init(args=parameter_args())
        DrlVoFixedDualInference = load_class(
            DRL_NODE_PATH, "DrlVoFixedDualInference", "stage7_existing_drlvo_node"
        )
        PointCVKalmanTracker = load_class(
            TRACKER_PATH, "PointCVKalmanTracker", "stage7_existing_tracker_core"
        )
        PointDetection = load_class(
            TRACKER_PATH, "PointDetection", "stage7_existing_tracker_core_types"
        )
        inference = DrlVoFixedDualInference()

        from geometry_msgs.msg import Point, PointStamped, Vector3
        from navigation_evaluation_msgs.msg import InferenceMetrics
        from semantic_nav_runtime.msg import TrackedPedestrian, TrackedPedestrianArray

        from basic_navigation import ROBOT_USD, make_runtime, parse_pose
        from lidar_sensor import DualPhysxRaycastLidar
        from minimal_human_backend import MinimalHumanBackend, MinimalHumanState
        from ros_bridge import RosControlBridge

        app, world, robot, controller, ros = make_runtime(
            "navigation", ROBOT_USD, use_ros=True, dual_scan=True
        )
        from isaacsim.core.api.objects import DynamicCapsule

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

        human = MinimalHumanState(
            track_id=11,
            position_xy_map=np.asarray([1.5, 0.8], dtype=np.float32),
            velocity_xy_map_absolute=np.zeros(2, dtype=np.float32),
            goal_xy_map=np.asarray([1.5, -0.8], dtype=np.float32),
            speed_mps=0.25,
        )
        human_backend = MinimalHumanBackend([human])
        human_body = world.scene.add(
            DynamicCapsule(
                prim_path="/World/PerceptionHuman/person_11",
                name="perception_human_11",
                position=np.asarray([1.5, 0.8, 0.85]),
                radius=0.24,
                height=1.2,
                color=np.asarray([0.9, 0.35, 0.2]),
                mass=70.0,
            )
        )
        sensor = DualPhysxRaycastLidar(sample_count=2000, rate_hz=10.0)
        detector = Detector(
            str(DRSPAAM_CHECKPOINT),
            model="DR-SPAAM",
            gpu=False,
            stride=1,
            panoramic_scan=True,
        )
        detector.set_laser_fov(360.0)
        tracker = PointCVKalmanTracker(min_hits=1, max_age=8)
        detection_rows: list[dict[str, object]] = []
        track_rows: list[dict[str, object]] = []
        command_rows: list[tuple[float, float, float]] = []
        matched_detection_count = 0
        matched_track_count = 0
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
            position_before, orientation_before, yaw_before = parse_pose(robot)
            human_backend.step(1.0 / 60.0)
            human_body.set_world_pose(
                np.asarray([human.position_xy_map[0], human.position_xy_map[1], 0.85]),
                np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
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

            if step % 6 == 0:
                pair = sensor.sample(sim_time, position, orientation)
                raw_scan = np.asarray(pair.scan_01.ranges, dtype=np.float32)
                raw_scan[~np.isfinite(raw_scan)] = 29.99
                raw_scan = np.minimum(raw_scan, 29.99)
                detections_xy, detection_scores, _ = detector(raw_scan)
                mask = np.asarray(detection_scores).reshape(-1) >= DRSPAAM_CONFIDENCE
                detections_xy = np.asarray(detections_xy)[mask]
                detection_scores = np.asarray(detection_scores).reshape(-1)[mask]
                human_world = human.position_xy_map.astype(float)
                world_detections = [to_world(position, yaw, point) for point in detections_xy]
                if any(float(np.linalg.norm(point - human_world)) <= 0.8 for point in world_detections):
                    matched_detection_count += 1
                detections = [
                    PointDetection(float(point[0]), float(point[1]), float(score))
                    for point, score in zip(detections_xy, detection_scores)
                ]
                estimates = tracker.update(
                    detections,
                    int(round(sim_time * 1_000_000_000)),
                )
                track_message = TrackedPedestrianArray()
                track_message.header.stamp = stamp
                track_message.header.frame_id = "odom"
                for estimate in estimates:
                    world_position = to_world(position, yaw, (estimate.x, estimate.y))
                    world_velocity = to_world(np.zeros(3), yaw, (estimate.vx, estimate.vy))
                    track = TrackedPedestrian()
                    track.track_id = int(estimate.track_id)
                    track.position = Point(x=float(world_position[0]), y=float(world_position[1]), z=0.0)
                    track.velocity = Vector3(x=float(world_velocity[0]), y=float(world_velocity[1]), z=0.0)
                    track.confidence = float(estimate.confidence)
                    track.age = int(estimate.age)
                    track.hits = int(estimate.hits)
                    track.misses = int(estimate.misses)
                    track.state = str(estimate.state)
                    track.time_since_update = float(estimate.time_since_update)
                    track_message.tracks.append(track)
                    if float(np.linalg.norm(world_position - human_world)) <= 0.8:
                        matched_track_count += 1
                tracks_pub.publish(track_message)
                ros.publish_dual_scan(pair)
                detection_rows.append(
                    {
                        "sim_time": sim_time,
                        "raw_detection_count": int(len(detections_xy)),
                        "max_confidence": float(np.max(detection_scores)) if len(detection_scores) else 0.0,
                        "human_position": human.position_xy_map.tolist(),
                    }
                )
                track_rows.append(
                    {
                        "sim_time": sim_time,
                        "track_count": len(estimates),
                        "confirmed_track_ids": [int(item.track_id) for item in estimates if item.state == "CONFIRMED"],
                    }
                )

        for _ in range(20):
            rclpy.spin_once(inference, timeout_sec=0.001)
            ros.spin_once()
        end_position, _, _ = parse_pose(robot)
        successful = [row for row in metrics_rows if row["success"]]
        checks = {
            "dr_spaam_detector_loaded": True,
            "scan_samples": len(detection_rows) >= 5,
            "detector_produced_detections": any(row["raw_detection_count"] > 0 for row in detection_rows),
            "detector_matched_simulated_human": matched_detection_count > 0,
            "tracker_produced_tracks": any(row["track_count"] > 0 for row in track_rows),
            "tracker_matched_simulated_human": matched_track_count > 0,
            "drlvo_inference_received": bool(metrics_rows),
            "drlvo_successful_inference": bool(successful),
            "cmd_vel_received": ros.received_count > 0,
            "finite_commands": bool(command_rows) and all(
                bool(np.isfinite(np.asarray(row, dtype=float)).all()) for row in command_rows
            ),
            "robot_pose_finite": bool(np.isfinite(np.asarray(end_position, dtype=float)).all()),
            "physics_duration": sim_time >= duration_s - 1.0 / 60.0,
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "7",
            "experience": str(BACKEND_ROOT / "generated/minimal_core_physx_no_rtx.kit"),
            "detector": "DR-SPAAM",
            "detector_checkpoint": str(DRSPAAM_CHECKPOINT),
            "detector_confidence_threshold": DRSPAAM_CONFIDENCE,
            "tracker": "PointCVKalmanTracker",
            "sim_time_s": sim_time,
            "wall_time_s": time.monotonic() - start_wall,
            "scan_sample_count": len(detection_rows),
            "matched_detection_count": matched_detection_count,
            "matched_track_count": matched_track_count,
            "detection_samples": detection_rows[:3],
            "track_samples": track_rows[:3],
            "drlvo_inference_count": len(metrics_rows),
            "drlvo_successful_inference_count": len(successful),
            "cmd_vel_count": ros.received_count,
            "start_position": np.asarray(start_position).tolist(),
            "end_position": np.asarray(end_position).tolist(),
            "checks": checks,
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "7", "error": repr(exc)}
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
        print("ISAAC5_STAGE7_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
