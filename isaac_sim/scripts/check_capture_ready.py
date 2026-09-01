#!/usr/bin/env python3
"""Check all live Isaac rosbag endpoints and fresh samples in one DDS context."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from navigation_evaluation_msgs.msg import InferenceMetrics
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from semantic_nav_gazebo.msg import PedestrianStateArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


EXPECTED_TYPES = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/scan_01": "sensor_msgs/msg/LaserScan",
    "/scan_02": "sensor_msgs/msg/LaserScan",
    "/scan_merged": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_stamped": "geometry_msgs/msg/TwistStamped",
    "/pedestrian_ground_truth": "semantic_nav_gazebo/msg/PedestrianStateArray",
    "/data_collection/episode_event": "std_msgs/msg/String",
    "/data_collection/sensor_config": "std_msgs/msg/String",
}
SENSOR_PREFLIGHT_TOPICS = {
    "/scan_01",
    "/scan_02",
    "/scan_merged",
    "/odom",
    "/tf",
    "/tf_static",
    "/clock",
    "/pedestrian_ground_truth",
    "/data_collection/sensor_config",
}
NAVIGATION_TYPES = {
    "/data_collection/goal_accepted": "geometry_msgs/msg/PointStamped",
    "/semantic_cnn/global_path": "nav_msgs/msg/Path",
    "/semantic_cnn/local_subgoal": "geometry_msgs/msg/PointStamped",
    "/semantic_cnn/final_goal": "geometry_msgs/msg/PointStamped",
    "/drl_vo/raw_model_cmd": "geometry_msgs/msg/Twist",
    "/navigation_evaluation/inference_metrics": (
        "navigation_evaluation_msgs/msg/InferenceMetrics"
    ),
}
NAVIGATION_SAMPLE_TYPES = {
    "/data_collection/goal_accepted": PointStamped,
    "/semantic_cnn/global_path": Path,
    "/semantic_cnn/local_subgoal": PointStamped,
    "/semantic_cnn/final_goal": PointStamped,
    "/drl_vo/raw_model_cmd": Twist,
    "/navigation_evaluation/inference_metrics": InferenceMetrics,
}
SAMPLE_TYPES = {
    "/clock": Clock,
    "/scan_01": LaserScan,
    "/scan_02": LaserScan,
    "/scan_merged": LaserScan,
    "/odom": Odometry,
    "/pedestrian_ground_truth": PedestrianStateArray,
    "/data_collection/sensor_config": String,
}


def stamp_ns(message: Clock) -> int:
    return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)


def header_stamp_ns(message: LaserScan) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class CaptureReadiness(Node):
    def __init__(
        self,
        require_lidar_intensity: bool = False,
        verify_lidar_rate: bool = False,
        require_realtime_lidar: bool = False,
        verify_motion: bool = False,
        minimum_motion_distance: float = 0.05,
        sensor_preflight: bool = False,
        verify_navigation: bool = False,
    ) -> None:
        super().__init__("isaac_6_capture_readiness")
        self.require_lidar_intensity = require_lidar_intensity
        self.verify_lidar_rate = verify_lidar_rate
        self.require_realtime_lidar = require_realtime_lidar
        self.verify_motion = verify_motion
        self.minimum_motion_distance = minimum_motion_distance
        self.sensor_preflight = sensor_preflight
        self.verify_navigation = verify_navigation
        self.moving_command_received = False
        self.first_odom_xy: tuple[float, float] | None = None
        self.latest_odom_xy: tuple[float, float] | None = None
        self.navigation_received = {
            topic: False for topic in NAVIGATION_SAMPLE_TYPES
        }
        self.global_path_pose_count = 0
        self.successful_inference_received = False
        self.received = {topic: False for topic in SAMPLE_TYPES}
        self.intensity_ready = {
            topic: False for topic in ("/scan_01", "/scan_02", "/scan_merged")
        }
        self.first_clock_ns: int | None = None
        self.last_clock_ns: int | None = None
        self.sensor_config_ready = False
        self.requested_lidar_rate_hz: int | None = None
        self.requested_lidar_samples: int | None = None
        self.scan_shapes: dict[str, tuple[int, int] | None] = {
            "/scan_01": None,
            "/scan_02": None,
            "/scan_merged": None,
        }
        self.scan_stamps_ns: dict[str, list[int]] = {
            "/scan_01": [],
            "/scan_02": [],
        }
        self.scan_wall_times: dict[str, list[float]] = {
            "/scan_01": [],
            "/scan_02": [],
        }
        self._sample_subscriptions = []
        for topic, message_type in SAMPLE_TYPES.items():
            if topic.startswith("/scan"):
                qos = qos_profile_sensor_data
            elif topic == "/data_collection/sensor_config":
                qos = QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
            else:
                qos = 10
            self._sample_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, name=topic: self.on_message(name, message),
                    qos,
                )
            )
        self._command_subscription = None
        if not self.sensor_preflight:
            self._command_subscription = self.create_subscription(
                Twist,
                "/cmd_vel",
                self.on_command,
                10,
            )
        self._navigation_subscriptions = []
        if self.verify_navigation:
            for topic, message_type in NAVIGATION_SAMPLE_TYPES.items():
                qos = (
                    QoSProfile(
                        depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    )
                    if topic == "/data_collection/goal_accepted"
                    else 10
                )
                self._navigation_subscriptions.append(
                    self.create_subscription(
                        message_type,
                        topic,
                        lambda message, name=topic: self.on_navigation_message(
                            name, message
                        ),
                        qos,
                    )
                )

    def on_message(self, topic: str, message) -> None:
        self.received[topic] = True
        if topic in self.intensity_ready:
            self.scan_shapes[topic] = (
                len(message.ranges),
                len(message.intensities),
            )
            self.intensity_ready[topic] = bool(
                len(message.intensities) == len(message.ranges)
                and any(float(value) > 0.0 for value in message.intensities)
            )
        if topic in self.scan_stamps_ns:
            value = header_stamp_ns(message)
            stamps = self.scan_stamps_ns[topic]
            if not stamps or value > stamps[-1]:
                stamps.append(value)
                self.scan_wall_times[topic].append(time.monotonic())
                if len(stamps) > 256:
                    del stamps[:-256]
                    del self.scan_wall_times[topic][:-256]
        if topic == "/odom":
            xy = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
            if self.first_odom_xy is None:
                self.first_odom_xy = xy
            self.latest_odom_xy = xy
        if topic == "/clock":
            value = stamp_ns(message)
            if self.first_clock_ns is None:
                self.first_clock_ns = value
            self.last_clock_ns = value
        elif topic == "/data_collection/sensor_config":
            try:
                config = json.loads(message.data)
                rate_hz = int(config["lidar_rate_hz"])
                samples = int(config["lidar_samples"])
                mode = str(config["lidar_mode"])
                expected_pairing_timestamp_domain = {
                    "rtx": "isaac_rtx_gmo_timestamp_ns",
                    "physx": "simulation_manager_physics_time",
                }.get(mode)
                self.sensor_config_ready = bool(
                    config.get("schema") == "isaac_sensor_config/v1"
                    and config.get("lidar_rate_basis") == "simulation_time"
                    and config.get("lidar_timestamp_domain")
                    == "simulation_manager_physics_time"
                    and config.get("lidar_pairing_timestamp_domain")
                    == expected_pairing_timestamp_domain
                    and 1 <= rate_hz <= 30
                    and 90 <= samples <= 4096
                )
                self.requested_lidar_rate_hz = (
                    rate_hz if self.sensor_config_ready else None
                )
                self.requested_lidar_samples = (
                    samples if self.sensor_config_ready else None
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                self.sensor_config_ready = False
                self.requested_lidar_rate_hz = None
                self.requested_lidar_samples = None

    def on_command(self, message: Twist) -> None:
        if (
            abs(float(message.linear.x)) > 1.0e-6
            or abs(float(message.linear.y)) > 1.0e-6
            or abs(float(message.angular.z)) > 1.0e-6
        ):
            self.moving_command_received = True

    def on_navigation_message(self, topic: str, message) -> None:
        self.navigation_received[topic] = True
        if topic == "/semantic_cnn/global_path":
            self.global_path_pose_count = max(
                self.global_path_pose_count, len(message.poses)
            )
        elif topic == "/navigation_evaluation/inference_metrics":
            self.successful_inference_received |= bool(message.success)

    def lidar_sample_count_ready(self) -> bool:
        target = self.requested_lidar_samples
        return bool(
            target is not None
            and all(
                shape is not None
                and shape[0] == target
                and shape[1] in (0, target)
                for shape in self.scan_shapes.values()
            )
        )

    def odom_motion_distance(self) -> float:
        if self.first_odom_xy is None or self.latest_odom_xy is None:
            return 0.0
        return math.hypot(
            self.latest_odom_xy[0] - self.first_odom_xy[0],
            self.latest_odom_xy[1] - self.first_odom_xy[1],
        )

    def motion_ready(self) -> bool:
        if not self.verify_motion:
            return True
        return bool(
            self.moving_command_received
            and self.odom_motion_distance() >= self.minimum_motion_distance
        )

    def navigation_ready(self) -> bool:
        if not self.verify_navigation:
            return True
        return bool(
            all(self.navigation_received.values())
            and self.global_path_pose_count >= 2
            and self.successful_inference_received
        )

    def graph_errors(self) -> tuple[list[str], dict[str, int]]:
        discovered = dict(self.get_topic_names_and_types())
        errors = []
        publisher_counts = {}
        for topic, expected_type in EXPECTED_TYPES.items():
            if self.sensor_preflight and topic not in SENSOR_PREFLIGHT_TOPICS:
                continue
            actual_types = discovered.get(topic, [])
            if actual_types != [expected_type]:
                errors.append(
                    f"{topic} types={actual_types!r}, expected [{expected_type!r}]"
                )
            count = len(self.get_publishers_info_by_topic(topic))
            publisher_counts[topic] = count
            if count <= 0:
                errors.append(f"{topic} has no publisher")
        if self.verify_navigation:
            for topic, expected_type in NAVIGATION_TYPES.items():
                actual_types = discovered.get(topic, [])
                if actual_types != [expected_type]:
                    errors.append(
                        f"{topic} types={actual_types!r}, "
                        f"expected [{expected_type!r}]"
                    )
                count = len(self.get_publishers_info_by_topic(topic))
                publisher_counts[topic] = count
                if count <= 0:
                    errors.append(f"{topic} has no publisher")
        if not self.sensor_preflight and publisher_counts.get("/cmd_vel") != 1:
            errors.append(
                "/cmd_vel must have exactly one publisher (teleop or autonomous controller), "
                f"found {publisher_counts.get('/cmd_vel', 0)}"
            )
        return errors, publisher_counts

    def clock_advanced(self) -> bool:
        return bool(
            self.first_clock_ns is not None
            and self.last_clock_ns is not None
            and self.last_clock_ns - self.first_clock_ns >= 200_000_000
        )

    def lidar_rate_results(self) -> dict[str, float]:
        results = {}
        for topic, stamps in self.scan_stamps_ns.items():
            if len(stamps) < 2:
                continue
            span_sec = (stamps[-1] - stamps[0]) / 1.0e9
            if span_sec > 0.0:
                results[topic] = (len(stamps) - 1) / span_sec
        return results

    def lidar_rate_ready(self) -> bool:
        if not self.verify_lidar_rate:
            return True
        target = self.requested_lidar_rate_hz
        if target is None:
            return False
        results = self.lidar_rate_results()
        if set(results) != set(self.scan_stamps_ns):
            return False
        for topic, stamps in self.scan_stamps_ns.items():
            span_sec = (stamps[-1] - stamps[0]) / 1.0e9
            if span_sec < 1.0:
                return False
            if abs(results[topic] - target) > max(0.25, target * 0.15):
                return False
        return True

    def lidar_wall_rate_results(self) -> dict[str, float]:
        results = {}
        for topic, values in self.scan_wall_times.items():
            if len(values) < 2:
                continue
            span_sec = values[-1] - values[0]
            if span_sec > 0.0:
                results[topic] = (len(values) - 1) / span_sec
        return results

    def lidar_wall_rate_ready(self) -> bool:
        if not self.require_realtime_lidar:
            return True
        target = self.requested_lidar_rate_hz
        if target is None:
            return False
        results = self.lidar_wall_rate_results()
        if set(results) != set(self.scan_wall_times):
            return False
        for topic, observed in results.items():
            values = self.scan_wall_times[topic]
            if values[-1] - values[0] < 1.0:
                return False
            if abs(observed - target) > max(0.25, target * 0.15):
                return False
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--sensor-preflight",
        action="store_true",
        help=(
            "Check only fresh simulator sensor/config topics. This is used "
            "before a teleop or autonomous /cmd_vel publisher is started."
        ),
    )
    parser.add_argument(
        "--require-lidar-intensity",
        action="store_true",
        help="Require aligned, non-zero intensity samples on both raw scans and /scan_merged.",
    )
    parser.add_argument(
        "--verify-navigation",
        action="store_true",
        help=(
            "Require an accepted goal, a non-trivial global path, local/final "
            "goals, a raw DRL-VO action, and successful inference telemetry."
        ),
    )
    parser.add_argument(
        "--verify-motion",
        action="store_true",
        help=(
            "Require a non-zero /cmd_vel command and at least the requested "
            "odometry displacement during this check."
        ),
    )
    parser.add_argument(
        "--minimum-motion-distance",
        type=float,
        default=0.05,
        help="Minimum odometry displacement in metres for --verify-motion.",
    )
    parser.add_argument(
        "--verify-lidar-rate",
        action="store_true",
        help="Measure both raw scan rates for at least one simulation second.",
    )
    parser.add_argument(
        "--require-realtime-lidar",
        action="store_true",
        help=(
            "Also require wall-clock scan delivery to match the configured "
            "rate; simulation-time rate remains the primary contract."
        ),
    )
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.minimum_motion_distance <= 0.0:
        parser.error("--minimum-motion-distance must be positive")
    if args.sensor_preflight and args.verify_motion:
        parser.error("--sensor-preflight cannot be combined with --verify-motion")
    if args.sensor_preflight and args.verify_navigation:
        parser.error("--sensor-preflight cannot be combined with --verify-navigation")

    rclpy.init()
    node = CaptureReadiness(
        args.require_lidar_intensity,
        args.verify_lidar_rate,
        args.require_realtime_lidar,
        args.verify_motion,
        args.minimum_motion_distance,
        args.sensor_preflight,
        args.verify_navigation,
    )
    deadline = time.monotonic() + args.timeout
    errors: list[str] = ["ROS graph not inspected"]
    publisher_counts: dict[str, int] = {}
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            errors, publisher_counts = node.graph_errors()
            intensity_ready = (
                all(node.intensity_ready.values())
                if node.require_lidar_intensity
                else True
            )
            if (
                not errors
                and all(node.received.values())
                and node.clock_advanced()
                and node.sensor_config_ready
                and node.lidar_sample_count_ready()
                and node.lidar_rate_ready()
                and node.lidar_wall_rate_ready()
                and intensity_ready
                and node.motion_ready()
                and node.navigation_ready()
            ):
                checked_types = (
                    {
                        topic: expected_type
                        for topic, expected_type in EXPECTED_TYPES.items()
                        if topic in SENSOR_PREFLIGHT_TOPICS
                    }
                    if node.sensor_preflight
                    else EXPECTED_TYPES
                )
                for topic, expected_type in checked_types.items():
                    print(
                        f"PASS {topic} {expected_type} "
                        f"publishers={publisher_counts[topic]}"
                    )
                if node.verify_navigation:
                    for topic, expected_type in NAVIGATION_TYPES.items():
                        print(
                            f"PASS {topic} {expected_type} "
                            f"publishers={publisher_counts[topic]}"
                        )
                for topic, received in node.received.items():
                    print(f"fresh_sample {topic}: {received}")
                for topic, ready in node.intensity_ready.items():
                    print(f"aligned_nonzero_intensity {topic}: {ready}")
                print("clock_advanced_0.2s: True")
                print("sensor_config_valid: True")
                print(
                    "lidar_sample_count_valid: True "
                    f"samples={node.requested_lidar_samples}"
                )
                for topic, observed in node.lidar_rate_results().items():
                    wall_observed = node.lidar_wall_rate_results().get(topic)
                    print(
                        f"lidar_rate {topic}: requested="
                        f"{node.requested_lidar_rate_hz}Hz "
                        f"sim_observed={observed:.3f}Hz "
                        f"wall_observed="
                        + (
                            f"{wall_observed:.3f}Hz"
                            if wall_observed is not None
                            else "unavailable"
                        )
                    )
                print(
                    "realtime_lidar_required: "
                    f"{node.require_realtime_lidar}"
                )
                print(f"moving_command_received: {node.moving_command_received}")
                print(
                    "odom_motion_distance_m: "
                    f"{node.odom_motion_distance():.3f}"
                )
                if node.verify_navigation:
                    print(
                        "navigation_contract_valid: True "
                        f"global_path_poses={node.global_path_pose_count} "
                        "successful_inference=True"
                    )
                if node.sensor_preflight:
                    print("ISAAC_SENSOR_PREFLIGHT=PASS")
                else:
                    print("ISAAC_CAPTURE_READY=PASS")
                return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    for error in errors:
        print(f"ERROR: {error}")
    for topic, received in node.received.items():
        if not received:
            print(f"ERROR: no fresh sample received on {topic}")
    if not node.clock_advanced():
        print("ERROR: /clock did not advance by 0.2 simulation seconds")
    if not node.sensor_config_ready:
        print("ERROR: sensor config is missing or invalid")
    if not node.lidar_sample_count_ready():
        print(
            "ERROR: scan lengths do not match configured lidar_samples="
            f"{node.requested_lidar_samples}; observed={node.scan_shapes}"
        )
    if args.verify_lidar_rate and not node.lidar_rate_ready():
        for topic in node.scan_stamps_ns:
            observed = node.lidar_rate_results().get(topic)
            print(
                f"ERROR: {topic} requested_rate="
                f"{node.requested_lidar_rate_hz}Hz observed_rate="
                f"{observed if observed is not None else 'unavailable'}Hz"
            )
    if args.require_realtime_lidar and not node.lidar_wall_rate_ready():
        for topic in node.scan_wall_times:
            observed = node.lidar_wall_rate_results().get(topic)
            print(
                f"ERROR: {topic} requested_realtime_rate="
                f"{node.requested_lidar_rate_hz}Hz observed_wall_rate="
                f"{observed if observed is not None else 'unavailable'}Hz"
            )
    if node.require_lidar_intensity:
        for topic, ready in node.intensity_ready.items():
            if not ready:
                print(
                    f"ERROR: {topic} has no aligned non-zero LaserScan intensities"
                )
    if args.verify_motion and not node.motion_ready():
        print(
            "ERROR: motion verification failed: "
            f"moving_command_received={node.moving_command_received}, "
            f"odom_distance={node.odom_motion_distance():.3f}m, "
            f"required={args.minimum_motion_distance:.3f}m"
        )
    if args.verify_navigation and not node.navigation_ready():
        for topic, received in node.navigation_received.items():
            if not received:
                print(f"ERROR: no fresh navigation sample received on {topic}")
        if node.global_path_pose_count < 2:
            print(
                "ERROR: global path is missing or trivial: "
                f"poses={node.global_path_pose_count}"
            )
        if not node.successful_inference_received:
            print("ERROR: no successful DRL-VO inference metric received")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
