#!/usr/bin/env python3
"""Bidirectional ROS 2 bridge for the isolated Isaac Sim 6.0.1 runner.

The system-Humble process forwards ``/cmd_vel`` to Isaac over UDP and turns
Isaac telemetry datagrams into the ROS topics needed by the existing Gazebo
rosbag/data pipeline.  Keeping rclpy outside Kit avoids mixing Ubuntu's Python
3.10 ROS packages with Isaac Sim 6.0.1's embedded Python runtime.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import socket
import struct
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from semantic_nav_gazebo.msg import PedestrianState, PedestrianStateArray
from navigation_evaluation_msgs.msg import SimulatorActuationState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from udp_telemetry import TelemetryDecoder
from isaac_actuation_contract import (
    COMMAND_PROTOCOL_VERSION,
    actual_velocity_from_actuation,
)


HOST = "127.0.0.1"
COMMAND_PORT = int(os.environ.get("ISAAC_CMD_VEL_UDP_PORT", "15973"))
TELEMETRY_PORT = int(os.environ.get("ISAAC_TELEMETRY_UDP_PORT", "15974"))
RESET_PORT = int(os.environ.get("ISAAC_RESET_UDP_PORT", "15975"))
COMMAND_PACKET = struct.Struct("!IQdddd")
TELEMETRY_SCHEMA = "isaac_6_warehouse_telemetry/v1"
MANUAL_EPISODE_SCHEMA = "isaac_manual_teleop_episode/v1"
MANUAL_EPISODE_EVENTS_ENABLED = os.environ.get(
    "ISAAC_MANUAL_EPISODE_EVENTS", "1"
).strip().lower() not in {"0", "false", "no", "off"}
SENSOR_CONFIG_SCHEMA = "isaac_sensor_config/v1"
BRIDGE_SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
STOP_DWELL_SEC = 0.5


def finite_vector(values: Iterable[object], length: int, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return result


class IsaacUdpRosBridge:
    def __init__(self) -> None:
        self.node = rclpy.create_node("isaac_6_udp_ros_bridge")
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_socket.bind((HOST, TELEMETRY_PORT))
        self.telemetry_socket.setblocking(False)
        self.reset_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_decoder = TelemetryDecoder()

        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.node.create_publisher(
            TFMessage, "/tf_static", static_qos
        )
        self.scan_pub = self.node.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.scan_01_pub = self.node.create_publisher(
            LaserScan, "/scan_01", qos_profile_sensor_data
        )
        self.scan_02_pub = self.node.create_publisher(
            LaserScan, "/scan_02", qos_profile_sensor_data
        )
        self.cmd_stamped_pub = self.node.create_publisher(
            TwistStamped, "/cmd_vel_stamped", 10
        )
        self.actuation_state_pub = self.node.create_publisher(
            SimulatorActuationState, "/isaac/actuation_state", 30
        )
        self.pedestrian_pub = self.node.create_publisher(
            PedestrianStateArray, "/pedestrian_ground_truth", 10
        )
        self.episode_pub = self.node.create_publisher(
            String, "/data_collection/episode_event", 10
        )
        config_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sensor_config_pub = self.node.create_publisher(
            String, "/data_collection/sensor_config", config_qos
        )
        self.reset_event_pub = self.node.create_publisher(
            String, "/isaac/reset_event", 10
        )
        self.command_sub = self.node.create_subscription(
            Twist, "/cmd_vel", self.on_command, 10
        )
        self.reset_sub = self.node.create_subscription(
            PoseStamped, "/isaac/reset_pose", self.on_reset_pose, 10
        )
        self.node.create_timer(0.005, self.poll_telemetry)
        self.node.create_timer(0.05, self.check_episode_stop)

        self.sim_time = 0.0
        self.last_telemetry_time = -math.inf
        self.telemetry_count = 0
        self.command_count = 0
        self.command_sequence_id = 0
        self.telemetry_sequence_id = 0
        self.reset_sequence_id = 0
        self.last_sensor_config = ""
        self.last_sensor_config_publish_sim_time = -math.inf
        self.discarded_count = 0
        self.episode_number = 0
        self.episode_active = False
        self.stop_started_sim_time: float | None = None
        self.robot_pose = [0.0, 0.0, 0.0]
        self.publish_static_tf()

    @staticmethod
    def stamp(seconds: float):
        seconds = max(0.0, float(seconds))
        whole = int(seconds)
        stamp = Clock().clock
        stamp.sec = whole
        stamp.nanosec = min(
            999_999_999, max(0, int(round((seconds - whole) * 1_000_000_000)))
        )
        return stamp

    def publish_static_tf(self) -> None:
        transforms = []
        for child, translation, yaw in (
            ("base_scan", (0.2, 0.13, 0.208), 0.0),
            ("base_scan_01", (0.2, 0.13, 0.208), 0.0),
            ("base_scan_02", (-0.2, -0.13, 0.208), math.pi),
        ):
            transform = TransformStamped()
            transform.header.stamp = self.stamp(self.sim_time)
            transform.header.frame_id = "base_link"
            transform.child_frame_id = child
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            transform.transform.rotation.z = math.sin(0.5 * yaw)
            transform.transform.rotation.w = math.cos(0.5 * yaw)
            transforms.append(transform)
        self.tf_static_pub.publish(TFMessage(transforms=transforms))

    def publish_episode_event(self, event: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                # This is intentionally distinct from
                # semantic_nav_episode_event/v1, whose formal V7 contract
                # requires a user-selected navigation goal.  A keyboard-only
                # session has no such goal and must not invent one.
                "schema": MANUAL_EPISODE_SCHEMA,
                "event": event,
                "episode_id": self.episode_number,
                "stamp_ns": int(self.node.get_clock().now().nanoseconds),
                "sim_time": self.sim_time,
                "pose": self.robot_pose,
            },
            separators=(",", ":"),
        )
        self.episode_pub.publish(message)
        print(
            f"[ISAAC-ROS-BRIDGE] episode {event}: "
            f"isaac_6_teleop_{self.episode_number:04d} at {self.sim_time:.3f}s",
            flush=True,
        )

    def on_command(self, message: Twist) -> None:
        command = finite_vector(
            (message.linear.x, message.linear.y, message.angular.z), 3, "cmd_vel"
        )
        self.command_sequence_id += 1
        self.command_socket.sendto(
            COMMAND_PACKET.pack(
                COMMAND_PROTOCOL_VERSION, self.command_sequence_id, self.sim_time, *command
            ),
            (HOST, COMMAND_PORT),
        )
        self.command_count += 1

        # rosbag2 discovers requested topics asynchronously.  In practice its
        # /cmd_vel subscription can appear seconds after /cmd_vel_stamped.  Do
        # not create control labels before the raw command stream is actually
        # being recorded, otherwise a short bag can contain unmatched labels.
        recorder_has_raw_command = any(
            endpoint.node_name == "rosbag2_recorder"
            for endpoint in self.node.get_subscriptions_info_by_topic("/cmd_vel")
        )
        if recorder_has_raw_command:
            stamped = TwistStamped()
            stamped.header.stamp = self.stamp(self.sim_time)
            stamped.header.frame_id = "base_link"
            stamped.twist = message
            self.cmd_stamped_pub.publish(stamped)

        if MANUAL_EPISODE_EVENTS_ENABLED:
            moving = any(abs(value) > 1.0e-6 for value in command)
            if moving:
                self.stop_started_sim_time = None
                if not self.episode_active and self.last_telemetry_time > -math.inf:
                    self.episode_number += 1
                    self.episode_active = True
                    self.publish_episode_event("start")
            elif self.episode_active and self.stop_started_sim_time is None:
                self.stop_started_sim_time = self.sim_time

        if self.command_count == 1:
            print(
                "[ISAAC-ROS-BRIDGE] First /cmd_vel forwarded: "
                f"vx={command[0]:.3f}, vy={command[1]:.3f}, wz={command[2]:.3f}",
                flush=True,
            )

    def on_reset_pose(self, message: PoseStamped) -> None:
        """Forward an explicit, finite map/odom-frame reset request to Isaac."""
        frame = message.header.frame_id.lstrip("/")
        if frame not in {"map", "odom"}:
            self.node.get_logger().error(
                f"rejecting Isaac reset in unsupported frame {message.header.frame_id!r}"
            )
            return
        pose = message.pose
        values = finite_vector(
            (
                pose.position.x,
                pose.position.y,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            6,
            "reset_pose",
        )
        qx, qy, qz, qw = values[2:]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1.0e-6:
            self.node.get_logger().error("rejecting Isaac reset with zero quaternion")
            return
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.reset_sequence_id += 1
        payload = {
            "schema": "isaac_reset_request/v1",
            "sequence_id": self.reset_sequence_id,
            "bridge_sim_time": self.sim_time,
            "frame_id": frame,
            "pose": [values[0], values[1], yaw],
        }
        self.reset_socket.sendto(
            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            (HOST, RESET_PORT),
        )

    def check_episode_stop(self) -> None:
        if (
            MANUAL_EPISODE_EVENTS_ENABLED
            and self.episode_active
            and self.stop_started_sim_time is not None
            and self.sim_time - self.stop_started_sim_time >= STOP_DWELL_SEC
        ):
            self.publish_episode_event("end")
            self.episode_active = False
            self.stop_started_sim_time = None

    def make_scan(self, payload: dict, frame_id: str) -> LaserScan:
        raw_ranges = payload["ranges"]
        if not isinstance(raw_ranges, list) or not raw_ranges:
            raise ValueError(f"{frame_id}.ranges must be a non-empty list")
        ranges = []
        for value in raw_ranges:
            if value is None:
                ranges.append(float("inf"))
            else:
                numeric = float(value)
                ranges.append(numeric if math.isfinite(numeric) else float("inf"))
        raw_intensities = payload.get("intensities")
        intensities: list[float] = []
        if raw_intensities is not None:
            if not isinstance(raw_intensities, list):
                raise ValueError(f"{frame_id}.intensities must be a list")
            if len(raw_intensities) != len(ranges):
                raise ValueError(
                    f"{frame_id}.intensities has {len(raw_intensities)} values, "
                    f"expected {len(ranges)}"
                )
            intensities = [float(value) for value in raw_intensities]
            if not all(
                math.isfinite(value) and value >= 0.0 for value in intensities
            ):
                raise ValueError(
                    f"{frame_id}.intensities must contain finite non-negative values"
                )
        angle_min = float(payload.get("angle_min", -math.pi))
        angle_increment = float(payload["angle_increment"])
        range_min = float(payload["range_min"])
        range_max = float(payload["range_max"])
        scan_time = float(payload.get("scan_time", 0.1))
        if not all(
            math.isfinite(value)
            for value in (angle_min, angle_increment, range_min, range_max, scan_time)
        ):
            raise ValueError(f"{frame_id} scan metadata must be finite")
        if (
            angle_increment <= 0.0
            or range_min < 0.0
            or range_max <= range_min
            or scan_time <= 0.0
        ):
            raise ValueError(f"{frame_id} scan metadata is invalid")

        message = LaserScan()
        # Keep validating the raw acquisition counter because it is part of
        # the producer contract, but never use it as a ROS timestamp. RTX GMO
        # timestampNs can advance more slowly than the USD timeline when
        # render captures are dropped. /clock, odom, and TF all use
        # self.sim_time, so LaserScan must use that same domain for tf2 and
        # slam_toolbox message filters to accept the scan.
        sensor_timestamp_ns = payload.get("sensor_timestamp_ns")
        if sensor_timestamp_ns is not None:
            sensor_timestamp_ns = int(sensor_timestamp_ns)
            if sensor_timestamp_ns < 0:
                raise ValueError(f"{frame_id}.sensor_timestamp_ns must be non-negative")
        message.header.stamp = self.stamp(self.sim_time)
        message.header.frame_id = frame_id
        message.angle_min = angle_min
        message.angle_increment = angle_increment
        message.angle_max = angle_min + (len(ranges) - 1) * angle_increment
        message.scan_time = scan_time
        message.time_increment = scan_time / len(ranges)
        message.range_min = range_min
        message.range_max = range_max
        message.ranges = ranges
        message.intensities = intensities
        return message

    def publish_odometry(self, pose: list[float], reported_velocity: list[float]) -> None:
        x, y, yaw = pose
        stamp = self.stamp(self.sim_time)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
        # This twist is the simulator-reported rigid-body state, retained for
        # controller compatibility and PhysX diagnostics.  Formal evaluation
        # derives independent motion truth from consecutive pose samples and
        # this same simulation-time stamp; it never treats this write/read-back
        # value as the sole command-tracking truth.
        odom.twist.twist.linear.x = reported_velocity[0]
        odom.twist.twist.linear.y = reported_velocity[1]
        odom.twist.twist.angular.z = reported_velocity[2]
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[transform]))

    def publish_actuation_state(self, payload: dict) -> list[float]:
        actuation = payload.get("actuation")
        if not isinstance(actuation, dict):
            raise ValueError("telemetry must contain actuation state")
        actual = list(actual_velocity_from_actuation(actuation))
        received = finite_vector(actuation["received_command"], 3, "received_command")
        applied = finite_vector(actuation["applied_command"], 3, "applied_command")
        source = str(actuation["actual_velocity_source"])
        state = SimulatorActuationState()
        state.header.stamp = self.stamp(self.sim_time)
        state.header.frame_id = "base_link"
        self.telemetry_sequence_id += 1
        state.telemetry_sequence_id = self.telemetry_sequence_id
        state.command_received = bool(actuation.get("command_received", False))
        state.command_sequence_id = int(actuation.get("command_sequence_id", 0))
        receive_time = actuation.get("bridge_receive_sim_time")
        if receive_time is not None and math.isfinite(float(receive_time)):
            state.bridge_receive_stamp = self.stamp(float(receive_time))
        state.received_command.linear.x, state.received_command.linear.y, state.received_command.angular.z = received
        state.applied_command.linear.x, state.applied_command.linear.y, state.applied_command.angular.z = applied
        state.actual_velocity.linear.x, state.actual_velocity.linear.y, state.actual_velocity.angular.z = actual
        state.actual_velocity_source = source
        age = actuation.get("command_age_sec")
        state.command_age_sec = float(age) if age is not None and math.isfinite(float(age)) else float("inf")
        state.watchdog_active = bool(actuation.get("watchdog_active", False))
        state.collision_protection_active = bool(actuation.get("collision_protection_active", False))
        state.control_reasons = [str(value) for value in actuation.get("control_reasons", [])]
        self.actuation_state_pub.publish(state)
        return actual

    def publish_pedestrians(self, payload: object) -> None:
        if not isinstance(payload, list):
            raise ValueError("pedestrians must be a list")
        output = PedestrianStateArray()
        output.header.stamp = self.stamp(self.sim_time)
        output.header.frame_id = "odom"
        for item in payload:
            if not isinstance(item, dict):
                raise ValueError("each pedestrian must be an object")
            position = finite_vector(item["position"], 3, "pedestrian.position")
            velocity = finite_vector(item["velocity"], 3, "pedestrian.velocity")
            yaw = float(item.get("yaw", 0.0))
            if not math.isfinite(yaw):
                raise ValueError("pedestrian.yaw must be finite")
            state = PedestrianState()
            state.id = str(item["id"])
            state.pose.position.x = position[0]
            state.pose.position.y = position[1]
            state.pose.position.z = position[2]
            state.pose.orientation.z = math.sin(0.5 * yaw)
            state.pose.orientation.w = math.cos(0.5 * yaw)
            state.velocity.linear.x = velocity[0]
            state.velocity.linear.y = velocity[1]
            state.velocity.linear.z = velocity[2]
            output.pedestrians.append(state)
        self.pedestrian_pub.publish(output)

    def publish_sensor_config(self, payload: object) -> None:
        if not isinstance(payload, dict):
            raise ValueError("sensor_config must be an object")
        if payload.get("schema") != SENSOR_CONFIG_SCHEMA:
            raise ValueError(
                f"unexpected sensor config schema: {payload.get('schema')!r}"
            )
        rate_hz = int(payload["lidar_rate_hz"])
        samples = int(payload["lidar_samples"])
        mode = str(payload["lidar_mode"])
        profile = str(payload["lidar_profile"])
        rate_basis = str(payload["lidar_rate_basis"])
        timestamp_domain = str(payload["lidar_timestamp_domain"])
        pairing_timestamp_domain = str(payload["lidar_pairing_timestamp_domain"])
        if not 1 <= rate_hz <= 30 or not 90 <= samples <= 4096:
            raise ValueError("sensor_config lidar rate or sample count is invalid")
        expected_pairing_timestamp_domain = {
            "rtx": "isaac_rtx_gmo_timestamp_ns",
            "physx": "isaac_telemetry_sim_time",
        }.get(mode)
        valid_profile = (
            profile in {"example_dense", "navigation_2d_32k", "rplidar_s2e"}
            if mode == "rtx"
            else profile == "physx_raycast"
        )
        if (
            not valid_profile
            or
            rate_basis != "simulation_time"
            or timestamp_domain != "isaac_telemetry_sim_time"
            or pairing_timestamp_domain != expected_pairing_timestamp_domain
        ):
            raise ValueError(
                "sensor_config lidar mode, profile, rate basis, and timestamp domain are inconsistent"
            )
        complete_payload = dict(payload)
        complete_payload["bridge_source_sha256"] = BRIDGE_SOURCE_SHA256
        encoded = json.dumps(
            complete_payload, sort_keys=True, separators=(",", ":")
        )
        if (
            encoded == self.last_sensor_config
            and self.sim_time < self.last_sensor_config_publish_sim_time + 1.0
        ):
            return
        message = String()
        message.data = encoded
        self.sensor_config_pub.publish(message)
        self.last_sensor_config = encoded
        self.last_sensor_config_publish_sim_time = self.sim_time

    def handle_telemetry(self, payload: dict) -> None:
        if payload.get("schema") != TELEMETRY_SCHEMA:
            raise ValueError(f"unexpected telemetry schema: {payload.get('schema')!r}")
        sim_time = float(payload["sim_time"])
        if not math.isfinite(sim_time) or sim_time < 0.0:
            raise ValueError("sim_time must be finite and non-negative")
        if sim_time + 1.0e-9 < self.sim_time:
            self.node.get_logger().warning(
                f"Isaac simulation time reset from {self.sim_time:.3f}s to {sim_time:.3f}s"
            )
            self.episode_active = False
            self.stop_started_sim_time = None
            self.publish_static_tf()
        self.sim_time = sim_time
        self.last_telemetry_time = sim_time
        sensor_config = payload.get("sensor_config")
        if sensor_config is not None:
            self.publish_sensor_config(sensor_config)

        clock = Clock()
        clock.clock = self.stamp(sim_time)
        self.clock_pub.publish(clock)
        event = payload.get("event")
        if event == "shutdown":
            if MANUAL_EPISODE_EVENTS_ENABLED and self.episode_active:
                self.publish_episode_event("end")
                self.episode_active = False
                self.stop_started_sim_time = None
            return
        reset_event = payload.get("reset_event")
        if isinstance(reset_event, dict):
            message = String()
            message.data = json.dumps(
                reset_event, sort_keys=True, separators=(",", ":")
            )
            self.reset_event_pub.publish(message)
        pose = finite_vector(payload["robot_pose"], 3, "robot_pose")
        # command is kept for legacy telemetry readers.  State below carries
        # received/applied/actual distinctly and is the evaluation truth.
        _legacy_command = finite_vector(payload["command"], 3, "command")
        self.robot_pose = pose
        actual_velocity = self.publish_actuation_state(payload)
        self.publish_odometry(pose, actual_velocity)

        if "pedestrians" in payload:
            self.publish_pedestrians(payload["pedestrians"])
        scans = payload.get("scans")
        if scans is not None:
            if not isinstance(scans, dict):
                raise ValueError("scans must be an object")
            front = self.make_scan(scans["scan_01"], "base_scan_01")
            rear = self.make_scan(scans["scan_02"], "base_scan_02")
            self.scan_01_pub.publish(front)
            self.scan_02_pub.publish(rear)
            legacy = self.make_scan(scans["scan_01"], "base_scan")
            self.scan_pub.publish(legacy)

        self.telemetry_count += 1
        if self.telemetry_count == 1:
            print(
                "[ISAAC-ROS-BRIDGE] First Isaac telemetry received; "
                "publishing /clock, /odom, /tf, dual scans and pedestrian truth",
                flush=True,
            )

    def poll_telemetry(self) -> None:
        while True:
            try:
                packet, _address = self.telemetry_socket.recvfrom(65_535)
            except BlockingIOError:
                return
            try:
                payload = self.telemetry_decoder.feed(packet)
                if payload is None:
                    continue
                self.handle_telemetry(payload)
            except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError) as error:
                self.discarded_count += 1
                if self.discarded_count == 1 or self.discarded_count % 100 == 0:
                    self.node.get_logger().warning(
                        f"discarding invalid Isaac telemetry datagram: {error}"
                    )

    def close(self) -> None:
        try:
            self.command_socket.sendto(
                COMMAND_PACKET.pack(
                    COMMAND_PROTOCOL_VERSION, self.command_sequence_id + 1,
                    self.sim_time, 0.0, 0.0, 0.0
                ), (HOST, COMMAND_PORT)
            )
        except OSError:
            pass
        if MANUAL_EPISODE_EVENTS_ENABLED and self.episode_active:
            # The launcher can shut down the ROS context before this relay's
            # finally block runs.  The end marker is best effort at teardown.
            try:
                if rclpy.ok():
                    self.publish_episode_event("end")
            except Exception as exc:
                self.node.get_logger().warning(
                    f"Could not publish final episode event during shutdown: {exc}"
                )
            self.episode_active = False
        self.command_socket.close()
        self.telemetry_socket.close()
        self.reset_socket.close()
        self.node.destroy_node()


def main() -> int:
    rclpy.init(args=None)
    bridge = IsaacUdpRosBridge()
    print(
        f"[ISAAC-ROS-BRIDGE] /cmd_vel -> udp://{HOST}:{COMMAND_PORT}; "
        f"telemetry udp://{HOST}:{TELEMETRY_PORT} -> ROS 2 ready",
        flush=True,
    )
    try:
        rclpy.spin(bridge.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble can surface an RCLError instead of ExternalShutdownException
        # when SIGTERM invalidates the context while the executor is waiting.
        # Suppress only that normal shutdown race; preserve real bridge errors.
        if rclpy.ok():
            raise
    finally:
        bridge.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
