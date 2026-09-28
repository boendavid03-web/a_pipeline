"""Minimal ROS 2 Humble topic boundary for the Isaac5 custom runtime."""

from __future__ import annotations

import math
import sys
import threading
import time

import numpy as np

from robot_controller import clamp_twist


class RosControlBridge:
    """ROS imports are delayed until the custom Kit has started."""

    def __init__(self, use_scan: bool = False, use_dual_scan: bool = False):
        # Isaac Sim 5.1 ships a Python 3.11 Humble bundle inside the ROS
        # bridge extension.  Importing rclpy directly keeps the ROS topic
        # boundary while avoiding the monolithic extension's dependency on
        # isaacsim.sensors.rtx and omni.hydra.rtx.
        bundled_ros_root = "/home/user/isaacsim/5.1.0/exts/isaacsim.ros2.bridge/humble/rclpy"
        if bundled_ros_root not in sys.path:
            sys.path.insert(0, bundled_ros_root)
        import rclpy
        from builtin_interfaces.msg import Time as RosTime
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.RosTime = RosTime
        self.TransformStamped = TransformStamped
        self.TFMessage = TFMessage
        self.lock = threading.Lock()
        self.use_dual_scan = bool(use_dual_scan)
        self._owns_rclpy_context = not rclpy.ok()
        self._command = (0.0, 0.0, 0.0)
        self._last_command_wall = -math.inf
        self.received_count = 0
        self.command_timeout_sec = 0.5
        if self._owns_rclpy_context:
            rclpy.init(args=None)
        self.node = rclpy.create_node("isaac5_basic_navigation")
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.node.create_publisher(TFMessage, "/tf_static", static_qos)
        self.scan_pub = None
        self.scan_01_pub = None
        self.scan_02_pub = None
        if use_scan:
            from sensor_msgs.msg import LaserScan

            self.scan_pub = self.node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        if use_dual_scan:
            from sensor_msgs.msg import LaserScan

            self.scan_01_pub = self.node.create_publisher(
                LaserScan, "/scan_01", qos_profile_sensor_data
            )
            self.scan_02_pub = self.node.create_publisher(
                LaserScan, "/scan_02", qos_profile_sensor_data
            )
        self.cmd_sub = self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.publish_static_tf()

    def stamp(self, seconds: float):
        seconds = max(0.0, float(seconds))
        whole = int(seconds)
        return self.RosTime(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))

    def _on_cmd_vel(self, message: object) -> None:
        command = clamp_twist(message.linear.x, message.linear.y, message.angular.z)
        with self.lock:
            self._command = command
            self._last_command_wall = time.monotonic()
            self.received_count += 1

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.001)

    def current_command(self) -> tuple[float, float, float]:
        with self.lock:
            if time.monotonic() - self._last_command_wall > self.command_timeout_sec:
                return 0.0, 0.0, 0.0
            return self._command

    def publish_clock(self, sim_time: float) -> None:
        from rosgraph_msgs.msg import Clock

        message = Clock()
        message.clock = self.stamp(sim_time)
        self.clock_pub.publish(message)

    def publish_static_tf(self) -> None:
        specs = (
            ("base_scan_01", 0.20, 0.13, 0.208, 0.0),
            ("base_scan_02", -0.20, -0.13, 0.208, math.pi),
        ) if self.use_dual_scan else (
            ("base_scan", 0.20, 0.13, 0.208, 0.0),
        )
        transforms = []
        for child_frame, x, y, z, yaw in specs:
            transform = self.TransformStamped()
            transform.header.stamp = self.stamp(0.0)
            transform.header.frame_id = "base_link"
            transform.child_frame_id = child_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.w = math.cos(0.5 * yaw)
            transform.transform.rotation.z = math.sin(0.5 * yaw)
            transforms.append(transform)
        self.tf_static_pub.publish(self.TFMessage(transforms=transforms))

    def publish_state(
        self,
        sim_time: float,
        position: np.ndarray,
        orientation: np.ndarray,
        body_twist: tuple[float, float, float],
    ) -> None:
        from nav_msgs.msg import Odometry

        stamp = self.stamp(sim_time)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.w = float(orientation[0])
        odom.pose.pose.orientation.x = float(orientation[1])
        odom.pose.pose.orientation.y = float(orientation[2])
        odom.pose.pose.orientation.z = float(orientation[3])
        odom.twist.twist.linear.x = float(body_twist[0])
        odom.twist.twist.linear.y = float(body_twist[1])
        odom.twist.twist.angular.z = float(body_twist[2])
        self.odom_pub.publish(odom)

        tf = self.TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = float(position[0])
        tf.transform.translation.y = float(position[1])
        tf.transform.translation.z = float(position[2])
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.publish(self.TFMessage(transforms=[tf]))

    def publish_scan(self, scan_data: object) -> None:
        if self.scan_pub is None:
            return
        from sensor_msgs.msg import LaserScan

        message = LaserScan()
        message.header.stamp = self.stamp(scan_data.sim_time)
        message.header.frame_id = scan_data.frame_id
        message.angle_min = float(scan_data.angle_min)
        message.angle_max = float(scan_data.angle_min + (len(scan_data.ranges) - 1) * scan_data.angle_increment)
        message.angle_increment = float(scan_data.angle_increment)
        message.scan_time = float(getattr(scan_data, "scan_time", 1.0 / 10.0))
        message.time_increment = message.scan_time / max(1, len(scan_data.ranges))
        message.range_min = float(scan_data.range_min)
        message.range_max = float(scan_data.range_max)
        message.ranges = [float(value) for value in scan_data.ranges]
        self.scan_pub.publish(message)

    def publish_dual_scan(self, scan_pair: object) -> None:
        """Publish both scans without changing their frame or timestamp."""

        if self.scan_01_pub is None or self.scan_02_pub is None:
            return
        from sensor_msgs.msg import LaserScan

        messages = []
        for scan_data in (scan_pair.scan_01, scan_pair.scan_02):
            message = LaserScan()
            message.header.stamp = self.stamp(scan_data.sim_time)
            message.header.frame_id = scan_data.frame_id
            message.angle_min = float(scan_data.angle_min)
            message.angle_max = float(
                scan_data.angle_min
                + (len(scan_data.ranges) - 1) * scan_data.angle_increment
            )
            message.angle_increment = float(scan_data.angle_increment)
            message.scan_time = float(getattr(scan_data, "scan_time", 1.0 / 10.0))
            message.time_increment = message.scan_time / max(1, len(scan_data.ranges))
            message.range_min = float(scan_data.range_min)
            message.range_max = float(scan_data.range_max)
            message.ranges = [float(value) for value in scan_data.ranges]
            messages.append(message)
        self.scan_01_pub.publish(messages[0])
        self.scan_02_pub.publish(messages[1])

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy_context and self.rclpy.ok():
            self.rclpy.shutdown()
