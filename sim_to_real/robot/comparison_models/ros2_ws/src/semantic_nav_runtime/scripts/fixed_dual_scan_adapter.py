#!/usr/bin/env python3
"""Republish two panoramic hardware scans on the fixed S3-Net layout.

The adapter performs nearest-beam circular resampling onto 2000 samples from
[-pi, +pi], converts returns outside [0.1, 8.0] m to +Inf, and preserves the
input timestamps and sensor frame IDs.  It does not publish TF or cmd_vel.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def circular_nearest_indices(
    input_count: int,
    input_angle_min: float,
    input_angle_increment: float,
    output_count: int,
    output_angle_min: float,
    output_angle_max: float,
) -> np.ndarray:
    if input_count < 2 or output_count < 2:
        raise ValueError("input_count and output_count must both be at least 2")
    if not math.isfinite(input_angle_increment) or input_angle_increment <= 0.0:
        raise ValueError("input angle_increment must be positive and finite")
    input_span = input_angle_increment * input_count
    if input_span < 2.0 * math.pi - 2.5 * input_angle_increment:
        raise ValueError("input LaserScan is not panoramic enough for circular resampling")

    target = np.linspace(
        output_angle_min,
        output_angle_max,
        output_count,
        dtype=np.float64,
    )
    phase = np.mod(target - float(input_angle_min), 2.0 * math.pi)
    return np.rint(phase / float(input_angle_increment)).astype(np.int64) % input_count


def adapt_scan_arrays(
    ranges,
    intensities,
    *,
    input_angle_min: float,
    input_angle_increment: float,
    input_range_min: float,
    input_range_max: float,
    output_count: int = 2000,
    output_angle_min: float = -math.pi,
    output_angle_max: float = math.pi,
    output_range_min: float = 0.1,
    output_range_max: float = 8.0,
) -> tuple[np.ndarray, np.ndarray]:
    source_ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    source_intensities = np.asarray(intensities, dtype=np.float32).reshape(-1)
    if source_ranges.size < 2:
        raise ValueError("input LaserScan must contain at least two ranges")
    if source_intensities.size not in (0, source_ranges.size):
        raise ValueError("intensities must be empty or match ranges")
    if not 0.0 <= output_range_min < output_range_max:
        raise ValueError("invalid output range interval")

    indices = circular_nearest_indices(
        source_ranges.size,
        input_angle_min,
        input_angle_increment,
        output_count,
        output_angle_min,
        output_angle_max,
    )
    selected_ranges = source_ranges[indices].copy()
    low = max(float(input_range_min), float(output_range_min))
    high = min(float(input_range_max), float(output_range_max))
    valid = np.isfinite(selected_ranges) & (selected_ranges >= low) & (selected_ranges <= high)
    selected_ranges[~valid] = np.inf

    if source_intensities.size:
        selected_intensities = source_intensities[indices].copy()
        selected_intensities[~np.isfinite(selected_intensities)] = 0.0
    else:
        selected_intensities = np.zeros(output_count, dtype=np.float32)
    return selected_ranges, selected_intensities


class FixedDualScanAdapter(Node):
    def __init__(self) -> None:
        super().__init__("fixed_dual_scan_adapter")
        self.input_topics = (
            str(self.declare_parameter("input_scan_01_topic", "/scan_01").value),
            str(self.declare_parameter("input_scan_02_topic", "/scan_02").value),
        )
        self.output_topics = (
            str(
                self.declare_parameter(
                    "output_scan_01_topic", "/sim_to_real/aligned/scan_01"
                ).value
            ),
            str(
                self.declare_parameter(
                    "output_scan_02_topic", "/sim_to_real/aligned/scan_02"
                ).value
            ),
        )
        self.output_count = int(self.declare_parameter("output_beams", 2000).value)
        self.output_range_min = float(self.declare_parameter("range_min", 0.1).value)
        self.output_range_max = float(self.declare_parameter("range_max", 8.0).value)
        self.expected_input_beams = int(
            self.declare_parameter("expected_input_beams", 2000).value
        )
        self.max_scan_age = float(self.declare_parameter("max_scan_age", 0.5).value)
        self.max_pair_time_difference = float(
            self.declare_parameter("max_pair_time_difference", 0.05).value
        )
        self.last_stamp_ns = [None, None]
        self.scan_publishers = tuple(
            self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
            for topic in self.output_topics
        )
        self.scan_subscriptions = tuple(
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, sensor=index: self.scan_callback(sensor, message),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(self.input_topics)
        )
        self.get_logger().info(
            "fixed dual scan adapter ready: "
            f"{self.input_topics} -> {self.output_topics}; "
            f"layout=2x{self.output_count}, angles=[-pi,+pi], "
            f"range=[{self.output_range_min},{self.output_range_max}] m"
        )

    def scan_callback(self, sensor: int, message: LaserScan) -> None:
        if not str(message.header.frame_id).strip():
            self.get_logger().error(f"scan_{sensor + 1:02d} rejected: empty frame_id")
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= 0 or (now_ns - stamp_ns) / 1e9 > self.max_scan_age:
            self.get_logger().error(f"scan_{sensor + 1:02d} rejected: stale or zero timestamp")
            return
        self.last_stamp_ns[sensor] = stamp_ns
        other_stamp_ns = self.last_stamp_ns[1 - sensor]
        if other_stamp_ns is not None and abs(stamp_ns - other_stamp_ns) / 1e9 > self.max_pair_time_difference:
            self.get_logger().warning(
                f"scan pair timestamp difference exceeds {self.max_pair_time_difference:.3f} s",
                throttle_duration_sec=2.0,
            )
        if not math.isfinite(float(message.angle_min)) or not math.isfinite(float(message.angle_increment)):
            self.get_logger().error(f"scan_{sensor + 1:02d} rejected: invalid angular metadata")
            return
        if not (math.isfinite(float(message.range_min)) and math.isfinite(float(message.range_max))
                and 0.0 <= float(message.range_min) < float(message.range_max)):
            self.get_logger().error(f"scan_{sensor + 1:02d} rejected: invalid range metadata")
            return
        if self.expected_input_beams > 0 and len(message.ranges) != self.expected_input_beams:
            self.get_logger().error(
                f"scan_{sensor + 1:02d} expected {self.expected_input_beams} beams, "
                f"got {len(message.ranges)}"
            )
            return
        try:
            ranges, intensities = adapt_scan_arrays(
                message.ranges,
                message.intensities,
                input_angle_min=float(message.angle_min),
                input_angle_increment=float(message.angle_increment),
                input_range_min=float(message.range_min),
                input_range_max=float(message.range_max),
                output_count=self.output_count,
                output_range_min=self.output_range_min,
                output_range_max=self.output_range_max,
            )
        except ValueError as exc:
            self.get_logger().error(f"scan_{sensor + 1:02d} rejected: {exc}")
            return

        output = LaserScan()
        output.header = message.header
        output.angle_min = -math.pi
        output.angle_max = math.pi
        output.angle_increment = (output.angle_max - output.angle_min) / float(
            self.output_count - 1
        )
        output.time_increment = (
            float(message.scan_time) / float(self.output_count - 1)
            if float(message.scan_time) > 0.0
            else 0.0
        )
        output.scan_time = float(message.scan_time)
        output.range_min = self.output_range_min
        output.range_max = self.output_range_max
        output.ranges = ranges.tolist()
        output.intensities = intensities.tolist()
        self.scan_publishers[sensor].publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedDualScanAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
