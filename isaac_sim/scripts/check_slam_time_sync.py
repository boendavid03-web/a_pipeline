#!/usr/bin/env python3
"""Verify that live merged scans and /clock share the Isaac timeline."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


EXPECTED_TYPES = {
    "/scan_merged": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}


def time_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SlamTimeSyncCheck(Node):
    def __init__(self) -> None:
        super().__init__("isaac_slam_time_sync_check")
        self.clock_ns: int | None = None
        self.scan_ns: int | None = None
        self.clock_received_at = -math.inf
        self.scan_received_at = -math.inf
        self._subscriptions = [
            self.create_subscription(Clock, "/clock", self.on_clock, 10),
            self.create_subscription(
                LaserScan,
                "/scan_merged",
                self.on_scan,
                qos_profile_sensor_data,
            ),
        ]

    def on_clock(self, message: Clock) -> None:
        self.clock_ns = time_ns(message.clock)
        self.clock_received_at = time.monotonic()

    def on_scan(self, message: LaserScan) -> None:
        self.scan_ns = time_ns(message.header.stamp)
        self.scan_received_at = time.monotonic()

    def synchronized_delta(self, freshness_sec: float) -> float | None:
        if self.clock_ns is None or self.scan_ns is None:
            return None
        now = time.monotonic()
        if (
            now - self.clock_received_at > freshness_sec
            or now - self.scan_received_at > freshness_sec
        ):
            return None
        return (self.clock_ns - self.scan_ns) / 1.0e9

    def graph_errors(self) -> list[str]:
        discovered = dict(self.get_topic_names_and_types())
        errors = []
        for topic, expected_type in EXPECTED_TYPES.items():
            actual_types = discovered.get(topic, [])
            if actual_types != [expected_type]:
                errors.append(
                    f"{topic} types={actual_types!r}, expected [{expected_type!r}]"
                )
                continue
            publisher_count = len(self.get_publishers_info_by_topic(topic))
            if publisher_count <= 0:
                errors.append(f"{topic} has no publisher")
        return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-delta", type=float, default=1.0)
    parser.add_argument("--freshness", type=float, default=2.0)
    args = parser.parse_args()
    if args.timeout <= 0.0 or args.max_delta <= 0.0 or args.freshness <= 0.0:
        parser.error("all timing arguments must be positive")

    rclpy.init()
    node = SlamTimeSyncCheck()
    deadline = time.monotonic() + args.timeout
    last_delta: float | None = None
    graph_errors = ["ROS graph not inspected"]
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            graph_errors = node.graph_errors()
            delta = node.synchronized_delta(args.freshness)
            if delta is None:
                continue
            last_delta = delta
            if not graph_errors and abs(delta) <= args.max_delta:
                print(
                    "PASS slam_input_graph "
                    + " ".join(EXPECTED_TYPES)
                )
                print(
                    "PASS scan_clock_alignment "
                    f"delta_sec={delta:.6f} max_sec={args.max_delta:.3f}"
                )
                return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    for error in graph_errors:
        print(f"FAIL slam_input_graph: {error}")
    if last_delta is None:
        print("FAIL scan_clock_alignment: no fresh /clock and /scan_merged pair")
    else:
        print(
            "FAIL scan_clock_alignment: "
            f"clock_minus_scan_sec={last_delta:.6f}, "
            f"allowed_abs_sec={args.max_delta:.3f}"
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
