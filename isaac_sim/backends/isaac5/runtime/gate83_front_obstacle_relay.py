#!/usr/bin/env python3
"""Relay dual scans with one bounded synthetic near-obstacle interval."""

from __future__ import annotations

import argparse
import copy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class FrontObstacleRelay(Node):
    def __init__(self, normal_until: float, recover_after: float, obstacle_range: float):
        super().__init__("isaac5_gate83_front_obstacle_relay")
        self.normal_until_ns = int(normal_until * 1e9)
        self.recover_after_ns = int(recover_after * 1e9)
        self.obstacle_range = float(obstacle_range)
        self.epoch_ns = None
        self.scan_publishers = tuple(
            self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
            for topic in ("/isaac5/gate83/scan_01", "/isaac5/gate83/scan_02")
        )
        self.state_publisher = self.create_publisher(
            String, "/isaac5/gate83/obstacle_state", 20
        )
        self.scan_subscriptions = tuple(
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, sensor=index: self.on_scan(sensor, message),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(("/scan_01", "/scan_02"))
        )

    def phase(self, now_ns: int) -> str:
        if self.epoch_ns is None:
            self.epoch_ns = now_ns
        elapsed = now_ns - self.epoch_ns
        if elapsed < self.normal_until_ns:
            return "normal"
        if elapsed < self.recover_after_ns:
            return "near_obstacle"
        return "recovery"

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        phase = self.phase(now_ns)
        output = copy.deepcopy(message)
        if phase == "near_obstacle":
            # A full near ring makes the test independent of the two sensor
            # yaw extrinsics.  It is an input-space proximity proxy, not a
            # claim that a physical collider/contact was created.
            output.ranges = [self.obstacle_range] * len(output.ranges)
        self.scan_publishers[sensor].publish(output)
        state = String()
        state.data = json.dumps({
            "phase": phase,
            "sensor": sensor + 1,
            "clock_ns": now_ns,
            "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
            "obstacle_range_m": self.obstacle_range if phase == "near_obstacle" else None,
        }, sort_keys=True)
        self.state_publisher.publish(state)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal-until", type=float, default=4.0)
    parser.add_argument("--recover-after", type=float, default=7.0)
    parser.add_argument("--obstacle-range", type=float, default=0.5)
    args, ros_args = parser.parse_known_args()
    if not 0 < args.normal_until < args.recover_after:
        raise SystemExit("expected 0 < normal-until < recover-after")
    if args.obstacle_range <= 0:
        raise SystemExit("obstacle-range must be positive")
    rclpy.init(args=ros_args)
    node = FrontObstacleRelay(args.normal_until, args.recover_after, args.obstacle_range)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
