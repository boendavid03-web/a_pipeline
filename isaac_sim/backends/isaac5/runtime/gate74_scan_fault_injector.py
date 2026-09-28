#!/usr/bin/env python3
"""Bounded scan-stamp fault injector for the Isaac 5 freshness gate.

The node never touches the original aligned scan topics.  It first republishes
them unchanged, then subtracts a fixed offset from both timestamps, and finally
returns to unchanged timestamps so the downstream recovery can be observed.
"""

from __future__ import annotations

import argparse
import copy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def set_stamp_ns(stamp, value: int) -> None:
    value = max(1, int(value))
    stamp.sec = value // NANOSECONDS_PER_SECOND
    stamp.nanosec = value % NANOSECONDS_PER_SECOND


class ScanFaultInjector(Node):
    def __init__(self, normal_until: float, stale_until: float, stale_offset: float):
        super().__init__("isaac5_gate74_scan_fault_injector")
        self.normal_until_ns = int(normal_until * NANOSECONDS_PER_SECOND)
        self.stale_until_ns = int(stale_until * NANOSECONDS_PER_SECOND)
        self.stale_offset_ns = int(stale_offset * NANOSECONDS_PER_SECOND)
        self.epoch_ns = None
        self.scan_publishers = (
            self.create_publisher(
                LaserScan, "/isaac5/gate74/scan_01", qos_profile_sensor_data
            ),
            self.create_publisher(
                LaserScan, "/isaac5/gate74/scan_02", qos_profile_sensor_data
            ),
        )
        self.state_publisher = self.create_publisher(
            String, "/isaac5/gate74/fault_state", 20
        )
        self.scan_subscriptions = tuple(
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, sensor=index: self.on_scan(sensor, message),
                qos_profile_sensor_data,
            )
            for index, topic in enumerate(
                ("/sim_to_real/aligned/scan_01", "/sim_to_real/aligned/scan_02")
            )
        )

    def phase(self, now_ns: int) -> str:
        if self.epoch_ns is None:
            self.epoch_ns = now_ns
        elapsed_ns = now_ns - self.epoch_ns
        if elapsed_ns < self.normal_until_ns:
            return "normal"
        if elapsed_ns < self.stale_until_ns:
            return "stale"
        return "recovery"

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        phase = self.phase(now_ns)
        original_ns = stamp_ns(message.header.stamp)
        output_ns = original_ns
        if phase == "stale":
            output_ns -= self.stale_offset_ns
        output = copy.deepcopy(message)
        set_stamp_ns(output.header.stamp, output_ns)
        self.scan_publishers[sensor].publish(output)
        state = String()
        state.data = json.dumps(
            {
                "phase": phase,
                "sensor": sensor + 1,
                "clock_ns": now_ns,
                "original_stamp_ns": original_ns,
                "output_stamp_ns": max(1, output_ns),
                "stale_offset_ns": self.stale_offset_ns if phase == "stale" else 0,
            },
            sort_keys=True,
        )
        self.state_publisher.publish(state)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal-until", type=float, default=4.0)
    parser.add_argument("--stale-until", type=float, default=7.5)
    parser.add_argument("--stale-offset", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    if not 0.0 < args.normal_until < args.stale_until:
        raise SystemExit("normal-until must be positive and less than stale-until")
    if args.stale_offset <= 0.0:
        raise SystemExit("stale-offset must be positive")
    rclpy.init(args=ros_args)
    node = ScanFaultInjector(args.normal_until, args.stale_until, args.stale_offset)
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
