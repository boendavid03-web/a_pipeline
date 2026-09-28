#!/usr/bin/env python3
"""Relay detected pedestrian tracks through a bounded freshness dropout."""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from semantic_nav_runtime.msg import TrackedPedestrianArray
from std_msgs.msg import String


class TrackDropoutRelay(Node):
    def __init__(self, normal_until: float, stale_after: float, recover_after: float):
        super().__init__("isaac5_gate82_track_dropout_relay")
        self.normal_until_ns = int(normal_until * 1e9)
        self.stale_after_ns = int(stale_after * 1e9)
        self.recover_after_ns = int(recover_after * 1e9)
        self.epoch_ns = None
        self.input_count = 0
        self.output_count = 0
        self.track_publisher = self.create_publisher(
            TrackedPedestrianArray, "/pedestrian_tracks", 20
        )
        self.state_publisher = self.create_publisher(
            String, "/isaac5/gate82/dropout_state", 20
        )
        self.track_subscription = self.create_subscription(
            TrackedPedestrianArray,
            "/pedestrian_tracks_raw",
            self.on_tracks,
            20,
        )

    def phase(self, now_ns: int) -> str:
        if self.epoch_ns is None:
            self.epoch_ns = now_ns
        elapsed = now_ns - self.epoch_ns
        if elapsed < self.normal_until_ns:
            return "normal"
        if elapsed < self.stale_after_ns:
            return "dropout_grace"
        if elapsed < self.recover_after_ns:
            return "stale"
        return "recovery"

    def on_tracks(self, message: TrackedPedestrianArray) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        phase = self.phase(now_ns)
        self.input_count += 1
        forwarded = phase in ("normal", "recovery")
        if forwarded:
            self.track_publisher.publish(message)
            self.output_count += 1
        state = String()
        state.data = json.dumps(
            {
                "phase": phase,
                "clock_ns": now_ns,
                "forwarded": forwarded,
                "input_count": self.input_count,
                "output_count": self.output_count,
            },
            sort_keys=True,
        )
        self.state_publisher.publish(state)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal-until", type=float, default=4.0)
    parser.add_argument("--stale-after", type=float, default=4.9)
    parser.add_argument("--recover-after", type=float, default=7.5)
    args, ros_args = parser.parse_known_args()
    if not 0 < args.normal_until < args.stale_after < args.recover_after:
        raise SystemExit("expected 0 < normal-until < stale-after < recover-after")
    rclpy.init(args=ros_args)
    node = TrackDropoutRelay(args.normal_until, args.stale_after, args.recover_after)
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
