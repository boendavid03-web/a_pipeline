#!/usr/bin/env python3
"""Runtime monitor for the bounded Gate 9 DRL-VO closed-loop smoke."""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from semantic_nav_runtime.msg import TrackedPedestrianArray
from sensor_msgs.msg import LaserScan


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate9_closed_loop_monitor")
        self.odom = []
        self.scans = [[], []]
        self.tracks = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        for sensor, topic in enumerate(("/scan_01", "/scan_02")):
            self.create_subscription(
                LaserScan, topic,
                lambda message, index=sensor: self.on_scan(index, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(TrackedPedestrianArray, "/pedestrian_tracks", self.on_tracks, 20)
        self.create_subscription(InferenceMetrics, "/navigation_evaluation/inference_metrics", self.on_metrics, 20)
        self.create_subscription(ActuationDecision, "/drl_vo/actuation_decision", self.on_decision, 30)
        self.create_subscription(Twist, "/isaac5/gate9/cmd_vel", self.on_command, 20)

    def on_odom(self, message: Odometry) -> None:
        self.odom.append({
            "receive_wall": time.monotonic(),
            "position": [float(message.pose.pose.position.x), float(message.pose.pose.position.y), float(message.pose.pose.position.z)],
            "orientation": [float(message.pose.pose.orientation.w), float(message.pose.pose.orientation.x), float(message.pose.pose.orientation.y), float(message.pose.pose.orientation.z)],
            "twist": [float(message.twist.twist.linear.x), float(message.twist.twist.angular.z)],
        })

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        self.scans[sensor].append({
            "receive_wall": time.monotonic(),
            "beams": len(message.ranges),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
        })

    def on_tracks(self, message: TrackedPedestrianArray) -> None:
        self.tracks.append({
            "receive_wall": time.monotonic(),
            "count": len(message.tracks),
            "confirmed": sum(track.state == "CONFIRMED" for track in message.tracks),
        })

    def on_metrics(self, message: InferenceMetrics) -> None:
        if message.producer_id == "drl_vo_policy":
            self.metrics.append({
                "receive_wall": time.monotonic(),
                "success": bool(message.success),
                "sequence_id": int(message.sequence_id),
                "policy_ms": float(message.policy_ms),
                "action": [float(value) for value in message.action],
            })

    def on_decision(self, message: ActuationDecision) -> None:
        if message.has_raw_action and message.inference_sequence_id > 0:
            self.decisions.append({
                "receive_wall": time.monotonic(),
                "inference_sequence_id": int(message.inference_sequence_id),
                "reasons": list(message.gate_reasons),
                "raw": [float(message.raw_physical_action.linear.x), float(message.raw_physical_action.angular.z)],
                "final": [float(message.final_command.linear.x), float(message.final_command.angular.z)],
                "front_min": float(message.front_min_range_m) if message.has_front_min_range else None,
            })

    def on_command(self, message: Twist) -> None:
        self.commands.append({
            "receive_wall": time.monotonic(),
            "value": [float(message.linear.x), float(message.angular.z)],
        })


def rate(rows) -> float | None:
    if len(rows) < 2:
        return None
    span = rows[-1]["receive_wall"] - rows[0]["receive_wall"]
    return (len(rows) - 1) / span if span > 0 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.5)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    positions = np.asarray([row["position"] for row in node.odom], dtype=float)
    travel = 0.0 if not len(positions) else float(np.max(np.linalg.norm(positions[:, :2] - positions[0, :2], axis=1)))
    successful = [row for row in node.metrics if row["success"]]
    nonzero_commands = [row for row in node.commands if abs(row["value"][0]) > 1e-4 or abs(row["value"][1]) > 1e-4]
    checks = {
        "dynamic_odom_finite": len(node.odom) >= 20
        and positions.shape[1:] == (3,)
        and bool(np.isfinite(positions).all()),
        "bounded_robot_motion": 0.10 <= travel <= 2.0,
        "dual_lidar_mobile_delivery": all(
            len(rows) >= 20 and rate(rows) is not None and 13.5 <= rate(rows) <= 16.5
            and all(row["beams"] == 2000 for row in rows)
            for rows in node.scans
        ),
        "tracked_people_in_closed_loop": any(row["confirmed"] > 0 for row in node.tracks),
        "drlvo_inference_in_closed_loop": len(successful) >= 10 and all(
            math.isfinite(row["policy_ms"]) and len(row["action"]) == 2
            and all(math.isfinite(value) for value in row["action"])
            for row in successful
        ),
        "bounded_actuation_decisions": len(node.decisions) >= 10 and all(
            -1e-6 <= row["final"][0] <= 0.300001 and abs(row["final"][1]) <= 1.000001
            for row in node.decisions
        ),
        "nonzero_commands_reach_simulator": len(nonzero_commands) >= 10 and all(
            -1e-6 <= row["value"][0] <= 0.300001 and abs(row["value"][1]) <= 1.000001
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "9-smoke",
        "component": "bounded DRL-VO closed-loop mobile Mecanum in animated crowd",
        "odom_count": len(node.odom),
        "robot_travel_m": travel,
        "scan_counts": [len(rows) for rows in node.scans],
        "scan_rates_hz": [rate(rows) for rows in node.scans],
        "track_frames": len(node.tracks),
        "successful_inferences": len(successful),
        "inference_rate_hz": rate(successful),
        "actuation_decisions": len(node.decisions),
        "command_count": len(node.commands),
        "nonzero_command_count": len(nonzero_commands),
        "checks": checks,
        "decision_sample": node.decisions[-1] if node.decisions else None,
        "final_odom": node.odom[-1] if node.odom else None,
    }
    print("GATE9_CLOSED_LOOP_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
