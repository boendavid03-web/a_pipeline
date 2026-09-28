#!/usr/bin/env python3
"""Gate 9 monitor for bounded DRL-VO navigation to one fixed odom goal."""

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
    def __init__(self, command_topic: str):
        super().__init__("isaac5_gate9_fixed_goal_monitor")
        self.odom = []
        self.scans = [[], []]
        self.tracks = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        for sensor, topic in enumerate(("/scan_01", "/scan_02")):
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, index=sensor: self.on_scan(index, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(TrackedPedestrianArray, "/pedestrian_tracks", self.on_tracks, 20)
        self.create_subscription(
            InferenceMetrics,
            "/navigation_evaluation/inference_metrics",
            self.on_metrics,
            20,
        )
        self.create_subscription(
            ActuationDecision,
            "/drl_vo/actuation_decision",
            self.on_decision,
            30,
        )
        self.create_subscription(Twist, command_topic, self.on_command, 20)

    def on_odom(self, message: Odometry) -> None:
        self.odom.append({
            "receive_wall": time.monotonic(),
            "position": [
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                float(message.pose.pose.position.z),
            ],
            "twist": [
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z),
            ],
        })

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        self.scans[sensor].append({
            "receive_wall": time.monotonic(),
            "beams": len(message.ranges),
        })

    def on_tracks(self, message: TrackedPedestrianArray) -> None:
        self.tracks.append({
            "receive_wall": time.monotonic(),
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
        self.decisions.append({
            "receive_wall": time.monotonic(),
            "has_raw_action": bool(message.has_raw_action),
            "inference_sequence_id": int(message.inference_sequence_id),
            "reasons": list(message.gate_reasons),
            "final": [
                float(message.final_command.linear.x),
                float(message.final_command.angular.z),
            ],
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
    return (len(rows) - 1) / span if span > 0.0 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-tolerance", type=float, default=0.35)
    parser.add_argument("--command-topic", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.command_topic)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    positions = np.asarray([row["position"] for row in node.odom], dtype=float)
    distances = (
        np.linalg.norm(positions[:, :2] - np.asarray([args.goal_x, args.goal_y]), axis=1)
        if len(positions)
        else np.asarray([], dtype=float)
    )
    initial_distance = float(distances[0]) if len(distances) else None
    minimum_distance = float(np.min(distances)) if len(distances) else None
    final_distance = float(distances[-1]) if len(distances) else None
    successful = [row for row in node.metrics if row["success"]]
    nonzero_commands = [
        row for row in node.commands
        if abs(row["value"][0]) > 1.0e-4 or abs(row["value"][1]) > 1.0e-4
    ]
    goal_decisions = [row for row in node.decisions if "goal_reached" in row["reasons"]]
    first_goal_wall = goal_decisions[0]["receive_wall"] if goal_decisions else math.inf
    post_goal_commands = [row for row in node.commands if row["receive_wall"] >= first_goal_wall]
    post_goal_zero = [
        row for row in post_goal_commands
        if abs(row["value"][0]) <= 1.0e-4 and abs(row["value"][1]) <= 1.0e-4
    ]
    checks = {
        "fixed_goal_started_outside_tolerance": initial_distance is not None
        and initial_distance >= 1.30,
        "fixed_goal_geometrically_reached": minimum_distance is not None
        and minimum_distance <= args.goal_tolerance,
        "policy_declared_goal_reached": len(goal_decisions) >= 3,
        "zero_command_after_goal": len(post_goal_zero) >= 3
        and len(post_goal_zero) == len(post_goal_commands),
        "dual_lidar_contract_while_moving": all(
            len(rows) >= 80
            and rate(rows) is not None
            and 13.5 <= rate(rows) <= 16.5
            and all(row["beams"] == 2000 for row in rows)
            for rows in node.scans
        ),
        "tracked_people_during_navigation": any(row["confirmed"] > 0 for row in node.tracks),
        "drlvo_inference_during_navigation": len(successful) >= 20
        and all(
            math.isfinite(row["policy_ms"])
            and len(row["action"]) == 2
            and all(math.isfinite(value) for value in row["action"])
            for row in successful
        ),
        "bounded_commands": len(nonzero_commands) >= 20
        and all(
            -1.0e-6 <= row["value"][0] <= 0.300001
            and abs(row["value"][1]) <= 1.000001
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "9-fixed-goal",
        "component": "DRL-VO closed-loop fixed-goal navigation in animated crowd",
        "goal_odom": [args.goal_x, args.goal_y],
        "goal_tolerance_m": args.goal_tolerance,
        "initial_goal_distance_m": initial_distance,
        "minimum_goal_distance_m": minimum_distance,
        "final_goal_distance_m": final_distance,
        "odom_count": len(node.odom),
        "scan_counts": [len(rows) for rows in node.scans],
        "scan_rates_hz": [rate(rows) for rows in node.scans],
        "track_frames": len(node.tracks),
        "successful_inferences": len(successful),
        "inference_rate_hz": rate(successful),
        "actuation_decisions": len(node.decisions),
        "goal_reached_decisions": len(goal_decisions),
        "command_count": len(node.commands),
        "nonzero_command_count": len(nonzero_commands),
        "post_goal_zero_command_count": len(post_goal_zero),
        "checks": checks,
        "final_odom": node.odom[-1] if node.odom else None,
    }
    print("GATE9_FIXED_GOAL_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
