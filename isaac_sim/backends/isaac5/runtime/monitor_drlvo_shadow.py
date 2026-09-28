#!/usr/bin/env python3
"""External evidence monitor for Gate 8.1 DRL-VO shadow inference."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from semantic_nav_runtime.msg import DrlVoTrainingState, TrackedPedestrianArray
from sensor_msgs.msg import LaserScan


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate81_drlvo_monitor")
        self.scans = [[], []]
        self.scan_stamps = [set(), set()]
        self.odom = []
        self.local_goals = []
        self.final_goals = []
        self.tracks = []
        self.states = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        for sensor, topic in enumerate(("/scan_01", "/scan_02")):
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, index=sensor: self.on_scan(index, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(
            PointStamped,
            "/semantic_cnn/local_subgoal",
            self.on_local_goal,
            20,
        )
        self.create_subscription(
            PointStamped,
            "/semantic_cnn/final_goal",
            self.on_final_goal,
            20,
        )
        self.create_subscription(
            TrackedPedestrianArray, "/pedestrian_tracks", self.on_tracks, 20
        )
        self.create_subscription(
            DrlVoTrainingState, "/drl_vo/training_state", self.on_state, 20
        )
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
        self.create_subscription(
            Twist, "/isaac5/gate81/drl_vo_cmd_shadow", self.on_command, 20
        )

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        stamp = stamp_ns(message.header.stamp)
        self.scan_stamps[sensor].add(stamp)
        self.scans[sensor].append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp,
                "frame_id": message.header.frame_id,
                "beams": len(message.ranges),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
            }
        )

    def on_odom(self, message: Odometry) -> None:
        self.odom.append(
            {
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "position": [
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                ],
            }
        )

    def on_local_goal(self, message: PointStamped) -> None:
        self.local_goals.append(
            {
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "goal": [float(message.point.x), float(message.point.y)],
            }
        )

    def on_final_goal(self, message: PointStamped) -> None:
        self.final_goals.append(
            {
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "goal": [float(message.point.x), float(message.point.y)],
            }
        )

    def on_tracks(self, message: TrackedPedestrianArray) -> None:
        tracks = []
        for track in message.tracks:
            tracks.append(
                {
                    "id": int(track.track_id),
                    "state": track.state,
                    "position": [float(track.position.x), float(track.position.y)],
                    "velocity": [float(track.velocity.x), float(track.velocity.y)],
                    "confidence": float(track.confidence),
                }
            )
        self.tracks.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "tracks": tracks,
            }
        )

    def on_state(self, message: DrlVoTrainingState) -> None:
        observation = message.observation
        self.states.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "observation_size": len(observation),
                "observation_finite": all(math.isfinite(value) for value in observation),
                "minimum_scan_range": float(message.minimum_scan_range),
                "goal_distance": float(message.goal_distance),
            }
        )

    def on_metrics(self, message: InferenceMetrics) -> None:
        self.metrics.append(
            {
                "receive_wall": time.monotonic(),
                "input_stamp_ns": stamp_ns(message.input_stamp),
                "sequence_id": int(message.sequence_id),
                "producer_id": message.producer_id,
                "success": bool(message.success),
                "device": message.device,
                "model_parameters": int(message.model_parameters),
                "policy_ms": float(message.policy_ms),
                "total_ms": float(message.total_ms),
                "action": [float(value) for value in message.action],
            }
        )

    def on_decision(self, message: ActuationDecision) -> None:
        self.decisions.append(
            {
                "receive_wall": time.monotonic(),
                "decision_sequence_id": int(message.decision_sequence_id),
                "inference_sequence_id": int(message.inference_sequence_id),
                "has_raw_action": bool(message.has_raw_action),
                "gated": bool(message.gated),
                "reasons": list(message.gate_reasons),
                "raw": [
                    float(message.raw_physical_action.linear.x),
                    float(message.raw_physical_action.angular.z),
                ],
                "final": [
                    float(message.final_command.linear.x),
                    float(message.final_command.angular.z),
                ],
                "front_min": (
                    float(message.front_min_range_m)
                    if message.has_front_min_range
                    else None
                ),
            }
        )

    def on_command(self, message: Twist) -> None:
        self.commands.append(
            {
                "receive_wall": time.monotonic(),
                "value": [float(message.linear.x), float(message.angular.z)],
            }
        )


def rate(rows) -> float | None:
    if len(rows) < 2:
        return None
    span = rows[-1]["receive_wall"] - rows[0]["receive_wall"]
    return (len(rows) - 1) / span if span > 0.0 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
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

    common_scan_stamps = node.scan_stamps[0] & node.scan_stamps[1]
    nonempty_tracks = [row for row in node.tracks if row["tracks"]]
    confirmed_tracks = [
        track
        for row in nonempty_tracks
        for track in row["tracks"]
        if track["state"] == "CONFIRMED"
    ]
    successful = [row for row in node.metrics if row["success"]]
    metric_ids = {row["sequence_id"] for row in successful}
    inference_decisions = [
        row
        for row in node.decisions
        if row["has_raw_action"] and row["inference_sequence_id"] > 0
    ]
    finite_tracks = all(
        math.isfinite(track["confidence"])
        and all(math.isfinite(value) for value in track["position"] + track["velocity"])
        for row in node.tracks
        for track in row["tracks"]
    )
    finite_metrics = all(
        math.isfinite(row["policy_ms"])
        and math.isfinite(row["total_ms"])
        and len(row["action"]) == 2
        and all(math.isfinite(value) for value in row["action"])
        for row in successful
    )
    finite_decisions = all(
        all(math.isfinite(value) for value in row["raw"] + row["final"])
        and -1.0e-6 <= row["final"][0] <= 0.300001
        and abs(row["final"][1]) <= 1.500001
        for row in inference_decisions
    )
    checks = {
        "raw_dual_lidar_contract": all(len(rows) >= 20 for rows in node.scans)
        and all(
            row["beams"] == 2000
            and 0.499 <= row["range_min"] <= 0.501
            and 49.99 <= row["range_max"] <= 50.01
            for rows in node.scans
            for row in rows
        ),
        "dual_lidar_delivery_rate": all(
            rate(rows) is not None and 13.5 <= rate(rows) <= 16.5
            for rows in node.scans
        ),
        "odom_contract": bool(node.odom)
        and all(row["frame_id"] == "odom" and row["child_frame_id"] == "base_link" for row in node.odom),
        "goal_contract": bool(node.local_goals)
        and bool(node.final_goals)
        and all(row["frame_id"] == "base_link" for row in node.local_goals)
        and all(row["frame_id"] == "odom" for row in node.final_goals),
        "drspaam_tracks_observed": bool(nonempty_tracks)
        and bool(confirmed_tracks)
        and finite_tracks
        and all(row["frame_id"] == "odom" for row in node.tracks),
        "observation_contract": len(node.states) >= 10
        and all(
            row["observation_size"] == 19202
            and row["observation_finite"]
            and row["frame_id"] == "base_link"
            and row["stamp_ns"] in common_scan_stamps
            and math.isfinite(row["minimum_scan_range"])
            and math.isfinite(row["goal_distance"])
            for row in node.states
        ),
        "policy_inference_success": len(successful) >= 10
        and len(successful) == len(node.metrics)
        and all(
            row["producer_id"] == "drl_vo_policy"
            and row["device"] == "cuda"
            and row["model_parameters"] > 0
            and row["input_stamp_ns"] in common_scan_stamps
            for row in successful
        )
        and finite_metrics,
        "actuation_decision_contract": len(inference_decisions) >= 10
        and all(row["inference_sequence_id"] in metric_ids for row in inference_decisions)
        and finite_decisions,
        "shadow_command_only": bool(node.commands)
        and all(
            all(math.isfinite(value) for value in row["value"])
            and -1.0e-6 <= row["value"][0] <= 0.300001
            and abs(row["value"][1]) <= 1.500001
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "8.1",
        "component": "DR-SPAAM tracked base DRL-VO shadow inference",
        "scan_counts": [len(rows) for rows in node.scans],
        "scan_rates_hz": [rate(rows) for rows in node.scans],
        "common_scan_stamps": len(common_scan_stamps),
        "odom_count": len(node.odom),
        "local_goal_count": len(node.local_goals),
        "final_goal_count": len(node.final_goals),
        "track_frame_count": len(node.tracks),
        "nonempty_track_frames": len(nonempty_tracks),
        "confirmed_track_samples": len(confirmed_tracks),
        "training_state_count": len(node.states),
        "observation_size": node.states[-1]["observation_size"] if node.states else None,
        "successful_inference_count": len(successful),
        "inference_rate_hz": rate(successful),
        "inference_decision_count": len(inference_decisions),
        "shadow_command_count": len(node.commands),
        "checks": checks,
        "metric_sample": successful[-1] if successful else None,
        "decision_sample": inference_decisions[-1] if inference_decisions else None,
    }
    print("GATE81_DRLVO_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
