#!/usr/bin/env python3
"""External ROS evidence monitor for Gate 7.3 S3-Net plus SemanticCNN."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate73_semantic_monitor")
        self.scan_stamps = [set(), set()]
        self.scan_rows = [[], []]
        self.labels = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        for sensor, topic in enumerate(("/sim_to_real/aligned/scan_01", "/sim_to_real/aligned/scan_02")):
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, index=sensor: self.on_scan(index, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(Image, "/s3net/labels", self.on_labels, 10)
        self.create_subscription(InferenceMetrics, "/navigation_evaluation/inference_metrics", self.on_metrics, 20)
        self.create_subscription(ActuationDecision, "/semantic_cnn/actuation_decision", self.on_decision, 30)
        self.create_subscription(Twist, "/isaac5/gate73/semantic_cmd_shadow", self.on_command, 20)

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        stamp = stamp_ns(message.header.stamp)
        self.scan_stamps[sensor].add(stamp)
        self.scan_rows[sensor].append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp,
                "beams": len(message.ranges),
                "frame_id": message.header.frame_id,
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
            }
        )

    def on_labels(self, message: Image) -> None:
        self.labels.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "height": int(message.height),
                "width": int(message.width),
                "encoding": message.encoding,
                "bytes": len(message.data),
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
                "policy_ms": float(message.policy_ms),
                "total_ms": float(message.total_ms),
                "device": message.device,
                "action": list(message.action),
            }
        )

    def on_decision(self, message: ActuationDecision) -> None:
        self.decisions.append(
            {
                "receive_wall": time.monotonic(),
                "input_stamp_ns": stamp_ns(message.input_stamp),
                "decision_sequence_id": int(message.decision_sequence_id),
                "inference_sequence_id": int(message.inference_sequence_id),
                "has_raw_action": bool(message.has_raw_action),
                "gated": bool(message.gated),
                "reasons": list(message.gate_reasons),
                "raw": [
                    float(message.raw_physical_action.linear.x),
                    float(message.raw_physical_action.angular.z),
                ],
                "final": [float(message.final_command.linear.x), float(message.final_command.angular.z)],
            }
        )

    def on_command(self, message: Twist) -> None:
        self.commands.append(
            {
                "receive_wall": time.monotonic(),
                "command": [float(message.linear.x), float(message.angular.z)],
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
    if common_scan_stamps:
        first_common_stamp = min(common_scan_stamps)
        last_common_stamp = max(common_scan_stamps)
        labels_in_scan_window = [
            row
            for row in node.labels
            if first_common_stamp <= row["stamp_ns"] <= last_common_stamp
        ]
    else:
        labels_in_scan_window = []
    label_matches = sum(
        row["stamp_ns"] in common_scan_stamps
        for row in labels_in_scan_window
    )
    metric_matches = sum(row["input_stamp_ns"] in common_scan_stamps for row in node.metrics)
    inference_decisions = [
        row
        for row in node.decisions
        if row["has_raw_action"] and row["inference_sequence_id"] > 0
    ]
    decision_matches = sum(
        row["input_stamp_ns"] in common_scan_stamps
        for row in inference_decisions
    )
    finite_metrics = all(
        math.isfinite(row["policy_ms"])
        and math.isfinite(row["total_ms"])
        and all(math.isfinite(float(value)) for value in row["action"])
        for row in node.metrics
    )
    finite_commands = all(
        all(math.isfinite(value) for value in row["command"])
        for row in node.commands
    )
    successful = [row for row in node.metrics if row["success"]]
    checks = {
        "aligned_dual_scans": all(len(rows) >= 20 for rows in node.scan_rows)
        and all(
            row["beams"] == 2000 and row["range_min"] <= 0.1001 and 7.99 <= row["range_max"] <= 8.01
            for rows in node.scan_rows
            for row in rows
        ),
        "s3net_labels_observed": len(node.labels) >= 5,
        "s3net_label_shape": bool(node.labels)
        and all(
            row["height"] == 2
            and row["width"] == 2000
            and row["encoding"] == "16SC1"
            and row["bytes"] == 8000
            for row in node.labels
        ),
        # A reliable labels subscription may replay up to its depth of samples
        # when the monitor joins, while the best-effort scan subscriptions do
        # not.  Judge causality only inside the scan interval actually observed
        # by this monitor; every label in that interval must match a dual scan.
        "s3net_labels_causal": bool(labels_in_scan_window)
        and label_matches == len(labels_in_scan_window),
        "semantic_inference_success": bool(successful)
        and len(successful) == len(node.metrics)
        and all(row["producer_id"] == "semantic_cnn_policy" for row in node.metrics)
        and finite_metrics,
        "semantic_input_stamp_causal": bool(node.metrics) and metric_matches == len(node.metrics),
        "actuation_decision_observed": bool(inference_decisions)
        and decision_matches == len(inference_decisions),
        "shadow_command_only": bool(node.commands) and finite_commands,
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "7.3",
        "component": "online S3-Net plus SemanticCNN shadow output",
        "aligned_scan_counts": [len(rows) for rows in node.scan_rows],
        "aligned_scan_rates_hz": [rate(rows) for rows in node.scan_rows],
        "common_scan_stamps": len(common_scan_stamps),
        "s3net_label_count": len(node.labels),
        "s3net_labels_in_scan_window": len(labels_in_scan_window),
        "s3net_label_rate_hz": rate(node.labels),
        "label_stamp_matches": label_matches,
        "semantic_metric_count": len(node.metrics),
        "successful_inference_count": len(successful),
        "semantic_metric_rate_hz": rate(node.metrics),
        "metric_stamp_matches": metric_matches,
        "decision_count": len(node.decisions),
        "inference_decision_count": len(inference_decisions),
        "decision_stamp_matches": decision_matches,
        "shadow_command_count": len(node.commands),
        "checks": checks,
        "metric_sample": successful[-1] if successful else None,
        "decision_sample": inference_decisions[-1] if inference_decisions else None,
    }
    print("GATE73_SEMANTIC_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
