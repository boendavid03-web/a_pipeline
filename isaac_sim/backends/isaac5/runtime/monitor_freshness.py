#!/usr/bin/env python3
"""Observe normal, stale, and recovered SemanticCNN behavior for Gate 7.4."""

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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate74_freshness_monitor")
        self.phase = None
        self.transitions = []
        self.state_rows = []
        self.scan_stamps = [set(), set()]
        self.metrics = []
        self.decisions = []
        self.commands = []
        for sensor, topic in enumerate(
            ("/isaac5/gate74/scan_01", "/isaac5/gate74/scan_02")
        ):
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, index=sensor: self.on_scan(index, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            String, "/isaac5/gate74/fault_state", self.on_state, 20
        )
        self.create_subscription(
            InferenceMetrics,
            "/navigation_evaluation/inference_metrics",
            self.on_metrics,
            20,
        )
        self.create_subscription(
            ActuationDecision,
            "/semantic_cnn/actuation_decision",
            self.on_decision,
            30,
        )
        self.create_subscription(
            Twist,
            "/isaac5/gate74/semantic_cmd_shadow",
            self.on_command,
            20,
        )

    def on_state(self, message: String) -> None:
        try:
            row = json.loads(message.data)
        except json.JSONDecodeError:
            return
        row["receive_wall"] = time.monotonic()
        self.state_rows.append(row)
        if row.get("phase") != self.phase:
            self.phase = row.get("phase")
            self.transitions.append(
                {"phase": self.phase, "receive_wall": row["receive_wall"]}
            )

    def on_scan(self, sensor: int, message: LaserScan) -> None:
        self.scan_stamps[sensor].add(stamp_ns(message.header.stamp))

    def on_metrics(self, message: InferenceMetrics) -> None:
        self.metrics.append(
            {
                "phase": self.phase,
                "receive_wall": time.monotonic(),
                "input_stamp_ns": stamp_ns(message.input_stamp),
                "sequence_id": int(message.sequence_id),
                "producer_id": message.producer_id,
                "success": bool(message.success),
                "policy_ms": float(message.policy_ms),
                "total_ms": float(message.total_ms),
                "action": [float(value) for value in message.action],
            }
        )

    def on_decision(self, message: ActuationDecision) -> None:
        self.decisions.append(
            {
                "phase": self.phase,
                "receive_wall": time.monotonic(),
                "input_stamp_ns": stamp_ns(message.input_stamp),
                "has_raw_action": bool(message.has_raw_action),
                "inference_sequence_id": int(message.inference_sequence_id),
                "reasons": list(message.gate_reasons),
                "final": [
                    float(message.final_command.linear.x),
                    float(message.final_command.angular.z),
                ],
            }
        )

    def on_command(self, message: Twist) -> None:
        self.commands.append(
            {
                "phase": self.phase,
                "receive_wall": time.monotonic(),
                "value": [float(message.linear.x), float(message.angular.z)],
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=11.5)
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

    common_stamps = node.scan_stamps[0] & node.scan_stamps[1]
    normal_metrics = [
        row for row in node.metrics if row["phase"] == "normal" and row["success"]
    ]
    stale_metrics = [
        row for row in node.metrics if row["phase"] == "stale" and row["success"]
    ]
    recovery_metrics = [
        row for row in node.metrics if row["phase"] == "recovery" and row["success"]
    ]
    stale_rejections = [
        row
        for row in node.decisions
        if row["phase"] == "stale"
        and not row["has_raw_action"]
        and "stale_or_unsynchronized_scan" in row["reasons"]
    ]
    stale_zero_decisions = [
        row
        for row in stale_rejections
        if all(math.isclose(value, 0.0, abs_tol=1.0e-9) for value in row["final"])
    ]
    stale_states = [row for row in node.state_rows if row.get("phase") == "stale"]
    phase_order = [row["phase"] for row in node.transitions]
    finite_metrics = all(
        math.isfinite(row["policy_ms"])
        and math.isfinite(row["total_ms"])
        and all(math.isfinite(value) for value in row["action"])
        for row in normal_metrics + recovery_metrics
    )
    checks = {
        "bounded_fault_sequence": phase_order[:3] == ["normal", "stale", "recovery"],
        "one_second_stale_offset": bool(stale_states)
        and all(
            int(row["original_stamp_ns"]) - int(row["output_stamp_ns"])
            == 1_000_000_000
            for row in stale_states
        ),
        "normal_inference_before_fault": len(normal_metrics) >= 10,
        "stale_inputs_blocked": len(stale_metrics) == 0 and len(stale_rejections) >= 10,
        "stale_output_zeroed": bool(stale_rejections)
        and len(stale_zero_decisions) == len(stale_rejections),
        "inference_recovers": len(recovery_metrics) >= 10,
        "successful_inputs_causal": all(
            row["input_stamp_ns"] in common_stamps
            for row in normal_metrics + recovery_metrics
        ),
        "finite_shadow_outputs": finite_metrics
        and all(
            all(math.isfinite(value) for value in row["value"])
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "7.4",
        "component": "SemanticCNN synchronization and causal freshness gate",
        "phase_order": phase_order,
        "fault_state_samples": len(node.state_rows),
        "dual_fault_scan_common_stamps": len(common_stamps),
        "normal_successful_inferences": len(normal_metrics),
        "stale_successful_inferences": len(stale_metrics),
        "stale_rejection_decisions": len(stale_rejections),
        "stale_zero_decisions": len(stale_zero_decisions),
        "recovery_successful_inferences": len(recovery_metrics),
        "shadow_command_count": len(node.commands),
        "checks": checks,
        "stale_rejection_sample": stale_rejections[-1] if stale_rejections else None,
        "recovery_metric_sample": recovery_metrics[-1] if recovery_metrics else None,
    }
    print("GATE74_FRESHNESS_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
