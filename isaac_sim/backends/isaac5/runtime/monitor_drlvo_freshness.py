#!/usr/bin/env python3
"""Verify DRL-VO stops on stale tracks and recovers on fresh tracks."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate82_drlvo_freshness_monitor")
        self.phase = None
        self.transitions = []
        self.states = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        self.create_subscription(String, "/isaac5/gate82/dropout_state", self.on_state, 20)
        self.create_subscription(InferenceMetrics, "/navigation_evaluation/inference_metrics", self.on_metrics, 20)
        self.create_subscription(ActuationDecision, "/drl_vo/actuation_decision", self.on_decision, 30)
        self.create_subscription(Twist, "/isaac5/gate82/drl_vo_cmd_shadow", self.on_command, 20)

    def on_state(self, message: String) -> None:
        try:
            row = json.loads(message.data)
        except json.JSONDecodeError:
            return
        row["receive_wall"] = time.monotonic()
        self.states.append(row)
        if row.get("phase") != self.phase:
            self.phase = row.get("phase")
            self.transitions.append(self.phase)

    def on_metrics(self, message: InferenceMetrics) -> None:
        self.metrics.append({
            "phase": self.phase,
            "success": bool(message.success),
            "sequence_id": int(message.sequence_id),
            "policy_ms": float(message.policy_ms),
            "action": [float(value) for value in message.action],
        })

    def on_decision(self, message: ActuationDecision) -> None:
        self.decisions.append({
            "phase": self.phase,
            "has_raw_action": bool(message.has_raw_action),
            "inference_sequence_id": int(message.inference_sequence_id),
            "reasons": list(message.gate_reasons),
            "final": [float(message.final_command.linear.x), float(message.final_command.angular.z)],
        })

    def on_command(self, message: Twist) -> None:
        self.commands.append({
            "phase": self.phase,
            "value": [float(message.linear.x), float(message.angular.z)],
        })


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

    successful = [row for row in node.metrics if row["success"]]
    normal = [row for row in successful if row["phase"] == "normal"]
    stale = [row for row in successful if row["phase"] == "stale"]
    recovery = [row for row in successful if row["phase"] == "recovery"]
    stale_stops = [
        row for row in node.decisions
        if row["phase"] == "stale"
        and not row["has_raw_action"]
        and any(reason in ("input_missing_or_stale", "watchdog_stale_input", "causal_pedestrian_tracks_missing_or_stale") for reason in row["reasons"])
    ]
    stale_zero = [
        row for row in stale_stops
        if all(math.isclose(value, 0.0, abs_tol=1e-9) for value in row["final"])
    ]
    normal_states = [row for row in node.states if row.get("phase") == "normal"]
    grace_states = [row for row in node.states if row.get("phase") == "dropout_grace"]
    stale_states = [row for row in node.states if row.get("phase") == "stale"]
    recovery_states = [row for row in node.states if row.get("phase") == "recovery"]
    checks = {
        "bounded_dropout_sequence": node.transitions[:4] == ["normal", "dropout_grace", "stale", "recovery"],
        "relay_forwarding_contract": bool(normal_states) and bool(grace_states) and bool(stale_states) and bool(recovery_states)
        and all(row["forwarded"] for row in normal_states + recovery_states)
        and all(not row["forwarded"] for row in grace_states + stale_states),
        "normal_inference_before_dropout": len(normal) >= 10,
        "stale_tracks_block_inference": len(stale) == 0 and len(stale_stops) >= 10,
        "stale_tracks_force_zero": bool(stale_stops) and len(stale_zero) == len(stale_stops),
        "fresh_tracks_recover_inference": len(recovery) >= 10,
        "finite_policy_and_shadow": all(
            math.isfinite(row["policy_ms"]) and len(row["action"]) == 2
            and all(math.isfinite(value) for value in row["action"])
            for row in normal + recovery
        ) and all(
            all(math.isfinite(value) for value in row["value"])
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "8.2",
        "component": "DRL-VO pedestrian-track receive-time freshness",
        "phase_order": node.transitions,
        "normal_successful_inferences": len(normal),
        "stale_successful_inferences": len(stale),
        "stale_stop_decisions": len(stale_stops),
        "stale_zero_decisions": len(stale_zero),
        "recovery_successful_inferences": len(recovery),
        "shadow_command_count": len(node.commands),
        "checks": checks,
        "stale_sample": stale_stops[-1] if stale_stops else None,
        "recovery_sample": recovery[-1] if recovery else None,
    }
    print("GATE82_DRLVO_FRESHNESS_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
