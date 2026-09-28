#!/usr/bin/env python3
"""Verify the DRL-VO near-obstacle linear-velocity veto and recovery."""

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
        super().__init__("isaac5_gate83_drlvo_front_veto_monitor")
        self.phase = None
        self.transitions = []
        self.states = []
        self.metrics = []
        self.decisions = []
        self.commands = []
        self.create_subscription(String, "/isaac5/gate83/obstacle_state", self.on_state, 20)
        self.create_subscription(InferenceMetrics, "/navigation_evaluation/inference_metrics", self.on_metrics, 20)
        self.create_subscription(ActuationDecision, "/drl_vo/actuation_decision", self.on_decision, 30)
        self.create_subscription(Twist, "/isaac5/gate83/drl_vo_cmd_shadow", self.on_command, 20)

    def on_state(self, message: String) -> None:
        try:
            row = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.states.append(row)
        if row.get("phase") != self.phase:
            self.phase = row.get("phase")
            self.transitions.append(self.phase)

    def on_metrics(self, message: InferenceMetrics) -> None:
        self.metrics.append({
            "phase": self.phase,
            "sequence_id": int(message.sequence_id),
            "success": bool(message.success),
            "action": [float(value) for value in message.action],
        })

    def on_decision(self, message: ActuationDecision) -> None:
        self.decisions.append({
            "phase": self.phase,
            "inference_sequence_id": int(message.inference_sequence_id),
            "has_raw_action": bool(message.has_raw_action),
            "reasons": list(message.gate_reasons),
            "raw": [float(message.raw_physical_action.linear.x), float(message.raw_physical_action.angular.z)],
            "final": [float(message.final_command.linear.x), float(message.final_command.angular.z)],
            "front_min": float(message.front_min_range_m) if message.has_front_min_range else None,
        })

    def on_command(self, message: Twist) -> None:
        self.commands.append({"phase": self.phase, "value": [float(message.linear.x), float(message.angular.z)]})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=11.0)
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

    successful_ids = {row["sequence_id"] for row in node.metrics if row["success"]}
    inferred = [
        row for row in node.decisions
        if row["has_raw_action"] and row["inference_sequence_id"] in successful_ids
    ]
    normal = [row for row in inferred if row["phase"] == "normal"]
    veto = [row for row in inferred if row["phase"] == "near_obstacle"]
    recovery = [row for row in inferred if row["phase"] == "recovery"]
    vetoed = [
        row for row in veto
        if "front_stop" in row["reasons"]
        and row["front_min"] is not None and row["front_min"] <= 0.5001
        and row["raw"][0] > 0.0
        and math.isclose(row["final"][0], 0.0, abs_tol=1e-9)
    ]
    recovered_motion = [
        row for row in recovery
        if row["front_min"] is not None and row["front_min"] > 0.5
        and row["final"][0] > 0.0
        and "front_stop" not in row["reasons"]
    ]
    obstacle_states = [row for row in node.states if row.get("phase") == "near_obstacle"]
    checks = {
        "bounded_obstacle_sequence": node.transitions[:3] == ["normal", "near_obstacle", "recovery"],
        "synthetic_range_exact": bool(obstacle_states)
        and all(math.isclose(float(row["obstacle_range_m"]), 0.5, abs_tol=1e-9) for row in obstacle_states),
        "normal_policy_decisions": len(normal) >= 10,
        "policy_continues_during_veto": len(veto) >= 10,
        "front_stop_vetoes_linear_motion": len(vetoed) >= 10 and len(vetoed) == len(veto),
        "motion_recovers_after_clear": len(recovered_motion) >= 10,
        "finite_bounded_shadow": all(
            all(math.isfinite(value) for value in row["value"])
            and -1e-6 <= row["value"][0] <= 0.300001
            and abs(row["value"][1]) <= 1.500001
            for row in node.commands
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "8.3",
        "component": "DRL-VO front-range safety veto",
        "truth_scope": "synthetic LiDAR proximity proxy; not PhysX contact truth",
        "phase_order": node.transitions,
        "normal_inference_decisions": len(normal),
        "near_obstacle_inference_decisions": len(veto),
        "front_stop_veto_decisions": len(vetoed),
        "recovery_inference_decisions": len(recovery),
        "recovered_motion_decisions": len(recovered_motion),
        "shadow_command_count": len(node.commands),
        "checks": checks,
        "veto_sample": vetoed[-1] if vetoed else None,
        "recovery_sample": recovered_motion[-1] if recovered_motion else None,
    }
    print("GATE83_DRLVO_FRONT_VETO_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
