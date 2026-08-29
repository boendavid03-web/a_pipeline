#!/usr/bin/env python3
"""Shared helpers for the deferred standalone Level 3 runtime checks."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter


LEVEL3_ROOT = Path(__file__).resolve().parents[1]
ALIGNMENT_FILE = LEVEL3_ROOT / "config/map_alignment.yaml"
RUNTIME_REPORT_DIR = LEVEL3_ROOT / "reports/runtime"


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def load_alignment() -> dict[str, Any]:
    import yaml

    return yaml.safe_load(ALIGNMENT_FILE.read_text(encoding="utf-8"))


def default_report_path(kind: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return RUNTIME_REPORT_DIR / f"{stamp}_{kind}.json"


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class MotionMonitor(Node):
    def __init__(self, name: str) -> None:
        super().__init__(
            name,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.odom_pose: Pose2D | None = None
        self.last_odom_twist = (math.nan, math.nan, math.nan)
        self.odom_count = 0
        self.last_command = (math.nan, math.nan, math.nan)
        self.maximum_abs_vy = 0.0
        self.command_count = 0
        self.latest_plan: NavPath | None = None
        self.plans: list[NavPath] = []
        self.odom_history: list[Pose2D] = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 20)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.odom_pose = Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(float(pose.orientation.z), float(pose.orientation.w)),
        )
        self.odom_history.append(self.odom_pose)
        twist = message.twist.twist
        self.last_odom_twist = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        self.odom_count += 1

    def _on_command(self, message: Twist) -> None:
        self.last_command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self.maximum_abs_vy = max(self.maximum_abs_vy, abs(self.last_command[1]))
        self.command_count += 1

    def _on_plan(self, message: NavPath) -> None:
        self.latest_plan = message
        self.plans.append(message)

    def map_pose(self, odom_pose: Pose2D) -> Pose2D:
        transform = load_alignment()["map_to_odom"]
        yaw = float(transform["yaw_rad"])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return Pose2D(
            cosine * odom_pose.x - sine * odom_pose.y + float(transform["x_m"]),
            sine * odom_pose.x + cosine * odom_pose.y + float(transform["y_m"]),
            normalize_angle(odom_pose.yaw + yaw),
        )

    def reset_goal_evidence(self, initial_pose: Pose2D) -> None:
        """Discard pre-goal samples so one action cannot inherit old evidence."""
        self.maximum_abs_vy = 0.0
        self.plans.clear()
        self.odom_history = [initial_pose]


def wait_for_odom(node: MotionMonitor, wall_timeout_sec: float = 15.0) -> Pose2D:
    import time

    deadline = time.monotonic() + wall_timeout_sec
    while rclpy.ok() and node.odom_pose is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom_pose is None:
        raise TimeoutError("no /odom message received")
    return node.odom_pose


def wait_for_final_zero(
    node: MotionMonitor,
    wall_timeout_sec: float = 3.0,
    minimum_command_count: int = 0,
    after_odom_count: int | None = None,
) -> bool:
    import time

    deadline = time.monotonic() + wall_timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        received_goal_command = node.command_count > minimum_command_count
        received_fresh_odom = (
            after_odom_count is None or node.odom_count > after_odom_count
        )
        command_stopped = max(abs(value) for value in node.last_command) <= 1.0e-3
        odom_stopped = max(abs(value) for value in node.last_odom_twist) <= 1.0e-2
        if (
            received_goal_command
            and received_fresh_odom
            and command_stopped
            and odom_stopped
        ):
            return True
    return False
