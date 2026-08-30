#!/usr/bin/env python3
"""Publish a reproducible sequence of navigation goals.

The node is deliberately opt-in from the launch files. With fixed_test=false
the existing manual goal picker is unchanged. In fixed mode, the first goal
is published after startup and each later goal is published only after the
arrival detector emits its ready event.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from semantic_nav_gazebo.msg import PedestrianStateArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


EVENT_SCHEMA = "semantic_nav_episode_event/v1"
REQUIRED_PREFLIGHT_INPUTS = (
    "clock",
    "odom",
    "scan_01",
    "scan_02",
    "pedestrian_ground_truth",
)


def missing_preflight_inputs(seen_inputs: set[str]) -> tuple[str, ...]:
    """Return required simulator inputs that have not produced real data."""
    return tuple(name for name in REQUIRED_PREFLIGHT_INPUTS if name not in seen_inputs)


def clock_has_advanced(previous_ns: int | None, current_ns: int) -> bool:
    """A single /clock sample is insufficient; require positive progression."""
    return previous_ns is not None and current_ns > previous_ns


def load_goal_sequence(path: str | Path) -> tuple[str, list[tuple[str, float, float]]]:
    """Load and validate a fixed goal suite for use by the ROS node/tests."""
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f"fixed goals file does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if payload.get("schema") != "fixed_navigation_goal_suite/v1":
        raise ValueError("fixed goals file has an unsupported schema")
    frame_id = str(payload.get("frame_id", "map")).lstrip("/") or "map"
    raw_goals = payload.get("goals")
    if not isinstance(raw_goals, list) or not raw_goals:
        raise ValueError("fixed goals file must contain a non-empty goals list")

    goals: list[tuple[str, float, float]] = []
    seen_ids: set[str] = set()
    for index, raw_goal in enumerate(raw_goals, start=1):
        if not isinstance(raw_goal, dict):
            raise ValueError(f"goal {index} must be a mapping")
        goal_id = str(raw_goal.get("id", f"goal_{index:02d}"))
        if not goal_id or goal_id in seen_ids:
            raise ValueError(f"goal {index} has a duplicate/empty id")
        try:
            x = float(raw_goal["x"])
            y = float(raw_goal["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"goal {goal_id} must provide numeric x and y") from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"goal {goal_id} coordinates must be finite")
        seen_ids.add(goal_id)
        goals.append((goal_id, x, y))
    return frame_id, goals


class FixedGoalSequence(Node):
    def __init__(self) -> None:
        super().__init__("fixed_goal_sequence")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("episode_event_topic", "/data_collection/episode_event")
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("inter_goal_delay_sec", 1.0)
        self.declare_parameter("readiness_timeout_sec", 60.0)
        self.declare_parameter("auto_shutdown_delay_sec", 2.0)
        self.declare_parameter("clock_topic", "/clock")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_01_topic", "/scan_01")
        self.declare_parameter("scan_02_topic", "/scan_02")
        self.declare_parameter(
            "pedestrian_ground_truth_topic", "/pedestrian_ground_truth"
        )

        goals_file = str(self.get_parameter("goals_file").value)
        if not goals_file:
            raise ValueError("goals_file is required when fixed goal mode is enabled")
        self.frame_id, self.goals = load_goal_sequence(goals_file)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.inter_goal_delay_sec = float(
            self.get_parameter("inter_goal_delay_sec").value
        )
        self.readiness_timeout_sec = float(
            self.get_parameter("readiness_timeout_sec").value
        )
        self.auto_shutdown_delay_sec = float(
            self.get_parameter("auto_shutdown_delay_sec").value
        )
        if not math.isfinite(self.start_delay_sec) or self.start_delay_sec < 0.0:
            raise ValueError("start_delay_sec must be a non-negative finite number")
        if not math.isfinite(self.inter_goal_delay_sec) or self.inter_goal_delay_sec < 0.0:
            raise ValueError(
                "inter_goal_delay_sec must be a non-negative finite number"
            )
        if (
            not math.isfinite(self.readiness_timeout_sec)
            or self.readiness_timeout_sec <= 0.0
        ):
            raise ValueError("readiness_timeout_sec must be a positive finite number")
        if (
            not math.isfinite(self.auto_shutdown_delay_sec)
            or self.auto_shutdown_delay_sec < 0.0
        ):
            raise ValueError(
                "auto_shutdown_delay_sec must be a non-negative finite number"
            )

        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("episode_event_topic").value),
            self.event_callback,
            10,
        )
        self.create_subscription(
            ClockMessage,
            str(self.get_parameter("clock_topic").value),
            self.clock_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            lambda _message: self.mark_preflight_input("odom"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_01_topic").value),
            lambda _message: self.mark_preflight_input("scan_01"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_02_topic").value),
            lambda _message: self.mark_preflight_input("scan_02"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PedestrianStateArray,
            str(self.get_parameter("pedestrian_ground_truth_topic").value),
            lambda _message: self.mark_preflight_input("pedestrian_ground_truth"),
            qos_profile_sensor_data,
        )
        self.goal_index = -1
        self.waiting_for_arrival = False
        self.finished = False
        self.seen_preflight_inputs: set[str] = set()
        self.last_clock_ns: int | None = None
        self.preflight_ready = False
        self.preflight_started_at = time.monotonic()
        self.next_publish_at_ns: int | None = None
        self.shutdown_at_monotonic: float | None = None
        self.shutdown_requested = False
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.timer_callback, clock=self.wall_clock)
        self.get_logger().info(
            f"FIXED_GOAL_SUITE_READY goals={len(self.goals)} "
            f"frame={self.frame_id} file={goals_file}"
        )

    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def clock_callback(self, message: ClockMessage) -> None:
        current_ns = int(message.clock.sec) * 1_000_000_000 + int(
            message.clock.nanosec
        )
        if clock_has_advanced(self.last_clock_ns, current_ns):
            self.mark_preflight_input("clock")
        self.last_clock_ns = current_ns

    def mark_preflight_input(self, name: str) -> None:
        if self.preflight_ready or name in self.seen_preflight_inputs:
            return
        self.seen_preflight_inputs.add(name)
        if missing_preflight_inputs(self.seen_preflight_inputs):
            return
        self.preflight_ready = True
        self.next_publish_at_ns = self.now_ns() + int(self.start_delay_sec * 1e9)
        self.get_logger().info(
            "FIXED_GOAL_PREFLIGHT_PASS "
            "topics=/clock,/odom,/scan_01,/scan_02,/pedestrian_ground_truth"
        )

    def publish_goal(self) -> None:
        self.goal_index += 1
        goal_id, x, y = self.goals[self.goal_index]
        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.waiting_for_arrival = True
        self.get_logger().info(
            f"FIXED_GOAL_PUBLISHED index={self.goal_index + 1}/{len(self.goals)} "
            f"id={goal_id} x={x:.6f} y={y:.6f}"
        )

    def timer_callback(self) -> None:
        if not self.preflight_ready:
            elapsed = time.monotonic() - self.preflight_started_at
            if elapsed >= self.readiness_timeout_sec:
                missing = ",".join(
                    missing_preflight_inputs(self.seen_preflight_inputs)
                )
                self.finished = True
                failure = (
                    f"FIXED_GOAL_PREFLIGHT_FAILED missing={missing} "
                    f"timeout_sec={self.readiness_timeout_sec:.3f}"
                )
                self.get_logger().error(failure)
                raise RuntimeError(failure)
            return
        if self.finished:
            if (
                self.shutdown_at_monotonic is not None
                and not self.shutdown_requested
                and time.monotonic() >= self.shutdown_at_monotonic
            ):
                self.shutdown_requested = True
                self.get_logger().info("FIXED_GOAL_SUITE_AUTO_SHUTDOWN")
                threading.Thread(
                    target=self.shutdown_context,
                    name="fixed-goal-auto-shutdown",
                    daemon=True,
                ).start()
            return
        if self.waiting_for_arrival or self.goal_index >= len(self.goals) - 1:
            return
        if self.next_publish_at_ns is not None and self.now_ns() >= self.next_publish_at_ns:
            self.publish_goal()

    def event_callback(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if event.get("schema") != EVENT_SCHEMA or event.get("event") != "ready":
            return
        if not self.waiting_for_arrival:
            return
        self.waiting_for_arrival = False
        if self.goal_index >= len(self.goals) - 1:
            self.finished = True
            self.shutdown_at_monotonic = (
                time.monotonic() + self.auto_shutdown_delay_sec
            )
            self.get_logger().info(
                f"FIXED_GOAL_SUITE_FINISHED goals={len(self.goals)} "
                f"auto_shutdown_in_sec={self.auto_shutdown_delay_sec:.3f}"
            )
            return
        self.next_publish_at_ns = self.now_ns() + int(self.inter_goal_delay_sec * 1e9)
        self.get_logger().info(
            f"FIXED_GOAL_REACHED index={self.goal_index + 1}/{len(self.goals)}; "
            f"next_goal_in_sec={self.inter_goal_delay_sec:.3f}"
        )

    @staticmethod
    def shutdown_context() -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = FixedGoalSequence()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
