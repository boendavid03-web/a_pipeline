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
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


EVENT_SCHEMA = "semantic_nav_episode_event/v1"


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
        self.declare_parameter("use_sim_time", True)

        goals_file = str(self.get_parameter("goals_file").value)
        if not goals_file:
            raise ValueError("goals_file is required when fixed goal mode is enabled")
        self.frame_id, self.goals = load_goal_sequence(goals_file)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.inter_goal_delay_sec = float(
            self.get_parameter("inter_goal_delay_sec").value
        )
        if not math.isfinite(self.start_delay_sec) or self.start_delay_sec < 0.0:
            raise ValueError("start_delay_sec must be a non-negative finite number")
        if not math.isfinite(self.inter_goal_delay_sec) or self.inter_goal_delay_sec < 0.0:
            raise ValueError(
                "inter_goal_delay_sec must be a non-negative finite number"
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
        self.goal_index = -1
        self.waiting_for_arrival = False
        self.finished = False
        self.next_publish_at_ns = self.now_ns() + int(self.start_delay_sec * 1e9)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            f"FIXED_GOAL_SUITE_READY goals={len(self.goals)} "
            f"frame={self.frame_id} file={goals_file}"
        )

    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

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
        if self.finished or self.waiting_for_arrival or self.goal_index >= len(self.goals) - 1:
            return
        if self.now_ns() >= self.next_publish_at_ns:
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
            self.get_logger().info(
                f"FIXED_GOAL_SUITE_FINISHED goals={len(self.goals)}"
            )
            return
        self.next_publish_at_ns = self.now_ns() + int(self.inter_goal_delay_sec * 1e9)
        self.get_logger().info(
            f"FIXED_GOAL_REACHED index={self.goal_index + 1}/{len(self.goals)}; "
            f"next_goal_in_sec={self.inter_goal_delay_sec:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = FixedGoalSequence()
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
