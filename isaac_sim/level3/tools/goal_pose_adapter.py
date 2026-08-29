#!/usr/bin/env python3
"""Forward standard map-frame /goal_pose messages to NavigateToPose."""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


class GoalPoseAdapter(Node):
    def __init__(self) -> None:
        super().__init__(
            "level3_goal_pose_adapter",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.active_result = None
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)

    def _on_goal(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != "map":
            self.get_logger().error(
                f"rejecting /goal_pose frame {pose.header.frame_id!r}; expected 'map'"
            )
            return
        if self.active_result is not None and not self.active_result.done():
            self.get_logger().warning("ignoring /goal_pose while a goal is active")
            return
        if not self.client.server_is_ready():
            self.get_logger().error("NavigateToPose action server is not ready")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if goal.pose.header.stamp.sec == 0 and goal.pose.header.stamp.nanosec == 0:
            goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: object) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("/goal_pose was rejected by NavigateToPose")
            return
        self.get_logger().info("/goal_pose accepted by NavigateToPose")
        self.active_result = handle.get_result_async()
        self.active_result.add_done_callback(self._on_result)

    def _on_result(self, future: object) -> None:
        wrapped = future.result()
        status = wrapped.status if wrapped is not None else -1
        self.get_logger().info(f"/goal_pose NavigateToPose finished with status={status}")


def parse_args() -> argparse.Namespace:
    return argparse.ArgumentParser(description=__doc__).parse_args()


def main() -> None:
    parse_args()
    rclpy.init()
    node = GoalPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # A launch-wide SIGINT is an expected clean shutdown path.
        pass
    finally:
        node.destroy_node()
        # SIGINT may already have shut down the default context through
        # rclpy's signal handler during fail-closed launch teardown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
