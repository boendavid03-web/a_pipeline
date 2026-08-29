#!/usr/bin/env python3
"""Submit a lateral FollowPath action to prove MPPI Omni linear.y output."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as PathMessage
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from runtime_common import (
    MotionMonitor,
    default_report_path,
    normalize_angle,
    quaternion_from_yaw,
    wait_for_final_zero,
    wait_for_odom,
    write_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, default=0.80)
    parser.add_argument("--path-samples", type=int, default=17)
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.75 <= args.distance <= 1.0 or args.path_samples < 5:
        raise ValueError("distance must be 0.75..1.0 m and path-samples at least 5")
    rclpy.init()
    node = MotionMonitor("level3_omni_follow_path_client")
    client = ActionClient(node, FollowPath, "/follow_path")
    try:
        initial_odom = wait_for_odom(node)
        command_count_before_goal = node.command_count
        node.reset_goal_evidence(initial_odom)
        initial_map = node.map_pose(initial_odom)
        if not client.wait_for_server(timeout_sec=15.0):
            raise TimeoutError("/follow_path action server is unavailable")
        path = PathMessage()
        path.header.frame_id = "map"
        path.header.stamp = node.get_clock().now().to_msg()
        quaternion_z, quaternion_w = quaternion_from_yaw(initial_map.yaw)
        for distance in [
            args.distance * index / (args.path_samples - 1)
            for index in range(args.path_samples)
        ]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = initial_map.x - math.sin(initial_map.yaw) * distance
            pose.pose.position.y = initial_map.y + math.cos(initial_map.yaw) * distance
            pose.pose.orientation.z = quaternion_z
            pose.pose.orientation.w = quaternion_w
            path.poses.append(pose)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
        if not send_future.done() or send_future.result() is None:
            raise TimeoutError("FollowPath goal request timed out")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("FollowPath goal was rejected")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.wall_timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
            raise TimeoutError("FollowPath exceeded wall-time safety timeout")
        action_result = result_future.result()
        odom_count_at_result = node.odom_count
        final_zero = wait_for_final_zero(
            node,
            minimum_command_count=command_count_before_goal,
            after_odom_count=odom_count_at_result,
        )
        final_odom = wait_for_odom(node, wall_timeout_sec=1.0)
        delta_x = final_odom.x - initial_odom.x
        delta_y = final_odom.y - initial_odom.y
        lateral_displacement = (
            -math.sin(initial_odom.yaw) * delta_x
            + math.cos(initial_odom.yaw) * delta_y
        )
        yaw_change = abs(normalize_angle(final_odom.yaw - initial_odom.yaw))
        maximum_yaw_excursion = max(
            (
                abs(normalize_angle(pose.yaw - initial_odom.yaw))
                for pose in node.odom_history
            ),
            default=math.inf,
        )
        succeeded = action_result.status == GoalStatus.STATUS_SUCCEEDED
        passed = (
            succeeded
            and node.maximum_abs_vy > 0.05
            and lateral_displacement >= 0.50
            and yaw_change <= 0.25
            and maximum_yaw_excursion <= 0.25
            and final_zero
        )
        output = (args.output or default_report_path("omni_follow_path")).resolve()
        write_report(
            output,
            {
                "schema": "a_pipeline_level3_omni_result/v1",
                "status": "PASS" if passed else "FAIL",
                "action_status": int(action_result.status),
                "action_succeeded": succeeded,
                "requested_lateral_distance_m": args.distance,
                "initial_odom": vars(initial_odom),
                "final_odom": vars(final_odom),
                "lateral_displacement_m": lateral_displacement,
                "yaw_change_rad": yaw_change,
                "maximum_yaw_excursion_rad": maximum_yaw_excursion,
                "maximum_abs_linear_y_mps": node.maximum_abs_vy,
                "final_cmd_vel": list(node.last_command),
                "final_odom_twist": list(node.last_odom_twist),
                "final_velocity_zero": final_zero,
                "command_count": node.command_count,
                "collision_check": "PENDING_ISAAC_RESULT_LOG",
            },
        )
        print(f"LEVEL3_OMNI_FOLLOW_PATH={'PASS' if passed else 'FAIL'}")
        print(f"RESULT_FILE={output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
