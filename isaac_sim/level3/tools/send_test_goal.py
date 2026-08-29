#!/usr/bin/env python3
"""Send one NavigateToPose goal and record the standalone Level 3 result."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
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
    parser.add_argument("x", type=float, help="goal X in map frame, metres")
    parser.add_argument("y", type=float, help="goal Y in map frame, metres")
    parser.add_argument("yaw", type=float, help="goal yaw in map frame, radians")
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--require-detour",
        action="store_true",
        help="Require the published global plan to be at least 2% longer than its chord.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        raise ValueError("goal must contain finite numbers")
    rclpy.init()
    node = MotionMonitor("level3_test_goal_client")
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    try:
        initial_odom = wait_for_odom(node)
        initial_map = node.map_pose(initial_odom)
        command_count_before_goal = node.command_count
        node.reset_goal_evidence(initial_odom)
        if not client.wait_for_server(timeout_sec=15.0):
            raise TimeoutError("/navigate_to_pose action server is unavailable")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quaternion_from_yaw(
            args.yaw
        )
        feedback_samples = []

        def feedback_callback(message: object) -> None:
            feedback = message.feedback
            feedback_samples.append(
                {
                    "distance_remaining_m": float(feedback.distance_remaining),
                    "recoveries": int(feedback.number_of_recoveries),
                }
            )

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
        if not send_future.done() or send_future.result() is None:
            raise TimeoutError("NavigateToPose goal request timed out")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.wall_timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
            raise TimeoutError("NavigateToPose exceeded wall-time safety timeout")
        action_result = result_future.result()
        odom_count_at_result = node.odom_count
        final_zero = wait_for_final_zero(
            node,
            minimum_command_count=command_count_before_goal,
            after_odom_count=odom_count_at_result,
        )
        final_odom = wait_for_odom(node, wall_timeout_sec=1.0)
        final_map = node.map_pose(final_odom)
        displacement = math.hypot(
            final_odom.x - initial_odom.x, final_odom.y - initial_odom.y
        )
        position_error = math.hypot(final_map.x - args.x, final_map.y - args.y)
        yaw_error = abs(normalize_angle(final_map.yaw - args.yaw))
        plan_points = 0
        plan_length = 0.0
        plan_straight_distance = 0.0
        plan_detour_ratio = 0.0
        accepted_plan_count = 0
        for plan in node.plans:
            poses = plan.poses
            if plan.header.frame_id != "map" or len(poses) < 2:
                continue
            # Ignore unrelated/stale plans: the last point must describe this
            # goal within the configured Nav2 position tolerance.
            if math.hypot(
                poses[-1].pose.position.x - args.x,
                poses[-1].pose.position.y - args.y,
            ) > 0.30:
                continue
            if math.hypot(
                poses[0].pose.position.x - initial_map.x,
                poses[0].pose.position.y - initial_map.y,
            ) > 0.50:
                continue
            accepted_plan_count += 1
            candidate_length = sum(
                math.hypot(
                    poses[index].pose.position.x
                    - poses[index - 1].pose.position.x,
                    poses[index].pose.position.y
                    - poses[index - 1].pose.position.y,
                )
                for index in range(1, len(poses))
            )
            candidate_chord = math.hypot(
                poses[-1].pose.position.x - poses[0].pose.position.x,
                poses[-1].pose.position.y - poses[0].pose.position.y,
            )
            candidate_ratio = (
                candidate_length / candidate_chord
                if candidate_chord > 1.0e-6
                else 0.0
            )
            if candidate_ratio > plan_detour_ratio:
                plan_points = len(poses)
                plan_length = candidate_length
                plan_straight_distance = candidate_chord
                plan_detour_ratio = candidate_ratio
        succeeded = action_result.status == GoalStatus.STATUS_SUCCEEDED
        passed = (
            succeeded
            and displacement >= 0.10
            and position_error <= 0.30
            and yaw_error <= 0.35
            and final_zero
            and (
                not args.require_detour
                or (plan_points >= 2 and plan_detour_ratio >= 1.02)
            )
        )
        output = (args.output or default_report_path("navigate_to_pose")).resolve()
        write_report(
            output,
            {
                "schema": "a_pipeline_level3_goal_result/v1",
                "status": "PASS" if passed else "FAIL",
                "action_status": int(action_result.status),
                "action_succeeded": succeeded,
                "goal_map": {"x_m": args.x, "y_m": args.y, "yaw_rad": args.yaw},
                "initial_odom": vars(initial_odom),
                "final_odom": vars(final_odom),
                "final_map": vars(final_map),
                "odom_displacement_m": displacement,
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "final_cmd_vel": list(node.last_command),
                "final_odom_twist": list(node.last_odom_twist),
                "final_velocity_zero": final_zero,
                "command_count": node.command_count,
                "global_plan_points": plan_points,
                "global_plan_length_m": plan_length,
                "global_plan_chord_m": plan_straight_distance,
                "global_plan_detour_ratio": plan_detour_ratio,
                "matching_global_plan_count": accepted_plan_count,
                "detour_required": args.require_detour,
                "feedback_tail": feedback_samples[-20:],
                "collision_check": "PENDING_ISAAC_RESULT_LOG",
            },
        )
        print(f"LEVEL3_NAVIGATE_TO_POSE={'PASS' if passed else 'FAIL'}")
        print(f"RESULT_FILE={output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
