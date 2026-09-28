#!/usr/bin/env python3
"""Publish the fixed VLA-style global goal in the formats used by both policies."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def transform_point(transform, goal_x: float, goal_y: float):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * goal_x - sine * goal_y + float(translation.x),
        sine * goal_x + cosine * goal_y + float(translation.y),
    )


class FixedGlobalGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("fixed_global_goal_bridge")
        self.global_frame = str(
            self.declare_parameter("global_frame", "map").value
        ).lstrip("/")
        self.base_frame = str(
            self.declare_parameter("base_frame", "base_link").value
        ).lstrip("/")
        self.world_frame = str(
            self.declare_parameter("world_frame", "odom").value
        ).lstrip("/")
        self.goal_x = float(self.declare_parameter("goal_x", -0.671).value)
        self.goal_y = float(self.declare_parameter("goal_y", 1.778).value)
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 10.0).value
        )
        self.local_topic = str(
            self.declare_parameter(
                "local_subgoal_topic", "/semantic_cnn/local_subgoal"
            ).value
        )
        self.final_topic = str(
            self.declare_parameter(
                "final_goal_topic", "/semantic_cnn/final_goal"
            ).value
        )
        self.pose_topic = str(
            self.declare_parameter(
                "pose_topic", "/sim_to_real/tf_odom"
            ).value
        )
        if not self.global_frame or not self.base_frame or not self.world_frame:
            raise ValueError("global_frame, world_frame and base_frame cannot be empty")
        if not all(
            math.isfinite(value)
            for value in (self.goal_x, self.goal_y, self.publish_rate)
        ) or self.publish_rate <= 0.0:
            raise ValueError("goal coordinates must be finite and publish_rate positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_publisher = self.create_publisher(
            PointStamped, self.local_topic, 10
        )
        self.final_publisher = self.create_publisher(
            PointStamped, self.final_topic, 10
        )
        self.pose_publisher = self.create_publisher(
            Odometry, self.pose_topic, 10
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_goal)
        self.get_logger().info(
            f"fixed goal=({self.goal_x:.3f}, {self.goal_y:.3f}) "
            f"frame={self.global_frame}; policy_world_frame={self.world_frame}; "
            f"base_frame={self.base_frame}"
        )

    def publish_goal(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.global_frame,
                rclpy.time.Time(),
            )
            local_x, local_y = transform_point(
                transform, self.goal_x, self.goal_y
            )
            pose_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            if self.world_frame == self.global_frame:
                final_x, final_y = self.goal_x, self.goal_y
            else:
                world_transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.global_frame,
                    rclpy.time.Time(),
                )
                final_x, final_y = transform_point(
                    world_transform, self.goal_x, self.goal_y
                )
        except TransformException as error:
            self.get_logger().warning(
                f"not publishing model goal: required TF from "
                f"{self.global_frame} to {self.base_frame}/{self.world_frame} "
                f"failed: {error}",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        pose = Odometry()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.child_frame_id = self.base_frame
        pose.pose.pose.position.x = float(
            pose_transform.transform.translation.x
        )
        pose.pose.pose.position.y = float(
            pose_transform.transform.translation.y
        )
        pose.pose.pose.position.z = float(
            pose_transform.transform.translation.z
        )
        pose.pose.pose.orientation = pose_transform.transform.rotation
        self.pose_publisher.publish(pose)

        local = PointStamped()
        local.header.stamp = stamp
        local.header.frame_id = self.base_frame
        local.point.x = float(local_x)
        local.point.y = float(local_y)
        self.local_publisher.publish(local)

        final = PointStamped()
        final.header.stamp = stamp
        final.header.frame_id = self.world_frame
        final.point.x = float(final_x)
        final.point.y = float(final_y)
        self.final_publisher.publish(final)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedGlobalGoalBridge()
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
