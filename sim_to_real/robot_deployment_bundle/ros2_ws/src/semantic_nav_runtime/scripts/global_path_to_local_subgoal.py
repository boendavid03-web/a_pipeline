#!/usr/bin/env python3
"""Convert a robot-supplied global path into fresh local and final goals."""
from __future__ import annotations

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from path_subgoal_core import choose_lookahead, map_to_base


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_xy_yaw(transform, xy, yaw=0.0):
    """Apply a planar TransformStamped to XY and return transformed yaw."""
    t, q = transform.transform.translation, transform.transform.rotation
    transform_yaw = yaw_from_quaternion(q)
    cosine, sine = math.cos(transform_yaw), math.sin(transform_yaw)
    return np.array([
        cosine * float(xy[0]) - sine * float(xy[1]) + float(t.x),
        sine * float(xy[0]) + cosine * float(xy[1]) + float(t.y),
    ]), float(yaw) + transform_yaw


class GlobalPathToLocalSubgoal(Node):
    def __init__(self):
        super().__init__('global_path_to_local_subgoal')
        self.path_topic = str(self.declare_parameter('path_topic', '/plan').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.goal_topic = str(self.declare_parameter('goal_topic', '/goal_pose').value)
        self.local_topic = str(self.declare_parameter('local_subgoal_topic', '/semantic_cnn/local_subgoal').value)
        self.final_topic = str(self.declare_parameter('final_goal_topic', '/semantic_cnn/final_goal').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.lookahead = float(self.declare_parameter('lookahead_distance', 1.2).value)
        self.rate = float(self.declare_parameter('publish_rate', 10.0).value)
        self.path_timeout = float(self.declare_parameter('path_timeout', 1.0).value)
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 0.3).value)
        if self.rate <= 0 or self.path_timeout <= 0 or self.odom_timeout <= 0 or self.lookahead < 0:
            raise ValueError('publish rate/timeouts must be positive and lookahead non-negative')
        self.path_xy = None; self.path_frame = None; self.path_stamp = None
        self.odom = None; self.odom_stamp = None; self.goal = None
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_pub = self.create_publisher(PointStamped, self.local_topic, 10)
        self.final_pub = self.create_publisher(PointStamped, self.final_topic, 10)
        self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        self.create_timer(1.0 / self.rate, self.publish)

    def on_path(self, message):
        try:
            points = np.asarray([[p.pose.position.x, p.pose.position.y] for p in message.poses], dtype=np.float64)
            if not len(points): raise ValueError('path is empty')
            if not np.isfinite(points).all(): raise ValueError('path contains NaN or Inf')
            frame = message.header.frame_id or self.map_frame
            if not frame: raise ValueError('path frame_id is empty')
        except ValueError as exc:
            self.path_xy = None; self.get_logger().error(f'invalid global path: {exc}'); return
        self.path_xy, self.path_frame = points, frame
        self.path_stamp = stamp_ns(message.header.stamp) or self.get_clock().now().nanoseconds

    def on_odom(self, message):
        position, q = message.pose.pose.position, message.pose.pose.orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(value)) for value in values):
            self.odom = None; self.get_logger().error('invalid odometry: NaN or Inf'); return
        self.odom = (float(position.x), float(position.y), yaw_from_quaternion(q), message.header.frame_id or self.odom_frame)
        self.odom_stamp = stamp_ns(message.header.stamp) or self.get_clock().now().nanoseconds

    def on_goal(self, message):
        if all(math.isfinite(float(v)) for v in (message.pose.position.x, message.pose.position.y)):
            self.goal = message
        else: self.get_logger().error('invalid final goal: NaN or Inf')

    def publish(self):
        now = self.get_clock().now().nanoseconds
        if self.path_xy is None: self.get_logger().warning('not publishing local subgoal: no valid path', throttle_duration_sec=2.0); return
        if now - self.path_stamp > self.path_timeout * 1e9: self.get_logger().warning('not publishing local subgoal: path timeout', throttle_duration_sec=2.0); return
        if self.odom is None or now - self.odom_stamp > self.odom_timeout * 1e9: self.get_logger().warning('not publishing local subgoal: odometry timeout', throttle_duration_sec=2.0); return
        try:
            odom_xy = np.array(self.odom[:2])
            odom_yaw = self.odom[2]
            if self.path_frame != self.odom[3]:
                odom_to_path = self.tf_buffer.lookup_transform(
                    self.path_frame, self.odom[3], rclpy.time.Time())
                odom_xy, odom_yaw = transform_xy_yaw(odom_to_path, odom_xy, odom_yaw)
            _, ahead_path = choose_lookahead(self.path_xy, odom_xy, self.lookahead)
            path_to_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.path_frame, rclpy.time.Time())
            ahead_base, _ = transform_xy_yaw(path_to_base, ahead_path)
            local = PointStamped(); local.header.stamp = self.get_clock().now().to_msg(); local.header.frame_id = self.base_frame
            local.point.x, local.point.y = float(ahead_base[0]), float(ahead_base[1]); self.local_pub.publish(local)
            if self.goal is not None:
                goal_xy = np.array([self.goal.pose.position.x, self.goal.pose.position.y])
                goal_frame = self.goal.header.frame_id or self.map_frame
                if goal_frame != self.map_frame:
                    goal_to_map = self.tf_buffer.lookup_transform(self.map_frame, goal_frame, rclpy.time.Time())
                    goal_xy, _ = transform_xy_yaw(goal_to_map, goal_xy)
                final = PointStamped(); final.header.stamp = local.header.stamp; final.header.frame_id = self.map_frame
                final.point.x, final.point.y = float(goal_xy[0]), float(goal_xy[1]); self.final_pub.publish(final)
        except (ValueError, TransformException) as exc:
            self.get_logger().warning(f'not publishing local subgoal: TF/geometry failure: {exc}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args); node = GlobalPathToLocalSubgoal()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
