#!/usr/bin/env python3
"""
Pure Pursuit ROS 2 Node

This script converts the ROS 1 Pure Pursuit code to ROS 2. It uses rclpy, tf2_ros, and the new
publisher/subscriber/timer APIs. Adjust tf frame names and parameters as needed.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
import numpy as np
import threading
import math
import tf_transformations
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        # Declare and get parameters
        self.declare_parameter('lookahead', 2.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('goal_margin', 0.9)
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 5.0)

        self.lookahead = self.get_parameter('lookahead').value
        self.rate = self.get_parameter('rate').value
        self.goal_margin = self.get_parameter('goal_margin').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value

        # Create tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create subscription to path
        self.subscription = self.create_subscription(
            Path,
            'path',
            self.path_callback,
            10
        )

        # Create publishers for cnn_goal and final_goal
        self.cnn_goal_pub = self.create_publisher(Point, 'cnn_goal', 10)
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 10)

        # Data and thread lock
        self.path = None
        self.lock = threading.Lock()
        self.timer = None

    def path_callback(self, msg: Path):
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg
        if self.timer is None:
            self.start()

    def start(self):
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_current_pose(self):
        try:
            # Using "base_link" here; change to "base_footprint" if needed.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=1.0))
            x = np.array([trans.transform.translation.x, trans.transform.translation.y])
            quat = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]
            (roll, pitch, theta) = tf_transformations.euler_from_quaternion(quat)
            self.get_logger().debug(f"x = {x[0]}, y = {x[1]}, theta = {theta}")
            return x, theta
        except Exception as e:
            self.get_logger().warn('Could not get robot pose: ' + str(e))
            return np.array([np.nan, np.nan]), np.nan

    def find_closest_point(self, x, seg=-1):
        pt_min = np.array([np.nan, np.nan])
        dist_min = np.inf
        seg_min = -1

        if self.path is None:
            self.get_logger().warn('Pure Pursuit: No path received yet')
            return pt_min, dist_min, seg_min

        if seg == -1:
            # iterate over all segments of the path
            for i in range(len(self.path.poses) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min = pt
                    dist_min = dist
                    seg_min = s
        else:
            # get start and end of segment
            p_start = np.array([self.path.poses[seg].pose.position.x,
                                self.path.poses[seg].pose.position.y])
            p_end = np.array([self.path.poses[seg+1].pose.position.x,
                              self.path.poses[seg+1].pose.position.y])
            v = p_end - p_start
            length_seg = np.linalg.norm(v)
            if length_seg == 0:
                return p_start, np.linalg.norm(x - p_start), seg
            v = v / length_seg
            dist_projected = np.dot(x - p_start, v)
            if dist_projected < 0.:
                pt_min = p_start
            elif dist_projected > length_seg:
                pt_min = p_end
            else:
                pt_min = p_start + dist_projected * v
            dist_min = np.linalg.norm(pt_min - x)
            seg_min = seg
        return pt_min, dist_min, seg_min

    def find_goal(self, x, pt, dist, seg):
        goal = None
        if dist > self.lookahead:
            # If robot is farther than lookahead distance from path, drive toward the path
            goal = pt
        else:
            seg_max = len(self.path.poses) - 2
            p_end = np.array([self.path.poses[seg+1].pose.position.x,
                              self.path.poses[seg+1].pose.position.y])
            dist_end = np.linalg.norm(x - p_end)
            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = np.array([self.path.poses[seg+1].pose.position.x,
                                  self.path.poses[seg+1].pose.position.y])
                dist_end = np.linalg.norm(x - p_end)
            if dist_end < self.lookahead:
                pt = np.array([self.path.poses[seg_max+1].pose.position.x,
                               self.path.poses[seg_max+1].pose.position.y])
            else:
                pt, dist, seg = self.find_closest_point(x, seg)
                p_start = np.array([self.path.poses[seg].pose.position.x,
                                    self.path.poses[seg].pose.position.y])
                p_end = np.array([self.path.poses[seg+1].pose.position.x,
                                  self.path.poses[seg+1].pose.position.y])
                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                if length_seg != 0:
                    v = v / length_seg
                    dist_projected_x = np.dot(x - pt, v)
                    dist_projected_y = np.linalg.norm(np.cross(x - pt, v))
                    pt = pt + (np.sqrt(self.lookahead**2 - dist_projected_y**2) + dist_projected_x) * v
            goal = pt

        end_goal_pos = [self.path.poses[-1].pose.position.x,
                        self.path.poses[-1].pose.position.y]
        end_goal_rot = [self.path.poses[-1].pose.orientation.x,
                        self.path.poses[-1].pose.orientation.y,
                        self.path.poses[-1].pose.orientation.z,
                        self.path.poses[-1].pose.orientation.w]
        return goal, end_goal_pos, end_goal_rot

    def timer_callback(self):
        with self.lock:
            x, theta = self.get_current_pose()
            if np.isnan(x[0]):
                return
            pt, dist, seg = self.find_closest_point(x)
            if np.isnan(pt).any():
                return
            goal, end_goal_pos, end_goal_rot = self.find_goal(x, pt, dist, seg)
            if goal is None or end_goal_pos is None:
                return

        # Transform goal into the robot's local coordinates
        map_T_robot = np.array([[np.cos(theta), -np.sin(theta), x[0]],
                                [np.sin(theta),  np.cos(theta), x[1]],
                                [0,              0,             1]])
        inv_map_T_robot = np.linalg.inv(map_T_robot)
        goal_hom = np.array([[goal[0]], [goal[1]], [1]])
        goal_local = np.matmul(inv_map_T_robot, goal_hom)[0:2].flatten()

        # Transform end goal similarly
        end_goal_hom = np.array([[end_goal_pos[0]], [end_goal_pos[1]], [1]])
        relative_goal = np.matmul(inv_map_T_robot, end_goal_hom)[0:2].flatten()

        # Compute difference to the goal orientation using quaternion math
        quat_robot = tf_transformations.quaternion_from_euler(0, 0, theta)
        quat_inv = tf_transformations.quaternion_inverse(quat_robot)
        orientation_to_target = tf_transformations.quaternion_multiply(end_goal_rot, quat_inv)
        yaw = tf_transformations.euler_from_quaternion(orientation_to_target)[2]

        # Publish the cnn_goal in the robot's local frame
        cnn_goal = Point()
        cnn_goal.x = goal_local[0]
        cnn_goal.y = goal_local[1]
        cnn_goal.z = 0.0
        if not np.isnan(cnn_goal.x) and not np.isnan(cnn_goal.y):
            self.cnn_goal_pub.publish(cnn_goal)

        # Publish the final_goal (relative goal position and orientation difference)
        final_goal = Point()
        final_goal.x = relative_goal[0]
        final_goal.y = relative_goal[1]
        final_goal.z = yaw
        if not np.isnan(final_goal.x) and not np.isnan(final_goal.y):
            self.final_goal_pub.publish(final_goal)


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

