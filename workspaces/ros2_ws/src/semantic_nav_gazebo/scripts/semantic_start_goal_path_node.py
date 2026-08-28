#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/data_collection/goal_accepted, /goal_pose, /odom, /semantic_cnn/final_goal, /semantic_cnn/global_path, /semantic_cnn/local_subgoal, /semantic_cnn/local_subgoal_marker
# 检测到的消息类型：Marker; Odometry, Path as PathMsg; PointStamped, PoseStamped
# 检测到的文件格式：YAML
# 可能使用的关键环境变量：NAVIGATION_PROJECT_ROOT, RELIABLE, SPHERE, TRANSIENT_LOCAL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo semantic_start_goal_path_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.645741956 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（通过 ros2 run 启动该 ROS 2 节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（通过 ros2 launch 启动该 ROS 2 场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_start_goal_path_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Publish an A* path and local subgoal for a requested semantic-nav goal."""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from navigation_evaluation_core import (
    astar,
    free_space_mask,
    grid_to_world,
    load_map,
    snap_to_free,
    world_to_grid,
)


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def simplify_path(path_xy, min_spacing):
    simplified = [path_xy[0]]
    for point in path_xy[1:]:
        if math.hypot(point[0] - simplified[-1][0], point[1] - simplified[-1][1]) >= min_spacing:
            simplified.append(point)
    if simplified[-1] != path_xy[-1]:
        simplified.append(path_xy[-1])
    return np.asarray(simplified, dtype=np.float32)


class SemanticStartGoalPathNode(Node):
    def __init__(self):
        super().__init__("semantic_start_goal_path")
        project_root = navigation_project_root()
        maps_root = project_root / "assets" / "maps" / "ros2_workspace" / "semantic_labeling_v6"
        self.declare_parameter(
            "map_yaml",
            str(maps_root / "v6_lidar04m_20m_static_map.yaml"),
        )
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 4.0)
        self.declare_parameter("auto_set_initial_goal", True)
        self.declare_parameter("lookahead", 1.2)
        self.declare_parameter("inflate_radius", 0.53)
        self.declare_parameter("snap_radius", 0.8)
        self.declare_parameter("path_spacing", 0.12)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/semantic_cnn/global_path")
        self.declare_parameter("local_subgoal_topic", "/semantic_cnn/local_subgoal")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter(
            "goal_accepted_topic", "/data_collection/goal_accepted"
        )
        self.declare_parameter("frame_id", "map")

        self.frame_id = self.get_parameter("frame_id").value
        map_yaml = Path(self.get_parameter("map_yaml").value)
        occ_img, self.resolution, self.origin_x, self.origin_y = load_map(map_yaml)
        self.height = occ_img.shape[0]
        free = free_space_mask(
            occ_img,
            self.resolution,
            float(self.get_parameter("inflate_radius").value),
        )
        self.free = free
        self.goal_xy = None
        self.goal_cell = None
        self.path_xy = None
        self.pose = None
        self.last_start_cell = None

        self.path_pub = self.create_publisher(PathMsg, self.get_parameter("path_topic").value, 1)
        self.subgoal_pub = self.create_publisher(PointStamped, self.get_parameter("local_subgoal_topic").value, 1)
        self.subgoal_marker_pub = self.create_publisher(
            Marker,
            "/semantic_cnn/local_subgoal_marker",
            1,
        )
        self.goal_pub = self.create_publisher(PointStamped, "/semantic_cnn/final_goal", 1)
        accepted_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_accepted_pub = self.create_publisher(
            PointStamped,
            self.get_parameter("goal_accepted_topic").value,
            accepted_qos,
        )
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )
        self.create_timer(0.2, self.timer_callback)
        if bool(self.get_parameter("auto_set_initial_goal").value):
            self.set_goal(
                float(self.get_parameter("goal_x").value),
                float(self.get_parameter("goal_y").value),
            )
        else:
            self.get_logger().info(
                "Waiting for the first /goal_pose selection"
            )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = np.asarray([p.x, p.y, yaw], dtype=np.float32)

    def set_goal(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Goal coordinates must be finite")
        requested_goal_xy = np.asarray([x, y], dtype=np.float32)
        goal_cell = snap_to_free(
            world_to_grid(
                requested_goal_xy[0],
                requested_goal_xy[1],
                self.height,
                self.resolution,
                self.origin_x,
                self.origin_y,
            ),
            self.free,
            self.resolution,
            float(self.get_parameter("snap_radius").value),
        )
        snapped_x, snapped_y = grid_to_world(
            goal_cell[0],
            goal_cell[1],
            self.height,
            self.resolution,
            self.origin_x,
            self.origin_y,
        )
        goal_xy = np.asarray([snapped_x, snapped_y], dtype=np.float32)
        self.goal_xy = goal_xy
        self.goal_cell = goal_cell
        self.path_xy = None
        self.last_start_cell = None

        accepted = PointStamped()
        accepted.header.stamp = self.get_clock().now().to_msg()
        accepted.header.frame_id = self.frame_id
        accepted.point.x = float(goal_xy[0])
        accepted.point.y = float(goal_xy[1])
        self.goal_accepted_pub.publish(accepted)
        self.get_logger().info(
            f"Goal request ({requested_goal_xy[0]:.2f}, "
            f"{requested_goal_xy[1]:.2f}) accepted at free-space center "
            f"({goal_xy[0]:.2f}, {goal_xy[1]:.2f}); "
            "the next plan will start at the current odometry pose"
        )

    def goal_callback(self, msg):
        frame = msg.header.frame_id.lstrip("/")
        if frame and frame != self.frame_id.lstrip("/"):
            self.get_logger().error(
                f"Rejecting goal in frame {frame!r}; expected {self.frame_id!r}"
            )
            return
        try:
            self.set_goal(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
            )
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"Rejecting goal: {exc}")

    def plan_from_current_pose(self):
        if self.pose is None or self.goal_cell is None:
            return
        if self.path_xy is not None:
            return
        start_cell = snap_to_free(
            world_to_grid(self.pose[0], self.pose[1], self.height, self.resolution, self.origin_x, self.origin_y),
            self.free,
            self.resolution,
            float(self.get_parameter("snap_radius").value),
        )
        cells = astar(start_cell, self.goal_cell, self.free)
        path = [
            grid_to_world(row, col, self.height, self.resolution, self.origin_x, self.origin_y)
            for row, col in cells
        ]
        self.path_xy = simplify_path(path, float(self.get_parameter("path_spacing").value))
        self.last_start_cell = start_cell
        self.get_logger().info(f"Planned path with {len(self.path_xy)} points")

    def local_subgoal(self):
        dists = np.hypot(self.path_xy[:, 0] - self.pose[0], self.path_xy[:, 1] - self.pose[1])
        idx = int(np.argmin(dists))
        lookahead = float(self.get_parameter("lookahead").value)
        total = 0.0
        target = self.path_xy[-1]
        for j in range(idx, len(self.path_xy) - 1):
            total += float(np.linalg.norm(self.path_xy[j + 1] - self.path_xy[j]))
            if total >= lookahead:
                target = self.path_xy[j + 1]
                break
        dx = float(target[0] - self.pose[0])
        dy = float(target[1] - self.pose[1])
        cy = math.cos(-float(self.pose[2]))
        sy = math.sin(-float(self.pose[2]))
        return cy * dx - sy * dy, sy * dx + cy * dy

    def timer_callback(self):
        if self.pose is None:
            return
        try:
            self.plan_from_current_pose()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        if self.path_xy is None:
            return

        stamp = self.get_clock().now().to_msg()
        path_msg = PathMsg()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id
        for x, y in self.path_xy:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        local_x, local_y = self.local_subgoal()
        subgoal = PointStamped()
        subgoal.header.stamp = stamp
        subgoal.header.frame_id = "base_link"
        subgoal.point.x = float(local_x)
        subgoal.point.y = float(local_y)
        self.subgoal_pub.publish(subgoal)

        yaw = float(self.pose[2])
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "local_subgoal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(
            self.pose[0] + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        )
        marker.pose.position.y = float(
            self.pose[1] + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        )
        marker.pose.position.z = 0.22
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.34
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.a = 1.0
        self.subgoal_marker_pub.publish(marker)

        final_goal = PointStamped()
        final_goal.header.stamp = stamp
        final_goal.header.frame_id = self.frame_id
        final_goal.point.x = float(self.goal_xy[0])
        final_goal.point.y = float(self.goal_xy[1])
        self.goal_pub.publish(final_goal)


def main():
    rclpy.init()
    node = SemanticStartGoalPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
