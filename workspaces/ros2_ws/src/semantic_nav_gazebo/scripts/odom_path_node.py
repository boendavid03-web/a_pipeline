#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/odom, /semantic_cnn/actual_trajectory
# 检测到的消息类型：Odometry, Path; PoseStamped
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo odom_path_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.643741916 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（通过 ros2 launch 启动该 ROS 2 场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜odom_path_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Publish a goal-scoped robot odometry trail as a nav_msgs/Path for RViz."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OdomPathNode(Node):
    def __init__(self):
        super().__init__("odom_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/semantic_cnn/actual_trajectory")
        self.declare_parameter(
            "goal_accepted_topic", "/data_collection/goal_accepted"
        )
        self.declare_parameter("min_distance", 0.05)
        self.declare_parameter("max_poses", 5000)
        self.declare_parameter("enabled", True)
        self.declare_parameter("start_on_goal", False)
        self.declare_parameter("clear_on_goal", True)

        self.path = Path()
        self.last_xy = None
        self.enabled = bool(self.get_parameter("enabled").value)
        self.start_on_goal = bool(self.get_parameter("start_on_goal").value)
        self.active = self.enabled and not self.start_on_goal
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(
            Path, self.get_parameter("path_topic").value, path_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("goal_accepted_topic").value,
            self.goal_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def clear_path(self, header=None):
        self.path = Path()
        if header is not None:
            self.path.header = header
        self.last_xy = None
        self.path_pub.publish(self.path)

    def goal_callback(self, msg):
        if bool(self.get_parameter("clear_on_goal").value):
            self.clear_path(msg.header)
        self.active = self.enabled

    def odom_callback(self, msg):
        if not self.active:
            # Keep clearing the RViz display while waiting for the first goal,
            # including when trajectory display was explicitly disabled.
            self.clear_path(msg.header)
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        min_distance = float(self.get_parameter("min_distance").value)

        if self.last_xy is not None:
            if math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < min_distance:
                return

        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(stamped)
        max_poses = int(self.get_parameter("max_poses").value)
        if max_poses > 0 and len(self.path.poses) > max_poses:
            self.path.poses = self.path.poses[-max_poses:]
        self.last_xy = (x, y)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
