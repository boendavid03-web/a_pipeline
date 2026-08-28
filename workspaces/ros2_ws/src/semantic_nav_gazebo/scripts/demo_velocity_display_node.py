#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /demo/velocity_marker
# 检测到的消息类型：Marker; Twist
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：DATA, LIVE, STALE, TEXT_VIEW_FACING, VELOCITY
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo demo_velocity_display_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-08-02 04:31:40.328678755 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.635564113 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜demo_velocity_display_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Publish the current demo velocity command as an RViz text marker."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from visualization_msgs.msg import Marker


class DemoVelocityDisplay(Node):
    def __init__(self) -> None:
        super().__init__("demo_velocity_display")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("marker_topic", "/demo/velocity_marker")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("stale_timeout", 0.5)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.last_cmd: Twist | None = None
        self.last_cmd_ns: int | None = None
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("marker_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_callback,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self.publish_marker)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def publish_marker(self) -> None:
        now = self.get_clock().now()
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = str(self.get_parameter("base_frame").value)
        marker.ns = "demo_velocity"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 1.75
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.24
        marker.color.a = 1.0
        marker.frame_locked = True

        if self.last_cmd is None or self.last_cmd_ns is None:
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.15
            marker.text = "CMD VELOCITY [NO DATA]\nv = N/A m/s   omega = N/A rad/s"
        else:
            age_seconds = max(0.0, (now.nanoseconds - self.last_cmd_ns) / 1e9)
            stale_timeout = float(self.get_parameter("stale_timeout").value)
            live = math.isfinite(age_seconds) and age_seconds <= stale_timeout
            if live:
                marker.color.r = 0.15
                marker.color.g = 1.0
                marker.color.b = 0.75
                state = "LIVE"
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.15
                state = f"STALE {age_seconds:.1f}s"
            marker.text = (
                f"CMD VELOCITY [{state}]\n"
                f"v = {self.last_cmd.linear.x:+.3f} m/s   "
                f"omega = {self.last_cmd.angular.z:+.3f} rad/s"
            )

        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoVelocityDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
