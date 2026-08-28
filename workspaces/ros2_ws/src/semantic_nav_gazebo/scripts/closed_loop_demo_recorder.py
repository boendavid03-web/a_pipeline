#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/data_collection/goal_accepted, /odom
# 检测到的消息类型：Odometry; PointStamped
# 检测到的文件格式：JSON
# 可能使用的关键环境变量：RELIABLE, TRANSIENT_LOCAL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo closed_loop_demo_recorder.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-18 00:58:07.956245713 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.634564093 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

# 【脚本说明｜闭环演示轨迹记录器】
# 用途：记录机器人从当前状态向目标点运动的闭环轨迹，并在到达目标或超时后生成统计摘要。
# 输入：ROS 2 参数；/odom（nav_msgs/Odometry）；可选的 /data_collection/goal_accepted（geometry_msgs/PointStamped）。
# 输出：参数 output_csv 指定的轨迹 CSV，以及同目录下的 closed_loop_demo_summary.json；不发布控制指令。
# 依赖关系：依赖导航/仿真系统发布里程计；follow_accepted_goal=true 时依赖目标发布节点
#            （例如 semantic_start_goal_path_node.py）发布 goal_accepted 事件。
# 被依赖关系：通常由 semantic_cnn_fixed_dual_start_goal_demo.launch.py 或
#              drl_vo_fixed_dual_start_goal_demo.launch.py 启动；输出可供实验分析使用。
# 文件定位：这是 src/semantic_nav_gazebo/scripts/ 下的当前源脚本；install/ 下同名文件只是指向它的符号链接，
#            不是备份或旧版本。重新构建 ROS 2 工作空间时，以本文件为准重新安装。

"""Persist an isolated closed-loop demo trajectory without changing navigation."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ClosedLoopDemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__("closed_loop_demo_recorder")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_csv", "")
        self.declare_parameter("goal_x", 16.0)
        self.declare_parameter("goal_y", 16.0)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter(
            "goal_accepted_topic", "/data_collection/goal_accepted"
        )
        self.declare_parameter("follow_accepted_goal", False)
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("moving_speed_threshold", 0.02)
        output = Path(str(self.get_parameter("output_csv").value))
        if not str(output):
            raise ValueError("output_csv must be provided")
        output.parent.mkdir(parents=True, exist_ok=True)
        self.output_csv = output
        self.summary_path = output.with_name("closed_loop_demo_summary.json")
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.follow_accepted_goal = bool(
            self.get_parameter("follow_accepted_goal").value
        )
        self.goal_active = not self.follow_accepted_goal
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.started_wall = time.monotonic()
        self.last_xy = None
        self.previous_xy = None
        self.previous_stamp = None
        self.moving_distance = 0.0
        self.moving_time = 0.0
        self.samples = 0
        self.min_goal_distance = float("inf")
        self.reached = False
        self.file = output.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=("stamp_sec", "stamp_nanosec", "x", "y", "linear_x", "angular_z", "goal_distance"),
        )
        self.writer.writeheader()
        self.file.flush()
        if self.follow_accepted_goal:
            accepted_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                PointStamped,
                str(self.get_parameter("goal_accepted_topic").value),
                self.goal_callback,
                accepted_qos,
            )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
        self.create_timer(0.5, self.timeout_callback)
        self.get_logger().info(f"Recording closed-loop trajectory to {self.output_csv}")

    def goal_callback(self, msg: PointStamped) -> None:
        goal_x = float(msg.point.x)
        goal_y = float(msg.point.y)
        if not math.isfinite(goal_x) or not math.isfinite(goal_y):
            self.get_logger().error("Ignoring non-finite accepted goal")
            return
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_active = True
        self.started_wall = time.monotonic()
        self.last_xy = None
        self.previous_xy = None
        self.previous_stamp = None
        self.moving_distance = 0.0
        self.moving_time = 0.0
        self.samples = 0
        self.min_goal_distance = float("inf")
        self.reached = False
        self.get_logger().info(
            f"Recording accepted goal ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def write_summary(self, reason: str) -> None:
        average_speed = (
            self.moving_distance / self.moving_time
            if self.moving_time > 0.0
            else 0.0
        )
        summary = {
            "output_csv": str(self.output_csv),
            "goal": [self.goal_x, self.goal_y],
            "goal_tolerance": self.goal_tolerance,
            "samples": self.samples,
            "minimum_goal_distance": self.min_goal_distance,
            "average_speed_mps": average_speed,
            "reached": self.reached,
            "finish_reason": reason,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    def finish(self, reason: str) -> None:
        self.write_summary(reason)
        self.file.close()
        self.get_logger().info(f"Closed-loop recording finished: {reason}")
        rclpy.shutdown()

    def odom_callback(self, msg: Odometry) -> None:
        if not self.goal_active:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.previous_xy is not None and self.previous_stamp is not None:
            elapsed = stamp - self.previous_stamp
            distance = math.hypot(
                x - self.previous_xy[0],
                y - self.previous_xy[1],
            )
            if (
                elapsed > 0.0
                and distance / elapsed
                >= float(self.get_parameter("moving_speed_threshold").value)
            ):
                self.moving_distance += distance
                self.moving_time += elapsed
        self.previous_xy = (x, y)
        self.previous_stamp = stamp
        goal_distance = math.hypot(x - self.goal_x, y - self.goal_y)
        self.min_goal_distance = min(self.min_goal_distance, goal_distance)
        reached = goal_distance <= self.goal_tolerance
        if (
            not reached
            and self.last_xy is not None
            and math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < 0.02
        ):
            return
        self.writer.writerow({
            "stamp_sec": int(msg.header.stamp.sec), "stamp_nanosec": int(msg.header.stamp.nanosec),
            "x": x, "y": y,
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
            "goal_distance": goal_distance,
        })
        self.file.flush()
        self.samples += 1
        self.last_xy = (x, y)
        if reached:
            self.reached = True
            self.finish("goal_tolerance_reached")

    def timeout_callback(self) -> None:
        if not self.goal_active:
            return
        if time.monotonic() - self.started_wall >= float(self.get_parameter("timeout_sec").value):
            self.finish("wall_timeout")


def main() -> None:
    rclpy.init()
    node = ClosedLoopDemoRecorder()
    try:
        rclpy.spin(node)
    finally:
        if not node.file.closed:
            node.finish("interrupted")
        node.destroy_node()


if __name__ == "__main__":
    main()
