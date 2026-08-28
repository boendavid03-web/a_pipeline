#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--frame-id, --invalid-ring-radius, --rate-hz, --sample
# 代码中检测到的 ROS 2 话题/路径字符串：/irregular_720/invalid_points, /irregular_720/self_points, /irregular_720/valid_points
# 检测到的消息类型：Header; PointCloud2
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_irregular_720_npz.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.850391663 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_irregular_720_npz.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜visualize_irregular_720_npz.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Publish one irregular-720 NPZ sample as separate valid/invalid RViz clouds."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", required=True, type=Path)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--invalid-ring-radius", type=float, default=8.0)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    return parser.parse_args()


class SamplePublisher(Node):
    def __init__(self, args):
        super().__init__("irregular_720_npz_visualizer")
        with np.load(args.sample) as data:
            self.valid = np.asarray(data["valid_mask"], dtype=bool)
            self.self_mask = np.asarray(data["self_mask"], dtype=bool)
            self.x = np.asarray(data["points_x_base"], dtype=np.float32)
            self.y = np.asarray(data["points_y_base"], dtype=np.float32)
            self.raw_angles = np.asarray(data["raw_angles_sensor"], dtype=np.float32)
        if self.valid.shape != (720,):
            raise ValueError(f"Expected a (720,) sample, got {self.valid.shape}")
        self.frame_id = args.frame_id
        self.ring_radius = args.invalid_ring_radius
        self.valid_pub = self.create_publisher(PointCloud2, "/irregular_720/valid_points", 1)
        self.invalid_pub = self.create_publisher(PointCloud2, "/irregular_720/invalid_points", 1)
        self.self_pub = self.create_publisher(PointCloud2, "/irregular_720/self_points", 1)
        self.create_timer(1.0 / args.rate_hz, self.publish)
        self.get_logger().info(f"Publishing {args.sample}: valid={int(self.valid.sum())}, invalid={int((~self.valid).sum())}")

    def cloud(self, points):
        header = self.get_clock().now().to_msg()
        from std_msgs.msg import Header
        stamped_header = Header(stamp=header, frame_id=self.frame_id)
        return point_cloud2.create_cloud_xyz32(stamped_header, points.tolist())

    def publish(self):
        valid_points = np.column_stack((self.x[self.valid], self.y[self.valid], np.zeros(self.valid.sum(), dtype=np.float32)))
        # Self hits retain their measured base-frame endpoint.  Other invalid
        # slots have no endpoint by definition, so show their slot angle on a
        # reference ring rather than inventing a range measurement.
        self_points = np.column_stack((self.x[self.self_mask], self.y[self.self_mask], np.zeros(self.self_mask.sum(), dtype=np.float32)))
        ring_mask = ~self.valid & ~self.self_mask
        ring_points = np.column_stack((self.ring_radius * np.cos(self.raw_angles[ring_mask]), self.ring_radius * np.sin(self.raw_angles[ring_mask]), np.zeros(ring_mask.sum(), dtype=np.float32)))
        self.valid_pub.publish(self.cloud(valid_points.astype(np.float32)))
        self.invalid_pub.publish(self.cloud(ring_points.astype(np.float32)))
        self.self_pub.publish(self.cloud(self_points.astype(np.float32)))


def main():
    args = parse_args()
    if args.rate_hz <= 0 or args.invalid_ring_radius <= 0:
        raise ValueError("--rate-hz and --invalid-ring-radius must be positive")
    rclpy.init()
    node = SamplePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
