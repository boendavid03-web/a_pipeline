#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--invalid-ring-radius, --sync-slop
# 代码中检测到的 ROS 2 话题/路径字符串：/irregular_720/scan_01/invalid_slots, /irregular_720/scan_01/self_points, /irregular_720/scan_01/valid_points, /irregular_720/scan_02/invalid_slots, /irregular_720/scan_02/self_points, /irregular_720/scan_02/valid_points, /scan_01, /scan_02
# 检测到的消息类型：Header; LaserScan, PointCloud2
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_live_irregular_720.py
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
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_live_irregular_720.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜visualize_live_irregular_720.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Show synchronized live /scan_01 + /scan_02 slots in RViz without merging."""

from __future__ import annotations

import argparse
import math

import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class LiveIrregularViewer(Node):
    def __init__(self, ring_radius, sync_slop):
        super().__init__("live_irregular_720_viewer")
        self.ring_radius = ring_radius
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.reported_beam_counts = set()
        self.cloud_publishers = {
            ("scan_01", "valid"): self.create_publisher(PointCloud2, "/irregular_720/scan_01/valid_points", qos_profile_sensor_data),
            ("scan_01", "invalid"): self.create_publisher(PointCloud2, "/irregular_720/scan_01/invalid_slots", qos_profile_sensor_data),
            ("scan_01", "self"): self.create_publisher(PointCloud2, "/irregular_720/scan_01/self_points", qos_profile_sensor_data),
            ("scan_02", "valid"): self.create_publisher(PointCloud2, "/irregular_720/scan_02/valid_points", qos_profile_sensor_data),
            ("scan_02", "invalid"): self.create_publisher(PointCloud2, "/irregular_720/scan_02/invalid_slots", qos_profile_sensor_data),
            ("scan_02", "self"): self.create_publisher(PointCloud2, "/irregular_720/scan_02/self_points", qos_profile_sensor_data),
        }
        scan_01 = message_filters.Subscriber(self, LaserScan, "/scan_01", qos_profile=qos_profile_sensor_data)
        scan_02 = message_filters.Subscriber(self, LaserScan, "/scan_02", qos_profile=qos_profile_sensor_data)
        sync = message_filters.ApproximateTimeSynchronizer([scan_01, scan_02], queue_size=10, slop=sync_slop)
        sync.registerCallback(self.callback)
        self.get_logger().info("Live view: scan_01=green/red/yellow; scan_02=cyan/magenta/orange")

    def transform_for(self, scan):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", scan.header.frame_id, Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        # The mounted sensors have roll/pitch flips, so extracting yaw would
        # reflect their planar data incorrectly.  Apply the XY part of the
        # complete quaternion rotation instead.
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.z * q.w)
        r10 = 2.0 * (q.x * q.y + q.z * q.w)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
        return float(t.x), float(t.y), r00, r01, r10, r11

    def points_for(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        beam_key = (scan.header.frame_id, ranges.size)
        if beam_key not in self.reported_beam_counts:
            self.reported_beam_counts.add(beam_key)
            self.get_logger().info(
                f"Visualizing {scan.header.frame_id} with {ranges.size} raw beams"
            )
        angles = float(scan.angle_min) + np.arange(ranges.size, dtype=np.float32) * float(scan.angle_increment)
        range_valid = np.isfinite(ranges) & (ranges >= float(scan.range_min)) & (ranges <= float(scan.range_max))
        transform = self.transform_for(scan)
        if transform is None:
            return [], [(self.ring_radius * math.cos(float(a)), self.ring_radius * math.sin(float(a)), 0.0) for a in angles], []
        tx, ty, r00, r01, r10, r11 = transform
        valid, invalid, self_points = [], [], []
        for raw_range, raw_angle, is_range_valid in zip(ranges, angles, range_valid):
            # Every invalid slot still has a visible red marker.  The ring
            # marker is explicitly a visualization aid, never a measurement.
            ring_sensor_x = self.ring_radius * math.cos(float(raw_angle))
            ring_sensor_y = self.ring_radius * math.sin(float(raw_angle))
            ring_x = tx + r00 * ring_sensor_x + r01 * ring_sensor_y
            ring_y = ty + r10 * ring_sensor_x + r11 * ring_sensor_y
            if not is_range_valid:
                invalid.append((ring_x, ring_y, 0.0))
                continue
            sx, sy = float(raw_range * math.cos(raw_angle)), float(raw_range * math.sin(raw_angle))
            bx, by = tx + r00 * sx + r01 * sy, ty + r10 * sx + r11 * sy
            if abs(bx) <= 0.36 and abs(by) <= 0.32:
                self_points.append((bx, by, 0.0))
            else:
                valid.append((bx, by, 0.0))
        return valid, invalid, self_points

    def cloud(self, stamp, points):
        return point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id="base_link"), points)

    def callback(self, scan_01, scan_02):
        a_valid, a_invalid, a_self = self.points_for(scan_01)
        b_valid, b_invalid, b_self = self.points_for(scan_02)
        stamp = scan_01.header.stamp if (scan_01.header.stamp.sec, scan_01.header.stamp.nanosec) >= (scan_02.header.stamp.sec, scan_02.header.stamp.nanosec) else scan_02.header.stamp
        for sensor, points in (("scan_01", (a_valid, a_invalid, a_self)), ("scan_02", (b_valid, b_invalid, b_self))):
            for state, state_points in zip(("valid", "invalid", "self"), points):
                self.cloud_publishers[(sensor, state)].publish(self.cloud(stamp, state_points))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--invalid-ring-radius", type=float, default=8.0)
    parser.add_argument("--sync-slop", type=float, default=0.5, help="Maximum /scan_01-/scan_02 timestamp difference in seconds")
    args = parser.parse_args()
    if args.invalid_ring_radius <= 0 or args.sync_slop <= 0:
        raise ValueError("--invalid-ring-radius and --sync-slop must be positive")
    rclpy.init()
    node = LiveIrregularViewer(args.invalid_ring_radius, args.sync_slop)
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
