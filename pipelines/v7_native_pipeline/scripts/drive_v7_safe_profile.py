#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--duration, --profile, --scan-timeout, --stop-distance
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /scan_merged
# 检测到的消息类型：LaserScan; Twist
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：PROFILES
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/drive_v7_safe_profile.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.488295204 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.913035738 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/drive_v7_safe_profile.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜drive_v7_safe_profile.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Drive deterministic v7 collection profiles with a front-LiDAR safety gate."""

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


PROFILES = {
    1: [
        (16.0, 0.24, 0.00),
        (10.0, 0.18, 0.32),
        (9.0, 0.00, 0.48),
        (16.0, 0.24, 0.00),
        (10.0, 0.18, -0.32),
        (9.0, 0.00, -0.48),
    ],
    2: [
        (12.0, 0.20, -0.36),
        (18.0, 0.25, 0.00),
        (11.0, 0.00, 0.52),
        (12.0, 0.20, 0.36),
        (18.0, 0.25, 0.00),
        (11.0, 0.00, -0.52),
    ],
    3: [
        (10.0, 0.00, 0.55),
        (20.0, 0.22, 0.18),
        (10.0, 0.00, -0.55),
        (20.0, 0.22, -0.18),
    ],
}


def angle_difference(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class SafeProfileDriver(Node):
    def __init__(self, profile, duration, stop_distance, scan_timeout):
        super().__init__("v7_safe_profile_driver")
        self.profile = PROFILES[profile]
        self.profile_id = profile
        self.duration = duration
        self.stop_distance = stop_distance
        self.scan_timeout = scan_timeout
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan_merged",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.scan_received_at = None
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.started_at = time.monotonic()
        self.last_log_at = 0.0
        self.safety_override_count = 0
        self.publish_count = 0

    def on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        def sector_min(center, half_width):
            mask = valid & (np.abs([angle_difference(a, center) for a in angles]) <= half_width)
            return float(np.min(ranges[mask])) if np.any(mask) else math.inf

        self.front_min = sector_min(0.0, math.radians(32.0))
        self.left_min = sector_min(math.radians(65.0), math.radians(35.0))
        self.right_min = sector_min(math.radians(-65.0), math.radians(35.0))
        self.scan_received_at = time.monotonic()

    def requested_command(self, elapsed):
        period = sum(segment[0] for segment in self.profile)
        within = elapsed % period
        for segment_duration, linear_x, angular_z in self.profile:
            if within < segment_duration:
                return linear_x, angular_z
            within -= segment_duration
        return 0.0, 0.0

    def safe_command(self, elapsed):
        linear_x, angular_z = self.requested_command(elapsed)
        now = time.monotonic()
        if self.scan_received_at is None or now - self.scan_received_at > self.scan_timeout:
            return 0.0, 0.0, "waiting_for_scan"
        if linear_x > 0.0 and self.front_min < self.stop_distance:
            turn_sign = 1.0 if self.left_min >= self.right_min else -1.0
            self.safety_override_count += 1
            return 0.0, turn_sign * 0.52, "front_safety_turn"
        return linear_x, angular_z, "profile"

    def publish(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        self.publish_count += 1

    def run(self):
        next_tick = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            elapsed = time.monotonic() - self.started_at
            if elapsed >= self.duration:
                break
            linear_x, angular_z, mode = self.safe_command(elapsed)
            self.publish(linear_x, angular_z)
            if elapsed - self.last_log_at >= 5.0:
                self.get_logger().info(
                    f"profile={self.profile_id} elapsed={elapsed:.1f}/{self.duration:.1f}s "
                    f"mode={mode} cmd=({linear_x:.2f},{angular_z:.2f}) "
                    f"ranges(front/left/right)=({self.front_min:.2f},"
                    f"{self.left_min:.2f},{self.right_min:.2f})"
                )
                self.last_log_at = elapsed
            next_tick += 0.1
            time.sleep(max(0.0, next_tick - time.monotonic()))

        for _ in range(10):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.03)
        self.get_logger().info(
            f"complete profile={self.profile_id} duration={self.duration:.1f}s "
            f"published={self.publish_count} safety_overrides={self.safety_override_count}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=int, choices=sorted(PROFILES), required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--stop-distance", type=float, default=1.0)
    parser.add_argument("--scan-timeout", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.stop_distance <= 0.0:
        parser.error("--stop-distance must be positive")

    rclpy.init()
    node = SafeProfileDriver(
        args.profile,
        args.duration,
        args.stop_distance,
        args.scan_timeout,
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
