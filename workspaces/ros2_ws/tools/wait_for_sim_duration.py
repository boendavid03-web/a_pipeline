#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--duration, --progress-interval, --startup-timeout
# 代码中检测到的 ROS 2 话题/路径字符串：/clock
# 检测到的消息类型：Clock
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：NANOSECONDS_PER_SECOND
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/wait_for_sim_duration.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 10:10:36.708830265 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:39.616259944 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（通过 ros2 run 启动该 ROS 2 节点）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/wait_for_sim_duration.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜wait_for_sim_duration.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Wait for a requested amount of ROS simulation time on /clock."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


class SimulationDurationWaiter(Node):
    def __init__(self, duration, progress_interval, startup_timeout):
        super().__init__("simulation_duration_waiter")
        self.duration = float(duration)
        self.progress_interval = float(progress_interval)
        self.startup_timeout = float(startup_timeout)
        self.started_wall_time = time.monotonic()
        self.last_message_wall_time = None
        self.first_sim_ns = None
        self.last_sim_ns = None
        self.next_progress = 0.0
        self.done = False
        self.failed = False
        self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.create_timer(1.0, self.watchdog_callback)

    def clock_callback(self, msg):
        now_ns = stamp_to_nanoseconds(msg.clock)
        self.last_message_wall_time = time.monotonic()
        if self.last_sim_ns is not None and now_ns < self.last_sim_ns:
            self.get_logger().warn(
                "Simulation clock moved backwards; restarting duration measurement"
            )
            self.first_sim_ns = now_ns
            self.next_progress = 0.0
        elif self.first_sim_ns is None:
            self.first_sim_ns = now_ns
        self.last_sim_ns = now_ns

        elapsed = (now_ns - self.first_sim_ns) / NANOSECONDS_PER_SECOND
        if elapsed + 1e-9 >= self.next_progress:
            remaining = max(0.0, self.duration - elapsed)
            print(
                f"simulation_elapsed={elapsed:.1f}s "
                f"simulation_remaining={remaining:.1f}s",
                flush=True,
            )
            self.next_progress += self.progress_interval
        if elapsed + 1e-9 >= self.duration:
            self.done = True

    def watchdog_callback(self):
        now = time.monotonic()
        if self.first_sim_ns is None:
            if now - self.started_wall_time > self.startup_timeout:
                self.get_logger().error(
                    f"No /clock message received within {self.startup_timeout:.1f}s"
                )
                self.failed = True
            return
        if (
            self.last_message_wall_time is not None
            and now - self.last_message_wall_time > self.startup_timeout
        ):
            self.get_logger().warn(
                "/clock has stopped; waiting for the simulation to resume"
            )
            self.last_message_wall_time = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--progress-interval", default=10.0, type=float)
    parser.add_argument("--startup-timeout", default=15.0, type=float)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.progress_interval <= 0.0:
        parser.error("--progress-interval must be positive")
    if args.startup_timeout <= 0.0:
        parser.error("--startup-timeout must be positive")

    rclpy.init()
    node = SimulationDurationWaiter(
        args.duration, args.progress_interval, args.startup_timeout
    )
    try:
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.done and not node.failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
