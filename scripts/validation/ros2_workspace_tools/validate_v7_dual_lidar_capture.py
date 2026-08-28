#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --messages, --range-max, --range-min, --rate, --samples, --timeout
# 代码中检测到的 ROS 2 话题/路径字符串：/scan_01, /scan_02
# 检测到的消息类型：LaserScan
# 检测到的文件格式：BAG, YAML
# 可能使用的关键环境变量：TOPICS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.952217972 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜validate_v7_dual_lidar_capture.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Validate raw v7 dual-LiDAR geometry and rates without modifying the data."""

from __future__ import annotations

import argparse
import math
import statistics
import time
from collections import defaultdict
from pathlib import Path

TOPICS = ("/scan_01", "/scan_02")


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def hz(values):
    if len(values) < 2 or values[-1] <= values[0]:
        return float("nan")
    return (len(values) - 1) / (values[-1] - values[0])


def summarize(
    topic,
    messages,
    receipt_times,
    expected_samples,
    expected_rate,
    expected_range_min,
    expected_range_max,
    require_wall_rate=False,
):
    if not messages:
        raise RuntimeError(f"No messages received for {topic}")
    lengths = [len(message.ranges) for message in messages]
    if set(lengths) != {expected_samples}:
        raise RuntimeError(
            f"{topic}: expected {expected_samples} beams, observed {sorted(set(lengths))}"
        )
    frames = sorted({message.header.frame_id for message in messages})
    if len(frames) != 1 or not frames[0]:
        raise RuntimeError(f"{topic}: invalid frame_id set {frames}")
    range_mins = {float(message.range_min) for message in messages}
    range_maxs = {float(message.range_max) for message in messages}
    if len(range_mins) != 1 or len(range_maxs) != 1:
        raise RuntimeError(
            f"{topic}: inconsistent ranges min={sorted(range_mins)} max={sorted(range_maxs)}"
        )
    observed_range_min = next(iter(range_mins))
    observed_range_max = next(iter(range_maxs))
    if expected_range_min is not None and not math.isclose(
        observed_range_min, expected_range_min, rel_tol=1e-6, abs_tol=1e-6
    ):
        raise RuntimeError(
            f"{topic}: range_min={observed_range_min:g}, expected {expected_range_min:g}"
        )
    if expected_range_max is not None and not math.isclose(
        observed_range_max, expected_range_max, rel_tol=1e-6, abs_tol=1e-6
    ):
        raise RuntimeError(
            f"{topic}: range_max={observed_range_max:g}, expected {expected_range_max:g}"
        )

    stamps = [stamp_seconds(message) for message in messages]
    sim_hz = hz(stamps)
    wall_hz = hz(receipt_times) if receipt_times else float("nan")
    increments = [float(message.angle_increment) for message in messages]
    angle_increment = statistics.median(increments)
    span = statistics.median(
        [float(message.angle_max - message.angle_min) for message in messages]
    )
    inclusive_expected = span / (expected_samples - 1) if expected_samples > 1 else 0.0
    circular_pitch = 2.0 * math.pi / expected_samples
    tolerance = max(1e-6, abs(inclusive_expected) * 1e-4)
    if expected_samples > 1 and not math.isclose(
        angle_increment, inclusive_expected, rel_tol=1e-4, abs_tol=tolerance
    ):
        raise RuntimeError(
            f"{topic}: angle_increment={angle_increment} does not span angle_min..angle_max"
        )
    if not math.isfinite(sim_hz) or abs(sim_hz - expected_rate) > max(0.25, expected_rate * 0.1):
        raise RuntimeError(
            f"{topic}: simulation-time rate {sim_hz:.6f} Hz is not near {expected_rate:g} Hz"
        )
    if require_wall_rate and (
        not math.isfinite(wall_hz)
        or abs(wall_hz - expected_rate) > max(0.25, expected_rate * 0.15)
    ):
        raise RuntimeError(
            f"{topic}: wall-time delivery rate {wall_hz:.6f} Hz is not near "
            f"{expected_rate:g} Hz"
        )

    wall_text = "n/a" if not math.isfinite(wall_hz) else f"{wall_hz:.6f}"
    print(
        f"topic={topic} messages={len(messages)} beams={expected_samples} "
        f"frame_id={frames[0]} range_min={observed_range_min:g} "
        f"range_max={observed_range_max:g} angle_increment={angle_increment:.12f} "
        f"inclusive_span_step={inclusive_expected:.12f} "
        f"circular_360deg_per_sample={circular_pitch:.12f} "
        f"header_span_sec={stamps[-1] - stamps[0]:.6f} "
        f"sim_time_hz={sim_hz:.6f} wall_time_hz={wall_text}"
    )


def validate_live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Collector(Node):
        def __init__(self):
            super().__init__("validate_v7_dual_lidar_capture")
            self.messages = defaultdict(list)
            self.receipts = defaultdict(list)
            for topic in TOPICS:
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda message, selected=topic: self.callback(selected, message),
                    qos_profile_sensor_data,
                )

        def callback(self, topic, message):
            if len(self.messages[topic]) < args.messages:
                self.messages[topic].append(message)
                self.receipts[topic].append(time.monotonic())

    rclpy.init()
    node = Collector()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and any(
            len(node.messages[topic]) < args.messages for topic in TOPICS
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        for topic in TOPICS:
            summarize(
                topic,
                node.messages[topic],
                node.receipts[topic],
                args.samples,
                args.rate,
                args.range_min,
                args.range_max,
                args.require_wall_rate,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def validate_bag(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = args.bag.expanduser().resolve()
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"ROS 2 bag metadata not found: {bag / 'metadata.yaml'}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = sorted(set(TOPICS) - set(topic_types))
    if missing:
        raise RuntimeError(f"Bag is missing topics: {missing}")
    message_types = {topic: get_message(topic_types[topic]) for topic in TOPICS}
    messages = defaultdict(list)
    receipts = defaultdict(list)
    while reader.has_next():
        topic, serialized, receipt_ns = reader.read_next()
        if topic in message_types:
            messages[topic].append(
                deserialize_message(serialized, message_types[topic])
            )
            receipts[topic].append(receipt_ns * 1e-9)
    for topic in TOPICS:
        summarize(
            topic,
            messages[topic],
            receipts[topic],
            args.samples,
            args.rate,
            args.range_min,
            args.range_max,
            args.require_wall_rate,
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    live = subparsers.add_parser("live")
    live.add_argument("--samples", type=int, required=True)
    live.add_argument("--rate", type=float, required=True)
    live.add_argument("--range-min", type=float)
    live.add_argument("--range-max", type=float)
    live.add_argument("--messages", type=int, default=30)
    live.add_argument("--timeout", type=float, default=30.0)
    live.add_argument(
        "--require-wall-rate",
        action="store_true",
        help="Also require receipt-time delivery rate to match --rate.",
    )
    bag = subparsers.add_parser("bag")
    bag.add_argument("--bag", type=Path, required=True)
    bag.add_argument("--samples", type=int, required=True)
    bag.add_argument("--rate", type=float, required=True)
    bag.add_argument("--range-min", type=float)
    bag.add_argument("--range-max", type=float)
    bag.add_argument(
        "--require-wall-rate",
        action="store_true",
        help="Also require rosbag receipt-time rate to match --rate.",
    )
    args = parser.parse_args()
    if args.samples <= 1 or args.rate <= 0:
        parser.error("--samples must be > 1 and --rate must be positive")
    if (args.range_min is None) != (args.range_max is None):
        parser.error("--range-min and --range-max must be supplied together")
    if args.range_min is not None and not (0 < args.range_min < args.range_max):
        parser.error("range must satisfy 0 < --range-min < --range-max")
    if args.mode == "live" and (args.messages < 2 or args.timeout <= 0):
        parser.error("--messages must be >= 2 and --timeout must be positive")
    return args


def main():
    args = parse_args()
    if args.mode == "live":
        validate_live(args)
    else:
        validate_bag(args)


if __name__ == "__main__":
    main()
