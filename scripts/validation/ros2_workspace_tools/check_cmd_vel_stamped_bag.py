#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /cmd_vel_stamped, /odom, /scan_merged, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG
# 可能使用的关键环境变量：MISSING, PASS, REQUIRED_TOPICS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.836309951 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:37.992044080 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_cmd_vel_stamped_bag.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_cmd_vel_stamped_bag.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_cmd_vel_stamped_bag.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Read-only checks for future bags containing /cmd_vel_stamped."""

import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = [
    "/cmd_vel",
    "/cmd_vel_stamped",
    "/clock",
    "/scan_merged",
    "/odom",
    "/tf",
    "/tf_static",
]


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_sec(ns):
    return float(ns) / 1_000_000_000.0


def time_range(values):
    if not values:
        return None
    return min(values), max(values)


def overlaps(a, b):
    return bool(a and b and max(a[0], b[0]) <= min(a[1], b[1]))


def summarize(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    mid = len(ordered) // 2
    median = ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
        "median": median,
        "nonzero": sum(1 for value in values if abs(value) > 1e-6),
    }


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = [topic for topic in REQUIRED_TOPICS if topic not in topic_types]

    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in ["/cmd_vel", "/cmd_vel_stamped", "/clock", "/scan_merged"]
        if topic in topic_types
    }
    counts = {topic: 0 for topic in topic_types}
    clocks = []
    scans = []
    cmd_angular = []
    stamped_times = []
    stamped_angular = []

    while reader.has_next():
        topic, data, _storage_time = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/clock":
            msg = deserialize_message(data, msg_types[topic])
            clocks.append(stamp_to_ns(msg.clock))
        elif topic == "/scan_merged":
            msg = deserialize_message(data, msg_types[topic])
            scans.append(stamp_to_ns(msg.header.stamp))
        elif topic == "/cmd_vel":
            msg = deserialize_message(data, msg_types[topic])
            cmd_angular.append(float(msg.angular.z))
        elif topic == "/cmd_vel_stamped":
            msg = deserialize_message(data, msg_types[topic])
            stamped_times.append(stamp_to_ns(msg.header.stamp))
            stamped_angular.append(float(msg.twist.angular.z))

    return missing, counts, clocks, scans, cmd_angular, stamped_times, stamped_angular


def close_distribution(a, b):
    if not a or not b:
        return False
    sa = summarize(a)
    sb = summarize(b)
    return (
        math.isclose(sa["min"], sb["min"], abs_tol=1e-6)
        and math.isclose(sa["max"], sb["max"], abs_tol=1e-6)
        and math.isclose(sa["mean"], sb["mean"], rel_tol=0.05, abs_tol=1e-3)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    args = parser.parse_args()

    missing, counts, clocks, scans, cmd_angular, stamped_times, stamped_angular = read_bag(args.bag)
    clock_range = time_range(clocks)
    scan_range = time_range(scans)
    stamped_range = time_range(stamped_times)
    in_clock_range = bool(
        stamped_range
        and clock_range
        and clock_range[0] <= stamped_range[0]
        and stamped_range[1] <= clock_range[1]
    )
    scan_overlap = overlaps(stamped_range, scan_range)
    angular_matches = close_distribution(cmd_angular, stamped_angular)

    print(f"bag: {args.bag}")
    for topic in REQUIRED_TOPICS:
        status = "PASS" if topic not in missing else "MISSING"
        print(f"{status} {topic} count={counts.get(topic, 0)}")
    print(f"cmd_vel_stamped_count_gt_0: {len(stamped_times) > 0}")
    print(
        "cmd_vel_stamped_stamp_range_sec: "
        f"{None if not stamped_range else (ns_to_sec(stamped_range[0]), ns_to_sec(stamped_range[1]))}"
    )
    print(
        "clock_sim_range_sec: "
        f"{None if not clock_range else (ns_to_sec(clock_range[0]), ns_to_sec(clock_range[1]))}"
    )
    print(
        "scan_merged_stamp_range_sec: "
        f"{None if not scan_range else (ns_to_sec(scan_range[0]), ns_to_sec(scan_range[1]))}"
    )
    print(f"cmd_vel_stamped_in_clock_range: {in_clock_range}")
    print(f"cmd_vel_stamped_scan_overlap: {scan_overlap}")
    print(f"cmd_vel_angular_z: {summarize(cmd_angular)}")
    print(f"cmd_vel_stamped_angular_z: {summarize(stamped_angular)}")
    print(f"angular_z_distribution_matches: {angular_matches}")

    if missing or not stamped_times or not in_clock_range or not scan_overlap or not angular_matches:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
