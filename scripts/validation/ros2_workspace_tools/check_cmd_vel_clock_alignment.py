#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /odom, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_clock_alignment.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.836309951 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:37.991044061 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_cmd_vel_clock_alignment.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_clock_alignment.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_cmd_vel_clock_alignment.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_cmd_vel_clock_alignment.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Read-only diagnosis for mapping headerless /cmd_vel storage time through /clock."""

import argparse
from bisect import bisect_left, bisect_right
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_sec(ns):
    return float(ns) / 1_000_000_000.0


def fmt_range(values):
    if not values:
        return "none"
    return f"{ns_to_sec(min(values)):.6f}..{ns_to_sec(max(values)):.6f}s"


def summarize(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    mid = len(ordered) // 2
    median = ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])
    return {
        "count": len(values),
        "min": round(min(values), 6),
        "max": round(max(values), 6),
        "mean": round(sum(values) / len(values), 6),
        "median": round(median, 6),
        "nonzero": sum(1 for value in values if abs(value) > 1e-6),
    }


def is_monotonic(pairs):
    return all(a[0] <= b[0] and a[1] <= b[1] for a, b in zip(pairs, pairs[1:]))


def interpolate_time(pairs, storage_times, storage_ns):
    index = bisect_left(storage_times, storage_ns)
    if index == 0:
        return pairs[0][1]
    if index >= len(pairs):
        return pairs[-1][1]
    left_storage, left_sim = pairs[index - 1]
    right_storage, right_sim = pairs[index]
    if right_storage == left_storage:
        return left_sim
    ratio = (storage_ns - left_storage) / float(right_storage - left_storage)
    return int(round(left_sim + ratio * (right_sim - left_sim)))


def nearest_cmd(cmds_by_sim, target_ns):
    if not cmds_by_sim:
        return None
    times = [item[0] for item in cmds_by_sim]
    index = bisect_left(times, target_ns)
    candidates = []
    if index < len(cmds_by_sim):
        candidates.append(cmds_by_sim[index])
    if index > 0:
        candidates.append(cmds_by_sim[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - target_ns))


def hold_last_cmd(cmds_by_sim, target_ns):
    if not cmds_by_sim:
        return None
    times = [item[0] for item in cmds_by_sim]
    index = bisect_right(times, target_ns) - 1
    if index < 0:
        return None
    return cmds_by_sim[index]


def first_nonzero(items, value_index):
    for item in items:
        if abs(item[value_index]) > 1e-6:
            return item
    return None


def sign_events(items, value_index):
    events = []
    last_sign = 0
    for item in items:
        value = item[value_index]
        sign = 1 if value > 1e-6 else -1 if value < -1e-6 else 0
        if sign != 0 and sign != last_sign:
            events.append((item[0], sign, value))
            last_sign = sign
        elif sign == 0:
            last_sign = 0
    return events


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = ["/clock", "/cmd_vel", "/scan_merged", "/odom"]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")

    msg_types = {topic: get_message(topic_types[topic]) for topic in required}
    clocks = []
    cmds = []
    scans = []
    odoms = []

    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if topic == "/clock":
            msg = deserialize_message(data, msg_types[topic])
            clocks.append((int(storage_ns), stamp_to_ns(msg.clock)))
        elif topic == "/cmd_vel":
            msg = deserialize_message(data, msg_types[topic])
            cmds.append(
                (
                    int(storage_ns),
                    float(msg.linear.x),
                    float(msg.linear.y),
                    float(msg.angular.z),
                )
            )
        elif topic == "/scan_merged":
            msg = deserialize_message(data, msg_types[topic])
            scans.append(stamp_to_ns(msg.header.stamp))
        elif topic == "/odom":
            msg = deserialize_message(data, msg_types[topic])
            stamp_ns = stamp_to_ns(msg.header.stamp)
            twist = msg.twist.twist
            odoms.append((stamp_ns, float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)))

    return clocks, cmds, scans, odoms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    args = parser.parse_args()

    clocks, cmds, scans, odoms = read_bag(args.bag)
    monotonic = is_monotonic(clocks)
    sorted_clocks = sorted(clocks)
    clock_storage_times = [item[0] for item in sorted_clocks]
    mapped_cmds = [
        (interpolate_time(sorted_clocks, clock_storage_times, storage_ns), linear_x, linear_y, angular_z)
        for storage_ns, linear_x, linear_y, angular_z in cmds
    ]
    mapped_cmds.sort(key=lambda item: item[0])
    scans_sorted = sorted(scans)
    nearest_values = [
        match[3] for scan_ns in scans_sorted if (match := nearest_cmd(mapped_cmds, scan_ns)) is not None
    ]
    hold_values = [
        match[3] for scan_ns in scans_sorted if (match := hold_last_cmd(mapped_cmds, scan_ns)) is not None
    ]
    raw_cmd_angular = [item[3] for item in cmds]
    odom_angular = [item[3] for item in odoms]
    first_cmd_turn = first_nonzero(mapped_cmds, 3)
    first_odom_turn = first_nonzero(odoms, 3)
    cmd_events = sign_events(mapped_cmds, 3)
    odom_events = sign_events(odoms, 3)

    event_deltas = []
    for cmd_time, cmd_sign, _value in cmd_events[:20]:
        same_sign = [event for event in odom_events if event[1] == cmd_sign]
        if same_sign:
            nearest = min(same_sign, key=lambda event: abs(event[0] - cmd_time))
            event_deltas.append(ns_to_sec(nearest[0] - cmd_time))
    abs_deltas = [abs(delta) for delta in event_deltas]
    event_match = bool(abs_deltas) and (sum(abs_deltas) / len(abs_deltas)) <= 2.0
    obvious_delay = bool(event_deltas) and abs(sum(event_deltas) / len(event_deltas)) > 1.0

    mapped_range = [item[0] for item in mapped_cmds]
    scan_overlap = bool(mapped_range and scans and max(min(mapped_range), min(scans)) <= min(max(mapped_range), max(scans)))
    conclusion = "safe"
    reasons = []
    if not monotonic:
        conclusion = "unsafe"
        reasons.append("clock mapping is not monotonic")
    if not scan_overlap:
        conclusion = "unsafe"
        reasons.append("mapped cmd_vel time range does not overlap scan time range")
    if not event_match:
        conclusion = "warning" if conclusion == "safe" else conclusion
        reasons.append("turning events need manual confirmation")
    if obvious_delay:
        conclusion = "warning" if conclusion == "safe" else conclusion
        reasons.append("possible control delay over 1s")

    print(f"bag: {args.bag}")
    print(f"clock_count: {len(clocks)}")
    print(f"clock_storage_range: {fmt_range([item[0] for item in clocks])}")
    print(f"clock_sim_range: {fmt_range([item[1] for item in clocks])}")
    print(f"cmd_vel_count: {len(cmds)}")
    print(f"cmd_vel_storage_range: {fmt_range([item[0] for item in cmds])}")
    print(f"cmd_vel_mapped_sim_range: {fmt_range(mapped_range)}")
    print(f"scan_merged_stamp_range: {fmt_range(scans)}")
    print(f"odom_stamp_range: {fmt_range([item[0] for item in odoms])}")
    print(f"raw_cmd_vel_angular_z: {summarize(raw_cmd_angular)}")
    print(f"mapped_nearest_scan_angular_z: {summarize(nearest_values)}")
    print(f"mapped_hold_last_scan_angular_z: {summarize(hold_values)}")
    print(f"odom_twist_angular_z: {summarize(odom_angular)}")
    print(
        "first_nonzero_cmd_vel_mapped_sim_time: "
        f"{None if first_cmd_turn is None else round(ns_to_sec(first_cmd_turn[0]), 6)}"
    )
    print(
        "first_nonzero_odom_twist_sim_time: "
        f"{None if first_odom_turn is None else round(ns_to_sec(first_odom_turn[0]), 6)}"
    )
    print(f"turn_event_delta_sec_cmd_to_odom: {[round(delta, 6) for delta in event_deltas[:20]]}")
    print(f"turn_events_roughly_align: {event_match}")
    print(f"obvious_control_delay: {obvious_delay}")
    print(f"clock_storage_to_sim_monotonic: {monotonic}")
    print(f"conclusion: {conclusion}")
    if reasons:
        print(f"reasons: {reasons}")


if __name__ == "__main__":
    main()
