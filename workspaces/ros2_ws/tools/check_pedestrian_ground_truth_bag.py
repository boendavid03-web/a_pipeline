#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --clock-boundary-tolerance-sec, --max-ground-truth-gap-sec, --max-median-velocity-error, --max-pair-gap-sec, --max-speed-ratio, --min-direction-cosine, --min-duration-sec, --min-speed-ratio
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /pedestrian_ground_truth
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG
# 可能使用的关键环境变量：EXPECTED_TOPIC, EXPECTED_TYPE, FAIL, NANOSECONDS_PER_SECOND, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_pedestrian_ground_truth_bag.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 10:10:36.789832368 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:31.258092381 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_pedestrian_ground_truth_bag.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_pedestrian_ground_truth_bag.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Validate pedestrian ground-truth timing and Pose/Twist kinematics in a bag."""

import argparse
import math
import statistics
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NANOSECONDS_PER_SECOND = 1_000_000_000
EXPECTED_TOPIC = "/pedestrian_ground_truth"
EXPECTED_TYPE = "semantic_nav_gazebo/msg/PedestrianStateArray"


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def percentile(values, percentile_value):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile_value) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if EXPECTED_TOPIC not in topic_types:
        raise RuntimeError(f"Bag is missing {EXPECTED_TOPIC}")
    if topic_types[EXPECTED_TOPIC] != EXPECTED_TYPE:
        raise RuntimeError(
            f"{EXPECTED_TOPIC} has type {topic_types[EXPECTED_TOPIC]!r}, "
            f"expected {EXPECTED_TYPE!r}"
        )
    if "/clock" not in topic_types:
        raise RuntimeError("Bag is missing /clock")

    pedestrian_type = get_message(topic_types[EXPECTED_TOPIC])
    clock_type = get_message(topic_types["/clock"])
    clock_stamps = []
    messages = []
    while reader.has_next():
        topic, data, _storage_time = reader.read_next()
        if topic == "/clock":
            msg = deserialize_message(data, clock_type)
            clock_stamps.append(stamp_to_nanoseconds(msg.clock))
        elif topic == EXPECTED_TOPIC:
            msg = deserialize_message(data, pedestrian_type)
            pedestrians = {
                state.id: (
                    float(state.pose.position.x),
                    float(state.pose.position.y),
                    float(state.velocity.linear.x),
                    float(state.velocity.linear.y),
                )
                for state in msg.pedestrians
            }
            messages.append(
                (
                    stamp_to_nanoseconds(msg.header.stamp),
                    str(msg.header.frame_id),
                    pedestrians,
                )
            )
    return clock_stamps, messages


def summarize(clock_stamps, messages, max_pair_gap, clock_boundary_tolerance):
    stamps = [item[0] for item in messages]
    frames = sorted(set(item[1] for item in messages))
    id_sets = [tuple(sorted(item[2])) for item in messages]
    reference_ids = id_sets[0] if id_sets else ()
    id_set_changes = sum(ids != reference_ids for ids in id_sets)
    decreasing_stamps = sum(current < previous for previous, current in zip(stamps, stamps[1:]))
    duplicate_stamps = sum(current == previous for previous, current in zip(stamps, stamps[1:]))
    positive_gaps = [
        (current - previous) / NANOSECONDS_PER_SECOND
        for previous, current in zip(stamps, stamps[1:])
        if current > previous
    ]

    ratios = []
    vector_errors = []
    direction_cosines = []
    pair_count = 0
    for previous, current in zip(messages, messages[1:]):
        dt = (current[0] - previous[0]) / NANOSECONDS_PER_SECOND
        if dt <= 0.0 or dt > max_pair_gap:
            continue
        common_ids = set(previous[2]).intersection(current[2])
        for pedestrian_id in common_ids:
            previous_state = previous[2][pedestrian_id]
            current_state = current[2][pedestrian_id]
            measured_vx = (current_state[0] - previous_state[0]) / dt
            measured_vy = (current_state[1] - previous_state[1]) / dt
            reported_vx = 0.5 * (previous_state[2] + current_state[2])
            reported_vy = 0.5 * (previous_state[3] + current_state[3])
            measured_speed = math.hypot(measured_vx, measured_vy)
            reported_speed = math.hypot(reported_vx, reported_vy)
            vector_errors.append(
                math.hypot(
                    measured_vx - reported_vx,
                    measured_vy - reported_vy,
                )
            )
            if reported_speed >= 0.1 and measured_speed >= 0.02:
                ratios.append(measured_speed / reported_speed)
                direction_cosines.append(
                    (
                        measured_vx * reported_vx
                        + measured_vy * reported_vy
                    )
                    / (measured_speed * reported_speed)
                )
            pair_count += 1

    duration = (
        (max(stamps) - min(stamps)) / NANOSECONDS_PER_SECOND if stamps else 0.0
    )
    return {
        "clock_count": len(clock_stamps),
        "message_count": len(messages),
        "duration": duration,
        "frames": frames,
        "pedestrian_ids": list(reference_ids),
        "id_set_changes": id_set_changes,
        "decreasing_stamps": decreasing_stamps,
        "duplicate_stamps": duplicate_stamps,
        "max_gap": max(positive_gaps) if positive_gaps else None,
        "clock_contains_ground_truth": bool(
            clock_stamps
            and stamps
            and min(clock_stamps) - clock_boundary_tolerance * NANOSECONDS_PER_SECOND
            <= min(stamps)
            and max(stamps)
            <= max(clock_stamps)
            + clock_boundary_tolerance * NANOSECONDS_PER_SECOND
        ),
        "pair_count": pair_count,
        "ratio_count": len(ratios),
        "speed_ratio_median": statistics.median(ratios) if ratios else None,
        "speed_ratio_p95": percentile(ratios, 95),
        "velocity_error_median": (
            statistics.median(vector_errors) if vector_errors else None
        ),
        "velocity_error_p95": percentile(vector_errors, 95),
        "direction_cosine_median": (
            statistics.median(direction_cosines)
            if direction_cosines
            else None
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--min-duration-sec", default=0.0, type=float)
    parser.add_argument("--max-ground-truth-gap-sec", default=0.5, type=float)
    parser.add_argument("--max-pair-gap-sec", default=0.5, type=float)
    parser.add_argument("--clock-boundary-tolerance-sec", default=0.5, type=float)
    parser.add_argument("--min-speed-ratio", default=0.65, type=float)
    parser.add_argument("--max-speed-ratio", default=1.35, type=float)
    parser.add_argument("--max-median-velocity-error", default=0.5, type=float)
    parser.add_argument("--min-direction-cosine", default=0.8, type=float)
    args = parser.parse_args()

    clock_stamps, messages = read_bag(args.bag)
    result = summarize(
        clock_stamps,
        messages,
        args.max_pair_gap_sec,
        args.clock_boundary_tolerance_sec,
    )
    for key, value in result.items():
        print(f"{key}: {value}")

    failures = []
    if result["message_count"] < 2:
        failures.append("fewer than two pedestrian ground-truth messages")
    if not result["pedestrian_ids"]:
        failures.append("no pedestrians")
    if len(result["frames"]) != 1:
        failures.append(f"ground-truth frame changed: {result['frames']}")
    if result["id_set_changes"]:
        failures.append(
            f"pedestrian ID set changed in {result['id_set_changes']} messages"
        )
    if result["decreasing_stamps"]:
        failures.append("ground-truth timestamps decreased")
    if result["duplicate_stamps"]:
        failures.append(
            f"ground-truth timestamps repeated {result['duplicate_stamps']} times"
        )
    if not result["clock_contains_ground_truth"]:
        failures.append("ground-truth timestamps are outside the /clock range")
    if result["duration"] < args.min_duration_sec:
        failures.append(
            f"simulation duration {result['duration']:.3f}s is shorter than "
            f"{args.min_duration_sec:.3f}s"
        )
    if (
        result["max_gap"] is None
        or result["max_gap"] > args.max_ground_truth_gap_sec
    ):
        failures.append(
            f"maximum ground-truth gap {result['max_gap']}s exceeds "
            f"{args.max_ground_truth_gap_sec:.3f}s"
        )
    if result["ratio_count"] < 10:
        failures.append("too few moving pedestrian samples for a kinematic check")
    else:
        ratio = result["speed_ratio_median"]
        if not args.min_speed_ratio <= ratio <= args.max_speed_ratio:
            failures.append(
                f"median measured/reported speed ratio {ratio:.3f} is outside "
                f"[{args.min_speed_ratio:.3f}, {args.max_speed_ratio:.3f}]"
            )
        if result["velocity_error_median"] > args.max_median_velocity_error:
            failures.append(
                f"median velocity vector error "
                f"{result['velocity_error_median']:.3f}m/s exceeds "
                f"{args.max_median_velocity_error:.3f}m/s"
            )
        if result["direction_cosine_median"] < args.min_direction_cosine:
            failures.append(
                f"median velocity direction cosine "
                f"{result['direction_cosine_median']:.3f} is below "
                f"{args.min_direction_cosine:.3f}"
            )

    if failures:
        print("pedestrian_ground_truth_kinematics: FAIL")
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("pedestrian_ground_truth_kinematics: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
