#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--report-json, --session
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel_stamped, /data_collection/episode_event, /semantic_cnn/final_goal, /semantic_cnn/global_path, /semantic_cnn/local_subgoal
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPZ, TXT
# 可能使用的关键环境变量：ARRAY_FIELDS, EPISODE_EVENT_SCHEMA, EPISODE_EVENT_TYPE, FAIL, JSON, ONLINE_GOAL_TYPES, PASS, PASS_WITH_WARNINGS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_fixed_dual_lidar_dataset.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Validate every sample in a v7 fixed dual-LiDAR slot dataset."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from v7_rosbag_to_fixed_dual_lidar_dataset import (
    episode_intervals_from_events,
    map_episode_events_to_sim_time,
)

ARRAY_FIELDS = (
    "raw_ranges",
    "raw_angles_sensor",
    "points_x_base",
    "points_y_base",
    "virtual_ranges",
    "virtual_angles",
    "range_valid_mask",
    "self_mask",
    "valid_mask",
    "semantic_label",
    "source_sensor",
    "raw_beam_index",
)
ONLINE_GOAL_TYPES = {
    "/semantic_cnn/global_path": "nav_msgs/msg/Path",
    "/semantic_cnn/local_subgoal": "geometry_msgs/msg/PointStamped",
    "/semantic_cnn/final_goal": "geometry_msgs/msg/PointStamped",
}
EPISODE_EVENT_SCHEMA = "semantic_nav_episode_event/v1"
EPISODE_EVENT_TYPE = "std_msgs/msg/String"
SUCCESSFUL_EPISODE_FINISH_REASONS = frozenset(
    ("goal_reached_and_stopped", "goal_tolerance_reached")
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--report-json", type=Path)
    parser.add_argument("--minimum-samples", type=int, default=0)
    parser.add_argument("--minimum-duration-sec", type=float, default=0.0)
    parser.add_argument(
        "--minimum-unique-command-vectors", type=int, default=0
    )
    parser.add_argument(
        "--minimum-nonzero-command-fraction", type=float, default=0.0
    )
    parser.add_argument(
        "--minimum-effective-sample-rate-hz", type=float, default=0.0
    )
    parser.add_argument(
        "--minimum-person-positive-sample-fraction",
        type=float,
        default=0.0,
    )
    parser.add_argument("--maximum-subgoal-age-ms", type=float)
    parser.add_argument("--maximum-cmd-vel-age-ms", type=float)
    parser.add_argument("--require-online-subgoal", action="store_true")
    parser.add_argument(
        "--require-successful-episodes-only", action="store_true"
    )
    parser.add_argument(
        "--require-ground-truth-person-labels", action="store_true"
    )
    parser.add_argument("--require-person-observations", action="store_true")
    parser.add_argument("--require-forward-only", action="store_true")
    parser.add_argument(
        "--require-pre-relay-command-labels", action="store_true"
    )
    parser.add_argument(
        "--maximum-person-truth-unmatched-samples", type=int
    )
    parser.add_argument("--fail-on-warnings", action="store_true")
    return parser.parse_args()


def split_entries(path):
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def load_session_label_names(metadata, errors):
    """Read the source label_names.txt and verify the conversion snapshot."""
    snapshot = metadata.get("label_names")
    if not isinstance(snapshot, list) or not all(isinstance(name, str) for name in snapshot):
        errors.append("metadata.label_names is not a string list")
        return []
    if len(snapshot) < 2 or snapshot[0] != "_background_":
        errors.append("metadata.label_names must start with _background_ and include a class")
        return []
    if len({name.casefold() for name in snapshot}) != len(snapshot):
        errors.append("metadata.label_names contains duplicate class names")
        return []

    source_value = metadata.get("label_names_source")
    if not isinstance(source_value, str) or source_value == "default":
        errors.append("metadata.label_names_source must name the source label_names.txt")
        return snapshot
    source = Path(source_value)
    if not source.is_file():
        errors.append(f"label_names source file is missing: {source}")
        return snapshot
    source_names = split_entries(source)
    if source_names != snapshot:
        errors.append(
            "source label_names.txt differs from metadata.label_names; "
            "do not reinterpret an existing converted session with changed class IDs"
        )
    source_sha256 = metadata.get("label_names_sha256")
    if source_sha256 is not None:
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if source_sha256 != digest:
            errors.append("source label_names.txt checksum differs from conversion metadata")
    return snapshot


def verify_provenance_file(metadata, path_key, hash_key, errors):
    value = metadata.get(path_key)
    expected = metadata.get(hash_key)
    if not isinstance(value, str) or not value:
        errors.append(f"metadata.{path_key} is missing")
        return
    path = Path(value)
    if not path.is_file():
        errors.append(f"provenance file is missing: {path}")
        return
    if not isinstance(expected, str) or len(expected) != 64:
        errors.append(f"metadata.{hash_key} is invalid")
        return
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        errors.append(f"{path_key} checksum differs from conversion metadata")


def label_id(label_names, target):
    target_normalized = target.casefold()
    for index, name in enumerate(label_names):
        if name.casefold() == target_normalized:
            return index
    return None


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def read_raw_scans(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in types]
    if missing:
        raise RuntimeError(f"bag is missing raw scan topics: {missing}")
    message_types = {topic: get_message(types[topic]) for topic in topics}
    scans = {topic: {} for topic in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            angles = (
                float(msg.angle_min)
                + np.arange(len(msg.ranges), dtype=np.float64) * float(msg.angle_increment)
            ).astype(np.float32)
            scans[topic][stamp_ns(msg)] = (
                np.asarray(msg.ranges, dtype=np.float32),
                angles,
                float(msg.range_min),
                float(msg.range_max),
            )
    return scans


def read_online_goal_contract(bag, metadata):
    topics = {
        metadata.get("global_path_topic", "/semantic_cnn/global_path"):
            ONLINE_GOAL_TYPES["/semantic_cnn/global_path"],
        metadata.get("local_subgoal_topic", "/semantic_cnn/local_subgoal"):
            ONLINE_GOAL_TYPES["/semantic_cnn/local_subgoal"],
        metadata.get("final_goal_topic", "/semantic_cnn/final_goal"):
            ONLINE_GOAL_TYPES["/semantic_cnn/final_goal"],
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in types]
    if missing:
        raise RuntimeError(f"bag is missing online goal topics: {missing}")
    wrong = [
        f"{topic}={types[topic]} (expected {expected})"
        for topic, expected in topics.items()
        if types[topic] != expected
    ]
    if wrong:
        raise RuntimeError("online goal topic type mismatch: " + "; ".join(wrong))

    message_types = {topic: get_message(types[topic]) for topic in topics}
    local_topic = metadata.get(
        "local_subgoal_topic", "/semantic_cnn/local_subgoal"
    )
    global_topic = metadata.get("global_path_topic", "/semantic_cnn/global_path")
    final_topic = metadata.get("final_goal_topic", "/semantic_cnn/final_goal")
    local_subgoals = []
    frames = {topic: set() for topic in topics}
    counts = Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        counts[topic] += 1
        frames[topic].add(msg.header.frame_id.lstrip("/"))
        if topic == local_topic:
            local_subgoals.append(
                (
                    stamp_ns(msg),
                    np.asarray(
                        [float(msg.point.x), float(msg.point.y)],
                        dtype=np.float32,
                    ),
                )
            )

    expected_frames = {
        local_topic: str(metadata.get("base_frame", "base_link")).lstrip("/"),
        global_topic: str(metadata.get("map_frame", "map")).lstrip("/"),
        final_topic: str(metadata.get("map_frame", "map")).lstrip("/"),
    }
    for topic, expected_frame in expected_frames.items():
        if counts[topic] == 0:
            raise RuntimeError(f"{topic} contains no messages")
        if frames[topic] != {expected_frame}:
            raise RuntimeError(
                f"{topic} frame(s) {sorted(frames[topic])} != {expected_frame}"
            )
    local_subgoals.sort(key=lambda item: item[0])
    return local_subgoals, counts


def read_episode_intervals(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get(topic) != EPISODE_EVENT_TYPE or "/clock" not in types:
        raise RuntimeError(
            f"{topic} or /clock has an invalid/missing type"
        )
    message_type = get_message(types[topic])
    clock_type = get_message(types["/clock"])
    events = []
    clocks = []
    while reader.has_next():
        current_topic, data, storage_time = reader.read_next()
        if current_topic == "/clock":
            msg = deserialize_message(data, clock_type)
            clocks.append(
                (
                    int(storage_time),
                    int(msg.clock.sec) * 1_000_000_000
                    + int(msg.clock.nanosec),
                )
            )
        elif current_topic == topic:
            msg = deserialize_message(data, message_type)
            try:
                event = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"{topic} contains invalid JSON") from exc
            if not isinstance(event, dict):
                raise RuntimeError(f"{topic} payload is not a JSON object")
            event["_storage_stamp_ns"] = int(storage_time)
            events.append(event)
    mapped, _audit = map_episode_events_to_sim_time(events, clocks)
    return episode_intervals_from_events(mapped)


def read_cmd_stamped(bag, topic="/cmd_vel_stamped"):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f"bag is missing {topic}")
    message_type = get_message(types[topic])
    commands = []
    while reader.has_next():
        current_topic, data, _ = reader.read_next()
        if current_topic != topic:
            continue
        msg = deserialize_message(data, message_type)
        commands.append(
            (
                stamp_ns(msg),
                np.asarray(
                    [
                        float(msg.twist.linear.x),
                        float(msg.twist.linear.y),
                        float(msg.twist.angular.z),
                    ],
                    dtype=np.float32,
                ),
            )
        )
    commands.sort(key=lambda item: item[0])
    return commands


def fixed_mask_from_calibration(calibration, key, beam_count, errors):
    runs_key = f"{key}_masked_beam_runs"
    indices_key = f"{key}_masked_beam_indices"
    count_key = f"{key}_masked_beam_count"
    mask = np.zeros(beam_count, dtype=np.bool_)
    runs = calibration.get(runs_key)
    if runs is not None:
        if not isinstance(runs, list):
            errors.append(f"self mask calibration {runs_key} is not a list")
            return None
        previous_end = -1
        for run in runs:
            if (
                not isinstance(run, list)
                or len(run) != 2
                or any(not isinstance(value, int) for value in run)
            ):
                errors.append(f"self mask calibration {runs_key} has an invalid run")
                return None
            start, end = run
            if start < 0 or end < start or end >= beam_count or start <= previous_end:
                errors.append(f"self mask calibration {runs_key} has invalid or overlapping runs")
                return None
            mask[start : end + 1] = True
            previous_end = end
    else:
        indices = calibration.get(indices_key)
        if not isinstance(indices, list) or any(not isinstance(value, int) for value in indices):
            errors.append(
                f"self mask calibration must provide {runs_key} or {indices_key}"
            )
            return None
        if len(set(indices)) != len(indices):
            errors.append(f"self mask calibration {indices_key} contains duplicates")
            return None
        if any(value < 0 or value >= beam_count for value in indices):
            errors.append(f"self mask calibration {indices_key} has out-of-range beam indices")
            return None
        mask[indices] = True
    if calibration.get(count_key) != int(mask.sum()):
        errors.append(f"self mask calibration {count_key} does not match stored beam mask")
        return None
    return mask


def expected_angles(layout):
    return (
        float(layout["angle_min"])
        + np.arange(int(layout["beam_count"]), dtype=np.float64)
        * float(layout["angle_increment"])
    ).astype(np.float32)


def range_valid_from_source(raw_ranges, range_min, range_max):
    """Mirror the converter's LaserScan validity contract from the source bag."""
    valid = np.isfinite(raw_ranges)
    if np.isfinite(range_min):
        valid &= raw_ranges >= range_min
    if np.isfinite(range_max):
        valid &= raw_ranges <= range_max
    return valid


def footprint_half_extents(calibration, errors):
    values = calibration.get("footprint_half_extents_m")
    if (
        not isinstance(values, list)
        or len(values) != 2
        or any(not isinstance(value, (int, float)) for value in values)
    ):
        errors.append("self mask calibration footprint_half_extents_m is invalid")
        return None
    extents = np.asarray(values, dtype=np.float64)
    if np.any(~np.isfinite(extents)) or np.any(extents <= 0.0):
        errors.append("self mask calibration footprint_half_extents_m must be positive")
        return None
    return extents


def main():
    args = parse_args()
    if min(
        args.minimum_samples,
        args.minimum_unique_command_vectors,
    ) < 0:
        raise ValueError("minimum count thresholds must be non-negative")
    if (
        not math.isfinite(args.minimum_duration_sec)
        or args.minimum_duration_sec < 0.0
    ):
        raise ValueError("minimum-duration-sec must be non-negative")
    if (
        not math.isfinite(args.minimum_nonzero_command_fraction)
        or not 0.0 <= args.minimum_nonzero_command_fraction <= 1.0
    ):
        raise ValueError(
            "minimum-nonzero-command-fraction must be in [0,1]"
        )
    if (
        not math.isfinite(args.minimum_effective_sample_rate_hz)
        or args.minimum_effective_sample_rate_hz < 0.0
    ):
        raise ValueError("minimum-effective-sample-rate-hz must be non-negative")
    if (
        not math.isfinite(args.minimum_person_positive_sample_fraction)
        or not 0.0
        <= args.minimum_person_positive_sample_fraction
        <= 1.0
    ):
        raise ValueError(
            "minimum-person-positive-sample-fraction must be in [0,1]"
        )
    if (
        args.maximum_subgoal_age_ms is not None
        and (
            not math.isfinite(args.maximum_subgoal_age_ms)
            or args.maximum_subgoal_age_ms < 0.0
        )
    ):
        raise ValueError("maximum-subgoal-age-ms must be non-negative")
    if (
        args.maximum_cmd_vel_age_ms is not None
        and (
            not math.isfinite(args.maximum_cmd_vel_age_ms)
            or args.maximum_cmd_vel_age_ms <= 0.0
        )
    ):
        raise ValueError("maximum-cmd-vel-age-ms must be positive and finite")
    if (
        args.maximum_person_truth_unmatched_samples is not None
        and args.maximum_person_truth_unmatched_samples < 0
    ):
        raise ValueError(
            "maximum-person-truth-unmatched-samples must be non-negative"
        )
    metadata_path = args.session / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(metadata_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    samples_01 = int(metadata["samples_01"])
    samples_02 = int(metadata["samples_02"])
    total = samples_01 + samples_02
    errors = []
    warnings = []
    label_names = load_session_label_names(metadata, errors)
    for path_key, hash_key in (
        ("map_yaml", "map_yaml_sha256"),
        ("semantic_label", "semantic_label_sha256"),
        ("occupancy_image", "occupancy_image_sha256"),
    ):
        verify_provenance_file(metadata, path_key, hash_key, errors)
    raw_subgoal_source = metadata.get("subgoal_source")
    if raw_subgoal_source in ("hindsight", "online"):
        subgoal_source = raw_subgoal_source
        current_subgoal_contract = True
    elif raw_subgoal_source is None or raw_subgoal_source == (
        "future robot pose transformed into the current base frame"
    ):
        subgoal_source = "hindsight"
        current_subgoal_contract = False
    else:
        subgoal_source = str(raw_subgoal_source)
        current_subgoal_contract = False
        errors.append(f"unknown subgoal_source: {raw_subgoal_source!r}")
    if args.require_online_subgoal and subgoal_source != "online":
        errors.append("quality gate requires online causal subgoals")
    if args.require_pre_relay_command_labels:
        if metadata.get("cmd_label_interface") != "pre-relay_ros_cmd_vel":
            errors.append(
                "quality gate requires pre-relay ROS /cmd_vel labels"
            )
        relay_scale = metadata.get("cmd_vel_angular_z_relay_scale")
        if (
            not isinstance(relay_scale, (int, float))
            or not math.isfinite(float(relay_scale))
            or float(relay_scale) <= 0.0
        ):
            errors.append("command-label relay scale is missing or invalid")
    person_label_mode = metadata.get("person_label_mode")
    if person_label_mode not in (
        "ground-truth-legs",
        "ground-truth-radius",
        "dynamic",
        "disabled",
    ):
        errors.append(f"unknown person_label_mode: {person_label_mode!r}")
    if (
        args.require_ground_truth_person_labels
        and person_label_mode != "ground-truth-legs"
    ):
        errors.append(
            "quality gate requires ground-truth-legs Person labels"
        )
    configured_person_label_id = label_id(label_names, "Person")
    if person_label_mode != "disabled" and configured_person_label_id is None:
        errors.append(
            "enabled Person labeling requires a Person entry in label_names.txt"
        )
    person_label_id = (
        configured_person_label_id if person_label_mode != "disabled" else None
    )
    required_contracts = (
        "no_resampling",
        "no_cross_sensor_deduplication",
        "no_angle_binning",
        "no_scan_fusion",
    )
    for key in required_contracts:
        if metadata.get(key) is not True:
            errors.append(f"metadata.{key} is not true")
    if metadata.get("uses_scan_merged") is not False:
        errors.append("metadata.uses_scan_merged is not false")
    reverse_motion_filter = metadata.get("reverse_motion_filter")
    if reverse_motion_filter is None:
        reverse_motion_filter = {"enabled": False, "policy": "legacy-unspecified"}
    elif not isinstance(reverse_motion_filter, dict):
        errors.append("metadata.reverse_motion_filter is not an object")
        reverse_motion_filter = {"enabled": False, "policy": "invalid"}
    reverse_filter_enabled = reverse_motion_filter.get("enabled") is True
    if args.require_forward_only and not reverse_filter_enabled:
        errors.append("quality gate requires forward-only recovery filtering")
    reverse_filter_epsilon = reverse_motion_filter.get("linear_x_epsilon")
    if reverse_filter_enabled:
        if (
            not isinstance(reverse_filter_epsilon, (int, float))
            or not np.isfinite(reverse_filter_epsilon)
            or reverse_filter_epsilon <= 0.0
        ):
            errors.append(
                "enabled reverse_motion_filter has invalid linear_x_epsilon"
            )
            reverse_filter_epsilon = 0.0
        reverse_removed = reverse_motion_filter.get("reverse_frames_removed")
        recovery_removed = reverse_motion_filter.get("recovery_frames_removed")
        total_removed = reverse_motion_filter.get("total_frames_removed")
        if any(
            not isinstance(value, int) or value < 0
            for value in (reverse_removed, recovery_removed, total_removed)
        ):
            errors.append("reverse_motion_filter removal counts are invalid")
        elif total_removed != reverse_removed + recovery_removed:
            errors.append(
                "reverse_motion_filter total does not equal reverse plus recovery"
            )
        drop_counts = metadata.get("drop_counts", {})
        if (
            int(drop_counts.get("reverse_command", -1)) != reverse_removed
            or int(drop_counts.get("reverse_recovery", -1)) != recovery_removed
        ):
            errors.append(
                "reverse_motion_filter counts differ from metadata.drop_counts"
            )
    if metadata.get("sensor_extrinsic_transform") != "full 3D quaternion TF, then XY projection":
        errors.append("metadata does not confirm the full 3D sensor extrinsic transform")
    if int(metadata.get("total_slots", -1)) != total:
        errors.append("metadata.total_slots does not equal samples_01 + samples_02")
    preview_files = metadata.get("projection_debug_files", [])
    if len(preview_files) != 3 or any(not (args.session / path).is_file() for path in preview_files):
        errors.append("three projection debug previews are not available")

    sample_files = sorted((args.session / "samples").glob("*.npz"))
    if len(sample_files) != int(metadata.get("samples", -1)):
        errors.append(
            f"sample count {len(sample_files)} != metadata {metadata.get('samples')}"
        )
    expected_sensor = np.concatenate(
        (np.zeros(samples_01, dtype=np.uint8), np.ones(samples_02, dtype=np.uint8))
    )
    expected_beam = np.concatenate(
        (np.arange(samples_01, dtype=np.int32), np.arange(samples_02, dtype=np.int32))
    )
    self_mask_mode = metadata.get("self_mask_mode", "per-frame-footprint-legacy")
    fixed_self_mask = None
    expected_angles_01 = None
    expected_angles_02 = None
    calibration = None
    calibration_footprint_extents = None
    if self_mask_mode == "first-synchronized-pair-fixed-beam-identity":
        calibration = metadata.get("self_mask_calibration")
        if not isinstance(calibration, dict):
            errors.append("fixed self-mask mode is missing self_mask_calibration")
        else:
            first_mask = fixed_mask_from_calibration(
                calibration, "scan_01", samples_01, errors
            )
            second_mask = fixed_mask_from_calibration(
                calibration, "scan_02", samples_02, errors
            )
            if first_mask is not None and second_mask is not None:
                fixed_self_mask = np.concatenate((first_mask, second_mask))
            if calibration.get("source_sample_index") != 0:
                errors.append("fixed self-mask calibration source_sample_index is not 0")
            calibration_footprint_extents = footprint_half_extents(calibration, errors)
        layouts = (metadata.get("scan_01_layout"), metadata.get("scan_02_layout"))
        for name, layout, expected_count in zip(
            ("scan_01", "scan_02"), layouts, (samples_01, samples_02)
        ):
            if not isinstance(layout, dict):
                errors.append(f"fixed self-mask mode is missing {name}_layout")
            elif int(layout.get("beam_count", -1)) != expected_count:
                errors.append(f"{name}_layout beam_count does not match metadata")
        if all(isinstance(layout, dict) for layout in layouts):
            expected_angles_01 = expected_angles(layouts[0])
            expected_angles_02 = expected_angles(layouts[1])
    elif self_mask_mode not in ("per-frame-footprint", "per-frame-footprint-legacy"):
        errors.append(f"unknown self_mask_mode: {self_mask_mode!r}")
    label_hist = Counter()
    sync_deltas = []
    cmd_nonzero = 0
    negative_linear_x_samples = 0
    valid_counts_01, valid_counts_02 = [], []
    previous_stamp = None
    source_bag = Path(metadata["bag"])
    raw_scans = read_raw_scans(
        source_bag,
        (metadata["scan_01_topic"], metadata["scan_02_topic"]),
    )
    source_commands = read_cmd_stamped(source_bag)
    source_command_times = [item[0] for item in source_commands]
    checked_cmd_ages_ns = []
    online_subgoals = []
    online_subgoal_times = []
    online_goal_counts = Counter()
    online_subgoal_max_age_ns = -1
    if subgoal_source == "online":
        max_age_value = metadata.get("subgoal_max_age_ms")
        if not isinstance(max_age_value, (int, float)) or max_age_value < 0:
            errors.append("metadata.subgoal_max_age_ms must be non-negative")
        else:
            online_subgoal_max_age_ns = int(
                round(float(max_age_value) * 1_000_000.0)
            )
        try:
            online_subgoals, online_goal_counts = read_online_goal_contract(
                source_bag, metadata
            )
            online_subgoal_times = [item[0] for item in online_subgoals]
        except RuntimeError as exc:
            errors.append(str(exc))
    raw_fidelity_samples = 0
    positions = []
    cmd_histogram = Counter()
    person_distances = []
    person_truth_unmatched_samples = 0
    person_positive_samples = 0
    first_stamp = None
    last_stamp = None
    checked_subgoal_ages_ns = []
    episode_count = int(metadata.get("episode_count", 0))
    episode_metadata = metadata.get("episodes", [])
    episode_filter = metadata.get("episode_filter", {})
    if (
        args.require_successful_episodes_only
        and episode_filter.get("mode") != "successful_only"
    ):
        errors.append(
            "quality gate requires successful-episodes-only conversion"
        )
    if args.require_successful_episodes_only:
        if episode_count <= 0:
            errors.append(
                "quality gate requires at least one selected successful episode"
            )
        if episode_filter.get("selected_episode_count") != episode_count:
            errors.append(
                "episode_filter.selected_episode_count differs from episode_count"
            )
    episode_intervals = []
    episode_sample_counts = Counter()
    episode_sample_stamps = {}
    sample_episode_by_name = {}
    if episode_count:
        if not isinstance(episode_metadata, list) or len(episode_metadata) != episode_count:
            errors.append("metadata.episodes does not match episode_count")
            episode_metadata = []
        if args.require_successful_episodes_only:
            for episode in episode_metadata:
                if episode.get("finish_reason") not in (
                    SUCCESSFUL_EPISODE_FINISH_REASONS
                ):
                    errors.append(
                        "successful-only dataset contains metadata for an "
                        "unsuccessful episode"
                    )
        try:
            episode_intervals = read_episode_intervals(
                source_bag,
                metadata.get(
                    "episode_event_topic", "/data_collection/episode_event"
                ),
            )
            if episode_filter.get("mode") == "successful_only":
                episode_intervals = [
                    interval
                    for interval in episode_intervals
                    if interval.get("finish_reason")
                    in SUCCESSFUL_EPISODE_FINISH_REASONS
                ]
        except (KeyError, RuntimeError, TypeError, ValueError) as exc:
            errors.append(str(exc))
        if len(episode_intervals) != episode_count:
            errors.append("source bag episode count differs from metadata")
        else:
            for source, converted in zip(episode_intervals, episode_metadata):
                for key in (
                    "episode_id",
                    "start_stamp_ns",
                    "end_stamp_ns",
                    "goal",
                    "finish_reason",
                ):
                    if converted.get(key) != source.get(key):
                        errors.append(
                            f"metadata episode {source['episode_id']} {key} "
                            "differs from source bag"
                        )
    for sample_index, path in enumerate(sample_files):
        with np.load(path) as sample:
            missing = [field for field in ARRAY_FIELDS if field not in sample]
            if missing:
                errors.append(f"{path.name}: missing {missing}")
                continue
            for field in ARRAY_FIELDS:
                if sample[field].shape != (total,):
                    errors.append(
                        f"{path.name}: {field} shape {sample[field].shape} != {(total,)}"
                    )
            if not np.array_equal(sample["source_sensor"], expected_sensor):
                errors.append(f"{path.name}: source_sensor contract mismatch")
            if not np.array_equal(sample["raw_beam_index"], expected_beam):
                errors.append(f"{path.name}: raw_beam_index contract mismatch")
            valid = sample["valid_mask"].astype(bool)
            range_valid = sample["range_valid_mask"].astype(bool)
            self_mask = sample["self_mask"].astype(bool)
            if fixed_self_mask is not None and not np.array_equal(self_mask, fixed_self_mask):
                errors.append(f"{path.name}: self mask differs from the fixed calibration")
            if not np.array_equal(valid, range_valid & ~self_mask):
                errors.append(f"{path.name}: valid mask contract mismatch")
            if sample_index == 0 and calibration_footprint_extents is not None:
                points_x = sample["points_x_base"]
                points_y = sample["points_y_base"]
                if np.any(~np.isfinite(points_x[range_valid])) or np.any(
                    ~np.isfinite(points_y[range_valid])
                ):
                    errors.append(
                        f"{path.name}: range-valid raw beams have non-finite base points"
                    )
                expected_first_self = range_valid & (
                    np.abs(points_x) <= calibration_footprint_extents[0]
                ) & (np.abs(points_y) <= calibration_footprint_extents[1])
                if not np.array_equal(self_mask, expected_first_self):
                    errors.append(
                        f"{path.name}: fixed self mask is not derived from the first "
                        "frame footprint calibration"
                    )
            virtual_ranges = sample["virtual_ranges"]
            virtual_angles = sample["virtual_angles"]
            if np.any(~np.isfinite(virtual_ranges[valid])) or np.any(
                ~np.isfinite(virtual_angles[valid])
            ):
                errors.append(f"{path.name}: valid slots have non-finite virtual coordinates")
            if np.any(~np.isnan(virtual_ranges[~valid])) or np.any(
                ~np.isnan(virtual_angles[~valid])
            ):
                errors.append(f"{path.name}: invalid slots do not use NaN virtual coordinates")
            semantic = sample["semantic_label"]
            if np.any(semantic[~valid] != -1):
                errors.append(f"{path.name}: invalid slots have non-ignore labels")
            allowed_max = len(label_names) - 1
            if np.any((semantic < -1) | (semantic > allowed_max)):
                errors.append(f"{path.name}: semantic label outside [-1,{allowed_max}]")
            if person_label_mode in ("ground-truth-legs", "ground-truth-radius"):
                ground_truth_fields = (
                    "pedestrian_ids",
                    "pedestrian_xy_map",
                    "pedestrian_velocity_map",
                    "pedestrian_truth_stamp_ns",
                    "person_nearest_distance_m",
                )
                if person_label_mode == "ground-truth-legs":
                    ground_truth_fields += (
                        "pedestrian_yaw_map",
                        "pedestrian_leg_xy_map",
                    )
                missing_truth = [
                    field for field in ground_truth_fields if field not in sample
                ]
                if missing_truth:
                    errors.append(
                        f"{path.name}: missing ground-truth audit fields {missing_truth}"
                    )
                else:
                    pedestrian_count = len(sample["pedestrian_ids"])
                    pedestrian_shapes = {
                        "pedestrian_xy_map": (pedestrian_count, 2),
                        "pedestrian_velocity_map": (pedestrian_count, 2),
                    }
                    if person_label_mode == "ground-truth-legs":
                        pedestrian_shapes.update(
                            {
                                "pedestrian_yaw_map": (pedestrian_count,),
                                "pedestrian_leg_xy_map": (pedestrian_count, 2, 2),
                            }
                        )
                    for field, expected_shape in pedestrian_shapes.items():
                        if sample[field].shape != expected_shape:
                            errors.append(
                                f"{path.name}: {field} shape {sample[field].shape} "
                                f"!= {expected_shape}"
                            )
                        elif np.any(~np.isfinite(sample[field])):
                            errors.append(
                                f"{path.name}: {field} contains non-finite values"
                            )
                    distances = sample["person_nearest_distance_m"]
                    if distances.shape != (total,):
                        errors.append(
                            f"{path.name}: person_nearest_distance_m shape "
                            f"{distances.shape} != {(total,)}"
                        )
                    person_mask = semantic == person_label_id
                    radius_key = (
                        "person_ground_truth_leg_match_radius_m"
                        if person_label_mode == "ground-truth-legs"
                        else "person_ground_truth_radius_m"
                    )
                    radius = float(metadata[radius_key])
                    if np.any(~np.isfinite(distances[person_mask])):
                        errors.append(
                            f"{path.name}: Person slots lack a finite truth distance"
                        )
                    if np.any(distances[person_mask] > radius + 1e-6):
                        errors.append(
                            f"{path.name}: Person slot exceeds truth radius {radius} m"
                        )
                    if int(sample["pedestrian_truth_stamp_ns"]) < 0:
                        person_truth_unmatched_samples += 1
                        if np.any(person_mask):
                            errors.append(
                                f"{path.name}: unmatched truth sample contains Person labels"
                            )
                    person_distances.extend(distances[person_mask].tolist())
            if sample["position"].shape != (3,):
                errors.append(f"{path.name}: position shape is not (3,)")
            if sample["velocity"].shape != (2,):
                errors.append(f"{path.name}: velocity shape is not (2,)")
            if sample["cmd_velocity"].shape != (3,):
                errors.append(f"{path.name}: cmd_velocity shape is not (3,)")
            cmd_audit_fields = ("cmd_vel_stamp_ns", "cmd_vel_age_ns")
            missing_cmd_audit = [
                field for field in cmd_audit_fields if field not in sample
            ]
            if missing_cmd_audit:
                errors.append(
                    f"{path.name}: missing command audit fields {missing_cmd_audit}"
                )
            if sample["sub_goal_local"].shape != (2,):
                errors.append(f"{path.name}: sub_goal_local shape is not (2,)")
            stamp_01 = int(sample["scan_01_stamp_ns"])
            stamp_02 = int(sample["scan_02_stamp_ns"])
            if not missing_cmd_audit:
                match_index = bisect.bisect_right(
                    source_command_times, stamp_01
                ) - 1
                if match_index < 0:
                    errors.append(
                        f"{path.name}: no causal /cmd_vel_stamped is available"
                    )
                else:
                    expected_cmd_stamp, expected_cmd = source_commands[match_index]
                    cmd_stamp = int(sample["cmd_vel_stamp_ns"])
                    cmd_age = int(sample["cmd_vel_age_ns"])
                    if cmd_stamp != expected_cmd_stamp:
                        errors.append(
                            f"{path.name}: command is not the causal hold-last source"
                        )
                    if cmd_stamp > stamp_01 or cmd_age != stamp_01 - cmd_stamp:
                        errors.append(
                            f"{path.name}: command timestamp/age is not causal"
                        )
                    if not np.array_equal(
                        sample["cmd_velocity"].astype(np.float32), expected_cmd
                    ):
                        errors.append(
                            f"{path.name}: command differs from source bag"
                        )
                    checked_cmd_ages_ns.append(cmd_age)
            if episode_count:
                if "episode_id" not in sample:
                    errors.append(f"{path.name}: missing episode_id")
                else:
                    episode_id = int(sample["episode_id"])
                    matching = [
                        interval
                        for interval in episode_intervals
                        if int(interval["episode_id"]) == episode_id
                    ]
                    if len(matching) != 1:
                        errors.append(
                            f"{path.name}: unknown episode_id {episode_id}"
                        )
                    elif not (
                        matching[0]["start_stamp_ns"]
                        <= stamp_01
                        <= matching[0]["end_stamp_ns"]
                    ):
                        errors.append(
                            f"{path.name}: scan is outside episode {episode_id}"
                        )
                    episode_sample_counts[episode_id] += 1
                    episode_sample_stamps.setdefault(episode_id, []).append(
                        stamp_01
                    )
                    sample_episode_by_name[path.name] = episode_id
            if current_subgoal_contract:
                audit_fields = (
                    "local_subgoal_stamp_ns",
                    "local_subgoal_age_ns",
                )
                missing_audit = [
                    field for field in audit_fields if field not in sample
                ]
                if missing_audit:
                    errors.append(
                        f"{path.name}: missing subgoal audit fields {missing_audit}"
                    )
                else:
                    subgoal_stamp_ns = int(sample["local_subgoal_stamp_ns"])
                    subgoal_age_ns = int(sample["local_subgoal_age_ns"])
                    if subgoal_source == "hindsight":
                        if subgoal_stamp_ns != -1 or subgoal_age_ns != -1:
                            errors.append(
                                f"{path.name}: hindsight subgoal audit stamps must be -1"
                            )
                    elif online_subgoals:
                        match_index = (
                            bisect.bisect_right(online_subgoal_times, stamp_01) - 1
                        )
                        if match_index < 0:
                            errors.append(
                                f"{path.name}: no causal online subgoal is available"
                            )
                        else:
                            expected_stamp, expected_xy = online_subgoals[match_index]
                            expected_age = stamp_01 - expected_stamp
                            if subgoal_stamp_ns != expected_stamp:
                                errors.append(
                                    f"{path.name}: online subgoal is not the causal "
                                    "hold-last source message"
                                )
                            if subgoal_stamp_ns > stamp_01:
                                errors.append(
                                    f"{path.name}: online subgoal comes from the future"
                                )
                            if subgoal_age_ns != expected_age:
                                errors.append(
                                    f"{path.name}: online subgoal age does not match stamps"
                                )
                            if (
                                expected_age < 0
                                or expected_age > online_subgoal_max_age_ns
                            ):
                                errors.append(
                                    f"{path.name}: online subgoal age "
                                    f"{expected_age / 1_000_000.0:.3f} ms exceeds "
                                    f"{online_subgoal_max_age_ns / 1_000_000.0:.3f} "
                                    "ms contract"
                                )
                            if not np.allclose(
                                sample["sub_goal_local"],
                                expected_xy,
                                rtol=0.0,
                                atol=1e-6,
                            ):
                                errors.append(
                                    f"{path.name}: sub_goal_local differs from source bag"
                                )
                            checked_subgoal_ages_ns.append(expected_age)
            if sample_index == 0 and calibration is not None:
                if stamp_01 != calibration.get("scan_01_stamp_ns"):
                    errors.append("first sample scan_01 timestamp differs from self-mask calibration")
                if stamp_02 != calibration.get("scan_02_stamp_ns"):
                    errors.append("first sample scan_02 timestamp differs from self-mask calibration")
            if expected_angles_01 is not None:
                if not np.allclose(
                    sample["raw_angles_sensor"][:samples_01],
                    expected_angles_01,
                    rtol=0.0,
                    atol=1e-6,
                ):
                    errors.append(f"{path.name}: scan_01 beam layout changed after calibration")
                if not np.allclose(
                    sample["raw_angles_sensor"][samples_01:],
                    expected_angles_02,
                    rtol=0.0,
                    atol=1e-6,
                ):
                    errors.append(f"{path.name}: scan_02 beam layout changed after calibration")
            first_stamp = stamp_01 if first_stamp is None else min(first_stamp, stamp_01)
            last_stamp = stamp_01 if last_stamp is None else max(last_stamp, stamp_01)
            if previous_stamp is not None and stamp_01 <= previous_stamp:
                errors.append(f"{path.name}: scan_01 stamps are not strictly increasing")
            previous_stamp = stamp_01
            expected_01 = raw_scans[metadata["scan_01_topic"]].get(stamp_01)
            expected_02 = raw_scans[metadata["scan_02_topic"]].get(stamp_02)
            if expected_01 is None or expected_02 is None:
                errors.append(f"{path.name}: source bag scan timestamp is missing")
            else:
                checks = (
                    (sample["raw_ranges"][:samples_01], expected_01[0], "scan_01 ranges"),
                    (sample["raw_ranges"][samples_01:], expected_02[0], "scan_02 ranges"),
                    (sample["raw_angles_sensor"][:samples_01], expected_01[1], "scan_01 angles"),
                    (sample["raw_angles_sensor"][samples_01:], expected_02[1], "scan_02 angles"),
                )
                for actual, expected, label in checks:
                    if actual.shape != expected.shape or not np.allclose(
                        actual, expected, rtol=0.0, atol=0.0, equal_nan=True
                    ):
                        errors.append(f"{path.name}: raw fidelity mismatch for {label}")
                expected_range_valid = np.concatenate(
                    (
                        range_valid_from_source(
                            expected_01[0], expected_01[2], expected_01[3]
                        ),
                        range_valid_from_source(
                            expected_02[0], expected_02[2], expected_02[3]
                        ),
                    )
                )
                if not np.array_equal(range_valid, expected_range_valid):
                    errors.append(
                        f"{path.name}: range_valid_mask does not match source LaserScan ranges"
                    )
                raw_fidelity_samples += 1
            sync_deltas.append(abs(stamp_02 - stamp_01) / 1_000_000.0)
            label_hist.update(int(value) for value in semantic.tolist())
            if person_label_id is not None:
                person_positive_samples += int(
                    np.any(semantic == person_label_id)
                )
            cmd_nonzero += int(np.any(np.abs(sample["cmd_velocity"]) > 1e-6))
            negative_linear_x_samples += int(
                float(sample["cmd_velocity"][0])
                < -float(reverse_filter_epsilon or 0.0)
            )
            cmd_histogram[tuple(float(value) for value in sample["cmd_velocity"])] += 1
            positions.append(sample["position"].astype(np.float64))
            valid_counts_01.append(int(valid[:samples_01].sum()))
            valid_counts_02.append(int(valid[samples_01:].sum()))

    if subgoal_source == "online":
        if int(metadata.get("subgoal_matched_samples", -1)) != len(sample_files):
            errors.append(
                "metadata.subgoal_matched_samples does not equal sample count"
            )
        leading_dropped = metadata.get(
            "subgoal_unmatched_leading_frames_dropped"
        )
        if not isinstance(leading_dropped, int) or leading_dropped < 0:
            errors.append(
                "metadata.subgoal_unmatched_leading_frames_dropped is invalid"
            )
        stale_expected = int(
            metadata.get("drop_counts", {}).get("stale_subgoal", 0)
        )
        if int(metadata.get("subgoal_stale_samples", -1)) != stale_expected:
            errors.append(
                "metadata.subgoal_stale_samples differs from drop_counts"
            )
        if len(checked_subgoal_ages_ns) != len(sample_files):
            errors.append("not every sample passed online subgoal verification")
        if checked_subgoal_ages_ns:
            recomputed_age_ms = {
                "subgoal_age_ms_min": min(checked_subgoal_ages_ns) / 1_000_000.0,
                "subgoal_age_ms_max": max(checked_subgoal_ages_ns) / 1_000_000.0,
                "subgoal_age_ms_mean": (
                    float(np.mean(checked_subgoal_ages_ns)) / 1_000_000.0
                ),
            }
            for key, expected in recomputed_age_ms.items():
                actual = metadata.get(key)
                if not isinstance(actual, (int, float)) or not np.isclose(
                    float(actual), expected, rtol=0.0, atol=1e-9
                ):
                    errors.append(f"metadata.{key} does not match sample audit")
            if (
                args.maximum_subgoal_age_ms is not None
                and max(checked_subgoal_ages_ns) / 1_000_000.0
                > args.maximum_subgoal_age_ms + 1e-9
            ):
                errors.append(
                    "online subgoal age exceeds the configured quality gate"
                )
        for key, topic in (
            ("local_subgoal_messages", metadata.get(
                "local_subgoal_topic", "/semantic_cnn/local_subgoal"
            )),
            ("global_path_messages", metadata.get(
                "global_path_topic", "/semantic_cnn/global_path"
            )),
            ("final_goal_messages", metadata.get(
                "final_goal_topic", "/semantic_cnn/final_goal"
            )),
        ):
            if online_goal_counts and int(metadata.get(key, -1)) != int(
                online_goal_counts[topic]
            ):
                errors.append(f"metadata.{key} differs from source bag")

    if reverse_filter_enabled and negative_linear_x_samples:
        errors.append(
            "forward-only dataset still contains "
            f"{negative_linear_x_samples} negative linear.x sample(s)"
        )

    if episode_count:
        expected_ids = {
            int(interval["episode_id"]) for interval in episode_intervals
        }
        if set(episode_sample_counts) != expected_ids:
            errors.append("not every complete episode has converted samples")
        for episode in episode_metadata:
            episode_id = int(episode.get("episode_id", -1))
            if int(episode.get("sample_count", -1)) != episode_sample_counts[episode_id]:
                errors.append(
                    f"metadata episode {episode_id} sample_count is incorrect"
                )
        if metadata.get("split_unit") != "episode":
            errors.append("episodic dataset split_unit is not 'episode'")

    splits = {}
    for name in ("train", "dev", "test"):
        path = args.session / f"{name}.txt"
        if not path.is_file():
            errors.append(f"missing {path.name}")
            splits[name] = []
        else:
            splits[name] = split_entries(path)
    split_sets = {name: set(values) for name, values in splits.items()}
    if split_sets["train"] & split_sets["dev"] or split_sets["train"] & split_sets["test"] or split_sets["dev"] & split_sets["test"]:
        errors.append("train/dev/test splits overlap")
    if set().union(*split_sets.values()) != {path.name for path in sample_files}:
        errors.append("train/dev/test union does not match sample files")
    if episode_count:
        split_episode_ids = {
            name: {
                sample_episode_by_name[value]
                for value in values
                if value in sample_episode_by_name
            }
            for name, values in splits.items()
        }
        if (
            split_episode_ids["train"] & split_episode_ids["dev"]
            or split_episode_ids["train"] & split_episode_ids["test"]
            or split_episode_ids["dev"] & split_episode_ids["test"]
        ):
            errors.append("an episode is split across train/dev/test")

    valid_total = sum(valid_counts_01) + sum(valid_counts_02)
    person_count = (
        int(label_hist.get(person_label_id, 0))
        if person_label_id is not None
        else None
    )
    person_fraction = (
        person_count / valid_total
        if person_count is not None and valid_total
        else None
    )
    duration_sec = (
        (last_stamp - first_stamp) / 1_000_000_000.0
        if first_stamp is not None and last_stamp is not None
        else 0.0
    )
    effective_duration_sec = (
        sum(
            (max(stamps) - min(stamps)) / 1_000_000_000.0
            for stamps in episode_sample_stamps.values()
            if stamps
        )
        if episode_count
        else duration_sec
    )
    nonzero_command_fraction = (
        cmd_nonzero / len(sample_files) if sample_files else 0.0
    )
    effective_sample_rate_hz = (
        len(sample_files) / effective_duration_sec
        if effective_duration_sec > 0.0
        else 0.0
    )
    person_positive_sample_fraction = (
        person_positive_samples / len(sample_files) if sample_files else 0.0
    )
    if len(sample_files) < args.minimum_samples:
        errors.append(
            f"sample count {len(sample_files)} is below required "
            f"{args.minimum_samples}"
        )
    if effective_duration_sec + 1e-9 < args.minimum_duration_sec:
        errors.append(
            f"effective simulation span {effective_duration_sec:.3f}s is below required "
            f"{args.minimum_duration_sec:.3f}s"
        )
    if len(cmd_histogram) < args.minimum_unique_command_vectors:
        errors.append(
            f"unique command count {len(cmd_histogram)} is below required "
            f"{args.minimum_unique_command_vectors}"
        )
    if (
        nonzero_command_fraction + 1e-12
        < args.minimum_nonzero_command_fraction
    ):
        errors.append(
            f"nonzero command fraction {nonzero_command_fraction:.3f} is "
            "below the configured quality gate"
        )
    if args.require_person_observations and not person_count:
        errors.append("quality gate requires at least one Person-labeled slot")
    if (
        effective_sample_rate_hz + 1e-12
        < args.minimum_effective_sample_rate_hz
    ):
        errors.append(
            f"effective sample rate {effective_sample_rate_hz:.3f} Hz is below required "
            f"{args.minimum_effective_sample_rate_hz:.3f} Hz"
        )
    if (
        person_positive_sample_fraction + 1e-12
        < args.minimum_person_positive_sample_fraction
    ):
        errors.append(
            "Person-positive sample fraction "
            f"{person_positive_sample_fraction:.3f} is below the configured quality gate"
        )
    if args.maximum_cmd_vel_age_ms is not None:
        configured_cmd_age = metadata.get("cmd_vel_max_age_ms")
        if (
            not isinstance(configured_cmd_age, (int, float))
            or not math.isfinite(float(configured_cmd_age))
            or float(configured_cmd_age) > args.maximum_cmd_vel_age_ms + 1e-9
        ):
            errors.append(
                "converter command-age limit is missing or weaker than the quality gate"
            )
        maximum_cmd_age_ns = int(
            round(args.maximum_cmd_vel_age_ms * 1_000_000.0)
        )
        if checked_cmd_ages_ns and max(checked_cmd_ages_ns) > maximum_cmd_age_ns:
            errors.append(
                "a causal command label exceeds the configured maximum age"
            )
    if (
        args.maximum_person_truth_unmatched_samples is not None
        and person_truth_unmatched_samples
        > args.maximum_person_truth_unmatched_samples
    ):
        errors.append(
            "unmatched pedestrian-truth sample count "
            f"{person_truth_unmatched_samples} exceeds required maximum "
            f"{args.maximum_person_truth_unmatched_samples}"
        )
    warning_sample_floor = (
        args.minimum_samples if args.minimum_samples > 0 else 500
    )
    warning_duration_floor = (
        args.minimum_duration_sec
        if args.minimum_duration_sec > 0.0
        else 60.0
    )
    if len(sample_files) < warning_sample_floor:
        warnings.append(
            f"only {len(sample_files)} samples; configured training-corpus floor is "
            f"{warning_sample_floor}"
        )
    if effective_duration_sec + 1e-9 < warning_duration_floor:
        warnings.append(
            f"only {effective_duration_sec:.3f}s of episode-contained simulation time; "
            f"configured training-corpus floor is {warning_duration_floor:.3f}s"
        )
    if person_fraction is not None and person_fraction > 0.25:
        warnings.append(
            f"Person occupies {person_fraction:.1%} of valid slots; inspect TF/map alignment"
        )
    if args.fail_on_warnings and warnings:
        errors.extend(f"strict warning: {warning}" for warning in warnings)
    path_length = 0.0
    if len(positions) > 1:
        position_array = np.stack(positions)
        path_length = float(np.linalg.norm(np.diff(position_array[:, :2], axis=0), axis=1).sum())

    report = {
        "status": "PASS_WITH_WARNINGS" if not errors and warnings else ("PASS" if not errors else "FAIL"),
        "session": str(args.session.resolve()),
        "samples": len(sample_files),
        "samples_01": samples_01,
        "samples_02": samples_02,
        "total_slots": total,
        "split_counts": {name: len(values) for name, values in splits.items()},
        "sync_delta_ms_max": max(sync_deltas) if sync_deltas else None,
        "simulation_span_sec": duration_sec,
        "effective_episode_span_sec": effective_duration_sec,
        "effective_sample_rate_hz": effective_sample_rate_hz,
        "episode_count": episode_count,
        "episode_filter": episode_filter,
        "path_length_m": path_length,
        "raw_fidelity_samples_checked": raw_fidelity_samples,
        "projection_debug_files": preview_files,
        "subgoal_source": subgoal_source,
        "subgoal_matched_samples": (
            len(checked_subgoal_ages_ns)
            if subgoal_source == "online"
            else 0
        ),
        "subgoal_unmatched_leading_frames_dropped": metadata.get(
            "subgoal_unmatched_leading_frames_dropped", 0
        ),
        "subgoal_age_ms": (
            {
                "min": min(checked_subgoal_ages_ns) / 1_000_000.0,
                "mean": float(np.mean(checked_subgoal_ages_ns)) / 1_000_000.0,
                "max": max(checked_subgoal_ages_ns) / 1_000_000.0,
            }
            if checked_subgoal_ages_ns
            else None
        ),
        "cmd_nonzero_samples": cmd_nonzero,
        "cmd_nonzero_fraction": nonzero_command_fraction,
        "cmd_label_interface": metadata.get("cmd_label_interface"),
        "cmd_vel_angular_z_relay_scale": metadata.get(
            "cmd_vel_angular_z_relay_scale"
        ),
        "cmd_vel_age_ms": (
            {
                "min": min(checked_cmd_ages_ns) / 1_000_000.0,
                "mean": float(np.mean(checked_cmd_ages_ns)) / 1_000_000.0,
                "max": max(checked_cmd_ages_ns) / 1_000_000.0,
            }
            if checked_cmd_ages_ns
            else None
        ),
        "negative_linear_x_samples": negative_linear_x_samples,
        "reverse_motion_filter": reverse_motion_filter,
        "cmd_unique_vectors": len(cmd_histogram),
        "cmd_histogram": {
            str(key): int(value) for key, value in sorted(cmd_histogram.items())
        },
        "average_valid_scan_01": float(np.mean(valid_counts_01)) if valid_counts_01 else None,
        "average_valid_scan_02": float(np.mean(valid_counts_02)) if valid_counts_02 else None,
        "self_mask_mode": self_mask_mode,
        "fixed_self_mask_scan_01_count": (
            int(fixed_self_mask[:samples_01].sum())
            if fixed_self_mask is not None
            else None
        ),
        "fixed_self_mask_scan_02_count": (
            int(fixed_self_mask[samples_01:].sum())
            if fixed_self_mask is not None
            else None
        ),
        "fixed_self_mask_first_frame_footprint_audited": (
            calibration_footprint_extents is not None
        ),
        "label_histogram_recomputed": {
            str(key): int(value) for key, value in sorted(label_hist.items())
        },
        "label_histogram_by_name": {
            f"{index}:{name}": int(label_hist.get(index, 0))
            for index, name in enumerate(label_names)
        },
        "label_names_source": metadata.get("label_names_source"),
        "label_names": label_names,
        "person_label_mode": person_label_mode,
        "person_label_id": person_label_id,
        "person_label_count": person_count,
        "person_positive_samples": person_positive_samples,
        "person_positive_sample_fraction": person_positive_sample_fraction,
        "person_fraction_of_valid_slots": person_fraction,
        "person_ground_truth_distance_m": (
            {
                "min": float(np.min(person_distances)),
                "mean": float(np.mean(person_distances)),
                "p50": float(np.percentile(person_distances, 50)),
                "p95": float(np.percentile(person_distances, 95)),
                "max": float(np.max(person_distances)),
            }
            if person_distances
            else None
        ),
        "person_ground_truth_unmatched_samples": person_truth_unmatched_samples,
        "quality_gate_configuration": {
            "minimum_samples": args.minimum_samples,
            "minimum_duration_sec": args.minimum_duration_sec,
            "minimum_unique_command_vectors": (
                args.minimum_unique_command_vectors
            ),
            "minimum_nonzero_command_fraction": (
                args.minimum_nonzero_command_fraction
            ),
            "minimum_effective_sample_rate_hz": (
                args.minimum_effective_sample_rate_hz
            ),
            "minimum_person_positive_sample_fraction": (
                args.minimum_person_positive_sample_fraction
            ),
            "maximum_subgoal_age_ms": args.maximum_subgoal_age_ms,
            "maximum_cmd_vel_age_ms": args.maximum_cmd_vel_age_ms,
            "maximum_person_truth_unmatched_samples": (
                args.maximum_person_truth_unmatched_samples
            ),
            "require_online_subgoal": args.require_online_subgoal,
            "require_successful_episodes_only": (
                args.require_successful_episodes_only
            ),
            "require_ground_truth_person_labels": (
                args.require_ground_truth_person_labels
            ),
            "require_person_observations": args.require_person_observations,
            "require_forward_only": args.require_forward_only,
            "require_pre_relay_command_labels": (
                args.require_pre_relay_command_labels
            ),
            "fail_on_warnings": args.fail_on_warnings,
        },
        "warnings": warnings,
        "errors": errors[:100],
        "error_count": len(errors),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.report_json:
        args.report_json.parent.mkdir(parents=True, exist_ok=True)
        args.report_json.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    if errors:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
