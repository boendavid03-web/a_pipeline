#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--allow-odom-map-alignment, --bag, --base-frame, --cmd-vel-stamped-topic, --cmd-vel-topic, --dev-ratio, --episode-event-topic, --exclude-reverse-linear-x, --final-goal-topic, --global-path-topic, --local-subgoal-topic, --map-frame, --map-yaml, --odom-topic, --output-root, --overwrite, --pedestrian-ground-truth-topic, --person-ground-truth-leg-match-radius-m, --person-ground-truth-max-delta-ms, --person-ground-truth-radius-m, --person-label-mode, --pose-source, --reverse-linear-x-epsilon, --reverse-recovery-frames, --samples-01, --samples-02, --scan-01-topic, --scan-02-topic, --self-mask-mode, --semantic-label, --session-name, --split-seed, --static-label-filter-radius, --subgoal-lookahead, --subgoal-max-age-ms, --subgoal-source, --sync-tolerance-ms, --test-ratio, --train-ratio
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /cmd_vel_stamped, /data_collection/episode_event, /odom, /pedestrian_ground_truth, /scan_01, /scan_02, /semantic_cnn/final_goal, /semantic_cnn/global_path, /semantic_cnn/local_subgoal, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, NPZ, PNG, TXT
# 可能使用的关键环境变量：ACTIVE_TOOLS, E402, EPISODE_EVENT_SCHEMA, EPISODE_EVENT_TYPE, IGNORE_LABEL, INFO, JSON, ONLINE_GOAL_TYPES, PEDESTRIAN_LEG_LATERAL_OFFSET_M, PEDESTRIAN_LEG_RADIUS_M, PERSON_LABEL_MODES, PROJECT_ROOT, SELF_FOOTPRINT_HALF_EXTENTS_M, SELF_MASK_MODES, SUBGOAL_SOURCES, ZERO_COMMAND_EPSILON
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.951217952 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜v7_rosbag_to_fixed_dual_lidar_dataset.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Convert paired raw v7 scans to fixed sensor-identity slots of arbitrary size.

The output is deliberately not a merged scan.  Slots ``0..samples_01-1`` are
the unmodified /scan_01 beams and the remaining slots are the unmodified
/scan_02 beams.  Sensor-to-base TF is used only to add endpoint coordinates;
it never changes, bins, deduplicates, or resamples the raw slots.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
from collections import Counter
from collections import deque
from pathlib import Path

import numpy as np
import rosbag2_py
from PIL import Image, ImageDraw
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ACTIVE_TOOLS = PROJECT_ROOT / "workspaces" / "ros2_ws" / "tools"
sys.path.insert(0, str(ACTIVE_TOOLS))

from convert_rosbag2_to_semantic2d_native_lidar import (  # noqa: E402
    IGNORE_LABEL,
    TfIndex,
    build_aligned_cmd_velocities,
    colorize_label_image,
    find_label_id,
    hold_last_by_time,
    interpolate_clock_time,
    is_monotonic_pairs,
    load_label_names,
    load_map_info,
    load_occupancy_map,
    local_subgoals,
    msg_time_ns,
    nearest_by_time,
    normalize_frame,
    semantic_for_scan,
    scan_endpoints_map,
    split_filenames,
    stamp_to_ns,
    tf_from_transform_stamped,
    time_range_ns,
    validate_label_image,
    yaw_from_quaternion,
)


SELF_MASK_MODES = (
    "first-synchronized-pair-fixed-beam-identity",
    "per-frame-footprint",
)
SELF_FOOTPRINT_HALF_EXTENTS_M = (0.36, 0.32)
ZERO_COMMAND_EPSILON = 1e-6
PEDESTRIAN_LEG_LATERAL_OFFSET_M = 0.07
PEDESTRIAN_LEG_RADIUS_M = 0.055
PERSON_LABEL_MODES = (
    "ground-truth-legs",
    "ground-truth-radius",
    "dynamic",
    "disabled",
)
SUBGOAL_SOURCES = ("hindsight", "online")
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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--session-name", required=True)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--samples-01", required=True, type=int)
    parser.add_argument("--samples-02", required=True, type=int)
    parser.add_argument("--scan-01-topic", default="/scan_01")
    parser.add_argument("--scan-02-topic", default="/scan_02")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped-topic", default="/cmd_vel_stamped")
    parser.add_argument(
        "--cmd-vel-max-age-ms",
        type=float,
        default=100.0,
        help="Drop scans whose causal executed-command label is older than this.",
    )
    parser.add_argument(
        "--cmd-label-interface",
        choices=("pre-relay_ros_cmd_vel",),
        default="pre-relay_ros_cmd_vel",
    )
    parser.add_argument(
        "--cmd-vel-angular-z-relay-scale", type=float, default=1.5
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument(
        "--pose-source",
        choices=("odom", "tf-map-base", "auto"),
        default="auto",
        help="Pose of base_link used to project base-frame endpoints into the map.",
    )
    parser.add_argument(
        "--allow-odom-map-alignment",
        action="store_true",
        help="Explicitly acknowledge that odom coordinates are aligned with the map.",
    )
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    parser.add_argument("--train-ratio", type=float, default=0.7)
    parser.add_argument("--dev-ratio", type=float, default=0.1)
    parser.add_argument("--test-ratio", type=float, default=0.2)
    parser.add_argument("--split-seed", type=int, default=0)
    parser.add_argument(
        "--subgoal-source",
        choices=SUBGOAL_SOURCES,
        default="hindsight",
    )
    parser.add_argument(
        "--subgoal-max-age-ms",
        type=float,
        default=300.0,
        help="Maximum causal hold-last age in online mode.",
    )
    parser.add_argument(
        "--local-subgoal-topic", default="/semantic_cnn/local_subgoal"
    )
    parser.add_argument("--global-path-topic", default="/semantic_cnn/global_path")
    parser.add_argument("--final-goal-topic", default="/semantic_cnn/final_goal")
    parser.add_argument(
        "--episode-event-topic", default="/data_collection/episode_event"
    )
    parser.add_argument(
        "--successful-episodes-only",
        action="store_true",
        help=(
            "Keep only complete episodes whose end reason denotes goal success. "
            "This is intended for unattended bags that may end with a failed episode."
        ),
    )
    parser.add_argument(
        "--include-terminal-stop-frames",
        action="store_true",
        help=(
            "Legacy opt-in: retain trailing zero-command frames at a successful "
            "goal. By default these frames are excluded from supervision."
        ),
    )
    parser.add_argument("--subgoal-lookahead", type=int, default=20)
    parser.add_argument("--static-label-filter-radius", type=int, default=2)
    parser.add_argument(
        "--person-label-mode", choices=PERSON_LABEL_MODES, default="dynamic"
    )
    parser.add_argument(
        "--pedestrian-ground-truth-topic", default="/pedestrian_ground_truth"
    )
    parser.add_argument(
        "--person-ground-truth-radius-m",
        type=float,
        default=0.25,
        help=(
            "In ground-truth-radius mode, label valid LiDAR endpoints within this "
            "XY distance of a pedestrian center as Person. Each simulated lower "
            "leg has radius 0.055 m at lateral offset +/-0.07 m; the remaining "
            "margin covers scan/truth timing skew."
        ),
    )
    parser.add_argument(
        "--person-ground-truth-leg-match-radius-m",
        type=float,
        default=0.105,
        help=(
            "In ground-truth-legs mode, label valid LiDAR endpoints within this "
            "distance of either expected lower-leg center. The physical leg radius "
            "is 0.055 m; the default adds 0.05 m timing and rendering tolerance."
        ),
    )
    parser.add_argument(
        "--person-ground-truth-max-delta-ms",
        type=float,
        default=150.0,
        help="Reject a scan when its nearest pedestrian truth frame is older than this.",
    )
    parser.add_argument(
        "--self-mask-mode",
        choices=SELF_MASK_MODES,
        default="first-synchronized-pair-fixed-beam-identity",
        help=(
            "How self returns are masked. The default freezes each raw beam identity "
            "from the first synchronized output pair so later nearby obstacles are not "
            "reclassified as robot body."
        ),
    )
    parser.add_argument(
        "--exclude-reverse-linear-x",
        action="store_true",
        help=(
            "Exclude scan pairs whose causal command has negative linear.x. "
            "Use this for forward-only navigation datasets."
        ),
    )
    parser.add_argument(
        "--reverse-linear-x-epsilon",
        type=float,
        default=1e-3,
        help=(
            "A command is reverse when linear.x is below the negative of this "
            "positive tolerance."
        ),
    )
    parser.add_argument(
        "--reverse-recovery-frames",
        type=int,
        default=15,
        help=(
            "When reverse exclusion is enabled, also exclude this many following "
            "scan pairs in the same episode."
        ),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def episode_intervals_from_events(events):
    """Validate episode events and return complete, non-overlapping intervals."""
    relevant = []
    for event in events:
        if not isinstance(event, dict):
            raise RuntimeError("episode event payload is not a JSON object")
        if event.get("schema") != EPISODE_EVENT_SCHEMA:
            raise RuntimeError(
                f"episode event has unsupported schema {event.get('schema')!r}"
            )
        if event.get("event") not in ("armed", "start", "end", "ready"):
            raise RuntimeError(
                f"episode event has unknown kind {event.get('event')!r}"
            )
        if event["event"] in ("start", "end"):
            relevant.append(event)
    relevant.sort(key=lambda item: int(item.get("stamp_ns", -1)))
    intervals = []
    active = None
    previous_id = 0
    for event in relevant:
        try:
            episode_id = int(event["episode_id"])
            stamp_ns = int(event["stamp_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("episode event has invalid id or timestamp") from exc
        if episode_id <= 0 or stamp_ns < 0:
            raise RuntimeError("episode id must be positive and timestamp non-negative")
        if event["event"] == "start":
            if active is not None:
                raise RuntimeError(
                    f"episode {episode_id} starts before episode "
                    f"{active['episode_id']} ends"
                )
            if episode_id <= previous_id:
                raise RuntimeError("episode ids are not strictly increasing")
            goal = event.get("goal")
            if (
                not isinstance(goal, list)
                or len(goal) != 2
                or not all(isinstance(value, (int, float)) for value in goal)
                or not all(math.isfinite(float(value)) for value in goal)
            ):
                raise RuntimeError(f"episode {episode_id} start has an invalid goal")
            active = {
                "episode_id": episode_id,
                "start_stamp_ns": stamp_ns,
                "end_stamp_ns": None,
                "goal": [float(goal[0]), float(goal[1])],
                "start_pose": event.get("pose"),
                "end_pose": None,
                "finish_reason": None,
            }
            for source_key in ("payload_stamp_ns", "storage_stamp_ns"):
                if source_key in event:
                    active[f"start_{source_key}"] = int(event[source_key])
        else:
            if active is None:
                raise RuntimeError(f"episode {episode_id} ends without a start")
            if episode_id != active["episode_id"]:
                raise RuntimeError(
                    f"episode end id {episode_id} does not match active "
                    f"episode {active['episode_id']}"
                )
            if stamp_ns <= active["start_stamp_ns"]:
                raise RuntimeError(
                    f"episode {episode_id} end is not after its start"
                )
            active["end_stamp_ns"] = stamp_ns
            for source_key in ("payload_stamp_ns", "storage_stamp_ns"):
                if source_key in event:
                    active[f"end_{source_key}"] = int(event[source_key])
            active["end_pose"] = event.get("pose")
            active["finish_reason"] = event.get("reason")
            intervals.append(active)
            previous_id = episode_id
            active = None
    if active is not None:
        raise RuntimeError(
            f"episode {active['episode_id']} has no recorded end event"
        )
    if events and not intervals:
        raise RuntimeError("episode event topic contains no complete episode")
    return intervals


def successful_episode_intervals(intervals):
    """Return goal-success intervals without renumbering their source IDs."""
    return [
        interval
        for interval in intervals
        if interval.get("finish_reason") in SUCCESSFUL_EPISODE_FINISH_REASONS
    ]


def terminal_goal_stop_keep_mask(
    commands, episode_ids, successful_episode_ids, epsilon=ZERO_COMMAND_EPSILON
):
    """Drop only the contiguous zero-command tail of successful episodes."""
    if len(commands) != len(episode_ids):
        raise ValueError("commands and episode_ids must have equal length")
    successful = {int(value) for value in successful_episode_ids}
    keep = [True] * len(commands)
    removed_by_episode = {}
    indices_by_episode = {}
    for index, episode_id in enumerate(episode_ids):
        indices_by_episode.setdefault(int(episode_id), []).append(index)
    for episode_id in successful:
        removed = 0
        for index in reversed(indices_by_episode.get(episode_id, [])):
            velocity = commands[index]
            if any(abs(float(value)) > float(epsilon) for value in velocity):
                break
            keep[index] = False
            removed += 1
        if removed:
            removed_by_episode[str(episode_id)] = removed
    return keep, {
        "enabled": True,
        "policy": "drop-contiguous-zero-command-tail-of-successful-episode",
        "epsilon": float(epsilon),
        "total_frames_removed": int(sum(removed_by_episode.values())),
        "removed_frames_by_episode": removed_by_episode,
    }


def map_episode_events_to_sim_time(events, clocks):
    """Map wall/storage-domain episode events into the bag simulation clock."""
    boundary_events = [
        event for event in events if event.get("event") in ("start", "end")
    ]
    if not boundary_events:
        return [], {
            "method": "none",
            "event_count": 0,
            "non_boundary_event_count": len(events),
            "payload_storage_delta_ns_max_abs": None,
        }
    if not clocks:
        raise RuntimeError("episode events require /clock for storage-to-sim mapping")
    ordered_clocks = sorted(clocks, key=lambda item: item[0])
    if not is_monotonic_pairs(ordered_clocks):
        raise RuntimeError("/clock storage-to-simulation mapping is not monotonic")
    storage_times = [int(item[0]) for item in ordered_clocks]
    storage_start, storage_end = storage_times[0], storage_times[-1]
    mapped = []
    payload_storage_deltas = []
    for event in boundary_events:
        if "_storage_stamp_ns" not in event:
            raise RuntimeError("episode event is missing rosbag storage timestamp")
        storage_stamp_ns = int(event["_storage_stamp_ns"])
        if not storage_start <= storage_stamp_ns <= storage_end:
            raise RuntimeError(
                "episode event storage timestamp is outside the recorded /clock range"
            )
        payload_stamp_ns = int(event.get("stamp_ns", -1))
        if payload_stamp_ns < 0:
            raise RuntimeError("episode event payload stamp_ns is invalid")
        payload_storage_deltas.append(payload_stamp_ns - storage_stamp_ns)
        converted = dict(event)
        converted.pop("_storage_stamp_ns", None)
        converted["payload_stamp_ns"] = payload_stamp_ns
        converted["storage_stamp_ns"] = storage_stamp_ns
        converted["stamp_ns"] = interpolate_clock_time(
            ordered_clocks, storage_times, storage_stamp_ns
        )
        mapped.append(converted)
    return mapped, {
        "method": "rosbag storage timestamp interpolated through /clock",
        "event_count": len(mapped),
        "non_boundary_event_count": len(events) - len(boundary_events),
        "clock_storage_range": time_range_ns(storage_times),
        "clock_sim_range": time_range_ns([int(item[1]) for item in ordered_clocks]),
        "payload_storage_delta_ns_max_abs": max(
            abs(value) for value in payload_storage_deltas
        ),
        "payload_storage_delta_ns_mean": float(np.mean(payload_storage_deltas)),
    }


def common_valid_time_range(clocks, scans_01, scans_02, odoms, cmd_stamped):
    """Return the closed simulation-time intersection used for conversion."""
    streams = {
        "clock": [int(item[1]) for item in clocks],
        "scan_01": [int(item[0]) for item in scans_01],
        "scan_02": [int(item[0]) for item in scans_02],
        "odom": [int(item[0]) for item in odoms],
        "cmd_vel_stamped": [int(item[0]) for item in cmd_stamped],
    }
    missing = [name for name, values in streams.items() if not values]
    if missing:
        raise RuntimeError(
            "cannot compute common valid time range; empty stream(s): "
            + ", ".join(missing)
        )
    ranges = {
        name: {"start_ns": min(values), "end_ns": max(values)}
        for name, values in streams.items()
    }
    start_ns = max(value["start_ns"] for value in ranges.values())
    end_ns = min(value["end_ns"] for value in ranges.values())
    if start_ns > end_ns:
        raise RuntimeError("clock/scan/odom/cmd streams have no common time range")
    return start_ns, end_ns, ranges


def episode_ids_for_stamps(stamps_ns, intervals):
    """Assign each timestamp to a complete episode, or zero when outside."""
    ids = []
    interval_index = 0
    for stamp_ns in stamps_ns:
        while (
            interval_index < len(intervals)
            and int(stamp_ns) > intervals[interval_index]["end_stamp_ns"]
        ):
            interval_index += 1
        if interval_index >= len(intervals):
            ids.append(0)
            continue
        interval = intervals[interval_index]
        if interval["start_stamp_ns"] <= int(stamp_ns) <= interval["end_stamp_ns"]:
            ids.append(int(interval["episode_id"]))
        else:
            ids.append(0)
    return ids


def local_subgoals_by_episode(positions, episode_ids, lookahead):
    """Compute hindsight subgoals without crossing an episode boundary."""
    if len(positions) != len(episode_ids):
        raise ValueError("positions and episode_ids must have equal length")
    if not len(positions):
        return np.empty((0, 2), dtype=np.float32)
    output = np.empty((len(positions), 2), dtype=np.float32)
    start = 0
    while start < len(positions):
        episode_id = episode_ids[start]
        end = start + 1
        while end < len(positions) and episode_ids[end] == episode_id:
            end += 1
        output[start:end] = local_subgoals(positions[start:end], lookahead)
        start = end
    return output


def split_filenames_by_episode(
    filenames, episode_ids, train_ratio, dev_ratio, split_seed
):
    """Split complete episodes, never frames from the same episode."""
    if len(filenames) != len(episode_ids):
        raise ValueError("filenames and episode_ids must have equal length")
    grouped = {}
    for filename, episode_id in zip(filenames, episode_ids):
        grouped.setdefault(int(episode_id), []).append(filename)
    episode_keys = sorted(grouped)
    rng = np.random.default_rng(split_seed)
    rng.shuffle(episode_keys)
    episode_count = len(episode_keys)
    test_ratio = max(0.0, 1.0 - train_ratio - dev_ratio)
    ratios = (train_ratio, dev_ratio, test_ratio)
    raw_counts = [episode_count * ratio for ratio in ratios]
    counts = [int(math.floor(value)) for value in raw_counts]
    for index in sorted(
        range(3),
        key=lambda item: raw_counts[item] - counts[item],
        reverse=True,
    )[: episode_count - sum(counts)]:
        counts[index] += 1
    minimum_counts = [
        int(train_ratio > 0.0 and episode_count >= 1),
        int(dev_ratio > 0.0 and episode_count >= 2),
        int(test_ratio > 0.0 and episode_count >= 3),
    ]
    for target in range(3):
        while counts[target] < minimum_counts[target]:
            donors = [
                index
                for index in range(3)
                if counts[index] > minimum_counts[index]
            ]
            if not donors:
                break
            donor = max(
                donors,
                key=lambda index: counts[index] - minimum_counts[index],
            )
            counts[donor] -= 1
            counts[target] += 1
    train_count, dev_count, _test_count = counts
    train_keys = set(episode_keys[:train_count])
    dev_keys = set(episode_keys[train_count : train_count + dev_count])
    test_keys = set(episode_keys[train_count + dev_count :])
    return tuple(
        [
            filename
            for filename, episode_id in zip(filenames, episode_ids)
            if int(episode_id) in keys
        ]
        for keys in (train_keys, dev_keys, test_keys)
    )


def causal_hold_last_subgoals(scan_stamps_ns, subgoals, max_age_ns):
    """Match each scan to the latest non-future subgoal.

    Missing and stale matches are represented by ``None`` and audited so a
    single bad frame cannot discard an otherwise valid episode.
    """
    if max_age_ns < 0:
        raise ValueError("max_age_ns must be non-negative")
    if not subgoals:
        return [None] * len(scan_stamps_ns), {
            "leading_unmatched_frames_dropped": len(scan_stamps_ns),
            "missing_causal_frames_dropped": len(scan_stamps_ns),
            "stale_frames_dropped": 0,
            "matched_frames": 0,
            "age_ns_min": None,
            "age_ns_max": None,
            "age_ns_mean": None,
        }

    ordered = sorted(subgoals, key=lambda item: item[0])
    matches = []
    leading_unmatched = 0
    matching_started = False
    missing_causal = 0
    stale = 0
    ages_ns = []
    for scan_stamp_ns in scan_stamps_ns:
        match, _ = hold_last_by_time(ordered, int(scan_stamp_ns))
        if match is None:
            matches.append(None)
            missing_causal += 1
            if not matching_started:
                leading_unmatched += 1
            continue

        subgoal_stamp_ns = int(match[0])
        age_ns = int(scan_stamp_ns) - subgoal_stamp_ns
        if age_ns < 0:
            raise RuntimeError(
                f"future online subgoal selected for scan {scan_stamp_ns}"
            )
        if age_ns > max_age_ns:
            matches.append(None)
            stale += 1
            matching_started = True
            continue
        matching_started = True
        ages_ns.append(age_ns)
        matches.append((subgoal_stamp_ns, match[1], age_ns))

    return matches, {
        "leading_unmatched_frames_dropped": leading_unmatched,
        "missing_causal_frames_dropped": missing_causal,
        "stale_frames_dropped": stale,
        "matched_frames": len(ages_ns),
        "age_ns_min": min(ages_ns) if ages_ns else None,
        "age_ns_max": max(ages_ns) if ages_ns else None,
        "age_ns_mean": float(np.mean(ages_ns)) if ages_ns else None,
    }


def causal_hold_last_subgoals_by_episode(
    scan_stamps_ns, episode_ids, subgoals, intervals, max_age_ns
):
    """Apply causal subgoal matching independently inside each episode."""
    if len(scan_stamps_ns) != len(episode_ids):
        raise ValueError("scan_stamps_ns and episode_ids must have equal length")
    interval_by_id = {
        int(interval["episode_id"]): interval for interval in intervals
    }
    matches = []
    leading = 0
    missing = 0
    stale = 0
    ages_ns = []
    start = 0
    while start < len(scan_stamps_ns):
        episode_id = int(episode_ids[start])
        end = start + 1
        while end < len(scan_stamps_ns) and int(episode_ids[end]) == episode_id:
            end += 1
        interval = interval_by_id[episode_id]
        episode_subgoals = [
            item
            for item in subgoals
            if interval["start_stamp_ns"] <= int(item[0]) <= interval["end_stamp_ns"]
        ]
        episode_matches, audit = causal_hold_last_subgoals(
            scan_stamps_ns[start:end],
            episode_subgoals,
            max_age_ns,
        )
        matches.extend(episode_matches)
        leading += audit["leading_unmatched_frames_dropped"]
        missing += audit["missing_causal_frames_dropped"]
        stale += audit["stale_frames_dropped"]
        ages_ns.extend(
            int(item[2]) for item in episode_matches if item is not None
        )
        start = end
    return matches, {
        "leading_unmatched_frames_dropped": leading,
        "missing_causal_frames_dropped": missing,
        "stale_frames_dropped": stale,
        "matched_frames": len(ages_ns),
        "age_ns_min": min(ages_ns) if ages_ns else None,
        "age_ns_max": max(ages_ns) if ages_ns else None,
        "age_ns_mean": float(np.mean(ages_ns)) if ages_ns else None,
    }


def subgoal_contract_metadata(
    source,
    lookahead,
    max_age_ms,
    leading_unmatched_frames,
    ages_ns,
    stale_frames=0,
):
    """Build the source and alignment portion of conversion metadata."""
    if source not in SUBGOAL_SOURCES:
        raise ValueError(f"unknown subgoal source: {source}")
    if source == "online":
        if not ages_ns:
            raise ValueError("online metadata requires at least one matched age")
        return {
            "subgoal_source": "online",
            "subgoal_alignment_method": (
                "causal hold-last at or before scan_01 header stamp"
            ),
            "subgoal_max_age_ms": float(max_age_ms),
            "subgoal_matched_samples": len(ages_ns),
            "subgoal_unmatched_leading_frames_dropped": int(
                leading_unmatched_frames
            ),
            "subgoal_stale_samples": int(stale_frames),
            "subgoal_age_ms_min": min(ages_ns) / 1_000_000.0,
            "subgoal_age_ms_max": max(ages_ns) / 1_000_000.0,
            "subgoal_age_ms_mean": float(np.mean(ages_ns)) / 1_000_000.0,
            "subgoal_lookahead": int(lookahead),
        }
    return {
        "subgoal_source": "hindsight",
        "subgoal_alignment_method": (
            "future robot pose transformed into the current base frame"
        ),
        "subgoal_max_age_ms": None,
        "subgoal_matched_samples": 0,
        "subgoal_unmatched_leading_frames_dropped": 0,
        "subgoal_stale_samples": 0,
        "subgoal_age_ms_min": None,
        "subgoal_age_ms_max": None,
        "subgoal_age_ms_mean": None,
        "subgoal_lookahead": int(lookahead),
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def quaternion_matrix(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(float).eps:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.asarray(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


class StaticTransformIndex3D:
    """Small full-quaternion TF graph for the static sensor extrinsics."""

    def __init__(self):
        self.transforms = {}

    def add(self, transform):
        parent = normalize_frame(transform.header.frame_id)
        child = normalize_frame(transform.child_frame_id)
        if not parent or not child:
            return
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = quaternion_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        matrix[:3, 3] = (translation.x, translation.y, translation.z)
        key = (parent, child)
        existing = self.transforms.get(key)
        if existing is not None and not np.allclose(
            existing, matrix, rtol=0.0, atol=1e-9
        ):
            raise RuntimeError(
                "Conflicting static TF for "
                f"{parent}->{child}; fixed raw beam identities would be ambiguous"
            )
        self.transforms[key] = matrix

    def lookup(self, parent, child):
        parent, child = normalize_frame(parent), normalize_frame(child)
        if parent == child:
            return np.eye(4, dtype=np.float64)
        queue = deque([(parent, np.eye(4, dtype=np.float64))])
        visited = {parent}
        while queue:
            frame, accumulated = queue.popleft()
            for (edge_parent, edge_child), matrix in self.transforms.items():
                if edge_parent == frame and edge_child not in visited:
                    result = accumulated @ matrix
                    if edge_child == child:
                        return result
                    visited.add(edge_child)
                    queue.append((edge_child, result))
                elif edge_child == frame and edge_parent not in visited:
                    result = accumulated @ np.linalg.inv(matrix)
                    if edge_parent == child:
                        return result
                    visited.add(edge_parent)
                    queue.append((edge_parent, result))
        return None


def read_dual_bag(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = (
        args.scan_01_topic,
        args.scan_02_topic,
        args.odom_topic,
        "/tf",
        "/tf_static",
    )
    missing = [topic for topic in required if topic not in types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")
    online_goal_types = {
        args.global_path_topic: ONLINE_GOAL_TYPES["/semantic_cnn/global_path"],
        args.local_subgoal_topic: ONLINE_GOAL_TYPES["/semantic_cnn/local_subgoal"],
        args.final_goal_topic: ONLINE_GOAL_TYPES["/semantic_cnn/final_goal"],
    }
    if args.subgoal_source == "online":
        missing_online = [
            topic for topic in online_goal_types if topic not in types
        ]
        if missing_online:
            raise RuntimeError(
                "Bag is missing required online goal topic(s): "
                + ", ".join(missing_online)
            )
        wrong_types = [
            f"{topic}={types[topic]} (expected {expected})"
            for topic, expected in online_goal_types.items()
            if types[topic] != expected
        ]
        if wrong_types:
            raise RuntimeError(
                "Online goal topic type mismatch: " + "; ".join(wrong_types)
            )
    if (
        args.episode_event_topic in types
        and types[args.episode_event_topic] != EPISODE_EVENT_TYPE
    ):
        raise RuntimeError(
            f"{args.episode_event_topic} has type "
            f"{types[args.episode_event_topic]!r}, expected {EPISODE_EVENT_TYPE!r}"
        )

    message_types = {topic: get_message(msg_type) for topic, msg_type in types.items()}
    (
        scans_01,
        scans_02,
        odoms,
        cmds,
        cmd_stamped,
        clocks,
        pedestrians,
        local_subgoal_messages,
        global_path_messages,
        final_goal_messages,
        episode_events,
    ) = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    topic_counts = Counter()
    tf_index = TfIndex()
    static_tf_3d = StaticTransformIndex3D()
    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        topic_counts[topic] += 1
        if topic == args.scan_01_topic:
            msg = deserialize_message(data, message_types[topic])
            scans_01.append((msg_time_ns(msg, storage_time), msg))
        elif topic == args.scan_02_topic:
            msg = deserialize_message(data, message_types[topic])
            scans_02.append((msg_time_ns(msg, storage_time), msg))
        elif topic == args.odom_topic:
            msg = deserialize_message(data, message_types[topic])
            pose, twist = msg.pose.pose, msg.twist.twist
            odoms.append(
                (
                    msg_time_ns(msg, storage_time),
                    (
                        float(pose.position.x),
                        float(pose.position.y),
                        yaw_from_quaternion(pose.orientation),
                        float(twist.linear.x),
                        float(twist.angular.z),
                    ),
                )
            )
        elif topic == args.cmd_vel_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            cmds.append(
                (
                    int(storage_time),
                    (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)),
                )
            )
        elif topic == args.cmd_vel_stamped_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            twist = msg.twist
            cmd_stamped.append(
                (
                    msg_time_ns(msg, storage_time),
                    (float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)),
                )
            )
        elif topic == "/clock" and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            clocks.append((int(storage_time), stamp_to_ns(msg.clock)))
        elif topic == args.pedestrian_ground_truth_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            states = []
            for state in msg.pedestrians:
                states.append(
                    (
                        str(state.id),
                        float(state.pose.position.x),
                        float(state.pose.position.y),
                        yaw_from_quaternion(state.pose.orientation),
                        float(state.velocity.linear.x),
                        float(state.velocity.linear.y),
                    )
                )
            pedestrians.append(
                (
                    msg_time_ns(msg, storage_time),
                    normalize_frame(msg.header.frame_id),
                    states,
                )
            )
        elif topic == args.local_subgoal_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            local_subgoal_messages.append(
                (
                    msg_time_ns(msg, storage_time),
                    (float(msg.point.x), float(msg.point.y)),
                    normalize_frame(msg.header.frame_id),
                )
            )
        elif topic == args.global_path_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            global_path_messages.append(
                (
                    msg_time_ns(msg, storage_time),
                    normalize_frame(msg.header.frame_id),
                )
            )
        elif topic == args.final_goal_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            final_goal_messages.append(
                (
                    msg_time_ns(msg, storage_time),
                    (float(msg.point.x), float(msg.point.y)),
                    normalize_frame(msg.header.frame_id),
                )
            )
        elif topic == args.episode_event_topic and topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f"{args.episode_event_topic} contains invalid JSON"
                ) from exc
            payload["_storage_stamp_ns"] = int(storage_time)
            episode_events.append(payload)
        elif topic in ("/tf", "/tf_static"):
            msg = deserialize_message(data, message_types[topic])
            for transform in msg.transforms:
                tf_index.add(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    msg_time_ns(transform, storage_time),
                    tf_from_transform_stamped(transform),
                    is_static=(topic == "/tf_static"),
                )
                if topic == "/tf_static":
                    static_tf_3d.add(transform)

    for values in (
        scans_01,
        scans_02,
        odoms,
        cmds,
        cmd_stamped,
        clocks,
        pedestrians,
        local_subgoal_messages,
        global_path_messages,
        final_goal_messages,
    ):
        values.sort(key=lambda item: item[0])
    tf_index.finalize()
    return (
        scans_01,
        scans_02,
        odoms,
        cmds,
        cmd_stamped,
        clocks,
        tf_index,
        static_tf_3d,
        topic_counts,
        pedestrians,
        local_subgoal_messages,
        global_path_messages,
        final_goal_messages,
        episode_events,
    )


def command_is_zero(command):
    return all(abs(float(value)) <= ZERO_COMMAND_EPSILON for value in command)


def reverse_motion_keep_mask(
    commands,
    episode_ids,
    linear_x_epsilon,
    recovery_frames,
):
    """Return a keep mask and audit for forward-only command filtering.

    Recovery state is reset at episode boundaries. A new reverse command resets
    the recovery budget, so separated reverse bursts cannot leak their recovery
    motion into the converted dataset.
    """
    if len(commands) != len(episode_ids):
        raise ValueError("commands and episode_ids must have equal length")
    if not math.isfinite(linear_x_epsilon) or linear_x_epsilon <= 0.0:
        raise ValueError("linear_x_epsilon must be positive and finite")
    if recovery_frames < 0:
        raise ValueError("recovery_frames must be non-negative")

    keep = [True] * len(commands)
    reverse_by_episode = Counter()
    recovery_by_episode = Counter()
    recovery_remaining = 0
    previous_episode_id = None
    for index, (command, episode_id) in enumerate(zip(commands, episode_ids)):
        episode_id = int(episode_id)
        if episode_id != previous_episode_id:
            recovery_remaining = 0
            previous_episode_id = episode_id
        values = np.asarray(command, dtype=np.float64).reshape(-1)
        if values.size != 3 or not np.all(np.isfinite(values)):
            raise ValueError("each command must contain three finite values")
        if float(values[0]) < -float(linear_x_epsilon):
            keep[index] = False
            reverse_by_episode[episode_id] += 1
            recovery_remaining = int(recovery_frames)
        elif recovery_remaining > 0:
            keep[index] = False
            recovery_by_episode[episode_id] += 1
            recovery_remaining -= 1

    affected_episode_ids = sorted(
        set(reverse_by_episode) | set(recovery_by_episode)
    )
    return keep, {
        "enabled": True,
        "policy": "exclude-reverse-command-and-following-recovery-frames",
        "linear_x_epsilon": float(linear_x_epsilon),
        "recovery_frames_requested": int(recovery_frames),
        "reverse_frames_removed": int(sum(reverse_by_episode.values())),
        "recovery_frames_removed": int(sum(recovery_by_episode.values())),
        "total_frames_removed": int(
            sum(reverse_by_episode.values()) + sum(recovery_by_episode.values())
        ),
        "affected_episode_ids": affected_episode_ids,
        "reverse_frames_by_episode": {
            str(key): int(value)
            for key, value in sorted(reverse_by_episode.items())
        },
        "recovery_frames_by_episode": {
            str(key): int(value)
            for key, value in sorted(recovery_by_episode.items())
        },
    }


def command_distribution(commands):
    values = np.asarray(commands, dtype=np.float64).reshape((-1, 3))
    moving = np.any(np.abs(values) > ZERO_COMMAND_EPSILON, axis=1)
    return {
        "count": int(len(values)),
        "linear_x": {
            "min": float(values[:, 0].min()),
            "max": float(values[:, 0].max()),
            "mean": float(values[:, 0].mean()),
        },
        "linear_y": {
            "min": float(values[:, 1].min()),
            "max": float(values[:, 1].max()),
            "mean": float(values[:, 1].mean()),
        },
        "angular_z": {
            "min": float(values[:, 2].min()),
            "max": float(values[:, 2].max()),
            "mean": float(values[:, 2].mean()),
        },
        "moving_fraction": float(np.mean(moving)),
        "stop_fraction": float(np.mean(~moving)),
        "left_turn_fraction": float(np.mean(values[:, 2] > ZERO_COMMAND_EPSILON)),
        "right_turn_fraction": float(np.mean(values[:, 2] < -ZERO_COMMAND_EPSILON)),
        "negative_linear_x_fraction": float(
            np.mean(values[:, 0] < -ZERO_COMMAND_EPSILON)
        ),
        "nonzero_linear_y_fraction": float(
            np.mean(np.abs(values[:, 1]) > ZERO_COMMAND_EPSILON)
        ),
        "drl_vo_linear_clip_fraction": float(
            np.mean((values[:, 0] < 0.0) | (values[:, 0] > 0.5))
        ),
    }


def trim_safe_boundary_cmd_vel_stamped(cmd_stamped, clocks, paired_scans):
    """Discard only auditable zero-velocity commands outside the /clock range.

    This never edits the bag or shifts simulation time.  A boundary trim is
    allowed only when every discarded command is zero and the first converted
    scan still has a retained command at or before its timestamp.
    """
    raw_count = len(cmd_stamped)
    audit = {
        "policy": "trim-zero-velocity-cmd-vel-stamped-outside-clock",
        "raw_count": raw_count,
        "retained_count": raw_count,
        "before_clock_count": 0,
        "after_clock_count": 0,
        "trimmed_count": 0,
        "trimmed_commands_all_zero": True,
        "scan_fully_in_clock_range": None,
        "first_paired_scan_has_in_clock_prior_command": None,
        "applied": False,
    }
    if not cmd_stamped or not clocks or not paired_scans:
        return cmd_stamped, audit

    clock_times = [item[1] for item in clocks]
    clock_start, clock_end = min(clock_times), max(clock_times)
    scan_times = [item[0] for item in paired_scans]
    scan_start, scan_end = min(scan_times), max(scan_times)
    before = [item for item in cmd_stamped if item[0] < clock_start]
    after = [item for item in cmd_stamped if item[0] > clock_end]
    retained = [
        item for item in cmd_stamped if clock_start <= item[0] <= clock_end
    ]
    trimmed = before + after
    all_zero = all(command_is_zero(item[1]) for item in trimmed)
    scan_in_clock = clock_start <= scan_start and scan_end <= clock_end
    first_scan_has_prior = any(item[0] <= scan_start for item in retained)
    audit.update(
        {
            "retained_count": len(retained),
            "before_clock_count": len(before),
            "after_clock_count": len(after),
            "trimmed_count": len(trimmed),
            "trimmed_commands_all_zero": all_zero,
            "clock_time_range": time_range_ns(clock_times),
            "paired_scan_time_range": time_range_ns(scan_times),
            "scan_fully_in_clock_range": scan_in_clock,
            "first_paired_scan_has_in_clock_prior_command": first_scan_has_prior,
            "applied": bool(trimmed),
        }
    )
    if trimmed and not all_zero:
        raise RuntimeError(
            "Refusing to trim nonzero /cmd_vel_stamped messages outside the /clock range"
        )
    if trimmed and not scan_in_clock:
        raise RuntimeError(
            "Refusing boundary command trim because converted scans are outside the /clock range"
        )
    if trimmed and not first_scan_has_prior:
        raise RuntimeError(
            "Refusing boundary command trim because the first converted scan has no retained prior command"
        )
    return retained, audit


def sensor_slots(msg, expected_samples, static_tf_3d, base_frame, fixed_self_mask=None):
    raw_ranges = np.asarray(msg.ranges, dtype=np.float32)
    if raw_ranges.shape != (expected_samples,):
        raise RuntimeError(
            f"Expected {expected_samples} beams from {msg.header.frame_id}, "
            f"got {raw_ranges.size}"
        )
    raw_angles = (
        float(msg.angle_min)
        + np.arange(expected_samples, dtype=np.float64) * float(msg.angle_increment)
    ).astype(np.float32)
    range_valid = np.isfinite(raw_ranges)
    if np.isfinite(float(msg.range_min)):
        range_valid &= raw_ranges >= float(msg.range_min)
    if np.isfinite(float(msg.range_max)):
        range_valid &= raw_ranges <= float(msg.range_max)

    frame = normalize_frame(msg.header.frame_id)
    transform = static_tf_3d.lookup(normalize_frame(base_frame), frame)
    if transform is None:
        raise RuntimeError(f"Cannot resolve full static TF {base_frame}->{frame}")
    points_x = np.full(expected_samples, np.nan, dtype=np.float32)
    points_y = np.full(expected_samples, np.nan, dtype=np.float32)
    indices = np.flatnonzero(range_valid)
    sensor_x = raw_ranges[indices] * np.cos(raw_angles[indices])
    sensor_y = raw_ranges[indices] * np.sin(raw_angles[indices])
    homogeneous = np.vstack(
        (
            sensor_x.astype(np.float64),
            sensor_y.astype(np.float64),
            np.zeros(len(indices), dtype=np.float64),
            np.ones(len(indices), dtype=np.float64),
        )
    )
    points_base = transform @ homogeneous
    points_x[indices] = points_base[0].astype(np.float32)
    points_y[indices] = points_base[1].astype(np.float32)

    footprint_self_mask = np.zeros(expected_samples, dtype=np.bool_)
    footprint_self_mask[indices] = (
        np.abs(points_x[indices]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[0]
    ) & (
        np.abs(points_y[indices]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[1]
    )
    if fixed_self_mask is None:
        self_mask = footprint_self_mask
    else:
        self_mask = np.asarray(fixed_self_mask, dtype=np.bool_).reshape(-1)
        if self_mask.shape != (expected_samples,):
            raise RuntimeError(
                f"Fixed self mask for {frame} has shape {self_mask.shape}, "
                f"expected {(expected_samples,)}"
            )
        self_mask = self_mask.copy()
    valid = range_valid & ~self_mask
    virtual_ranges = np.full(expected_samples, np.nan, dtype=np.float32)
    virtual_angles = np.full(expected_samples, np.nan, dtype=np.float32)
    usable = np.flatnonzero(valid)
    virtual_ranges[usable] = np.hypot(points_x[usable], points_y[usable])
    virtual_angles[usable] = np.arctan2(points_y[usable], points_x[usable])
    return {
        "raw_ranges": raw_ranges,
        "raw_angles_sensor": raw_angles,
        "points_x_base": points_x,
        "points_y_base": points_y,
        "virtual_ranges": virtual_ranges,
        "virtual_angles": virtual_angles,
        "range_valid_mask": range_valid,
        "self_mask": self_mask,
        "footprint_self_mask": footprint_self_mask,
        "valid_mask": valid,
        "frame_id": frame,
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
    }


def sensor_layout(slots):
    """Return the fields that define a raw beam identity."""
    return {
        "frame_id": slots["frame_id"],
        "beam_count": int(slots["raw_ranges"].size),
        "angle_min": float(slots["angle_min"]),
        "angle_max": float(slots["angle_max"]),
        "angle_increment": float(slots["angle_increment"]),
        "range_min": float(slots["range_min"]),
        "range_max": float(slots["range_max"]),
    }


def mask_runs(mask):
    """Encode a fixed beam mask as inclusive index runs for compact metadata."""
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    run_starts = np.r_[0, np.flatnonzero(np.diff(indices) != 1) + 1]
    run_ends = np.r_[run_starts[1:] - 1, len(indices) - 1]
    return [
        [int(indices[start]), int(indices[end])]
        for start, end in zip(run_starts, run_ends)
    ]


def require_matching_sensor_layout(expected, actual, sensor_name):
    if expected["frame_id"] != actual["frame_id"]:
        raise RuntimeError(
            f"{sensor_name} frame changed from {expected['frame_id']} to "
            f"{actual['frame_id']}; fixed self beam identities are no longer valid"
        )
    if expected["beam_count"] != actual["beam_count"]:
        raise RuntimeError(
            f"{sensor_name} beam count changed from {expected['beam_count']} to "
            f"{actual['beam_count']}; fixed self beam identities are no longer valid"
        )
    for field in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"):
        if not math.isclose(expected[field], actual[field], rel_tol=0.0, abs_tol=1e-6):
            raise RuntimeError(
                f"{sensor_name} {field} changed from {expected[field]} to "
                f"{actual[field]}; fixed self beam identities are no longer valid"
            )


def pair_synchronized_scans(scans_01, scans_02, tolerance_ns):
    """Pair raw scans once, monotonically, and without reusing scan_02 messages."""
    pairs = []
    skipped_01 = 0
    skipped_02 = 0
    index_02 = 0
    for stamp_01, scan_01 in scans_01:
        while (
            index_02 < len(scans_02)
            and scans_02[index_02][0] < stamp_01 - tolerance_ns
        ):
            skipped_02 += 1
            index_02 += 1
        if index_02 >= len(scans_02):
            skipped_01 += 1
            continue

        candidate_end = index_02
        while (
            candidate_end < len(scans_02)
            and scans_02[candidate_end][0] <= stamp_01 + tolerance_ns
        ):
            candidate_end += 1
        if candidate_end == index_02:
            skipped_01 += 1
            continue

        matched_index = min(
            range(index_02, candidate_end),
            key=lambda index: (abs(scans_02[index][0] - stamp_01), scans_02[index][0]),
        )
        skipped_02 += matched_index - index_02
        stamp_02, scan_02 = scans_02[matched_index]
        pairs.append((stamp_01, scan_01, stamp_02, scan_02))
        index_02 = matched_index + 1

    skipped_02 += len(scans_02) - index_02
    return pairs, skipped_01, skipped_02


def projection_pose(args, stamp_ns, odom_pose, tf_index):
    map_frame = normalize_frame(args.map_frame)
    base_frame = normalize_frame(args.base_frame)
    if args.pose_source in ("auto", "tf-map-base"):
        pose = tf_index.lookup(map_frame, base_frame, stamp_ns)
        if pose is not None:
            return pose, "tf-map-base"
        if args.pose_source == "tf-map-base":
            raise RuntimeError(f"Cannot resolve TF {map_frame}->{base_frame} at {stamp_ns}")
    if not args.allow_odom_map_alignment:
        raise RuntimeError(
            "No map->base_link TF is available. Pass --allow-odom-map-alignment only "
            "when Gazebo odom coordinates are known to be aligned with this map."
        )
    return tuple(float(v) for v in odom_pose[:3]), "odom-explicit-map-alignment"


def concatenate_sensor_fields(first, second, field):
    return np.concatenate((first[field], second[field]))


def transform_xy_points(points_xy, transform):
    """Apply a planar (x, y, yaw) TF to an N x 2 array."""
    if not len(points_xy):
        return np.empty((0, 2), dtype=np.float64)
    x, y, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = np.empty_like(points_xy, dtype=np.float64)
    result[:, 0] = x + cosine * points_xy[:, 0] - sine * points_xy[:, 1]
    result[:, 1] = y + sine * points_xy[:, 0] + cosine * points_xy[:, 1]
    return result


def rotate_xy_vectors(vectors_xy, yaw):
    """Rotate N x 2 vectors without applying translation."""
    if not len(vectors_xy):
        return np.empty((0, 2), dtype=np.float64)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = np.empty_like(vectors_xy, dtype=np.float64)
    result[:, 0] = cosine * vectors_xy[:, 0] - sine * vectors_xy[:, 1]
    result[:, 1] = sine * vectors_xy[:, 0] + cosine * vectors_xy[:, 1]
    return result


def pedestrian_truth_in_map(truth_match, stamp_ns, tf_index, map_frame):
    """Return pedestrian IDs, center poses, and source time in the map frame."""
    truth_stamp_ns, truth_frame, states = truth_match
    ids = [state[0] for state in states]
    points = np.asarray([(state[1], state[2]) for state in states], dtype=np.float64)
    yaws = np.asarray([state[3] for state in states], dtype=np.float64)
    velocities = np.asarray(
        [(state[4], state[5]) for state in states], dtype=np.float64
    )
    map_frame = normalize_frame(map_frame)
    truth_frame = normalize_frame(truth_frame)
    if truth_frame != map_frame:
        transform = tf_index.lookup(map_frame, truth_frame, stamp_ns)
        if transform is None:
            raise RuntimeError(
                f"Cannot resolve pedestrian truth TF {map_frame}->{truth_frame} "
                f"at scan timestamp {stamp_ns}"
            )
        points = transform_xy_points(points, transform)
        velocities = rotate_xy_vectors(velocities, float(transform[2]))
        yaws = np.arctan2(
            np.sin(yaws + float(transform[2])),
            np.cos(yaws + float(transform[2])),
        )
    return ids, points, yaws, velocities, int(truth_stamp_ns)


def pedestrian_leg_centers(pedestrian_xy, pedestrian_yaw):
    """Return left/right lower-leg centers in [pedestrian, side, xy] order."""
    if not len(pedestrian_xy):
        return np.empty((0, 2, 2), dtype=np.float64)
    lateral = np.column_stack(
        (-np.sin(pedestrian_yaw), np.cos(pedestrian_yaw))
    )
    return np.stack(
        (
            pedestrian_xy - PEDESTRIAN_LEG_LATERAL_OFFSET_M * lateral,
            pedestrian_xy + PEDESTRIAN_LEG_LATERAL_OFFSET_M * lateral,
        ),
        axis=1,
    )


def apply_ground_truth_person_labels(
    semantic, valid_mask, ranges, angles, pose, pedestrian_xy, person_label_id, radius_m
):
    """Label endpoints near a known pedestrian and return per-slot nearest distances."""
    nearest = np.full(len(semantic), np.nan, dtype=np.float32)
    project_indices, world_x, world_y = scan_endpoints_map(
        ranges, angles, valid_mask, pose
    )
    if not len(project_indices) or not len(pedestrian_xy):
        return semantic, nearest
    endpoints = np.column_stack((world_x, world_y))
    distances = np.linalg.norm(
        endpoints[:, np.newaxis, :] - pedestrian_xy[np.newaxis, :, :], axis=2
    )
    endpoint_nearest = distances.min(axis=1)
    nearest[project_indices] = endpoint_nearest.astype(np.float32)
    semantic[project_indices[endpoint_nearest <= radius_m]] = person_label_id
    return semantic, nearest


def write_sample(path, first, second, semantic, position, velocity, cmd_velocity, subgoal,
                 subgoal_stamp_ns, subgoal_age_ns, stamp_01, stamp_02,
                 cmd_stamp_ns, cmd_age_ns, episode_id,
                 samples_01, samples_02, pedestrian_ids,
                 pedestrian_xy_map, pedestrian_yaw_map, pedestrian_leg_xy_map,
                 pedestrian_truth_stamp_ns, person_nearest_distance_m,
                 pedestrian_velocity_map):
    arrays = {
        key: concatenate_sensor_fields(first, second, key)
        for key in (
            "raw_ranges",
            "raw_angles_sensor",
            "points_x_base",
            "points_y_base",
            "virtual_ranges",
            "virtual_angles",
            "range_valid_mask",
            "self_mask",
            "valid_mask",
        )
    }
    np.savez_compressed(
        path,
        **arrays,
        semantic_label=semantic.astype(np.int16),
        source_sensor=np.concatenate(
            (np.zeros(samples_01, dtype=np.uint8), np.ones(samples_02, dtype=np.uint8))
        ),
        raw_beam_index=np.concatenate(
            (np.arange(samples_01, dtype=np.int32), np.arange(samples_02, dtype=np.int32))
        ),
        position=np.asarray(position, dtype=np.float32),
        velocity=np.asarray(velocity, dtype=np.float32),
        cmd_velocity=np.asarray(cmd_velocity, dtype=np.float32),
        cmd_vel_stamp_ns=np.int64(cmd_stamp_ns),
        cmd_vel_age_ns=np.int64(cmd_age_ns),
        sub_goal_local=np.asarray(subgoal, dtype=np.float32),
        local_subgoal_stamp_ns=np.int64(subgoal_stamp_ns),
        local_subgoal_age_ns=np.int64(subgoal_age_ns),
        scan_01_stamp_ns=np.int64(stamp_01),
        scan_02_stamp_ns=np.int64(stamp_02),
        episode_id=np.int64(episode_id),
        pedestrian_ids=np.asarray(pedestrian_ids, dtype=np.str_),
        pedestrian_xy_map=np.asarray(pedestrian_xy_map, dtype=np.float32).reshape((-1, 2)),
        pedestrian_yaw_map=np.asarray(pedestrian_yaw_map, dtype=np.float32),
        pedestrian_leg_xy_map=np.asarray(
            pedestrian_leg_xy_map, dtype=np.float32
        ).reshape((-1, 2, 2)),
        pedestrian_velocity_map=np.asarray(
            pedestrian_velocity_map, dtype=np.float32
        ).reshape((-1, 2)),
        pedestrian_truth_stamp_ns=np.int64(pedestrian_truth_stamp_ns),
        person_nearest_distance_m=np.asarray(person_nearest_distance_m, dtype=np.float32),
    )


def write_projection_previews(
    session_dir, pending, label_img, resolution, origin_x, origin_y, person_label_id,
    person_radius_m, person_label_mode
):
    preview_dir = session_dir / "projection_debug"
    preview_dir.mkdir()
    height, width = label_img.shape
    base = colorize_label_image(label_img)
    written = []
    for index in sorted({0, len(pending) // 2, len(pending) - 1}):
        first, second, semantic, pose = pending[index][:4]
        pedestrian_xy_map = pending[index][11]
        pedestrian_leg_xy_map = pending[index][13]
        truth_targets = (
            pedestrian_leg_xy_map.reshape((-1, 2))
            if person_label_mode == "ground-truth-legs"
            else pedestrian_xy_map
        )
        points_x = concatenate_sensor_fields(first, second, "points_x_base")
        points_y = concatenate_sensor_fields(first, second, "points_y_base")
        valid = concatenate_sensor_fields(first, second, "valid_mask")
        x, y, yaw = pose
        usable = np.flatnonzero(valid)
        world_x = x + math.cos(yaw) * points_x[usable] - math.sin(yaw) * points_y[usable]
        world_y = y + math.sin(yaw) * points_x[usable] + math.cos(yaw) * points_y[usable]
        cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
        rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
        in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        image = base.copy()
        for slot, row, col in zip(usable[in_map], rows[in_map], cols[in_map]):
            color = (
                (255, 255, 0)
                if person_label_id is not None
                and int(semantic[slot]) == person_label_id
                else (0, 255, 255)
            )
            image[max(0, row - 1) : min(height, row + 2), max(0, col - 1) : min(width, col + 2)] = color
        robot_col = int(math.floor((x - origin_x) / resolution))
        robot_row = int(height - 1 - math.floor((y - origin_y) / resolution))
        if 0 <= robot_col < width and 0 <= robot_row < height:
            image[max(0, robot_row - 3) : min(height, robot_row + 4), max(0, robot_col - 3) : min(width, robot_col + 4)] = (255, 0, 0)
        preview = Image.fromarray(image)
        draw = ImageDraw.Draw(preview)
        radius_pixels = max(1, int(round(person_radius_m / resolution)))
        for pedestrian_x, pedestrian_y in truth_targets:
            col = int(math.floor((pedestrian_x - origin_x) / resolution))
            row = int(height - 1 - math.floor((pedestrian_y - origin_y) / resolution))
            draw.ellipse(
                (
                    col - radius_pixels,
                    row - radius_pixels,
                    col + radius_pixels,
                    row + radius_pixels,
                ),
                outline=(255, 0, 255),
                width=2,
            )
        relative = Path("projection_debug") / f"projection_{index:07d}.png"
        preview.save(session_dir / relative)
        written.append(str(relative))
    return written


def main():
    args = parse_args()
    if min(args.samples_01, args.samples_02) <= 0:
        raise ValueError("--samples-01 and --samples-02 must be positive")
    ratios = (args.train_ratio, args.dev_ratio, args.test_ratio)
    if min(ratios) < 0 or not math.isclose(sum(ratios), 1.0, abs_tol=1e-9):
        raise ValueError("train/dev/test ratios must be non-negative and sum to 1")
    if args.sync_tolerance_ms < 0:
        raise ValueError("--sync-tolerance-ms must be non-negative")
    if args.subgoal_max_age_ms < 0:
        raise ValueError("--subgoal-max-age-ms must be non-negative")
    if not math.isfinite(args.cmd_vel_max_age_ms) or args.cmd_vel_max_age_ms <= 0:
        raise ValueError("--cmd-vel-max-age-ms must be positive and finite")
    if (
        not math.isfinite(args.cmd_vel_angular_z_relay_scale)
        or args.cmd_vel_angular_z_relay_scale <= 0.0
    ):
        raise ValueError(
            "--cmd-vel-angular-z-relay-scale must be positive and finite"
        )
    if args.pose_source == "odom" and not args.allow_odom_map_alignment:
        raise ValueError("--pose-source odom requires --allow-odom-map-alignment")
    for required in (args.bag, args.map_yaml, args.semantic_label):
        if not required.exists():
            raise FileNotFoundError(required)

    session_dir = args.output_root / args.session_name
    if session_dir.exists():
        if not args.overwrite:
            raise FileExistsError(f"{session_dir} exists; refusing to overwrite")
        shutil.rmtree(session_dir)
    args.output_root.mkdir(parents=True, exist_ok=True)
    temp_dir = args.output_root / f".{args.session_name}.incomplete-{os.getpid()}"
    if temp_dir.exists():
        shutil.rmtree(temp_dir)
    samples_dir = temp_dir / "samples"
    samples_dir.mkdir(parents=True)

    try:
        (
            scans_01,
            scans_02,
            odoms,
            cmds,
            cmd_stamped,
            clocks,
            tf_index,
            static_tf_3d,
            topic_counts,
            pedestrians,
            local_subgoal_messages,
            global_path_messages,
            final_goal_messages,
            episode_events,
        ) = read_dual_bag(args)
        if not scans_01 or not scans_02 or not odoms:
            raise RuntimeError("Both scan streams and odometry must contain messages")
        if args.subgoal_source == "online":
            empty_topics = [
                topic
                for topic, messages in (
                    (args.local_subgoal_topic, local_subgoal_messages),
                    (args.global_path_topic, global_path_messages),
                    (args.final_goal_topic, final_goal_messages),
                )
                if not messages
            ]
            if empty_topics:
                raise RuntimeError(
                    "Required online goal topic(s) contain no messages: "
                    + ", ".join(empty_topics)
                )
            base_frame = normalize_frame(args.base_frame)
            map_frame = normalize_frame(args.map_frame)
            local_frames = {item[2] for item in local_subgoal_messages}
            path_frames = {item[1] for item in global_path_messages}
            final_frames = {item[2] for item in final_goal_messages}
            if local_frames != {base_frame}:
                raise RuntimeError(
                    f"{args.local_subgoal_topic} frame(s) {sorted(local_frames)} "
                    f"do not equal {base_frame}"
                )
            if path_frames != {map_frame}:
                raise RuntimeError(
                    f"{args.global_path_topic} frame(s) {sorted(path_frames)} "
                    f"do not equal {map_frame}"
                )
            if final_frames != {map_frame}:
                raise RuntimeError(
                    f"{args.final_goal_topic} frame(s) {sorted(final_frames)} "
                    f"do not equal {map_frame}"
                )
            if any(
                not all(math.isfinite(value) for value in item[1])
                for item in local_subgoal_messages
            ):
                raise RuntimeError(
                    f"{args.local_subgoal_topic} contains non-finite coordinates"
                )

        resolution, origin_x, origin_y, origin_yaw = load_map_info(args.map_yaml)
        label_img = np.asarray(Image.open(args.semantic_label))
        if label_img.ndim == 3:
            label_img = label_img[:, :, 0]
        label_img = label_img.astype(np.int64)
        occupancy_img, occupancy_free, occupancy_path, negate, occupied_thresh, free_thresh = (
            load_occupancy_map(args.map_yaml)
        )
        if label_img.shape != occupancy_img.shape:
            raise ValueError(
                f"semantic label shape {label_img.shape} != occupancy shape {occupancy_img.shape}"
        )
        label_names, label_names_source = load_label_names(args.semantic_label)
        if label_names_source == "default":
            raise FileNotFoundError(
                "missing label_names.txt next to semantic label: "
                f"{args.semantic_label.with_name('label_names.txt')}"
            )
        label_names_path = Path(label_names_source)
        validate_label_image(label_img, label_names)
        person_label_id = find_label_id(label_names, "Person")
        if args.person_label_mode != "disabled" and person_label_id is None:
            raise ValueError(
                "Person labeling requires a Person entry in "
                f"{label_names_path}; set --person-label-mode disabled to turn "
                "off free-space Person labeling"
            )
        if args.person_label_mode in ("ground-truth-legs", "ground-truth-radius"):
            if not pedestrians:
                raise RuntimeError(
                    f"{args.pedestrian_ground_truth_topic} has no messages"
                )
            if args.person_ground_truth_radius_m <= 0:
                raise ValueError("--person-ground-truth-radius-m must be positive")
            if args.person_ground_truth_leg_match_radius_m <= 0:
                raise ValueError(
                    "--person-ground-truth-leg-match-radius-m must be positive"
                )
            if args.person_ground_truth_max_delta_ms < 0:
                raise ValueError(
                    "--person-ground-truth-max-delta-ms must be non-negative"
                )
        tolerance_ns = int(round(args.sync_tolerance_ms * 1_000_000.0))
        paired_scans, sync_skipped, unpaired_scan_02 = pair_synchronized_scans(
            scans_01, scans_02, tolerance_ns
        )
        if not paired_scans:
            raise RuntimeError("No synchronized dual-LiDAR scan pairs were found")
        initial_paired_scan_count = len(paired_scans)
        common_start_ns, common_end_ns, common_stream_ranges = (
            common_valid_time_range(
                clocks, scans_01, scans_02, odoms, cmd_stamped
            )
        )
        retained_common = [
            pair
            for pair in paired_scans
            if common_start_ns <= int(pair[0]) <= common_end_ns
            and common_start_ns <= int(pair[2]) <= common_end_ns
        ]
        drop_counts = Counter(
            {
                "outside_common_range": len(paired_scans) - len(retained_common),
                "outside_episode": 0,
                "no_causal_cmd": 0,
                "stale_cmd_vel": 0,
                "no_causal_subgoal": 0,
                "stale_subgoal": 0,
                "scan_01_sync_unmatched": sync_skipped,
                "scan_02_sync_unmatched": unpaired_scan_02,
                "odom_unavailable": 0,
                "terminal_goal_stop": 0,
            }
        )
        paired_scans = retained_common
        mapped_episode_events, episode_time_mapping = (
            map_episode_events_to_sim_time(episode_events, clocks)
        )
        episode_intervals = episode_intervals_from_events(mapped_episode_events)
        source_complete_episode_count = len(episode_intervals)
        if args.successful_episodes_only:
            episode_intervals = successful_episode_intervals(episode_intervals)
            if not episode_intervals:
                raise RuntimeError(
                    "--successful-episodes-only found no goal-success episode"
                )
        paired_episode_ids = [0] * len(paired_scans)
        episode_unassigned_paired_scans_dropped = 0
        if episode_intervals:
            paired_episode_ids = episode_ids_for_stamps(
                [item[0] for item in paired_scans], episode_intervals
            )
            retained = [
                (pair, episode_id)
                for pair, episode_id in zip(paired_scans, paired_episode_ids)
                if episode_id > 0
            ]
            episode_unassigned_paired_scans_dropped = (
                len(paired_scans) - len(retained)
            )
            drop_counts["outside_episode"] = (
                episode_unassigned_paired_scans_dropped
            )
            paired_scans = [item[0] for item in retained]
            paired_episode_ids = [item[1] for item in retained]
            if not paired_scans:
                raise RuntimeError(
                    "No synchronized scan pair falls inside a complete episode"
                )
        subgoal_alignment = {
            "leading_unmatched_frames_dropped": 0,
            "missing_causal_frames_dropped": 0,
            "stale_frames_dropped": 0,
            "matched_frames": 0,
            "age_ns_min": None,
            "age_ns_max": None,
            "age_ns_mean": None,
        }
        paired_subgoal_matches = [None] * len(paired_scans)
        if args.subgoal_source == "online":
            subgoal_match_args = (
                [item[0] for item in paired_scans],
                local_subgoal_messages,
                int(round(args.subgoal_max_age_ms * 1_000_000.0)),
            )
            if episode_intervals:
                paired_subgoal_matches, subgoal_alignment = (
                    causal_hold_last_subgoals_by_episode(
                        subgoal_match_args[0],
                        paired_episode_ids,
                        subgoal_match_args[1],
                        episode_intervals,
                        subgoal_match_args[2],
                    )
                )
            else:
                paired_subgoal_matches, subgoal_alignment = (
                    causal_hold_last_subgoals(*subgoal_match_args)
                )
            retained = [
                (pair, match, episode_id)
                for pair, match, episode_id in zip(
                    paired_scans, paired_subgoal_matches, paired_episode_ids
                )
                if match is not None
            ]
            if subgoal_alignment["leading_unmatched_frames_dropped"]:
                print(
                    "INFO: dropping "
                    f"{subgoal_alignment['leading_unmatched_frames_dropped']} "
                    "leading synchronized frame(s) before the first causal "
                    "online subgoal",
                    file=sys.stderr,
                )
            paired_scans = [item[0] for item in retained]
            paired_subgoal_matches = [item[1] for item in retained]
            paired_episode_ids = [item[2] for item in retained]
            drop_counts["no_causal_subgoal"] = subgoal_alignment[
                "missing_causal_frames_dropped"
            ]
            drop_counts["stale_subgoal"] = subgoal_alignment[
                "stale_frames_dropped"
            ]
        cmd_stamped_raw_count = len(cmd_stamped)
        cmd_stamped, cmd_boundary_trim = trim_safe_boundary_cmd_vel_stamped(
            cmd_stamped, clocks, paired_scans
        )
        cmd_alignment = build_aligned_cmd_velocities(cmds, cmd_stamped, clocks)
        if cmd_alignment["alignment_status"] != "safe":
            raise RuntimeError(
                f"Unsafe command alignment: {cmd_alignment['warnings']}"
            )
        aligned_cmds = cmd_alignment["commands"]
        if (
            not math.isfinite(args.reverse_linear_x_epsilon)
            or args.reverse_linear_x_epsilon <= 0.0
        ):
            raise ValueError(
                "--reverse-linear-x-epsilon must be positive and finite"
            )
        if args.reverse_recovery_frames < 0:
            raise ValueError("--reverse-recovery-frames must be non-negative")

        retained_with_commands = []
        for pair, subgoal_match, episode_id in zip(
            paired_scans, paired_subgoal_matches, paired_episode_ids
        ):
            cmd_match, cmd_status = hold_last_by_time(
                aligned_cmds, int(pair[0])
            )
            if cmd_match is None:
                drop_counts["no_causal_cmd"] += 1
                continue
            retained_with_commands.append(
                (pair, subgoal_match, episode_id, cmd_match, cmd_status)
            )
        paired_scans = [item[0] for item in retained_with_commands]
        paired_subgoal_matches = [
            item[1] for item in retained_with_commands
        ]
        paired_episode_ids = [item[2] for item in retained_with_commands]
        paired_cmd_matches = [item[3] for item in retained_with_commands]
        paired_cmd_statuses = [item[4] for item in retained_with_commands]

        reverse_motion_filter = {
            "enabled": False,
            "policy": "disabled",
            "linear_x_epsilon": float(args.reverse_linear_x_epsilon),
            "recovery_frames_requested": int(args.reverse_recovery_frames),
            "reverse_frames_removed": 0,
            "recovery_frames_removed": 0,
            "total_frames_removed": 0,
            "affected_episode_ids": [],
            "reverse_frames_by_episode": {},
            "recovery_frames_by_episode": {},
        }
        if args.exclude_reverse_linear_x:
            keep_mask, reverse_motion_filter = reverse_motion_keep_mask(
                [item[1] for item in paired_cmd_matches],
                paired_episode_ids,
                args.reverse_linear_x_epsilon,
                args.reverse_recovery_frames,
            )
            retained_forward_only = [
                item
                for item, keep in zip(
                    zip(
                        paired_scans,
                        paired_subgoal_matches,
                        paired_episode_ids,
                        paired_cmd_matches,
                        paired_cmd_statuses,
                    ),
                    keep_mask,
                )
                if keep
            ]
            drop_counts["reverse_command"] = reverse_motion_filter[
                "reverse_frames_removed"
            ]
            drop_counts["reverse_recovery"] = reverse_motion_filter[
                "recovery_frames_removed"
            ]
            paired_scans = [item[0] for item in retained_forward_only]
            paired_subgoal_matches = [
                item[1] for item in retained_forward_only
            ]
            paired_episode_ids = [
                item[2] for item in retained_forward_only
            ]
            paired_cmd_matches = [
                item[3] for item in retained_forward_only
            ]
            paired_cmd_statuses = [
                item[4] for item in retained_forward_only
            ]
            if not paired_scans:
                raise RuntimeError(
                    "Forward-only filtering removed every synchronized sample"
                )

        terminal_goal_stop_filter = {
            "enabled": False,
            "policy": "included-by-explicit-legacy-option",
            "epsilon": float(ZERO_COMMAND_EPSILON),
            "total_frames_removed": 0,
            "removed_frames_by_episode": {},
        }
        if not args.include_terminal_stop_frames and episode_intervals:
            successful_ids = {
                int(interval["episode_id"])
                for interval in episode_intervals
                if interval.get("finish_reason")
                in SUCCESSFUL_EPISODE_FINISH_REASONS
            }
            keep_mask, terminal_goal_stop_filter = terminal_goal_stop_keep_mask(
                [item[1] for item in paired_cmd_matches],
                paired_episode_ids,
                successful_ids,
            )
            retained_nonterminal = [
                item
                for item, keep in zip(
                    zip(
                        paired_scans,
                        paired_subgoal_matches,
                        paired_episode_ids,
                        paired_cmd_matches,
                        paired_cmd_statuses,
                    ),
                    keep_mask,
                )
                if keep
            ]
            drop_counts["terminal_goal_stop"] = terminal_goal_stop_filter[
                "total_frames_removed"
            ]
            paired_scans = [item[0] for item in retained_nonterminal]
            paired_subgoal_matches = [item[1] for item in retained_nonterminal]
            paired_episode_ids = [item[2] for item in retained_nonterminal]
            paired_cmd_matches = [item[3] for item in retained_nonterminal]
            paired_cmd_statuses = [item[4] for item in retained_nonterminal]
            if not paired_scans:
                raise RuntimeError(
                    "Terminal-goal stop filtering removed every synchronized sample"
                )

        pending = []
        positions = []
        label_hist = Counter()
        pose_sources = Counter()
        sync_deltas_ns = []
        valid_counts_01, valid_counts_02 = [], []
        self_counts_01, self_counts_02 = [], []
        cmd_status_counts = Counter()
        person_truth_deltas_ns = []
        person_truth_unmatched_scans = 0
        person_labeled_distances = []
        frames_01, frames_02 = Counter(), Counter()
        odom_skipped = 0
        initial_layout_01 = None
        initial_layout_02 = None
        fixed_self_mask_01 = None
        fixed_self_mask_02 = None
        self_mask_calibration = None
        retained_subgoal_matches = []
        sample_episode_ids = []

        for (
            stamp_01,
            scan_01,
            stamp_02,
            scan_02,
        ), subgoal_match, episode_id, cmd_match, cmd_status in zip(
            paired_scans,
            paired_subgoal_matches,
            paired_episode_ids,
            paired_cmd_matches,
            paired_cmd_statuses,
        ):
            cmd_stamp_ns = int(cmd_match[0])
            cmd_age_ns = int(stamp_01) - cmd_stamp_ns
            if cmd_age_ns < 0:
                raise RuntimeError("future /cmd_vel_stamped selected for a scan")
            if cmd_age_ns > int(round(args.cmd_vel_max_age_ms * 1_000_000.0)):
                drop_counts["stale_cmd_vel"] += 1
                continue
            cmd_velocity = cmd_match[1]
            odom_match = nearest_by_time(odoms, stamp_01)
            if odom_match is None:
                odom_skipped += 1
                drop_counts["odom_unavailable"] += 1
                continue
            odom_pose = odom_match[1]
            pose, pose_source = projection_pose(args, stamp_01, odom_pose, tf_index)
            first = sensor_slots(
                scan_01,
                args.samples_01,
                static_tf_3d,
                args.base_frame,
                fixed_self_mask_01,
            )
            second = sensor_slots(
                scan_02,
                args.samples_02,
                static_tf_3d,
                args.base_frame,
                fixed_self_mask_02,
            )
            current_layout_01 = sensor_layout(first)
            current_layout_02 = sensor_layout(second)
            if initial_layout_01 is None:
                initial_layout_01 = current_layout_01
                initial_layout_02 = current_layout_02
                if args.self_mask_mode == "first-synchronized-pair-fixed-beam-identity":
                    fixed_self_mask_01 = first["footprint_self_mask"].copy()
                    fixed_self_mask_02 = second["footprint_self_mask"].copy()
                    first["self_mask"] = fixed_self_mask_01.copy()
                    second["self_mask"] = fixed_self_mask_02.copy()
                    first["valid_mask"] = first["range_valid_mask"] & ~first["self_mask"]
                    second["valid_mask"] = second["range_valid_mask"] & ~second["self_mask"]
                    for slots in (first, second):
                        invalid = ~slots["valid_mask"]
                        slots["virtual_ranges"][invalid] = np.nan
                        slots["virtual_angles"][invalid] = np.nan
                    self_mask_calibration = {
                        "source_sample_index": 0,
                        "scan_01_stamp_ns": int(stamp_01),
                        "scan_02_stamp_ns": int(stamp_02),
                        "scan_01_masked_beam_runs": mask_runs(fixed_self_mask_01),
                        "scan_02_masked_beam_runs": mask_runs(fixed_self_mask_02),
                        "scan_01_masked_beam_count": int(fixed_self_mask_01.sum()),
                        "scan_02_masked_beam_count": int(fixed_self_mask_02.sum()),
                        "footprint_half_extents_m": list(SELF_FOOTPRINT_HALF_EXTENTS_M),
                    }
            else:
                require_matching_sensor_layout(
                    initial_layout_01, current_layout_01, args.scan_01_topic
                )
                require_matching_sensor_layout(
                    initial_layout_02, current_layout_02, args.scan_02_topic
                )
            virtual_ranges = concatenate_sensor_fields(first, second, "virtual_ranges")
            virtual_angles = concatenate_sensor_fields(first, second, "virtual_angles")
            valid = concatenate_sensor_fields(first, second, "valid_mask")
            semantic = semantic_for_scan(
                virtual_ranges,
                virtual_angles,
                valid,
                pose,
                label_img,
                occupancy_free,
                resolution,
                origin_x,
                origin_y,
                label_names,
                args.static_label_filter_radius,
                "dynamic" if args.person_label_mode == "dynamic" else "disabled",
            )
            pedestrian_ids = []
            pedestrian_xy_map = np.empty((0, 2), dtype=np.float64)
            pedestrian_yaw_map = np.empty(0, dtype=np.float64)
            pedestrian_leg_xy_map = np.empty((0, 2, 2), dtype=np.float64)
            pedestrian_velocity_map = np.empty((0, 2), dtype=np.float64)
            pedestrian_truth_stamp_ns = -1
            person_nearest_distance_m = np.full(
                len(semantic), np.nan, dtype=np.float32
            )
            if args.person_label_mode in ("ground-truth-legs", "ground-truth-radius"):
                truth_match = nearest_by_time(pedestrians, stamp_01)
                truth_delta_ns = abs(int(truth_match[0]) - int(stamp_01))
                max_delta_ns = int(
                    round(args.person_ground_truth_max_delta_ms * 1_000_000.0)
                )
                if truth_delta_ns > max_delta_ns:
                    # A bag can contain scan frames just outside the truth topic's
                    # active interval. Leaving Person disabled for those frames is
                    # conservative and prevents stale truth from creating labels.
                    person_truth_unmatched_scans += 1
                else:
                    (
                        pedestrian_ids,
                        pedestrian_xy_map,
                        pedestrian_yaw_map,
                        pedestrian_velocity_map,
                        pedestrian_truth_stamp_ns,
                    ) = pedestrian_truth_in_map(
                        truth_match, stamp_01, tf_index, args.map_frame
                    )
                    pedestrian_leg_xy_map = pedestrian_leg_centers(
                        pedestrian_xy_map, pedestrian_yaw_map
                    )
                    if args.person_label_mode == "ground-truth-legs":
                        person_targets = pedestrian_leg_xy_map.reshape((-1, 2))
                        person_match_radius = (
                            args.person_ground_truth_leg_match_radius_m
                        )
                    else:
                        person_targets = pedestrian_xy_map
                        person_match_radius = args.person_ground_truth_radius_m
                    semantic, person_nearest_distance_m = apply_ground_truth_person_labels(
                        semantic,
                        valid,
                        virtual_ranges,
                        virtual_angles,
                        pose,
                        person_targets,
                        person_label_id,
                        person_match_radius,
                    )
                    person_truth_deltas_ns.append(truth_delta_ns)
                    person_labeled_distances.extend(
                        person_nearest_distance_m[semantic == person_label_id].tolist()
                    )
            pending.append(
                (
                    first,
                    second,
                    semantic,
                    pose,
                    (odom_pose[3], odom_pose[4]),
                    cmd_velocity,
                    stamp_01,
                    stamp_02,
                    cmd_stamp_ns,
                    cmd_age_ns,
                    pedestrian_ids,
                    pedestrian_xy_map,
                    pedestrian_yaw_map,
                    pedestrian_leg_xy_map,
                    pedestrian_truth_stamp_ns,
                    person_nearest_distance_m,
                    pedestrian_velocity_map,
                )
            )
            retained_subgoal_matches.append(subgoal_match)
            sample_episode_ids.append(episode_id)
            positions.append(np.asarray(pose, dtype=np.float32))
            label_hist.update(int(value) for value in semantic.tolist())
            pose_sources[pose_source] += 1
            sync_deltas_ns.append(abs(stamp_02 - stamp_01))
            valid_counts_01.append(int(first["valid_mask"].sum()))
            valid_counts_02.append(int(second["valid_mask"].sum()))
            self_counts_01.append(int(first["self_mask"].sum()))
            self_counts_02.append(int(second["self_mask"].sum()))
            cmd_status_counts[cmd_status] += 1
            frames_01[first["frame_id"]] += 1
            frames_02[second["frame_id"]] += 1

        if not pending:
            raise RuntimeError("No synchronized dual-LiDAR samples were converted")
        positions_array = np.stack(positions).astype(np.float32)
        if args.subgoal_source == "online":
            subgoals = np.asarray(
                [item[1] for item in retained_subgoal_matches], dtype=np.float32
            )
            subgoal_stamps_ns = [
                int(item[0]) for item in retained_subgoal_matches
            ]
            subgoal_ages_ns = [
                int(item[2]) for item in retained_subgoal_matches
            ]
        else:
            subgoals = local_subgoals_by_episode(
                positions_array,
                sample_episode_ids,
                args.subgoal_lookahead,
            )
            subgoal_stamps_ns = [-1] * len(pending)
            subgoal_ages_ns = [-1] * len(pending)
        filenames = []
        for index, (sample, subgoal, subgoal_stamp_ns, subgoal_age_ns) in enumerate(
            zip(pending, subgoals, subgoal_stamps_ns, subgoal_ages_ns)
        ):
            name = f"{index:07d}.npz"
            filenames.append(name)
            write_sample(
                samples_dir / name,
                *sample[:6],
                subgoal,
                subgoal_stamp_ns,
                subgoal_age_ns,
                sample[6],
                sample[7],
                sample[8],
                sample[9],
                sample_episode_ids[index],
                args.samples_01,
                args.samples_02,
                *sample[10:],
            )

        if episode_intervals:
            train, dev, test = split_filenames_by_episode(
                filenames,
                sample_episode_ids,
                args.train_ratio,
                args.dev_ratio,
                args.split_seed,
            )
            split_unit = "episode"
        else:
            train, dev, test = split_filenames(
                filenames, args.train_ratio, args.dev_ratio, args.split_seed
            )
            split_unit = "frame-legacy"
        for split, values in (("train", train), ("dev", dev), ("test", test)):
            (temp_dir / f"{split}.txt").write_text(
                "\n".join(values) + "\n", encoding="utf-8"
            )
        projection_debug_files = write_projection_previews(
            temp_dir,
            pending,
            label_img,
            resolution,
            origin_x,
            origin_y,
            person_label_id,
            (
                args.person_ground_truth_leg_match_radius_m
                if args.person_label_mode == "ground-truth-legs"
                else args.person_ground_truth_radius_m
            ),
            args.person_label_mode,
        )
        subgoal_metadata = subgoal_contract_metadata(
            args.subgoal_source,
            args.subgoal_lookahead,
            args.subgoal_max_age_ms,
            subgoal_alignment["leading_unmatched_frames_dropped"],
            (
                subgoal_ages_ns
                if args.subgoal_source == "online"
                else []
            ),
            subgoal_alignment["stale_frames_dropped"],
        )
        episode_sample_counts = Counter(int(value) for value in sample_episode_ids)
        episode_metadata = [
            {
                **interval,
                "sample_count": int(
                    episode_sample_counts[int(interval["episode_id"])]
                ),
            }
            for interval in episode_intervals
        ]

        metadata = {
            "format": "v7-fixed-dual-lidar-slots-v3",
            "bag": str(args.bag.resolve()),
            "output_root": str(args.output_root.resolve()),
            "session_name": args.session_name,
            "scan_01_topic": args.scan_01_topic,
            "scan_02_topic": args.scan_02_topic,
            "samples_01": args.samples_01,
            "samples_02": args.samples_02,
            "total_slots": args.samples_01 + args.samples_02,
            "slot_contract": (
                f"slots 0..{args.samples_01 - 1} are raw {args.scan_01_topic}; "
                f"slots {args.samples_01}..{args.samples_01 + args.samples_02 - 1} "
                f"are raw {args.scan_02_topic}"
            ),
            "no_resampling": True,
            "no_cross_sensor_deduplication": True,
            "no_angle_binning": True,
            "no_scan_fusion": True,
            "uses_scan_merged": False,
            **subgoal_metadata,
            "local_subgoal_topic": args.local_subgoal_topic,
            "global_path_topic": args.global_path_topic,
            "final_goal_topic": args.final_goal_topic,
            "local_subgoal_messages": len(local_subgoal_messages),
            "global_path_messages": len(global_path_messages),
            "final_goal_messages": len(final_goal_messages),
            "episode_event_topic": args.episode_event_topic,
            "episode_event_messages": len(episode_events),
            "episode_time_mapping": episode_time_mapping,
            "episode_count": len(episode_metadata),
            "episodes": episode_metadata,
            "episode_filter": {
                "mode": (
                    "successful_only"
                    if args.successful_episodes_only
                    else "all_complete"
                ),
                "source_complete_episode_count": source_complete_episode_count,
                "selected_episode_count": len(episode_metadata),
                "successful_finish_reasons": sorted(
                    SUCCESSFUL_EPISODE_FINISH_REASONS
                ),
            },
            "terminal_goal_stop_filter": terminal_goal_stop_filter,
            "episode_unassigned_paired_scans_dropped": (
                episode_unassigned_paired_scans_dropped
            ),
            "sensor_extrinsic_transform": "full 3D quaternion TF, then XY projection",
            "scan_01_layout": initial_layout_01,
            "scan_02_layout": initial_layout_02,
            "self_mask_mode": args.self_mask_mode,
            "self_mask_calibration": self_mask_calibration,
            "projection_debug_files": projection_debug_files,
            "map_yaml": str(args.map_yaml.resolve()),
            "map_yaml_sha256": sha256_file(args.map_yaml),
            "semantic_label": str(args.semantic_label.resolve()),
            "semantic_label_sha256": sha256_file(args.semantic_label),
            "occupancy_image": str(occupancy_path.resolve()),
            "occupancy_image_sha256": sha256_file(occupancy_path),
            "map": {
                "resolution": resolution,
                "origin": [origin_x, origin_y, origin_yaw],
                "negate": negate,
                "occupied_thresh": occupied_thresh,
                "free_thresh": free_thresh,
            },
            "label_names": label_names,
            "label_names_source": label_names_source,
            "label_names_sha256": sha256_file(label_names_path),
            "person_label_mode": args.person_label_mode,
            "person_label_id": person_label_id if args.person_label_mode != "disabled" else None,
            "person_label_rule": (
                "valid LiDAR endpoint within configured XY radius of either "
                "lower-leg center derived from nearest-time pedestrian pose -> Person"
                if args.person_label_mode == "ground-truth-legs"
                else
                "valid LiDAR endpoint within configured XY radius of nearest-time "
                "pedestrian ground-truth center -> Person"
                if args.person_label_mode == "ground-truth-radius"
                else "unlabeled endpoint in occupancy free space -> Person"
                if args.person_label_mode == "dynamic"
                else "disabled; unlabeled endpoint -> ignore=-1"
            ),
            "pedestrian_ground_truth_topic": args.pedestrian_ground_truth_topic,
            "person_ground_truth_radius_m": (
                args.person_ground_truth_radius_m
                if args.person_label_mode == "ground-truth-radius"
                else None
            ),
            "person_ground_truth_leg_lateral_offset_m": (
                PEDESTRIAN_LEG_LATERAL_OFFSET_M
                if args.person_label_mode == "ground-truth-legs"
                else None
            ),
            "person_ground_truth_leg_physical_radius_m": (
                PEDESTRIAN_LEG_RADIUS_M
                if args.person_label_mode == "ground-truth-legs"
                else None
            ),
            "person_ground_truth_leg_match_radius_m": (
                args.person_ground_truth_leg_match_radius_m
                if args.person_label_mode == "ground-truth-legs"
                else None
            ),
            "person_ground_truth_max_delta_ms": (
                args.person_ground_truth_max_delta_ms
                if args.person_label_mode in ("ground-truth-legs", "ground-truth-radius")
                else None
            ),
            "person_ground_truth_match_delta_ms_max": (
                max(person_truth_deltas_ns) / 1_000_000.0
                if person_truth_deltas_ns
                else None
            ),
            "person_ground_truth_match_delta_ms_mean": (
                float(np.mean(person_truth_deltas_ns)) / 1_000_000.0
                if person_truth_deltas_ns
                else None
            ),
            "person_ground_truth_unmatched_scans": person_truth_unmatched_scans,
            "pedestrian_ground_truth_exported": args.person_label_mode
            in ("ground-truth-legs", "ground-truth-radius"),
            "pedestrian_count": max(
                (len(sample[10]) for sample in pending),
                default=0,
            ),
            "person_labeled_nearest_distance_m_max": (
                max(person_labeled_distances) if person_labeled_distances else None
            ),
            "static_label_filter_radius": args.static_label_filter_radius,
            "pose_source_requested": args.pose_source,
            "pose_source_used": dict(pose_sources),
            "odom_map_alignment_explicitly_allowed": bool(
                args.allow_odom_map_alignment
            ),
            "base_frame": normalize_frame(args.base_frame),
            "map_frame": normalize_frame(args.map_frame),
            "scan_01_frames": dict(frames_01),
            "scan_02_frames": dict(frames_02),
            "sync_tolerance_ms": args.sync_tolerance_ms,
            "sync_delta_ms_max": max(sync_deltas_ns) / 1_000_000.0,
            "sync_delta_ms_mean": float(np.mean(sync_deltas_ns)) / 1_000_000.0,
            "scan_01_messages": len(scans_01),
            "scan_02_messages": len(scans_02),
            "sync_skipped": sync_skipped,
            "unpaired_scan_02": unpaired_scan_02,
            "odom_skipped": odom_skipped,
            "initial_paired_scans": initial_paired_scan_count,
            "common_valid_time_range": {
                "start_ns": common_start_ns,
                "end_ns": common_end_ns,
                "streams": common_stream_ranges,
            },
            "drop_counts": {
                key: int(value) for key, value in sorted(drop_counts.items())
            },
            "samples": len(filenames),
            "train_samples": len(train),
            "dev_samples": len(dev),
            "test_samples": len(test),
            "split_seed": args.split_seed,
            "split_unit": split_unit,
            "train_ratio": args.train_ratio,
            "dev_ratio": args.dev_ratio,
            "test_ratio": args.test_ratio,
            "label_histogram": {
                str(key): int(value) for key, value in sorted(label_hist.items())
            },
            "average_valid_scan_01": float(np.mean(valid_counts_01)),
            "average_valid_scan_02": float(np.mean(valid_counts_02)),
            "average_self_mask_scan_01": float(np.mean(self_counts_01)),
            "average_self_mask_scan_02": float(np.mean(self_counts_02)),
            "cmd_velocities_source": cmd_alignment["source"],
            "cmd_vel_alignment_method": cmd_alignment["alignment_method"],
            "cmd_vel_alignment_status": cmd_alignment["alignment_status"],
            "cmd_vel_max_age_ms": args.cmd_vel_max_age_ms,
            "cmd_label_interface": args.cmd_label_interface,
            "cmd_vel_angular_z_relay_scale": (
                args.cmd_vel_angular_z_relay_scale
            ),
            "cmd_vel_match_status_counts": dict(cmd_status_counts),
            "cmd_vel_distribution": command_distribution(
                [sample[5] for sample in pending]
            ),
            "cmd_vel_stamped_raw_count": cmd_stamped_raw_count,
            "cmd_vel_stamped_count": len(cmd_stamped),
            "cmd_vel_stamped_boundary_trim": cmd_boundary_trim,
            "reverse_motion_filter": reverse_motion_filter,
            "scan_time_range": time_range_ns([item[0] for item in scans_01]),
            "odom_time_range": time_range_ns([item[0] for item in odoms]),
            "cmd_vel_mapped_time_range": time_range_ns(
                [item[0] for item in aligned_cmds]
            ),
            "topic_counts": dict(topic_counts),
        }
        (temp_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        temp_dir.rename(session_dir)
        print(json.dumps(metadata, indent=2, sort_keys=True))
    except Exception:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise


if __name__ == "__main__":
    main()
