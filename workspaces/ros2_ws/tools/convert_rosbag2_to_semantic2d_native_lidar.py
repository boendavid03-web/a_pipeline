#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --base-frame, --cmd-vel-stamped-topic, --cmd-vel-topic, --dev-ratio, --map-frame, --map-yaml, --odom-topic, --output-root, --overwrite, --person-label-mode, --pose-source, --rewrite-dataset-index, --scan-topic, --semantic-label, --session-name, --split-seed, --static-label-filter-radius, --strict-tf, --subgoal-lookahead, --test-ratio, --train-ratio, --write-projection-debug
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /cmd_vel_stamped, /odom, /scan_01, /scan_02, /scan_merged, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, NPY, PNG, TXT
# 可能使用的关键环境变量：CMD_VELOCITY_DIM, DEFAULT_LABEL_NAMES, IGNORE_LABEL, PERSON_LABEL_MODES, POSE_SOURCES, WARNING
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_native_lidar.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.685303552 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:31.258092381 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_native_lidar.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜convert_rosbag2_to_semantic2d_native_lidar.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Convert a ROS 2 LaserScan/Odometry bag to a native-LiDAR Semantic2D dataset."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import defaultdict, deque
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IGNORE_LABEL = -1
PERSON_LABEL_MODES = ("dynamic", "disabled")
CMD_VELOCITY_DIM = 3
POSE_SOURCES = ("odom", "tf-map-base", "tf-map-scan", "auto")
DEFAULT_LABEL_NAMES = [
    "_background_",
    "Chair",
    "Door",
    "Elevator",
    "Person",
    "Pillar",
    "Sofa",
    "Table",
    "Trash bin",
    "Wall",
]


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def msg_time_ns(msg, fallback_ns: int) -> int:
    header = getattr(msg, "header", None)
    if header is None:
        return fallback_ns
    return stamp_to_ns(header.stamp)


def load_label_names(semantic_label: Path):
    label_names_path = semantic_label.with_name("label_names.txt")
    if label_names_path.exists():
        names = [line.strip() for line in label_names_path.read_text(encoding="utf-8").splitlines()]
        names = [name for name in names if name]
        if names:
            return names, str(label_names_path)
    return list(DEFAULT_LABEL_NAMES), "default"


def label_name(label_names, label: int) -> str:
    if label == IGNORE_LABEL:
        return "ignore"
    if 0 <= label < len(label_names):
        return label_names[label]
    return str(label)


def find_label_id(label_names, name: str):
    normalized_name = name.casefold()
    for index, label in enumerate(label_names):
        if label.casefold() == normalized_name:
            return index
    return None


def validate_label_image(label_img, label_names):
    if len(label_names) < 2 or label_names[0] != "_background_":
        raise ValueError(
            "label_names.txt must start with _background_ and include at least one semantic class"
        )
    invalid = (label_img < 0) | (label_img >= len(label_names))
    if np.any(invalid):
        values = np.unique(label_img[invalid])[:10].tolist()
        raise ValueError(
            f"semantic label image contains IDs outside label_names.txt: {values}; "
            f"expected 0..{len(label_names) - 1}"
        )


def summarize_values(values):
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "max": None,
            "unique_count": 0,
            "unique_preview": [],
        }
    unique = np.unique(arr)
    return {
        "count": int(arr.size),
        "min": float(arr.min()),
        "mean": float(arr.mean()),
        "max": float(arr.max()),
        "unique_count": int(unique.size),
        "unique_preview": [float(v) for v in unique[:12]],
    }


def summarize_vectors(samples, fields):
    return {
        name: summarize_values([sample[index] for sample in samples])
        for name, index in fields
    }


def time_range_ns(times):
    if not times:
        return {
            "count": 0,
            "start_ns": None,
            "end_ns": None,
            "start_sec": None,
            "end_sec": None,
        }
    start = int(min(times))
    end = int(max(times))
    return {
        "count": int(len(times)),
        "start_ns": start,
        "end_ns": end,
        "start_sec": float(start / 1_000_000_000.0),
        "end_sec": float(end / 1_000_000_000.0),
    }


def ranges_overlap(a: dict, b: dict) -> bool:
    if not a.get("count") or not b.get("count"):
        return False
    return not (a["end_ns"] < b["start_ns"] or b["end_ns"] < a["start_ns"])


def is_monotonic_pairs(pairs) -> bool:
    return all(a[0] <= b[0] and a[1] <= b[1] for a, b in zip(pairs, pairs[1:]))


def interpolate_clock_time(clock_pairs, storage_times, storage_ns: int) -> int:
    idx = bisect.bisect_left(storage_times, storage_ns)
    if idx == 0:
        return int(clock_pairs[0][1])
    if idx >= len(clock_pairs):
        return int(clock_pairs[-1][1])
    left_storage, left_sim = clock_pairs[idx - 1]
    right_storage, right_sim = clock_pairs[idx]
    if right_storage == left_storage:
        return int(left_sim)
    ratio = (storage_ns - left_storage) / float(right_storage - left_storage)
    return int(round(left_sim + ratio * (right_sim - left_sim)))


def hold_last_by_time(items, stamp_ns: int):
    if not items:
        return None, "missing"
    times = [item[0] for item in items]
    idx = bisect.bisect_right(times, stamp_ns) - 1
    if idx < 0:
        return None, "before_first"
    if idx >= len(items) - 1 and stamp_ns > times[-1]:
        return items[-1], "after_final"
    return items[idx], "matched"


def build_aligned_cmd_velocities(cmds, cmd_stamped, clock_pairs):
    warnings = []
    clock_pairs = sorted(clock_pairs, key=lambda item: item[0])
    cmd_stamped = sorted(cmd_stamped, key=lambda item: item[0])
    cmd_vel_stamped_available = bool(cmd_stamped)
    clock_monotonic = is_monotonic_pairs(clock_pairs) if clock_pairs else False
    clock_mapping_status = "not_used"
    mapped_cmds = []
    source = None
    alignment_method = "none"
    alignment_status = "unavailable"

    if cmd_stamped:
        mapped_cmds = cmd_stamped
        source = "/cmd_vel_stamped"
        alignment_method = "header_stamp"
        alignment_status = "safe"
    elif cmds and clock_pairs and clock_monotonic:
        storage_times = [item[0] for item in clock_pairs]
        mapped_cmds = [
            (interpolate_clock_time(clock_pairs, storage_times, storage_ns), command)
            for storage_ns, command in cmds
        ]
        mapped_cmds.sort(key=lambda item: item[0])
        source = "/clock mapped /cmd_vel"
        alignment_method = "clock_storage_to_sim_interpolation"
        alignment_status = "safe"
        clock_mapping_status = "safe"
    elif cmds and clock_pairs and not clock_monotonic:
        warnings.append("clock storage_time -> sim_time mapping is not monotonic")
        clock_mapping_status = "unsafe"
    elif cmds:
        warnings.append("cmd_vel is headerless and /clock is unavailable")
        clock_mapping_status = "unavailable"
    else:
        warnings.append("no cmd_vel or cmd_vel_stamped messages available")
        clock_mapping_status = "unavailable"

    if cmd_stamped:
        clock_mapping_status = "not_used"
    return {
        "commands": mapped_cmds,
        "source": source,
        "alignment_method": alignment_method,
        "alignment_status": alignment_status,
        "warnings": warnings,
        "clock_mapping_status": clock_mapping_status,
        "clock_mapping_monotonic": clock_monotonic,
        "cmd_vel_stamped_available": cmd_vel_stamped_available,
    }


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_frame(frame: str) -> str:
    return str(frame or "").strip().lstrip("/")


def compose_tf(a, b):
    """Compose T_a_b-style 2D transforms: a parent->mid, b mid->child."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca = math.cos(ayaw)
    sa = math.sin(ayaw)
    return (
        ax + ca * bx - sa * by,
        ay + sa * bx + ca * by,
        normalize_angle(ayaw + byaw),
    )


def invert_tf(t):
    x, y, yaw = t
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, normalize_angle(-yaw))


def odom_pose_to_tf(pose):
    return float(pose[0]), float(pose[1]), float(pose[2])


class TfIndex:
    def __init__(self):
        self._edges = defaultdict(list)
        self._static_edges = {}

    def add(self, parent: str, child: str, stamp_ns: int, transform, is_static: bool = False):
        parent = normalize_frame(parent)
        child = normalize_frame(child)
        if not parent or not child:
            return
        transform = (float(transform[0]), float(transform[1]), float(transform[2]))
        key = (parent, child)
        if is_static:
            self._static_edges[key] = transform
        else:
            self._edges[key].append((int(stamp_ns), transform))

    def finalize(self):
        for values in self._edges.values():
            values.sort(key=lambda item: item[0])

    @property
    def dynamic_edges(self):
        return set(self._edges.keys())

    @property
    def static_edges(self):
        return set(self._static_edges.keys())

    def edge_transform(self, parent: str, child: str, stamp_ns: int):
        key = (normalize_frame(parent), normalize_frame(child))
        if key in self._static_edges:
            return self._static_edges[key]
        values = self._edges.get(key)
        if not values:
            return None
        times = [item[0] for item in values]
        idx = bisect.bisect_left(times, stamp_ns)
        if idx <= 0:
            return values[0][1]
        if idx >= len(values):
            return values[-1][1]
        before = values[idx - 1]
        after = values[idx]
        if abs(before[0] - stamp_ns) <= abs(after[0] - stamp_ns):
            return before[1]
        return after[1]

    def lookup(self, parent: str, child: str, stamp_ns: int):
        parent = normalize_frame(parent)
        child = normalize_frame(child)
        if not parent or not child:
            return None
        if parent == child:
            return (0.0, 0.0, 0.0)

        frames = set()
        edges = set(self._edges.keys()) | set(self._static_edges.keys())
        for edge_parent, edge_child in edges:
            frames.add(edge_parent)
            frames.add(edge_child)
        if parent not in frames or child not in frames:
            return None

        queue = deque([(parent, (0.0, 0.0, 0.0))])
        visited = {parent}
        while queue:
            current, current_tf = queue.popleft()
            for edge_parent, edge_child in edges:
                if edge_parent == current and edge_child not in visited:
                    edge_tf = self.edge_transform(edge_parent, edge_child, stamp_ns)
                    if edge_tf is None:
                        continue
                    next_tf = compose_tf(current_tf, edge_tf)
                    if edge_child == child:
                        return next_tf
                    visited.add(edge_child)
                    queue.append((edge_child, next_tf))
                elif edge_child == current and edge_parent not in visited:
                    edge_tf = self.edge_transform(edge_parent, edge_child, stamp_ns)
                    if edge_tf is None:
                        continue
                    next_tf = compose_tf(current_tf, invert_tf(edge_tf))
                    if edge_parent == child:
                        return next_tf
                    visited.add(edge_parent)
                    queue.append((edge_parent, next_tf))
        return None


def tf_from_transform_stamped(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return float(t.x), float(t.y), yaw_from_quaternion(q)


def nearest_by_time(items, stamp_ns: int):
    if not items:
        return None
    times = [item[0] for item in items]
    idx = bisect.bisect_left(times, stamp_ns)
    if idx <= 0:
        return items[0]
    if idx >= len(items):
        return items[-1]
    before = items[idx - 1]
    after = items[idx]
    if abs(before[0] - stamp_ns) <= abs(after[0] - stamp_ns):
        return before
    return after


def scan_profile(msg):
    ranges = getattr(msg, "ranges", [])
    return {
        "frame_id": normalize_frame(getattr(getattr(msg, "header", None), "frame_id", "")),
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
        "beam_count": int(len(ranges)),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
    }


def update_scan_summary(summary: dict, msg):
    profile = scan_profile(msg)
    summary["count"] += 1
    summary["frame_ids"][profile["frame_id"]] += 1
    for key in ("angle_min", "angle_max", "angle_increment", "beam_count", "range_min", "range_max"):
        summary[key].append(profile[key])


def compact_scan_summary(summary: dict):
    if summary["count"] == 0:
        return {"count": 0}
    result = {
        "count": int(summary["count"]),
        "frame_ids": dict(summary["frame_ids"]),
    }
    for key in ("angle_min", "angle_max", "angle_increment", "beam_count", "range_min", "range_max"):
        values = summary[key]
        result[f"{key}_min"] = float(min(values)) if key != "beam_count" else int(min(values))
        result[f"{key}_max"] = float(max(values)) if key != "beam_count" else int(max(values))
        unique = sorted(set(values))
        if key == "beam_count":
            result[f"{key}_unique"] = [int(v) for v in unique[:20]]
        else:
            result[f"{key}_unique"] = [float(v) for v in unique[:20]]
        result[f"{key}_unique_truncated"] = len(unique) > 20
    return result


def read_bag(
    bag_path: Path,
    scan_topic: str,
    odom_topic: str,
    cmd_vel_topic: str,
    cmd_vel_stamped_topic: str = "/cmd_vel_stamped",
    inspect_scan_topics=None,
    include_tf: bool = False,
    return_extras: bool = False,
    include_clock_times: bool = False,
    include_cmd_vel_stamped: bool = False,
):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = [scan_topic, odom_topic]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")

    inspect_scan_topics = list(inspect_scan_topics or [])
    for topic in inspect_scan_topics:
        if topic not in topic_types:
            continue

    scan_type = get_message(topic_types[scan_topic])
    odom_type = get_message(topic_types[odom_topic])
    cmd_type = get_message(topic_types[cmd_vel_topic]) if cmd_vel_topic in topic_types else None
    cmd_stamped_type = (
        get_message(topic_types[cmd_vel_stamped_topic])
        if cmd_vel_stamped_topic in topic_types
        else None
    )
    clock_type = get_message(topic_types["/clock"]) if "/clock" in topic_types else None
    scan_topic_types = {
        topic: get_message(topic_types[topic])
        for topic in set([scan_topic] + inspect_scan_topics)
        if topic in topic_types
    }
    tf_type = get_message(topic_types["/tf"]) if include_tf and "/tf" in topic_types else None
    tf_static_type = (
        get_message(topic_types["/tf_static"]) if include_tf and "/tf_static" in topic_types else None
    )

    scans = []
    odoms = []
    cmds = []
    cmd_stamped = []
    clocks = []
    topic_counts = Counter()
    tf_index = TfIndex()
    scan_summaries = defaultdict(
        lambda: {
            "count": 0,
            "frame_ids": Counter(),
            "angle_min": [],
            "angle_max": [],
            "angle_increment": [],
            "beam_count": [],
            "range_min": [],
            "range_max": [],
        }
    )

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        topic_counts[topic] += 1
        if topic == scan_topic:
            msg = deserialize_message(data, scan_type)
            scans.append((msg_time_ns(msg, storage_time), msg))
            update_scan_summary(scan_summaries[topic], msg)
        elif topic in scan_topic_types:
            msg = deserialize_message(data, scan_topic_types[topic])
            update_scan_summary(scan_summaries[topic], msg)
        elif topic == odom_topic:
            msg = deserialize_message(data, odom_type)
            pose = msg.pose.pose
            twist = msg.twist.twist
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
        elif cmd_type is not None and topic == cmd_vel_topic:
            msg = deserialize_message(data, cmd_type)
            cmds.append(
                (
                    int(storage_time),
                    (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)),
                )
            )
        elif cmd_stamped_type is not None and topic == cmd_vel_stamped_topic:
            msg = deserialize_message(data, cmd_stamped_type)
            twist = msg.twist
            cmd_stamped.append(
                (
                    msg_time_ns(msg, storage_time),
                    (float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)),
                )
            )
        elif clock_type is not None and topic == "/clock":
            msg = deserialize_message(data, clock_type)
            clocks.append((int(storage_time), stamp_to_ns(msg.clock)))
        elif tf_type is not None and topic == "/tf":
            msg = deserialize_message(data, tf_type)
            for transform in msg.transforms:
                stamp_ns = msg_time_ns(transform, storage_time)
                tf_index.add(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    stamp_ns,
                    tf_from_transform_stamped(transform),
                    is_static=False,
                )
        elif tf_static_type is not None and topic == "/tf_static":
            msg = deserialize_message(data, tf_static_type)
            for transform in msg.transforms:
                stamp_ns = msg_time_ns(transform, storage_time)
                tf_index.add(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    stamp_ns,
                    tf_from_transform_stamped(transform),
                    is_static=True,
                )

    scans.sort(key=lambda item: item[0])
    odoms.sort(key=lambda item: item[0])
    cmds.sort(key=lambda item: item[0])
    cmd_stamped.sort(key=lambda item: item[0])
    clocks.sort(key=lambda item: item[0])
    tf_index.finalize()
    scan_summaries = {topic: compact_scan_summary(summary) for topic, summary in scan_summaries.items()}
    if return_extras:
        if include_clock_times:
            if include_cmd_vel_stamped:
                return (
                    scans,
                    odoms,
                    cmds,
                    cmd_stamped,
                    topic_types,
                    topic_counts,
                    tf_index,
                    scan_summaries,
                    clocks,
                )
            return scans, odoms, cmds, topic_types, topic_counts, tf_index, scan_summaries, clocks
        if include_cmd_vel_stamped:
            return scans, odoms, cmds, cmd_stamped, topic_types, topic_counts, tf_index, scan_summaries
        return scans, odoms, cmds, topic_types, topic_counts, tf_index, scan_summaries
    if include_cmd_vel_stamped:
        return scans, odoms, cmds, cmd_stamped, topic_types, topic_counts
    return scans, odoms, cmds, topic_types, topic_counts


def load_map_info(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    origin = data["origin"]
    return float(data["resolution"]), float(origin[0]), float(origin[1]), float(origin[2])


def load_occupancy_map(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    image_path = Path(data["image"]).expanduser()
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    occupancy_img = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)

    negate = int(data.get("negate", 0))
    if negate not in (0, 1):
        raise ValueError(f"map negate must be 0 or 1, got {negate}")
    occupied_thresh = float(data["occupied_thresh"])
    free_thresh = float(data["free_thresh"])
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError(
            "map thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1"
        )

    # Person generation deliberately uses the raw trinary PGM convention:
    # only pure white is free; gray, black, and out-of-map cells are not.
    free_mask = occupancy_img == 254
    return occupancy_img, free_mask, image_path, negate, occupied_thresh, free_thresh


def native_scan(msg):
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    if len(ranges) == 0:
        raise RuntimeError("Encountered an empty LaserScan ranges array")

    angles = (
        float(msg.angle_min) + np.arange(len(ranges), dtype=np.float64) * float(msg.angle_increment)
    ).astype(np.float32)
    valid_mask = np.isfinite(ranges)
    range_min = float(msg.range_min)
    range_max = float(msg.range_max)
    if np.isfinite(range_min):
        valid_mask &= ranges >= range_min
    if np.isfinite(range_max):
        valid_mask &= ranges <= range_max

    intensities = np.asarray(msg.intensities, dtype=np.float32)
    if len(intensities) == len(ranges):
        intensities = intensities.astype(np.float32, copy=False)
    else:
        intensities = np.zeros(len(ranges), dtype=np.float32)

    return ranges.astype(np.float32, copy=False), intensities, angles, valid_mask.astype(np.bool_)


def scan_endpoints_map(ranges, angles, valid_mask, pose):
    x, y, yaw = pose
    project_mask = valid_mask & np.isfinite(ranges) & np.isfinite(angles)
    if not np.any(project_mask):
        return np.asarray([], dtype=np.int64), np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64)

    project_indices = np.flatnonzero(project_mask)
    project_ranges = ranges[project_indices]
    project_angles = angles[project_indices]
    world_x = x + project_ranges * np.cos(yaw + project_angles)
    world_y = y + project_ranges * np.sin(yaw + project_angles)
    return project_indices, world_x, world_y


def static_labels_for_pixels(label_img, rows, cols, filter_radius, num_classes):
    if filter_radius < 0:
        raise ValueError("static label filter radius must be non-negative")
    if filter_radius == 0:
        return label_img[rows, cols].astype(np.int64)

    height, width = label_img.shape[:2]
    static_labels = np.zeros(len(rows), dtype=np.int64)
    for index, (row, col) in enumerate(zip(rows, cols)):
        patch = label_img[
            max(0, int(row) - filter_radius) : min(height, int(row) + filter_radius),
            max(0, int(col) - filter_radius) : min(width, int(col) + filter_radius),
        ]
        nonzero = patch[patch != 0].astype(np.int64)
        if nonzero.size:
            static_labels[index] = int(np.bincount(nonzero, minlength=num_classes).argmax())
    return static_labels


def semantic_for_scan(
    ranges,
    angles,
    valid_mask,
    pose,
    label_img,
    occupancy_free,
    resolution,
    origin_x,
    origin_y,
    label_names,
    static_label_filter_radius=2,
    person_label_mode="dynamic",
):
    if person_label_mode not in PERSON_LABEL_MODES:
        raise ValueError(
            f"unsupported person label mode {person_label_mode!r}; "
            f"expected one of {PERSON_LABEL_MODES}"
        )
    height, width = label_img.shape[:2]
    labels = np.full(len(ranges), IGNORE_LABEL, dtype=np.int64)
    project_indices, world_x, world_y = scan_endpoints_map(ranges, angles, valid_mask, pose)
    if len(project_indices) == 0:
        return labels

    cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
    in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    mapped_indices = project_indices[in_map]
    static_labels = static_labels_for_pixels(
        label_img,
        rows[in_map],
        cols[in_map],
        static_label_filter_radius,
        len(label_names),
    )
    labels[mapped_indices] = np.where(static_labels != 0, static_labels, IGNORE_LABEL)
    person_label_id = find_label_id(label_names, "Person")
    if person_label_mode == "dynamic" and person_label_id is not None:
        free_space = occupancy_free[rows[in_map], cols[in_map]]
        labels[mapped_indices] = np.where(
            static_labels != 0,
            static_labels,
            np.where(free_space, person_label_id, IGNORE_LABEL),
        )
    return labels


def select_projection_pose(
    requested_source: str,
    scan_frame: str,
    stamp_ns: int,
    odom_pose,
    tf_index: TfIndex,
    map_frame: str,
    base_frame: str,
    strict_tf: bool,
):
    scan_frame = normalize_frame(scan_frame)
    map_frame = normalize_frame(map_frame)
    base_frame = normalize_frame(base_frame)
    warnings = []

    def odom_fallback():
        if strict_tf:
            raise RuntimeError("--strict-tf is enabled and no map-frame TF pose is available")
        if scan_frame != base_frame:
            base_to_scan = tf_index.lookup(base_frame, scan_frame, stamp_ns)
            if base_to_scan is None:
                raise RuntimeError(
                    f"Cannot fallback to odom: scan frame '{scan_frame}' is not '{base_frame}' "
                    f"and TF {base_frame}->{scan_frame} is unavailable"
                )
            pose = compose_tf(odom_pose_to_tf(odom_pose), base_to_scan)
        else:
            pose = odom_pose_to_tf(odom_pose)
        warnings.append("Falling back to odom pose; this assumes odom and map are aligned.")
        return pose, "odom", False, False, True, warnings

    if requested_source in ("auto", "tf-map-scan"):
        map_to_scan = tf_index.lookup(map_frame, scan_frame, stamp_ns)
        if map_to_scan is not None:
            return map_to_scan, "tf-map-scan", True, False, False, warnings
        if requested_source == "tf-map-scan":
            raise RuntimeError(f"Could not resolve TF {map_frame}->{scan_frame}")

    if requested_source in ("auto", "tf-map-base"):
        map_to_base = tf_index.lookup(map_frame, base_frame, stamp_ns)
        if map_to_base is not None and scan_frame == base_frame:
            return map_to_base, "tf-map-base", False, True, False, warnings
        if requested_source == "tf-map-base":
            if scan_frame != base_frame:
                raise RuntimeError(
                    f"--pose-source tf-map-base is unsafe because scan frame '{scan_frame}' "
                    f"is not base frame '{base_frame}'"
                )
            raise RuntimeError(f"Could not resolve TF {map_frame}->{base_frame}")

    if requested_source in ("auto", "odom"):
        return odom_fallback()

    raise RuntimeError(f"Unsupported pose source: {requested_source}")


def colorize_label_image(label_img: np.ndarray):
    palette = np.asarray(
        [
            [30, 30, 30],
            [231, 76, 60],
            [52, 152, 219],
            [46, 204, 113],
            [241, 196, 15],
            [155, 89, 182],
            [230, 126, 34],
            [26, 188, 156],
            [149, 165, 166],
            [236, 240, 241],
            [192, 57, 43],
            [41, 128, 185],
        ],
        dtype=np.uint8,
    )
    labels = np.asarray(label_img, dtype=np.int64)
    out = np.zeros((*labels.shape, 3), dtype=np.uint8)
    valid = labels >= 0
    out[valid] = palette[labels[valid] % len(palette)]
    return out


def write_projection_debug(
    debug_dir: Path,
    sample_indices,
    samples,
    label_img,
    resolution,
    origin_x,
    origin_y,
    label_names,
):
    debug_dir.mkdir(parents=True, exist_ok=True)
    height, width = label_img.shape[:2]
    base = colorize_label_image(label_img)
    written = []
    for sample_index in sample_indices:
        scan, angles, valid_mask, semantic, pose = samples[sample_index]
        image = Image.fromarray(base.copy(), mode="RGB")
        px = image.load()
        project_indices, world_x, world_y = scan_endpoints_map(scan, angles, valid_mask, pose)
        if len(project_indices) > 0:
            cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
            rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
            in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            for idx, col, row in zip(project_indices[in_map], cols[in_map], rows[in_map]):
                if int(semantic[idx]) == IGNORE_LABEL:
                    color = (255, 0, 0)
                elif label_name(label_names, int(semantic[idx])).casefold() == "person":
                    color = (255, 0, 255)
                else:
                    color = (0, 255, 255)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        xx = int(col) + dx
                        yy = int(row) + dy
                        if 0 <= xx < width and 0 <= yy < height:
                            px[xx, yy] = color
        robot_col = int(math.floor((pose[0] - origin_x) / resolution))
        robot_row = int(height - 1 - math.floor((pose[1] - origin_y) / resolution))
        if 0 <= robot_col < width and 0 <= robot_row < height:
            for dx in range(-4, 5):
                for dy in range(-4, 5):
                    if abs(dx) == abs(dy) or dx == 0 or dy == 0:
                        xx = robot_col + dx
                        yy = robot_row + dy
                        if 0 <= xx < width and 0 <= yy < height:
                            px[xx, yy] = (255, 255, 0)
        out_path = debug_dir / f"projection_{sample_index:07d}.png"
        image.save(out_path)
        written.append(str(out_path))
    return written


def write_scan_fusion_report(report_path: Path, scan_summaries: dict, scan_topic: str):
    lines = ["# Native LiDAR Scan Fusion Check", ""]
    for topic in ("/scan_01", "/scan_02", scan_topic):
        info = scan_summaries.get(topic, {"count": 0})
        lines.append(f"## {topic}")
        lines.append(f"- count: {info.get('count', 0)}")
        if info.get("count", 0):
            lines.append(f"- frame_ids: `{info.get('frame_ids', {})}`")
            lines.append(f"- angle_min: {info.get('angle_min_min')} .. {info.get('angle_min_max')}")
            lines.append(f"- angle_max: {info.get('angle_max_min')} .. {info.get('angle_max_max')}")
            lines.append(
                f"- angle_increment: {info.get('angle_increment_min')} .. {info.get('angle_increment_max')}"
            )
            lines.append(f"- beam_count_unique: {info.get('beam_count_unique')}")
            lines.append(f"- range_min: {info.get('range_min_min')} .. {info.get('range_min_max')}")
            lines.append(f"- range_max: {info.get('range_max_min')} .. {info.get('range_max_max')}")
        lines.append("")

    merged = scan_summaries.get(scan_topic, {})
    warnings = []
    if not merged.get("count"):
        warnings.append(f"{scan_topic} is missing.")
    if len(merged.get("frame_ids", {})) > 1:
        warnings.append(f"{scan_topic} has multiple frame_id values.")
    if len(merged.get("beam_count_unique", [])) > 1:
        warnings.append(f"{scan_topic} beam count changes across the bag.")
    if warnings:
        lines.append("## Assessment")
        lines.extend(f"- WARNING: {warning}" for warning in warnings)
    else:
        lines.append("## Assessment")
        lines.append("- `scan_merged` has a stable frame and geometry profile in this bag.")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def local_subgoals(poses: np.ndarray, lookahead: int) -> np.ndarray:
    goals = np.zeros((len(poses), 2), dtype=np.float32)
    for i, pose in enumerate(poses):
        j = min(i + lookahead, len(poses) - 1)
        dx = float(poses[j, 0] - pose[0])
        dy = float(poses[j, 1] - pose[1])
        cy = math.cos(-float(pose[2]))
        sy = math.sin(-float(pose[2]))
        goals[i, 0] = cy * dx - sy * dy
        goals[i, 1] = sy * dx + cy * dy
    return goals


def split_filenames(filenames, train_ratio: float, dev_ratio: float, split_seed: int):
    indices = np.arange(len(filenames))
    rng = np.random.default_rng(split_seed)
    rng.shuffle(indices)

    train_count = int(round(len(filenames) * train_ratio))
    dev_count = int(round(len(filenames) * dev_ratio))
    train_count = min(train_count, len(filenames))
    dev_count = min(dev_count, len(filenames) - train_count)

    train_indices = indices[:train_count]
    dev_indices = indices[train_count : train_count + dev_count]
    test_indices = indices[train_count + dev_count :]

    train = [filenames[i] for i in train_indices]
    dev = [filenames[i] for i in dev_indices]
    test = [filenames[i] for i in test_indices]
    return train, dev, test


def update_dataset_index(output_root: Path, session_name: str, rewrite: bool):
    index_path = output_root / "dataset.txt"
    normalized_session = session_name.strip().rstrip("/")
    sessions = []

    def valid_cmd_session(name: str) -> bool:
        session_dir = output_root / name
        if not session_dir.is_dir():
            return False
        if not all((session_dir / split).is_file() for split in ("train.txt", "dev.txt", "test.txt")):
            return False
        scans_dir = session_dir / "scans_lidar"
        cmds_dir = session_dir / "cmd_velocities"
        if not scans_dir.is_dir() or not cmds_dir.is_dir():
            return False
        return len(list(scans_dir.glob("*.npy"))) == len(list(cmds_dir.glob("*.npy")))

    if index_path.exists() and not rewrite:
        for line in index_path.read_text(encoding="utf-8").splitlines():
            name = line.strip().rstrip("/")
            if name and valid_cmd_session(name) and name not in sessions:
                sessions.append(name)

    sessions = [name for name in sessions if name != normalized_session]
    if valid_cmd_session(normalized_session):
        sessions.append(normalized_session)
    index_path.write_text("\n".join(sessions) + "\n", encoding="utf-8")
    return sessions


def write_dataset_label_names(output_root: Path, label_names):
    output_root.mkdir(parents=True, exist_ok=True)
    path = output_root / "label_names.txt"
    content = "\n".join(label_names) + "\n"
    if path.exists() and path.read_text(encoding="utf-8") != content:
        raise ValueError(
            f"{path} has a different class mapping; do not mix sessions with different label IDs"
        )
    path.write_text(content, encoding="utf-8")


def write_report(report_path: Path, metadata: dict):
    hist_lines = "\n".join(
        f"- {label} {label_name(metadata['label_names'], int(label))}: {count}"
        for label, count in sorted(metadata["label_histogram"].items(), key=lambda kv: int(kv[0]))
    )
    cmd_warnings = metadata.get("cmd_vel_warnings", [])
    velocity_lines = [
        "## Velocity",
        "- velocities/ = odom_twist，表示实际速度状态",
        "- cmd_velocities/ = 对齐后的控制命令，表示模型未来要输出的动作标签",
        f"- velocity source used: `{metadata['velocity_source_used']}`",
        f"- cmd_velocities generated: {metadata['cmd_velocities_generated']}",
        f"- cmd_velocity_dim: {metadata['cmd_velocity_dim']}",
        f"- cmd_velocities source: `{metadata['cmd_velocities_source']}`",
        f"- cmd_vel alignment method: `{metadata['cmd_vel_alignment_method']}`",
        f"- cmd_vel match policy: `{metadata['cmd_vel_match_policy']}`",
        f"- cmd_vel alignment status: `{metadata['cmd_vel_alignment_status']}`",
        f"- cmd_vel stamped available: {metadata['cmd_vel_stamped_available']}",
        f"- cmd_vel stamped count: {metadata['cmd_vel_stamped_count']}",
        f"- cmd_vel clock mapping status: `{metadata['cmd_vel_clock_mapping_status']}`",
        f"- cmd_vel clock mapping monotonic: {metadata['cmd_vel_clock_mapping_monotonic']}",
        f"- cmd_vel time basis mismatch: {metadata['cmd_vel_time_basis_mismatch']}",
        f"- cmd_vel alignment warnings: {cmd_warnings}",
        f"- scan time range sec: {metadata['scan_time_range']['start_sec']}..{metadata['scan_time_range']['end_sec']}",
        f"- odom time range sec: {metadata['odom_time_range']['start_sec']}..{metadata['odom_time_range']['end_sec']}",
        f"- cmd_vel time range sec: {metadata['cmd_vel_time_range']['start_sec']}..{metadata['cmd_vel_time_range']['end_sec']}",
        f"- cmd_vel mapped time range sec: {metadata['cmd_vel_mapped_time_range']['start_sec']}..{metadata['cmd_vel_mapped_time_range']['end_sec']}",
        f"- clock time range sec: {metadata['clock_time_range']['start_sec']}..{metadata['clock_time_range']['end_sec']}",
        f"- raw cmd_vel angular.z: {metadata['raw_cmd_vel_stats']['angular_z']}",
        f"- cmd_velocities angular.z: {metadata['cmd_velocities_angular_z_stats']}",
        f"- cmd_velocities nonzero count: {metadata['cmd_velocities_nonzero_count']}",
        f"- cmd_velocities no prior count: {metadata['cmd_velocities_no_prior_count']}",
        f"- cmd_velocities hold-last after final count: {metadata['cmd_velocities_hold_last_after_final_count']}",
        f"- raw odom twist angular.z: {metadata['raw_odom_twist_stats']['angular_z']}",
        f"- converted velocity angular_z: {metadata['converted_velocity_stats']['angular_z']}",
    ]
    report = f"""# ROS 2 Bag to Native-LiDAR Semantic2D Conversion

## Inputs
- bag: `{metadata['bag']}`
- map yaml: `{metadata['map_yaml']}`
- semantic label: `{metadata['semantic_label']}`
- scan topic: `{metadata['scan_topic']}`
- odom topic: `{metadata['odom_topic']}`
- cmd_vel topic: `{metadata['cmd_vel_topic']}`

## TF / Projection
- pose source used: `{metadata['pose_source_used']}`
- scan frame: `{metadata['scan_frame_id']}`
- base frame: `{metadata['base_frame']}`
- map frame: `{metadata['map_frame']}`
- tf alignment status: `{metadata['tf_alignment_status']}`
- used map->scan TF: {metadata['used_map_to_scan_tf']}
- used map->base TF: {metadata['used_map_to_base_tf']}
- fallback to odom: {metadata['fallback_to_odom']}
- projection debug dir: `{metadata['projection_debug_dir']}`
- warnings: {metadata['tf_alignment_warnings']}

## Output
- dataset root: `{metadata['output_root']}`
- session: `{metadata['session_name']}`
- samples: {metadata['samples']}
- native lidar: {metadata['native_lidar']}
- interpolated to baseline 1081: {metadata['interpolated_to_baseline_1081']}
- beam count unique: {metadata['beam_count_unique']}
- ignore label: {metadata['ignore_label']}
- train samples: {metadata['train_samples']}
- dev samples: {metadata['dev_samples']}
- test samples: {metadata['test_samples']}
- split ratios: {metadata['train_ratio']}/{metadata['dev_ratio']}/{metadata['test_ratio']}
- split seed: {metadata['split_seed']}

## Person Labeling
- person label mode: `{metadata['person_label_mode']}`
- rule: `{metadata['person_label_rule']}`
- static label filter radius: {metadata['map']['static_label_filter_radius']}
- occupancy image: `{metadata['map']['occupancy_image']}`
- occupancy free threshold: {metadata['map']['free_thresh']}
- occupancy occupied threshold: {metadata['map']['occupied_thresh']}

{chr(10).join(velocity_lines)}

## Label Histogram
- label names source: `{metadata['label_names_source']}`
{hist_lines}
"""
    report_path.write_text(report, encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--session-name", required=True)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--cmd-vel-stamped-topic", default="/cmd_vel_stamped")
    parser.add_argument("--train-ratio", default=0.7, type=float)
    parser.add_argument("--dev-ratio", default=0.1, type=float)
    parser.add_argument("--test-ratio", default=0.2, type=float)
    parser.add_argument("--split-seed", default=0, type=int)
    parser.add_argument("--subgoal-lookahead", default=20, type=int)
    parser.add_argument("--pose-source", choices=POSE_SOURCES, default="auto")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument(
        "--static-label-filter-radius",
        default=2,
        type=int,
        help="Static-label neighborhood radius; 2 reproduces the original Semantic2D 4x4 vote, 0 uses exact pixels.",
    )
    parser.add_argument(
        "--person-label-mode",
        choices=PERSON_LABEL_MODES,
        default="dynamic",
        help="dynamic labels free unlabeled endpoints as the configured Person class; disabled always ignores them",
    )
    parser.add_argument("--strict-tf", action="store_true")
    parser.add_argument("--write-projection-debug", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--rewrite-dataset-index",
        action="store_true",
        help="Rewrite output-root/dataset.txt to contain only this session",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    ratios = [args.train_ratio, args.dev_ratio, args.test_ratio]
    if any(ratio < 0.0 for ratio in ratios):
        raise ValueError("split ratios must be non-negative")
    if not math.isclose(sum(ratios), 1.0, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError("--train-ratio + --dev-ratio + --test-ratio must equal 1.0")
    if "-" not in args.session_name:
        raise ValueError("session name must contain '-' because the baseline loaders filter dataset.txt lines that way")

    session_dir = args.output_root / args.session_name
    if session_dir.exists():
        if not args.overwrite:
            raise FileExistsError(f"{session_dir} exists; pass --overwrite to replace it")
        shutil.rmtree(session_dir)
    session_dir.mkdir(parents=True)

    subdirs = [
        "scans_lidar",
        "intensities_lidar",
        "angles_lidar",
        "valid_mask_lidar",
        "semantic_label",
        "positions",
        "velocities",
        "sub_goals_local",
    ]
    for subdir in subdirs:
        (session_dir / subdir).mkdir()

    (
        scans,
        odoms,
        cmds,
        cmd_stamped,
        topic_types,
        topic_counts,
        tf_index,
        scan_summaries,
        clocks,
    ) = read_bag(
        args.bag,
        args.scan_topic,
        args.odom_topic,
        args.cmd_vel_topic,
        args.cmd_vel_stamped_topic,
        inspect_scan_topics=["/scan_01", "/scan_02", args.scan_topic],
        include_tf=True,
        return_extras=True,
        include_clock_times=True,
        include_cmd_vel_stamped=True,
    )
    if not scans:
        raise RuntimeError(f"No scans found on {args.scan_topic}")
    if not odoms:
        raise RuntimeError(f"No odometry found on {args.odom_topic}")

    scan_time_range = time_range_ns([item[0] for item in scans])
    odom_time_range = time_range_ns([item[0] for item in odoms])
    cmd_vel_time_range = time_range_ns([item[0] for item in cmds])
    cmd_vel_stamped_time_range = time_range_ns([item[0] for item in cmd_stamped])
    clock_storage_time_range = time_range_ns([item[0] for item in clocks])
    clock_time_range = time_range_ns([item[1] for item in clocks])
    cmd_vel_time_basis_mismatch = bool(cmds) and not ranges_overlap(scan_time_range, cmd_vel_time_range)
    cmd_vel_warnings = []
    if cmd_vel_time_basis_mismatch:
        cmd_vel_warnings.append(
            "cmd_vel is headerless and storage_time is not aligned with scan sim time"
        )
    velocity_source_used = "odom_twist"
    cmd_alignment = build_aligned_cmd_velocities(cmds, cmd_stamped, clocks)
    cmd_vel_warnings.extend(cmd_alignment["warnings"])
    aligned_cmds = cmd_alignment["commands"]
    cmd_vel_alignment_status = cmd_alignment["alignment_status"]
    cmd_velocities_generated = bool(aligned_cmds)
    cmd_velocities_no_prior_count = 0
    cmd_velocities_hold_last_after_final_count = 0
    if cmd_velocities_generated:
        (session_dir / "cmd_velocities").mkdir()

    resolution, origin_x, origin_y, origin_yaw = load_map_info(args.map_yaml)
    label_img = np.asarray(Image.open(args.semantic_label))
    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.int64)
    (
        occupancy_img,
        occupancy_free,
        occupancy_image_path,
        map_negate,
        occupied_thresh,
        free_thresh,
    ) = load_occupancy_map(args.map_yaml)
    if occupancy_img.shape != label_img.shape:
        raise ValueError(
            f"occupancy map shape {occupancy_img.shape} does not match "
            f"semantic label shape {label_img.shape}"
        )
    label_names, label_names_source = load_label_names(args.semantic_label)
    validate_label_image(label_img, label_names)
    write_dataset_label_names(args.output_root, label_names)
    dynamic_person_label_id = find_label_id(label_names, "Person")
    if args.person_label_mode == "dynamic" and dynamic_person_label_id is None:
        print("WARNING: Person is not in label_names.txt; free unlabeled endpoints will be ignored")

    all_scans = []
    all_intensities = []
    all_angles = []
    all_valid_masks = []
    all_labels = []
    all_positions = []
    all_velocities = []
    all_cmd_velocities = []
    debug_samples = []
    beam_counts = []
    valid_counts = []
    angle_min_values = []
    angle_max_values = []
    angle_increment_values = []
    pose_source_counts = Counter()
    used_map_to_scan_tf = False
    used_map_to_base_tf = False
    fallback_to_odom = False
    tf_alignment_warnings = []
    scan_frame_ids = Counter()

    for stamp_ns, scan_msg in scans:
        odom_match = nearest_by_time(odoms, stamp_ns)
        if odom_match is None:
            continue
        x, y, yaw, odom_lin, odom_ang = odom_match[1]
        lin_x, ang_z = odom_lin, odom_ang

        scan, intensity, angles, valid_mask = native_scan(scan_msg)
        scan_frame = normalize_frame(scan_msg.header.frame_id)
        scan_frame_ids[scan_frame] += 1
        pose, pose_source, used_scan_tf, used_base_tf, used_odom, pose_warnings = select_projection_pose(
            args.pose_source,
            scan_frame,
            stamp_ns,
            (x, y, yaw),
            tf_index,
            args.map_frame,
            args.base_frame,
            args.strict_tf,
        )
        pose_source_counts[pose_source] += 1
        used_map_to_scan_tf = used_map_to_scan_tf or used_scan_tf
        used_map_to_base_tf = used_map_to_base_tf or used_base_tf
        fallback_to_odom = fallback_to_odom or used_odom
        tf_alignment_warnings.extend(pose_warnings)
        beam_counts.append(int(len(scan)))
        valid_counts.append(int(valid_mask.sum()))
        angle_min_values.append(float(angles[0]))
        angle_max_values.append(float(angles[-1]))
        angle_increment_values.append(float(scan_msg.angle_increment))
        semantic = semantic_for_scan(
            scan,
            angles,
            valid_mask,
            pose,
            label_img,
            occupancy_free,
            resolution,
            origin_x,
            origin_y,
            label_names,
            args.static_label_filter_radius,
            args.person_label_mode,
        )

        all_scans.append(scan)
        all_intensities.append(intensity)
        all_angles.append(angles)
        all_valid_masks.append(valid_mask)
        all_labels.append(semantic)
        all_positions.append(np.asarray(pose, dtype=np.float32))
        all_velocities.append(np.asarray([lin_x, ang_z], dtype=np.float32))
        if cmd_velocities_generated:
            cmd_match, cmd_match_status = hold_last_by_time(aligned_cmds, stamp_ns)
            if cmd_match_status == "before_first":
                cmd_velocities_no_prior_count += 1
                cmd_velocity = (0.0, 0.0, 0.0)
            elif cmd_match_status == "after_final":
                cmd_velocities_hold_last_after_final_count += 1
                cmd_velocity = cmd_match[1]
            elif cmd_match is not None:
                cmd_velocity = cmd_match[1]
            else:
                cmd_velocity = (0.0, 0.0, 0.0)
            all_cmd_velocities.append(np.asarray(cmd_velocity, dtype=np.float32))
        debug_samples.append((scan, angles, valid_mask, semantic, pose))

    if not all_scans:
        raise RuntimeError("No samples were converted")

    positions = np.stack(all_positions).astype(np.float32)
    subgoals = local_subgoals(positions, args.subgoal_lookahead)
    label_hist = Counter()
    filenames = []

    for i, (scan, intensity, angles, valid_mask, semantic, position, velocity, cmd_velocity, subgoal) in enumerate(
        zip(
            all_scans,
            all_intensities,
            all_angles,
            all_valid_masks,
            all_labels,
            all_positions,
            all_velocities,
            all_cmd_velocities if cmd_velocities_generated else [None] * len(all_velocities),
            subgoals,
        )
    ):
        name = f"{i:07d}.npy"
        filenames.append(name)
        np.save(session_dir / "scans_lidar" / name, scan.astype(np.float32))
        np.save(session_dir / "intensities_lidar" / name, intensity.astype(np.float32))
        np.save(session_dir / "angles_lidar" / name, angles.astype(np.float32))
        np.save(session_dir / "valid_mask_lidar" / name, valid_mask.astype(np.bool_))
        np.save(session_dir / "semantic_label" / name, semantic.astype(np.int64))
        np.save(session_dir / "positions" / name, position.astype(np.float32))
        np.save(session_dir / "velocities" / name, velocity.astype(np.float32))
        if cmd_velocities_generated:
            np.save(session_dir / "cmd_velocities" / name, cmd_velocity.astype(np.float32))
        np.save(session_dir / "sub_goals_local" / name, subgoal.astype(np.float32))
        label_hist.update(int(v) for v in semantic.tolist())

    train, dev, test = split_filenames(
        filenames,
        args.train_ratio,
        args.dev_ratio,
        args.split_seed,
    )

    (session_dir / "train.txt").write_text("\n".join(train) + "\n", encoding="utf-8")
    (session_dir / "dev.txt").write_text("\n".join(dev) + "\n", encoding="utf-8")
    (session_dir / "test.txt").write_text("\n".join(test) + "\n", encoding="utf-8")
    dataset_index_sessions = update_dataset_index(
        args.output_root,
        args.session_name,
        args.rewrite_dataset_index,
    )

    projection_debug_dir = ""
    projection_debug_files = []
    if args.write_projection_debug:
        indices = sorted(set([0, len(debug_samples) // 2, len(debug_samples) - 1]))
        debug_dir = session_dir / "projection_debug"
        projection_debug_files = write_projection_debug(
            debug_dir,
            indices,
            debug_samples,
            label_img,
            resolution,
            origin_x,
            origin_y,
            label_names,
        )
        projection_debug_dir = str(debug_dir)

    tf_alignment_warnings = sorted(set(tf_alignment_warnings))
    if fallback_to_odom:
        tf_alignment_status = "warning"
    elif used_map_to_scan_tf or used_map_to_base_tf:
        tf_alignment_status = "safe"
    else:
        tf_alignment_status = "unsafe"

    pose_source_used = (
        pose_source_counts.most_common(1)[0][0]
        if len(pose_source_counts) == 1
        else dict(pose_source_counts)
    )
    scan_frame_id = (
        scan_frame_ids.most_common(1)[0][0] if len(scan_frame_ids) == 1 else dict(scan_frame_ids)
    )
    converted_velocity_stats = summarize_vectors(
        [tuple(float(v) for v in velocity.tolist()) for velocity in all_velocities],
        [("linear_x", 0), ("angular_z", 1)],
    )
    cmd_velocities_stats = summarize_vectors(
        [tuple(float(v) for v in velocity.tolist()) for velocity in all_cmd_velocities],
        [("linear_x", 0), ("linear_y", 1), ("angular_z", 2)],
    )
    cmd_velocities_nonzero_count = int(
        sum(bool(np.any(np.abs(velocity) > 1e-6)) for velocity in all_cmd_velocities)
    )
    cmd_velocities_angular_z_stats = cmd_velocities_stats["angular_z"]

    metadata = {
        "bag": str(args.bag),
        "output_root": str(args.output_root),
        "session_name": args.session_name,
        "person_label_mode": args.person_label_mode,
        "person_label_rule": (
            "static 4x4 neighborhood empty and PGM endpoint == 254 -> Person class; "
            "otherwise unlabeled endpoint -> ignore=-1"
            if args.person_label_mode == "dynamic" and dynamic_person_label_id is not None
            else "static neighborhood majority; every unlabeled, non-free, unknown, or out-of-map "
            "endpoint -> ignore=-1; dynamic Person labeling is disabled"
        ),
        "map_yaml": str(args.map_yaml),
        "semantic_label": str(args.semantic_label),
        "scan_topic": args.scan_topic,
        "odom_topic": args.odom_topic,
        "cmd_vel_topic": args.cmd_vel_topic,
        "cmd_vel_stamped_topic": args.cmd_vel_stamped_topic,
        "pose_source_requested": args.pose_source,
        "pose_source_used": pose_source_used,
        "scan_frame_id": scan_frame_id,
        "base_frame": normalize_frame(args.base_frame),
        "map_frame": normalize_frame(args.map_frame),
        "tf_alignment_status": tf_alignment_status,
        "tf_alignment_warnings": tf_alignment_warnings,
        "used_map_to_scan_tf": used_map_to_scan_tf,
        "used_map_to_base_tf": used_map_to_base_tf,
        "fallback_to_odom": fallback_to_odom,
        "projection_debug_dir": projection_debug_dir,
        "projection_debug_files": projection_debug_files,
        "velocity_source_used": velocity_source_used,
        "cmd_velocities_generated": cmd_velocities_generated,
        "cmd_velocity_dim": CMD_VELOCITY_DIM,
        "cmd_velocities_source": cmd_alignment["source"],
        "cmd_vel_alignment_method": cmd_alignment["alignment_method"],
        "cmd_vel_match_policy": "hold-last",
        "cmd_vel_time_source": "header_stamp" if cmd_stamped else "storage_time",
        "cmd_vel_alignment_status": cmd_vel_alignment_status,
        "cmd_vel_alignment_warnings": cmd_vel_warnings,
        "cmd_vel_warnings": cmd_vel_warnings,
        "cmd_vel_clock_mapping_status": cmd_alignment["clock_mapping_status"],
        "cmd_vel_clock_mapping_monotonic": cmd_alignment["clock_mapping_monotonic"],
        "cmd_vel_mapped_time_range": time_range_ns([item[0] for item in aligned_cmds]),
        "cmd_velocities_stats": cmd_velocities_stats,
        "cmd_velocities_no_prior_count": cmd_velocities_no_prior_count,
        "cmd_velocities_hold_last_after_final_count": cmd_velocities_hold_last_after_final_count,
        "cmd_velocities_nonzero_count": cmd_velocities_nonzero_count,
        "cmd_velocities_angular_z_stats": cmd_velocities_angular_z_stats,
        "cmd_vel_stamped_available": cmd_alignment["cmd_vel_stamped_available"],
        "cmd_vel_stamped_count": len(cmd_stamped),
        "cmd_vel_time_basis_mismatch": cmd_vel_time_basis_mismatch,
        "raw_cmd_vel_stats": summarize_vectors(
            [item[1] for item in cmds],
            [("linear_x", 0), ("linear_y", 1), ("angular_z", 2)],
        ),
        "raw_cmd_vel_stamped_stats": summarize_vectors(
            [item[1] for item in cmd_stamped],
            [("linear_x", 0), ("linear_y", 1), ("angular_z", 2)],
        ),
        "raw_odom_twist_stats": summarize_vectors(
            [(item[1][3], item[1][4]) for item in odoms],
            [("linear_x", 0), ("angular_z", 1)],
        ),
        "converted_velocity_stats": converted_velocity_stats,
        "scan_time_range": scan_time_range,
        "odom_time_range": odom_time_range,
        "cmd_vel_time_range": cmd_vel_time_range,
        "cmd_vel_stamped_time_range": cmd_vel_stamped_time_range,
        "clock_storage_time_range": clock_storage_time_range,
        "clock_time_range": clock_time_range,
        "native_lidar": True,
        "interpolated_to_baseline_1081": False,
        "ignore_label": IGNORE_LABEL,
        "beam_count_min": int(min(beam_counts)),
        "beam_count_max": int(max(beam_counts)),
        "beam_count_unique": [int(v) for v in sorted(set(beam_counts))],
        "beam_count_histogram": {str(k): int(v) for k, v in sorted(Counter(beam_counts).items())},
        "valid_beam_count_min": int(min(valid_counts)),
        "valid_beam_count_max": int(max(valid_counts)),
        "valid_beam_fraction": float(sum(valid_counts) / sum(beam_counts)),
        "angle_min_min": float(min(angle_min_values)),
        "angle_min_max": float(max(angle_min_values)),
        "angle_max_min": float(min(angle_max_values)),
        "angle_max_max": float(max(angle_max_values)),
        "angle_increment_min": float(min(angle_increment_values)),
        "angle_increment_max": float(max(angle_increment_values)),
        "train_ratio": args.train_ratio,
        "dev_ratio": args.dev_ratio,
        "test_ratio": args.test_ratio,
        "split_seed": args.split_seed,
        "subgoal_lookahead": args.subgoal_lookahead,
        "samples": len(filenames),
        "train_samples": len(train),
        "dev_samples": len(dev),
        "test_samples": len(test),
        "dataset_index_sessions": dataset_index_sessions,
        "map": {
            "resolution": resolution,
            "origin": [origin_x, origin_y, origin_yaw],
            "label_shape": list(label_img.shape),
            "occupancy_image": str(occupancy_image_path),
            "occupancy_shape": list(occupancy_img.shape),
            "negate": map_negate,
            "occupied_thresh": occupied_thresh,
            "free_thresh": free_thresh,
            "static_label_filter_radius": args.static_label_filter_radius,
            "static_label_filter_window": (
                1 if args.static_label_filter_radius == 0 else 2 * args.static_label_filter_radius
            ),
            "person_label_rule": (
                "no nonzero static label in 4x4 neighborhood and occupancy_pixel == 254 -> Person class"
                if args.person_label_mode == "dynamic" and dynamic_person_label_id is not None
                else "no nonzero static label in filtered neighborhood -> ignore=-1"
            ),
        },
        "label_names": label_names,
        "label_names_source": label_names_source,
        "topic_types": topic_types,
        "topic_counts": dict(topic_counts),
        "tf_dynamic_edges": [list(edge) for edge in sorted(tf_index.dynamic_edges)],
        "tf_static_edges": [list(edge) for edge in sorted(tf_index.static_edges)],
        "scan_fusion": scan_summaries,
        "label_histogram": {str(k): int(v) for k, v in sorted(label_hist.items())},
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    write_report(session_dir / "conversion_report.md", metadata)
    write_scan_fusion_report(session_dir / "scan_fusion_check_report.md", scan_summaries, args.scan_topic)

    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
