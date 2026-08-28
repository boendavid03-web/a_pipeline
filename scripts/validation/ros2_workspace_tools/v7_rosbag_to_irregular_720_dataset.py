#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --base-frame, --dev-ratio, --map-frame, --map-yaml, --odom-topic, --output-root, --overwrite, --pose-source, --scan-01-topic, --scan-02-topic, --semantic-label, --session-name, --split-seed, --strict-tf, --subgoal-lookahead, --sync-tolerance-ms, --test-ratio, --train-ratio
# 代码中检测到的 ROS 2 话题/路径字符串：/odom, /scan_01, /scan_02, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, NPY, NPZ, TXT
# 可能使用的关键环境变量：IGNORE_LABEL, SLOTS_PER_SENSOR, TOTAL_SLOTS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_irregular_720_dataset.py
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
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_irregular_720_dataset.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜v7_rosbag_to_irregular_720_dataset.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Convert paired v7 LiDAR scans into fixed-slot, irregular virtual 720-beam samples.

This is deliberately independent of the /scan_merged 360-beam converter.  Slots
0..359 always represent /scan_01 and slots 360..719 always represent /scan_02.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Keep map projection, TF semantics, pose selection, and split behavior exactly
# aligned with the maintained native-LiDAR converter.
from convert_rosbag2_to_semantic2d_native_lidar import (
    IGNORE_LABEL,
    TfIndex,
    load_label_names,
    load_map_info,
    local_subgoals,
    msg_time_ns,
    nearest_by_time,
    normalize_frame,
    select_projection_pose,
    split_filenames,
    tf_from_transform_stamped,
    yaw_from_quaternion,
)

SLOTS_PER_SENSOR = 360
TOTAL_SLOTS = SLOTS_PER_SENSOR * 2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--session-name", required=True)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--scan-01-topic", default="/scan_01")
    parser.add_argument("--scan-02-topic", default="/scan_02")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--pose-source", choices=("odom", "tf-map-base", "tf-map-scan", "auto"), default="auto")
    parser.add_argument("--strict-tf", action="store_true")
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    parser.add_argument("--train-ratio", type=float, default=0.7)
    parser.add_argument("--dev-ratio", type=float, default=0.1)
    parser.add_argument("--test-ratio", type=float, default=0.2)
    parser.add_argument("--split-seed", type=int, default=0)
    parser.add_argument("--subgoal-lookahead", type=int, default=20)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def read_dual_bag(bag_path, scan_01_topic, scan_02_topic, odom_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = (scan_01_topic, scan_02_topic, odom_topic, "/tf", "/tf_static")
    missing = [topic for topic in required if topic not in types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")
    scan_01_type = get_message(types[scan_01_topic])
    scan_02_type = get_message(types[scan_02_topic])
    odom_type = get_message(types[odom_topic])
    tf_type = get_message(types["/tf"])
    tf_static_type = get_message(types["/tf_static"])
    scans_01, scans_02, odoms = [], [], []
    topic_counts = Counter()
    tf_index = TfIndex()
    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        topic_counts[topic] += 1
        if topic == scan_01_topic:
            msg = deserialize_message(data, scan_01_type)
            scans_01.append((msg_time_ns(msg, storage_time), msg))
        elif topic == scan_02_topic:
            msg = deserialize_message(data, scan_02_type)
            scans_02.append((msg_time_ns(msg, storage_time), msg))
        elif topic == odom_topic:
            msg = deserialize_message(data, odom_type)
            pose, twist = msg.pose.pose, msg.twist.twist
            odoms.append((msg_time_ns(msg, storage_time), (float(pose.position.x), float(pose.position.y), yaw_from_quaternion(pose.orientation), float(twist.linear.x), float(twist.angular.z))))
        elif topic == "/tf":
            msg = deserialize_message(data, tf_type)
            for transform in msg.transforms:
                tf_index.add(transform.header.frame_id, transform.child_frame_id, msg_time_ns(transform, storage_time), tf_from_transform_stamped(transform))
        elif topic == "/tf_static":
            msg = deserialize_message(data, tf_static_type)
            for transform in msg.transforms:
                tf_index.add(transform.header.frame_id, transform.child_frame_id, msg_time_ns(transform, storage_time), tf_from_transform_stamped(transform), is_static=True)
    scans_01.sort(key=lambda item: item[0])
    scans_02.sort(key=lambda item: item[0])
    odoms.sort(key=lambda item: item[0])
    tf_index.finalize()
    return scans_01, scans_02, odoms, tf_index, topic_counts


def sensor_slots(msg, stamp_ns, tf_index, base_frame):
    """Build exactly 360 slots for one sensor, without any cross-beam operation."""
    raw_ranges = np.asarray(msg.ranges, dtype=np.float32)
    if raw_ranges.shape != (SLOTS_PER_SENSOR,):
        raise RuntimeError(f"Expected {SLOTS_PER_SENSOR} beams from {msg.header.frame_id}, got {raw_ranges.size}")
    raw_angles = (float(msg.angle_min) + np.arange(SLOTS_PER_SENSOR, dtype=np.float32) * float(msg.angle_increment)).astype(np.float32)
    range_valid = np.isfinite(raw_ranges) & (raw_ranges >= float(msg.range_min)) & (raw_ranges <= float(msg.range_max))
    x_base = np.full(SLOTS_PER_SENSOR, np.nan, dtype=np.float32)
    y_base = np.full(SLOTS_PER_SENSOR, np.nan, dtype=np.float32)
    virtual_ranges = np.full(SLOTS_PER_SENSOR, np.nan, dtype=np.float32)
    virtual_angles = np.full(SLOTS_PER_SENSOR, np.nan, dtype=np.float32)
    tf_valid = np.zeros(SLOTS_PER_SENSOR, dtype=np.bool_)
    self_mask = np.zeros(SLOTS_PER_SENSOR, dtype=np.bool_)
    transform = tf_index.lookup(normalize_frame(base_frame), normalize_frame(msg.header.frame_id), stamp_ns)
    if transform is not None:
        tx, ty, yaw = transform
        indices = np.flatnonzero(range_valid)
        x_sensor = raw_ranges[indices] * np.cos(raw_angles[indices])
        y_sensor = raw_ranges[indices] * np.sin(raw_angles[indices])
        x_base[indices] = tx + math.cos(yaw) * x_sensor - math.sin(yaw) * y_sensor
        y_base[indices] = ty + math.sin(yaw) * x_sensor + math.cos(yaw) * y_sensor
        tf_valid[indices] = True
        self_mask[indices] = (np.abs(x_base[indices]) <= 0.36) & (np.abs(y_base[indices]) <= 0.32)
        usable = indices[~self_mask[indices]]
        virtual_ranges[usable] = np.hypot(x_base[usable], y_base[usable])
        virtual_angles[usable] = np.arctan2(y_base[usable], x_base[usable])
    valid = range_valid & tf_valid & ~self_mask
    return raw_ranges, raw_angles, x_base, y_base, virtual_ranges, virtual_angles, range_valid, tf_valid, self_mask, valid


def semantic_for_base_points(x_base, y_base, valid_mask, pose, label_img, resolution, origin_x, origin_y):
    labels = np.full(TOTAL_SLOTS, IGNORE_LABEL, dtype=np.int16)
    indices = np.flatnonzero(valid_mask)
    if not len(indices):
        return labels
    x, y, yaw = pose
    world_x = x + math.cos(yaw) * x_base[indices] - math.sin(yaw) * y_base[indices]
    world_y = y + math.sin(yaw) * x_base[indices] + math.cos(yaw) * y_base[indices]
    height, width = label_img.shape[:2]
    cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
    in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    labels[indices[in_map]] = label_img[rows[in_map], cols[in_map]].astype(np.int16)
    return labels


def sorted_analysis(angles, ranges, valid_mask):
    out_indices = np.full(TOTAL_SLOTS, -1, dtype=np.int16)
    out_angles = np.full(TOTAL_SLOTS, np.nan, dtype=np.float32)
    out_ranges = np.full(TOTAL_SLOTS, np.nan, dtype=np.float32)
    out_valid = np.zeros(TOTAL_SLOTS, dtype=np.bool_)
    gaps = np.full(TOTAL_SLOTS, np.nan, dtype=np.float32)
    slots = np.flatnonzero(valid_mask)
    slots = slots[np.argsort(angles[slots])]
    count = len(slots)
    out_indices[:count], out_angles[:count], out_ranges[:count], out_valid[:count] = slots, angles[slots], ranges[slots], True
    if count > 1:
        gaps[:count - 1] = np.diff(out_angles[:count])
        wrap_gap = float((out_angles[0] + 2.0 * math.pi) - out_angles[count - 1])
    else:
        wrap_gap = float("nan")
    return out_indices, out_angles, out_ranges, out_valid, gaps, np.float32(wrap_gap), np.int32(count)


def has_near_angle_distinct_ranges(angles, ranges, valid_mask):
    slots = np.flatnonzero(valid_mask)
    if len(slots) < 2:
        return False
    order = slots[np.argsort(angles[slots])]
    pairs = np.column_stack((order[:-1], order[1:]))
    wrap = np.asarray([[order[-1], order[0]]])
    for left, right in np.vstack((pairs, wrap)):
        gap = abs(float(math.atan2(math.sin(angles[right] - angles[left]), math.cos(angles[right] - angles[left]))))
        if gap <= 0.01 and abs(float(ranges[right] - ranges[left])) > 0.05:
            return True
    return False


def main():
    args = parse_args()
    if not math.isclose(args.train_ratio + args.dev_ratio + args.test_ratio, 1.0, abs_tol=1e-9) or min(args.train_ratio, args.dev_ratio, args.test_ratio) < 0:
        raise ValueError("train/dev/test ratios must be non-negative and sum to 1")
    session_dir = args.output_root / args.session_name
    if session_dir.exists():
        if not args.overwrite:
            raise FileExistsError(f"{session_dir} exists; pass --overwrite to replace it")
        shutil.rmtree(session_dir)
    samples_dir = session_dir / "samples"
    samples_dir.mkdir(parents=True)
    scans_01, scans_02, odoms, tf_index, topic_counts = read_dual_bag(args.bag, args.scan_01_topic, args.scan_02_topic, args.odom_topic)
    if not scans_01 or not scans_02 or not odoms:
        raise RuntimeError("Both scan streams and odometry must contain messages")
    resolution, origin_x, origin_y, _ = load_map_info(args.map_yaml)
    label_img = np.asarray(Image.open(args.semantic_label))
    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.int64)
    label_names, label_names_source = load_label_names(args.semantic_label)
    tolerance_ns = int(args.sync_tolerance_ms * 1_000_000)
    filenames, positions = [], []
    label_hist, pose_source_counts = Counter(), Counter()
    range_invalid_counts, self_counts, valid_counts, valid_01_counts, valid_02_counts = [], [], [], [], []
    sync_skipped = 0
    near_angle_distinct_preserved = False
    for stamp_01, scan_01 in scans_01:
        scan_02_match = nearest_by_time(scans_02, stamp_01)
        if scan_02_match is None or abs(scan_02_match[0] - stamp_01) > tolerance_ns:
            sync_skipped += 1
            continue
        odom_match = nearest_by_time(odoms, stamp_01)
        if odom_match is None:
            continue
        _, (ox, oy, oyaw, linear_x, angular_z) = odom_match
        pose, pose_source, _, _, _, _ = select_projection_pose(args.pose_source, normalize_frame(scan_01.header.frame_id), stamp_01, (ox, oy, oyaw), tf_index, args.map_frame, args.base_frame, args.strict_tf)
        pose_source_counts[pose_source] += 1
        first = sensor_slots(scan_01, stamp_01, tf_index, args.base_frame)
        second = sensor_slots(scan_02_match[1], scan_02_match[0], tf_index, args.base_frame)
        raw_ranges, raw_angles, points_x, points_y, virtual_ranges, virtual_angles, range_valid, tf_valid, self_mask, valid = [np.concatenate((first[i], second[i])) for i in range(10)]
        semantic = semantic_for_base_points(points_x, points_y, valid, pose, label_img, resolution, origin_x, origin_y)
        analysis = sorted_analysis(virtual_angles, virtual_ranges, valid)
        sample_name = f"{len(filenames):07d}.npz"
        np.savez_compressed(samples_dir / sample_name,
            raw_ranges=raw_ranges.astype(np.float32), raw_angles_sensor=raw_angles.astype(np.float32),
            points_x_base=points_x.astype(np.float32), points_y_base=points_y.astype(np.float32),
            virtual_ranges=virtual_ranges.astype(np.float32), virtual_angles=virtual_angles.astype(np.float32),
            range_valid_mask=range_valid.astype(np.bool_), tf_valid_mask=tf_valid.astype(np.bool_), self_mask=self_mask.astype(np.bool_), valid_mask=valid.astype(np.bool_),
            source_sensor=np.repeat(np.asarray([0, 1], dtype=np.uint8), SLOTS_PER_SENSOR), raw_beam_index=np.tile(np.arange(SLOTS_PER_SENSOR, dtype=np.int16), 2), semantic_label=semantic,
            sorted_slot_indices=analysis[0], sorted_virtual_angles=analysis[1], sorted_virtual_ranges=analysis[2], sorted_valid_mask=analysis[3], angle_gaps=analysis[4], wrap_gap=analysis[5], valid_count=analysis[6],
            position=np.asarray(pose, dtype=np.float32), velocity=np.asarray([linear_x, angular_z], dtype=np.float32), scan_01_stamp_ns=np.int64(stamp_01), scan_02_stamp_ns=np.int64(scan_02_match[0]))
        filenames.append(sample_name)
        positions.append(np.asarray(pose, dtype=np.float32))
        label_hist.update(int(v) for v in semantic.tolist())
        range_invalid_counts.append(int((~range_valid).sum()))
        self_counts.append(int(self_mask.sum()))
        valid_counts.append(int(valid.sum()))
        valid_01_counts.append(int(valid[:SLOTS_PER_SENSOR].sum()))
        valid_02_counts.append(int(valid[SLOTS_PER_SENSOR:].sum()))
        near_angle_distinct_preserved |= has_near_angle_distinct_ranges(virtual_angles, virtual_ranges, valid)
    if not filenames:
        raise RuntimeError("No synchronized scan pairs were converted")
    positions = np.stack(positions)
    np.save(session_dir / "positions.npy", positions)
    np.save(session_dir / "sub_goals_local.npy", local_subgoals(positions, args.subgoal_lookahead))
    train, dev, test = split_filenames(filenames, args.train_ratio, args.dev_ratio, args.split_seed)
    for split, values in (("train", train), ("dev", dev), ("test", test)):
        (session_dir / f"{split}.txt").write_text("\n".join(values) + "\n", encoding="utf-8")
    metadata = {
        "bag": str(args.bag), "output_root": str(args.output_root), "session_name": args.session_name,
        "scan_01_topic": args.scan_01_topic, "scan_02_topic": args.scan_02_topic, "slots_per_sensor": SLOTS_PER_SENSOR, "total_slots": TOTAL_SLOTS,
        "sync_tolerance_ms": args.sync_tolerance_ms, "scan_01_messages": len(scans_01), "scan_02_messages": len(scans_02), "sync_skipped": sync_skipped,
        "samples": len(filenames), "train_samples": len(train), "dev_samples": len(dev), "test_samples": len(test), "split_seed": args.split_seed,
        "pose_source_used": dict(pose_source_counts), "label_names": label_names, "label_names_source": label_names_source,
        "label_histogram": {str(k): int(v) for k, v in sorted(label_hist.items())}, "topic_counts": dict(topic_counts),
        "average_range_invalid": float(np.mean(range_invalid_counts)), "average_self_points": float(np.mean(self_counts)), "average_valid_points": float(np.mean(valid_counts)),
        "average_valid_scan_01": float(np.mean(valid_01_counts)), "average_valid_scan_02": float(np.mean(valid_02_counts)),
        "near_angle_distinct_ranges_preserved": bool(near_angle_distinct_preserved),
        "no_cross_sensor_deduplication": True, "uses_scan_merged": False,
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
