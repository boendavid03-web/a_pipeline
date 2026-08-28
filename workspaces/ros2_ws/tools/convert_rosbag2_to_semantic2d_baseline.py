#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --cmd-vel-topic, --dev-ratio, --map-yaml, --odom-topic, --out-of-map-wall-radius-px, --output-root, --overwrite, --scan-topic, --semantic-label, --semantic-snap-radius-px, --session-name, --skip-dataset-index, --source-angle-max, --source-angle-min, --subgoal-lookahead, --target-points
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, NPY, TXT
# 可能使用的关键环境变量：BASELINE_ANGLE_MAX, BASELINE_ANGLE_MIN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_baseline.py
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_baseline.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_baseline.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_baseline.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜convert_rosbag2_to_semantic2d_baseline.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Convert a ROS 2 LaserScan/Odometry bag to Semantic2D baseline datasets."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BASELINE_ANGLE_MIN = -3.0 * math.pi / 4.0
BASELINE_ANGLE_MAX = 3.0 * math.pi / 4.0


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def msg_time_ns(msg, fallback_ns: int) -> int:
    header = getattr(msg, "header", None)
    if header is None:
        return fallback_ns
    return stamp_to_ns(header.stamp)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def read_bag(bag_path: Path, scan_topic: str, odom_topic: str, cmd_vel_topic: str):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = [scan_topic, odom_topic, cmd_vel_topic]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")

    scan_type = get_message(topic_types[scan_topic])
    odom_type = get_message(topic_types[odom_topic])
    cmd_type = get_message(topic_types[cmd_vel_topic])

    scans = []
    odoms = []
    cmds = []
    topic_counts = Counter()

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        topic_counts[topic] += 1
        if topic == scan_topic:
            msg = deserialize_message(data, scan_type)
            scans.append((int(storage_time), msg))
        elif topic == odom_topic:
            msg = deserialize_message(data, odom_type)
            pose = msg.pose.pose
            twist = msg.twist.twist
            odoms.append(
                (
                    int(storage_time),
                    (
                        float(pose.position.x),
                        float(pose.position.y),
                        yaw_from_quaternion(pose.orientation),
                        float(twist.linear.x),
                        float(twist.angular.z),
                    ),
                )
            )
        elif topic == cmd_vel_topic:
            msg = deserialize_message(data, cmd_type)
            cmds.append(
                (
                    int(storage_time),
                    (float(msg.linear.x), float(msg.angular.z)),
                )
            )

    scans.sort(key=lambda item: item[0])
    odoms.sort(key=lambda item: item[0])
    cmds.sort(key=lambda item: item[0])
    return scans, odoms, cmds, topic_types, topic_counts


def load_map_info(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    origin = data["origin"]
    return float(data["resolution"]), float(origin[0]), float(origin[1]), float(origin[2])


def sanitize_ranges(ranges: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    out = ranges.astype(np.float32, copy=True)
    invalid = ~np.isfinite(out)
    out[invalid] = range_max
    out = np.clip(out, max(range_min, 0.0), range_max)
    return out


def resample_scan(msg, target_points: int, source_angle_min, source_angle_max):
    ranges = sanitize_ranges(np.asarray(msg.ranges, dtype=np.float32), msg.range_min, msg.range_max)
    if len(ranges) == 0:
        raise RuntimeError("Encountered an empty LaserScan ranges array")

    src_min = float(msg.angle_min if source_angle_min is None else source_angle_min)
    src_max = float(msg.angle_max if source_angle_max is None else source_angle_max)
    src_angles = np.linspace(src_min, src_max, len(ranges), dtype=np.float64)

    target_angles = np.linspace(BASELINE_ANGLE_MIN, BASELINE_ANGLE_MAX, target_points, dtype=np.float64)
    if target_angles[0] < src_angles[0] or target_angles[-1] > src_angles[-1]:
        target_angles = np.linspace(src_angles[0], src_angles[-1], target_points, dtype=np.float64)

    resampled = np.interp(target_angles, src_angles, ranges).astype(np.float32)

    intensities = np.asarray(msg.intensities, dtype=np.float32)
    if len(intensities) == len(ranges):
        intensities = np.interp(target_angles, src_angles, intensities).astype(np.float32)
    else:
        intensities = np.zeros(target_points, dtype=np.float32)

    return resampled, intensities, target_angles.astype(np.float32)


def semantic_snap_offsets(radius_px: int):
    if radius_px <= 0:
        return []
    offsets = []
    radius_sq = radius_px * radius_px
    for dr in range(-radius_px, radius_px + 1):
        for dc in range(-radius_px, radius_px + 1):
            dist_sq = dr * dr + dc * dc
            if dist_sq == 0 or dist_sq > radius_sq:
                continue
            offsets.append((dist_sq, dr, dc))
    offsets.sort()
    return [(dr, dc) for _dist_sq, dr, dc in offsets]


def semantic_for_scan(
    ranges,
    angles,
    pose,
    label_img,
    resolution,
    origin_x,
    origin_y,
    snap_offsets=None,
    out_of_map_wall_radius_px: float = 0.0,
):
    x, y, yaw = pose
    height, width = label_img.shape[:2]
    world_x = x + ranges * np.cos(yaw + angles)
    world_y = y + ranges * np.sin(yaw + angles)
    cols_float = (world_x - origin_x) / resolution
    map_rows_float = (world_y - origin_y) / resolution
    rows_float = (height - 1) - map_rows_float
    cols = np.floor(cols_float).astype(np.int64)
    rows = height - 1 - np.floor(map_rows_float).astype(np.int64)
    finite = np.isfinite(ranges)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height) & finite
    labels = np.zeros(len(ranges), dtype=np.int64)
    labels[valid] = label_img[rows[valid], cols[valid]].astype(np.int64)
    if snap_offsets:
        snap_mask = valid & (labels == 0)
        snap_indices = np.flatnonzero(snap_mask)
        for idx in snap_indices:
            row = rows[idx]
            col = cols[idx]
            for dr, dc in snap_offsets:
                rr = row + dr
                cc = col + dc
                if 0 <= rr < height and 0 <= cc < width:
                    label = int(label_img[rr, cc])
                    if label != 0:
                        labels[idx] = label
                        break
    if out_of_map_wall_radius_px > 0.0:
        out_of_map = finite & ~valid
        out_indices = np.flatnonzero(out_of_map)
        if len(out_indices):
            clipped_cols_float = np.clip(cols_float[out_indices], 0.0, float(width - 1))
            clipped_rows_float = np.clip(rows_float[out_indices], 0.0, float(height - 1))
            outside_dist = np.hypot(
                cols_float[out_indices] - clipped_cols_float,
                rows_float[out_indices] - clipped_rows_float,
            )
            near_indices = out_indices[outside_dist <= out_of_map_wall_radius_px]
            if len(near_indices):
                clipped_cols = np.floor(
                    np.clip(cols_float[near_indices], 0.0, float(width - 1))
                ).astype(np.int64)
                clipped_rows = np.floor(
                    np.clip(rows_float[near_indices], 0.0, float(height - 1))
                ).astype(np.int64)
                wall_mask = label_img[clipped_rows, clipped_cols] == 9
                labels[near_indices[wall_mask]] = 9
    return labels


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


def write_report(report_path: Path, metadata: dict):
    hist_lines = "\n".join(
        f"- {label}: {count}" for label, count in sorted(metadata["label_histogram"].items(), key=lambda kv: int(kv[0]))
    )
    report = f"""# ROS 2 Bag to Semantic2D Baseline Conversion

## Inputs
- bag: `{metadata['bag']}`
- map yaml: `{metadata['map_yaml']}`
- semantic label: `{metadata['semantic_label']}`
- scan topic: `{metadata['scan_topic']}`
- odom topic: `{metadata['odom_topic']}`
- cmd_vel topic: `{metadata['cmd_vel_topic']}`

## Output
- dataset root: `{metadata['output_root']}`
- session: `{metadata['session_name']}`
- samples: {metadata['samples']}
- target points: {metadata['target_points']}
- train samples: {metadata['train_samples']}
- dev samples: {metadata['dev_samples']}

## Label Histogram
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
    parser.add_argument("--target-points", default=1081, type=int)
    parser.add_argument("--source-angle-min", default=None, type=float)
    parser.add_argument("--source-angle-max", default=None, type=float)
    parser.add_argument("--dev-ratio", default=0.2, type=float)
    parser.add_argument("--subgoal-lookahead", default=20, type=int)
    parser.add_argument("--semantic-snap-radius-px", default=3, type=int)
    parser.add_argument("--out-of-map-wall-radius-px", default=0.0, type=float)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--skip-dataset-index", action="store_true", help="Do not rewrite output-root/dataset.txt")
    return parser.parse_args()


def main():
    args = parse_args()
    if not (0.0 <= args.dev_ratio < 1.0):
        raise ValueError("--dev-ratio must be in [0, 1)")
    if args.semantic_snap_radius_px < 0:
        raise ValueError("--semantic-snap-radius-px must be >= 0")
    if args.out_of_map_wall_radius_px < 0.0:
        raise ValueError("--out-of-map-wall-radius-px must be >= 0")
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
        "semantic_label",
        "positions",
        "velocities",
        "sub_goals_local",
    ]
    for subdir in subdirs:
        (session_dir / subdir).mkdir()

    scans, odoms, cmds, topic_types, topic_counts = read_bag(
        args.bag, args.scan_topic, args.odom_topic, args.cmd_vel_topic
    )
    if not scans:
        raise RuntimeError(f"No scans found on {args.scan_topic}")
    if not odoms:
        raise RuntimeError(f"No odometry found on {args.odom_topic}")

    resolution, origin_x, origin_y, origin_yaw = load_map_info(args.map_yaml)
    label_img = np.asarray(Image.open(args.semantic_label))
    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.int64)
    snap_offsets = semantic_snap_offsets(args.semantic_snap_radius_px)

    all_scans = []
    all_intensities = []
    all_labels = []
    all_positions = []
    all_velocities = []
    angle_min_used = None
    angle_max_used = None

    for stamp_ns, scan_msg in scans:
        odom_match = nearest_by_time(odoms, stamp_ns)
        cmd_match = nearest_by_time(cmds, stamp_ns)
        if odom_match is None:
            continue
        x, y, yaw, odom_lin, odom_ang = odom_match[1]
        if cmd_match is not None:
            lin_x, ang_z = cmd_match[1]
        else:
            lin_x, ang_z = odom_lin, odom_ang

        scan, intensity, angles = resample_scan(
            scan_msg, args.target_points, args.source_angle_min, args.source_angle_max
        )
        angle_min_used = float(angles[0])
        angle_max_used = float(angles[-1])
        semantic = semantic_for_scan(
            scan,
            angles,
            (x, y, yaw),
            label_img,
            resolution,
            origin_x,
            origin_y,
            snap_offsets,
            args.out_of_map_wall_radius_px,
        )

        all_scans.append(scan)
        all_intensities.append(intensity)
        all_labels.append(semantic)
        all_positions.append(np.asarray([x, y, yaw], dtype=np.float32))
        all_velocities.append(np.asarray([lin_x, ang_z], dtype=np.float32))

    if not all_scans:
        raise RuntimeError("No samples were converted")

    positions = np.stack(all_positions).astype(np.float32)
    subgoals = local_subgoals(positions, args.subgoal_lookahead)
    label_hist = Counter()
    filenames = []
    train = []
    dev = []
    dev_every = int(round(1.0 / args.dev_ratio)) if args.dev_ratio > 0 else 0

    for i, (scan, intensity, semantic, position, velocity, subgoal) in enumerate(
        zip(all_scans, all_intensities, all_labels, all_positions, all_velocities, subgoals)
    ):
        name = f"{i:07d}.npy"
        filenames.append(name)
        np.save(session_dir / "scans_lidar" / name, scan.astype(np.float32))
        np.save(session_dir / "intensities_lidar" / name, intensity.astype(np.float32))
        np.save(session_dir / "semantic_label" / name, semantic.astype(np.int64))
        np.save(session_dir / "positions" / name, position.astype(np.float32))
        np.save(session_dir / "velocities" / name, velocity.astype(np.float32))
        np.save(session_dir / "sub_goals_local" / name, subgoal.astype(np.float32))
        label_hist.update(int(v) for v in semantic.tolist())
        if dev_every and i % dev_every == 0:
            dev.append(name)
        else:
            train.append(name)

    (session_dir / "train.txt").write_text("\n".join(train) + "\n", encoding="utf-8")
    (session_dir / "dev.txt").write_text("\n".join(dev) + "\n", encoding="utf-8")
    if not args.skip_dataset_index:
        (args.output_root / "dataset.txt").write_text(f"{args.session_name}/\n", encoding="utf-8")

    metadata = {
        "bag": str(args.bag),
        "output_root": str(args.output_root),
        "session_name": args.session_name,
        "map_yaml": str(args.map_yaml),
        "semantic_label": str(args.semantic_label),
        "scan_topic": args.scan_topic,
        "odom_topic": args.odom_topic,
        "cmd_vel_topic": args.cmd_vel_topic,
        "target_points": args.target_points,
        "source_angle_min": args.source_angle_min,
        "source_angle_max": args.source_angle_max,
        "target_angle_min_used": angle_min_used,
        "target_angle_max_used": angle_max_used,
        "dev_ratio": args.dev_ratio,
        "subgoal_lookahead": args.subgoal_lookahead,
        "semantic_snap_radius_px": args.semantic_snap_radius_px,
        "out_of_map_wall_radius_px": args.out_of_map_wall_radius_px,
        "samples": len(filenames),
        "train_samples": len(train),
        "dev_samples": len(dev),
        "map": {
            "resolution": resolution,
            "origin": [origin_x, origin_y, origin_yaw],
            "label_shape": list(label_img.shape),
        },
        "topic_types": topic_types,
        "topic_counts": dict(topic_counts),
        "label_histogram": {str(k): int(v) for k, v in sorted(label_hist.items())},
    }
    (session_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    write_report(session_dir / "conversion_report.md", metadata)

    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
