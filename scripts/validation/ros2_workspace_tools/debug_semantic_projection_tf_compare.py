#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --base-frame, --map-frame, --map-yaml, --max-points-per-class, --odom-frame, --odom-topic, --output-dir, --overlay-sample-every, --scan-topic, --semantic-label, --semantic-viz, --target-points
# 代码中检测到的 ROS 2 话题/路径字符串：/odom, /scan_merged, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, PNG
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_tf_compare.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.950217933 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_tf_compare.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_tf_compare.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_tf_compare.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜debug_semantic_projection_tf_compare.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Compare odom-frame and TF map-frame semantic projections for a ROS 2 bag."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.ndimage import distance_transform_edt

from convert_rosbag2_to_semantic2d_baseline import (
    load_map_info,
    nearest_by_time,
    resample_scan,
    stamp_to_ns,
    yaw_from_quaternion,
)


def normalize_frame(frame: str) -> str:
    return frame.strip("/")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_to_pose2d(transform):
    t = transform.translation
    return float(t.x), float(t.y), yaw_from_quaternion(transform.rotation)


def invert_pose2d(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return -c * x - s * y, s * x - c * y, normalize_angle(-yaw)


def compose_pose2d(first, second):
    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    c = math.cos(yaw1)
    s = math.sin(yaw1)
    return (
        x1 + c * x2 - s * y2,
        y1 + s * x2 + c * y2,
        normalize_angle(yaw1 + yaw2),
    )


def nearest_timed_pose(items, stamp_ns: int):
    if not items:
        return None
    times = [item[0] for item in items]
    idx = bisect.bisect_left(times, stamp_ns)
    if idx <= 0:
        item = items[0]
    elif idx >= len(items):
        item = items[-1]
    else:
        before = items[idx - 1]
        after = items[idx]
        item = before if abs(before[0] - stamp_ns) <= abs(after[0] - stamp_ns) else after
    return item[2], abs(item[0] - stamp_ns), item[1]


def lookup_transform(dynamic_tfs, static_tfs, parent: str, child: str, stamp_ns: int):
    parent = normalize_frame(parent)
    child = normalize_frame(child)
    if parent == child:
        return (0.0, 0.0, 0.0), 0, "identity"

    key = (parent, child)
    if key in dynamic_tfs:
        pose, age_ns, storage_ns = nearest_timed_pose(dynamic_tfs[key], stamp_ns)
        return pose, age_ns, f"dynamic:{storage_ns}"
    if key in static_tfs:
        return static_tfs[key], 0, "static"

    inverse_key = (child, parent)
    if inverse_key in dynamic_tfs:
        pose, age_ns, storage_ns = nearest_timed_pose(dynamic_tfs[inverse_key], stamp_ns)
        return invert_pose2d(pose), age_ns, f"dynamic_inverse:{storage_ns}"
    if inverse_key in static_tfs:
        return invert_pose2d(static_tfs[inverse_key]), 0, "static_inverse"

    return None


def lookup_map_to_scan(dynamic_tfs, static_tfs, map_frame: str, odom_frame: str, base_frame: str, scan_frame: str, stamp_ns: int):
    direct = lookup_transform(dynamic_tfs, static_tfs, map_frame, scan_frame, stamp_ns)
    if direct is not None:
        pose, age_ns, source = direct
        return pose, [age_ns], [source], []

    missing = []
    map_to_odom = lookup_transform(dynamic_tfs, static_tfs, map_frame, odom_frame, stamp_ns)
    if map_to_odom is None:
        missing.append(f"{map_frame}->{odom_frame}")
    odom_to_base = lookup_transform(dynamic_tfs, static_tfs, odom_frame, base_frame, stamp_ns)
    if odom_to_base is None:
        missing.append(f"{odom_frame}->{base_frame}")
    if missing:
        return None, [], [], missing

    pose = compose_pose2d(map_to_odom[0], odom_to_base[0])
    ages = [map_to_odom[1], odom_to_base[1]]
    sources = [map_to_odom[2], odom_to_base[2]]

    if normalize_frame(scan_frame) != normalize_frame(base_frame):
        base_to_scan = lookup_transform(dynamic_tfs, static_tfs, base_frame, scan_frame, stamp_ns)
        if base_to_scan is None:
            return None, ages, sources, [f"{base_frame}->{scan_frame}"]
        pose = compose_pose2d(pose, base_to_scan[0])
        ages.append(base_to_scan[1])
        sources.append(base_to_scan[2])

    return pose, ages, sources, []


def project_labels(ranges, angles, pose, label_img, resolution, origin_x, origin_y):
    x, y, yaw = pose
    height, width = label_img.shape[:2]
    world_x = x + ranges * np.cos(yaw + angles)
    world_y = y + ranges * np.sin(yaw + angles)
    cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
    finite = np.isfinite(ranges)
    in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height) & finite
    labels = np.zeros(len(ranges), dtype=np.int64)
    labels[in_map] = label_img[rows[in_map], cols[in_map]].astype(np.int64)
    return labels, rows, cols, finite, in_map


def empty_method_stats():
    return {
        "counts": Counter(),
        "label_histogram": Counter(),
        "other_distance_counts_px": Counter(),
        "other_points": [],
        "non_other_points": [],
    }


def add_projection_stats(method, labels, rows, cols, finite, in_map, distance_to_nonzero, collect_points: bool):
    counts = method["counts"]
    counts["total_beams"] += int(len(labels))
    counts["finite_beams"] += int(finite.sum())
    counts["in_map_finite"] += int(in_map.sum())
    counts["out_of_map_or_invalid"] += int((~in_map).sum())
    counts["other_all"] += int((labels == 0).sum())
    other_in_map = (labels == 0) & in_map
    non_other_in_map = (labels != 0) & in_map
    counts["other_in_map_finite"] += int(other_in_map.sum())
    counts["non_other_in_map_finite"] += int(non_other_in_map.sum())
    method["label_histogram"].update(int(v) for v in labels.tolist())

    if other_in_map.any():
        distances = distance_to_nonzero[rows[other_in_map], cols[other_in_map]]
        for radius in (1, 2, 3, 5):
            method["other_distance_counts_px"][str(radius)] += int((distances <= radius).sum())
    if collect_points:
        if other_in_map.any():
            method["other_points"].append(np.column_stack([cols[other_in_map], rows[other_in_map]]))
        if non_other_in_map.any():
            method["non_other_points"].append(np.column_stack([cols[non_other_in_map], rows[non_other_in_map]]))


def summarize_method(method):
    counts = {k: int(v) for k, v in method["counts"].items()}
    total = counts.get("total_beams", 0)
    in_map = counts.get("in_map_finite", 0)
    other_in_map = counts.get("other_in_map_finite", 0)

    def pct(n, d):
        return 100.0 * float(n) / float(d) if d else 0.0

    return {
        "counts": counts,
        "label_histogram": {str(k): int(v) for k, v in sorted(method["label_histogram"].items())},
        "summary": {
            "other_all_pct": pct(counts.get("other_all", 0), total),
            "out_of_map_or_invalid_pct": pct(counts.get("out_of_map_or_invalid", 0), total),
            "other_in_map_finite_pct": pct(other_in_map, in_map),
        },
        "other_near_nonzero_px": {
            radius: {
                "count": int(count),
                "pct_of_other_in_map_finite": pct(count, other_in_map),
            }
            for radius, count in sorted(method["other_distance_counts_px"].items(), key=lambda kv: int(kv[0]))
        },
    }


def stack_and_limit(chunks, limit: int):
    if not chunks:
        return np.empty((0, 2), dtype=np.int64)
    points = np.vstack(chunks)
    if len(points) > limit:
        rng = np.random.default_rng(0)
        points = points[rng.choice(len(points), limit, replace=False)]
    return points


def draw_overlay(path: Path, base_img, method, width: int, height: int, max_points: int):
    other_pts = stack_and_limit(method["other_points"], max_points)
    non_other_pts = stack_and_limit(method["non_other_points"], max_points)
    fig, ax = plt.subplots(figsize=(12, 9), dpi=180)
    ax.imshow(base_img)
    if len(non_other_pts):
        ax.scatter(non_other_pts[:, 0], non_other_pts[:, 1], s=0.8, c="#1f77ff", alpha=0.35, label="label != 0")
    if len(other_pts):
        ax.scatter(other_pts[:, 0], other_pts[:, 1], s=0.8, c="#ff2b2b", alpha=0.45, label="label == 0")
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_axis_off()
    ax.legend(loc="lower right", markerscale=8, framealpha=0.85)
    fig.tight_layout(pad=0)
    fig.savefig(path, bbox_inches="tight", pad_inches=0)
    plt.close(fig)


def read_bag_inputs(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = [args.scan_topic, args.odom_topic, "/tf", "/tf_static"]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {', '.join(missing)}")

    scan_type = get_message(topic_types[args.scan_topic])
    odom_type = get_message(topic_types[args.odom_topic])
    tf_type = get_message(topic_types["/tf"])
    tf_static_type = get_message(topic_types["/tf_static"])

    scans = []
    odoms = []
    dynamic_tfs = defaultdict(list)
    static_tfs = {}
    tf_pair_counts = Counter()
    tf_pair_topics = defaultdict(Counter)
    topic_counts = Counter()

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        storage_time = int(storage_time)
        topic_counts[topic] += 1
        if topic == args.scan_topic:
            msg = deserialize_message(data, scan_type)
            scans.append((storage_time, stamp_to_ns(msg.header.stamp), normalize_frame(msg.header.frame_id), msg))
        elif topic == args.odom_topic:
            msg = deserialize_message(data, odom_type)
            pose = msg.pose.pose
            twist = msg.twist.twist
            odoms.append(
                (
                    storage_time,
                    (
                        float(pose.position.x),
                        float(pose.position.y),
                        yaw_from_quaternion(pose.orientation),
                        float(twist.linear.x),
                        float(twist.angular.z),
                    ),
                )
            )
        elif topic in ("/tf", "/tf_static"):
            msg_type = tf_type if topic == "/tf" else tf_static_type
            msg = deserialize_message(data, msg_type)
            for tr in msg.transforms:
                parent = normalize_frame(tr.header.frame_id)
                child = normalize_frame(tr.child_frame_id)
                stamp_ns = stamp_to_ns(tr.header.stamp)
                pose = transform_to_pose2d(tr.transform)
                key = (parent, child)
                tf_pair_counts[key] += 1
                tf_pair_topics[key][topic] += 1
                if topic == "/tf_static":
                    static_tfs[key] = pose
                else:
                    dynamic_tfs[key].append((stamp_ns, storage_time, pose))

    for values in dynamic_tfs.values():
        values.sort(key=lambda item: item[0])
    scans.sort(key=lambda item: item[0])
    odoms.sort(key=lambda item: item[0])

    tf_pairs = {}
    for key in sorted(tf_pair_counts):
        times = [item[0] for item in dynamic_tfs.get(key, [])]
        tf_pairs[f"{key[0]}->{key[1]}"] = {
            "count": int(tf_pair_counts[key]),
            "topics": dict(tf_pair_topics[key]),
            "stamp_min_ns": int(min(times)) if times else None,
            "stamp_max_ns": int(max(times)) if times else None,
            "static": key in static_tfs,
        }

    return scans, odoms, dynamic_tfs, static_tfs, topic_types, topic_counts, tf_pairs


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--semantic-viz", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--target-points", default=1081, type=int)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--overlay-sample-every", default=5, type=int)
    parser.add_argument("--max-points-per-class", default=120000, type=int)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    resolution, origin_x, origin_y, _ = load_map_info(args.map_yaml)
    label_img = np.asarray(Image.open(args.semantic_label))
    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.int64)
    height, width = label_img.shape[:2]
    distance_to_nonzero = distance_transform_edt(label_img == 0)

    if args.semantic_viz and args.semantic_viz.exists():
        base_img = Image.open(args.semantic_viz).convert("RGB")
    else:
        base_arr = np.zeros((height, width, 3), dtype=np.uint8)
        base_arr[label_img == 0] = (35, 35, 35)
        base_arr[label_img != 0] = (185, 185, 185)
        base_img = Image.fromarray(base_arr)

    scans, odoms, dynamic_tfs, static_tfs, topic_types, topic_counts, tf_pairs = read_bag_inputs(args)
    if not scans:
        raise RuntimeError(f"No scans found on {args.scan_topic}")
    if not odoms:
        raise RuntimeError(f"No odometry found on {args.odom_topic}")

    methods = {"odom": empty_method_stats(), "tf": empty_method_stats()}
    missing_tf = Counter()
    tf_age_ns = []
    pose_delta_xy = []
    pose_delta_yaw = []
    scan_frames = Counter()

    for scan_idx, (scan_storage_ns, scan_stamp_ns, scan_frame, scan_msg) in enumerate(scans):
        scan_frames[scan_frame] += 1
        ranges, _intensity, angles = resample_scan(scan_msg, args.target_points, None, None)
        collect_points = args.overlay_sample_every <= 1 or scan_idx % args.overlay_sample_every == 0

        odom_match = nearest_by_time(odoms, scan_storage_ns)
        if odom_match is not None:
            x, y, yaw, _odom_lin, _odom_ang = odom_match[1]
            labels, rows, cols, finite, in_map = project_labels(
                ranges, angles, (x, y, yaw), label_img, resolution, origin_x, origin_y
            )
            add_projection_stats(methods["odom"], labels, rows, cols, finite, in_map, distance_to_nonzero, collect_points)
            odom_pose = (x, y, yaw)
        else:
            odom_pose = None

        tf_pose, ages, _sources, missing = lookup_map_to_scan(
            dynamic_tfs,
            static_tfs,
            args.map_frame,
            args.odom_frame,
            args.base_frame,
            scan_frame,
            scan_stamp_ns,
        )
        if missing:
            missing_tf.update(missing)
            methods["tf"]["counts"]["missing_pose_scans"] += 1
            methods["tf"]["counts"]["missing_pose_beams"] += int(len(ranges))
            methods["tf"]["counts"]["total_beams"] += int(len(ranges))
            methods["tf"]["counts"]["out_of_map_or_invalid"] += int(len(ranges))
            methods["tf"]["counts"]["other_all"] += int(len(ranges))
            methods["tf"]["label_histogram"].update([0] * len(ranges))
            continue

        tf_age_ns.extend(ages)
        labels, rows, cols, finite, in_map = project_labels(
            ranges, angles, tf_pose, label_img, resolution, origin_x, origin_y
        )
        add_projection_stats(methods["tf"], labels, rows, cols, finite, in_map, distance_to_nonzero, collect_points)

        if odom_pose is not None:
            dx = tf_pose[0] - odom_pose[0]
            dy = tf_pose[1] - odom_pose[1]
            pose_delta_xy.append(math.hypot(dx, dy))
            pose_delta_yaw.append(abs(normalize_angle(tf_pose[2] - odom_pose[2])))

    draw_overlay(
        args.output_dir / "semantic_projection_overlay_odom.png",
        base_img,
        methods["odom"],
        width,
        height,
        args.max_points_per_class,
    )
    draw_overlay(
        args.output_dir / "semantic_projection_overlay_tf.png",
        base_img,
        methods["tf"],
        width,
        height,
        args.max_points_per_class,
    )

    def array_stats(values):
        if not values:
            return None
        arr = np.asarray(values, dtype=np.float64)
        return {
            "min": float(np.min(arr)),
            "mean": float(np.mean(arr)),
            "median": float(np.median(arr)),
            "p95": float(np.percentile(arr, 95)),
            "max": float(np.max(arr)),
        }

    report = {
        "bag": str(args.bag),
        "scan_topic": args.scan_topic,
        "odom_topic": args.odom_topic,
        "topic_types": topic_types,
        "topic_counts": dict(topic_counts),
        "tf_pairs": tf_pairs,
        "scan_frames": dict(scan_frames),
        "missing_tf": dict(missing_tf),
        "tf_lookup_age_ns": array_stats(tf_age_ns),
        "pose_delta_tf_minus_odom_m": array_stats(pose_delta_xy),
        "pose_delta_tf_minus_odom_yaw_rad": array_stats(pose_delta_yaw),
        "methods": {name: summarize_method(method) for name, method in methods.items()},
    }
    stats_path = args.output_dir / "semantic_projection_tf_compare_stats.json"
    stats_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
