#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --cmd-vel-topic, --map-yaml, --max-points-per-class, --odom-topic, --output-dir, --overlay-prefix, --overlay-sample-every, --scan-topic, --semantic-label, --semantic-viz, --snap-radii, --target-points
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, PNG, TXT
# 可能使用的关键环境变量：DEFAULT_LABEL_NAMES
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_snap_sweep.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.685303552 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:31.259092401 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_snap_sweep.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_snap_sweep.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_snap_sweep.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜debug_semantic_projection_snap_sweep.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Draw odom-frame semantic projections and sweep semantic snap radii."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

from convert_rosbag2_to_semantic2d_baseline import (
    load_map_info,
    nearest_by_time,
    read_bag,
    resample_scan,
)


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


def load_label_names(semantic_label: Path):
    label_names_path = semantic_label.with_name("label_names.txt")
    if label_names_path.exists():
        names = [line.strip() for line in label_names_path.read_text(encoding="utf-8").splitlines()]
        names = [name for name in names if name]
        if names:
            return names, str(label_names_path)
    return list(DEFAULT_LABEL_NAMES), "default"


def label_name(label_names, label: int) -> str:
    if 0 <= label < len(label_names):
        return label_names[label]
    return str(label)


def project(ranges, angles, pose, label_img, resolution, origin_x, origin_y):
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


def snap_labels(labels, rows, cols, in_map, distance_to_nonzero, nearest_nonzero_label, radius_px: int):
    if radius_px <= 0:
        return labels
    snapped = labels.copy()
    candidates = (snapped == 0) & in_map
    close = np.zeros(len(labels), dtype=bool)
    candidate_indices = np.flatnonzero(candidates)
    if len(candidate_indices):
        candidate_rows = rows[candidate_indices]
        candidate_cols = cols[candidate_indices]
        close[candidate_indices] = distance_to_nonzero[candidate_rows, candidate_cols] <= radius_px
    snapped[close] = nearest_nonzero_label[rows[close], cols[close]]
    return snapped


def add_label_stats(stats, labels, finite, in_map):
    counts = stats["counts"]
    counts["total_beams"] += int(len(labels))
    counts["finite_beams"] += int(finite.sum())
    counts["in_map_finite"] += int(in_map.sum())
    counts["out_of_map_or_invalid"] += int((~in_map).sum())
    counts["other_all"] += int((labels == 0).sum())
    counts["other_in_map_finite"] += int(((labels == 0) & in_map).sum())
    stats["hist"].update(int(v) for v in labels.tolist())


def summarize_stats(stats, label_names):
    counts = {k: int(v) for k, v in stats["counts"].items()}
    hist = {str(k): int(v) for k, v in sorted(stats["hist"].items())}
    total = counts.get("total_beams", 0)
    in_map = counts.get("in_map_finite", 0)

    def pct(value, denom):
        return 100.0 * float(value) / float(denom) if denom else 0.0

    return {
        "counts": counts,
        "label_histogram": hist,
        "summary": {
            "other_all_pct": pct(counts.get("other_all", 0), total),
            "out_of_map_or_invalid_pct": pct(counts.get("out_of_map_or_invalid", 0), total),
            "other_in_map_finite_pct": pct(counts.get("other_in_map_finite", 0), in_map),
        },
        "class_percent_total": {
            str(k): {
                "name": label_name(label_names, k),
                "count": int(stats["hist"].get(k, 0)),
                "pct": pct(stats["hist"].get(k, 0), total),
            }
            for k in range(10)
        },
    }


def stack_and_limit(chunks, max_points):
    if not chunks:
        return np.empty((0, 2), dtype=np.int64)
    points = np.vstack(chunks)
    if len(points) > max_points:
        rng = np.random.default_rng(0)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def draw_overlay(path, base_img, other_chunks, non_other_chunks, width, height, max_points, include_non_other=True):
    other_pts = stack_and_limit(other_chunks, max_points)
    non_other_pts = stack_and_limit(non_other_chunks, max_points)
    fig, ax = plt.subplots(figsize=(12, 9), dpi=180)
    ax.imshow(base_img)
    if include_non_other and len(non_other_pts):
        ax.scatter(non_other_pts[:, 0], non_other_pts[:, 1], s=0.8, c="#1f77ff", alpha=0.35, label="label != 0")
    if len(other_pts):
        ax.scatter(other_pts[:, 0], other_pts[:, 1], s=0.8, c="#ff2b2b", alpha=0.45, label="label == 0")
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_axis_off()
    if include_non_other:
        ax.legend(loc="lower right", markerscale=8, framealpha=0.85)
    fig.tight_layout(pad=0)
    fig.savefig(path, bbox_inches="tight", pad_inches=0)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--semantic-viz", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--overlay-prefix", default="new_label_projection")
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--target-points", default=1081, type=int)
    parser.add_argument("--snap-radii", default="0,1,2,3,5")
    parser.add_argument("--overlay-sample-every", default=5, type=int)
    parser.add_argument("--max-points-per-class", default=120000, type=int)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    snap_radii = [int(item) for item in args.snap_radii.split(",") if item.strip()]
    if any(radius < 0 for radius in snap_radii):
        raise ValueError("--snap-radii values must be >= 0")

    resolution, origin_x, origin_y, _ = load_map_info(args.map_yaml)
    label_img = np.asarray(Image.open(args.semantic_label))
    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.int64)
    label_names, label_names_source = load_label_names(args.semantic_label)
    height, width = label_img.shape[:2]

    label_values, label_counts = np.unique(label_img, return_counts=True)
    total_pixels = int(label_img.size)
    label_pixel_hist = {
        str(k): {
            "name": label_name(label_names, k),
            "count": int(label_counts[np.where(label_values == k)][0]) if k in label_values else 0,
            "pct": (
                100.0 * int(label_counts[np.where(label_values == k)][0]) / total_pixels
                if k in label_values
                else 0.0
            ),
        }
        for k in range(10)
    }

    distance_to_nonzero, nearest_indices = distance_transform_edt(label_img == 0, return_indices=True)
    nearest_nonzero_label = label_img[nearest_indices[0], nearest_indices[1]]

    if args.semantic_viz and args.semantic_viz.exists():
        base_img = Image.open(args.semantic_viz).convert("RGB")
    else:
        base_arr = np.zeros((height, width, 3), dtype=np.uint8)
        base_arr[label_img == 0] = (35, 35, 35)
        base_arr[label_img != 0] = (185, 185, 185)
        base_img = Image.fromarray(base_arr)

    scans, odoms, _cmds, topic_types, topic_counts = read_bag(
        args.bag, args.scan_topic, args.odom_topic, args.cmd_vel_topic
    )

    stats_by_radius = {
        radius: {"counts": Counter(), "hist": Counter()}
        for radius in snap_radii
    }
    near_counts = Counter()
    other_chunks = []
    non_other_chunks = []
    snap_other_chunks = {radius: [] for radius in snap_radii}
    range_counts = Counter()

    for scan_idx, (stamp_ns, scan_msg) in enumerate(scans):
        odom_match = nearest_by_time(odoms, stamp_ns)
        if odom_match is None:
            continue

        x, y, yaw, _odom_lin, _odom_ang = odom_match[1]
        ranges, _intensities, angles = resample_scan(scan_msg, args.target_points, None, None)
        labels, rows, cols, finite, in_map = project(
            ranges, angles, (x, y, yaw), label_img, resolution, origin_x, origin_y
        )

        range_counts["total_beams"] += int(len(ranges))
        range_counts["max_range_like"] += int((ranges >= (float(scan_msg.range_max) - 1e-3)).sum())
        range_counts["range_min_or_less"] += int((ranges <= (float(scan_msg.range_min) + 1e-3)).sum())

        other_in_map = (labels == 0) & in_map
        if other_in_map.any():
            distances = distance_to_nonzero[rows[other_in_map], cols[other_in_map]]
            for radius in (1, 2, 3, 5):
                near_counts[str(radius)] += int((distances <= radius).sum())

        collect_points = args.overlay_sample_every <= 1 or scan_idx % args.overlay_sample_every == 0
        if collect_points:
            if other_in_map.any():
                other_chunks.append(np.column_stack([cols[other_in_map], rows[other_in_map]]))
            non_other_in_map = (labels != 0) & in_map
            if non_other_in_map.any():
                non_other_chunks.append(np.column_stack([cols[non_other_in_map], rows[non_other_in_map]]))

        for radius in snap_radii:
            snapped = snap_labels(labels, rows, cols, in_map, distance_to_nonzero, nearest_nonzero_label, radius)
            add_label_stats(stats_by_radius[radius], snapped, finite, in_map)
            if collect_points:
                snapped_other_in_map = (snapped == 0) & in_map
                if snapped_other_in_map.any():
                    snap_other_chunks[radius].append(
                        np.column_stack([cols[snapped_other_in_map], rows[snapped_other_in_map]])
                    )

    overlay_path = args.output_dir / f"{args.overlay_prefix}_overlay_odom.png"
    other_only_path = args.output_dir / f"{args.overlay_prefix}_other_only_odom.png"
    draw_overlay(
        overlay_path,
        base_img,
        other_chunks,
        non_other_chunks,
        width,
        height,
        args.max_points_per_class,
        include_non_other=True,
    )
    draw_overlay(
        other_only_path,
        base_img,
        other_chunks,
        non_other_chunks,
        width,
        height,
        args.max_points_per_class,
        include_non_other=False,
    )
    snap_other_outputs = {}
    for radius in snap_radii:
        path = args.output_dir / f"{args.overlay_prefix}_snap{radius}_other_only_odom.png"
        draw_overlay(
            path,
            base_img,
            snap_other_chunks[radius],
            [],
            width,
            height,
            args.max_points_per_class,
            include_non_other=False,
        )
        snap_other_outputs[str(radius)] = str(path)

    snap0 = stats_by_radius[min(snap_radii)] if 0 in stats_by_radius else next(iter(stats_by_radius.values()))
    other_in_map = snap0["counts"].get("other_in_map_finite", 0)

    def pct(value, denom):
        return 100.0 * float(value) / float(denom) if denom else 0.0

    report = {
        "bag": str(args.bag),
        "map_yaml": str(args.map_yaml),
        "semantic_label": str(args.semantic_label),
        "label_names_source": label_names_source,
        "semantic_viz": str(args.semantic_viz) if args.semantic_viz else None,
        "topic_types": topic_types,
        "topic_counts": dict(topic_counts),
        "label_pixels": {
            "shape": [int(height), int(width)],
            "total": total_pixels,
            "classes": label_pixel_hist,
        },
        "range_summary": {
            "total_beams": int(range_counts["total_beams"]),
            "max_range_like": int(range_counts["max_range_like"]),
            "max_range_like_pct": pct(range_counts["max_range_like"], range_counts["total_beams"]),
            "range_min_or_less": int(range_counts["range_min_or_less"]),
            "range_min_or_less_pct": pct(range_counts["range_min_or_less"], range_counts["total_beams"]),
        },
        "snap": {
            str(radius): summarize_stats(stats_by_radius[radius], label_names)
            for radius in snap_radii
        },
        "snap0_other_near_nonzero_px": {
            str(radius): {
                "count": int(near_counts[str(radius)]),
                "pct_of_other_in_map_finite": pct(near_counts[str(radius)], other_in_map),
            }
            for radius in (1, 2, 3, 5)
        },
        "outputs": {
            "overlay": str(overlay_path),
            "other_only": str(other_only_path),
            "snap_other_only": snap_other_outputs,
        },
    }
    stats_path = args.output_dir / f"{args.overlay_prefix}_stats_odom.json"
    stats_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
