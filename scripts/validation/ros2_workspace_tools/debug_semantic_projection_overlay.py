#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --cmd-vel-topic, --map-yaml, --max-points-per-class, --odom-topic, --output-dir, --sample-every, --scan-topic, --semantic-label, --semantic-viz, --target-points
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON, PNG
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_overlay.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.949217914 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_overlay.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/debug_semantic_projection_overlay.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/debug_semantic_projection_overlay.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜debug_semantic_projection_overlay.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Draw Semantic2D endpoint projections over a semantic label image."""

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

from convert_rosbag2_to_semantic2d_baseline import (
    load_map_info,
    nearest_by_time,
    read_bag,
    resample_scan,
    semantic_for_scan,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--semantic-viz", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--target-points", default=1081, type=int)
    parser.add_argument("--sample-every", default=5, type=int)
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

    if args.semantic_viz and args.semantic_viz.exists():
        base = Image.open(args.semantic_viz).convert("RGB")
    else:
        base_arr = np.zeros((height, width, 3), dtype=np.uint8)
        base_arr[label_img == 0] = (35, 35, 35)
        base_arr[label_img != 0] = (185, 185, 185)
        base = Image.fromarray(base_arr)

    scans, odoms, _cmds, _topic_types, topic_counts = read_bag(
        args.bag, args.scan_topic, args.odom_topic, args.cmd_vel_topic
    )

    other_points = []
    non_other_points = []
    counts = Counter()
    label_hist = Counter()

    for scan_idx, (stamp_ns, scan_msg) in enumerate(scans):
        if args.sample_every > 1 and scan_idx % args.sample_every != 0:
            continue
        odom_match = nearest_by_time(odoms, stamp_ns)
        if odom_match is None:
            continue

        x, y, yaw, _odom_lin, _odom_ang = odom_match[1]
        ranges, _intensity, angles = resample_scan(scan_msg, args.target_points, None, None)
        finite = np.isfinite(ranges)
        world_x = x + ranges * np.cos(yaw + angles)
        world_y = y + ranges * np.sin(yaw + angles)
        cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
        rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
        in_map = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height) & finite

        labels = semantic_for_scan(ranges, angles, (x, y, yaw), label_img, resolution, origin_x, origin_y)
        label_hist.update(int(v) for v in labels.tolist())
        counts["total_beams"] += len(ranges)
        counts["finite_beams"] += int(finite.sum())
        counts["in_map_finite"] += int(in_map.sum())
        counts["out_of_map_or_invalid"] += int((~in_map).sum())
        counts["other_all"] += int((labels == 0).sum())
        counts["other_in_map_finite"] += int(((labels == 0) & in_map).sum())
        counts["non_other_in_map_finite"] += int(((labels != 0) & in_map).sum())

        other_mask = (labels == 0) & in_map
        non_other_mask = (labels != 0) & in_map
        if other_mask.any():
            other_points.append(np.column_stack([cols[other_mask], rows[other_mask]]))
        if non_other_mask.any():
            non_other_points.append(np.column_stack([cols[non_other_mask], rows[non_other_mask]]))

    def stack_and_limit(chunks):
        if not chunks:
            return np.empty((0, 2), dtype=np.int64)
        pts = np.vstack(chunks)
        if len(pts) > args.max_points_per_class:
            rng = np.random.default_rng(0)
            pts = pts[rng.choice(len(pts), args.max_points_per_class, replace=False)]
        return pts

    other_pts = stack_and_limit(other_points)
    non_other_pts = stack_and_limit(non_other_points)

    def draw(path: Path, include_non_other: bool):
        fig, ax = plt.subplots(figsize=(12, 9), dpi=180)
        ax.imshow(base)
        if include_non_other and len(non_other_pts):
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

    draw(args.output_dir / "semantic_projection_overlay_test.png", include_non_other=True)
    draw(args.output_dir / "semantic_projection_other_only_test.png", include_non_other=False)

    stats = {
        "bag": str(args.bag),
        "sample_every": args.sample_every,
        "topic_counts": dict(topic_counts),
        "counts": {k: int(v) for k, v in counts.items()},
        "label_histogram_sampled": {str(k): int(v) for k, v in sorted(label_hist.items())},
    }
    (args.output_dir / "semantic_projection_overlay_stats.json").write_text(
        json.dumps(stats, indent=2), encoding="utf-8"
    )
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
