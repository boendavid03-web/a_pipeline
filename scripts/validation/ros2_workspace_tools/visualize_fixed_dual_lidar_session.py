#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--animation-width, --dpi, --fps, --frame-index, --max-frames, --no-mp4, --output-dir, --session, --stride
# 代码中检测到的 ROS 2 话题/路径字符串：/scan_01, /scan_02
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPZ, PNG, TXT, YAML
# 可能使用的关键环境变量：ADAPTIVE, E402, LANCZOS, PASS, REQUIRED_ARRAYS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_fixed_dual_lidar_session.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 08:25:34.484384754 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.849391644 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07d_visualize_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07d_visualize_fixed_dual_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_fixed_dual_lidar_session.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07d_visualize_fixed_dual_lidar.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜visualize_fixed_dual_lidar_session.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Visualize and summarize one fixed dual-LiDAR NPZ session.

This tool reads converted artifacts only.  It never modifies the source session,
and it refuses to write into an existing output directory.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402
from PIL import Image  # noqa: E402


REQUIRED_ARRAYS = {
    "raw_ranges",
    "raw_angles_sensor",
    "points_x_base",
    "points_y_base",
    "range_valid_mask",
    "self_mask",
    "valid_mask",
    "semantic_label",
    "source_sensor",
    "raw_beam_index",
    "position",
    "velocity",
    "cmd_velocity",
    "sub_goal_local",
    "scan_01_stamp_ns",
    "scan_02_stamp_ns",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--max-frames",
        type=int,
        default=60,
        help="Maximum PNG/animation frames; the quality report still covers the full session.",
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument(
        "--frame-index",
        type=int,
        help="Render only this zero-based source frame (still report the full session).",
    )
    parser.add_argument("--animation-width", type=int, default=960)
    parser.add_argument("--dpi", type=int, default=100)
    parser.add_argument("--no-mp4", action="store_true")
    return parser.parse_args()


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve_recorded_path(value, session):
    if not value:
        return None
    path = Path(value).expanduser()
    return path if path.is_absolute() else session / path


def read_label_names(session, metadata, map_yaml):
    candidates = [
        session / "label_names.txt",
        resolve_recorded_path(metadata.get("label_names_source"), session),
        map_yaml.parent / "label_names.txt",
    ]
    for path in candidates:
        if path is not None and path.is_file():
            names = [
                line.strip()
                for line in path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            if not names:
                raise ValueError(f"Empty label name file: {path}")
            recorded = metadata.get("label_names")
            if recorded is not None and names != recorded:
                raise ValueError(
                    f"{path} differs from metadata.json label_names; refusing ambiguous colors"
                )
            return names, path.resolve()
    raise FileNotFoundError(
        "No label_names.txt found in the session, at metadata.label_names_source, "
        "or next to map.yaml"
    )


def load_map_assets(session, metadata):
    map_yaml = resolve_recorded_path(metadata.get("map_yaml"), session)
    semantic_path = resolve_recorded_path(metadata.get("semantic_label"), session)
    if map_yaml is None or not map_yaml.is_file():
        raise FileNotFoundError(f"metadata map_yaml is missing: {map_yaml}")
    if semantic_path is None or not semantic_path.is_file():
        raise FileNotFoundError(f"metadata semantic_label is missing: {semantic_path}")
    with map_yaml.open("r", encoding="utf-8") as stream:
        map_config = yaml.safe_load(stream)
    occupancy_path = Path(map_config["image"]).expanduser()
    if not occupancy_path.is_absolute():
        occupancy_path = map_yaml.parent / occupancy_path
    if not occupancy_path.is_file():
        raise FileNotFoundError(f"map.yaml occupancy image is missing: {occupancy_path}")
    resolution = float(map_config["resolution"])
    origin = tuple(float(value) for value in map_config["origin"])
    if resolution <= 0 or len(origin) != 3:
        raise ValueError(f"Invalid map resolution/origin in {map_yaml}")
    occupancy = np.asarray(Image.open(occupancy_path).convert("L"))
    semantic = np.asarray(Image.open(semantic_path))
    if semantic.ndim == 3:
        semantic = semantic[:, :, 0]
    semantic = semantic.astype(np.int64)
    if occupancy.shape != semantic.shape:
        raise ValueError(
            f"Occupancy shape {occupancy.shape} != semantic shape {semantic.shape}"
        )
    if int(map_config.get("negate", 0)):
        occupancy = 255 - occupancy
    return {
        "yaml": map_yaml.resolve(),
        "occupancy_path": occupancy_path.resolve(),
        "semantic_path": semantic_path.resolve(),
        "occupancy": occupancy,
        "semantic": semantic,
        "resolution": resolution,
        "origin": origin,
    }


def class_colors(count):
    cmap = plt.get_cmap("tab20", max(count, 1))
    return [cmap(index) for index in range(count)]


def validate_first_sample(path, metadata, label_names):
    with np.load(path, allow_pickle=False) as data:
        missing = sorted(REQUIRED_ARRAYS - set(data.files))
        if missing:
            raise ValueError(f"{path} is missing arrays: {', '.join(missing)}")
        slots = int(np.asarray(data["semantic_label"]).size)
        fields = (
            "raw_ranges",
            "raw_angles_sensor",
            "points_x_base",
            "points_y_base",
            "range_valid_mask",
            "self_mask",
            "valid_mask",
            "source_sensor",
            "raw_beam_index",
        )
        wrong = [name for name in fields if np.asarray(data[name]).shape != (slots,)]
        if wrong:
            raise ValueError(f"{path} has non-slot-shaped arrays: {', '.join(wrong)}")
        source_sensor = np.asarray(data["source_sensor"], dtype=np.int64)
        beam_index = np.asarray(data["raw_beam_index"], dtype=np.int64)
        source_ids = [int(value) for value in np.unique(source_sensor)]
        if len(source_ids) != 2:
            raise ValueError(f"Expected exactly two source_sensor IDs, got {source_ids}")
        sensor_counts = [int(np.count_nonzero(source_sensor == value)) for value in source_ids]
        expected = [metadata.get("samples_01"), metadata.get("samples_02")]
        if all(value is not None for value in expected) and sensor_counts != [int(v) for v in expected]:
            raise ValueError(
                f"NPZ sensor counts {sensor_counts} differ from metadata {expected}"
            )
        if metadata.get("total_slots") is not None and slots != int(metadata["total_slots"]):
            raise ValueError(
                f"NPZ slot count {slots} differs from metadata total_slots={metadata['total_slots']}"
            )
        for source_id, count in zip(source_ids, sensor_counts):
            actual = beam_index[source_sensor == source_id]
            if not np.array_equal(actual, np.arange(count, dtype=actual.dtype)):
                raise ValueError(
                    f"source_sensor={source_id} raw_beam_index is not contiguous 0..{count - 1}"
                )
        labels = np.asarray(data["semantic_label"], dtype=np.int64)
        bad_labels = np.unique(labels[(labels >= len(label_names))])
        if bad_labels.size:
            raise ValueError(f"Labels outside label_names.txt range: {bad_labels.tolist()}")
    return slots, source_ids, sensor_counts, source_sensor.copy(), beam_index.copy()


def sensor_descriptions(metadata, source_ids, sensor_counts):
    topics = [metadata.get("scan_01_topic", "/scan_01"), metadata.get("scan_02_topic", "/scan_02")]
    layouts = [metadata.get("scan_01_layout", {}), metadata.get("scan_02_layout", {})]
    descriptions = []
    for source_id, count, topic, layout in zip(source_ids, sensor_counts, topics, layouts):
        descriptions.append(
            {
                "source_id": source_id,
                "count": count,
                "topic": topic,
                "range_max": float(layout.get("range_max", 1.0)),
                "frame_id": layout.get("frame_id", "unknown"),
            }
        )
    return descriptions


def sample_metrics(path, expected_slots, fixed_sources, fixed_beams, person_id, class_count):
    with np.load(path, allow_pickle=False) as data:
        missing = sorted(REQUIRED_ARRAYS - set(data.files))
        if missing:
            raise ValueError(f"{path} is missing arrays: {', '.join(missing)}")
        labels = np.asarray(data["semantic_label"], dtype=np.int64)
        range_valid = np.asarray(data["range_valid_mask"], dtype=bool)
        self_mask = np.asarray(data["self_mask"], dtype=bool)
        valid = np.asarray(data["valid_mask"], dtype=bool)
        sources = np.asarray(data["source_sensor"], dtype=np.int64)
        beams = np.asarray(data["raw_beam_index"], dtype=np.int64)
        if any(array.shape != (expected_slots,) for array in (labels, range_valid, self_mask, valid, sources, beams)):
            raise ValueError(f"Slot array shape changed in {path}")
        if not np.array_equal(sources, fixed_sources) or not np.array_equal(beams, fixed_beams):
            raise ValueError(f"Fixed source_sensor/raw_beam_index identity changed in {path}")
        bad_labels = np.unique(labels[labels >= class_count])
        if bad_labels.size:
            raise ValueError(f"Labels outside label_names.txt range in {path}: {bad_labels.tolist()}")
        expected_valid = range_valid & ~self_mask
        invariant_errors = int(np.count_nonzero(valid != expected_valid))
        class_hist = Counter(int(value) for value in labels[valid & (labels >= 0)])
        cmd = np.asarray(data["cmd_velocity"], dtype=float).reshape(-1)
        odom = np.asarray(data["velocity"], dtype=float).reshape(-1)
        position = np.asarray(data["position"], dtype=float).reshape(-1)
        subgoal = np.asarray(data["sub_goal_local"], dtype=float).reshape(-1)
        if cmd.size != 3 or odom.size != 2 or position.size != 3 or subgoal.size != 2:
            raise ValueError(f"Pose/velocity/sub-goal shape changed in {path}")
        return {
            "source_file": path.name,
            "range_valid_count": int(range_valid.sum()),
            "range_invalid_count": int((~range_valid).sum()),
            "self_mask_count": int(self_mask.sum()),
            "valid_count": int(valid.sum()),
            "ignore_count": int((labels < 0).sum()),
            "valid_ignore_count": int((valid & (labels < 0)).sum()),
            "person_count": (
                int(np.count_nonzero(valid & (labels == person_id)))
                if person_id is not None
                else None
            ),
            "valid_mask_invariant_errors": invariant_errors,
            "class_histogram": dict(class_hist),
            "position": position.tolist(),
            "odom_velocity": odom.tolist(),
            "cmd_velocity": cmd.tolist(),
            "sub_goal_local": subgoal.tolist(),
            "scan_01_stamp_ns": int(np.asarray(data["scan_01_stamp_ns"]).item()),
            "scan_02_stamp_ns": int(np.asarray(data["scan_02_stamp_ns"]).item()),
        }


def summarize_counts(per_frame, key):
    values = np.asarray([frame[key] for frame in per_frame], dtype=float)
    return {
        "total": int(values.sum()),
        "min": int(values.min()),
        "max": int(values.max()),
        "mean": float(values.mean()),
    }


def time_axis(per_frame):
    stamps = np.asarray([frame["scan_01_stamp_ns"] for frame in per_frame], dtype=np.int64)
    return (stamps - stamps[0]).astype(np.float64) / 1e9


def plot_velocity_axis(axis, seconds, odom, cmd, current_index=None):
    axis.plot(seconds, odom[:, 0], color="tab:blue", linewidth=1.0, label="odom linear_x")
    axis.plot(seconds, cmd[:, 0], color="tab:cyan", linewidth=1.0, label="cmd linear_x")
    axis.plot(seconds, cmd[:, 1], color="tab:green", linewidth=0.9, label="cmd linear_y")
    axis.plot(seconds, odom[:, 1], color="tab:red", linewidth=1.0, label="odom angular_z")
    axis.plot(seconds, cmd[:, 2], color="tab:orange", linewidth=1.0, label="cmd angular_z")
    if current_index is not None:
        axis.axvline(seconds[current_index], color="black", linestyle="--", linewidth=0.8)
    axis.set_xlabel("session time (s)")
    axis.set_ylabel("velocity")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=7, ncol=2, loc="upper right")


def save_velocity_curves(output, per_frame, dpi):
    seconds = time_axis(per_frame)
    odom = np.asarray([frame["odom_velocity"] for frame in per_frame])
    cmd = np.asarray([frame["cmd_velocity"] for frame in per_frame])
    fig, axis = plt.subplots(figsize=(12, 5))
    plot_velocity_axis(axis, seconds, odom, cmd)
    axis.set_title("Command and odometry velocity over the full session")
    fig.tight_layout()
    path = output / "velocity_curves.png"
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path


def semantic_overlay(label_image, colors):
    overlay = np.zeros((*label_image.shape, 4), dtype=float)
    for label_id, color in enumerate(colors):
        if label_id == 0:
            continue
        mask = label_image == label_id
        overlay[mask, :3] = color[:3]
        overlay[mask, 3] = 0.42
    return overlay


def transformed_map_bounds(width, height, resolution, origin):
    local = np.asarray(
        [[0.0, 0.0], [width * resolution, 0.0], [0.0, height * resolution], [width * resolution, height * resolution]]
    )
    cosine, sine = math.cos(origin[2]), math.sin(origin[2])
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    world = local @ rotation.T + np.asarray(origin[:2])
    return world[:, 0].min(), world[:, 0].max(), world[:, 1].min(), world[:, 1].max()


def draw_map(axis, map_assets, colors, positions, sample, valid, labels, points_x, points_y):
    occupancy = map_assets["occupancy"]
    height, width = occupancy.shape
    resolution = map_assets["resolution"]
    origin = map_assets["origin"]
    transform = Affine2D().rotate(origin[2]).translate(origin[0], origin[1]) + axis.transData
    extent = (0.0, width * resolution, 0.0, height * resolution)
    axis.imshow(occupancy, cmap="gray", origin="upper", extent=extent, transform=transform, zorder=0)
    axis.imshow(
        semantic_overlay(map_assets["semantic"], colors),
        origin="upper",
        extent=extent,
        transform=transform,
        zorder=1,
    )
    axis.plot(positions[:, 0], positions[:, 1], color="deepskyblue", linewidth=1.0, alpha=0.8, label="trajectory", zorder=2)
    x, y, yaw = sample["position"]
    usable = valid & np.isfinite(points_x) & np.isfinite(points_y)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world_x = x + cosine * points_x[usable] - sine * points_y[usable]
    world_y = y + sine * points_x[usable] + cosine * points_y[usable]
    usable_labels = labels[usable]
    ignore = usable_labels < 0
    if np.any(ignore):
        axis.scatter(world_x[ignore], world_y[ignore], s=2, c="black", alpha=0.45, zorder=3)
    for label_id, color in enumerate(colors):
        mask = usable_labels == label_id
        if np.any(mask):
            axis.scatter(world_x[mask], world_y[mask], s=3, color=color, alpha=0.7, zorder=3)
    axis.scatter([x], [y], marker="o", s=38, color="red", edgecolors="white", linewidths=0.7, zorder=5)
    heading_length = max(0.45, 8.0 * resolution)
    axis.arrow(x, y, heading_length * cosine, heading_length * sine, color="red", width=0.025, head_width=0.16, length_includes_head=True, zorder=5)
    subgoal = np.asarray(sample["sub_goal_local"], dtype=float)
    goal_dx = cosine * subgoal[0] - sine * subgoal[1]
    goal_dy = sine * subgoal[0] + cosine * subgoal[1]
    axis.arrow(x, y, goal_dx, goal_dy, color="magenta", width=0.018, head_width=0.13, length_includes_head=True, zorder=5)
    min_x, max_x, min_y, max_y = transformed_map_bounds(width, height, resolution, origin)
    axis.set_xlim(min_x, max_x)
    axis.set_ylim(min_y, max_y)
    axis.set_aspect("equal", adjustable="box")
    axis.set_title("Map overlay: trajectory, heading, sub-goal and base_link endpoints")
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")


def draw_sensor_polar(axis, sample, sensor, colors, label_names):
    sources = sample["source_sensor"]
    mask = sources == sensor["source_id"]
    angles = sample["raw_angles_sensor"][mask]
    ranges = sample["raw_ranges"][mask]
    range_valid = sample["range_valid_mask"][mask]
    self_mask = sample["self_mask"][mask]
    valid = sample["valid_mask"][mask]
    labels = sample["semantic_label"][mask]
    ring = max(sensor["range_max"], float(np.nanmax(ranges[range_valid])) if np.any(range_valid) else 1.0)
    invalid = ~range_valid
    if np.any(invalid):
        axis.scatter(angles[invalid], np.full(invalid.sum(), ring), s=5, marker="x", color="0.65", alpha=0.55)
    self_radius = np.where(range_valid & np.isfinite(ranges), ranges, ring)
    if np.any(self_mask):
        axis.scatter(angles[self_mask], self_radius[self_mask], s=8, marker="x", color="darkorange", alpha=0.8)
    valid_ignore = valid & (labels < 0)
    if np.any(valid_ignore):
        axis.scatter(angles[valid_ignore], ranges[valid_ignore], s=4, color="black", alpha=0.6)
    for label_id, color in enumerate(colors):
        selected = valid & (labels == label_id)
        if np.any(selected):
            axis.scatter(angles[selected], ranges[selected], s=5, color=color, alpha=0.75)
    axis.set_ylim(0.0, ring * 1.06)
    axis.set_theta_zero_location("E")
    axis.set_title(f"{sensor['topic']} raw polar ({sensor['count']} slots)", fontsize=10)
    axis.grid(alpha=0.3)


def load_plot_sample(path):
    with np.load(path, allow_pickle=False) as data:
        return {name: np.asarray(data[name]).copy() for name in REQUIRED_ARRAYS}


def render_frame(path, sample_path, source_index, metadata, map_assets, label_names, colors,
                 sensors, positions, seconds, odom, cmd, dpi):
    sample = load_plot_sample(sample_path)
    fig = plt.figure(figsize=(15, 9))
    grid = fig.add_gridspec(2, 3, width_ratios=(1.45, 1.0, 1.0), hspace=0.28, wspace=0.28)
    map_axis = fig.add_subplot(grid[:, 0])
    polar_01 = fig.add_subplot(grid[0, 1], projection="polar")
    polar_02 = fig.add_subplot(grid[0, 2], projection="polar")
    velocity_axis = fig.add_subplot(grid[1, 1:])
    valid = sample["valid_mask"].astype(bool)
    labels = sample["semantic_label"].astype(np.int64)
    draw_map(
        map_axis,
        map_assets,
        colors,
        positions,
        sample,
        valid,
        labels,
        sample["points_x_base"],
        sample["points_y_base"],
    )
    draw_sensor_polar(polar_01, sample, sensors[0], colors, label_names)
    draw_sensor_polar(polar_02, sample, sensors[1], colors, label_names)
    plot_velocity_axis(velocity_axis, seconds, odom, cmd, current_index=source_index)
    stamp_01 = int(sample["scan_01_stamp_ns"].item())
    stamp_02 = int(sample["scan_02_stamp_ns"].item())
    cmd_now = sample["cmd_velocity"].tolist()
    odom_now = sample["velocity"].tolist()
    subgoal = sample["sub_goal_local"].tolist()
    fig.suptitle(
        f"frame={source_index} file={sample_path.name} | scan stamps={stamp_01}/{stamp_02} ns\n"
        f"cmd=[{cmd_now[0]:+.3f}, {cmd_now[1]:+.3f}, {cmd_now[2]:+.3f}] "
        f"odom=[{odom_now[0]:+.3f}, {odom_now[1]:+.3f}] "
        f"sub_goal_local=[{subgoal[0]:+.3f}, {subgoal[1]:+.3f}]",
        fontsize=11,
    )
    legend_items = [
        Line2D([], [], marker="x", linestyle="", color="0.55", label="range invalid"),
        Line2D([], [], marker="x", linestyle="", color="darkorange", label="self-mask"),
        Line2D([], [], marker="o", linestyle="", color="black", label="valid ignore"),
    ]
    legend_items.extend(
        Patch(facecolor=colors[index], label=f"{index}: {name}")
        for index, name in enumerate(label_names)
    )
    fig.legend(handles=legend_items, loc="lower center", ncol=min(10, len(legend_items)), fontsize=7)
    fig.subplots_adjust(top=0.88, bottom=0.10, left=0.055, right=0.98)
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def write_gif(frame_paths, output_path, fps, width):
    frames = []
    resampling = getattr(Image, "Resampling", Image).LANCZOS
    palette = getattr(Image, "Palette", Image).ADAPTIVE
    for path in frame_paths:
        with Image.open(path) as image:
            frame = image.convert("RGB")
            if frame.width > width:
                height = max(2, int(round(frame.height * width / frame.width)))
                frame = frame.resize((width, height), resampling)
            frames.append(frame.convert("P", palette=palette, colors=256))
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=max(1, int(round(1000.0 / fps))),
        loop=0,
        disposal=2,
    )


def write_mp4(frames_dir, output_path, fps):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return {"status": "skipped", "reason": "ffmpeg not found"}
    command = [
        ffmpeg,
        "-loglevel",
        "error",
        "-n",
        "-framerate",
        str(fps),
        "-i",
        str(frames_dir / "frame_%04d.png"),
        "-vf",
        "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        return {
            "status": "failed",
            "reason": result.stderr.strip() or f"ffmpeg exited {result.returncode}",
            "command": command,
        }
    return {"status": "written", "path": str(output_path), "command": command}


def write_report_markdown(path, report, label_names):
    summary = report["quality_summary"]
    lines = [
        "# Fixed dual-LiDAR visualization report",
        "",
        f"- Session: `{report['session']['name']}`",
        f"- Source frames: {report['session']['frame_count']}",
        f"- Slots per frame: {report['session']['total_slots']} ({report['session']['sensor_slot_counts']})",
        f"- Rendered frames: {len(report['rendering']['source_frame_indices'])}",
        f"- Frame selection: stride={report['rendering']['stride']}, max_frames={report['rendering']['max_frames']}",
        f"- Self-mask mode: `{report['session']['self_mask_mode']}`",
        "",
        "## Quality counts",
        "",
        "| Metric | Total | Min/frame | Mean/frame | Max/frame |",
        "|---|---:|---:|---:|---:|",
    ]
    for key in ("valid_count", "self_mask_count", "range_invalid_count", "ignore_count", "valid_ignore_count", "person_count"):
        if key not in summary:
            continue
        values = summary[key]
        lines.append(
            f"| {key} | {values['total']} | {values['min']} | {values['mean']:.2f} | {values['max']} |"
        )
    lines.extend(
        [
            "",
            "`ignore_count` counts every negative semantic label; `valid_ignore_count` is the subset that remains geometrically valid.",
            "",
            "## Valid semantic class histogram",
            "",
            "| ID | Class | Count |",
            "|---:|---|---:|",
        ]
    )
    histogram = report["class_histogram"]
    for label_id, name in enumerate(label_names):
        lines.append(f"| {label_id} | {name.replace('|', '/')} | {histogram[str(label_id)]['count']} |")
    lines.extend(
        [
            "",
            "## Person-label interpretation",
            "",
            report["person_label_interpretation"],
            "",
            "## Outputs",
            "",
            f"- PNG frames: `frames/`",
            f"- GIF: `{report['outputs']['gif']}`",
            f"- MP4: `{report['outputs']['mp4']['status']}`",
            "- Full-session velocity curves: `velocity_curves.png`",
            "- Machine-readable per-frame metrics: `report.json`",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    if args.stride <= 0 or args.max_frames <= 0 or args.fps <= 0:
        raise ValueError("--stride, --max-frames and --fps must be positive")
    if args.animation_width <= 0 or args.dpi <= 0:
        raise ValueError("--animation-width and --dpi must be positive")
    session = args.session.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not session.is_dir():
        raise NotADirectoryError(session)
    metadata_path = session / "metadata.json"
    samples_dir = session / "samples"
    if not metadata_path.is_file() or not samples_dir.is_dir():
        raise FileNotFoundError(f"Expected metadata.json and samples/ under {session}")
    sample_paths = sorted(samples_dir.glob("*.npz"))
    if not sample_paths:
        raise FileNotFoundError(f"No samples/*.npz under {session}")
    if output.exists():
        raise FileExistsError(f"Output already exists; refusing to overwrite: {output}")

    metadata = read_json(metadata_path)
    map_assets = load_map_assets(session, metadata)
    label_names, label_names_path = read_label_names(session, metadata, map_assets["yaml"])
    slots, source_ids, sensor_counts, fixed_sources, fixed_beams = validate_first_sample(
        sample_paths[0], metadata, label_names
    )
    sensors = sensor_descriptions(metadata, source_ids, sensor_counts)
    person_id = next(
        (index for index, name in enumerate(label_names) if name.casefold() == "person"),
        None,
    )
    recorded_person_id = metadata.get("person_label_id")
    if recorded_person_id is not None and int(recorded_person_id) != person_id:
        raise ValueError(
            f"metadata person_label_id={recorded_person_id} differs from label_names.txt Person ID={person_id}"
        )

    per_frame = []
    class_histogram = Counter()
    for index, path in enumerate(sample_paths):
        metrics = sample_metrics(
            path, slots, fixed_sources, fixed_beams, person_id, len(label_names)
        )
        metrics["frame_index"] = index
        class_histogram.update(metrics.pop("class_histogram"))
        per_frame.append(metrics)
    invariant_errors = sum(frame["valid_mask_invariant_errors"] for frame in per_frame)
    if invariant_errors:
        raise ValueError(
            f"valid_mask != range_valid_mask & ~self_mask at {invariant_errors} slots"
        )

    if args.frame_index is not None:
        if not 0 <= args.frame_index < len(sample_paths):
            raise IndexError(
                f"--frame-index {args.frame_index} outside 0..{len(sample_paths) - 1}"
            )
        selected_indices = [args.frame_index]
    else:
        selected_indices = list(range(0, len(sample_paths), args.stride))[: args.max_frames]

    output.mkdir(parents=True, exist_ok=False)
    frames_dir = output / "frames"
    frames_dir.mkdir()
    colors = class_colors(len(label_names))
    positions = np.asarray([frame["position"] for frame in per_frame])
    seconds = time_axis(per_frame)
    odom = np.asarray([frame["odom_velocity"] for frame in per_frame])
    cmd = np.asarray([frame["cmd_velocity"] for frame in per_frame])
    save_velocity_curves(output, per_frame, args.dpi)

    rendered = []
    for sequence, source_index in enumerate(selected_indices):
        frame_path = frames_dir / f"frame_{sequence:04d}.png"
        render_frame(
            frame_path,
            sample_paths[source_index],
            source_index,
            metadata,
            map_assets,
            label_names,
            colors,
            sensors,
            positions,
            seconds,
            odom,
            cmd,
            args.dpi,
        )
        rendered.append(frame_path)

    gif_path = output / "preview.gif"
    write_gif(rendered, gif_path, args.fps, args.animation_width)
    if args.no_mp4:
        mp4_result = {"status": "skipped", "reason": "--no-mp4"}
    else:
        mp4_result = write_mp4(frames_dir, output / "overview.mp4", args.fps)

    quality_keys = [
        "range_valid_count",
        "range_invalid_count",
        "self_mask_count",
        "valid_count",
        "ignore_count",
        "valid_ignore_count",
    ]
    if person_id is not None:
        quality_keys.append("person_count")
    person_mode = metadata.get("person_label_mode")
    if person_id is not None and person_mode == "dynamic":
        person_note = (
            f"Class {person_id} ({label_names[person_id]}) is produced by the recorded dynamic rule: "
            "free-space unlabeled endpoints are assigned Person. It is not per-instance manual ground truth."
        )
    elif person_id is not None:
        person_note = (
            f"Class {person_id} ({label_names[person_id]}) exists in label_names.txt; "
            f"metadata person_label_mode is {person_mode!r}."
        )
    else:
        person_note = "No Person class exists in the file-defined class table."

    report = {
        "format": "fixed-dual-lidar-visualization-report-v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "session": {
            "name": metadata.get("session_name", session.name),
            "path": str(session),
            "format": metadata.get("format"),
            "frame_count": len(sample_paths),
            "total_slots": slots,
            "sensor_ids": source_ids,
            "sensor_slot_counts": sensor_counts,
            "sensor_topics": [sensor["topic"] for sensor in sensors],
            "self_mask_mode": metadata.get("self_mask_mode"),
            "person_label_mode": person_mode,
        },
        "sources": {
            "metadata": str(metadata_path.resolve()),
            "label_names": str(label_names_path),
            "map_yaml": str(map_assets["yaml"]),
            "occupancy_image": str(map_assets["occupancy_path"]),
            "semantic_label": str(map_assets["semantic_path"]),
        },
        "rendering": {
            "stride": args.stride,
            "max_frames": args.max_frames,
            "fps": args.fps,
            "source_frame_indices": selected_indices,
        },
        "quality_summary": {key: summarize_counts(per_frame, key) for key in quality_keys},
        "class_histogram": {
            str(index): {"name": name, "count": int(class_histogram.get(index, 0))}
            for index, name in enumerate(label_names)
        },
        "person_label_interpretation": person_note,
        "outputs": {
            "frames": "frames/",
            "gif": gif_path.name,
            "mp4": mp4_result,
            "velocity_curves": "velocity_curves.png",
        },
        "per_frame": per_frame,
    }
    report_path = output / "report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_report_markdown(output / "report.md", report, label_names)
    print(f"PASS: visualized {len(rendered)} of {len(sample_paths)} frames")
    print(f"Output: {output}")
    print(f"GIF: {gif_path}")
    print(f"MP4: {mp4_result['status']}")
    print(f"Report: {report_path}")


if __name__ == "__main__":
    main()
