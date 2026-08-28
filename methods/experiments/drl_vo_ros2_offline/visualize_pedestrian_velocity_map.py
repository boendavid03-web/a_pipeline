#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--animation-fps, --animation-half-window, --animation-stride, --center-frame, --map-yaml, --output-root, --samples
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPZ, PNG, TXT
# 可能使用的关键环境变量：PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/visualize_pedestrian_velocity_map.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 13:18:33.448505028 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.568386320 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/visualize_pedestrian_velocity_map.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Visualize the pedestrian position/velocity input used by offline DRL-VO."""

from __future__ import annotations

import argparse
import json
import math
import platform
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

from observation_adapter import pedestrian_velocity_map, rotate_map_to_base


@dataclass(frozen=True)
class Frame:
    path: Path
    timestamp_ns: int
    pose: np.ndarray
    ids: np.ndarray
    positions_map: np.ndarray
    velocities_map: np.ndarray
    positions_base: np.ndarray
    velocities_base: np.ndarray
    visible: np.ndarray
    nearest_visible_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--center-frame", type=int)
    parser.add_argument("--animation-half-window", type=int, default=150)
    parser.add_argument("--animation-stride", type=int, default=5)
    parser.add_argument("--animation-fps", type=int, default=5)
    return parser.parse_args()


def load_frame(path: Path) -> Frame:
    with np.load(path, allow_pickle=False) as sample:
        pose = sample["position"].astype(np.float32)
        ids = sample["pedestrian_ids"].astype(np.str_)
        positions_map = sample["pedestrian_xy_map"].astype(np.float32)
        velocities_map = sample["pedestrian_velocity_map"].astype(np.float32)
        timestamp_ns = int(sample["scan_01_stamp_ns"])
    positions_base = rotate_map_to_base(positions_map - pose[:2], float(pose[2]))
    velocities_base = rotate_map_to_base(velocities_map, float(pose[2]))
    visible = (
        (positions_base[:, 0] >= 0.0)
        & (positions_base[:, 0] <= 20.0)
        & (np.abs(positions_base[:, 1]) <= 10.0)
    )
    nearest = (
        float(np.min(np.linalg.norm(positions_base[visible], axis=1)))
        if np.any(visible)
        else math.inf
    )
    return Frame(
        path=path,
        timestamp_ns=timestamp_ns,
        pose=pose,
        ids=ids,
        positions_map=positions_map,
        velocities_map=velocities_map,
        positions_base=positions_base,
        velocities_base=velocities_base,
        visible=visible,
        nearest_visible_m=nearest,
    )


def load_map(map_yaml: Path | None):
    if map_yaml is None:
        return None
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.asarray(Image.open(image_path).convert("L"))
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    extent = (
        origin_x,
        origin_x + image.shape[1] * resolution,
        origin_y,
        origin_y + image.shape[0] * resolution,
    )
    return image, extent


def draw_world(axis, frame: Frame, map_data) -> None:
    if map_data is not None:
        image, extent = map_data
        axis.imshow(
            image,
            cmap="gray",
            origin="upper",
            extent=extent,
            vmin=0,
            vmax=255,
            alpha=0.7,
        )
    axis.scatter(
        frame.positions_map[:, 0],
        frame.positions_map[:, 1],
        s=55,
        c="#e6550d",
        edgecolors="black",
        linewidths=0.7,
        zorder=3,
        label="pedestrian",
    )
    axis.quiver(
        frame.positions_map[:, 0],
        frame.positions_map[:, 1],
        frame.velocities_map[:, 0],
        frame.velocities_map[:, 1],
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#d62728",
        width=0.006,
        zorder=4,
    )
    for pedestrian_id, position in zip(frame.ids, frame.positions_map):
        axis.annotate(
            str(pedestrian_id),
            position,
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7,
        )
    x, y, yaw = (float(value) for value in frame.pose)
    axis.scatter([x], [y], marker="*", s=150, c="#1f77b4", zorder=5, label="robot")
    axis.arrow(
        x,
        y,
        math.cos(yaw),
        math.sin(yaw),
        width=0.025,
        head_width=0.25,
        color="#1f77b4",
        length_includes_head=True,
        zorder=5,
    )
    axis.set_title("Map frame: positions and velocities")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.set_aspect("equal")
    axis.grid(alpha=0.2)
    axis.legend(loc="best", fontsize=8)


def draw_local(axis, frame: Frame, title: str) -> None:
    visible_positions = frame.positions_base[frame.visible]
    visible_velocities = frame.velocities_base[frame.visible]
    visible_ids = frame.ids[frame.visible]
    axis.scatter(
        visible_positions[:, 1],
        visible_positions[:, 0],
        s=60,
        c=np.linalg.norm(visible_velocities, axis=1),
        cmap="viridis",
        vmin=0.0,
        vmax=1.5,
        edgecolors="black",
        linewidths=0.7,
        zorder=3,
    )
    axis.quiver(
        visible_positions[:, 1],
        visible_positions[:, 0],
        visible_velocities[:, 1],
        visible_velocities[:, 0],
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#d62728",
        width=0.006,
        zorder=4,
    )
    for pedestrian_id, position in zip(visible_ids, visible_positions):
        axis.annotate(
            str(pedestrian_id),
            (float(position[1]), float(position[0])),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7,
        )
    axis.scatter([0.0], [0.0], marker="*", s=150, c="#1f77b4", zorder=5)
    axis.arrow(
        0.0,
        0.0,
        0.0,
        1.0,
        width=0.025,
        head_width=0.25,
        color="#1f77b4",
        length_includes_head=True,
        zorder=5,
    )
    axis.set_xlim(-10.0, 10.0)
    axis.set_ylim(0.0, 20.0)
    axis.set_aspect("equal")
    axis.set_xlabel("lateral y in base_link [m]")
    axis.set_ylabel("forward x in base_link [m]")
    axis.set_title(title)
    axis.grid(alpha=0.25)


def draw_velocity_channel(
    axis,
    channel: np.ndarray,
    visible_positions: np.ndarray,
    title: str,
) -> None:
    image = axis.imshow(
        channel[:, ::-1],
        origin="lower",
        extent=(-10.0, 10.0, 0.0, 20.0),
        cmap="coolwarm",
        vmin=-1.5,
        vmax=1.5,
        interpolation="nearest",
        aspect="equal",
    )
    if len(visible_positions):
        axis.scatter(
            visible_positions[:, 1],
            visible_positions[:, 0],
            s=24,
            facecolors="none",
            edgecolors="black",
            linewidths=0.7,
        )
    axis.set_xlabel("lateral y [m]")
    axis.set_ylabel("forward x [m]")
    axis.set_title(title)
    return image


def write_overview(
    output_path: Path,
    frame: Frame,
    map_data,
    relative_time_s: float,
) -> dict[str, object]:
    velocity_grid, nearest = pedestrian_velocity_map(
        frame.positions_map,
        frame.velocities_map,
        frame.pose,
    )
    figure, axes = plt.subplots(2, 2, figsize=(14, 11), constrained_layout=True)
    draw_world(axes[0, 0], frame, map_data)
    draw_local(
        axes[0, 1],
        frame,
        (
            f"DRL-VO field of view | visible={int(np.sum(frame.visible))}/{len(frame.ids)} "
            f"| nearest={nearest:.2f} m"
        ),
    )
    visible_positions = frame.positions_base[frame.visible]
    image_x = draw_velocity_channel(
        axes[1, 0],
        velocity_grid[0],
        visible_positions,
        "DRL-VO channel 0: pedestrian forward velocity vx",
    )
    image_y = draw_velocity_channel(
        axes[1, 1],
        velocity_grid[1],
        visible_positions,
        "DRL-VO channel 1: pedestrian lateral velocity vy",
    )
    figure.colorbar(image_x, ax=axes[1, 0], label="velocity [m/s]", shrink=0.8)
    figure.colorbar(image_y, ax=axes[1, 1], label="velocity [m/s]", shrink=0.8)
    figure.suptitle(
        f"Pedestrian position/velocity input | frame={frame.path.stem} "
        f"| t={relative_time_s:.2f} s",
        fontsize=14,
    )
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    nonzero_cells = int(
        np.count_nonzero(np.any(np.abs(velocity_grid) > 1e-8, axis=0))
    )
    return {
        "velocity_grid_shape": list(velocity_grid.shape),
        "visible_pedestrians": int(np.sum(frame.visible)),
        "nearest_visible_m": frame.nearest_visible_m,
        "nonzero_velocity_cells": nonzero_cells,
    }


def write_animation(
    output_path: Path,
    frames: list[Frame],
    selected_indices: list[int],
    first_timestamp_ns: int,
    fps: int,
) -> None:
    figure, axis = plt.subplots(figsize=(7, 7), constrained_layout=True)

    def update(animation_index: int):
        axis.clear()
        frame_index = selected_indices[animation_index]
        frame = frames[frame_index]
        elapsed = (frame.timestamp_ns - first_timestamp_ns) / 1e9
        draw_local(
            axis,
            frame,
            (
                f"Pedestrian velocity map | frame={frame_index} | t={elapsed:.2f}s\n"
                f"visible={int(np.sum(frame.visible))}/{len(frame.ids)}, "
                f"nearest={frame.nearest_visible_m:.2f}m"
            ),
        )
        return ()

    movie = animation.FuncAnimation(
        figure,
        update,
        frames=len(selected_indices),
        interval=1000 / fps,
        blit=False,
    )
    movie.save(output_path, writer=animation.PillowWriter(fps=fps), dpi=110)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    if args.animation_half_window < 0:
        raise ValueError("--animation-half-window must be non-negative")
    if args.animation_stride <= 0 or args.animation_fps <= 0:
        raise ValueError("animation stride and fps must be positive")
    sample_paths = sorted(args.samples.glob("*.npz"))
    if not sample_paths:
        raise FileNotFoundError(f"No NPZ samples under {args.samples}")
    frames = [load_frame(path) for path in sample_paths]
    if args.center_frame is None:
        finite_nearest = np.asarray(
            [frame.nearest_visible_m for frame in frames],
            dtype=np.float64,
        )
        if not np.isfinite(finite_nearest).any():
            raise RuntimeError("No pedestrians enter the DRL-VO field of view")
        center_index = int(np.argmin(finite_nearest))
    else:
        center_index = args.center_frame
    if not 0 <= center_index < len(frames):
        raise IndexError(f"center frame {center_index} outside 0..{len(frames) - 1}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    overview_path = output_dir / "pedestrian_velocity_map_overview.png"
    animation_path = output_dir / "pedestrian_velocity_map_animation.gif"
    map_data = load_map(args.map_yaml)
    relative_time_s = (
        frames[center_index].timestamp_ns - frames[0].timestamp_ns
    ) / 1e9
    overview = write_overview(
        overview_path,
        frames[center_index],
        map_data,
        relative_time_s,
    )

    animation_start = max(0, center_index - args.animation_half_window)
    animation_end = min(
        len(frames) - 1,
        center_index + args.animation_half_window,
    )
    animation_indices = list(
        range(animation_start, animation_end + 1, args.animation_stride)
    )
    write_animation(
        animation_path,
        frames,
        animation_indices,
        frames[0].timestamp_ns,
        args.animation_fps,
    )

    visible_counts = np.asarray(
        [np.sum(frame.visible) for frame in frames],
        dtype=np.int64,
    )
    pedestrian_speeds = np.concatenate(
        [np.linalg.norm(frame.velocities_map, axis=1) for frame in frames]
    )
    summary = {
        "status": "PASS",
        "input": {
            "samples": str(args.samples.resolve()),
            "map_yaml": str(args.map_yaml.resolve()) if args.map_yaml else None,
            "read_only": True,
        },
        "frames": len(frames),
        "selected_frame": center_index,
        "selected_timestamp_ns": frames[center_index].timestamp_ns,
        "selected_relative_time_s": relative_time_s,
        "selected_pedestrian_ids": frames[center_index].ids.tolist(),
        "overview": overview,
        "all_frames": {
            "visible_count_min": int(np.min(visible_counts)),
            "visible_count_max": int(np.max(visible_counts)),
            "visible_count_mean": float(np.mean(visible_counts)),
            "pedestrian_speed_min_m_s": float(np.min(pedestrian_speeds)),
            "pedestrian_speed_max_m_s": float(np.max(pedestrian_speeds)),
            "pedestrian_speed_mean_m_s": float(np.mean(pedestrian_speeds)),
        },
        "animation": {
            "start_frame": animation_start,
            "end_frame": animation_end,
            "stride": args.animation_stride,
            "rendered_frames": len(animation_indices),
            "fps": args.animation_fps,
        },
        "outputs": {
            "overview_png": str(overview_path),
            "animation_gif": str(animation_path),
        },
        "safety": {
            "ros_used": False,
            "topics_published": [],
            "source_data_modified": False,
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "run_command.txt").write_text(
        " ".join([sys.executable, *sys.argv]) + "\n",
        encoding="utf-8",
    )
    environment = {
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "matplotlib": matplotlib.__version__,
    }
    (output_dir / "environment.json").write_text(
        json.dumps(environment, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output_dir": str(output_dir), **summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
