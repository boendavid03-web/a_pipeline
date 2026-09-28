#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, TXT
# 可能使用的关键环境变量：PERSON_LABEL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/dataset.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:36:31.422662451 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.568386320 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/dataset.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Dataset and target construction for temporal dual-LiDAR pedestrian detection.

Inference inputs are deliberately limited to LiDAR ranges/angles, validity masks,
relative timestamps, and robot poses used for ego-motion compensation.  Semantic
and pedestrian ground truth are read only when ``build_targets=True``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset


PERSON_LABEL = 6


@dataclass(frozen=True)
class BEVSpec:
    extent_m: float = 8.0
    resolution_m: float = 0.10

    def __post_init__(self) -> None:
        if self.extent_m <= 0.0 or self.resolution_m <= 0.0:
            raise ValueError("BEV extent and resolution must be positive")
        cells = 2.0 * self.extent_m / self.resolution_m
        if not math.isclose(cells, round(cells), rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("2 * extent_m must be divisible by resolution_m")

    @property
    def size(self) -> int:
        return int(round(2.0 * self.extent_m / self.resolution_m))

    def metric_to_grid(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        points = np.asarray(xy, dtype=np.float64).reshape((-1, 2))
        grid_x = (points[:, 0] + self.extent_m) / self.resolution_m
        grid_y = (points[:, 1] + self.extent_m) / self.resolution_m
        return grid_x, grid_y

    def grid_to_metric(
        self, grid_x: np.ndarray, grid_y: np.ndarray
    ) -> np.ndarray:
        grid_x = np.asarray(grid_x, dtype=np.float64)
        grid_y = np.asarray(grid_y, dtype=np.float64)
        x = grid_x * self.resolution_m - self.extent_m
        y = grid_y * self.resolution_m - self.extent_m
        return np.stack((x, y), axis=-1)


@dataclass(frozen=True)
class _FrameRef:
    session_dir: Path
    session_name: str
    split: str
    episode_id: int
    name: str
    scan_timestamp_ns: int


def _rotation(yaw: float) -> np.ndarray:
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    return np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float64)


def base_to_map(points_base: np.ndarray, robot_pose_map: np.ndarray) -> np.ndarray:
    points = np.asarray(points_base, dtype=np.float64).reshape((-1, 2))
    pose = np.asarray(robot_pose_map, dtype=np.float64).reshape(3)
    return points @ _rotation(pose[2]).T + pose[:2]


def map_to_base(points_map: np.ndarray, robot_pose_map: np.ndarray) -> np.ndarray:
    points = np.asarray(points_map, dtype=np.float64).reshape((-1, 2))
    pose = np.asarray(robot_pose_map, dtype=np.float64).reshape(3)
    return (points - pose[:2]) @ _rotation(pose[2])


def vector_map_to_base(vectors_map: np.ndarray, robot_yaw_map: float) -> np.ndarray:
    vectors = np.asarray(vectors_map, dtype=np.float64).reshape((-1, 2))
    return vectors @ _rotation(float(robot_yaw_map))


def _draw_gaussian(
    heatmap: np.ndarray,
    center_x: float,
    center_y: float,
    sigma_cells: float,
) -> None:
    radius = max(1, int(math.ceil(3.0 * sigma_cells)))
    x0 = max(0, int(math.floor(center_x)) - radius)
    x1 = min(heatmap.shape[1] - 1, int(math.floor(center_x)) + radius)
    y0 = max(0, int(math.floor(center_y)) - radius)
    y1 = min(heatmap.shape[0] - 1, int(math.floor(center_y)) + radius)
    if x1 < x0 or y1 < y0:
        return
    xs = np.arange(x0, x1 + 1, dtype=np.float32)
    ys = np.arange(y0, y1 + 1, dtype=np.float32)
    gaussian = np.exp(
        -(
            np.square(xs[None, :] - center_x)
            + np.square(ys[:, None] - center_y)
        )
        / (2.0 * sigma_cells * sigma_cells)
    )
    patch = heatmap[y0 : y1 + 1, x0 : x1 + 1]
    np.maximum(patch, gaussian, out=patch)


def encode_temporal_bev(channels: np.ndarray, mode: str) -> np.ndarray:
    """Encode aligned per-frame sensor occupancy without changing its shape."""
    values = np.asarray(channels, dtype=np.float32)
    if values.ndim != 3 or values.shape[0] % 2 != 0:
        raise ValueError("temporal BEV must have shape [2*T,H,W]")
    if mode == "occupancy":
        return values
    if mode != "current_plus_deltas":
        raise ValueError(f"unsupported input encoding: {mode}")
    temporal = values.reshape((-1, 2, values.shape[1], values.shape[2]))
    encoded = np.empty_like(temporal)
    encoded[0] = temporal[-1]
    encoded[1:] = temporal[1:] - temporal[:-1]
    return encoded.reshape(values.shape)


class TemporalDualLidarDataset(Dataset):
    """Causal LiDAR windows with optional CenterNet-style supervision."""

    def __init__(
        self,
        dataset_root: Path,
        split: str,
        *,
        history_frames: int = 8,
        bev_spec: BEVSpec | None = None,
        build_targets: bool = True,
        min_person_points: int = 1,
        leg_match_radius_m: float = 0.12,
        target_sigma_cells: float = 1.5,
        input_encoding: str = "occupancy",
        max_samples: int | None = None,
    ) -> None:
        self.dataset_root = Path(dataset_root).expanduser().resolve()
        self.split = str(split)
        self.history_frames = int(history_frames)
        self.bev_spec = bev_spec or BEVSpec()
        self.build_targets = bool(build_targets)
        self.min_person_points = int(min_person_points)
        self.leg_match_radius_m = float(leg_match_radius_m)
        self.target_sigma_cells = float(target_sigma_cells)
        self.input_encoding = str(input_encoding)
        if self.split not in {"train", "dev", "test"}:
            raise ValueError("split must be train, dev, or test")
        if self.history_frames < 1:
            raise ValueError("history_frames must be at least one")
        if self.min_person_points < 1:
            raise ValueError("min_person_points must be at least one")
        if self.input_encoding not in {"occupancy", "current_plus_deltas"}:
            raise ValueError("unsupported input_encoding")
        if not self.dataset_root.is_dir():
            raise FileNotFoundError(f"dataset root not found: {self.dataset_root}")

        self._metadata: Dict[Path, Dict[str, object]] = {}
        self._windows: List[Tuple[_FrameRef, ...]] = []
        for session_dir in sorted(path for path in self.dataset_root.iterdir() if path.is_dir()):
            metadata_path = session_dir / "metadata.json"
            if not metadata_path.is_file():
                continue
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            if metadata.get("split_role") != self.split:
                continue
            if int(metadata.get("total_slots", 0)) != 4000:
                raise ValueError(f"{session_dir} does not use the 4000-slot contract")
            split_names = {
                line.strip()
                for line in (session_dir / f"{self.split}.txt").read_text(
                    encoding="utf-8"
                ).splitlines()
                if line.strip()
            }
            self._metadata[session_dir] = metadata
            episodes: Dict[int, List[_FrameRef]] = {}
            for frame in metadata["frames"]:
                name = str(frame["name"])
                if name not in split_names:
                    continue
                episode_id = int(frame["episode_id"])
                ref = _FrameRef(
                    session_dir=session_dir,
                    session_name=session_dir.name,
                    split=self.split,
                    episode_id=episode_id,
                    name=name,
                    scan_timestamp_ns=max(
                        int(frame["scan_01_stamp_ns"]),
                        int(frame["scan_02_stamp_ns"]),
                    ),
                )
                episodes.setdefault(episode_id, []).append(ref)
            for episode_id in sorted(episodes):
                frames = episodes[episode_id]
                frames.sort(key=lambda item: item.scan_timestamp_ns)
                for end in range(self.history_frames - 1, len(frames)):
                    window = tuple(frames[end - self.history_frames + 1 : end + 1])
                    if any(
                        window[index].scan_timestamp_ns
                        >= window[index + 1].scan_timestamp_ns
                        for index in range(len(window) - 1)
                    ):
                        raise ValueError(
                            f"non-increasing timestamps in {session_dir}, episode {episode_id}"
                        )
                    self._windows.append(window)
        if not self._windows:
            raise ValueError(f"no {self.split} windows found under {self.dataset_root}")
        if max_samples is not None and 0 < int(max_samples) < len(self._windows):
            indices = np.linspace(
                0, len(self._windows) - 1, num=int(max_samples), dtype=np.int64
            )
            self._windows = [self._windows[int(index)] for index in indices]

    def __len__(self) -> int:
        return len(self._windows)

    @lru_cache(maxsize=256)
    def _load_frame_input(
        self, session_dir_text: str, name: str
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        session_dir = Path(session_dir_text)
        ranges = np.load(
            session_dir / "virtual_ranges_lidar" / name, allow_pickle=False
        ).astype(np.float64, copy=False)
        angles = np.load(
            session_dir / "virtual_angles_lidar" / name, allow_pickle=False
        ).astype(np.float64, copy=False)
        valid = np.load(
            session_dir / "valid_mask_lidar" / name, allow_pickle=False
        ).astype(bool, copy=False)
        pose = np.load(session_dir / "positions" / name, allow_pickle=False).astype(
            np.float64, copy=False
        )
        if ranges.shape != (4000,) or angles.shape != (4000,) or valid.shape != (4000,):
            raise ValueError(f"invalid fixed-slot shape for {session_dir / name}")
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError(f"invalid robot pose for {session_dir / name}")
        valid = valid & np.isfinite(ranges) & np.isfinite(angles) & (ranges > 0.0)
        slot_indices = np.flatnonzero(valid)
        xy = np.column_stack(
            (
                ranges[slot_indices] * np.cos(angles[slot_indices]),
                ranges[slot_indices] * np.sin(angles[slot_indices]),
            )
        )
        return (
            xy.astype(np.float32),
            slot_indices.astype(np.int32),
            pose.astype(np.float32),
        )

    def _window_to_bev(
        self, window: Sequence[_FrameRef]
    ) -> Tuple[np.ndarray, np.ndarray]:
        current_xy, _, current_pose = self._load_frame_input(
            str(window[-1].session_dir), window[-1].name
        )
        del current_xy
        channels = np.zeros(
            (self.history_frames * 2, self.bev_spec.size, self.bev_spec.size),
            dtype=np.float32,
        )
        for time_index, frame in enumerate(window):
            xy_base, slots, pose = self._load_frame_input(
                str(frame.session_dir), frame.name
            )
            points_map = base_to_map(xy_base, pose)
            points_current = map_to_base(points_map, current_pose)
            grid_x, grid_y = self.bev_spec.metric_to_grid(points_current)
            cols = np.floor(grid_x).astype(np.int64)
            rows = np.floor(grid_y).astype(np.int64)
            inside = (
                (cols >= 0)
                & (cols < self.bev_spec.size)
                & (rows >= 0)
                & (rows < self.bev_spec.size)
            )
            for sensor in (0, 1):
                sensor_mask = inside & ((slots // 2000) == sensor)
                channel = channels[time_index * 2 + sensor]
                np.add.at(channel, (rows[sensor_mask], cols[sensor_mask]), 1.0)
                channel[:] = np.minimum(channel, 3.0) / 3.0
        return (
            encode_temporal_bev(channels, self.input_encoding),
            current_pose.copy(),
        )

    def _build_current_targets(
        self, current: _FrameRef, current_pose: np.ndarray
    ) -> Dict[str, np.ndarray | int]:
        size = self.bev_spec.size
        heatmap = np.zeros((1, size, size), dtype=np.float32)
        offset = np.zeros((2, size, size), dtype=np.float32)
        velocity = np.zeros((2, size, size), dtype=np.float32)
        regression_mask = np.zeros((1, size, size), dtype=np.float32)
        session_dir = current.session_dir
        name = current.name

        xy_base, slots, _ = self._load_frame_input(str(session_dir), name)
        labels = np.load(
            session_dir / "semantic_label" / name, allow_pickle=False
        ).reshape(4000)
        person_points_base = xy_base[labels[slots] == PERSON_LABEL]
        if person_points_base.size == 0:
            return {
                "heatmap": heatmap,
                "offset": offset,
                "velocity": velocity,
                "regression_mask": regression_mask,
                "target_count": 0,
                "target_collisions": 0,
            }

        pedestrian_positions = np.load(
            session_dir / "pedestrian_positions" / name, allow_pickle=False
        ).astype(np.float64, copy=False)
        pedestrian_velocities = np.load(
            session_dir / "pedestrian_velocities" / name, allow_pickle=False
        ).astype(np.float64, copy=False)
        leg_positions = np.load(
            session_dir / "pedestrian_leg_positions" / name, allow_pickle=False
        ).astype(np.float64, copy=False)
        truth_timestamp_ns = int(
            np.load(
                session_dir / "pedestrian_truth_timestamps" / name,
                allow_pickle=False,
            )
        )
        count = pedestrian_positions.shape[0]
        if (
            pedestrian_positions.shape != (count, 2)
            or pedestrian_velocities.shape != (count, 2)
            or leg_positions.shape != (count, 2, 2)
        ):
            raise ValueError(f"invalid pedestrian truth shape for {session_dir / name}")

        truth_dt_s = (current.scan_timestamp_ns - truth_timestamp_ns) / 1e9
        aligned_positions = pedestrian_positions + pedestrian_velocities * truth_dt_s
        aligned_legs = (
            leg_positions + pedestrian_velocities[:, None, :] * truth_dt_s
        )
        person_points_map = base_to_map(person_points_base, current_pose)
        squared = np.sum(
            np.square(
                person_points_map[:, None, None, :]
                - aligned_legs[None, :, :, :]
            ),
            axis=-1,
        )
        point_to_person_distance = np.sqrt(np.min(squared, axis=2))
        nearest_person = np.argmin(point_to_person_distance, axis=1)
        nearest_distance = point_to_person_distance[
            np.arange(len(person_points_map)), nearest_person
        ]
        support_counts = np.bincount(
            nearest_person[nearest_distance <= self.leg_match_radius_m],
            minlength=count,
        )
        visible = np.flatnonzero(support_counts >= self.min_person_points)
        if visible.size == 0:
            return {
                "heatmap": heatmap,
                "offset": offset,
                "velocity": velocity,
                "regression_mask": regression_mask,
                "target_count": 0,
                "target_collisions": 0,
            }

        centers_base = map_to_base(aligned_positions[visible], current_pose)
        velocities_base = vector_map_to_base(
            pedestrian_velocities[visible], float(current_pose[2])
        )
        grid_x, grid_y = self.bev_spec.metric_to_grid(centers_base)
        ordered = sorted(
            range(len(visible)),
            key=lambda index: (-int(support_counts[visible[index]]), int(visible[index])),
        )
        target_count = 0
        collisions = 0
        for index in ordered:
            gx = float(grid_x[index])
            gy = float(grid_y[index])
            col = int(math.floor(gx))
            row = int(math.floor(gy))
            if not (0 <= col < size and 0 <= row < size):
                continue
            _draw_gaussian(
                heatmap[0], gx, gy, sigma_cells=self.target_sigma_cells
            )
            heatmap[0, row, col] = 1.0
            if regression_mask[0, row, col] > 0.0:
                collisions += 1
                continue
            offset[:, row, col] = [gx - col, gy - row]
            velocity[:, row, col] = velocities_base[index].astype(np.float32)
            regression_mask[0, row, col] = 1.0
            target_count += 1
        return {
            "heatmap": heatmap,
            "offset": offset,
            "velocity": velocity,
            "regression_mask": regression_mask,
            "target_count": target_count,
            "target_collisions": collisions,
        }

    def __getitem__(self, index: int) -> Dict[str, object]:
        window = self._windows[int(index)]
        bev, current_pose = self._window_to_bev(window)
        current = window[-1]
        sample: Dict[str, object] = {
            "input": torch.from_numpy(bev),
            "robot_pose_map": torch.from_numpy(current_pose.astype(np.float32)),
            "timestamp_ns": int(current.scan_timestamp_ns),
            "episode_id": int(current.episode_id),
            "session_name": current.session_name,
            "name": current.name,
        }
        if self.build_targets:
            targets = self._build_current_targets(current, current_pose)
            for key in ("heatmap", "offset", "velocity", "regression_mask"):
                sample[key] = torch.from_numpy(np.asarray(targets[key]))
            sample["target_count"] = int(targets["target_count"])
            sample["target_collisions"] = int(targets["target_collisions"])
        return sample

    def contract_dict(self) -> Dict[str, object]:
        return {
            "dataset_root": str(self.dataset_root),
            "split": self.split,
            "windows": len(self),
            "history_frames": self.history_frames,
            "input_shape": [
                self.history_frames * 2,
                self.bev_spec.size,
                self.bev_spec.size,
            ],
            "bev_extent_m": self.bev_spec.extent_m,
            "bev_resolution_m": self.bev_spec.resolution_m,
            "input_encoding": self.input_encoding,
            "build_targets": self.build_targets,
            "inference_ground_truth_inputs": [],
            "target_ground_truth_inputs": (
                [
                    "semantic_label",
                    "pedestrian_positions",
                    "pedestrian_velocities",
                    "pedestrian_leg_positions",
                    "pedestrian_truth_timestamps",
                ]
                if self.build_targets
                else []
            ),
        }
