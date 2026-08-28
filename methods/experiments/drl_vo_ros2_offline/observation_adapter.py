"""Convert validated fixed-dual lidar samples to legacy DRL-VO observations."""
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：COASTING, CONFIRMED, FRONT_SCAN, FULL_SCAN_BEAMS, LEGACY_ANGLE_INCREMENT, LEGACY_ANGLE_MIN, OBSERVATION_SIZE, PED_MAP_SHAPE, SCAN_HISTORY, TENTATIVE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/observation_adapter.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 11:42:54.206394635 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.812228331 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（ros2 launch 启动该场景）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/observation_adapter.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


OBSERVATION_SIZE = 19202
PED_MAP_SHAPE = (2, 80, 80)
SCAN_HISTORY = 10
FULL_SCAN_BEAMS = 1080
FRONT_SCAN = slice(180, 900)
LEGACY_ANGLE_MIN = -3.0 * math.pi / 4.0
LEGACY_ANGLE_INCREMENT = math.pi / 720.0


@dataclass(frozen=True)
class AdaptedFrame:
    observation: np.ndarray
    front_scan: np.ndarray
    semantic_map: np.ndarray | None
    pedestrian_map: np.ndarray
    goal_local: np.ndarray
    scan_coverage: float
    nearest_obstacle_m: float
    nearest_pedestrian_m: float
    pedestrian_ttc_0p6_s: float
    closest_approach_distance_m: float
    time_to_closest_approach_s: float
    timestamp_ns: int
    recorded_cmd: np.ndarray
    episode_id: int


def _track_value(track: object, name: str) -> Any:
    if isinstance(track, Mapping):
        if name not in track:
            raise KeyError(name)
        return track[name]
    return getattr(track, name)


def _empty_track_map_diagnostics() -> dict[str, object]:
    return {
        "written_track_ids": [],
        "written_cells": [],
        "excluded_tracks": [],
        "same_cell_conflict_count": 0,
        "dropped_track_count": 0,
    }


def tracks_to_drl_vo_ped_map_with_diagnostics(
    tracks: Sequence[object],
    robot_pose_map: np.ndarray,
    *,
    coasting_max_time_s: float = 0.5,
    max_track_age_s: float = 1.0,
    include_tentative: bool = False,
) -> tuple[np.ndarray, dict[str, object]]:
    """Convert map-frame tracks to the legacy absolute-velocity map.

    Channel 0 is ``vx_base`` and channel 1 is ``vy_base``.  Velocity is only
    rotated into robot axes; robot velocity is deliberately not subtracted.
    """

    tracks = list(tracks)
    result = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
    diagnostics = _empty_track_map_diagnostics()
    if coasting_max_time_s < 0.0 or max_track_age_s < 0.0:
        raise ValueError("track age limits cannot be negative")

    pose = np.asarray(robot_pose_map, dtype=np.float64)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        diagnostics["excluded_tracks"] = [
            {"track_id": None, "reason": "invalid_robot_pose"}
            for _ in tracks
        ]
        diagnostics["dropped_track_count"] = len(tracks)
        return result, diagnostics

    candidates = []
    excluded = []
    state_priority = {"CONFIRMED": 0, "COASTING": 1, "TENTATIVE": 2}
    for input_index, track in enumerate(tracks):
        track_id: int | None = None
        try:
            track_id = int(_track_value(track, "track_id"))
            state = str(_track_value(track, "track_state")).upper()
            confidence = float(_track_value(track, "confidence"))
            age_s = float(_track_value(track, "time_since_update_s"))
            position_map = np.asarray(
                _track_value(track, "position_xy_map"), dtype=np.float64
            )
            velocity_map = np.asarray(
                _track_value(track, "velocity_xy_map_absolute"),
                dtype=np.float64,
            )
        except (AttributeError, KeyError, TypeError, ValueError):
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "invalid_track_fields",
                    "input_index": input_index,
                }
            )
            continue
        if (
            position_map.shape != (2,)
            or velocity_map.shape != (2,)
            or not np.isfinite(position_map).all()
            or not np.isfinite(velocity_map).all()
            or not math.isfinite(confidence)
            or not math.isfinite(age_s)
        ):
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "non_finite_track",
                    "input_index": input_index,
                }
            )
            continue
        if age_s < 0.0 or age_s > max_track_age_s:
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "track_timeout",
                    "input_index": input_index,
                }
            )
            continue
        if state == "COASTING" and age_s > coasting_max_time_s:
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "coasting_timeout",
                    "input_index": input_index,
                }
            )
            continue
        if state == "TENTATIVE" and not include_tentative:
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "tentative_disabled",
                    "input_index": input_index,
                }
            )
            continue
        if state not in state_priority:
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "unsupported_track_state",
                    "input_index": input_index,
                }
            )
            continue

        position_base = rotate_map_to_base(
            (position_map - pose[:2]).reshape(1, 2), float(pose[2])
        )[0]
        ped_velocity_robot_axes_absolute = rotate_map_to_base(
            velocity_map.reshape(1, 2), float(pose[2])
        )[0]
        x_base, y_base = map(float, position_base)
        if not (0.0 <= x_base <= 20.0 and abs(y_base) <= 10.0):
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "outside_drl_fov",
                    "input_index": input_index,
                }
            )
            continue
        row = min(int(math.floor(x_base / 0.25)), 79)
        col = min(int(math.floor((10.0 - y_base) / 0.25)), 79)
        distance = float(np.linalg.norm(position_base))
        priority = (
            state_priority[state],
            age_s,
            -confidence,
            distance,
            track_id,
            input_index,
        )
        candidates.append(
            (
                priority,
                track_id,
                state,
                row,
                col,
                ped_velocity_robot_axes_absolute,
                input_index,
            )
        )

    occupied: dict[tuple[int, int], int] = {}
    written_cells = []
    written_ids = []
    conflict_count = 0
    for (
        _priority,
        track_id,
        state,
        row,
        col,
        velocity,
        input_index,
    ) in sorted(candidates, key=lambda item: item[0]):
        cell = (row, col)
        if cell in occupied:
            conflict_count += 1
            excluded.append(
                {
                    "track_id": track_id,
                    "reason": "same_cell_conflict",
                    "winner_track_id": occupied[cell],
                    "input_index": input_index,
                }
            )
            continue
        occupied[cell] = track_id
        result[0, row, col] = np.float32(velocity[0])
        result[1, row, col] = np.float32(velocity[1])
        written_ids.append(track_id)
        written_cells.append(
            {
                "track_id": track_id,
                "track_state": state,
                "row": row,
                "col": col,
            }
        )

    if not np.isfinite(result).all():
        result.fill(0.0)
        excluded = [
            {"track_id": None, "reason": "non_finite_output"}
            for _ in tracks
        ]
        written_ids = []
        written_cells = []
        conflict_count = 0
    diagnostics.update(
        {
            "written_track_ids": written_ids,
            "written_cells": written_cells,
            "excluded_tracks": sorted(
                excluded,
                key=lambda item: (
                    item["track_id"] is None,
                    item["track_id"] if item["track_id"] is not None else 0,
                    str(item["reason"]),
                    int(item.get("input_index", 0)),
                ),
            ),
            "same_cell_conflict_count": conflict_count,
            "dropped_track_count": len(excluded),
        }
    )
    return result, diagnostics


def tracks_to_drl_vo_ped_map(
    tracks: Sequence[object],
    robot_pose_map: np.ndarray,
    *,
    coasting_max_time_s: float = 0.5,
    max_track_age_s: float = 1.0,
    include_tentative: bool = False,
) -> np.ndarray:
    """Return an unnormalized ``float32`` DRL-VO pedestrian map."""

    result, _ = tracks_to_drl_vo_ped_map_with_diagnostics(
        tracks,
        robot_pose_map,
        coasting_max_time_s=coasting_max_time_s,
        max_track_age_s=max_track_age_s,
        include_tentative=include_tentative,
    )
    return result


def observation_with_pedestrian_map(
    observation: np.ndarray,
    pedestrian_map_mps: np.ndarray,
) -> np.ndarray:
    """Replace only the pedestrian prefix of an existing DRL-VO observation."""

    base = np.asarray(observation, dtype=np.float32)
    pedestrian_map = np.asarray(pedestrian_map_mps, dtype=np.float32)
    if base.shape != (OBSERVATION_SIZE,):
        raise ValueError(f"observation must have shape ({OBSERVATION_SIZE},)")
    if pedestrian_map.shape != PED_MAP_SHAPE:
        raise ValueError(f"pedestrian map must have shape {PED_MAP_SHAPE}")
    if not np.isfinite(base).all() or not np.isfinite(pedestrian_map).all():
        raise ValueError("observation inputs must be finite")
    result = base.copy()
    result[:12800] = np.clip(pedestrian_map / 2.0, -1.0, 1.0).reshape(-1)
    return result


def rotate_map_to_base(vectors: np.ndarray, robot_yaw: float) -> np.ndarray:
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    result = np.empty_like(vectors, dtype=np.float32)
    result[:, 0] = c * vectors[:, 0] + s * vectors[:, 1]
    result[:, 1] = -s * vectors[:, 0] + c * vectors[:, 1]
    return result


def pedestrian_velocity_map(
    pedestrian_xy_map: np.ndarray,
    pedestrian_velocity_map: np.ndarray,
    robot_pose_map: np.ndarray,
) -> tuple[np.ndarray, float]:
    relative_map = pedestrian_xy_map.astype(np.float32) - robot_pose_map[:2]
    relative_base = rotate_map_to_base(relative_map, float(robot_pose_map[2]))
    velocity_base = rotate_map_to_base(
        pedestrian_velocity_map.astype(np.float32),
        float(robot_pose_map[2]),
    )

    result = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
    distances = np.linalg.norm(relative_base, axis=1)
    for position, velocity in zip(relative_base, velocity_base):
        x, y = (float(position[0]), float(position[1]))
        if 0.0 <= x <= 20.0 and abs(y) <= 10.0:
            row = min(int(math.floor(x / 0.25)), 79)
            col = min(int(math.floor((10.0 - y) / 0.25)), 79)
            result[0, row, col] = velocity[0]
            result[1, row, col] = velocity[1]
    nearest = float(np.min(distances)) if len(distances) else math.nan
    return result, nearest


def pedestrian_risk_proxies(
    pedestrian_xy_map: np.ndarray,
    pedestrian_velocity_map: np.ndarray,
    robot_pose_map: np.ndarray,
    robot_linear_velocity: float,
    collision_radius: float = 0.6,
) -> tuple[float, float, float]:
    relative_map = pedestrian_xy_map.astype(np.float32) - robot_pose_map[:2]
    positions = rotate_map_to_base(relative_map, float(robot_pose_map[2]))
    velocities = rotate_map_to_base(
        pedestrian_velocity_map.astype(np.float32),
        float(robot_pose_map[2]),
    )
    velocities[:, 0] -= robot_linear_velocity

    best_ttc = math.inf
    best_closest_distance = math.inf
    best_closest_time = math.nan
    for position, velocity in zip(positions, velocities):
        speed_squared = float(np.dot(velocity, velocity))
        if speed_squared <= 1e-8:
            closest_time = 0.0
        else:
            closest_time = max(0.0, -float(np.dot(position, velocity)) / speed_squared)
        closest_position = position + closest_time * velocity
        closest_distance = float(np.linalg.norm(closest_position))
        if closest_distance < best_closest_distance:
            best_closest_distance = closest_distance
            best_closest_time = closest_time

        a = speed_squared
        b = 2.0 * float(np.dot(position, velocity))
        c = float(np.dot(position, position)) - collision_radius**2
        if c <= 0.0:
            best_ttc = 0.0
        elif a > 1e-8:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0.0:
                root = (-b - math.sqrt(discriminant)) / (2.0 * a)
                if root >= 0.0:
                    best_ttc = min(best_ttc, root)

    return (
        best_ttc if math.isfinite(best_ttc) else math.nan,
        best_closest_distance if math.isfinite(best_closest_distance) else math.nan,
        best_closest_time,
    )


def dual_lidar_to_legacy_scan(
    virtual_ranges: np.ndarray,
    virtual_angles: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    front, _semantic, coverage, nearest = _dual_lidar_to_legacy_front(
        virtual_ranges,
        virtual_angles,
    )
    return front, coverage, nearest


def _dual_lidar_to_legacy_front(
    virtual_ranges: np.ndarray,
    virtual_angles: np.ndarray,
    semantic_labels: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray | None, float, float]:
    if virtual_ranges.shape != virtual_angles.shape:
        raise ValueError("virtual range and angle shapes must match")
    if semantic_labels is not None and semantic_labels.shape != virtual_ranges.shape:
        raise ValueError("semantic labels must match virtual range shape")

    full_scan = np.zeros(FULL_SCAN_BEAMS, dtype=np.float32)
    full_semantic = (
        np.full(FULL_SCAN_BEAMS, -1, dtype=np.int64)
        if semantic_labels is not None
        else None
    )
    valid = (
        np.isfinite(virtual_ranges)
        & np.isfinite(virtual_angles)
        & (virtual_ranges > 0.0)
    )
    ranges = virtual_ranges[valid].astype(np.float32)
    angles = virtual_angles[valid].astype(np.float64)
    labels = (
        semantic_labels[valid].astype(np.int64)
        if semantic_labels is not None
        else None
    )
    indices = np.rint(
        (angles - LEGACY_ANGLE_MIN) / LEGACY_ANGLE_INCREMENT
    ).astype(np.int64)
    inside = (indices >= 0) & (indices < FULL_SCAN_BEAMS)
    indices = indices[inside]
    ranges = ranges[inside]
    if labels is not None:
        labels = labels[inside]

    if len(indices):
        order = np.lexsort((np.arange(len(indices)), ranges, indices))
        sorted_indices = indices[order]
        first = np.concatenate(
            ([True], sorted_indices[1:] != sorted_indices[:-1])
        )
        selected = order[first]
        full_scan[indices[selected]] = ranges[selected]
        if full_semantic is not None and labels is not None:
            full_semantic[indices[selected]] = labels[selected]

    front = full_scan[FRONT_SCAN].copy()
    front_semantic = (
        full_semantic[FRONT_SCAN].copy() if full_semantic is not None else None
    )
    coverage = float(np.count_nonzero(front) / front.size)
    nonzero = front[front > 0.0]
    nearest = float(np.min(nonzero)) if len(nonzero) else math.nan
    return front, front_semantic, coverage, nearest


def dual_lidar_to_legacy_semantic(
    virtual_ranges: np.ndarray,
    virtual_angles: np.ndarray,
    semantic_labels: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    front, semantic, coverage, nearest = _dual_lidar_to_legacy_front(
        virtual_ranges,
        virtual_angles,
        semantic_labels,
    )
    if semantic is None:
        raise RuntimeError("Semantic projection unexpectedly returned no labels")
    return front, semantic, coverage, nearest


def compress_scan_history(scan_history: np.ndarray) -> np.ndarray:
    if scan_history.shape != (SCAN_HISTORY, 720):
        raise ValueError(f"Expected scan history (10, 720), got {scan_history.shape}")
    groups = scan_history.reshape(SCAN_HISTORY, 80, 9)
    reduced = np.empty((SCAN_HISTORY, 2, 80), dtype=np.float32)
    reduced[:, 0, :] = np.min(groups, axis=2)
    reduced[:, 1, :] = np.mean(groups, axis=2)
    scan_20x80 = reduced.reshape(20, 80)
    return np.tile(scan_20x80.reshape(-1), 4).astype(np.float32)


def compress_semantic_history(
    scan_history: np.ndarray,
    semantic_history: np.ndarray,
) -> np.ndarray:
    if scan_history.shape != (SCAN_HISTORY, 720):
        raise ValueError(f"Expected scan history (10, 720), got {scan_history.shape}")
    if semantic_history.shape != scan_history.shape:
        raise ValueError("Semantic history must match scan history")

    ranges = scan_history.reshape(SCAN_HISTORY, 80, 9)
    labels = semantic_history.reshape(SCAN_HISTORY, 80, 9).astype(np.int64)
    reduced = np.full((SCAN_HISTORY, 2, 80), -1, dtype=np.int64)
    for history_index in range(SCAN_HISTORY):
        for bin_index in range(80):
            group_ranges = ranges[history_index, bin_index]
            group_labels = labels[history_index, bin_index]
            labeled = (group_ranges > 0.0) & (group_labels >= 0)
            if not np.any(labeled):
                continue
            labeled_indices = np.flatnonzero(labeled)
            nearest = labeled_indices[
                int(np.argmin(group_ranges[labeled_indices]))
            ]
            reduced[history_index, 0, bin_index] = group_labels[nearest]
            reduced[history_index, 1, bin_index] = int(
                np.bincount(group_labels[labeled_indices]).argmax()
            )
    semantic_20x80 = reduced.reshape(20, 80)
    return np.tile(semantic_20x80, (4, 1)).astype(np.int16)


class ObservationAdapter:
    def __init__(self, include_semantics: bool = False) -> None:
        self.include_semantics = include_semantics
        self._scan_history: list[np.ndarray] = []
        self._semantic_history: list[np.ndarray] = []
        self._sequence_key: tuple[str | None, int] | None = None
        self._last_timestamp_ns: int | None = None

    def reset(self) -> None:
        self._scan_history.clear()
        self._semantic_history.clear()
        self._sequence_key = None
        self._last_timestamp_ns = None

    def adapt(
        self,
        sample_path: str | Path,
        sequence_id: str | None = None,
    ) -> AdaptedFrame:
        with np.load(sample_path, allow_pickle=False) as sample:
            pose = sample["position"].astype(np.float32)
            ped_map, nearest_pedestrian = pedestrian_velocity_map(
                sample["pedestrian_xy_map"],
                sample["pedestrian_velocity_map"],
                pose,
            )
            robot_velocity = sample["velocity"].astype(np.float32)
            ttc, closest_distance, closest_time = pedestrian_risk_proxies(
                sample["pedestrian_xy_map"],
                sample["pedestrian_velocity_map"],
                pose,
                float(robot_velocity[0]),
            )
            return self._adapt_loaded_sample(
                sample,
                sample_path,
                ped_map,
                sequence_id=sequence_id,
                nearest_pedestrian=nearest_pedestrian,
                ttc=ttc,
                closest_distance=closest_distance,
                closest_time=closest_time,
            )

    def adapt_with_pedestrian_map(
        self,
        sample_path: str | Path,
        pedestrian_map_mps: np.ndarray,
        sequence_id: str | None = None,
    ) -> AdaptedFrame:
        """Adapt sensors and goal without reading semantic or pedestrian truth."""

        ped_map = np.asarray(pedestrian_map_mps, dtype=np.float32)
        if ped_map.shape != PED_MAP_SHAPE or not np.isfinite(ped_map).all():
            raise ValueError(
                f"pedestrian map must be finite with shape {PED_MAP_SHAPE}"
            )
        with np.load(sample_path, allow_pickle=False) as sample:
            return self._adapt_loaded_sample(
                sample,
                sample_path,
                ped_map,
                sequence_id=sequence_id,
                nearest_pedestrian=math.nan,
                ttc=math.nan,
                closest_distance=math.nan,
                closest_time=math.nan,
            )

    def _adapt_loaded_sample(
        self,
        sample: Any,
        sample_path: str | Path,
        ped_map: np.ndarray,
        *,
        sequence_id: str | None,
        nearest_pedestrian: float,
        ttc: float,
        closest_distance: float,
        closest_time: float,
    ) -> AdaptedFrame:
        if "episode_id" not in sample:
            raise KeyError(f"episode_id is missing from {sample_path}")
        episode_id = int(sample["episode_id"])
        sequence_key = (sequence_id, episode_id)
        if self._sequence_key != sequence_key:
            self.reset()
            self._sequence_key = sequence_key
        timestamp_ns = int(sample["scan_01_stamp_ns"])
        if (
            self._last_timestamp_ns is not None
            and timestamp_ns <= self._last_timestamp_ns
        ):
            raise ValueError("observation timestamps must be strictly increasing")

        if self.include_semantics:
            if "semantic_label" not in sample:
                raise KeyError(f"semantic_label is missing from {sample_path}")
            (
                front_scan,
                front_semantic,
                coverage,
                nearest_obstacle,
            ) = dual_lidar_to_legacy_semantic(
                sample["virtual_ranges"],
                sample["virtual_angles"],
                sample["semantic_label"],
            )
        else:
            front_scan, coverage, nearest_obstacle = dual_lidar_to_legacy_scan(
                sample["virtual_ranges"],
                sample["virtual_angles"],
            )
            front_semantic = None
        goal = sample["sub_goal_local"].astype(np.float32)
        recorded_cmd = sample["cmd_velocity"].astype(np.float32)
        self._last_timestamp_ns = timestamp_ns

        self._scan_history.append(front_scan)
        if len(self._scan_history) > SCAN_HISTORY:
            self._scan_history.pop(0)
        padded = [self._scan_history[0]] * (SCAN_HISTORY - len(self._scan_history))
        history = np.stack(padded + self._scan_history)

        semantic_map = None
        if front_semantic is not None:
            self._semantic_history.append(front_semantic)
            if len(self._semantic_history) > SCAN_HISTORY:
                self._semantic_history.pop(0)
            semantic_padded = [self._semantic_history[0]] * (
                SCAN_HISTORY - len(self._semantic_history)
            )
            semantic_history = np.stack(
                semantic_padded + self._semantic_history
            )
            semantic_map = compress_semantic_history(history, semantic_history)

        ped_normalized = np.clip(ped_map / 2.0, -1.0, 1.0)
        scan_normalized = compress_scan_history(history) / 15.0 - 1.0
        goal_normalized = goal / 2.0
        observation = np.concatenate(
            (ped_normalized.reshape(-1), scan_normalized, goal_normalized)
        ).astype(np.float32)
        if observation.shape != (OBSERVATION_SIZE,):
            raise RuntimeError(f"Unexpected observation shape {observation.shape}")
        if not np.isfinite(observation).all():
            raise RuntimeError(f"Non-finite observation from {sample_path}")

        return AdaptedFrame(
            observation=observation,
            front_scan=front_scan,
            semantic_map=semantic_map,
            pedestrian_map=ped_map,
            goal_local=goal,
            scan_coverage=coverage,
            nearest_obstacle_m=nearest_obstacle,
            nearest_pedestrian_m=nearest_pedestrian,
            pedestrian_ttc_0p6_s=ttc,
            closest_approach_distance_m=closest_distance,
            time_to_closest_approach_s=closest_time,
            timestamp_ns=timestamp_ns,
            recorded_cmd=recorded_cmd,
            episode_id=episode_id,
        )


def normalized_to_physical(action: np.ndarray) -> np.ndarray:
    clipped = np.clip(action.astype(np.float32), -1.0, 1.0)
    return np.asarray(
        [(clipped[0] + 1.0) * 0.25, clipped[1] * 2.0],
        dtype=np.float32,
    )
