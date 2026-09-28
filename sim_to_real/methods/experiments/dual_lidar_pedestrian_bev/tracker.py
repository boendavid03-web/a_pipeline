#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：COASTING, CONFIRMED, TENTATIVE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/tracker.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:36:31.423662483 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.378546463 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/tracker.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Deterministic constant-velocity tracking for neural pedestrian detections."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .dataset import base_to_map, _rotation
from .model import DecodedDetection


@dataclass(frozen=True)
class MapDetection:
    position_xy_map: np.ndarray
    velocity_xy_map_absolute: np.ndarray
    confidence: float


@dataclass(frozen=True)
class TrackedPedestrian:
    track_id: int
    position_xy_map: np.ndarray
    velocity_xy_map_absolute: np.ndarray
    confidence: float
    track_state: str
    time_since_update_s: float


def detections_base_to_map(
    detections: Sequence[DecodedDetection], robot_pose_map: np.ndarray
) -> List[MapDetection]:
    pose = np.asarray(robot_pose_map, dtype=np.float64).reshape(3)
    if not np.isfinite(pose).all():
        raise ValueError("robot pose must be finite")
    rotation = _rotation(float(pose[2]))
    converted = []
    for detection in detections:
        position = base_to_map(
            np.asarray(detection.position_xy_base).reshape(1, 2), pose
        )[0]
        velocity = (
            np.asarray(detection.velocity_xy_robot_axes_absolute, dtype=np.float64)
            .reshape(1, 2)
            @ rotation.T
        )[0]
        converted.append(
            MapDetection(
                position_xy_map=position,
                velocity_xy_map_absolute=velocity,
                confidence=float(detection.confidence),
            )
        )
    return converted


def linear_sum_assignment(cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rectangular Hungarian assignment without a SciPy runtime dependency."""

    costs = np.asarray(cost_matrix, dtype=np.float64)
    if costs.ndim != 2:
        raise ValueError("cost matrix must be two-dimensional")
    original_rows, original_cols = costs.shape
    if original_rows == 0 or original_cols == 0:
        return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
    transposed = original_rows > original_cols
    if transposed:
        costs = costs.T
    rows, cols = costs.shape
    u = np.zeros(rows + 1, dtype=np.float64)
    v = np.zeros(cols + 1, dtype=np.float64)
    p = np.zeros(cols + 1, dtype=np.int64)
    way = np.zeros(cols + 1, dtype=np.int64)
    for row in range(1, rows + 1):
        p[0] = row
        min_values = np.full(cols + 1, np.inf, dtype=np.float64)
        used = np.zeros(cols + 1, dtype=bool)
        column0 = 0
        while True:
            used[column0] = True
            row0 = p[column0]
            delta = np.inf
            column1 = 0
            for column in range(1, cols + 1):
                if used[column]:
                    continue
                current = costs[row0 - 1, column - 1] - u[row0] - v[column]
                if current < min_values[column]:
                    min_values[column] = current
                    way[column] = column0
                if min_values[column] < delta:
                    delta = min_values[column]
                    column1 = column
            if not np.isfinite(delta):
                raise ValueError("assignment cost matrix contains no finite solution")
            for column in range(cols + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    min_values[column] -= delta
            column0 = column1
            if p[column0] == 0:
                break
        while True:
            column1 = way[column0]
            p[column0] = p[column1]
            column0 = column1
            if column0 == 0:
                break
    row_indices = []
    col_indices = []
    for column in range(1, cols + 1):
        if p[column] != 0:
            row_indices.append(int(p[column] - 1))
            col_indices.append(int(column - 1))
    row_array = np.asarray(row_indices, dtype=np.int64)
    col_array = np.asarray(col_indices, dtype=np.int64)
    order = np.argsort(row_array, kind="stable")
    row_array = row_array[order]
    col_array = col_array[order]
    if transposed:
        return col_array, row_array
    return row_array, col_array


class _Track:
    def __init__(
        self, track_id: int, detection: MapDetection, timestamp_ns: int
    ) -> None:
        self.track_id = int(track_id)
        self.state = np.concatenate(
            (
                np.asarray(detection.position_xy_map, dtype=np.float64),
                np.asarray(detection.velocity_xy_map_absolute, dtype=np.float64),
            )
        )
        self.covariance = np.diag([0.15**2, 0.15**2, 0.5**2, 0.5**2])
        self.created_ns = int(timestamp_ns)
        self.last_predict_ns = int(timestamp_ns)
        self.last_update_ns = int(timestamp_ns)
        self.hit_count = 1
        self.track_state = "TENTATIVE"
        self.confidence = float(detection.confidence)

    def predict(self, timestamp_ns: int, acceleration_sigma: float) -> None:
        dt = max(0.0, (int(timestamp_ns) - self.last_predict_ns) / 1e9)
        transition = np.asarray(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        process = acceleration_sigma**2 * np.asarray(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=np.float64,
        )
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_predict_ns = int(timestamp_ns)

    def update(
        self,
        detection: MapDetection,
        timestamp_ns: int,
        *,
        position_measurement_scale: float,
        velocity_measurement_scale: float,
    ) -> None:
        measurement = np.concatenate(
            (
                np.asarray(detection.position_xy_map, dtype=np.float64),
                np.asarray(detection.velocity_xy_map_absolute, dtype=np.float64),
            )
        )
        confidence = float(np.clip(detection.confidence, 0.05, 1.0))
        measurement_covariance = np.diag(
            [
                (
                    position_measurement_scale
                    * (0.08 + 0.20 * (1.0 - confidence))
                )
                ** 2,
                (
                    position_measurement_scale
                    * (0.08 + 0.20 * (1.0 - confidence))
                )
                ** 2,
                (
                    velocity_measurement_scale
                    * (0.20 + 0.60 * (1.0 - confidence))
                )
                ** 2,
                (
                    velocity_measurement_scale
                    * (0.20 + 0.60 * (1.0 - confidence))
                )
                ** 2,
            ]
        )
        innovation = measurement - self.state
        innovation_covariance = self.covariance + measurement_covariance
        gain = self.covariance @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(4, dtype=np.float64)
        self.covariance = (
            (identity - gain)
            @ self.covariance
            @ (identity - gain).T
            + gain @ measurement_covariance @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_update_ns = int(timestamp_ns)
        self.hit_count += 1
        if self.hit_count >= 3:
            self.track_state = "CONFIRMED"
        self.confidence = float(
            np.clip(0.65 * self.confidence + 0.35 * detection.confidence, 0.0, 1.0)
        )

    def snapshot(self, timestamp_ns: int) -> TrackedPedestrian:
        time_since_update = (int(timestamp_ns) - self.last_update_ns) / 1e9
        return TrackedPedestrian(
            track_id=self.track_id,
            position_xy_map=self.state[:2].copy(),
            velocity_xy_map_absolute=self.state[2:].copy(),
            confidence=float(self.confidence),
            track_state=self.track_state,
            time_since_update_s=float(time_since_update),
        )


class PedestrianTracker:
    def __init__(
        self,
        *,
        position_gate_m: float = 0.5,
        velocity_gate_mps: float = 1.5,
        tentative_timeout_s: float = 0.33,
        confirmed_timeout_s: float = 1.0,
        acceleration_sigma_mps2: float = 3.0,
        position_measurement_scale: float = 0.75,
        velocity_measurement_scale: float = 2.0,
        association_velocity_weight: float = 0.4,
    ) -> None:
        self.position_gate_m = float(position_gate_m)
        self.velocity_gate_mps = float(velocity_gate_mps)
        self.tentative_timeout_s = float(tentative_timeout_s)
        self.confirmed_timeout_s = float(confirmed_timeout_s)
        self.acceleration_sigma_mps2 = float(acceleration_sigma_mps2)
        self.position_measurement_scale = float(position_measurement_scale)
        self.velocity_measurement_scale = float(velocity_measurement_scale)
        self.association_velocity_weight = float(association_velocity_weight)
        if self.position_measurement_scale <= 0.0:
            raise ValueError("position_measurement_scale must be positive")
        if self.velocity_measurement_scale <= 0.0:
            raise ValueError("velocity_measurement_scale must be positive")
        if self.association_velocity_weight < 0.0:
            raise ValueError("association_velocity_weight cannot be negative")
        self.tracks: List[_Track] = []
        self.next_track_id = 1
        self.last_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.tracks = []
        self.next_track_id = 1
        self.last_timestamp_ns = None

    def update(
        self, detections: Sequence[MapDetection], timestamp_ns: int
    ) -> List[TrackedPedestrian]:
        timestamp_ns = int(timestamp_ns)
        if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
            raise ValueError("tracker timestamps must be strictly increasing")
        self.last_timestamp_ns = timestamp_ns
        detections = list(detections)
        for detection in detections:
            position = np.asarray(detection.position_xy_map, dtype=np.float64)
            velocity = np.asarray(
                detection.velocity_xy_map_absolute, dtype=np.float64
            )
            if (
                position.shape != (2,)
                or velocity.shape != (2,)
                or not np.isfinite(position).all()
                or not np.isfinite(velocity).all()
                or not math.isfinite(float(detection.confidence))
            ):
                raise ValueError("tracker detections must be finite 2D states")
        detections.sort(
            key=lambda item: (
                -float(item.confidence),
                float(item.position_xy_map[0]),
                float(item.position_xy_map[1]),
                float(item.velocity_xy_map_absolute[0]),
                float(item.velocity_xy_map_absolute[1]),
            )
        )
        for track in self.tracks:
            track.predict(timestamp_ns, self.acceleration_sigma_mps2)

        matched_tracks = set()
        matched_detections = set()
        if self.tracks and detections:
            invalid = 1e9
            costs = np.full((len(self.tracks), len(detections)), invalid)
            for row, track in enumerate(self.tracks):
                for col, detection in enumerate(detections):
                    position_distance = float(
                        np.linalg.norm(
                            track.state[:2]
                            - np.asarray(detection.position_xy_map, dtype=np.float64)
                        )
                    )
                    velocity_distance = float(
                        np.linalg.norm(
                            track.state[2:]
                            - np.asarray(
                                detection.velocity_xy_map_absolute,
                                dtype=np.float64,
                            )
                        )
                    )
                    if (
                        position_distance <= self.position_gate_m
                        and velocity_distance <= self.velocity_gate_mps
                    ):
                        costs[row, col] = (
                            position_distance
                            + self.association_velocity_weight
                            * velocity_distance
                            - 0.05 * float(detection.confidence)
                            + 1e-9 * (row * len(detections) + col)
                        )
            rows, cols = linear_sum_assignment(costs)
            for row, col in zip(rows.tolist(), cols.tolist()):
                if costs[row, col] >= invalid:
                    continue
                self.tracks[row].update(
                    detections[col],
                    timestamp_ns,
                    position_measurement_scale=self.position_measurement_scale,
                    velocity_measurement_scale=self.velocity_measurement_scale,
                )
                matched_tracks.add(row)
                matched_detections.add(col)

        for col, detection in enumerate(detections):
            if col in matched_detections:
                continue
            self.tracks.append(
                _Track(self.next_track_id, detection, timestamp_ns)
            )
            self.next_track_id += 1

        survivors = []
        for row, track in enumerate(self.tracks):
            time_since_update = (timestamp_ns - track.last_update_ns) / 1e9
            if row not in matched_tracks and time_since_update > 0.0:
                track.track_state = (
                    "COASTING"
                    if track.track_state in {"CONFIRMED", "COASTING"}
                    else "TENTATIVE"
                )
                track.confidence *= math.exp(
                    -time_since_update
                    / (
                        self.confirmed_timeout_s
                        if track.track_state == "COASTING"
                        else self.tentative_timeout_s
                    )
                )
            timeout = (
                self.confirmed_timeout_s
                if track.track_state == "COASTING"
                else self.tentative_timeout_s
            )
            if time_since_update <= timeout:
                survivors.append(track)
        self.tracks = survivors
        return [
            track.snapshot(timestamp_ns)
            for track in sorted(self.tracks, key=lambda item: item.track_id)
        ]
