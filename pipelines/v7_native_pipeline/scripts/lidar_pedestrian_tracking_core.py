#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：WORLD, YAML
# 可能使用的关键环境变量：COASTING, CONFIRMED, DBSCAN, DELETED, DOUBLE, DOUBLE_LEG, FUSION_DUAL, FUSION_SINGLE_DEGRADED, MEASUREMENT_DOUBLE, MEASUREMENT_MERGED_BODY, MEASUREMENT_SINGLE, MERGED_BODY, SINGLE, SINGLE_LEG, TENTATIVE, TENTATIVE_STATIC, TRACK_COASTING, TRACK_CONFIRMED, TRACK_DELETED, TRACK_TENTATIVE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/lidar_pedestrian_tracking_core.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-24 08:03:27.464948723 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.914035757 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/lidar_pedestrian_tracking_core.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜lidar_pedestrian_tracking_core.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Truth-independent dual-2D-LiDAR pedestrian detection and tracking core.

This module deliberately has no dataset loader and no semantic or pedestrian
ground-truth inputs.  The offline evaluator extracts the small EstimatorFrame
contract before calling this code.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree


MEASUREMENT_SINGLE = "SINGLE_LEG"
MEASUREMENT_DOUBLE = "DOUBLE_LEG"
MEASUREMENT_MERGED_BODY = "MERGED_BODY"

TRACK_TENTATIVE = "TENTATIVE"
TRACK_TENTATIVE_STATIC = "TENTATIVE_STATIC"
TRACK_CONFIRMED = "CONFIRMED"
TRACK_COASTING = "COASTING"
TRACK_DELETED = "DELETED"

FUSION_DUAL = "dual"
FUSION_SINGLE_DEGRADED = "single_degraded"


def _as_float_array(value, shape: Tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} has shape {array.shape}, expected {shape}")
    return array


@dataclass(frozen=True)
class EstimatorFrame:
    """Only data that the estimator is authorized to consume."""

    frame_index: int
    scan_ranges: np.ndarray
    virtual_angles: np.ndarray
    valid_mask: np.ndarray
    source_sensor: np.ndarray
    robot_pose_map: np.ndarray
    scan_timestamp_lidar_1: int
    scan_timestamp_lidar_2: int

    def __post_init__(self) -> None:
        ranges = np.asarray(self.scan_ranges, dtype=np.float64)
        angles = np.asarray(self.virtual_angles, dtype=np.float64)
        valid = np.asarray(self.valid_mask, dtype=np.bool_)
        source = np.asarray(self.source_sensor, dtype=np.uint8)
        if ranges.ndim != 1:
            raise ValueError("scan_ranges must be one-dimensional")
        for name, array in (
            ("virtual_angles", angles),
            ("valid_mask", valid),
            ("source_sensor", source),
        ):
            if array.shape != ranges.shape:
                raise ValueError(
                    f"{name} has shape {array.shape}, expected {ranges.shape}"
                )
        if np.any((source != 0) & (source != 1)):
            raise ValueError("source_sensor values must be 0 or 1")
        poses = _as_float_array(self.robot_pose_map, (2, 3), "robot_pose_map")
        if not np.isfinite(poses).all():
            raise ValueError("robot_pose_map contains non-finite values")
        if int(self.scan_timestamp_lidar_1) < 0 or int(
            self.scan_timestamp_lidar_2
        ) < 0:
            raise ValueError("scan timestamps must be non-negative")
        object.__setattr__(self, "scan_ranges", ranges)
        object.__setattr__(self, "virtual_angles", angles)
        object.__setattr__(self, "valid_mask", valid)
        object.__setattr__(self, "source_sensor", source)
        object.__setattr__(self, "robot_pose_map", poses)


@dataclass(frozen=True)
class TrackerConfig:
    static_distance_threshold_m: float = 0.25
    cluster_epsilon_m: float = 0.10
    cluster_min_points: int = 3
    max_sensor_time_skew_s: float = 0.01
    double_leg_min_separation_m: float = 0.08
    double_leg_max_separation_m: float = 0.30
    expected_leg_separation_m: float = 0.14
    merged_body_min_width_m: float = 0.18
    double_sigma_m: float = 0.08
    single_sigma_m: float = 0.16
    merged_sigma_m: float = 0.25
    mahalanobis_gate_d2: float = 9.21
    euclidean_gate_m: float = 0.8
    process_acceleration_sigma_mps2: float = 4.0
    reliable_motion_speed_mps: float = 0.15
    reliable_motion_displacement_m: float = 0.10
    tentative_miss_timeout_s: float = 0.33
    tentative_max_age_s: float = 0.8
    confirmed_coast_timeout_s: float = 1.0
    newborn_pair_unmatched_cost: float = 1.0
    tie_break_epsilon: float = 1e-9

    def validate(self) -> None:
        positive = (
            "static_distance_threshold_m",
            "cluster_epsilon_m",
            "max_sensor_time_skew_s",
            "double_leg_min_separation_m",
            "double_leg_max_separation_m",
            "expected_leg_separation_m",
            "merged_body_min_width_m",
            "double_sigma_m",
            "single_sigma_m",
            "merged_sigma_m",
            "mahalanobis_gate_d2",
            "euclidean_gate_m",
            "process_acceleration_sigma_mps2",
            "tentative_miss_timeout_s",
            "tentative_max_age_s",
            "confirmed_coast_timeout_s",
        )
        for name in positive:
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.cluster_min_points < 1:
            raise ValueError("cluster_min_points must be at least one")
        if not (
            self.double_leg_min_separation_m
            < self.double_leg_max_separation_m
        ):
            raise ValueError("invalid double-leg separation interval")
        if not (
            self.double_sigma_m < self.single_sigma_m < self.merged_sigma_m
        ):
            raise ValueError(
                "measurement noise must satisfy DOUBLE < SINGLE < MERGED_BODY"
            )


class OccupancyMap:
    """ROS trinary occupancy map with full SE(2) origin handling."""

    def __init__(
        self,
        image: np.ndarray,
        resolution: float,
        origin: Sequence[float],
        negate: int,
        occupied_thresh: float,
        free_thresh: float,
        image_path: Path,
        yaml_path: Path,
    ) -> None:
        self.image = np.asarray(image, dtype=np.uint8)
        if self.image.ndim != 2:
            raise ValueError("occupancy image must be grayscale")
        self.height, self.width = self.image.shape
        self.resolution = float(resolution)
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if len(origin) != 3:
            raise ValueError("map origin must contain x, y, yaw")
        self.origin_x, self.origin_y, self.origin_yaw = (
            float(origin[0]),
            float(origin[1]),
            float(origin[2]),
        )
        self.negate = int(negate)
        if self.negate not in (0, 1):
            raise ValueError("map negate must be 0 or 1")
        self.occupied_thresh = float(occupied_thresh)
        self.free_thresh = float(free_thresh)
        if not 0.0 <= self.free_thresh < self.occupied_thresh <= 1.0:
            raise ValueError(
                "map thresholds must satisfy 0 <= free < occupied <= 1"
            )
        color = self.image.astype(np.float64) / 255.0
        occupancy_probability = color if self.negate else 1.0 - color
        self.occupied_mask = occupancy_probability > self.occupied_thresh
        self.free_mask = occupancy_probability < self.free_thresh
        self.unknown_mask = ~(self.occupied_mask | self.free_mask)
        self.static_distance_m = (
            distance_transform_edt(~self.occupied_mask) * self.resolution
        )
        self.image_path = Path(image_path)
        self.yaml_path = Path(yaml_path)

    @classmethod
    def from_yaml(cls, path: Path) -> "OccupancyMap":
        path = Path(path)
        metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
        required = (
            "image",
            "resolution",
            "origin",
            "negate",
            "occupied_thresh",
            "free_thresh",
        )
        missing = [name for name in required if name not in metadata]
        if missing:
            raise ValueError(f"map YAML is missing fields: {missing}")
        image_path = Path(str(metadata["image"])).expanduser()
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        if not image_path.is_file():
            raise FileNotFoundError(f"occupancy image not found: {image_path}")
        image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        return cls(
            image=image,
            resolution=metadata["resolution"],
            origin=metadata["origin"],
            negate=metadata["negate"],
            occupied_thresh=metadata["occupied_thresh"],
            free_thresh=metadata["free_thresh"],
            image_path=image_path.resolve(),
            yaml_path=path.resolve(),
        )

    def world_to_grid(
        self, points_xy: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        points = np.asarray(points_xy, dtype=np.float64).reshape((-1, 2))
        dx = points[:, 0] - self.origin_x
        dy = points[:, 1] - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        cols = np.floor(local_x / self.resolution).astype(np.int64)
        grid_y = np.floor(local_y / self.resolution).astype(np.int64)
        rows = self.height - 1 - grid_y
        inside = (
            (cols >= 0)
            & (cols < self.width)
            & (rows >= 0)
            & (rows < self.height)
        )
        return rows, cols, inside

    def grid_to_world(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        rows = np.asarray(rows, dtype=np.int64).reshape(-1)
        cols = np.asarray(cols, dtype=np.int64).reshape(-1)
        if rows.shape != cols.shape:
            raise ValueError("rows and cols must have matching shapes")
        local_x = (cols.astype(np.float64) + 0.5) * self.resolution
        grid_y = self.height - 1 - rows
        local_y = (grid_y.astype(np.float64) + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        world_x = self.origin_x + cosine * local_x - sine * local_y
        world_y = self.origin_y + sine * local_x + cosine * local_y
        return np.column_stack((world_x, world_y))

    def audit_dict(self) -> Dict[str, object]:
        return {
            "yaml_path": str(self.yaml_path),
            "image_path": str(self.image_path),
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, self.origin_yaw],
            "negate": self.negate,
            "occupied_thresh": self.occupied_thresh,
            "free_thresh": self.free_thresh,
            "occupied_cells": int(self.occupied_mask.sum()),
            "free_cells": int(self.free_mask.sum()),
            "unknown_cells": int(self.unknown_mask.sum()),
        }


@dataclass
class PointCloudFrame:
    slot_indices: np.ndarray
    points_map: np.ndarray
    source_sensor: np.ndarray
    ranges: np.ndarray
    static_distance_m: np.ndarray
    candidate_mask: np.ndarray
    fusion_timestamp_ns: int
    sensor_time_skew_s: float
    fusion_mode: str
    active_sensor: Optional[int]

    @property
    def candidate_indices(self) -> np.ndarray:
        return self.slot_indices[self.candidate_mask]

    @property
    def candidate_points(self) -> np.ndarray:
        return self.points_map[self.candidate_mask]


@dataclass
class LegCluster:
    cluster_id: int
    point_indices: np.ndarray
    centroid: np.ndarray
    covariance: np.ndarray
    width: float
    point_count: int
    source_mask: int
    static_distance_min: float
    static_distance_mean: float
    range_mean: float

    def provisional_mode(self, config: TrackerConfig) -> str:
        if self.width >= config.merged_body_min_width_m:
            return MEASUREMENT_MERGED_BODY
        return MEASUREMENT_SINGLE


@dataclass
class PersonMeasurement:
    measurement_id: int
    mode: str
    cluster_ids: Tuple[int, ...]
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    conditioned_track_id: Optional[int] = None


@dataclass
class AssociationRecord:
    stage: str
    track_id: Optional[int]
    cluster_id: Optional[int]
    measurement_id: Optional[int]
    mahalanobis_d2: Optional[float]
    euclidean_distance: Optional[float]
    gate_passed: bool
    assignment_result: str
    rejection_reason: str = ""


@dataclass
class TrackSnapshot:
    track_id: int
    track_state: str
    predicted_state: np.ndarray
    updated_state: np.ndarray
    covariance: np.ndarray
    innovation: Optional[np.ndarray]
    age_s: float
    time_since_update_s: float
    visible_hit_count: int
    consecutive_miss_count: int
    confidence: float
    support_mode: Optional[str]
    support_cluster_ids: Tuple[int, ...]
    support_measurement_id: Optional[int]
    had_reliable_motion: bool
    had_multileg_support: bool


@dataclass
class EstimatorResult:
    frame_index: int
    fusion_timestamp_ns: int
    sensor_time_skew_s: float
    fusion_mode: str
    active_sensor: Optional[int]
    point_cloud: PointCloudFrame
    clusters: List[LegCluster]
    measurements: List[PersonMeasurement]
    associations: List[AssociationRecord]
    tracks: List[TrackSnapshot]


def frame_points_map(
    frame: EstimatorFrame,
    occupancy_map: OccupancyMap,
    config: TrackerConfig,
) -> PointCloudFrame:
    timestamp_1 = int(frame.scan_timestamp_lidar_1)
    timestamp_2 = int(frame.scan_timestamp_lidar_2)
    skew_s = abs(timestamp_1 - timestamp_2) / 1e9
    fusion_timestamp = max(timestamp_1, timestamp_2)
    active_sensor: Optional[int] = None
    active = np.ones(frame.scan_ranges.shape, dtype=np.bool_)
    fusion_mode = FUSION_DUAL
    if skew_s > config.max_sensor_time_skew_s:
        active_sensor = 0 if timestamp_1 >= timestamp_2 else 1
        active &= frame.source_sensor == active_sensor
        fusion_mode = FUSION_SINGLE_DEGRADED

    valid = (
        active
        & frame.valid_mask
        & np.isfinite(frame.scan_ranges)
        & np.isfinite(frame.virtual_angles)
        & (frame.scan_ranges > 0.0)
    )
    slot_indices = np.flatnonzero(valid)
    sources = frame.source_sensor[slot_indices]
    ranges = frame.scan_ranges[slot_indices]
    angles = frame.virtual_angles[slot_indices]
    points = np.empty((len(slot_indices), 2), dtype=np.float64)
    for sensor in (0, 1):
        selected = sources == sensor
        if not np.any(selected):
            continue
        pose = frame.robot_pose_map[sensor]
        bearing = pose[2] + angles[selected]
        points[selected, 0] = pose[0] + ranges[selected] * np.cos(bearing)
        points[selected, 1] = pose[1] + ranges[selected] * np.sin(bearing)

    rows, cols, inside = occupancy_map.world_to_grid(points)
    is_free = np.zeros(len(points), dtype=np.bool_)
    static_distance = np.full(len(points), np.nan, dtype=np.float64)
    mapped = np.flatnonzero(inside)
    is_free[mapped] = occupancy_map.free_mask[rows[mapped], cols[mapped]]
    static_distance[mapped] = occupancy_map.static_distance_m[
        rows[mapped], cols[mapped]
    ]
    candidate = (
        inside
        & is_free
        & np.isfinite(static_distance)
        & (static_distance > config.static_distance_threshold_m)
    )
    return PointCloudFrame(
        slot_indices=slot_indices,
        points_map=points,
        source_sensor=sources,
        ranges=ranges,
        static_distance_m=static_distance,
        candidate_mask=candidate,
        fusion_timestamp_ns=fusion_timestamp,
        sensor_time_skew_s=skew_s,
        fusion_mode=fusion_mode,
        active_sensor=active_sensor,
    )


def _deterministic_dbscan(
    points: np.ndarray, epsilon: float, min_points: int
) -> List[np.ndarray]:
    """Small deterministic DBSCAN implementation using a cKDTree."""

    points = np.asarray(points, dtype=np.float64).reshape((-1, 2))
    if not len(points):
        return []
    tree = cKDTree(points)
    neighbors = [
        np.asarray(sorted(tree.query_ball_point(points[index], epsilon)), dtype=np.int64)
        for index in range(len(points))
    ]
    core = np.asarray([len(items) >= min_points for items in neighbors])
    labels = np.full(len(points), -1, dtype=np.int64)
    cluster_id = 0
    for seed in range(len(points)):
        if not core[seed] or labels[seed] >= 0:
            continue
        labels[seed] = cluster_id
        queue = deque([seed])
        while queue:
            current = queue.popleft()
            for candidate in neighbors[current]:
                if labels[candidate] < 0:
                    labels[candidate] = cluster_id
                    if core[candidate]:
                        queue.append(int(candidate))
        cluster_id += 1
    return [np.flatnonzero(labels == index) for index in range(cluster_id)]


def cluster_candidate_points(
    cloud: PointCloudFrame, config: TrackerConfig
) -> List[LegCluster]:
    points = cloud.candidate_points
    candidate_slots = cloud.candidate_indices
    candidate_sources = cloud.source_sensor[cloud.candidate_mask]
    candidate_ranges = cloud.ranges[cloud.candidate_mask]
    candidate_static_distance = cloud.static_distance_m[cloud.candidate_mask]
    components = _deterministic_dbscan(
        points, config.cluster_epsilon_m, config.cluster_min_points
    )
    unsorted: List[LegCluster] = []
    for component in components:
        selected = points[component]
        centroid = selected.mean(axis=0)
        if len(selected) > 1:
            covariance = np.cov(selected, rowvar=False, ddof=1)
        else:
            covariance = np.eye(2, dtype=np.float64) * 1e-6
        covariance = np.asarray(covariance, dtype=np.float64).reshape((2, 2))
        extent = selected.max(axis=0) - selected.min(axis=0)
        width = float(np.linalg.norm(extent))
        source_mask = 0
        for sensor in np.unique(candidate_sources[component]):
            source_mask |= 1 << int(sensor)
        unsorted.append(
            LegCluster(
                cluster_id=-1,
                point_indices=candidate_slots[component].astype(np.int64),
                centroid=centroid,
                covariance=covariance,
                width=width,
                point_count=len(component),
                source_mask=source_mask,
                static_distance_min=float(
                    np.min(candidate_static_distance[component])
                ),
                static_distance_mean=float(
                    np.mean(candidate_static_distance[component])
                ),
                range_mean=float(np.mean(candidate_ranges[component])),
            )
        )
    unsorted.sort(
        key=lambda cluster: (
            float(cluster.centroid[0]),
            float(cluster.centroid[1]),
            int(np.min(cluster.point_indices)),
        )
    )
    for cluster_id, cluster in enumerate(unsorted):
        cluster.cluster_id = cluster_id
    return unsorted


def _measurement_covariance(mode: str, config: TrackerConfig) -> np.ndarray:
    sigma = {
        MEASUREMENT_DOUBLE: config.double_sigma_m,
        MEASUREMENT_SINGLE: config.single_sigma_m,
        MEASUREMENT_MERGED_BODY: config.merged_sigma_m,
    }[mode]
    return np.eye(2, dtype=np.float64) * sigma * sigma


def _measurement_confidence(
    mode: str, clusters: Sequence[LegCluster]
) -> float:
    base = {
        MEASUREMENT_SINGLE: 0.45,
        MEASUREMENT_DOUBLE: 0.80,
        MEASUREMENT_MERGED_BODY: 0.60,
    }[mode]
    if any(cluster.source_mask == 3 for cluster in clusters):
        base += 0.10
    point_bonus = min(0.10, 0.005 * sum(c.point_count for c in clusters))
    return float(min(1.0, base + point_bonus))


class _Track:
    def __init__(
        self,
        track_id: int,
        measurement: PersonMeasurement,
        timestamp_ns: int,
    ) -> None:
        self.track_id = int(track_id)
        self.state = np.asarray(
            [measurement.position[0], measurement.position[1], 0.0, 0.0],
            dtype=np.float64,
        )
        self.covariance = np.diag(
            [
                measurement.covariance[0, 0],
                measurement.covariance[1, 1],
                1.0,
                1.0,
            ]
        )
        self.track_state = TRACK_TENTATIVE
        self.created_ns = int(timestamp_ns)
        self.last_predict_ns = int(timestamp_ns)
        self.last_update_ns = int(timestamp_ns)
        self.age_s = 0.0
        self.time_since_update_s = 0.0
        self.visible_hit_count = 1
        self.consecutive_miss_count = 0
        self.hit_history: deque = deque([True], maxlen=10)
        self.measurement_history: deque = deque(
            [(int(timestamp_ns), measurement.position.copy())], maxlen=20
        )
        self.had_reliable_motion = False
        self.had_multileg_support = measurement.mode in (
            MEASUREMENT_DOUBLE,
            MEASUREMENT_MERGED_BODY,
        )
        self.support_mode: Optional[str] = measurement.mode
        self.support_cluster_ids = measurement.cluster_ids
        self.support_measurement_id: Optional[int] = measurement.measurement_id
        self.predicted_state = self.state.copy()
        self.innovation: Optional[np.ndarray] = np.zeros(2, dtype=np.float64)
        self.confidence = measurement.confidence * 0.6

    def priority_key(self) -> Tuple[int, float, int]:
        rank = {
            TRACK_CONFIRMED: 0,
            TRACK_COASTING: 1,
            TRACK_TENTATIVE: 2,
            TRACK_TENTATIVE_STATIC: 3,
        }.get(self.track_state, 4)
        return rank, -self.age_s, self.track_id

    def predict(self, timestamp_ns: int, acceleration_sigma: float) -> None:
        timestamp_ns = int(timestamp_ns)
        dt = max(0.0, (timestamp_ns - self.last_predict_ns) / 1e9)
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
        process_noise = acceleration_sigma * acceleration_sigma * np.asarray(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=np.float64,
        )
        self.state = transition @ self.state
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_predict_ns = timestamp_ns
        self.age_s = (timestamp_ns - self.created_ns) / 1e9
        self.time_since_update_s = (timestamp_ns - self.last_update_ns) / 1e9
        self.predicted_state = self.state.copy()
        self.innovation = None
        self.support_mode = None
        self.support_cluster_ids = ()
        self.support_measurement_id = None

    def gate(
        self,
        position: np.ndarray,
        measurement_covariance: np.ndarray,
        config: TrackerConfig,
    ) -> Tuple[bool, float, float, str]:
        observation = np.asarray(position, dtype=np.float64).reshape(2)
        innovation = observation - self.state[:2]
        euclidean = float(np.linalg.norm(innovation))
        innovation_covariance = (
            self.covariance[:2, :2] + measurement_covariance
        )
        try:
            solved = np.linalg.solve(innovation_covariance, innovation)
        except np.linalg.LinAlgError:
            return False, float("inf"), euclidean, "singular_innovation_covariance"
        mahalanobis = float(innovation.T @ solved)
        if euclidean > config.euclidean_gate_m:
            return False, mahalanobis, euclidean, "euclidean_gate"
        if mahalanobis > config.mahalanobis_gate_d2:
            return False, mahalanobis, euclidean, "mahalanobis_gate"
        return True, mahalanobis, euclidean, ""

    def update(
        self,
        measurement: PersonMeasurement,
        timestamp_ns: int,
        config: TrackerConfig,
    ) -> None:
        innovation = measurement.position - self.state[:2]
        innovation_covariance = self.covariance[:2, :2] + measurement.covariance
        kalman_gain = self.covariance[:, :2] @ np.linalg.inv(
            innovation_covariance
        )
        self.state = self.state + kalman_gain @ innovation
        identity = np.eye(4, dtype=np.float64)
        observation_matrix = np.asarray(
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
            dtype=np.float64,
        )
        joseph_left = identity - kalman_gain @ observation_matrix
        self.covariance = (
            joseph_left @ self.covariance @ joseph_left.T
            + kalman_gain @ measurement.covariance @ kalman_gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_update_ns = int(timestamp_ns)
        self.time_since_update_s = 0.0
        self.visible_hit_count += 1
        self.consecutive_miss_count = 0
        self.hit_history.append(True)
        self.measurement_history.append(
            (int(timestamp_ns), measurement.position.copy())
        )
        self.innovation = innovation.copy()
        self.support_mode = measurement.mode
        self.support_cluster_ids = measurement.cluster_ids
        self.support_measurement_id = measurement.measurement_id
        if measurement.mode in (MEASUREMENT_DOUBLE, MEASUREMENT_MERGED_BODY):
            self.had_multileg_support = True
        self._update_motion_evidence(config)
        recent_five = list(self.hit_history)[-5:]
        if len(recent_five) >= 3 and sum(recent_five) >= 3:
            if self.had_reliable_motion or self.had_multileg_support:
                self.track_state = TRACK_CONFIRMED
            elif self.age_s >= 0.3:
                self.track_state = TRACK_TENTATIVE_STATIC
        if self.track_state == TRACK_COASTING:
            self.track_state = TRACK_CONFIRMED
        if (
            self.track_state in (TRACK_TENTATIVE, TRACK_TENTATIVE_STATIC)
            and self.age_s > config.tentative_max_age_s
        ):
            self.track_state = TRACK_DELETED
        state_factor = 1.0 if self.track_state == TRACK_CONFIRMED else 0.65
        self.confidence = float(
            min(1.0, state_factor * (0.5 + 0.5 * measurement.confidence))
        )

    def _update_motion_evidence(self, config: TrackerConfig) -> None:
        if len(self.measurement_history) < 2:
            return
        _, first = self.measurement_history[0]
        _, last = self.measurement_history[-1]
        displacement = float(np.linalg.norm(last - first))
        speed = float(np.linalg.norm(self.state[2:]))
        if (
            displacement >= config.reliable_motion_displacement_m
            and speed >= config.reliable_motion_speed_mps
        ):
            self.had_reliable_motion = True

    def mark_missed(self, config: TrackerConfig) -> None:
        self.hit_history.append(False)
        self.consecutive_miss_count += 1
        if self.track_state in (TRACK_CONFIRMED, TRACK_COASTING):
            self.track_state = TRACK_COASTING
            self.confidence = float(
                max(
                    0.0,
                    self.confidence
                    * math.exp(
                        -self.time_since_update_s
                        / config.confirmed_coast_timeout_s
                    ),
                )
            )
            if self.time_since_update_s > config.confirmed_coast_timeout_s:
                self.track_state = TRACK_DELETED
            return
        if self.time_since_update_s > config.tentative_miss_timeout_s:
            self.track_state = TRACK_DELETED
        elif (
            self.track_state in (TRACK_TENTATIVE, TRACK_TENTATIVE_STATIC)
            and self.age_s > config.tentative_max_age_s
        ):
            self.track_state = TRACK_DELETED

    def snapshot(self) -> TrackSnapshot:
        return TrackSnapshot(
            track_id=self.track_id,
            track_state=self.track_state,
            predicted_state=self.predicted_state.copy(),
            updated_state=self.state.copy(),
            covariance=self.covariance.copy(),
            innovation=None if self.innovation is None else self.innovation.copy(),
            age_s=float(self.age_s),
            time_since_update_s=float(self.time_since_update_s),
            visible_hit_count=int(self.visible_hit_count),
            consecutive_miss_count=int(self.consecutive_miss_count),
            confidence=float(self.confidence),
            support_mode=self.support_mode,
            support_cluster_ids=tuple(self.support_cluster_ids),
            support_measurement_id=self.support_measurement_id,
            had_reliable_motion=bool(self.had_reliable_motion),
            had_multileg_support=bool(self.had_multileg_support),
        )


def _minimum_cost_disjoint_pairs(
    clusters: Sequence[LegCluster], config: TrackerConfig
) -> Tuple[List[Tuple[int, int]], List[int]]:
    """Exact deterministic set packing for the normally small newborn set."""

    clusters = list(clusters)
    count = len(clusters)
    if count == 0:
        return [], []
    if count > 20:
        raise RuntimeError(
            "more than 20 unassigned clusters; refusing a non-global pairing fallback"
        )
    pair_cost: Dict[Tuple[int, int], float] = {}
    for first in range(count):
        if (
            clusters[first].provisional_mode(config)
            == MEASUREMENT_MERGED_BODY
        ):
            continue
        for second in range(first + 1, count):
            if (
                clusters[second].provisional_mode(config)
                == MEASUREMENT_MERGED_BODY
            ):
                continue
            separation = float(
                np.linalg.norm(
                    clusters[first].centroid - clusters[second].centroid
                )
            )
            if not (
                config.double_leg_min_separation_m
                <= separation
                <= config.double_leg_max_separation_m
            ):
                continue
            cost = abs(separation - config.expected_leg_separation_m) / (
                config.double_leg_max_separation_m
                - config.double_leg_min_separation_m
            )
            pair_cost[(first, second)] = float(cost)

    @lru_cache(maxsize=None)
    def solve(mask: int):
        if mask == 0:
            return 0.0, (), ()
        first = (mask & -mask).bit_length() - 1
        remaining = mask & ~(1 << first)
        rest_cost, rest_pairs, rest_singles = solve(remaining)
        best = (
            config.newborn_pair_unmatched_cost + rest_cost,
            rest_pairs,
            (first,) + rest_singles,
        )
        for second in range(first + 1, count):
            if not (remaining & (1 << second)):
                continue
            key = (first, second)
            if key not in pair_cost:
                continue
            sub_cost, sub_pairs, sub_singles = solve(
                remaining & ~(1 << second)
            )
            candidate = (
                pair_cost[key] + sub_cost,
                ((first, second),) + sub_pairs,
                sub_singles,
            )
            if candidate < best:
                best = candidate
        return best

    _, pairs, singles = solve((1 << count) - 1)
    return list(pairs), list(singles)


class LidarPedestrianEstimator:
    """Map filter, deterministic clustering, and direct track-conditioned updates."""

    def __init__(
        self, occupancy_map: OccupancyMap, config: Optional[TrackerConfig] = None
    ) -> None:
        self.map = occupancy_map
        self.config = config or TrackerConfig()
        self.config.validate()
        self.tracks: List[_Track] = []
        self.next_track_id = 1
        self.next_measurement_id = 1
        self.last_timestamp_ns: Optional[int] = None

    def _new_measurement(
        self,
        mode: str,
        clusters: Sequence[LegCluster],
        conditioned_track_id: Optional[int] = None,
    ) -> PersonMeasurement:
        if mode == MEASUREMENT_DOUBLE:
            position = 0.5 * (clusters[0].centroid + clusters[1].centroid)
        else:
            position = clusters[0].centroid.copy()
        measurement = PersonMeasurement(
            measurement_id=self.next_measurement_id,
            mode=mode,
            cluster_ids=tuple(cluster.cluster_id for cluster in clusters),
            position=np.asarray(position, dtype=np.float64),
            covariance=_measurement_covariance(mode, self.config),
            confidence=_measurement_confidence(mode, clusters),
            conditioned_track_id=conditioned_track_id,
        )
        self.next_measurement_id += 1
        return measurement

    def _global_primary_assignment(
        self,
        tracks: Sequence[_Track],
        clusters: Sequence[LegCluster],
        records: List[AssociationRecord],
    ) -> Dict[int, int]:
        if not tracks or not clusters:
            return {}
        invalid = 1e12
        costs = np.full((len(tracks), len(clusters)), invalid, dtype=np.float64)
        details: Dict[Tuple[int, int], Tuple[bool, float, float, str]] = {}
        for row, track in enumerate(tracks):
            for col, cluster in enumerate(clusters):
                mode = cluster.provisional_mode(self.config)
                covariance = _measurement_covariance(mode, self.config)
                gate, d2, euclidean, reason = track.gate(
                    cluster.centroid, covariance, self.config
                )
                details[(row, col)] = (gate, d2, euclidean, reason)
                if gate:
                    costs[row, col] = (
                        d2
                        + self.config.tie_break_epsilon
                        * (row + 1)
                        * (len(clusters) - col)
                    )
        rows, cols = linear_sum_assignment(costs)
        assigned: Dict[int, int] = {}
        assigned_pairs = {
            (int(row), int(col))
            for row, col in zip(rows, cols)
            if costs[row, col] < invalid
        }
        for row, track in enumerate(tracks):
            for col, cluster in enumerate(clusters):
                gate, d2, euclidean, reason = details[(row, col)]
                selected = (row, col) in assigned_pairs
                records.append(
                    AssociationRecord(
                        stage="primary_cluster",
                        track_id=track.track_id,
                        cluster_id=cluster.cluster_id,
                        measurement_id=None,
                        mahalanobis_d2=d2,
                        euclidean_distance=euclidean,
                        gate_passed=gate,
                        assignment_result="assigned" if selected else "not_assigned",
                        rejection_reason=reason,
                    )
                )
                if selected:
                    assigned[track.track_id] = cluster.cluster_id
        return assigned

    def _global_auxiliary_assignment(
        self,
        tracks_by_id: Dict[int, _Track],
        clusters_by_id: Dict[int, LegCluster],
        primary: Dict[int, int],
        available_cluster_ids: Sequence[int],
        records: List[AssociationRecord],
    ) -> Dict[int, int]:
        eligible_tracks = [
            track_id
            for track_id in sorted(primary)
            if clusters_by_id[primary[track_id]].provisional_mode(self.config)
            != MEASUREMENT_MERGED_BODY
        ]
        available = sorted(available_cluster_ids)
        if not eligible_tracks or not available:
            return {}
        invalid = 1e12
        costs = np.full(
            (len(eligible_tracks), len(available)), invalid, dtype=np.float64
        )
        details: Dict[Tuple[int, int], Tuple[bool, float, float, str]] = {}
        for row, track_id in enumerate(eligible_tracks):
            track = tracks_by_id[track_id]
            primary_cluster = clusters_by_id[primary[track_id]]
            for col, cluster_id in enumerate(available):
                cluster = clusters_by_id[cluster_id]
                separation = float(
                    np.linalg.norm(cluster.centroid - primary_cluster.centroid)
                )
                if (
                    cluster.provisional_mode(self.config)
                    == MEASUREMENT_MERGED_BODY
                    or separation < self.config.double_leg_min_separation_m
                    or separation > self.config.double_leg_max_separation_m
                ):
                    details[(row, col)] = (
                        False,
                        float("inf"),
                        separation,
                        "leg_separation",
                    )
                    continue
                gate, d2, euclidean, reason = track.gate(
                    cluster.centroid,
                    _measurement_covariance(MEASUREMENT_SINGLE, self.config),
                    self.config,
                )
                details[(row, col)] = (gate, d2, euclidean, reason)
                if gate:
                    separation_cost = abs(
                        separation - self.config.expected_leg_separation_m
                    )
                    costs[row, col] = (
                        d2
                        + separation_cost
                        + self.config.tie_break_epsilon
                        * (row + 1)
                        * (len(available) - col)
                    )
        rows, cols = linear_sum_assignment(costs)
        selected_pairs = {
            (int(row), int(col))
            for row, col in zip(rows, cols)
            if costs[row, col] < invalid
        }
        auxiliary: Dict[int, int] = {}
        for row, track_id in enumerate(eligible_tracks):
            for col, cluster_id in enumerate(available):
                gate, d2, euclidean, reason = details.get(
                    (row, col),
                    (False, float("inf"), float("inf"), "not_evaluated"),
                )
                selected = (row, col) in selected_pairs
                records.append(
                    AssociationRecord(
                        stage="auxiliary_cluster",
                        track_id=track_id,
                        cluster_id=cluster_id,
                        measurement_id=None,
                        mahalanobis_d2=d2,
                        euclidean_distance=euclidean,
                        gate_passed=gate,
                        assignment_result="assigned" if selected else "not_assigned",
                        rejection_reason=reason,
                    )
                )
                if selected:
                    auxiliary[track_id] = cluster_id
        return auxiliary

    def process(self, frame: EstimatorFrame) -> EstimatorResult:
        timestamp_ns = max(
            int(frame.scan_timestamp_lidar_1),
            int(frame.scan_timestamp_lidar_2),
        )
        if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
            raise ValueError("estimator frame timestamps must be strictly increasing")
        self.last_timestamp_ns = timestamp_ns
        cloud = frame_points_map(frame, self.map, self.config)
        clusters = cluster_candidate_points(cloud, self.config)
        clusters_by_id = {cluster.cluster_id: cluster for cluster in clusters}
        associations: List[AssociationRecord] = []
        measurements: List[PersonMeasurement] = []

        for track in sorted(self.tracks, key=lambda item: item.track_id):
            track.predict(
                timestamp_ns, self.config.process_acceleration_sigma_mps2
            )
        ordered_tracks = sorted(self.tracks, key=lambda item: item.priority_key())
        tracks_by_id = {track.track_id: track for track in ordered_tracks}
        primary = self._global_primary_assignment(
            ordered_tracks, clusters, associations
        )
        used_cluster_ids = set(primary.values())
        auxiliary = self._global_auxiliary_assignment(
            tracks_by_id=tracks_by_id,
            clusters_by_id=clusters_by_id,
            primary=primary,
            available_cluster_ids=[
                cluster.cluster_id
                for cluster in clusters
                if cluster.cluster_id not in used_cluster_ids
            ],
            records=associations,
        )
        used_cluster_ids.update(auxiliary.values())
        updated_track_ids = set()
        for track_id in sorted(primary):
            track = tracks_by_id[track_id]
            primary_cluster = clusters_by_id[primary[track_id]]
            support = [primary_cluster]
            if track_id in auxiliary:
                support.append(clusters_by_id[auxiliary[track_id]])
                mode = MEASUREMENT_DOUBLE
            else:
                mode = primary_cluster.provisional_mode(self.config)
            measurement = self._new_measurement(
                mode, support, conditioned_track_id=track_id
            )
            measurements.append(measurement)
            gate, d2, euclidean, reason = track.gate(
                measurement.position, measurement.covariance, self.config
            )
            if gate:
                track.update(measurement, timestamp_ns, self.config)
                updated_track_ids.add(track_id)
                result = "direct_update"
            else:
                result = "measurement_rejected"
            associations.append(
                AssociationRecord(
                    stage="conditioned_measurement",
                    track_id=track_id,
                    cluster_id=None,
                    measurement_id=measurement.measurement_id,
                    mahalanobis_d2=d2,
                    euclidean_distance=euclidean,
                    gate_passed=gate,
                    assignment_result=result,
                    rejection_reason=reason,
                )
            )

        for track in ordered_tracks:
            if track.track_id not in updated_track_ids:
                track.mark_missed(self.config)

        remaining = [
            cluster
            for cluster in clusters
            if cluster.cluster_id not in used_cluster_ids
        ]
        pairs, singles = _minimum_cost_disjoint_pairs(remaining, self.config)
        newborn_measurements: List[PersonMeasurement] = []
        paired_indices = {index for pair in pairs for index in pair}
        if paired_indices.intersection(singles):
            raise AssertionError("newborn pair and single sets overlap")
        for first, second in pairs:
            measurement = self._new_measurement(
                MEASUREMENT_DOUBLE, [remaining[first], remaining[second]]
            )
            newborn_measurements.append(measurement)
            used_cluster_ids.update(measurement.cluster_ids)
            associations.append(
                AssociationRecord(
                    stage="newborn_pair",
                    track_id=None,
                    cluster_id=None,
                    measurement_id=measurement.measurement_id,
                    mahalanobis_d2=None,
                    euclidean_distance=float(
                        np.linalg.norm(
                            remaining[first].centroid
                            - remaining[second].centroid
                        )
                    ),
                    gate_passed=True,
                    assignment_result="newborn_measurement",
                )
            )
        for index in singles:
            cluster = remaining[index]
            mode = cluster.provisional_mode(self.config)
            measurement = self._new_measurement(mode, [cluster])
            newborn_measurements.append(measurement)
            used_cluster_ids.add(cluster.cluster_id)
            associations.append(
                AssociationRecord(
                    stage="newborn_single",
                    track_id=None,
                    cluster_id=cluster.cluster_id,
                    measurement_id=measurement.measurement_id,
                    mahalanobis_d2=None,
                    euclidean_distance=None,
                    gate_passed=True,
                    assignment_result="newborn_measurement",
                )
            )
        measurements.extend(newborn_measurements)
        for measurement in newborn_measurements:
            track = _Track(self.next_track_id, measurement, timestamp_ns)
            self.next_track_id += 1
            self.tracks.append(track)

        if len(used_cluster_ids) != len(set(used_cluster_ids)):
            raise AssertionError("cluster assignment is not globally exclusive")
        referenced = [
            cluster_id
            for measurement in measurements
            for cluster_id in measurement.cluster_ids
        ]
        if len(referenced) != len(set(referenced)):
            raise AssertionError(
                "a cluster supports more than one person measurement"
            )
        snapshots = [
            track.snapshot()
            for track in sorted(self.tracks, key=lambda item: item.track_id)
        ]
        self.tracks = [
            track for track in self.tracks if track.track_state != TRACK_DELETED
        ]
        return EstimatorResult(
            frame_index=frame.frame_index,
            fusion_timestamp_ns=cloud.fusion_timestamp_ns,
            sensor_time_skew_s=cloud.sensor_time_skew_s,
            fusion_mode=cloud.fusion_mode,
            active_sensor=cloud.active_sensor,
            point_cloud=cloud,
            clusters=clusters,
            measurements=measurements,
            associations=associations,
            tracks=snapshots,
        )
