#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--cluster-epsilon-m, --cluster-min-points, --dataset, --high-dynamics-acceleration-mps2, --map-yaml, --max-frames, --max-sensor-time-skew-s, --max-truth-interpolation-gap-s, --output-root, --person-match-radius-m, --run-name, --static-distance-threshold-m, --velocity-fit-half-window-s, --velocity-fit-max-residual-m, --velocity-fit-min-samples, --velocity-fit-min-span-s, --velocity-track-min-age-s, --write-figure
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PNG, TXT, WORLD
# 可能使用的关键环境变量：ALLOWED_ESTIMATOR_INPUTS, FUSION_DUAL, MEASUREMENT_DOUBLE, MEASUREMENT_MERGED_BODY, MEASUREMENT_SINGLE, RANGE_BINS, TRACK_COASTING, TRACK_CONFIRMED, TRACK_DELETED, YAML
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/evaluate_lidar_pedestrian_tracking.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-24 08:08:02.216629659 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.914035757 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/evaluate_lidar_pedestrian_tracking.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜evaluate_lidar_pedestrian_tracking.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Evaluate truth-independent dual-2D-LiDAR pedestrian tracking offline.

The estimator receives only EstimatorFrame instances.  This outer evaluator is
the sole owner of semantic labels and pedestrian ground truth.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import linear_sum_assignment

from lidar_pedestrian_tracking_core import (
    EstimatorFrame,
    EstimatorResult,
    FUSION_DUAL,
    LidarPedestrianEstimator,
    MEASUREMENT_DOUBLE,
    MEASUREMENT_MERGED_BODY,
    MEASUREMENT_SINGLE,
    OccupancyMap,
    TRACK_COASTING,
    TRACK_CONFIRMED,
    TRACK_DELETED,
    TrackerConfig,
)


ALLOWED_ESTIMATOR_INPUTS = (
    "scan_ranges",
    "virtual_angles",
    "valid_mask",
    "source_sensor",
    "robot_pose_map",
    "scan_timestamp_lidar_1",
    "scan_timestamp_lidar_2",
)
RANGE_BINS = (
    ("0-2", 0.0, 2.0),
    ("2-4", 2.0, 4.0),
    ("4-6", 4.0, 6.0),
    ("6-8", 6.0, 8.0),
    ("8+", 8.0, float("inf")),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--run-name", default="")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--static-distance-threshold-m", type=float, default=0.25)
    parser.add_argument("--cluster-epsilon-m", type=float, default=0.10)
    parser.add_argument("--cluster-min-points", type=int, default=3)
    parser.add_argument("--max-sensor-time-skew-s", type=float, default=0.01)
    parser.add_argument("--person-match-radius-m", type=float, default=0.5)
    parser.add_argument("--velocity-fit-half-window-s", type=float, default=0.25)
    parser.add_argument("--velocity-fit-min-samples", type=int, default=5)
    parser.add_argument("--velocity-fit-min-span-s", type=float, default=0.30)
    parser.add_argument("--max-truth-interpolation-gap-s", type=float, default=0.15)
    parser.add_argument("--velocity-fit-max-residual-m", type=float, default=0.12)
    parser.add_argument("--high-dynamics-acceleration-mps2", type=float, default=3.0)
    parser.add_argument("--velocity-track-min-age-s", type=float, default=0.5)
    parser.add_argument(
        "--write-figure", action=argparse.BooleanOptionalAction, default=True
    )
    return parser.parse_args()


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def json_value(value, digits: int = 9):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist(), digits)
    if isinstance(value, np.generic):
        return json_value(value.item(), digits)
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return round(value, digits)
    if isinstance(value, dict):
        return {str(key): json_value(item, digits) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item, digits) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path: Path, value) -> None:
    path.write_text(
        json.dumps(json_value(value), indent=2, sort_keys=True, ensure_ascii=False)
        + "\n",
        encoding="utf-8",
    )


def write_jsonl_record(stream, value) -> None:
    stream.write(
        json.dumps(
            json_value(value),
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
        + "\n"
    )


def safe_rate(numerator: float, denominator: float) -> Optional[float]:
    if denominator <= 0:
        return None
    return float(numerator / denominator)


def rmse(vectors: Sequence[np.ndarray]) -> Optional[float]:
    if not vectors:
        return None
    array = np.asarray(vectors, dtype=np.float64).reshape((-1, 2))
    return float(np.sqrt(np.mean(np.sum(array * array, axis=1))))


def bias(vectors: Sequence[np.ndarray]) -> Optional[List[float]]:
    if not vectors:
        return None
    return np.mean(np.asarray(vectors, dtype=np.float64), axis=0).tolist()


def range_bin_name(distance: float) -> str:
    for name, low, high in RANGE_BINS:
        if low <= distance < high:
            return name
    return "8+"


def match_positions(
    estimates: np.ndarray,
    truth: np.ndarray,
    max_distance: float,
) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
    estimates = np.asarray(estimates, dtype=np.float64).reshape((-1, 2))
    truth = np.asarray(truth, dtype=np.float64).reshape((-1, 2))
    if not len(estimates) or not len(truth):
        return [], list(range(len(estimates))), list(range(len(truth)))
    distances = np.linalg.norm(
        estimates[:, np.newaxis, :] - truth[np.newaxis, :, :], axis=2
    )
    rows_rank = np.arange(1, len(estimates) + 1, dtype=np.float64)[:, None]
    columns_rank = np.arange(len(truth), 0, -1, dtype=np.float64)[None, :]
    epsilon = 1e-12 * rows_rank * columns_rank
    rows, cols = linear_sum_assignment(distances + epsilon)
    matches = [
        (int(row), int(col), float(distances[row, col]))
        for row, col in zip(rows, cols)
        if distances[row, col] <= max_distance
    ]
    matched_estimates = {item[0] for item in matches}
    matched_truth = {item[1] for item in matches}
    return (
        matches,
        [index for index in range(len(estimates)) if index not in matched_estimates],
        [index for index in range(len(truth)) if index not in matched_truth],
    )


def interpolate_pose(
    target_ns: int,
    timestamps_ns: np.ndarray,
    positions: np.ndarray,
    unwrapped_yaw: np.ndarray,
) -> np.ndarray:
    target = float(target_ns)
    x = np.interp(target, timestamps_ns, positions[:, 0])
    y = np.interp(target, timestamps_ns, positions[:, 1])
    yaw = np.interp(target, timestamps_ns, unwrapped_yaw)
    return np.asarray([x, y, math.atan2(math.sin(yaw), math.cos(yaw))])


class DatasetReader:
    """Separates estimator-array extraction from evaluator-only ground truth."""

    def __init__(self, root: Path, map_yaml: Optional[Path], max_frames: int) -> None:
        self.root = Path(root).resolve()
        metadata_path = self.root / "metadata.json"
        if not metadata_path.is_file():
            raise FileNotFoundError(f"missing dataset metadata: {metadata_path}")
        self.metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if self.metadata.get("format") != "semantic2d-fixed-dual-native-v3":
            raise ValueError(
                f"unsupported dataset format: {self.metadata.get('format')!r}"
            )
        self.frames = list(self.metadata.get("frames", []))
        if max_frames > 0:
            self.frames = self.frames[:max_frames]
        if not self.frames:
            raise ValueError("dataset contains no frames")
        required_estimator_dirs = (
            "virtual_ranges_lidar",
            "virtual_angles_lidar",
            "valid_mask_lidar",
            "source_sensor",
            "positions",
        )
        required_evaluator_dirs = (
            "semantic_label",
            "pedestrian_ids",
            "pedestrian_positions",
            "pedestrian_leg_positions",
            "pedestrian_velocities",
            "pedestrian_truth_timestamps",
        )
        for directory in required_estimator_dirs + required_evaluator_dirs:
            if not (self.root / directory).is_dir():
                raise FileNotFoundError(
                    f"missing required dataset field directory: {directory}"
                )
        self.estimator_field_sources = {
            "scan_ranges": "virtual_ranges_lidar",
            "virtual_angles": "virtual_angles_lidar",
            "valid_mask": "valid_mask_lidar",
            "source_sensor": "source_sensor",
            "robot_pose_map": "positions",
            "scan_timestamp_lidar_1": "metadata.frames[].scan_01_stamp_ns",
            "scan_timestamp_lidar_2": "metadata.frames[].scan_02_stamp_ns",
        }
        if tuple(self.estimator_field_sources) != ALLOWED_ESTIMATOR_INPUTS:
            raise AssertionError("estimator input whitelist changed unexpectedly")
        self.scan_1_timestamps = np.asarray(
            [int(frame["scan_01_stamp_ns"]) for frame in self.frames],
            dtype=np.int64,
        )
        self.scan_2_timestamps = np.asarray(
            [int(frame["scan_02_stamp_ns"]) for frame in self.frames],
            dtype=np.int64,
        )
        self.robot_poses_at_scan_1 = np.stack(
            [
                np.load(self.root / "positions" / frame["name"]).astype(np.float64)
                for frame in self.frames
            ]
        )
        if self.robot_poses_at_scan_1.shape != (len(self.frames), 3):
            raise ValueError("positions must contain one [x,y,yaw] pose per frame")
        self.unwrapped_robot_yaw = np.unwrap(self.robot_poses_at_scan_1[:, 2])
        metadata_map = Path(str(self.metadata.get("map_yaml", "")))
        if not metadata_map.is_file():
            source_session = Path(
                str(self.metadata.get("source_npz_session", ""))
            )
            source_metadata_path = source_session / "metadata.json"
            if source_metadata_path.is_file():
                source_metadata = json.loads(
                    source_metadata_path.read_text(encoding="utf-8")
                )
                metadata_map = Path(str(source_metadata.get("map_yaml", "")))
        chosen_map = Path(map_yaml) if map_yaml is not None else metadata_map
        if not chosen_map.is_file():
            raise FileNotFoundError(
                "map YAML was not provided and metadata map_yaml is unavailable: "
                f"{chosen_map}"
            )
        self.map_yaml = chosen_map.resolve()
        names = list(self.metadata.get("label_names", []))
        if not names:
            label_names_path = self.root / "label_names.txt"
            names = [
                line.strip()
                for line in label_names_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        if "Person" not in names:
            raise ValueError("dataset label names do not contain Person")
        self.person_label_id = names.index("Person")

    def estimator_frame(self, index: int) -> EstimatorFrame:
        frame = self.frames[index]
        name = frame["name"]
        timestamp_1 = int(frame["scan_01_stamp_ns"])
        timestamp_2 = int(frame["scan_02_stamp_ns"])
        pose_1 = interpolate_pose(
            timestamp_1,
            self.scan_1_timestamps,
            self.robot_poses_at_scan_1,
            self.unwrapped_robot_yaw,
        )
        pose_2 = interpolate_pose(
            timestamp_2,
            self.scan_1_timestamps,
            self.robot_poses_at_scan_1,
            self.unwrapped_robot_yaw,
        )
        return EstimatorFrame(
            frame_index=index,
            scan_ranges=np.load(self.root / "virtual_ranges_lidar" / name),
            virtual_angles=np.load(self.root / "virtual_angles_lidar" / name),
            valid_mask=np.load(self.root / "valid_mask_lidar" / name),
            source_sensor=np.load(self.root / "source_sensor" / name),
            robot_pose_map=np.stack((pose_1, pose_2)),
            scan_timestamp_lidar_1=timestamp_1,
            scan_timestamp_lidar_2=timestamp_2,
        )

    def truth_frame(self, index: int) -> Dict[str, object]:
        name = self.frames[index]["name"]
        return {
            "ids": np.load(self.root / "pedestrian_ids" / name).astype(str),
            "positions": np.load(
                self.root / "pedestrian_positions" / name
            ).astype(np.float64),
            "leg_positions": np.load(
                self.root / "pedestrian_leg_positions" / name
            ).astype(np.float64),
            "raw_velocities": np.load(
                self.root / "pedestrian_velocities" / name
            ).astype(np.float64),
            "truth_timestamp_ns": int(
                np.load(self.root / "pedestrian_truth_timestamps" / name)
            ),
            "semantic_label": np.load(self.root / "semantic_label" / name),
        }


def build_data_audit(reader: DatasetReader, occupancy_map: OccupancyMap) -> Dict:
    scan_timestamps = reader.scan_1_timestamps
    scan_dt = np.diff(scan_timestamps) / 1e9
    sensor_skew_s = np.abs(
        reader.scan_1_timestamps - reader.scan_2_timestamps
    ) / 1e9
    poses = reader.robot_poses_at_scan_1
    yaw = reader.unwrapped_robot_yaw
    position_steps = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    yaw_steps = np.abs(np.diff(yaw))
    id_sets = []
    duplicate_id_frames = []
    truth_timestamps = []
    truth_by_frame = []
    for index in range(len(reader.frames)):
        truth = reader.truth_frame(index)
        ids = truth["ids"].tolist()
        if len(ids) != len(set(ids)):
            duplicate_id_frames.append(index)
        id_sets.append(tuple(sorted(ids)))
        truth_timestamps.append(int(truth["truth_timestamp_ns"]))
        truth_by_frame.append(truth)
    id_set_change_frames = [
        index
        for index in range(1, len(id_sets))
        if id_sets[index] != id_sets[index - 1]
    ]
    unique_truth = np.unique(np.asarray(truth_timestamps, dtype=np.int64))
    truth_gaps = np.diff(unique_truth) / 1e9
    positions_by_truth_stamp: Dict[
        Tuple[str, int], List[np.ndarray]
    ] = defaultdict(list)
    for truth in truth_by_frame:
        truth_stamp = int(truth["truth_timestamp_ns"])
        for offset, pid in enumerate(truth["ids"].tolist()):
            positions_by_truth_stamp[(pid, truth_stamp)].append(
                truth["positions"][offset]
            )
    conflicting_truth_position_groups = 0
    conflicting_truth_position_max_spread_m = 0.0
    for stamped_positions in positions_by_truth_stamp.values():
        if len(stamped_positions) < 2:
            continue
        array = np.stack(stamped_positions)
        spread = float(
            np.max(np.linalg.norm(array - array[0], axis=1))
        )
        if spread > 1e-6:
            conflicting_truth_position_groups += 1
            conflicting_truth_position_max_spread_m = max(
                conflicting_truth_position_max_spread_m, spread
            )

    finite_difference_speeds = []
    recorded_speeds = []
    speed_ratios = []
    direction_cosines = []
    for index in range(len(truth_by_frame) - 1):
        first = truth_by_frame[index]
        second = truth_by_frame[index + 1]
        first_lookup = {
            pid: offset for offset, pid in enumerate(first["ids"].tolist())
        }
        second_lookup = {
            pid: offset for offset, pid in enumerate(second["ids"].tolist())
        }
        dt = scan_dt[index]
        for pid in sorted(set(first_lookup).intersection(second_lookup)):
            first_index = first_lookup[pid]
            second_index = second_lookup[pid]
            derivative = (
                second["positions"][second_index]
                - first["positions"][first_index]
            ) / dt
            recorded = 0.5 * (
                first["raw_velocities"][first_index]
                + second["raw_velocities"][second_index]
            )
            derivative_speed = float(np.linalg.norm(derivative))
            recorded_speed = float(np.linalg.norm(recorded))
            if derivative_speed <= 0.05 or recorded_speed <= 0.05:
                continue
            finite_difference_speeds.append(derivative_speed)
            recorded_speeds.append(recorded_speed)
            speed_ratios.append(derivative_speed / recorded_speed)
            direction_cosines.append(
                float(
                    derivative.dot(recorded)
                    / (derivative_speed * recorded_speed)
                )
            )

    def percentiles(values: Sequence[float]) -> Optional[Dict[str, float]]:
        if not values:
            return None
        result = np.percentile(values, [0, 5, 50, 95, 99, 100])
        return dict(zip(("min", "p05", "p50", "p95", "p99", "max"), result))

    return {
        "dataset": str(reader.root),
        "dataset_format": reader.metadata.get("format"),
        "development_evaluation_bag": True,
        "estimator_input_whitelist": list(ALLOWED_ESTIMATOR_INPUTS),
        "estimator_field_sources": reader.estimator_field_sources,
        "range_source_note": (
            "scan_ranges is extracted from virtual_ranges_lidar so base-frame "
            "virtual angles and ranges reconstruct identical endpoints"
        ),
        "map": occupancy_map.audit_dict(),
        "frame_count": len(reader.frames),
        "scan_time": {
            "strictly_increasing": bool(np.all(scan_dt > 0.0)),
            "dt_s": percentiles(scan_dt.tolist()),
            "sensor_time_skew_s": percentiles(sensor_skew_s.tolist()),
            "frames_over_default_skew_limit": int(
                np.sum(sensor_skew_s > 0.01)
            ),
        },
        "robot_pose_map": {
            "max_position_step_m": float(position_steps.max(initial=0.0)),
            "max_implied_speed_mps": float(
                np.max(position_steps / scan_dt) if len(scan_dt) else 0.0
            ),
            "max_unwrapped_yaw_step_rad": float(yaw_steps.max(initial=0.0)),
            "pose_continuity_verified": bool(
                np.all(scan_dt > 0.0)
                and position_steps.max(initial=0.0) < 0.5
                and yaw_steps.max(initial=0.0) < math.pi / 2.0
            ),
        },
        "pedestrian_ids": {
            "persistent_ids": list(id_sets[0]),
            "persistent_id_count": len(id_sets[0]),
            "duplicate_id_frames": duplicate_id_frames,
            "id_set_change_frames": id_set_change_frames,
            "stable_across_frames": not duplicate_id_frames
            and not id_set_change_frames,
        },
        "pedestrian_truth_time": {
            "sample_count": len(truth_timestamps),
            "unique_timestamp_count": len(unique_truth),
            "repeated_scan_matches": len(truth_timestamps) - len(unique_truth),
            "unique_gap_s": percentiles(truth_gaps.tolist()),
            "conflicting_duplicate_position_groups": (
                conflicting_truth_position_groups
            ),
            "conflicting_duplicate_position_max_spread_m": (
                conflicting_truth_position_max_spread_m
            ),
            "velocity_fit_time_basis": "scan_timestamp",
            "velocity_fit_time_basis_reason": (
                "map-frame positions can differ under one repeated source truth "
                "timestamp; scan time is the only unique point-cloud-aligned axis"
            ),
        },
        "recorded_velocity_consistency": {
            "finite_difference_speed_mps": percentiles(
                finite_difference_speeds
            ),
            "recorded_speed_mps": percentiles(recorded_speeds),
            "finite_difference_over_recorded_ratio": percentiles(speed_ratios),
            "direction_cosine": percentiles(direction_cosines),
            "raw_pedestrian_velocity_is_reference": False,
        },
    }


@dataclass(frozen=True)
class VelocityReference:
    velocity_map: np.ndarray
    acceleration_map: np.ndarray
    fit_residual_m: float
    fit_time_span_s: float
    fit_sample_count: int
    max_time_gap_s: float
    nearest_truth_delta_s: float
    reference_valid: bool
    main_metric_valid: bool
    edge_reference: bool
    high_dynamics: bool
    source_timestamp_conflict: bool


class VelocityReferenceBuilder:
    def __init__(
        self,
        reader: DatasetReader,
        half_window_s: float,
        min_samples: int,
        min_span_s: float,
        max_gap_s: float,
        max_residual_m: float,
        high_dynamics_acceleration_mps2: float,
    ) -> None:
        self.half_window_s = float(half_window_s)
        self.min_samples = int(min_samples)
        self.min_span_s = float(min_span_s)
        self.max_gap_s = float(max_gap_s)
        self.max_residual_m = float(max_residual_m)
        self.high_dynamics_acceleration_mps2 = float(
            high_dynamics_acceleration_mps2
        )
        samples: Dict[str, Dict[int, Tuple[np.ndarray, int]]] = defaultdict(dict)
        positions_by_source_stamp: Dict[
            str, Dict[int, List[np.ndarray]]
        ] = defaultdict(lambda: defaultdict(list))
        for index in range(len(reader.frames)):
            truth = reader.truth_frame(index)
            scan_stamp = int(reader.scan_1_timestamps[index])
            source_stamp = int(truth["truth_timestamp_ns"])
            for offset, pid in enumerate(truth["ids"].tolist()):
                position = truth["positions"][offset].copy()
                samples[pid][scan_stamp] = (position, source_stamp)
                positions_by_source_stamp[pid][source_stamp].append(position)
        self.conflicting_source_stamps: Dict[str, set[int]] = defaultdict(set)
        for pid, by_source_stamp in positions_by_source_stamp.items():
            for source_stamp, stamped_positions in by_source_stamp.items():
                if len(stamped_positions) < 2:
                    continue
                array = np.stack(stamped_positions)
                if np.max(np.linalg.norm(array - array[0], axis=1)) > 1e-6:
                    self.conflicting_source_stamps[pid].add(source_stamp)
        self.trajectories: Dict[
            str, Tuple[np.ndarray, np.ndarray, np.ndarray]
        ] = {}
        for pid, by_time in samples.items():
            stamps = np.asarray(sorted(by_time), dtype=np.int64)
            positions = np.stack(
                [by_time[int(stamp)][0] for stamp in stamps]
            )
            source_stamps = np.asarray(
                [by_time[int(stamp)][1] for stamp in stamps],
                dtype=np.int64,
            )
            self.trajectories[pid] = (stamps, positions, source_stamps)
        self.cache: Dict[Tuple[str, int], VelocityReference] = {}

    @staticmethod
    def _huber_quadratic(
        relative_time_s: np.ndarray, positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        design = np.column_stack(
            (
                np.ones(len(relative_time_s)),
                relative_time_s,
                relative_time_s * relative_time_s,
            )
        )
        weights = np.ones(len(relative_time_s), dtype=np.float64)
        coefficients = np.linalg.lstsq(design, positions, rcond=None)[0]
        for _ in range(20):
            prediction = design @ coefficients
            residual_vector = positions - prediction
            residual_norm = np.linalg.norm(residual_vector, axis=1)
            median = float(np.median(residual_norm))
            scale = max(1e-6, median / 0.6745)
            delta = 1.345 * scale
            next_weights = np.ones_like(weights)
            large = residual_norm > delta
            next_weights[large] = delta / residual_norm[large]
            weighted_design = design * np.sqrt(next_weights)[:, np.newaxis]
            weighted_positions = positions * np.sqrt(next_weights)[:, np.newaxis]
            next_coefficients = np.linalg.lstsq(
                weighted_design, weighted_positions, rcond=None
            )[0]
            if np.max(np.abs(next_coefficients - coefficients)) < 1e-9:
                coefficients = next_coefficients
                weights = next_weights
                break
            coefficients = next_coefficients
            weights = next_weights
        final_residual = positions - design @ coefficients
        rms = float(
            np.sqrt(np.mean(np.sum(final_residual * final_residual, axis=1)))
        )
        velocity = coefficients[1]
        acceleration = 2.0 * coefficients[2]
        return velocity, acceleration, rms

    def reference(self, pid: str, target_ns: int) -> VelocityReference:
        key = (str(pid), int(target_ns))
        if key in self.cache:
            return self.cache[key]
        stamps_ns, positions, source_stamps_ns = self.trajectories[str(pid)]
        times_s = stamps_ns.astype(np.float64) / 1e9
        target_s = float(target_ns) / 1e9
        edge = (
            target_s - times_s[0] < self.half_window_s
            or times_s[-1] - target_s < self.half_window_s
        )
        if target_s - times_s[0] < self.half_window_s:
            selected = (times_s >= target_s) & (
                times_s <= target_s + 2.0 * self.half_window_s
            )
        elif times_s[-1] - target_s < self.half_window_s:
            selected = (times_s <= target_s) & (
                times_s >= target_s - 2.0 * self.half_window_s
            )
        else:
            selected = np.abs(times_s - target_s) <= self.half_window_s
        indices = np.flatnonzero(selected)
        count = len(indices)
        if count:
            selected_times = times_s[indices]
            span = float(selected_times[-1] - selected_times[0])
            selected_source_stamps = source_stamps_ns[indices]
            unique_source_stamps = np.unique(selected_source_stamps)
            source_gaps = np.diff(unique_source_stamps.astype(np.float64)) / 1e9
            max_gap = float(source_gaps.max(initial=0.0))
            nearest_scan_index = indices[
                int(np.argmin(np.abs(selected_times - target_s)))
            ]
            nearest_delta = abs(
                int(source_stamps_ns[nearest_scan_index]) - int(target_ns)
            ) / 1e9
            source_conflict = any(
                int(stamp) in self.conflicting_source_stamps[str(pid)]
                for stamp in selected_source_stamps
            )
        else:
            selected_times = np.empty(0)
            span = 0.0
            max_gap = float("inf")
            nearest_delta = float("inf")
            source_conflict = False
        velocity = np.asarray([np.nan, np.nan], dtype=np.float64)
        acceleration = np.asarray([np.nan, np.nan], dtype=np.float64)
        residual = float("inf")
        if count >= 3:
            velocity, acceleration, residual = self._huber_quadratic(
                selected_times - target_s, positions[indices]
            )
        valid = bool(
            count >= self.min_samples
            and span >= self.min_span_s
            and max_gap <= self.max_gap_s
            and nearest_delta <= self.max_gap_s
            and residual <= self.max_residual_m
            and np.isfinite(velocity).all()
            and not source_conflict
        )
        high_dynamics = bool(
            np.isfinite(acceleration).all()
            and np.linalg.norm(acceleration)
            >= self.high_dynamics_acceleration_mps2
        )
        reference = VelocityReference(
            velocity_map=velocity,
            acceleration_map=acceleration,
            fit_residual_m=residual,
            fit_time_span_s=span,
            fit_sample_count=count,
            max_time_gap_s=max_gap,
            nearest_truth_delta_s=nearest_delta,
            reference_valid=valid,
            main_metric_valid=valid and not edge,
            edge_reference=edge,
            high_dynamics=high_dynamics,
            source_timestamp_conflict=source_conflict,
        )
        self.cache[key] = reference
        return reference


def cluster_dict(cluster) -> Dict:
    return {
        "cluster_id": cluster.cluster_id,
        "point_indices": cluster.point_indices,
        "centroid": cluster.centroid,
        "covariance": cluster.covariance,
        "width": cluster.width,
        "point_count": cluster.point_count,
        "source_mask": cluster.source_mask,
        "static_distance_min": cluster.static_distance_min,
        "static_distance_mean": cluster.static_distance_mean,
        "range_mean": cluster.range_mean,
    }


def measurement_dict(measurement) -> Dict:
    return {
        "measurement_id": measurement.measurement_id,
        "mode": measurement.mode,
        "cluster_ids": measurement.cluster_ids,
        "position": measurement.position,
        "covariance": measurement.covariance,
        "confidence": measurement.confidence,
        "conditioned_track_id": measurement.conditioned_track_id,
    }


def association_dict(association) -> Dict:
    return asdict(association)


def track_dict(track) -> Dict:
    return {
        "track_id": track.track_id,
        "track_state": track.track_state,
        "predicted_state": track.predicted_state,
        "updated_state": track.updated_state,
        "covariance": track.covariance,
        "innovation": track.innovation,
        "age_s": track.age_s,
        "time_since_update_s": track.time_since_update_s,
        "visible_hit_count": track.visible_hit_count,
        "consecutive_miss_count": track.consecutive_miss_count,
        "confidence": track.confidence,
        "support_mode": track.support_mode,
        "support_cluster_ids": track.support_cluster_ids,
        "support_measurement_id": track.support_measurement_id,
        "had_reliable_motion": track.had_reliable_motion,
        "had_multileg_support": track.had_multileg_support,
    }


class MetricsAccumulator:
    def __init__(
        self,
        person_match_radius_m: float,
        person_label_id: int,
        cluster_min_points: int,
        confirmation_grace_s: float,
        coast_timeout_s: float,
        velocity_track_min_age_s: float,
    ) -> None:
        self.radius = float(person_match_radius_m)
        self.person_label_id = int(person_label_id)
        self.cluster_min_points = int(cluster_min_points)
        self.confirmation_grace_s = float(confirmation_grace_s)
        self.coast_timeout_s = float(coast_timeout_s)
        self.velocity_track_min_age_s = float(velocity_track_min_age_s)
        self.person_points_total = 0
        self.person_points_retained = 0
        self.cluster_total = 0
        self.cluster_matched = 0
        self.cluster_visible_total = 0
        self.cluster_visible_matched = 0
        self.cluster_fragment_excess = 0
        self.cluster_multi_person_merges = 0
        self.measurement_total = 0
        self.measurement_matched = 0
        self.measurement_visible_total = 0
        self.measurement_visible_matched = 0
        self.raw_track_visible_total = 0
        self.raw_track_visible_matched = 0
        self.confirmed_visible_total_after_grace = 0
        self.confirmed_visible_matched_after_grace = 0
        self.false_confirmed_track_seconds = 0.0
        self.duration_s = 0.0
        self.position_errors: List[np.ndarray] = []
        self.position_errors_by_mode: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.track_position_errors: List[np.ndarray] = []
        self.track_position_errors_by_mode: Dict[
            str, List[np.ndarray]
        ] = defaultdict(list)
        self.velocity_errors: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.velocity_high_dynamic_errors: List[np.ndarray] = []
        self.velocity_matched_track_samples = 0
        self.velocity_reference_total = 0
        self.velocity_reference_valid = 0
        self.velocity_reference_main_valid = 0
        self.velocity_reference_edge = 0
        self.velocity_reference_high_dynamics = 0
        self.velocity_reference_source_conflicts = 0
        self.unmatched_clusters = 0
        self.unmatched_measurements = 0
        self.frame_count = 0
        self.id_switches = 0
        self.fragmentations = 0
        self.gt_last_track: Dict[str, Tuple[int, int]] = {}
        self.visible_episode_start: Dict[str, int] = {}
        self.visible_previous: Dict[str, bool] = defaultdict(bool)
        self.episode_confirmed: Dict[str, bool] = defaultdict(bool)
        self.confirmation_latencies_s: List[float] = []
        self.range_stats = {
            name: {
                "person_points_total": 0,
                "person_points_retained": 0,
                "visible_person_frames": 0,
                "cluster_detected_person_frames": 0,
                "cluster_fragments": 0,
                "cluster_total": 0,
                "false_clusters": 0,
                "multi_person_merges": 0,
            }
            for name, _, _ in RANGE_BINS
        }

    def _visible_people(
        self,
        result: EstimatorResult,
        truth: Dict[str, object],
    ) -> Tuple[List[int], Dict[int, int]]:
        labels = truth["semantic_label"]
        active_slots = result.point_cloud.slot_indices
        active_labels = labels[active_slots]
        person_active = active_labels == self.person_label_id
        self.person_points_total += int(person_active.sum())
        candidate_slots = set(result.point_cloud.candidate_indices.tolist())
        retained = sum(
            int(slot) in candidate_slots
            for slot in active_slots[person_active].tolist()
        )
        self.person_points_retained += retained

        leg_positions = truth["leg_positions"].reshape((-1, 2))
        owners: Dict[int, int] = {}
        counts: Dict[int, int] = defaultdict(int)
        person_indices = np.flatnonzero(person_active)
        for local_index in person_indices:
            point = result.point_cloud.points_map[local_index]
            owner = int(
                np.linalg.norm(leg_positions - point, axis=1).argmin() // 2
            )
            counts[owner] += 1
            owners[int(active_slots[local_index])] = owner
            bin_name = range_bin_name(result.point_cloud.ranges[local_index])
            self.range_stats[bin_name]["person_points_total"] += 1
            if int(active_slots[local_index]) in candidate_slots:
                self.range_stats[bin_name]["person_points_retained"] += 1
        visible = sorted(
            owner
            for owner, count in counts.items()
            if count >= self.cluster_min_points
        )
        return visible, owners

    def update(
        self,
        result: EstimatorResult,
        truth: Dict[str, object],
        velocity_references: Dict[str, VelocityReference],
        robot_pose_map: np.ndarray,
        dt_s: float,
    ) -> None:
        self.frame_count += 1
        self.duration_s += float(dt_s)
        ids = truth["ids"].tolist()
        positions = truth["positions"]
        visible, _ = self._visible_people(result, truth)
        visible_positions = positions[visible] if visible else np.empty((0, 2))
        for pid in ids:
            reference = velocity_references[pid]
            self.velocity_reference_total += 1
            self.velocity_reference_valid += int(reference.reference_valid)
            self.velocity_reference_main_valid += int(
                reference.main_metric_valid
            )
            self.velocity_reference_edge += int(reference.edge_reference)
            self.velocity_reference_high_dynamics += int(
                reference.high_dynamics
            )
            self.velocity_reference_source_conflicts += int(
                reference.source_timestamp_conflict
            )

        cluster_nearby_people = [
            [
                visible[index]
                for index in range(len(visible))
                if np.linalg.norm(cluster.centroid - visible_positions[index])
                <= self.radius
            ]
            for cluster in result.clusters
        ]
        matched_cluster_count = sum(bool(items) for items in cluster_nearby_people)
        self.cluster_total += len(result.clusters)
        self.cluster_matched += matched_cluster_count
        self.cluster_visible_total += len(visible)
        self.cluster_visible_matched += sum(
            any(
                person_index in nearby
                for nearby in cluster_nearby_people
            )
            for person_index in visible
        )
        self.unmatched_clusters += len(result.clusters) - matched_cluster_count

        for cluster, nearby_people in zip(
            result.clusters, cluster_nearby_people
        ):
            name = range_bin_name(cluster.range_mean)
            self.range_stats[name]["cluster_total"] += 1
            if not nearby_people:
                self.range_stats[name]["false_clusters"] += 1
            if len(nearby_people) > 1:
                self.range_stats[name]["multi_person_merges"] += 1

        for visible_index in visible:
            distance = float(
                np.linalg.norm(positions[visible_index] - robot_pose_map[:2])
            )
            name = range_bin_name(distance)
            self.range_stats[name]["visible_person_frames"] += 1
            nearby = [
                cluster_index
                for cluster_index, cluster in enumerate(result.clusters)
                if np.linalg.norm(cluster.centroid - positions[visible_index])
                <= self.radius
            ]
            if nearby:
                self.range_stats[name]["cluster_detected_person_frames"] += 1
            if len(nearby) > 1:
                excess = len(nearby) - 1
                self.cluster_fragment_excess += excess
                self.range_stats[name]["cluster_fragments"] += excess
        for nearby_people in cluster_nearby_people:
            if len(nearby_people) > 1:
                self.cluster_multi_person_merges += 1

        measurement_positions = np.asarray(
            [measurement.position for measurement in result.measurements],
            dtype=np.float64,
        ).reshape((-1, 2))
        measurement_matches, unmatched_measurements, _ = match_positions(
            measurement_positions, visible_positions, self.radius
        )
        self.measurement_total += len(result.measurements)
        self.measurement_matched += len(measurement_matches)
        self.measurement_visible_total += len(visible)
        self.measurement_visible_matched += len(
            {visible[truth_index] for _, truth_index, _ in measurement_matches}
        )
        self.unmatched_measurements += len(unmatched_measurements)
        for measurement_index, truth_index, _ in measurement_matches:
            measurement = result.measurements[measurement_index]
            error = measurement.position - visible_positions[truth_index]
            self.position_errors.append(error)
            self.position_errors_by_mode[measurement.mode].append(error)

        raw_tracks = [
            track for track in result.tracks if track.track_state != TRACK_DELETED
        ]
        confirmed_tracks = [
            track
            for track in result.tracks
            if track.track_state in (TRACK_CONFIRMED, TRACK_COASTING)
        ]
        raw_positions = np.asarray(
            [track.updated_state[:2] for track in raw_tracks], dtype=np.float64
        ).reshape((-1, 2))
        confirmed_positions = np.asarray(
            [track.updated_state[:2] for track in confirmed_tracks],
            dtype=np.float64,
        ).reshape((-1, 2))
        raw_matches, _, _ = match_positions(
            raw_positions, visible_positions, self.radius
        )
        confirmed_visible_matches, _, _ = match_positions(
            confirmed_positions, visible_positions, self.radius
        )
        self.raw_track_visible_total += len(visible)
        self.raw_track_visible_matched += len(
            {truth_index for _, truth_index, _ in raw_matches}
        )

        timestamp_ns = result.fusion_timestamp_ns
        confirmed_by_visible = {
            visible[truth_index]: confirmed_tracks[track_index]
            for track_index, truth_index, _ in confirmed_visible_matches
        }
        for person_index, pid in enumerate(ids):
            is_visible = person_index in visible
            if is_visible and not self.visible_previous[pid]:
                self.visible_episode_start[pid] = timestamp_ns
                self.episode_confirmed[pid] = False
            if not is_visible:
                self.visible_previous[pid] = False
                continue
            self.visible_previous[pid] = True
            episode_age_s = (
                timestamp_ns - self.visible_episode_start[pid]
            ) / 1e9
            if episode_age_s >= self.confirmation_grace_s:
                self.confirmed_visible_total_after_grace += 1
                if person_index in confirmed_by_visible:
                    self.confirmed_visible_matched_after_grace += 1
            if (
                person_index in confirmed_by_visible
                and not self.episode_confirmed[pid]
            ):
                self.confirmation_latencies_s.append(max(0.0, episode_age_s))
                self.episode_confirmed[pid] = True

        for person_index, track in confirmed_by_visible.items():
            pid = ids[person_index]
            prior = self.gt_last_track.get(pid)
            if prior is not None and prior[0] != track.track_id:
                gap_s = (timestamp_ns - prior[1]) / 1e9
                if gap_s <= self.coast_timeout_s:
                    self.id_switches += 1
                else:
                    self.fragmentations += 1
            self.gt_last_track[pid] = (track.track_id, timestamp_ns)

        all_matches, unmatched_confirmed, _ = match_positions(
            confirmed_positions, positions, self.radius
        )
        self.false_confirmed_track_seconds += len(unmatched_confirmed) * dt_s
        for track_index, truth_index, _ in all_matches:
            track = confirmed_tracks[track_index]
            pid = ids[truth_index]
            reference = velocity_references[pid]
            track_position_error = track.updated_state[:2] - positions[truth_index]
            self.track_position_errors.append(track_position_error)
            if track.support_mode is not None:
                self.track_position_errors_by_mode[track.support_mode].append(
                    track_position_error
                )
            self.velocity_matched_track_samples += 1
            if (
                track.age_s < self.velocity_track_min_age_s
                or not reference.main_metric_valid
            ):
                continue
            error = track.updated_state[2:] - reference.velocity_map
            category = (
                "observed"
                if track.support_measurement_id is not None
                and track.track_state == TRACK_CONFIRMED
                else "coasting"
            )
            self.velocity_errors[category].append(error)
            self.velocity_errors["all"].append(error)
            if reference.high_dynamics:
                self.velocity_high_dynamic_errors.append(error)

    def final_metrics(self) -> Dict:
        measurement_position_modes = {}
        track_position_modes = {}
        for mode in (
            MEASUREMENT_DOUBLE,
            MEASUREMENT_SINGLE,
            MEASUREMENT_MERGED_BODY,
        ):
            measurement_errors = self.position_errors_by_mode[mode]
            track_errors = self.track_position_errors_by_mode[mode]
            measurement_position_modes[mode] = {
                "matched_count": len(measurement_errors),
                "rmse_m": rmse(measurement_errors),
                "bias_xy_m": bias(measurement_errors),
            }
            track_position_modes[mode] = {
                "matched_count": len(track_errors),
                "rmse_m": rmse(track_errors),
                "bias_xy_m": bias(track_errors),
            }
        duration_minutes = self.duration_s / 60.0
        return {
            "frame_count": self.frame_count,
            "duration_s": self.duration_s,
            "person_point_filter": {
                "total": self.person_points_total,
                "retained": self.person_points_retained,
                "retention_rate": safe_rate(
                    self.person_points_retained, self.person_points_total
                ),
            },
            "clusters": {
                "total": self.cluster_total,
                "matched": self.cluster_matched,
                "precision": safe_rate(
                    self.cluster_matched, self.cluster_total
                ),
                "visible_person_frames": self.cluster_visible_total,
                "detected_visible_person_frames": self.cluster_visible_matched,
                "recall": safe_rate(
                    self.cluster_visible_matched, self.cluster_visible_total
                ),
                "fragment_excess": self.cluster_fragment_excess,
                "multi_person_merge_count": self.cluster_multi_person_merges,
                "unmatched_per_frame": safe_rate(
                    self.unmatched_clusters, self.frame_count
                ),
            },
            "measurements": {
                "total": self.measurement_total,
                "matched": self.measurement_matched,
                "precision": safe_rate(
                    self.measurement_matched, self.measurement_total
                ),
                "visible_person_frames": self.measurement_visible_total,
                "detected_visible_person_frames": self.measurement_visible_matched,
                "recall": safe_rate(
                    self.measurement_visible_matched,
                    self.measurement_visible_total,
                ),
                "unmatched_per_frame": safe_rate(
                    self.unmatched_measurements, self.frame_count
                ),
            },
            "tracks": {
                "raw_track_recall": safe_rate(
                    self.raw_track_visible_matched,
                    self.raw_track_visible_total,
                ),
                "confirmed_track_recall_after_grace": safe_rate(
                    self.confirmed_visible_matched_after_grace,
                    self.confirmed_visible_total_after_grace,
                ),
                "confirmed_visible_denominator_after_grace": (
                    self.confirmed_visible_total_after_grace
                ),
                "confirmation_latency_s": {
                    "count": len(self.confirmation_latencies_s),
                    "mean": (
                        float(np.mean(self.confirmation_latencies_s))
                        if self.confirmation_latencies_s
                        else None
                    ),
                    "p95": (
                        float(np.percentile(self.confirmation_latencies_s, 95))
                        if self.confirmation_latencies_s
                        else None
                    ),
                },
                "false_confirmed_track_seconds": (
                    self.false_confirmed_track_seconds
                ),
                "false_confirmed_track_seconds_per_minute": safe_rate(
                    self.false_confirmed_track_seconds, duration_minutes
                ),
                "id_switches": self.id_switches,
                "fragmentations": self.fragmentations,
            },
            "position": {
                "confirmed_track": {
                    "matched_count": len(self.track_position_errors),
                    "rmse_m": rmse(self.track_position_errors),
                    "bias_xy_m": bias(self.track_position_errors),
                    "by_support_mode": track_position_modes,
                },
                "measurement": {
                    "matched_count": len(self.position_errors),
                    "rmse_m": rmse(self.position_errors),
                    "bias_xy_m": bias(self.position_errors),
                    "by_mode": measurement_position_modes,
                },
                "position_rmse_double_leg": track_position_modes[
                    MEASUREMENT_DOUBLE
                ]["rmse_m"],
                "position_rmse_single_leg": track_position_modes[
                    MEASUREMENT_SINGLE
                ]["rmse_m"],
                "position_rmse_merged_body": track_position_modes[
                    MEASUREMENT_MERGED_BODY
                ]["rmse_m"],
                "position_bias_x": (
                    bias(self.track_position_errors)[0]
                    if self.track_position_errors
                    else None
                ),
                "position_bias_y": (
                    bias(self.track_position_errors)[1]
                    if self.track_position_errors
                    else None
                ),
                "interpretation": (
                    "Errors include tracker/measurement error and the observation-model "
                    "offset between LiDAR leg geometry and the simulated body reference center."
                ),
            },
            "velocity_reference": {
                "all_gt_samples": self.velocity_reference_total,
                "matched_track_samples": self.velocity_matched_track_samples,
                "reference_valid": self.velocity_reference_valid,
                "main_metric_valid": self.velocity_reference_main_valid,
                "edge_reference": self.velocity_reference_edge,
                "high_dynamics": self.velocity_reference_high_dynamics,
                "source_timestamp_conflict": (
                    self.velocity_reference_source_conflicts
                ),
                "reference_valid_rate": safe_rate(
                    self.velocity_reference_valid,
                    self.velocity_reference_total,
                ),
                "main_valid_rate": safe_rate(
                    self.velocity_reference_main_valid,
                    self.velocity_reference_total,
                ),
                "edge_reference_rate": safe_rate(
                    self.velocity_reference_edge,
                    self.velocity_reference_total,
                ),
                "high_dynamics_rate": safe_rate(
                    self.velocity_reference_high_dynamics,
                    self.velocity_reference_total,
                ),
                "source_timestamp_conflict_rate": safe_rate(
                    self.velocity_reference_source_conflicts,
                    self.velocity_reference_total,
                ),
            },
            "velocity": {
                "rmse_observed_mps": rmse(self.velocity_errors["observed"]),
                "observed_count": len(self.velocity_errors["observed"]),
                "rmse_coasting_mps": rmse(self.velocity_errors["coasting"]),
                "coasting_count": len(self.velocity_errors["coasting"]),
                "rmse_all_mps": rmse(self.velocity_errors["all"]),
                "all_count": len(self.velocity_errors["all"]),
                "rmse_high_dynamics_mps": rmse(
                    self.velocity_high_dynamic_errors
                ),
                "high_dynamics_count": len(
                    self.velocity_high_dynamic_errors
                ),
            },
            "range_bins": self.range_stats,
        }


def make_output_dir(args: argparse.Namespace) -> Path:
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    run_name = args.run_name.strip()
    if not run_name:
        run_name = (
            datetime.now().strftime("%Y%m%d_%H%M%S")
            + "_dual_lidar_pedestrian_tracking"
        )
    output_dir = output_root / run_name
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {output_dir}")
    output_dir.mkdir()
    return output_dir


def deterministic_color(identifier: str, bright: bool = True) -> Tuple[int, int, int]:
    value = sum((index + 1) * ord(char) for index, char in enumerate(identifier))
    base = 96 if bright else 48
    span = 160 if bright else 112
    return (
        base + (value * 37) % span,
        base + (value * 67) % span,
        base + (value * 97) % span,
    )


def write_trajectory_figure(
    path: Path,
    occupancy_map: OccupancyMap,
    truth_paths: Dict[str, List[np.ndarray]],
    track_paths: Dict[int, List[np.ndarray]],
) -> None:
    base = np.stack(
        (
            occupancy_map.image,
            occupancy_map.image,
            occupancy_map.image,
        ),
        axis=2,
    )
    image = Image.fromarray(base.astype(np.uint8), mode="RGB")
    draw = ImageDraw.Draw(image)

    def pixels(points: Sequence[np.ndarray]) -> List[Tuple[int, int]]:
        if len(points) < 2:
            return []
        array = np.asarray(points, dtype=np.float64).reshape((-1, 2))
        rows, cols, inside = occupancy_map.world_to_grid(array)
        return [
            (int(col), int(row))
            for row, col, valid in zip(rows, cols, inside)
            if valid
        ]

    for pid in sorted(truth_paths):
        points = pixels(truth_paths[pid])
        if len(points) >= 2:
            draw.line(points, fill=deterministic_color(pid, bright=False), width=1)
    for track_id in sorted(track_paths):
        points = pixels(track_paths[track_id])
        if len(points) >= 2:
            draw.line(
                points,
                fill=deterministic_color(f"track_{track_id}", bright=True),
                width=2,
            )
    image.save(path)


def run(args: argparse.Namespace) -> Path:
    np.random.seed(0)
    reader = DatasetReader(args.dataset, args.map_yaml, args.max_frames)
    occupancy_map = OccupancyMap.from_yaml(reader.map_yaml)
    tracker_config = TrackerConfig(
        static_distance_threshold_m=args.static_distance_threshold_m,
        cluster_epsilon_m=args.cluster_epsilon_m,
        cluster_min_points=args.cluster_min_points,
        max_sensor_time_skew_s=args.max_sensor_time_skew_s,
    )
    tracker_config.validate()
    audit = build_data_audit(reader, occupancy_map)
    if not audit["scan_time"]["strictly_increasing"]:
        raise ValueError("scan timestamps are not strictly increasing")
    if not audit["robot_pose_map"]["pose_continuity_verified"]:
        raise ValueError("robot map pose failed continuity audit")
    if not audit["pedestrian_ids"]["stable_across_frames"]:
        raise ValueError("pedestrian IDs are not persistent and unique")

    reference_builder = VelocityReferenceBuilder(
        reader=reader,
        half_window_s=args.velocity_fit_half_window_s,
        min_samples=args.velocity_fit_min_samples,
        min_span_s=args.velocity_fit_min_span_s,
        max_gap_s=args.max_truth_interpolation_gap_s,
        max_residual_m=args.velocity_fit_max_residual_m,
        high_dynamics_acceleration_mps2=args.high_dynamics_acceleration_mps2,
    )
    output_dir = make_output_dir(args)
    write_json(output_dir / "data_audit.json", audit)
    write_json(
        output_dir / "config.json",
        {
            "dataset": str(reader.root),
            "map_yaml": str(reader.map_yaml),
            "development_evaluation_bag": True,
            "random_seed": 0,
            "estimator_input_whitelist": list(ALLOWED_ESTIMATOR_INPUTS),
            "tracker": asdict(tracker_config),
            "evaluation": {
                "person_match_radius_m": args.person_match_radius_m,
                "velocity_fit_half_window_s": args.velocity_fit_half_window_s,
                "velocity_fit_min_samples": args.velocity_fit_min_samples,
                "velocity_fit_min_span_s": args.velocity_fit_min_span_s,
                "max_truth_interpolation_gap_s": (
                    args.max_truth_interpolation_gap_s
                ),
                "velocity_fit_max_residual_m": (
                    args.velocity_fit_max_residual_m
                ),
                "high_dynamics_acceleration_mps2": (
                    args.high_dynamics_acceleration_mps2
                ),
                "velocity_track_min_age_s": args.velocity_track_min_age_s,
            },
            "deterministic_ordering": {
                "clusters": "centroid_x, centroid_y, min_point_index",
                "tracks": "state priority, older age, track_id",
                "newborn_ids": "stable measurement order, monotonically increasing",
                "json_float_digits": 9,
            },
        },
    )

    scan_dt = np.diff(reader.scan_1_timestamps) / 1e9
    median_dt = float(np.median(scan_dt)) if len(scan_dt) else 0.066
    metrics = MetricsAccumulator(
        person_match_radius_m=args.person_match_radius_m,
        person_label_id=reader.person_label_id,
        cluster_min_points=args.cluster_min_points,
        confirmation_grace_s=2.0 * median_dt,
        coast_timeout_s=tracker_config.confirmed_coast_timeout_s,
        velocity_track_min_age_s=args.velocity_track_min_age_s,
    )
    estimator = LidarPedestrianEstimator(occupancy_map, tracker_config)
    truth_paths: Dict[str, List[np.ndarray]] = defaultdict(list)
    track_paths: Dict[int, List[np.ndarray]] = defaultdict(list)
    streams = {
        name: (output_dir / f"{name}.jsonl").open("w", encoding="utf-8")
        for name in (
            "clusters",
            "measurements",
            "associations",
            "tracks",
            "velocity_references",
        )
    }
    try:
        for index in range(len(reader.frames)):
            estimator_frame = reader.estimator_frame(index)
            result = estimator.process(estimator_frame)
            truth = reader.truth_frame(index)
            velocity_references = {
                pid: reference_builder.reference(pid, result.fusion_timestamp_ns)
                for pid in truth["ids"].tolist()
            }
            dt_s = median_dt if index == 0 else float(scan_dt[index - 1])
            fusion_pose = (
                estimator_frame.robot_pose_map[
                    result.active_sensor
                    if result.active_sensor is not None
                    else 0
                ]
            )
            metrics.update(
                result=result,
                truth=truth,
                velocity_references=velocity_references,
                robot_pose_map=fusion_pose,
                dt_s=dt_s,
            )
            common = {
                "frame_index": index,
                "fusion_timestamp_ns": result.fusion_timestamp_ns,
                "scan_timestamp_lidar_1": (
                    estimator_frame.scan_timestamp_lidar_1
                ),
                "scan_timestamp_lidar_2": (
                    estimator_frame.scan_timestamp_lidar_2
                ),
                "sensor_time_skew_s": result.sensor_time_skew_s,
                "fusion_mode": result.fusion_mode,
                "active_sensor": result.active_sensor,
            }
            write_jsonl_record(
                streams["clusters"],
                {
                    **common,
                    "active_point_count": len(result.point_cloud.slot_indices),
                    "candidate_point_count": int(
                        result.point_cloud.candidate_mask.sum()
                    ),
                    "clusters": [
                        cluster_dict(cluster) for cluster in result.clusters
                    ],
                },
            )
            write_jsonl_record(
                streams["measurements"],
                {
                    **common,
                    "measurements": [
                        measurement_dict(measurement)
                        for measurement in result.measurements
                    ],
                },
            )
            write_jsonl_record(
                streams["associations"],
                {
                    **common,
                    "associations": [
                        association_dict(association)
                        for association in result.associations
                    ],
                },
            )
            write_jsonl_record(
                streams["tracks"],
                {
                    **common,
                    "tracks": [track_dict(track) for track in result.tracks],
                },
            )
            write_jsonl_record(
                streams["velocity_references"],
                {
                    **common,
                    "references": [
                        {
                            "pedestrian_id": pid,
                            **asdict(velocity_references[pid]),
                        }
                        for pid in sorted(velocity_references)
                    ],
                },
            )
            for offset, pid in enumerate(truth["ids"].tolist()):
                truth_paths[pid].append(truth["positions"][offset].copy())
            for track in result.tracks:
                if track.track_state in (TRACK_CONFIRMED, TRACK_COASTING):
                    track_paths[track.track_id].append(
                        track.updated_state[:2].copy()
                    )
    finally:
        for stream in streams.values():
            stream.close()

    final_metrics = metrics.final_metrics()
    final_metrics["development_evaluation_bag"] = True
    final_metrics["raw_pedestrian_velocity_used_as_reference"] = False
    final_metrics["fusion"] = {
        "max_sensor_time_skew_s": tracker_config.max_sensor_time_skew_s,
        "policy": (
            "dual when within skew threshold; otherwise newer sensor only"
        ),
    }
    write_json(output_dir / "metrics.json", final_metrics)
    if args.write_figure:
        write_trajectory_figure(
            output_dir / "trajectory_overview.png",
            occupancy_map,
            truth_paths,
            track_paths,
        )
    return output_dir


def main() -> None:
    args = parse_args()
    output_dir = run(args)
    metrics = json.loads(
        (output_dir / "metrics.json").read_text(encoding="utf-8")
    )
    print(f"output_dir={output_dir}")
    print(
        "point_retention="
        f"{metrics['person_point_filter']['retention_rate']} "
        f"cluster_recall={metrics['clusters']['recall']} "
        f"measurement_recall={metrics['measurements']['recall']} "
        f"confirmed_recall="
        f"{metrics['tracks']['confirmed_track_recall_after_grace']} "
        f"velocity_rmse={metrics['velocity']['rmse_all_mps']}"
    )


if __name__ == "__main__":
    main()
