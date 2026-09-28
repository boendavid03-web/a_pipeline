"""Isaac5 to legacy DRL-VO observation-contract adapter.

This module does not import torch, alter the network, or alter checkpoints. It
converts the non-RTX Isaac5 dual LaserScan contract into the existing legacy
front scan/history/goal/action contract used by DRL-VO.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from lidar_sensor import DualLaserScanData, LaserScanData


HERE = Path(__file__).resolve().parent
WORKSPACE_ROOT = HERE.parents[3]
LEGACY_ADAPTER_PATH = (
    WORKSPACE_ROOT
    / "sim_to_real/robot/comparison_models/runtime_code/methods/experiments"
    / "drl_vo_ros2_offline/observation_adapter.py"
).resolve()


def _load_legacy_adapter():
    if not LEGACY_ADAPTER_PATH.is_file():
        raise FileNotFoundError(LEGACY_ADAPTER_PATH)
    spec = importlib.util.spec_from_file_location(
        "isaac5_legacy_drlvo_observation_adapter", LEGACY_ADAPTER_PATH
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load legacy adapter: {LEGACY_ADAPTER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


LEGACY = _load_legacy_adapter()
OBSERVATION_SIZE = int(LEGACY.OBSERVATION_SIZE)
PED_MAP_SHAPE = tuple(LEGACY.PED_MAP_SHAPE)
SCAN_HISTORY = int(LEGACY.SCAN_HISTORY)
FRONT_SCAN_BEAMS = 720


@dataclass(frozen=True)
class PointStampedContract:
    """ROS PointStamped-equivalent data without requiring rclpy."""

    frame_id: str
    stamp_ns: int
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class DrlvoObservation:
    observation: np.ndarray
    front_scan: np.ndarray
    scan_history: np.ndarray
    pedestrian_map: np.ndarray
    local_goal: PointStampedContract
    final_goal: PointStampedContract
    timestamp_ns: int


def _sensor_to_base(scan: LaserScanData, translation: tuple[float, float], yaw: float):
    raw = np.asarray(scan.ranges, dtype=np.float32)
    if raw.shape != (len(scan.ranges),) or raw.size < 4:
        raise ValueError("invalid LaserScan range vector")
    if not np.isfinite(raw[~np.isposinf(raw)]).all():
        raise ValueError("LaserScan contains NaN or negative infinity")
    raw_angles = float(scan.angle_min) + np.arange(raw.size, dtype=np.float32) * float(
        scan.angle_increment
    )
    geometric = raw.copy()
    geometric[np.isposinf(geometric)] = float(scan.range_max)
    valid = np.isfinite(geometric) & (geometric >= float(scan.range_min))
    x_sensor = geometric * np.cos(raw_angles)
    y_sensor = geometric * np.sin(raw_angles)
    c, s = math.cos(yaw), math.sin(yaw)
    x_base = np.full(raw.size, np.nan, dtype=np.float32)
    y_base = np.full(raw.size, np.nan, dtype=np.float32)
    x_base[valid] = float(translation[0]) + c * x_sensor[valid] - s * y_sensor[valid]
    y_base[valid] = float(translation[1]) + s * x_sensor[valid] + c * y_sensor[valid]
    ranges = np.full(raw.size, np.nan, dtype=np.float32)
    angles = np.full(raw.size, np.nan, dtype=np.float32)
    ranges[valid] = np.hypot(x_base[valid], y_base[valid])
    angles[valid] = np.arctan2(y_base[valid], x_base[valid])
    return ranges, angles


def _dual_to_legacy_front(pair: DualLaserScanData) -> np.ndarray:
    if pair.scan_01.sim_time != pair.scan_02.sim_time:
        raise ValueError("dual scans must share the same simulation timestamp")
    ranges_01, angles_01 = _sensor_to_base(
        pair.scan_01, (0.20, 0.13), 0.0
    )
    ranges_02, angles_02 = _sensor_to_base(
        pair.scan_02, (-0.20, -0.13), math.pi
    )
    virtual_ranges = np.concatenate((ranges_01, ranges_02))
    virtual_angles = np.concatenate((angles_01, angles_02))
    front, _coverage, _nearest = LEGACY.dual_lidar_to_legacy_scan(
        virtual_ranges, virtual_angles
    )
    if front.shape != (FRONT_SCAN_BEAMS,):
        raise RuntimeError(f"legacy front scan shape is {front.shape}")
    return front.astype(np.float32, copy=False)


def _local_goal(robot_pose_odom: np.ndarray, final_goal_xy: np.ndarray) -> np.ndarray:
    pose = np.asarray(robot_pose_odom, dtype=np.float32).reshape(-1)
    goal = np.asarray(final_goal_xy, dtype=np.float32).reshape(-1)
    if pose.shape != (3,) or goal.shape != (2,):
        raise ValueError("robot pose must be [x,y,yaw] and goal must be [x,y]")
    if not np.isfinite(pose).all() or not np.isfinite(goal).all():
        raise ValueError("robot pose and goal must be finite")
    dx, dy = float(goal[0] - pose[0]), float(goal[1] - pose[1])
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    return np.asarray([c * dx + s * dy, -s * dx + c * dy], dtype=np.float32)


class DrlvoObservationAdapter:
    """Build the exact 19,202-element legacy DRL-VO observation."""

    def __init__(self) -> None:
        self._history: deque[np.ndarray] = deque(maxlen=SCAN_HISTORY)
        self._last_timestamp_ns: int | None = None

    def reset(self) -> None:
        self._history.clear()
        self._last_timestamp_ns = None

    def adapt(
        self,
        pair: DualLaserScanData,
        robot_pose_odom: np.ndarray,
        final_goal_xy: np.ndarray,
        pedestrian_map_mps: np.ndarray,
    ) -> DrlvoObservation:
        ped = np.asarray(pedestrian_map_mps, dtype=np.float32)
        if ped.shape != PED_MAP_SHAPE or not np.isfinite(ped).all():
            raise ValueError(f"pedestrian map must be finite with shape {PED_MAP_SHAPE}")
        timestamp_ns = int(round(float(pair.scan_01.sim_time) * 1_000_000_000.0))
        if self._last_timestamp_ns is not None and timestamp_ns <= self._last_timestamp_ns:
            raise ValueError("dual scan timestamps must be strictly increasing")
        front = _dual_to_legacy_front(pair)
        self._history.append(front)
        history_values = list(self._history)
        padded = [history_values[0]] * (SCAN_HISTORY - len(history_values))
        history = np.stack(padded + history_values).astype(np.float32)
        local_goal = _local_goal(robot_pose_odom, final_goal_xy)
        normalized = np.concatenate(
            (
                np.clip(ped / 2.0, -1.0, 1.0).reshape(-1),
                LEGACY.compress_scan_history(history) / 15.0 - 1.0,
                local_goal / 2.0,
            )
        ).astype(np.float32)
        if normalized.shape != (OBSERVATION_SIZE,) or not np.isfinite(normalized).all():
            raise RuntimeError(f"invalid DRL-VO observation shape/content: {normalized.shape}")
        final = np.asarray(final_goal_xy, dtype=np.float32)
        observation = DrlvoObservation(
            observation=normalized,
            front_scan=front,
            scan_history=history,
            pedestrian_map=ped.copy(),
            local_goal=PointStampedContract("base_link", timestamp_ns, float(local_goal[0]), float(local_goal[1])),
            final_goal=PointStampedContract("odom", timestamp_ns, float(final[0]), float(final[1])),
            timestamp_ns=timestamp_ns,
        )
        self._last_timestamp_ns = timestamp_ns
        return observation

    @staticmethod
    def normalized_action_to_cmd_vel(action: np.ndarray) -> tuple[float, float, float]:
        normalized = np.asarray(action, dtype=np.float32).reshape(-1)
        if normalized.shape != (2,) or not np.isfinite(normalized).all():
            raise ValueError("legacy DRL-VO action must be a finite 2-vector")
        physical = LEGACY.normalized_to_physical(normalized)
        return float(physical[0]), 0.0, float(physical[1])
