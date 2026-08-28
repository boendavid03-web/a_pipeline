#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /drl_vo/episode_reset, /drl_vo/raw_model_cmd, /drl_vo/training_state, /odom, /pedestrian_ground_truth, /scan_01, /scan_02, /semantic_cnn/final_goal, /semantic_cnn/local_subgoal
# 检测到的消息类型：DrlVoTrainingState, PedestrianStateArray; Empty; LaserScan; Odometry; PointStamped, Twist
# 检测到的文件格式：PNG, PT, YAML
# 可能使用的关键环境变量：CUDA, DRLVO_CODE, E402, EXPECTED_BASE_WEIGHT_ITEMS, EXPECTED_SEMANTIC_WEIGHT_ITEMS, IGNORE_LABEL, NANOSECONDS_PER_SECOND, NAVIGATION_PROJECT_ROOT, OBSERVATION_SIZE, PEDESTRIAN_LEG_LATERAL_OFFSET_M, PEDESTRIAN_LEG_MATCH_RADIUS_M, PED_MAP_SHAPE, PERSON_LABEL, PROJECT_ROOT, SCAN_HISTORY, SELF_FOOTPRINT_HALF_EXTENTS_M, SEMANTIC_NUM_CLASSES, STATIC_LABEL_FILTER_RADIUS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo drl_vo_fixed_dual_inference_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:36:01.319630985 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.642741897 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_drl_vo_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_drl_vo_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_drl_vo_fixed_dual_helpers.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜drl_vo_fixed_dual_inference_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""ROS 2 fixed-dual-lidar inference for DRL-VO policies.

This node intentionally rebuilds the exact 19,202-element observation used by
``methods/experiments/drl_vo_ros2_offline``. The original/base policies accept
oracle, truth-free dual-LiDAR predicted, or zero pedestrian velocity maps.
Semantic policies retain the recorded oracle lower-leg Person-label contract,
so those modes remain simulation research interfaces.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import message_filters
import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from semantic_nav_gazebo.msg import DrlVoTrainingState, PedestrianStateArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener


PROJECT_ROOT = Path(
    os.environ.get(
        "NAVIGATION_PROJECT_ROOT",
        Path(__file__).resolve().parents[5],
    )
).resolve()
DRLVO_CODE = PROJECT_ROOT / "methods" / "experiments" / "drl_vo_ros2_offline"
if str(DRLVO_CODE) not in sys.path:
    sys.path.insert(0, str(DRLVO_CODE))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drlvo_model import (  # noqa: E402
    DrlVoPolicy,
    load_policy_strict,
    load_trained_semantic_policy,
)
from observation_adapter import (  # noqa: E402
    OBSERVATION_SIZE,
    PED_MAP_SHAPE,
    SCAN_HISTORY,
    compress_scan_history,
    compress_semantic_history,
    dual_lidar_to_legacy_scan,
    dual_lidar_to_legacy_semantic,
    normalized_to_physical,
    pedestrian_velocity_map,
    tracks_to_drl_vo_ped_map_with_diagnostics,
)
from methods.experiments.dual_lidar_pedestrian_bev.dataset import (  # noqa: E402
    BEVSpec,
    base_to_map,
    map_to_base,
)
from methods.experiments.dual_lidar_pedestrian_bev.model import (  # noqa: E402
    TemporalBEVPedestrianDetector,
    decode_detections,
)
from methods.experiments.dual_lidar_pedestrian_bev.tracker import (  # noqa: E402
    PedestrianTracker,
    detections_base_to_map,
)
from drl_vo_control_contract import (  # noqa: E402
    ActuationSample,
    actuation_deadlock_detected,
    final_goal_rearms_after_reset,
)


IGNORE_LABEL = -1
PERSON_LABEL = 6
SEMANTIC_NUM_CLASSES = 7
STATIC_LABEL_FILTER_RADIUS = 2
PEDESTRIAN_LEG_LATERAL_OFFSET_M = 0.07
PEDESTRIAN_LEG_MATCH_RADIUS_M = 0.105
SELF_FOOTPRINT_HALF_EXTENTS_M = (0.36, 0.32)
EXPECTED_BASE_WEIGHT_ITEMS = 163
EXPECTED_SEMANTIC_WEIGHT_ITEMS = 172
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class TemporalLidarFrame:
    points_xy_base: np.ndarray
    sensor_indices: np.ndarray
    robot_pose_map: np.ndarray
    timestamp_ns: int


def build_temporal_lidar_bev(
    frames: list[TemporalLidarFrame],
    bev_spec: BEVSpec,
    history_frames: int,
) -> np.ndarray:
    """Rebuild the training-time [2T,H,W] ego-compensated input."""

    if len(frames) != history_frames:
        raise ValueError(
            f"perception history must contain {history_frames} frames"
        )
    timestamps = [int(frame.timestamp_ns) for frame in frames]
    if any(
        timestamps[index] >= timestamps[index + 1]
        for index in range(len(timestamps) - 1)
    ):
        raise ValueError("perception timestamps must be strictly increasing")
    current_pose = np.asarray(frames[-1].robot_pose_map, dtype=np.float64)
    if current_pose.shape != (3,) or not np.isfinite(current_pose).all():
        raise ValueError("current robot pose must be finite [x,y,yaw]")
    channels = np.zeros(
        (history_frames * 2, bev_spec.size, bev_spec.size),
        dtype=np.float32,
    )
    for time_index, frame in enumerate(frames):
        points = np.asarray(frame.points_xy_base, dtype=np.float64)
        sensors = np.asarray(frame.sensor_indices, dtype=np.int64)
        pose = np.asarray(frame.robot_pose_map, dtype=np.float64)
        if (
            points.ndim != 2
            or points.shape[1:] != (2,)
            or sensors.shape != (len(points),)
            or pose.shape != (3,)
            or not np.isfinite(points).all()
            or not np.isfinite(pose).all()
            or np.any((sensors < 0) | (sensors > 1))
        ):
            raise ValueError("invalid temporal lidar frame")
        points_current = map_to_base(base_to_map(points, pose), current_pose)
        grid_x, grid_y = bev_spec.metric_to_grid(points_current)
        cols = np.floor(grid_x).astype(np.int64)
        rows = np.floor(grid_y).astype(np.int64)
        inside = (
            (cols >= 0)
            & (cols < bev_spec.size)
            & (rows >= 0)
            & (rows < bev_spec.size)
        )
        for sensor in (0, 1):
            selected = inside & (sensors == sensor)
            channel = channels[time_index * 2 + sensor]
            np.add.at(channel, (rows[selected], cols[selected]), 1.0)
            channel[:] = np.minimum(channel, 3.0) / 3.0
    return channels


def load_perception_checkpoint(
    checkpoint_path: Path,
    device: str,
) -> tuple[TemporalBEVPedestrianDetector, BEVSpec, int]:
    checkpoint = torch.load(
        checkpoint_path, map_location=device, weights_only=False
    )
    required = {
        "model_state_dict",
        "history_frames",
        "base_channels",
        "bev_extent_m",
        "bev_resolution_m",
    }
    missing = required - set(checkpoint)
    if missing:
        raise KeyError(f"perception checkpoint fields missing: {sorted(missing)}")
    history_frames = int(checkpoint["history_frames"])
    bev_spec = BEVSpec(
        float(checkpoint["bev_extent_m"]),
        float(checkpoint["bev_resolution_m"]),
    )
    model = TemporalBEVPedestrianDetector(
        history_frames=history_frames,
        base_channels=int(checkpoint["base_channels"]),
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.eval()
    return model, bev_spec, history_frames


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def time_is_fresh(
    reference_ns: int | None,
    stamp_ns: int | None,
    timeout_seconds: float,
) -> bool:
    if reference_ns is None or stamp_ns is None or timeout_seconds < 0.0:
        return False
    age_ns = int(reference_ns) - int(stamp_ns)
    return 0 <= age_ns <= int(timeout_seconds * NANOSECONDS_PER_SECOND)


def latest_causal_sample(
    samples,
    reference_ns: int | None,
    max_age_seconds: float,
):
    """Return the newest sample not later than the reference timestamp."""
    if reference_ns is None or max_age_seconds < 0.0:
        return None
    max_age_ns = int(max_age_seconds * NANOSECONDS_PER_SECOND)
    best = None
    best_stamp_ns = None
    for stamp_ns, value in samples:
        age_ns = int(reference_ns) - int(stamp_ns)
        if age_ns < 0 or age_ns > max_age_ns:
            continue
        if best_stamp_ns is None or int(stamp_ns) >= best_stamp_ns:
            best = value
            best_stamp_ns = int(stamp_ns)
    if best is None:
        return None
    return best, best_stamp_ns


def clock_rolled_back(previous_ns: int | None, current_ns: int) -> bool:
    return previous_ns is not None and int(current_ns) < int(previous_ns)


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def rotate_point(point, quaternion):
    """Apply a quaternion rotation to one xyz point."""
    px, py, pz = point
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    return (
        px + qw * tx + (qy * tz - qz * ty),
        py + qw * ty + (qz * tx - qx * tz),
        pz + qw * tz + (qx * ty - qy * tx),
    )


def scan_layout(scan: LaserScan) -> tuple:
    return (
        scan.header.frame_id.strip(),
        len(scan.ranges),
        float(scan.angle_min),
        float(scan.angle_max),
        float(scan.angle_increment),
        float(scan.range_min),
        float(scan.range_max),
    )


def scan_layout_matches(first: tuple, second: tuple) -> bool:
    if first[:2] != second[:2]:
        return False
    return all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=1e-6)
        for a, b in zip(first[2:], second[2:])
    )


def transform_scan_to_base(
    raw_ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    scan_range_min: float,
    scan_range_max: float,
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
    configured_range_min: float,
    configured_range_max: float,
    frozen_self_mask: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Transform raw beam endpoints and apply the fixed-beam self-mask contract."""
    raw = np.asarray(raw_ranges, dtype=np.float32).reshape(-1)
    raw_angles = (
        float(angle_min)
        + np.arange(raw.size, dtype=np.float32) * float(angle_increment)
    )
    low = max(float(scan_range_min), float(configured_range_min))
    high = min(float(scan_range_max), float(configured_range_max))
    if not math.isfinite(low) or not math.isfinite(high) or low >= high:
        raise ValueError(f"invalid effective scan range [{low}, {high}]")

    # LaserScan uses +Inf for a beam that did not hit anything within the
    # sensor range.  Treat that as free space up to the effective range_max,
    # rather than dropping every open-space beam and triggering the
    # stop-on-empty-front fail-safe.  NaN and -Inf remain invalid samples.
    no_return = np.isposinf(raw)
    finite_return = np.isfinite(raw) & (raw >= low) & (raw <= high)
    range_valid = finite_return | no_return
    geometric_ranges = raw.copy()
    geometric_ranges[no_return] = high
    point_x = np.full(raw.size, np.nan, dtype=np.float32)
    point_y = np.full(raw.size, np.nan, dtype=np.float32)
    for index in np.flatnonzero(range_valid):
        value = float(geometric_ranges[index])
        rotated_x, rotated_y, _ = rotate_point(
            (
                value * math.cos(float(raw_angles[index])),
                value * math.sin(float(raw_angles[index])),
                0.0,
            ),
            quaternion,
        )
        point_x[index] = float(translation[0] + rotated_x)
        point_y[index] = float(translation[1] + rotated_y)

    footprint_mask = np.zeros(raw.size, dtype=np.bool_)
    valid_indices = np.flatnonzero(range_valid)
    footprint_mask[valid_indices] = (
        np.abs(point_x[valid_indices]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[0]
    ) & (
        np.abs(point_y[valid_indices]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[1]
    )
    if frozen_self_mask is None:
        self_mask = footprint_mask
    else:
        self_mask = np.asarray(frozen_self_mask, dtype=np.bool_).reshape(-1)
        if self_mask.shape != raw.shape:
            raise ValueError(
                f"fixed self mask shape {self_mask.shape} != scan shape {raw.shape}"
            )

    valid = range_valid & ~self_mask
    virtual_ranges = np.full(raw.size, np.nan, dtype=np.float32)
    virtual_angles = np.full(raw.size, np.nan, dtype=np.float32)
    usable = np.flatnonzero(valid)
    virtual_ranges[usable] = np.hypot(point_x[usable], point_y[usable])
    virtual_angles[usable] = np.arctan2(point_y[usable], point_x[usable])
    return virtual_ranges, virtual_angles, valid, footprint_mask


def static_labels_for_pixels(
    label_image: np.ndarray,
    rows: np.ndarray,
    cols: np.ndarray,
    filter_radius: int,
    num_classes: int,
) -> np.ndarray:
    """Match the converter's non-background neighborhood majority rule."""
    if filter_radius < 0:
        raise ValueError("static label filter radius must be non-negative")
    if filter_radius == 0:
        return label_image[rows, cols].astype(np.int64)

    height, width = label_image.shape[:2]
    labels = np.zeros(len(rows), dtype=np.int64)
    for index, (row, col) in enumerate(zip(rows, cols)):
        patch = label_image[
            max(0, int(row) - filter_radius) : min(
                height, int(row) + filter_radius
            ),
            max(0, int(col) - filter_radius) : min(
                width, int(col) + filter_radius
            ),
        ]
        nonzero = patch[patch != 0].astype(np.int64)
        if nonzero.size:
            labels[index] = int(
                np.bincount(nonzero, minlength=num_classes).argmax()
            )
    return labels


def pedestrian_leg_centers(
    pedestrian_xy: np.ndarray,
    pedestrian_yaw: np.ndarray,
) -> np.ndarray:
    positions = np.asarray(pedestrian_xy, dtype=np.float64).reshape((-1, 2))
    yaws = np.asarray(pedestrian_yaw, dtype=np.float64).reshape(-1)
    if len(positions) != len(yaws):
        raise ValueError("pedestrian position/yaw lengths differ")
    if not len(positions):
        return np.empty((0, 2), dtype=np.float64)
    lateral = np.column_stack((-np.sin(yaws), np.cos(yaws)))
    return np.concatenate(
        (
            positions
            - PEDESTRIAN_LEG_LATERAL_OFFSET_M * lateral,
            positions
            + PEDESTRIAN_LEG_LATERAL_OFFSET_M * lateral,
        ),
        axis=0,
    )


def semantic_labels_for_virtual_scan(
    virtual_ranges: np.ndarray,
    virtual_angles: np.ndarray,
    pose_xy_yaw: np.ndarray,
    label_image: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    pedestrian_xy: np.ndarray,
    pedestrian_yaw: np.ndarray,
    static_filter_radius: int = STATIC_LABEL_FILTER_RADIUS,
    num_classes: int = SEMANTIC_NUM_CLASSES,
    person_label: int = PERSON_LABEL,
) -> np.ndarray:
    """Project static labels, then reproduce the oracle lower-leg Person rule."""
    ranges = np.asarray(virtual_ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(virtual_angles, dtype=np.float32).reshape(-1)
    if ranges.shape != angles.shape:
        raise ValueError("virtual ranges and angles must have matching shapes")
    pose = np.asarray(pose_xy_yaw, dtype=np.float64).reshape(-1)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("pose must be a finite [x, y, yaw] vector")

    labels = np.full(ranges.size, IGNORE_LABEL, dtype=np.int64)
    valid = (
        np.isfinite(ranges)
        & np.isfinite(angles)
        & (ranges > 0.0)
    )
    indices = np.flatnonzero(valid)
    if not len(indices):
        return labels

    world_x = pose[0] + ranges[indices] * np.cos(pose[2] + angles[indices])
    world_y = pose[1] + ranges[indices] * np.sin(pose[2] + angles[indices])
    height, width = label_image.shape[:2]
    cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    rows = (
        height
        - 1
        - np.floor((world_y - origin_y) / resolution).astype(np.int64)
    )
    in_map = (
        (cols >= 0)
        & (cols < width)
        & (rows >= 0)
        & (rows < height)
    )
    mapped = indices[in_map]
    static_labels = static_labels_for_pixels(
        label_image,
        rows[in_map],
        cols[in_map],
        static_filter_radius,
        num_classes,
    )
    labels[mapped] = np.where(
        static_labels != 0,
        static_labels,
        IGNORE_LABEL,
    )

    legs = pedestrian_leg_centers(pedestrian_xy, pedestrian_yaw)
    if len(legs):
        endpoints = np.column_stack((world_x, world_y))
        nearest_leg_distance = np.linalg.norm(
            endpoints[:, np.newaxis, :] - legs[np.newaxis, :, :],
            axis=2,
        ).min(axis=1)
        labels[
            indices[nearest_leg_distance <= PEDESTRIAN_LEG_MATCH_RADIUS_M]
        ] = person_label
    return labels


def padded_history(history: deque[np.ndarray]) -> np.ndarray:
    if not history:
        raise ValueError("cannot pad an empty history")
    values = list(history)
    return np.stack([values[0]] * (SCAN_HISTORY - len(values)) + values)


def build_observation(
    pedestrian_map: np.ndarray,
    scan_history: np.ndarray,
    local_goal: np.ndarray,
) -> np.ndarray:
    pedestrian = np.asarray(pedestrian_map, dtype=np.float32)
    scans = np.asarray(scan_history, dtype=np.float32)
    goal = np.asarray(local_goal, dtype=np.float32).reshape(-1)
    if pedestrian.shape != (2, 80, 80):
        raise ValueError(f"unexpected pedestrian map shape {pedestrian.shape}")
    if scans.shape != (SCAN_HISTORY, 720):
        raise ValueError(f"unexpected scan history shape {scans.shape}")
    if goal.shape != (2,):
        raise ValueError(f"unexpected local goal shape {goal.shape}")
    observation = np.concatenate(
        (
            np.clip(pedestrian / 2.0, -1.0, 1.0).reshape(-1),
            compress_scan_history(scans) / 15.0 - 1.0,
            goal / 2.0,
        )
    ).astype(np.float32)
    if observation.shape != (OBSERVATION_SIZE,):
        raise RuntimeError(f"unexpected observation shape {observation.shape}")
    if not np.isfinite(observation).all():
        raise ValueError("observation contains NaN or Inf")
    return observation


def minimum_front_range(
    virtual_ranges: np.ndarray,
    virtual_angles: np.ndarray,
    valid_mask: np.ndarray,
    half_angle: float,
) -> float | None:
    ranges = np.asarray(virtual_ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(virtual_angles, dtype=np.float32).reshape(-1)
    valid = np.asarray(valid_mask, dtype=np.bool_).reshape(-1)
    if ranges.shape != angles.shape or ranges.shape != valid.shape:
        raise ValueError("front-range inputs must have matching shapes")
    selected = ranges[
        valid
        & np.isfinite(ranges)
        & np.isfinite(angles)
        & (ranges > 0.0)
        & (np.abs(angles) <= float(half_angle))
    ]
    return float(np.min(selected)) if selected.size else None


def limit_physical_action(
    normalized_action: np.ndarray,
    max_linear: float,
    max_angular: float,
    front_min: float,
    front_stop_distance: float,
    local_goal_y: float,
    front_stop_angular_deadband: float,
    front_stop_min_angular: float,
) -> tuple[np.ndarray, np.ndarray, bool]:
    action = np.asarray(normalized_action, dtype=np.float32).reshape(-1)
    if action.shape != (2,) or not np.isfinite(action).all():
        raise ValueError("model action must be a finite 2-vector")
    raw_physical = normalized_to_physical(action)
    command = np.asarray(
        [
            np.clip(raw_physical[0], 0.0, max_linear),
            np.clip(raw_physical[1], -max_angular, max_angular),
        ],
        dtype=np.float32,
    )
    front_stop = math.isfinite(front_min) and front_min <= front_stop_distance
    if front_stop:
        command[0] = 0.0
        if abs(float(command[1])) < front_stop_angular_deadband:
            direction = 1.0 if local_goal_y >= 0.0 else -1.0
            command[1] = direction * min(
                front_stop_min_angular,
                max_angular,
            )
    return raw_physical, command, front_stop


def load_policy_checkpoint(
    checkpoint: Path,
    mode: str,
    device: str,
):
    if mode == "original":
        policy, weight_items = load_policy_strict(checkpoint)
        if weight_items != EXPECTED_BASE_WEIGHT_ITEMS:
            raise RuntimeError(
                f"original checkpoint has {weight_items} items; "
                f"expected {EXPECTED_BASE_WEIGHT_ITEMS}"
            )
    elif mode == "base":
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)
        if not isinstance(state, dict):
            raise TypeError(f"expected state dict, got {type(state)!r}")
        if len(state) != EXPECTED_BASE_WEIGHT_ITEMS:
            raise RuntimeError(
                f"base checkpoint has {len(state)} items; "
                f"expected {EXPECTED_BASE_WEIGHT_ITEMS}"
            )
        policy = DrlVoPolicy()
        policy.load_state_dict(state, strict=True)
        weight_items = len(state)
    elif mode in ("semantic", "semantic_no_ped"):
        policy, weight_items = load_trained_semantic_policy(
            checkpoint,
            SEMANTIC_NUM_CLASSES,
        )
        if weight_items != EXPECTED_SEMANTIC_WEIGHT_ITEMS:
            raise RuntimeError(
                f"semantic checkpoint has {weight_items} items; "
                f"expected {EXPECTED_SEMANTIC_WEIGHT_ITEMS}"
            )
    else:
        raise ValueError(
            "mode must be 'original', 'base', 'semantic', or 'semantic_no_ped'"
        )
    policy.to(device)
    policy.eval()
    return policy, weight_items


class DrlVoFixedDualInference(Node):
    def __init__(self) -> None:
        super().__init__("drl_vo_fixed_dual_inference")

        run_root = (
            PROJECT_ROOT / "runs" / "20260717_042135_v7_dual"
        )
        self.declare_parameter("mode", "base")
        self.declare_parameter("model", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter(
            "map_yaml",
            str(run_root / "maps" / "semantic_label" / "map.yaml"),
        )
        self.declare_parameter(
            "semantic_label",
            str(run_root / "maps" / "semantic_label" / "label.png"),
        )
        self.declare_parameter("scan_01_topic", "/scan_01")
        self.declare_parameter("scan_02_topic", "/scan_02")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "local_subgoal_topic",
            "/semantic_cnn/local_subgoal",
        )
        self.declare_parameter(
            "final_goal_topic",
            "/semantic_cnn/final_goal",
        )
        self.declare_parameter(
            "pedestrian_ground_truth_topic",
            "/pedestrian_ground_truth",
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "inference_metrics_topic", "/navigation_evaluation/inference_metrics"
        )
        self.declare_parameter("raw_cmd_topic", "/drl_vo/raw_model_cmd")
        self.declare_parameter("actuation_decision_topic", "/drl_vo/actuation_decision")
        self.declare_parameter(
            "training_state_topic", "/drl_vo/training_state"
        )
        self.declare_parameter(
            "episode_reset_topic", "/drl_vo/episode_reset"
        )
        self.declare_parameter(
            "control_event_topic", "/drl_vo/control_event"
        )
        self.declare_parameter("publish_policy_actions", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pedestrian_frame", "odom")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("enable_fixed_self_filter", True)
        self.declare_parameter("require_full_history", True)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("odom_timeout", 0.3)
        self.declare_parameter("subgoal_timeout", 0.3)
        self.declare_parameter("final_goal_timeout", 0.5)
        self.declare_parameter("pedestrian_truth_timeout", 0.15)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("front_half_angle", 0.35)
        self.declare_parameter("front_stop_distance", 0.5)
        self.declare_parameter("stop_on_empty_front", True)
        self.declare_parameter("front_stop_angular_deadband", 0.05)
        self.declare_parameter("front_stop_min_angular", 0.35)
        self.declare_parameter("enable_actuation_deadlock_detection", False)
        self.declare_parameter("actuation_deadlock_window_sec", 2.5)
        self.declare_parameter("actuation_deadlock_min_command_ratio", 0.8)
        self.declare_parameter("actuation_deadlock_goal_x_threshold", -0.05)
        self.declare_parameter("actuation_deadlock_max_linear_command", 0.02)
        self.declare_parameter("actuation_deadlock_min_angular_command", 0.05)
        self.declare_parameter("actuation_deadlock_max_displacement_m", 0.02)
        self.declare_parameter("actuation_deadlock_max_yaw_progress_rad", 0.03)
        self.declare_parameter("max_linear", 0.3)
        self.declare_parameter("max_angular", 1.5)
        self.declare_parameter("odom_jump_reset_distance", 1.0)
        self.declare_parameter("odom_jump_reset_yaw", 1.0)
        self.declare_parameter("static_label_filter_radius", 2)
        self.declare_parameter("oracle_pedestrian_velocity", True)
        self.declare_parameter("oracle_person_semantics", True)
        self.declare_parameter("require_pedestrian_truth", True)
        self.declare_parameter("pedestrian_source", "oracle")
        self.declare_parameter(
            "perception_model",
            str(
                run_root
                / "training"
                / "dual_lidar_pedestrian_bev"
                / "20260731_opt_velw100_h12_c24_v1"
                / "checkpoints"
                / "epoch_014.pt"
            ),
        )
        self.declare_parameter("perception_confidence_threshold", 0.4)
        self.declare_parameter("perception_topk", 30)
        self.declare_parameter("perception_nms_radius_m", 0.30)
        self.declare_parameter("coasting_max_time_s", 0.5)
        self.declare_parameter("max_track_age_s", 1.0)
        self.declare_parameter("include_tentative_tracks", False)
        self.declare_parameter("perception_metrics_path", "")

        self.mode = str(self.get_parameter("mode").value)
        self.uses_semantics = self.mode in ("semantic", "semantic_no_ped")
        self.pedestrian_source = str(
            self.get_parameter("pedestrian_source").value
        )
        if self.pedestrian_source not in ("oracle", "predicted", "zero"):
            raise ValueError(
                "pedestrian_source must be 'oracle', 'predicted', or 'zero'"
            )
        if self.mode == "semantic_no_ped":
            self.pedestrian_source = "zero"
        positive_parameters = (
            "sync_slop",
            "tf_timeout",
            "range_min",
            "range_max",
            "scan_timeout",
            "odom_timeout",
            "subgoal_timeout",
            "final_goal_timeout",
            "pedestrian_truth_timeout",
            "goal_tolerance",
            "front_half_angle",
            "front_stop_distance",
            "actuation_deadlock_window_sec",
            "max_linear",
            "max_angular",
            "odom_jump_reset_distance",
            "odom_jump_reset_yaw",
        )
        for parameter_name in positive_parameters:
            value = float(self.get_parameter(parameter_name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{parameter_name} must be a positive finite number"
                )
        if float(self.get_parameter("range_min").value) >= float(
            self.get_parameter("range_max").value
        ):
            raise ValueError("range_min must be smaller than range_max")
        for parameter_name in (
            "front_stop_angular_deadband",
            "front_stop_min_angular",
            "actuation_deadlock_max_linear_command",
            "actuation_deadlock_min_angular_command",
            "actuation_deadlock_max_displacement_m",
            "actuation_deadlock_max_yaw_progress_rad",
            "perception_nms_radius_m",
            "coasting_max_time_s",
            "max_track_age_s",
        ):
            value = float(self.get_parameter(parameter_name).value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{parameter_name} must be a non-negative finite number"
                )
        deadlock_goal_x_threshold = float(
            self.get_parameter("actuation_deadlock_goal_x_threshold").value
        )
        if not math.isfinite(deadlock_goal_x_threshold):
            raise ValueError(
                "actuation_deadlock_goal_x_threshold must be finite"
            )
        deadlock_command_ratio = float(
            self.get_parameter("actuation_deadlock_min_command_ratio").value
        )
        if not 0.0 < deadlock_command_ratio <= 1.0:
            raise ValueError(
                "actuation_deadlock_min_command_ratio must be in (0,1]"
            )
        confidence_threshold = float(
            self.get_parameter("perception_confidence_threshold").value
        )
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError(
                "perception_confidence_threshold must be in [0,1]"
            )
        if int(self.get_parameter("perception_topk").value) < 1:
            raise ValueError("perception_topk must be positive")
        device_requested = str(self.get_parameter("device").value)
        if device_requested == "auto":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device_requested
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")

        model_path = Path(str(self.get_parameter("model").value)).resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"DRL-VO checkpoint does not exist: {model_path}")
        if (
            self.pedestrian_source == "oracle"
            and not bool(self.get_parameter("oracle_pedestrian_velocity").value)
        ):
            raise ValueError(
                "oracle pedestrian source requires "
                "oracle_pedestrian_velocity:=true"
            )
        self.require_pedestrian_truth = bool(
            self.get_parameter("require_pedestrian_truth").value
        )
        if (
            self.pedestrian_source == "oracle"
            and not self.require_pedestrian_truth
        ):
            raise ValueError(
                "oracle pedestrian source requires fresh pedestrian truth"
            )
        if (
            self.pedestrian_source in ("predicted", "zero")
            and self.require_pedestrian_truth
            and not self.uses_semantics
        ):
            raise ValueError(
                f"{self.pedestrian_source} pedestrian source must use "
                "require_pedestrian_truth:=false"
            )
        if (
            self.pedestrian_source == "predicted"
            and self.mode not in ("original", "base")
        ):
            raise ValueError(
                "predicted pedestrian source supports original/base policies only"
            )
        if (
            self.uses_semantics
            and not bool(self.get_parameter("oracle_person_semantics").value)
        ):
            raise ValueError(
                "semantic mode was trained with ground-truth lower-leg Person "
                "labels and requires oracle_person_semantics:=true"
            )
        if self.uses_semantics and not self.require_pedestrian_truth:
            raise ValueError(
                "semantic mode requires fresh pedestrian truth to reproduce "
                "the oracle lower-leg Person-label contract"
            )
        if (
            self.uses_semantics
            and int(self.get_parameter("static_label_filter_radius").value)
            != STATIC_LABEL_FILTER_RADIUS
        ):
            raise ValueError(
                "semantic mode requires static_label_filter_radius:=2 to "
                "match training"
            )
        self.policy, weight_items = load_policy_checkpoint(
            model_path,
            self.mode,
            self.device,
        )
        self.model_parameters = sum(
            parameter.numel() for parameter in self.policy.parameters()
        )
        self.inference_sequence_id = 0
        self.perception_model = None
        self.perception_bev_spec = None
        self.perception_history_frames = 0
        self.perception_tracker = PedestrianTracker()
        self.perception_history: deque[TemporalLidarFrame] = deque()
        self.last_perception_timestamp_ns: int | None = None
        self.perception_metrics_stream = None
        if self.pedestrian_source == "predicted":
            perception_model_path = Path(
                str(self.get_parameter("perception_model").value)
            ).resolve()
            if not perception_model_path.is_file():
                raise FileNotFoundError(
                    f"perception checkpoint does not exist: {perception_model_path}"
                )
            (
                self.perception_model,
                self.perception_bev_spec,
                self.perception_history_frames,
            ) = load_perception_checkpoint(
                perception_model_path,
                self.device,
            )
            self.perception_history = deque(
                maxlen=self.perception_history_frames
            )
            metrics_path_text = str(
                self.get_parameter("perception_metrics_path").value
            ).strip()
            if metrics_path_text:
                metrics_path = Path(metrics_path_text).expanduser().resolve()
                metrics_path.parent.mkdir(parents=True, exist_ok=True)
                self.perception_metrics_stream = metrics_path.open(
                    "x", encoding="utf-8"
                )

        self.resolution, self.origin_x, self.origin_y, self.label_image = (
            self._load_semantic_map()
        )
        self.scan_history: deque[np.ndarray] = deque(maxlen=SCAN_HISTORY)
        self.semantic_history: deque[np.ndarray] = deque(maxlen=SCAN_HISTORY)
        self.sensor_layouts: dict[str, tuple] = {}
        self.fixed_self_masks: dict[str, np.ndarray] = {}
        self.pose: np.ndarray | None = None
        self.pose_stamp_ns: int | None = None
        self.subgoal: np.ndarray | None = None
        self.subgoal_stamp_ns: int | None = None
        self.subgoal_history: deque[tuple[int, np.ndarray]] = deque(maxlen=100)
        self.final_goal: np.ndarray | None = None
        self.final_goal_stamp_ns: int | None = None
        self.actions_inhibited_after_reset = False
        self.reset_goal: np.ndarray | None = None
        self.actuation_history: deque[ActuationSample] = deque()
        self.pedestrian_xy = np.empty((0, 2), dtype=np.float32)
        self.pedestrian_yaw = np.empty(0, dtype=np.float32)
        self.pedestrian_velocity = np.empty((0, 2), dtype=np.float32)
        self.pedestrian_stamp_ns: int | None = None
        self.last_scan_clock_ns: int | None = None
        self.last_clock_ns: int | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.raw_cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("raw_cmd_topic").value),
            10,
        )
        self.actuation_decision_pub = self.create_publisher(
            ActuationDecision,
            str(self.get_parameter("actuation_decision_topic").value),
            30,
        )
        self.actuation_decision_sequence_id = 0
        self.training_state_pub = self.create_publisher(
            DrlVoTrainingState,
            str(self.get_parameter("training_state_topic").value),
            10,
        )
        self.inference_metrics_pub = self.create_publisher(
            InferenceMetrics,
            str(self.get_parameter("inference_metrics_topic").value),
            10,
        )
        self.control_event_pub = self.create_publisher(
            String,
            str(self.get_parameter("control_event_topic").value),
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("episode_reset_topic").value),
            self.episode_reset_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("local_subgoal_topic").value),
            self.subgoal_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("final_goal_topic").value),
            self.final_goal_callback,
            10,
        )
        if self.require_pedestrian_truth:
            self.create_subscription(
                PedestrianStateArray,
                str(self.get_parameter("pedestrian_ground_truth_topic").value),
                self.pedestrian_callback,
                10,
            )
        scan_01_sub = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_01_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        scan_02_sub = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_02_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.scan_sync = message_filters.ApproximateTimeSynchronizer(
            [scan_01_sub, scan_02_sub],
            queue_size=10,
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.scan_sync.registerCallback(self.scan_callback)
        self.create_timer(0.1, self.watchdog_callback)
        if self.pedestrian_source == "oracle":
            self.get_logger().warning(
                "DRL-VO online demo uses oracle Gazebo pedestrian positions and "
                f"velocities; mode={self.mode}, weights={weight_items}, "
                f"device={self.device}, checkpoint={model_path}"
            )
        elif self.pedestrian_source == "predicted":
            self.get_logger().warning(
                "DRL-VO online demo uses truth-free dual-lidar pedestrian "
                f"prediction; mode={self.mode}, weights={weight_items}, "
                f"device={self.device}, checkpoint={model_path}"
            )
        elif self.uses_semantics:
            self.get_logger().warning(
                "semantic_no_ped fixes pedestrian vx/vy channels at zero but "
                "still uses oracle Gazebo pedestrian positions for Person "
                f"labels; weights={weight_items}, device={self.device}, "
                f"checkpoint={model_path}"
            )
        else:
            self.get_logger().warning(
                "pedestrian_source=zero: forcing an all-zero pedestrian "
                f"velocity map; mode={self.mode}, weights={weight_items}, "
                f"device={self.device}, checkpoint={model_path}. This is OOD "
                "and is only for static-scene safety integration, not model "
                "comparison."
            )

    def _emit_inference_metrics(
        self,
        sequence_id: int,
        input_stamp,
        preprocessing_ms: float,
        policy_ms: float,
        postprocessing_ms: float,
        total_ms: float,
        success: bool,
        raw_physical: np.ndarray | None = None,
    ) -> None:
        """Publish passive telemetry without changing command/control behavior."""
        metrics = InferenceMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.input_stamp = input_stamp
        metrics.sequence_id = sequence_id
        metrics.producer_id = "drl_vo_policy"
        metrics.success = success
        metrics.preprocessing_ms = float(preprocessing_ms)
        metrics.policy_ms = float(policy_ms)
        metrics.postprocessing_ms = float(postprocessing_ms)
        metrics.total_ms = float(total_ms)
        metrics.device = self.device
        metrics.model_parameters = int(self.model_parameters)
        if self.device.startswith("cuda"):
            metrics.cuda_memory_allocated_bytes = int(
                torch.cuda.memory_allocated(self.device)
            )
            metrics.cuda_peak_memory_bytes = int(
                torch.cuda.max_memory_allocated(self.device)
            )
        else:
            metrics.cuda_memory_allocated_bytes = 0
            metrics.cuda_peak_memory_bytes = 0
        metrics.action_encoding = "twist_linear_x_mps_angular_z_radps"
        metrics.action = (
            [float(raw_physical[0]), float(raw_physical[1])]
            if raw_physical is not None
            else []
        )
        self.inference_metrics_pub.publish(metrics)

    def _publish_inference_metrics(self, *args, **kwargs) -> None:
        """Keep telemetry failures from affecting policy/control behavior."""
        try:
            self._emit_inference_metrics(*args, **kwargs)
        except Exception as exc:  # ROS publication must remain best-effort.
            self.get_logger().warning(
                f"inference metrics publication failed: {exc}",
                throttle_duration_sec=2.0,
            )

    def _load_semantic_map(self):
        map_yaml = Path(str(self.get_parameter("map_yaml").value))
        label_path = Path(str(self.get_parameter("semantic_label").value))
        with map_yaml.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        origin = metadata["origin"]
        if len(origin) < 3 or not math.isclose(
            float(origin[2]), 0.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("the training semantic projection requires map yaw 0")
        label_array = np.asarray(Image.open(label_path))
        if label_array.ndim == 3:
            label_array = label_array[:, :, 0]
        label_array = label_array.astype(np.int64)
        if (
            label_array.ndim != 2
            or np.min(label_array) < 0
            or np.max(label_array) >= SEMANTIC_NUM_CLASSES
        ):
            raise ValueError(
                "semantic label image must be 2-D with labels in [0, 6]"
            )
        return (
            float(metadata["resolution"]),
            float(origin[0]),
            float(origin[1]),
            label_array,
        )

    def _clock_now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _observe_clock(self) -> int:
        now_ns = self._clock_now_ns()
        if clock_rolled_back(self.last_clock_ns, now_ns):
            self.get_logger().warning(
                "simulation clock rolled back; clearing all temporal inputs"
            )
            self._reset_runtime_inputs(clear_sensor_contract=True)
            self.publish_stop("simulation_clock_rollback")
        self.last_clock_ns = now_ns
        return now_ns

    def _clear_history(self) -> None:
        self.scan_history.clear()
        self.semantic_history.clear()
        self._clear_perception_history()

    def _clear_perception_history(self) -> None:
        if hasattr(self, "perception_history"):
            self.perception_history.clear()
        if hasattr(self, "perception_tracker"):
            self.perception_tracker.reset()
        self.last_perception_timestamp_ns = None

    def _record_perception_stats(
        self,
        timestamp_ns: int,
        stats: dict[str, float | int],
    ) -> None:
        if self.perception_metrics_stream is None:
            return
        record = {
            "timestamp_ns": int(timestamp_ns),
            **stats,
        }
        self.perception_metrics_stream.write(
            json.dumps(record, sort_keys=True) + "\n"
        )
        self.perception_metrics_stream.flush()

    def _predicted_pedestrian_map(
        self,
        virtual_ranges: np.ndarray,
        virtual_angles: np.ndarray,
        valid: np.ndarray,
        sensor_01_size: int,
        timestamp_ns: int,
    ) -> tuple[np.ndarray, dict[str, float | int]]:
        """Run the truth-free temporal detector, tracker, and DRL map adapter."""

        zero_map = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
        if (
            self.perception_model is None
            or self.perception_bev_spec is None
            or self.pose is None
        ):
            return zero_map, {"warming_up": 1}
        started = time.perf_counter()
        try:
            if (
                self.last_perception_timestamp_ns is not None
                and timestamp_ns <= self.last_perception_timestamp_ns
            ):
                raise ValueError(
                    "perception timestamps must be strictly increasing"
                )
            usable = (
                np.asarray(valid, dtype=np.bool_)
                & np.isfinite(virtual_ranges)
                & np.isfinite(virtual_angles)
                & (virtual_ranges > 0.0)
            )
            indices = np.flatnonzero(usable)
            points = np.column_stack(
                (
                    virtual_ranges[indices] * np.cos(virtual_angles[indices]),
                    virtual_ranges[indices] * np.sin(virtual_angles[indices]),
                )
            ).astype(np.float32)
            sensors = (indices >= int(sensor_01_size)).astype(np.int64)
            self.perception_history.append(
                TemporalLidarFrame(
                    points_xy_base=points,
                    sensor_indices=sensors,
                    robot_pose_map=self.pose.copy(),
                    timestamp_ns=int(timestamp_ns),
                )
            )
            self.last_perception_timestamp_ns = int(timestamp_ns)
            if len(self.perception_history) < self.perception_history_frames:
                return zero_map, {"warming_up": 1}

            bev = build_temporal_lidar_bev(
                list(self.perception_history),
                self.perception_bev_spec,
                self.perception_history_frames,
            )
            inference_started = time.perf_counter()
            with torch.inference_mode():
                outputs = self.perception_model(
                    torch.from_numpy(bev).unsqueeze(0).to(self.device)
                )
            if self.device.startswith("cuda"):
                torch.cuda.synchronize(self.device)
            inference_ms = (
                time.perf_counter() - inference_started
            ) * 1000.0
            if any(
                not torch.isfinite(output).all()
                for output in outputs.values()
            ):
                raise ValueError("perception model produced NaN or Inf")
            detections_base = decode_detections(
                outputs,
                self.perception_bev_spec,
                confidence_threshold=float(
                    self.get_parameter(
                        "perception_confidence_threshold"
                    ).value
                ),
                topk=int(self.get_parameter("perception_topk").value),
                nms_radius_m=float(
                    self.get_parameter("perception_nms_radius_m").value
                ),
            )[0]
            detections_map = detections_base_to_map(
                detections_base, self.pose
            )
            tracker_started = time.perf_counter()
            tracks = self.perception_tracker.update(
                detections_map, timestamp_ns
            )
            tracker_ms = (
                time.perf_counter() - tracker_started
            ) * 1000.0
            pedestrian_map, diagnostics = (
                tracks_to_drl_vo_ped_map_with_diagnostics(
                    tracks,
                    self.pose,
                    coasting_max_time_s=float(
                        self.get_parameter("coasting_max_time_s").value
                    ),
                    max_track_age_s=float(
                        self.get_parameter("max_track_age_s").value
                    ),
                    include_tentative=bool(
                        self.get_parameter(
                            "include_tentative_tracks"
                        ).value
                    ),
                )
            )
            return pedestrian_map, {
                "warming_up": 0,
                "detections": len(detections_base),
                "tracks": len(tracks),
                "written_tracks": len(diagnostics["written_track_ids"]),
                "same_cell_conflicts": int(
                    diagnostics["same_cell_conflict_count"]
                ),
                "dropped_tracks": int(diagnostics["dropped_track_count"]),
                "inference_ms": inference_ms,
                "tracker_ms": tracker_ms,
                "end_to_end_ms": (
                    time.perf_counter() - started
                )
                * 1000.0,
            }
        except Exception as exc:
            self._clear_perception_history()
            self.get_logger().error(
                f"pedestrian perception failed; using zero map: {exc}",
                throttle_duration_sec=2.0,
            )
            return zero_map, {
                "warming_up": 0,
                "error": 1,
                "end_to_end_ms": (
                    time.perf_counter() - started
                )
                * 1000.0,
            }

    def _clear_subgoal_state(self) -> None:
        self.subgoal = None
        self.subgoal_stamp_ns = None
        self.subgoal_history.clear()

    def _reset_runtime_inputs(self, clear_sensor_contract: bool) -> None:
        self._clear_history()
        self._clear_actuation_deadlock_state()
        self.pose = None
        self.pose_stamp_ns = None
        self._clear_subgoal_state()
        self.final_goal = None
        self.final_goal_stamp_ns = None
        self.pedestrian_xy = np.empty((0, 2), dtype=np.float32)
        self.pedestrian_yaw = np.empty(0, dtype=np.float32)
        self.pedestrian_velocity = np.empty((0, 2), dtype=np.float32)
        self.pedestrian_stamp_ns = None
        self.last_scan_clock_ns = None
        if clear_sensor_contract:
            self.sensor_layouts.clear()
            self.fixed_self_masks.clear()

    def _publish_actuation_decision(
        self,
        raw_physical: np.ndarray | None,
        command: np.ndarray | None,
        *,
        front_min: float | None = None,
        reasons: tuple[str, ...] = (),
        inference_sequence_id: int = 0,
    ) -> None:
        """Atomically expose raw/final policy control without changing control."""
        message = ActuationDecision()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("base_frame").value)
        message.input_stamp = message.header.stamp
        self.actuation_decision_sequence_id += 1
        message.decision_sequence_id = self.actuation_decision_sequence_id
        message.inference_sequence_id = int(inference_sequence_id)
        message.has_raw_action = raw_physical is not None
        final = np.zeros(2, dtype=float) if command is None else np.asarray(command, dtype=float)
        if raw_physical is not None:
            raw = np.asarray(raw_physical, dtype=float)
            message.raw_physical_action.linear.x = float(raw[0])
            message.raw_physical_action.angular.z = float(raw[1])
            if not reasons and not np.allclose(raw, final, rtol=0.0, atol=1.0e-6):
                reasons = ("limit_or_gate",)
        message.final_command.linear.x = float(final[0])
        message.final_command.angular.z = float(final[1])
        message.gated = bool(reasons)
        message.gate_reasons = list(reasons)
        message.has_front_min_range = front_min is not None and math.isfinite(front_min)
        message.front_min_range_m = float(front_min) if message.has_front_min_range else 0.0
        self.actuation_decision_pub.publish(message)

    def publish_stop(self, reason: str = "stop") -> None:
        self._publish_actuation_decision(None, None, reasons=(reason,))
        self.cmd_pub.publish(Twist())

    def _clear_actuation_deadlock_state(self) -> None:
        self.actuation_history.clear()

    def _observe_actuation_deadlock(
        self,
        stamp_ns: int,
        command: np.ndarray,
        causal_subgoal: np.ndarray,
    ) -> bool:
        """Latch a stop and notify the scheduler after verified non-response."""

        enabled = bool(
            self.get_parameter("enable_actuation_deadlock_detection").value
        )
        if (
            not enabled
            or self.actions_inhibited_after_reset
            or self.pose is None
            or self.final_goal is None
            or float(np.linalg.norm(self.final_goal - self.pose[:2]))
            <= float(self.get_parameter("goal_tolerance").value)
        ):
            self._clear_actuation_deadlock_state()
            return False
        goal_x_threshold = float(
            self.get_parameter("actuation_deadlock_goal_x_threshold").value
        )
        max_linear_command = float(
            self.get_parameter("actuation_deadlock_max_linear_command").value
        )
        minimum_angular_command = float(
            self.get_parameter("actuation_deadlock_min_angular_command").value
        )
        monitoring_window = (
            float(causal_subgoal[0]) < goal_x_threshold
            and abs(float(command[0])) <= max_linear_command
        )
        if not monitoring_window:
            self._clear_actuation_deadlock_state()
            return False
        self.actuation_history.append(
            ActuationSample(
                stamp_ns=int(stamp_ns),
                command_linear=float(command[0]),
                command_angular=float(command[1]),
                x=float(self.pose[0]),
                y=float(self.pose[1]),
                yaw=float(self.pose[2]),
            )
        )
        window_sec = float(
            self.get_parameter("actuation_deadlock_window_sec").value
        )
        cutoff_ns = int(stamp_ns) - int(window_sec * 1e9)
        while (
            len(self.actuation_history) > 1
            and self.actuation_history[1].stamp_ns <= cutoff_ns
        ):
            self.actuation_history.popleft()
        detected = actuation_deadlock_detected(
            self.actuation_history,
            window_sec,
            float(
                self.get_parameter(
                    "actuation_deadlock_min_command_ratio"
                ).value
            ),
            max_linear_command,
            minimum_angular_command,
            float(
                self.get_parameter(
                    "actuation_deadlock_max_displacement_m"
                ).value
            ),
            float(
                self.get_parameter(
                    "actuation_deadlock_max_yaw_progress_rad"
                ).value
            ),
        )
        if not detected:
            return False
        history = list(self.actuation_history)
        payload = {
            "schema": "drl_vo_control_event/v1",
            "event": "actuation_deadlock",
            "stamp_ns": int(stamp_ns),
            "window_sec": (
                history[-1].stamp_ns - history[0].stamp_ns
            ) / 1e9,
            "local_goal": [
                float(causal_subgoal[0]),
                float(causal_subgoal[1]),
            ],
            "final_goal": [float(value) for value in self.final_goal],
            "command": [float(command[0]), float(command[1])],
            "pose": [float(value) for value in self.pose],
        }
        message = String()
        message.data = json.dumps(
            payload, sort_keys=True, separators=(",", ":")
        )
        self.control_event_pub.publish(message)
        self.reset_goal = self.final_goal.copy()
        self.actions_inhibited_after_reset = True
        self._clear_history()
        self._clear_subgoal_state()
        self._clear_actuation_deadlock_state()
        self.publish_stop("actuation_deadlock")
        self.get_logger().warning(
            "ACTUATION_DEADLOCK: command persisted without pose/yaw response; "
            "teacher stopped and current episode must be discarded"
        )
        return True

    def episode_reset_callback(self, _message: Empty) -> None:
        """Clear every causal history when the training environment resets."""

        self.reset_goal = (
            self.final_goal.copy() if self.final_goal is not None else None
        )
        self.actions_inhibited_after_reset = True
        self._clear_actuation_deadlock_state()
        self._clear_history()
        self._clear_subgoal_state()
        self.last_scan_clock_ns = None
        self.publish_stop("episode_reset_inhibit")

    def odom_callback(self, message: Odometry) -> None:
        self._observe_clock()
        expected_frame = str(self.get_parameter("odom_frame").value).lstrip("/")
        actual_frame = message.header.frame_id.lstrip("/")
        if actual_frame and actual_frame != expected_frame:
            self.get_logger().error(
                f"rejecting odom frame {actual_frame!r}; expected {expected_frame!r}"
            )
            self.pose = None
            self.pose_stamp_ns = None
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("invalid_odom_frame")
            return
        position = message.pose.pose.position
        pose = np.asarray(
            [
                float(position.x),
                float(position.y),
                yaw_from_quaternion(message.pose.pose.orientation),
            ],
            dtype=np.float32,
        )
        if not np.isfinite(pose).all():
            self.pose = None
            self.pose_stamp_ns = None
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("nonfinite_odom")
            return
        pose_jump = False
        if self.pose is not None:
            translation_jump = float(np.linalg.norm(pose[:2] - self.pose[:2]))
            yaw_delta = float(pose[2] - self.pose[2])
            yaw_jump = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
            pose_jump = (
                translation_jump
                > float(
                    self.get_parameter("odom_jump_reset_distance").value
                )
                or yaw_jump
                > float(self.get_parameter("odom_jump_reset_yaw").value)
            )
        if pose_jump:
            self.get_logger().warning(
                "odom jump detected; clearing scan and local-subgoal history"
            )
            self._clear_subgoal_state()
            self._clear_history()
            self._clear_actuation_deadlock_state()
            self.publish_stop("odom_reset_jump")
        self.pose = pose
        self.pose_stamp_ns = stamp_to_nanoseconds(message.header.stamp)

    def subgoal_callback(self, message: PointStamped) -> None:
        self._observe_clock()
        expected_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        actual_frame = message.header.frame_id.lstrip("/")
        if actual_frame and actual_frame != expected_frame:
            self.get_logger().error(
                f"rejecting subgoal frame {actual_frame!r}; expected {expected_frame!r}"
            )
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("invalid_subgoal_frame")
            return
        goal = np.asarray(
            [float(message.point.x), float(message.point.y)],
            dtype=np.float32,
        )
        if not np.isfinite(goal).all():
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("nonfinite_subgoal")
            return
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        self.subgoal_history.append((stamp_ns, goal.copy()))
        if self.subgoal_stamp_ns is None or stamp_ns >= self.subgoal_stamp_ns:
            self.subgoal = goal
            self.subgoal_stamp_ns = stamp_ns

    def final_goal_callback(self, message: PointStamped) -> None:
        self._observe_clock()
        expected_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        actual_frame = message.header.frame_id.lstrip("/")
        if actual_frame and actual_frame != expected_frame:
            self.get_logger().error(
                f"rejecting final-goal frame {actual_frame!r}; "
                f"expected {expected_frame!r}"
            )
            self.final_goal = None
            self.final_goal_stamp_ns = None
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("invalid_final_goal_frame")
            return
        goal = np.asarray(
            [float(message.point.x), float(message.point.y)],
            dtype=np.float32,
        )
        if not np.isfinite(goal).all():
            self.final_goal = None
            self.final_goal_stamp_ns = None
            self._clear_subgoal_state()
            self._clear_history()
            self.publish_stop("nonfinite_final_goal")
            return
        if (
            self.final_goal is not None
            and float(np.linalg.norm(goal - self.final_goal)) > 1e-4
        ):
            self._clear_history()
            self.publish_stop("goal_changed")
        self.final_goal = goal
        self.final_goal_stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if self.actions_inhibited_after_reset:
            goal_changed = final_goal_rearms_after_reset(
                self.reset_goal, goal, tolerance_m=1e-4
            )
            if goal_changed:
                self.actions_inhibited_after_reset = False
                self.reset_goal = None
                self._clear_actuation_deadlock_state()
                self._clear_history()
                self.publish_stop("reset_rearmed")
                self.get_logger().info(
                    "Fresh final goal received; teacher actions re-armed"
                )
            else:
                self.publish_stop("reset_inhibit")

    def pedestrian_callback(self, message: PedestrianStateArray) -> None:
        self._observe_clock()
        expected_frame = str(
            self.get_parameter("pedestrian_frame").value
        ).lstrip("/")
        actual_frame = message.header.frame_id.lstrip("/")
        if actual_frame and actual_frame != expected_frame:
            self.get_logger().error(
                f"rejecting pedestrian frame {actual_frame!r}; "
                f"expected {expected_frame!r}"
            )
            self.pedestrian_stamp_ns = None
            self._clear_history()
            return
        xy = []
        yaw = []
        velocity = []
        for pedestrian in message.pedestrians:
            xy.append(
                [
                    float(pedestrian.pose.position.x),
                    float(pedestrian.pose.position.y),
                ]
            )
            yaw.append(yaw_from_quaternion(pedestrian.pose.orientation))
            velocity.append(
                [
                    float(pedestrian.velocity.linear.x),
                    float(pedestrian.velocity.linear.y),
                ]
            )
        xy_array = np.asarray(xy, dtype=np.float32).reshape((-1, 2))
        yaw_array = np.asarray(yaw, dtype=np.float32)
        velocity_array = np.asarray(velocity, dtype=np.float32).reshape((-1, 2))
        if not (
            np.isfinite(xy_array).all()
            and np.isfinite(yaw_array).all()
            and np.isfinite(velocity_array).all()
        ):
            self.pedestrian_stamp_ns = None
            self._clear_history()
            self.publish_stop("nonfinite_pedestrian_truth")
            return
        self.pedestrian_xy = xy_array
        self.pedestrian_yaw = yaw_array
        self.pedestrian_velocity = velocity_array
        self.pedestrian_stamp_ns = stamp_to_nanoseconds(message.header.stamp)

    def _input_status(
        self,
        now_ns: int,
        require_subgoal: bool = True,
    ) -> tuple[bool, str]:
        checks = [
            (
                self.pose is not None
                and time_is_fresh(
                    now_ns,
                    self.pose_stamp_ns,
                    float(self.get_parameter("odom_timeout").value),
                ),
                "odom missing or stale",
            ),
        ]
        if require_subgoal:
            checks.append(
                (
                    self.subgoal is not None
                    and time_is_fresh(
                        now_ns,
                        self.subgoal_stamp_ns,
                        float(self.get_parameter("subgoal_timeout").value),
                    ),
                    "local subgoal missing or stale",
                )
            )
        checks.extend(
            [
                (
                    self.final_goal is not None
                    and time_is_fresh(
                        now_ns,
                        self.final_goal_stamp_ns,
                        float(self.get_parameter("final_goal_timeout").value),
                    ),
                    "final goal missing or stale",
                ),
            ]
        )
        for passed, reason in checks:
            if not passed:
                return False, reason
        if self.require_pedestrian_truth and not time_is_fresh(
            now_ns,
            self.pedestrian_stamp_ns,
            float(self.get_parameter("pedestrian_truth_timeout").value),
        ):
            return False, "pedestrian truth missing or stale"
        return True, "ready"

    def _virtualize_scan(
        self,
        message: LaserScan,
        sensor_key: str,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        current_layout = scan_layout(message)
        previous_layout = self.sensor_layouts.get(sensor_key)
        if (
            previous_layout is not None
            and not scan_layout_matches(previous_layout, current_layout)
        ):
            raise RuntimeError(
                f"{sensor_key} LaserScan layout changed; fixed-beam contract invalid"
            )
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("base_frame").value),
                message.header.frame_id.strip(),
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("tf_timeout").value)
                ),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"TF unavailable for {message.header.frame_id}: {exc}"
            ) from exc
        translation_msg = transform.transform.translation
        rotation_msg = transform.transform.rotation
        frozen_mask = (
            self.fixed_self_masks.get(sensor_key)
            if bool(self.get_parameter("enable_fixed_self_filter").value)
            else np.zeros(len(message.ranges), dtype=np.bool_)
        )
        ranges, angles, valid, footprint_mask = transform_scan_to_base(
            np.asarray(message.ranges, dtype=np.float32),
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
            (
                float(translation_msg.x),
                float(translation_msg.y),
                float(translation_msg.z),
            ),
            (
                float(rotation_msg.x),
                float(rotation_msg.y),
                float(rotation_msg.z),
                float(rotation_msg.w),
            ),
            float(self.get_parameter("range_min").value),
            float(self.get_parameter("range_max").value),
            frozen_mask,
        )
        if previous_layout is None:
            self.sensor_layouts[sensor_key] = current_layout
            if bool(self.get_parameter("enable_fixed_self_filter").value):
                self.fixed_self_masks[sensor_key] = footprint_mask.copy()
        return ranges, angles, valid

    def scan_callback(
        self,
        scan_01: LaserScan,
        scan_02: LaserScan,
    ) -> None:
        now_ns = self._observe_clock()
        self.last_scan_clock_ns = now_ns
        scan_01_stamp_ns = stamp_to_nanoseconds(scan_01.header.stamp)
        scan_02_stamp_ns = stamp_to_nanoseconds(scan_02.header.stamp)
        if (
            abs(scan_01_stamp_ns - scan_02_stamp_ns)
            > int(
                float(self.get_parameter("sync_slop").value)
                * NANOSECONDS_PER_SECOND
            )
            or not time_is_fresh(
                now_ns,
                scan_01_stamp_ns,
                float(self.get_parameter("scan_timeout").value),
            )
            or not time_is_fresh(
                now_ns,
                scan_02_stamp_ns,
                float(self.get_parameter("scan_timeout").value),
            )
        ):
            self._clear_history()
            self.publish_stop("scan_pair_stale_or_unsynchronized")
            return
        ready, reason = self._input_status(now_ns, require_subgoal=False)
        if not ready:
            self._clear_history()
            self.publish_stop("input_missing_or_stale")
            self.get_logger().warning(reason, throttle_duration_sec=2.0)
            return
        causal_subgoal_sample = latest_causal_sample(
            self.subgoal_history,
            scan_01_stamp_ns,
            float(self.get_parameter("subgoal_timeout").value),
        )
        if causal_subgoal_sample is None:
            self._clear_history()
            self.publish_stop("causal_subgoal_missing_or_stale")
            self.get_logger().warning(
                "no causal local subgoal at or before scan_01 within timeout",
                throttle_duration_sec=2.0,
            )
            return
        causal_subgoal, _causal_subgoal_stamp_ns = causal_subgoal_sample

        try:
            ranges_01, angles_01, valid_01 = self._virtualize_scan(
                scan_01,
                "scan_01",
            )
            ranges_02, angles_02, valid_02 = self._virtualize_scan(
                scan_02,
                "scan_02",
            )
        except (RuntimeError, ValueError) as exc:
            self._clear_history()
            self.publish_stop("scan_contract_error")
            self.get_logger().error(str(exc))
            return

        virtual_ranges = np.concatenate((ranges_01, ranges_02))
        virtual_angles = np.concatenate((angles_01, angles_02))
        valid = np.concatenate((valid_01, valid_02))
        if self.pedestrian_source == "predicted" and (
            len(ranges_01) != 2000 or len(ranges_02) != 2000
        ):
            self._clear_history()
            self.publish_stop("dual_lidar_contract_mismatch")
            self.get_logger().error(
                "predicted pedestrian source requires the trained "
                "2000+2000 fixed-slot LiDAR contract"
            )
            return
        front_min_value = minimum_front_range(
            virtual_ranges,
            virtual_angles,
            valid,
            float(self.get_parameter("front_half_angle").value),
        )
        if (
            front_min_value is None
            and bool(self.get_parameter("stop_on_empty_front").value)
        ):
            self._clear_history()
            self.publish_stop("empty_front_scan")
            self.get_logger().warning(
                "front scan has no valid returns; fail-safe stop",
                throttle_duration_sec=2.0,
            )
            return
        front_min = (
            front_min_value
            if front_min_value is not None
            else float("inf")
        )

        if self.uses_semantics:
            semantic_labels = semantic_labels_for_virtual_scan(
                virtual_ranges,
                virtual_angles,
                self.pose,
                self.label_image,
                self.resolution,
                self.origin_x,
                self.origin_y,
                self.pedestrian_xy,
                self.pedestrian_yaw,
                static_filter_radius=int(
                    self.get_parameter("static_label_filter_radius").value
                ),
            )
            (
                front_scan,
                front_semantic,
                _coverage,
                _nearest,
            ) = dual_lidar_to_legacy_semantic(
                virtual_ranges,
                virtual_angles,
                semantic_labels,
            )
            self.semantic_history.append(front_semantic)
        else:
            front_scan, _coverage, _nearest = dual_lidar_to_legacy_scan(
                virtual_ranges,
                virtual_angles,
            )
        self.scan_history.append(front_scan)
        predicted_pedestrian_map = None
        if self.pedestrian_source == "predicted":
            perception_timestamp_ns = max(
                scan_01_stamp_ns, scan_02_stamp_ns
            )
            predicted_pedestrian_map, perception_stats = (
                self._predicted_pedestrian_map(
                    virtual_ranges,
                    virtual_angles,
                    valid,
                    len(ranges_01),
                    perception_timestamp_ns,
                )
            )
            self._record_perception_stats(
                perception_timestamp_ns,
                perception_stats,
            )
            if (
                not bool(perception_stats.get("warming_up", 0))
                and "end_to_end_ms" in perception_stats
            ):
                self.get_logger().info(
                    "pedestrian perception "
                    f"tracks={perception_stats.get('tracks', 0)}, "
                    f"written={perception_stats.get('written_tracks', 0)}, "
                    f"latency={perception_stats['end_to_end_ms']:.2f} ms",
                    throttle_duration_sec=2.0,
                )

        if (
            bool(self.get_parameter("require_full_history").value)
            and len(self.scan_history) < SCAN_HISTORY
        ):
            self.publish_stop("history_warmup")
            return

        history = padded_history(self.scan_history)
        if self.pedestrian_source == "oracle":
            pedestrian_map, _nearest_pedestrian = pedestrian_velocity_map(
                self.pedestrian_xy,
                self.pedestrian_velocity,
                self.pose,
            )
        elif self.pedestrian_source == "predicted":
            pedestrian_map = (
                predicted_pedestrian_map
                if predicted_pedestrian_map is not None
                else np.zeros(PED_MAP_SHAPE, dtype=np.float32)
            )
        else:
            pedestrian_map = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
        telemetry_sequence_id = None
        actuation_deadlock = False
        inference_started = time.perf_counter()
        preprocessing_ms = policy_ms = postprocessing_ms = 0.0
        try:
            observation = build_observation(
                pedestrian_map,
                history,
                causal_subgoal,
            )
            semantic_map = None
            if self.uses_semantics:
                semantic_history = padded_history(self.semantic_history)
                semantic_map = compress_semantic_history(
                    history,
                    semantic_history,
                )
                if (
                    semantic_map.shape != (80, 80)
                    or np.min(semantic_map) < IGNORE_LABEL
                    or np.max(semantic_map) >= SEMANTIC_NUM_CLASSES
                ):
                    raise ValueError(
                        "semantic map violates shape or categorical range"
                    )
            usable_scan_ranges = virtual_ranges[
                valid & np.isfinite(virtual_ranges) & (virtual_ranges > 0.0)
            ]
            training_state = DrlVoTrainingState()
            training_state.header.stamp = scan_01.header.stamp
            training_state.header.frame_id = str(
                self.get_parameter("base_frame").value
            )
            training_state.observation = observation.tolist()
            training_state.minimum_scan_range = float(
                np.min(usable_scan_ranges)
                if usable_scan_ranges.size
                else math.inf
            )
            training_state.goal_distance = float(
                np.linalg.norm(self.final_goal - self.pose[:2])
            )
            self.training_state_pub.publish(training_state)
            if self.actions_inhibited_after_reset:
                self.publish_stop("reset_inhibit")
                return
            if not bool(
                self.get_parameter("publish_policy_actions").value
            ):
                return
            # Tensor construction is preprocessing. Synchronize around the
            # policy call so CUDA work cannot leak into the next measurement.
            observation_tensor = (
                torch.from_numpy(observation).unsqueeze(0).to(self.device)
            )
            semantic_tensor = (
                torch.from_numpy(semantic_map).unsqueeze(0).to(self.device)
                if self.uses_semantics
                else None
            )
            if self.device.startswith("cuda"):
                torch.cuda.synchronize(self.device)
            policy_started = time.perf_counter()
            preprocessing_ms = (policy_started - inference_started) * 1000.0
            self.inference_sequence_id += 1
            telemetry_sequence_id = self.inference_sequence_id
            with torch.inference_mode():
                if self.uses_semantics:
                    action_tensor = self.policy.deterministic_action(
                        observation_tensor,
                        semantic_tensor,
                    )
                else:
                    action_tensor = self.policy.deterministic_action(
                        observation_tensor
                    )
                normalized_action = (
                    action_tensor.squeeze(0).detach().cpu().numpy()
                )
            if self.device.startswith("cuda"):
                torch.cuda.synchronize(self.device)
            postprocessing_started = time.perf_counter()
            policy_ms = (postprocessing_started - policy_started) * 1000.0
            raw_physical, command, front_stop = limit_physical_action(
                normalized_action,
                float(self.get_parameter("max_linear").value),
                float(self.get_parameter("max_angular").value),
                front_min,
                float(self.get_parameter("front_stop_distance").value),
                float(causal_subgoal[1]),
                float(
                    self.get_parameter(
                        "front_stop_angular_deadband"
                    ).value
                ),
                float(
                    self.get_parameter("front_stop_min_angular").value
                ),
            )
            finished = time.perf_counter()
            postprocessing_ms = (finished - postprocessing_started) * 1000.0
            self._publish_inference_metrics(
                telemetry_sequence_id,
                scan_01.header.stamp,
                preprocessing_ms,
                policy_ms,
                postprocessing_ms,
                (finished - inference_started) * 1000.0,
                True,
                raw_physical,
            )
            actuation_deadlock = self._observe_actuation_deadlock(
                scan_01_stamp_ns,
                command,
                causal_subgoal,
            )
        except (RuntimeError, ValueError) as exc:
            if telemetry_sequence_id is not None:
                finished = time.perf_counter()
                self._publish_inference_metrics(
                    telemetry_sequence_id,
                    scan_01.header.stamp,
                    preprocessing_ms,
                    policy_ms,
                    postprocessing_ms,
                    (finished - inference_started) * 1000.0,
                    False,
                )
            self.publish_stop("inference_error")
            self.get_logger().error(f"inference safety gate: {exc}")
            return

        if actuation_deadlock:
            return

        if (
            float(np.linalg.norm(self.final_goal - self.pose[:2]))
            <= float(self.get_parameter("goal_tolerance").value)
        ):
            self.publish_stop("goal_reached")
            return

        raw_message = Twist()
        raw_message.linear.x = float(raw_physical[0])
        raw_message.angular.z = float(raw_physical[1])
        command_message = Twist()
        command_message.linear.x = float(command[0])
        command_message.angular.z = float(command[1])
        self.raw_cmd_pub.publish(raw_message)
        gate_reasons = []
        if not math.isclose(float(raw_physical[0]), float(command[0]), abs_tol=1.0e-6):
            gate_reasons.append("linear_limit")
        if not math.isclose(float(raw_physical[1]), float(command[1]), abs_tol=1.0e-6):
            gate_reasons.append("angular_limit")
        if front_stop:
            gate_reasons.append("front_stop")
        self._publish_actuation_decision(
            raw_physical,
            command,
            front_min=float(front_min),
            reasons=tuple(dict.fromkeys(gate_reasons)),
            inference_sequence_id=int(telemetry_sequence_id or 0),
        )
        self.cmd_pub.publish(command_message)

    def watchdog_callback(self) -> None:
        now_ns = self._observe_clock()
        scan_fresh = time_is_fresh(
            now_ns,
            self.last_scan_clock_ns,
            float(self.get_parameter("scan_timeout").value),
        )
        inputs_ready, _reason = self._input_status(now_ns)
        if not scan_fresh or not inputs_ready:
            self._clear_history()
            self._clear_actuation_deadlock_state()
            self.publish_stop("watchdog_stale_input")

    def destroy_node(self):
        if (
            self.perception_metrics_stream is not None
            and not self.perception_metrics_stream.closed
        ):
            self.perception_metrics_stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DrlVoFixedDualInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
