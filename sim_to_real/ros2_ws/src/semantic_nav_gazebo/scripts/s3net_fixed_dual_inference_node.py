#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/s3net/label_colors, /s3net/labels, /s3net/semantic_markers, /scan_01, /scan_02
# 检测到的消息类型：ColorRGBA; Image; LaserScan; Marker, MarkerArray; Point
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：CONTRACT_RANGE_MAX, CONTRACT_RANGE_MIN, CUDA, DELETEALL, EXPECTED_BEAMS, FEATURE_MODE, IGNORE_LABEL, JSON, LABEL_COLORS, LABEL_NAMES, NAVIGATION_PROJECT_ROOT, NUM_CLASSES, POINTS, SAMPLING_STRATEGIES, SELF_HALF_EXTENTS_M, TEXT_VIEW_FACING
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo s3net_fixed_dual_inference_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:35:44.565100264 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.643741916 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_s3net_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_s3net_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/model.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_s3net_fixed_dual_helpers.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜s3net_fixed_dual_inference_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Perception-only fixed-dual S3-Net inference for the v7 Gazebo setup.

This node never publishes a velocity command.  It preserves the S3-Net training
contract by classifying each raw 2000-beam scan independently with the
``range_incidence`` feature pair and the run-specific normalization statistics.
Predictions are published both as a machine-readable signed label image and as
colored points for RViz.

The trained S3-Net samples its VAE latent even while ``model.eval()`` is active.
The default ``contract`` sampling strategy intentionally leaves that behavior
unchanged.  Optional seeded strategies make the random sequence easier to
repeat, but they do not replace sampling with a posterior-mean inference path.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import message_filters
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


EXPECTED_BEAMS = 2000
NUM_CLASSES = 7
IGNORE_LABEL = -1
CONTRACT_RANGE_MIN = 0.1
CONTRACT_RANGE_MAX = 8.0
SELF_HALF_EXTENTS_M = (0.36, 0.32)
FEATURE_MODE = "range_incidence"
SAMPLING_STRATEGIES = ("contract", "seeded_sequence", "frame_seeded")

LABEL_NAMES = (
    "_background_",
    "Chair",
    "Pillar",
    "Sofa",
    "Table",
    "Wall",
    "Person",
)

# Deliberately high-contrast colors for sparse LiDAR points in RViz.
LABEL_COLORS = np.asarray(
    (
        (145, 145, 145),
        (255, 179, 0),
        (128, 62, 214),
        (0, 158, 115),
        (230, 159, 0),
        (86, 180, 233),
        (213, 94, 0),
    ),
    dtype=np.uint8,
)


@dataclass(frozen=True)
class NormalizationStats:
    scan_mean: float
    scan_std: float
    incidence_mean: float
    incidence_std: float


@dataclass(frozen=True)
class SensorGeometry:
    raw_ranges: np.ndarray
    raw_angles: np.ndarray
    range_valid: np.ndarray
    footprint_self_mask: np.ndarray
    points_base: np.ndarray


def navigation_project_root() -> Path:
    return Path(
        os.environ.get(
            "NAVIGATION_PROJECT_ROOT",
            Path(__file__).resolve().parents[5],
        )
    )


def load_normalization_stats(path: str | Path) -> NormalizationStats:
    stats_path = Path(path)
    payload = json.loads(stats_path.read_text(encoding="utf-8"))
    normalization = payload.get("normalization", payload)
    try:
        scan = normalization["scan"]
        incidence = normalization["angle_incidence"]
        result = NormalizationStats(
            scan_mean=float(scan["mean"]),
            scan_std=float(scan["std"]),
            incidence_mean=float(incidence["mean"]),
            incidence_std=float(incidence["std"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"invalid S3-Net normalization statistics: {stats_path}"
        ) from exc
    values = (
        result.scan_mean,
        result.scan_std,
        result.incidence_mean,
        result.incidence_std,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("S3-Net normalization statistics must be finite")
    if result.scan_std <= 0.0 or result.incidence_std <= 0.0:
        raise ValueError("S3-Net normalization standard deviations must be positive")
    return result


def validate_checkpoint_contract(checkpoint: dict) -> None:
    feature_mode = checkpoint.get("feature_mode")
    input_channels = int(checkpoint.get("input_channels", -1))
    output_channels = int(checkpoint.get("num_output_channels", -1))
    if feature_mode != FEATURE_MODE:
        raise ValueError(
            f"checkpoint feature_mode must be {FEATURE_MODE!r}, got {feature_mode!r}"
        )
    if input_channels != 2:
        raise ValueError(
            f"checkpoint must have 2 input channels, got {input_channels}"
        )
    if output_channels != NUM_CLASSES:
        raise ValueError(
            f"checkpoint must have {NUM_CLASSES} output classes, got {output_channels}"
        )


def validate_sampling_strategy(strategy: str) -> str:
    if strategy not in SAMPLING_STRATEGIES:
        raise ValueError(
            f"sampling_strategy must be one of {SAMPLING_STRATEGIES}, "
            f"got {strategy!r}"
        )
    return strategy


def sampling_seed_for_frame(
    strategy: str,
    base_seed: int,
    frame_index: int,
) -> int | None:
    """Return a seed to apply immediately before this frame, if any."""

    validate_sampling_strategy(strategy)
    if strategy == "frame_seeded":
        return int(base_seed) + int(frame_index)
    return None


def raw_angles(angle_min: float, angle_increment: float, beam_count: int) -> np.ndarray:
    if beam_count <= 0:
        raise ValueError("beam_count must be positive")
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        raise ValueError("LaserScan angles must be finite")
    if angle_increment <= 0.0:
        raise ValueError("LaserScan angle_increment must be positive")
    return (
        float(angle_min)
        + np.arange(beam_count, dtype=np.float64) * float(angle_increment)
    ).astype(np.float32)


def range_valid_mask(
    ranges: np.ndarray,
    angles: np.ndarray,
    message_range_min: float,
    message_range_max: float,
) -> np.ndarray:
    ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(angles, dtype=np.float32).reshape(-1)
    if ranges.shape != angles.shape:
        raise ValueError("ranges and angles must have the same shape")
    low = max(
        CONTRACT_RANGE_MIN,
        float(message_range_min)
        if math.isfinite(float(message_range_min))
        else CONTRACT_RANGE_MIN,
    )
    high = min(
        CONTRACT_RANGE_MAX,
        float(message_range_max)
        if math.isfinite(float(message_range_max))
        else CONTRACT_RANGE_MAX,
    )
    if low >= high:
        raise ValueError(f"invalid effective LaserScan range [{low}, {high}]")
    return (
        np.isfinite(ranges)
        & np.isfinite(angles)
        & (ranges >= low)
        & (ranges <= high)
    )


def validate_scan_layout(
    beam_count: int,
    message_range_min: float,
    message_range_max: float,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    tolerance: float = 1e-3,
) -> None:
    if beam_count != EXPECTED_BEAMS:
        raise ValueError(
            f"expected {EXPECTED_BEAMS} raw beams, got {beam_count}"
        )
    expected = (
        (message_range_min, CONTRACT_RANGE_MIN, "range_min"),
        (message_range_max, CONTRACT_RANGE_MAX, "range_max"),
        (angle_min, -math.pi, "angle_min"),
        (angle_max, math.pi, "angle_max"),
    )
    for actual, wanted, name in expected:
        if not math.isfinite(float(actual)) or not math.isclose(
            float(actual),
            wanted,
            rel_tol=0.0,
            abs_tol=tolerance,
        ):
            raise ValueError(f"{name} must be {wanted:g}, got {actual!r}")
    expected_increment = (float(angle_max) - float(angle_min)) / (beam_count - 1)
    if not math.isclose(
        float(angle_increment),
        expected_increment,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise ValueError(
            "angle_increment does not match the fixed 2000-beam layout: "
            f"{angle_increment!r} vs {expected_increment!r}"
        )


def quaternion_matrix(quaternion) -> np.ndarray:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(float).eps:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.asarray(
        (
            (1.0 - yy - zz, xy - wz, xz + wy),
            (xy + wz, 1.0 - xx - zz, yz - wx),
            (xz - wy, yz + wx, 1.0 - xx - yy),
        ),
        dtype=np.float64,
    )


def project_sensor_geometry(
    ranges: np.ndarray,
    angles: np.ndarray,
    valid: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(angles, dtype=np.float32).reshape(-1)
    valid = np.asarray(valid, dtype=np.bool_).reshape(-1)
    if not (ranges.shape == angles.shape == valid.shape):
        raise ValueError("ranges, angles, and valid mask must have the same shape")
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64).reshape(-1)
    if rotation.shape != (3, 3) or translation.shape != (3,):
        raise ValueError("rotation/translation shape mismatch")

    points_base = np.full((len(ranges), 3), np.nan, dtype=np.float32)
    indices = np.flatnonzero(valid)
    sensor_points = np.column_stack(
        (
            ranges[indices] * np.cos(angles[indices]),
            ranges[indices] * np.sin(angles[indices]),
            np.zeros(len(indices), dtype=np.float32),
        )
    ).astype(np.float64)
    points_base[indices] = (
        sensor_points @ rotation.T + translation
    ).astype(np.float32)
    self_mask = np.zeros(len(ranges), dtype=np.bool_)
    self_mask[indices] = (
        np.abs(points_base[indices, 0]) <= SELF_HALF_EXTENTS_M[0]
    ) & (
        np.abs(points_base[indices, 1]) <= SELF_HALF_EXTENTS_M[1]
    )
    return points_base, self_mask


def normalize_s3_inputs(
    ranges: np.ndarray,
    angles: np.ndarray,
    valid: np.ndarray,
    stats: NormalizationStats,
    incidence_function: Callable[..., np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reproduce the range/incidence cleanup in the training dataset loader."""

    ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(angles, dtype=np.float32).reshape(-1)
    valid = np.asarray(valid, dtype=np.bool_).reshape(-1)
    if not (ranges.shape == angles.shape == valid.shape):
        raise ValueError("ranges, angles, and valid mask must have the same shape")

    incidence = np.asarray(
        incidence_function(ranges.copy(), angles),
        dtype=np.float32,
    ).reshape(-1)
    if incidence.shape != ranges.shape:
        raise ValueError("incidence function returned the wrong shape")
    clean_ranges = np.nan_to_num(
        ranges,
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    incidence = np.nan_to_num(
        incidence,
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    normalized_ranges = (clean_ranges - stats.scan_mean) / stats.scan_std
    normalized_incidence = (
        incidence - stats.incidence_mean
    ) / stats.incidence_std
    normalized_ranges[~valid] = 0.0
    normalized_incidence[~valid] = 0.0
    intensity = np.zeros_like(normalized_ranges, dtype=np.float32)
    if not (
        np.all(np.isfinite(normalized_ranges))
        and np.all(np.isfinite(normalized_incidence))
    ):
        raise ValueError("non-finite S3-Net input after cleanup")
    return (
        normalized_ranges.astype(np.float32),
        intensity,
        normalized_incidence.astype(np.float32),
    )


def stack_label_rows(
    labels_01: np.ndarray,
    labels_02: np.ndarray,
) -> np.ndarray:
    rows = np.stack(
        (
            np.asarray(labels_01, dtype=np.int16).reshape(-1),
            np.asarray(labels_02, dtype=np.int16).reshape(-1),
        )
    )
    if rows.shape != (2, EXPECTED_BEAMS):
        raise ValueError(
            f"label rows must have shape (2, {EXPECTED_BEAMS}), got {rows.shape}"
        )
    if np.any(rows < IGNORE_LABEL) or np.any(rows >= NUM_CLASSES):
        raise ValueError("labels must be in [-1, 6]")
    return np.ascontiguousarray(rows)


def labels_to_rgb(label_rows: np.ndarray) -> np.ndarray:
    labels = np.asarray(label_rows, dtype=np.int16)
    if labels.shape != (2, EXPECTED_BEAMS):
        raise ValueError(
            f"label_rows must have shape (2, {EXPECTED_BEAMS})"
        )
    rgb = np.zeros((2, EXPECTED_BEAMS, 3), dtype=np.uint8)
    valid = labels >= 0
    rgb[valid] = LABEL_COLORS[labels[valid]]
    return rgb


def image_message(array: np.ndarray, encoding: str, stamp, frame_id: str) -> Image:
    image = np.ascontiguousarray(array)
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = encoding
    message.is_bigendian = sys.byteorder == "big"
    message.step = int(image.strides[0])
    message.data = image.tobytes()
    return message


def load_s3net_model(
    model_code: Path,
    model_path: Path,
    device: torch.device,
):
    model_file = model_code / "model.py"
    spec = importlib.util.spec_from_file_location(
        "fixed_dual_s3net_model",
        model_file,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import S3-Net source: {model_file}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    checkpoint = torch.load(
        model_path,
        map_location=device,
        weights_only=False,
    )
    if not isinstance(checkpoint, dict):
        raise TypeError(f"S3-Net checkpoint must be a dictionary: {model_path}")
    validate_checkpoint_contract(checkpoint)
    model = module.S3Net(
        input_channels=2,
        output_channels=NUM_CLASSES,
        feature_mode=FEATURE_MODE,
    )
    state = checkpoint.get("model_state_dict", checkpoint.get("model"))
    if not isinstance(state, dict):
        raise TypeError("S3-Net checkpoint has no model state dictionary")
    state = {
        key.replace("module.", "", 1): value
        for key, value in state.items()
    }
    model.load_state_dict(state, strict=True)
    model.to(device)
    model.eval()
    return model, module, checkpoint


class FixedDualS3NetInference(Node):
    def __init__(self) -> None:
        super().__init__("s3net_fixed_dual_inference")
        self.declare_parameter("model", "")
        self.declare_parameter("model_code", "")
        self.declare_parameter("stats_json", "")
        self.declare_parameter("scan_01_topic", "/scan_01")
        self.declare_parameter("scan_02_topic", "/scan_02")
        self.declare_parameter("labels_topic", "/s3net/labels")
        self.declare_parameter("label_colors_topic", "/s3net/label_colors")
        self.declare_parameter("markers_topic", "/s3net/semantic_markers")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("device", "auto")
        self.declare_parameter("sampling_strategy", "contract")
        self.declare_parameter("sampling_seed", 1337)
        self.declare_parameter("visualization_rate_hz", 5.0)
        self.declare_parameter("point_size", 0.045)
        self.declare_parameter("enforce_message_layout", True)

        model_path = Path(str(self.get_parameter("model").value))
        model_code = Path(str(self.get_parameter("model_code").value))
        stats_path = Path(str(self.get_parameter("stats_json").value))
        if not model_path.is_file():
            raise FileNotFoundError(f"S3-Net model does not exist: {model_path}")
        if not (model_code / "model.py").is_file():
            raise FileNotFoundError(
                f"S3-Net model code does not exist: {model_code / 'model.py'}"
            )
        if not stats_path.is_file():
            raise FileNotFoundError(
                f"S3-Net stats JSON does not exist: {stats_path}"
            )

        requested_device = str(self.get_parameter("device").value)
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(requested_device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")

        self.stats = load_normalization_stats(stats_path)
        self.model, self.model_module, self.checkpoint = load_s3net_model(
            model_code,
            model_path,
            self.device,
        )
        self.sampling_strategy = validate_sampling_strategy(
            str(self.get_parameter("sampling_strategy").value)
        )
        self.sampling_seed = int(self.get_parameter("sampling_seed").value)
        if self.sampling_strategy == "seeded_sequence":
            torch.manual_seed(self.sampling_seed)

        self.visualization_rate_hz = float(
            self.get_parameter("visualization_rate_hz").value
        )
        if (
            not math.isfinite(self.visualization_rate_hz)
            or self.visualization_rate_hz <= 0.0
        ):
            raise ValueError("visualization_rate_hz must be positive and finite")
        self.point_size = float(self.get_parameter("point_size").value)
        if not math.isfinite(self.point_size) or self.point_size <= 0.0:
            raise ValueError("point_size must be positive and finite")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fixed_self_masks: tuple[np.ndarray, np.ndarray] | None = None
        self.frame_index = 0
        self.last_marker_publish_ns: int | None = None
        self.total_inference_seconds = 0.0

        self.labels_pub = self.create_publisher(
            Image,
            str(self.get_parameter("labels_topic").value),
            5,
        )
        self.colors_pub = self.create_publisher(
            Image,
            str(self.get_parameter("label_colors_topic").value),
            5,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("markers_topic").value),
            5,
        )

        scan_01 = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_01_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        scan_02 = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_02_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            (scan_01, scan_02),
            queue_size=10,
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.synchronizer.registerCallback(self.scan_callback)

        epoch = self.checkpoint.get("epoch", "unknown")
        self.get_logger().warning(
            "S3-Net eval() retains stochastic VAE sampling; "
            f"sampling_strategy={self.sampling_strategy!r}, seed={self.sampling_seed}"
        )
        self.get_logger().info(
            "loaded perception-only fixed-dual S3-Net "
            f"epoch={epoch}, device={self.device}, feature_mode={FEATURE_MODE}, "
            f"beams=2x{EXPECTED_BEAMS}, range=[{CONTRACT_RANGE_MIN}, "
            f"{CONTRACT_RANGE_MAX}] m; this node has no cmd_vel publisher"
        )

    def lookup_transform(self, frame_id: str):
        return self.tf_buffer.lookup_transform(
            str(self.get_parameter("base_frame").value),
            frame_id.strip(),
            Time(),
            timeout=Duration(
                seconds=float(self.get_parameter("tf_timeout").value)
            ),
        )

    def sensor_geometry(self, scan: LaserScan) -> SensorGeometry:
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        if bool(self.get_parameter("enforce_message_layout").value):
            validate_scan_layout(
                len(ranges),
                float(scan.range_min),
                float(scan.range_max),
                float(scan.angle_min),
                float(scan.angle_max),
                float(scan.angle_increment),
            )
        elif len(ranges) != EXPECTED_BEAMS:
            raise ValueError(
                f"S3-Net requires exactly {EXPECTED_BEAMS} beams, got {len(ranges)}"
            )
        angles = raw_angles(
            float(scan.angle_min),
            float(scan.angle_increment),
            len(ranges),
        )
        valid = range_valid_mask(
            ranges,
            angles,
            float(scan.range_min),
            float(scan.range_max),
        )
        transform = self.lookup_transform(scan.header.frame_id)
        rotation = quaternion_matrix(transform.transform.rotation)
        translation = np.asarray(
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            dtype=np.float64,
        )
        points, footprint_self_mask = project_sensor_geometry(
            ranges,
            angles,
            valid,
            rotation,
            translation,
        )
        return SensorGeometry(
            raw_ranges=ranges,
            raw_angles=angles,
            range_valid=valid,
            footprint_self_mask=footprint_self_mask,
            points_base=points,
        )

    def infer_pair(
        self,
        first: SensorGeometry,
        second: SensorGeometry,
    ) -> tuple[np.ndarray, np.ndarray, tuple[np.ndarray, np.ndarray]]:
        if self.fixed_self_masks is None:
            self.fixed_self_masks = (
                first.footprint_self_mask.copy(),
                second.footprint_self_mask.copy(),
            )
            self.get_logger().info(
                "calibrated fixed beam-identity self masks from first pair: "
                f"scan_01={int(self.fixed_self_masks[0].sum())}, "
                f"scan_02={int(self.fixed_self_masks[1].sum())}"
            )
        valid_pair = (
            first.range_valid & ~self.fixed_self_masks[0],
            second.range_valid & ~self.fixed_self_masks[1],
        )
        prepared = [
            normalize_s3_inputs(
                geometry.raw_ranges,
                geometry.raw_angles,
                valid,
                self.stats,
                self.model_module.angle_incidence_from_scan,
            )
            for geometry, valid in zip((first, second), valid_pair)
        ]
        ranges = torch.from_numpy(
            np.stack((prepared[0][0], prepared[1][0]))
        ).to(self.device)
        intensities = torch.from_numpy(
            np.stack((prepared[0][1], prepared[1][1]))
        ).to(self.device)
        incidences = torch.from_numpy(
            np.stack((prepared[0][2], prepared[1][2]))
        ).to(self.device)

        per_frame_seed = sampling_seed_for_frame(
            self.sampling_strategy,
            self.sampling_seed,
            self.frame_index,
        )
        if per_frame_seed is not None:
            torch.manual_seed(per_frame_seed)
        started = time.perf_counter()
        with torch.inference_mode():
            _probabilities, logits, _kl = self.model(
                ranges,
                intensities,
                incidences,
            )
        self.total_inference_seconds += time.perf_counter() - started
        if tuple(logits.shape) != (2, NUM_CLASSES, EXPECTED_BEAMS):
            raise RuntimeError(
                "unexpected S3-Net output shape "
                f"{tuple(logits.shape)}; expected "
                f"(2, {NUM_CLASSES}, {EXPECTED_BEAMS})"
            )
        predicted = logits.argmax(dim=1).cpu().numpy().astype(np.int16)
        predicted[0, ~valid_pair[0]] = IGNORE_LABEL
        predicted[1, ~valid_pair[1]] = IGNORE_LABEL
        return predicted[0], predicted[1], valid_pair

    @staticmethod
    def initialize_marker(
        marker: Marker,
        stamp,
        frame_id: str,
        namespace: str,
        marker_id: int,
        marker_type: int,
    ) -> None:
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True

    def publish_markers(
        self,
        stamp,
        first: SensorGeometry,
        second: SensorGeometry,
        label_rows: np.ndarray,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(1e9 / self.visualization_rate_hz)
        if (
            self.last_marker_publish_ns is not None
            and now_ns >= self.last_marker_publish_ns
            and now_ns - self.last_marker_publish_ns < period_ns
        ):
            return
        self.last_marker_publish_ns = now_ns
        frame_id = str(self.get_parameter("base_frame").value)
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        all_points = np.concatenate((first.points_base, second.points_base))
        all_labels = label_rows.reshape(-1)
        counts = []
        for label, (name, color) in enumerate(zip(LABEL_NAMES, LABEL_COLORS)):
            selected = np.flatnonzero(all_labels == label)
            counts.append(len(selected))
            marker = Marker()
            self.initialize_marker(
                marker,
                stamp,
                frame_id,
                "s3net_classes",
                label,
                Marker.POINTS,
            )
            marker.scale.x = self.point_size
            marker.scale.y = self.point_size
            marker.color = ColorRGBA(
                r=float(color[0]) / 255.0,
                g=float(color[1]) / 255.0,
                b=float(color[2]) / 255.0,
                a=0.95,
            )
            marker.points = [
                Point(
                    x=float(all_points[index, 0]),
                    y=float(all_points[index, 1]),
                    z=float(all_points[index, 2]),
                )
                for index in selected
            ]
            marker_array.markers.append(marker)

        status = Marker()
        self.initialize_marker(
            status,
            stamp,
            frame_id,
            "s3net_status",
            NUM_CLASSES + 1,
            Marker.TEXT_VIEW_FACING,
        )
        status.pose.position.x = 0.0
        status.pose.position.y = 0.0
        status.pose.position.z = 1.25
        status.scale.z = 0.18
        status.color.r = 1.0
        status.color.g = 1.0
        status.color.b = 1.0
        status.color.a = 1.0
        mean_ms = (
            1000.0 * self.total_inference_seconds / max(1, self.frame_index + 1)
        )
        status.text = (
            f"S3-Net epoch={self.checkpoint.get('epoch', '?')} "
            f"sampling={self.sampling_strategy} frame={self.frame_index} "
            f"mean={mean_ms:.1f}ms\n"
            + " | ".join(
                f"{name}:{count}"
                for name, count in zip(LABEL_NAMES[1:], counts[1:])
            )
        )
        marker_array.markers.append(status)
        self.markers_pub.publish(marker_array)

    def scan_callback(self, scan_01: LaserScan, scan_02: LaserScan) -> None:
        try:
            first = self.sensor_geometry(scan_01)
            second = self.sensor_geometry(scan_02)
            labels_01, labels_02, _valid_pair = self.infer_pair(first, second)
            label_rows = stack_label_rows(labels_01, labels_02)
        except (TransformException, RuntimeError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"S3-Net frame rejected: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        frame_id = str(self.get_parameter("base_frame").value)
        stamp = scan_01.header.stamp
        self.labels_pub.publish(
            image_message(label_rows, "16SC1", stamp, frame_id)
        )
        self.colors_pub.publish(
            image_message(labels_to_rgb(label_rows), "rgb8", stamp, frame_id)
        )
        self.publish_markers(stamp, first, second, label_rows)
        self.frame_index += 1


def main() -> None:
    rclpy.init()
    node = FixedDualS3NetInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
