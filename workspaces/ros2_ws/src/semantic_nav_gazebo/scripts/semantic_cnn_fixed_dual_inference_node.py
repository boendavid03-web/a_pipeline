#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_01, /scan_02, /semantic_cnn/actuation_decision, /semantic_cnn/debug/cmd_vel, /semantic_cnn/debug/markers, /semantic_cnn/debug/raw_cmd, /semantic_cnn/debug/scan_map, /semantic_cnn/debug/semantic_map, /semantic_cnn/final_goal, /semantic_cnn/local_subgoal, /semantic_cnn/raw_model_cmd
# 检测到的消息类型：ActuationDecision; ColorRGBA; Image as ImageMsg; LaserScan; Marker, MarkerArray; Odometry; Point, PointStamped, Twist
# 检测到的文件格式：PNG, YAML
# 可能使用的关键环境变量：CUBE_LIST, GOAL_MU, GOAL_STD, IGNORE_LABEL, IMG_SIZE, NANOSECONDS_PER_SECOND, NAVIGATION_PROJECT_ROOT, POINTS, POOL_ANGLE_MAX, POOL_ANGLE_MIN, POOL_MODES, SCAN_MU, SCAN_STD, SEQ_LEN, SPHERE, TEXT_VIEW_FACING
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo semantic_cnn_fixed_dual_inference_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-18 00:21:44.705422570 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.644741936 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_semantic_cnn_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_semantic_cnn_fixed_dual_helpers.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/model.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_semantic_cnn_fixed_dual_helpers.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_fixed_dual_inference_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Closed-loop fixed-dual SemanticCNN controller for the v7 Gazebo demo.

The node intentionally keeps the fixed-dual training contract: two synchronized
raw scans are transformed into base_link virtual range/angle points and pooled
over the front 180 degrees.  It is separate from the legacy /scan_merged demo.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import message_filters
import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import ActuationDecision, InferenceMetrics
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


IMG_SIZE = 80
SEQ_LEN = 10
IGNORE_LABEL = -1
POOL_MODES = ("global_virtual_angle_80", "sensor_split_40x2")
POOL_ANGLE_MIN = -math.pi / 2.0
POOL_ANGLE_MAX = math.pi / 2.0
SCAN_MU = 4.518406
SCAN_STD = 8.2914915
GOAL_MU = 0.30655652
GOAL_STD = 0.5378557
NANOSECONDS_PER_SECOND = 1_000_000_000


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


def checkpoint_goal_normalization(checkpoint):
    """Return the exact sub-goal normalization recorded by training.

    Historical checkpoints did not carry normalization metadata and used the
    same scalar constants for x/y.  Preserve that fallback while requiring
    modern checkpoint metadata to be finite, two-dimensional, and usable.
    """
    normalization = checkpoint.get("normalization")
    if normalization is None:
        return (
            np.full(2, GOAL_MU, dtype=np.float32),
            np.full(2, GOAL_STD, dtype=np.float32),
            "legacy_constants",
        )
    if not isinstance(normalization, dict):
        raise ValueError("checkpoint normalization must be a mapping")
    mean = np.asarray(normalization.get("sub_goal_mean"), dtype=np.float32).reshape(-1)
    std = np.asarray(normalization.get("sub_goal_std"), dtype=np.float32).reshape(-1)
    if mean.shape != (2,) or std.shape != (2,):
        raise ValueError("checkpoint sub-goal normalization must contain two values")
    if not np.isfinite(mean).all() or not np.isfinite(std).all():
        raise ValueError("checkpoint sub-goal normalization must be finite")
    if np.any(std <= 0.0):
        raise ValueError("checkpoint sub-goal normalization std must be positive")
    return mean, std, str(normalization.get("source") or "checkpoint")


def clock_rolled_back(previous_ns: int | None, current_ns: int) -> bool:
    return previous_ns is not None and int(current_ns) < int(previous_ns)


def navigation_project_root() -> Path:
    return Path(os.environ.get("NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5]))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate_point(point, quaternion):
    """Apply a quaternion rotation; matches v7_dual_laser_scan_merger.py."""
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


def load_map_info(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    origin = metadata["origin"]
    return float(metadata["resolution"]), float(origin[0]), float(origin[1])


def load_model(model_code: Path, model_path: Path, device: str):
    model_file = model_code / "model.py"
    spec = importlib.util.spec_from_file_location("fixed_dual_semantic_cnn_model", model_file)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SemanticCNN source: {model_file}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    model = module.SemanticCNN(module.Bottleneck, [2, 1, 1])
    checkpoint = torch.load(model_path, map_location=device)
    model.load_state_dict(checkpoint["model"])
    goal_mean, goal_std, normalization_source = checkpoint_goal_normalization(
        checkpoint
    )
    model.to(device)
    model.eval()
    return model, module, goal_mean, goal_std, normalization_source


def majority_label(labels: np.ndarray) -> int:
    labels = np.asarray(labels, dtype=np.int64).reshape(-1)
    labels = labels[labels >= 0]
    return 0 if labels.size == 0 else int(np.bincount(labels, minlength=256).argmax())


def float_image(array: np.ndarray, stamp, frame_id: str) -> ImageMsg:
    """Preserve the exact float32 CNN input in a sensor_msgs/Image."""
    image = np.ascontiguousarray(array, dtype=np.float32)
    msg = ImageMsg()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape
    msg.encoding = "32FC1"
    msg.is_bigendian = sys.byteorder == "big"
    msg.step = msg.width * image.dtype.itemsize
    msg.data = image.tobytes()
    return msg


class FixedDualSemanticCnnInference(Node):
    def __init__(self) -> None:
        super().__init__("semantic_cnn_fixed_dual_inference")
        project_root = navigation_project_root()
        self.declare_parameter("map_yaml", str(project_root / "maps" / "map.yaml"))
        self.declare_parameter("semantic_label", str(project_root / "maps" / "label.png"))
        self.declare_parameter("model", "")
        self.declare_parameter("model_code", "")
        self.declare_parameter("scan_01_topic", "/scan_01")
        self.declare_parameter("scan_02_topic", "/scan_02")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("local_subgoal_topic", "/semantic_cnn/local_subgoal")
        self.declare_parameter("final_goal_topic", "/semantic_cnn/final_goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "actuation_decision_topic", "/semantic_cnn/actuation_decision"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 50.0)
        self.declare_parameter("pool_range_max", 50.0)
        self.declare_parameter("pool_mode", "global_virtual_angle_80")
        self.declare_parameter("enable_self_filter", True)
        self.declare_parameter("self_filter_min_x", -0.36)
        self.declare_parameter("self_filter_max_x", 0.36)
        self.declare_parameter("self_filter_min_y", -0.32)
        self.declare_parameter("self_filter_max_y", 0.32)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("front_stop_distance", 1.0)
        self.declare_parameter("stop_on_empty_front", True)
        self.declare_parameter("front_stop_angular_deadband", 0.05)
        self.declare_parameter("front_stop_min_angular", 0.35)
        self.declare_parameter("max_linear", 0.3)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("subgoal_timeout", 0.3)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("odom_timeout", 0.3)
        self.declare_parameter("odom_jump_reset_distance", 1.0)
        self.declare_parameter("odom_jump_reset_yaw", 1.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("device", "cuda" if torch.cuda.is_available() else "cpu")
        self.declare_parameter("visualize", True)
        self.declare_parameter("publish_debug_images", True)
        self.declare_parameter("debug_rate_hz", 5.0)
        self.declare_parameter(
            "inference_metrics_topic",
            "/navigation_evaluation/inference_metrics",
        )

        self.device = str(self.get_parameter("device").value)
        model_path = Path(str(self.get_parameter("model").value))
        model_code = Path(str(self.get_parameter("model_code").value))
        if not model_path.is_file() or not (model_code / "model.py").is_file():
            raise FileNotFoundError("model and model_code/model.py must both exist")
        (
            self.model,
            self.model_module,
            self.goal_mean,
            self.goal_std,
            self.normalization_source,
        ) = load_model(model_code, model_path, self.device)
        self.model_parameters = sum(
            parameter.numel() for parameter in self.model.parameters()
        )
        self.inference_sequence_id = 0
        self.pool_range_max = float(self.get_parameter("pool_range_max").value)
        if self.pool_range_max <= 0.0:
            raise ValueError("pool_range_max must be positive")
        self.pool_mode = str(self.get_parameter("pool_mode").value)
        if self.pool_mode not in POOL_MODES:
            raise ValueError(f"pool_mode must be one of {POOL_MODES}")
        for parameter_name in (
            "subgoal_timeout",
            "scan_timeout",
            "odom_timeout",
            "odom_jump_reset_distance",
            "odom_jump_reset_yaw",
            "command_timeout",
        ):
            value = float(self.get_parameter(parameter_name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{parameter_name} must be a positive finite number"
                )

        self.resolution, self.origin_x, self.origin_y = load_map_info(
            Path(str(self.get_parameter("map_yaml").value))
        )
        label_img = np.asarray(Image.open(str(self.get_parameter("semantic_label").value)))
        self.label_img = label_img[:, :, 0] if label_img.ndim == 3 else label_img
        self.label_img = self.label_img.astype(np.int64)
        self.pose = None
        self.pose_stamp_ns = None
        self.subgoal = None
        self.subgoal_stamp_ns = None
        self.subgoal_history = deque(maxlen=100)
        self.final_goal = None
        self.last_clock_ns = None
        self.scan_history = deque(maxlen=SEQ_LEN)
        self.angle_history = deque(maxlen=SEQ_LEN)
        self.semantic_history = deque(maxlen=SEQ_LEN)
        self.valid_history = deque(maxlen=SEQ_LEN)
        self.last_scan_time = None
        self.scan_count = 0
        self.last_debug_publish_ns = None
        self.last_grid_marker_ns = None
        self.debug_ranges = None
        self.debug_angles = None
        self.debug_valid = None
        self.debug_scan_map = None
        self.debug_semantic_map = None
        self.last_raw_linear = 0.0
        self.last_raw_angular = 0.0
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.front_min = float("inf")
        self.front_stop = False
        self.goal_stop = False
        self.scan_timeout = False
        self.visualize = bool(self.get_parameter("visualize").value)
        self.publish_debug_images = bool(self.get_parameter("publish_debug_images").value)
        self.debug_rate_hz = float(self.get_parameter("debug_rate_hz").value)
        if self.debug_rate_hz <= 0.0:
            raise ValueError("debug_rate_hz must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.raw_pub = self.create_publisher(Twist, "/semantic_cnn/raw_model_cmd", 10)
        self.actuation_decision_pub = self.create_publisher(
            ActuationDecision,
            str(self.get_parameter("actuation_decision_topic").value),
            30,
        )
        self.actuation_decision_sequence_id = 0
        self.debug_raw_pub = self.create_publisher(Twist, "/semantic_cnn/debug/raw_cmd", 10)
        self.debug_cmd_pub = self.create_publisher(Twist, "/semantic_cnn/debug/cmd_vel", 10)
        self.inference_metrics_pub = self.create_publisher(
            InferenceMetrics,
            str(self.get_parameter("inference_metrics_topic").value),
            10,
        )
        self.debug_marker_pub = (
            self.create_publisher(MarkerArray, "/semantic_cnn/debug/markers", 10)
            if self.visualize else None
        )
        self.scan_map_pub = (
            self.create_publisher(ImageMsg, "/semantic_cnn/debug/scan_map", 2)
            if self.publish_debug_images else None
        )
        self.semantic_map_pub = (
            self.create_publisher(ImageMsg, "/semantic_cnn/debug/semantic_map", 2)
            if self.publish_debug_images else None
        )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
        self.create_subscription(PointStamped, str(self.get_parameter("local_subgoal_topic").value), self.subgoal_callback, 10)
        self.create_subscription(PointStamped, str(self.get_parameter("final_goal_topic").value), self.final_goal_callback, 10)
        scan_01_sub = message_filters.Subscriber(
            self, LaserScan, str(self.get_parameter("scan_01_topic").value), qos_profile=qos_profile_sensor_data
        )
        scan_02_sub = message_filters.Subscriber(
            self, LaserScan, str(self.get_parameter("scan_02_topic").value), qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [scan_01_sub, scan_02_sub], queue_size=10, slop=float(self.get_parameter("sync_slop").value)
        )
        self.sync.registerCallback(self.scan_callback)
        self.create_timer(0.1, self.command_watchdog)
        self.get_logger().info(
            "fixed-dual SemanticCNN loaded on "
            f"{self.device}: {scan_01_sub.topic} + {scan_02_sub.topic}; "
            f"sub-goal normalization={self.normalization_source} "
            f"mean={self.goal_mean.tolist()} std={self.goal_std.tolist()}"
        )

    def _emit_inference_metrics(
        self,
        sequence_id: int,
        input_stamp,
        preprocessing_ms: float,
        policy_ms: float,
        postprocessing_ms: float,
        total_ms: float,
        action: np.ndarray,
    ) -> None:
        metrics = InferenceMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.input_stamp = input_stamp
        metrics.sequence_id = int(sequence_id)
        metrics.producer_id = "semantic_cnn_policy"
        metrics.success = True
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
        metrics.action_encoding = "twist_linear_x_mps_angular_z_radps"
        metrics.action = [float(action[0]), float(action[1])]
        self.inference_metrics_pub.publish(metrics)

    def publish_inference_metrics(self, *args, **kwargs) -> None:
        """Keep passive evaluation telemetry out of the control failure path."""
        try:
            self._emit_inference_metrics(*args, **kwargs)
        except Exception as exc:
            self.get_logger().warning(
                f"inference metrics publication failed: {exc}",
                throttle_duration_sec=2.0,
            )

    def odom_callback(self, msg: Odometry) -> None:
        self.observe_clock()
        p = msg.pose.pose.position
        pose = np.asarray(
            [p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)],
            dtype=np.float32,
        )
        if not np.isfinite(pose).all():
            self.pose = None
            self.pose_stamp_ns = None
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
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
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
        self.pose = pose
        self.pose_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)

    def clear_temporal_history(self) -> None:
        self.scan_history.clear()
        self.angle_history.clear()
        self.semantic_history.clear()
        self.valid_history.clear()

    def clear_subgoal_state(self) -> None:
        self.subgoal = None
        self.subgoal_stamp_ns = None
        self.subgoal_history.clear()

    def reset_runtime_inputs(self) -> None:
        self.pose = None
        self.pose_stamp_ns = None
        self.clear_subgoal_state()
        self.final_goal = None
        self.clear_temporal_history()
        self.last_scan_time = None
        self.scan_timeout = True

    def observe_clock(self) -> int:
        now_ns = int(self.get_clock().now().nanoseconds)
        if clock_rolled_back(self.last_clock_ns, now_ns):
            self.get_logger().warning(
                "simulation clock rolled back; clearing all temporal inputs"
            )
            self.reset_runtime_inputs()
            self.publish_stop()
        self.last_clock_ns = now_ns
        return now_ns

    def subgoal_callback(self, msg: PointStamped) -> None:
        self.observe_clock()
        expected_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        actual_frame = msg.header.frame_id.lstrip("/")
        if actual_frame and actual_frame != expected_frame:
            self.get_logger().error(
                f"rejecting subgoal frame {actual_frame!r}; "
                f"expected {expected_frame!r}"
            )
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
            return
        subgoal = np.asarray([msg.point.x, msg.point.y], dtype=np.float32)
        if not np.isfinite(subgoal).all():
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
            return
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.subgoal_history.append((stamp_ns, subgoal.copy()))
        if self.subgoal_stamp_ns is None or stamp_ns >= self.subgoal_stamp_ns:
            self.subgoal = subgoal
            self.subgoal_stamp_ns = stamp_ns

    def final_goal_callback(self, msg: PointStamped) -> None:
        self.observe_clock()
        final_goal = np.asarray([msg.point.x, msg.point.y], dtype=np.float32)
        if not np.isfinite(final_goal).all():
            self.final_goal = None
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
            return
        if (
            self.final_goal is not None
            and float(np.linalg.norm(final_goal - self.final_goal)) > 1e-4
        ):
            self.clear_subgoal_state()
            self.clear_temporal_history()
            self.publish_stop()
        self.final_goal = final_goal

    def publish_actuation_decision(
        self,
        raw_action: np.ndarray | None,
        command: np.ndarray | None,
        *,
        input_stamp=None,
        reasons: tuple[str, ...] = (),
        front_min: float | None = None,
    ) -> None:
        """Publish model/final command telemetry for model-agnostic evaluation."""
        message = ActuationDecision()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("base_frame").value)
        message.input_stamp = (
            input_stamp if input_stamp is not None else message.header.stamp
        )
        self.actuation_decision_sequence_id += 1
        message.decision_sequence_id = self.actuation_decision_sequence_id
        message.inference_sequence_id = self.inference_sequence_id
        message.has_raw_action = raw_action is not None
        if raw_action is not None:
            raw = np.asarray(raw_action, dtype=np.float64).reshape(-1)
            if raw.shape != (2,) or not np.isfinite(raw).all():
                raise ValueError("raw SemanticCNN action must contain two finite values")
            message.raw_physical_action.linear.x = float(raw[0])
            message.raw_physical_action.angular.z = float(raw[1])
        final = (
            np.zeros(2, dtype=np.float64)
            if command is None
            else np.asarray(command, dtype=np.float64).reshape(-1)
        )
        if final.shape != (2,) or not np.isfinite(final).all():
            raise ValueError("final SemanticCNN command must contain two finite values")
        message.final_command.linear.x = float(final[0])
        message.final_command.angular.z = float(final[1])
        message.gated = bool(reasons)
        message.gate_reasons = list(reasons)
        message.has_front_min_range = front_min is not None and math.isfinite(front_min)
        message.front_min_range_m = float(front_min) if message.has_front_min_range else 0.0
        self.actuation_decision_pub.publish(message)

    def publish_stop(self, reason: str = "stop", input_stamp=None) -> None:
        cmd = Twist()
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        try:
            self.publish_actuation_decision(
                None, None, input_stamp=input_stamp, reasons=(reason,)
            )
        except Exception as exc:
            self.get_logger().warning(
                f"SemanticCNN actuation telemetry failed: {exc}",
                throttle_duration_sec=2.0,
            )
        self.cmd_pub.publish(cmd)
        self.debug_cmd_pub.publish(cmd)

    def command_watchdog(self) -> None:
        self.observe_clock()
        if self.last_scan_time is not None:
            elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
            self.scan_timeout = elapsed > float(self.get_parameter("command_timeout").value)
        if self.scan_timeout:
            self.publish_stop()
        self.publish_debug_markers()

    @staticmethod
    def initialize_marker(marker: Marker, stamp, frame_id: str, namespace: str, marker_id: int, marker_type: int) -> None:
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

    def cnn_grid_marker(self, stamp, array: np.ndarray, marker_id: int, y_offset: float, semantic: bool) -> Marker:
        marker = Marker()
        self.initialize_marker(marker, stamp, "map", "cnn_maps", marker_id, Marker.CUBE_LIST)
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.02
        for row in range(IMG_SIZE):
            for col in range(IMG_SIZE):
                point = Point()
                point.x = -2.0 + (col - (IMG_SIZE - 1) / 2.0) * marker.scale.x
                point.y = y_offset + ((IMG_SIZE - 1) / 2.0 - row) * marker.scale.y
                point.z = 0.18
                marker.points.append(point)
                value = float(array[row, col])
                color = ColorRGBA()
                if semantic:
                    label = max(0, int(round(value)))
                    color.r = ((label * 73) % 255) / 255.0
                    color.g = ((label * 151) % 255) / 255.0
                    color.b = ((label * 199) % 255) / 255.0
                else:
                    color.r = 1.0 - float(np.clip(value, 0.0, 1.0))
                    color.g = float(np.clip(value, 0.0, 1.0))
                    color.b = 0.15
                color.a = 0.95
                marker.colors.append(color)
        return marker

    def cnn_grid_text(self, stamp, marker_id: int, y_offset: float, text: str) -> Marker:
        marker = Marker()
        self.initialize_marker(marker, stamp, "map", "cnn_maps", marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = -2.0
        marker.pose.position.y = y_offset
        marker.pose.position.z = 0.45
        marker.scale.z = 0.28
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = text
        return marker

    def publish_debug_markers(self) -> None:
        if self.debug_marker_pub is None:
            return
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        period_ns = int(1e9 / self.debug_rate_hz)
        if (
            self.last_debug_publish_ns is not None
            and 0 <= now_ns - self.last_debug_publish_ns < period_ns
        ):
            return
        self.last_debug_publish_ns = now_ns
        stamp = now.to_msg()
        markers = MarkerArray()

        points = Marker()
        self.initialize_marker(points, stamp, str(self.get_parameter("base_frame").value), "valid_points", 0, Marker.POINTS)
        points.frame_locked = True
        points.scale.x = 0.045
        points.scale.y = 0.045
        points.color.r = 0.1
        points.color.g = 1.0
        points.color.b = 0.55
        points.color.a = 0.8
        if self.debug_ranges is not None:
            for value, angle in zip(
                self.debug_ranges[self.debug_valid], self.debug_angles[self.debug_valid]
            ):
                point = Point()
                point.x = float(value * math.cos(float(angle)))
                point.y = float(value * math.sin(float(angle)))
                point.z = 0.08
                points.points.append(point)
        markers.markers.append(points)

        if self.final_goal is not None:
            goal = Marker()
            self.initialize_marker(goal, stamp, "map", "navigation", 1, Marker.SPHERE)
            goal.pose.position.x = float(self.final_goal[0])
            goal.pose.position.y = float(self.final_goal[1])
            goal.pose.position.z = 0.22
            goal.scale.x = goal.scale.y = goal.scale.z = 0.45
            goal.color.r = 1.0
            goal.color.a = 1.0
            markers.markers.append(goal)

        if self.subgoal is not None:
            subgoal = Marker()
            self.initialize_marker(subgoal, stamp, str(self.get_parameter("base_frame").value), "navigation", 2, Marker.SPHERE)
            subgoal.frame_locked = True
            subgoal.pose.position.x = float(self.subgoal[0])
            subgoal.pose.position.y = float(self.subgoal[1])
            subgoal.pose.position.z = 0.22
            subgoal.scale.x = subgoal.scale.y = subgoal.scale.z = 0.34
            subgoal.color.r = 1.0
            subgoal.color.g = 0.85
            subgoal.color.a = 1.0
            markers.markers.append(subgoal)

            subgoal_text = Marker()
            self.initialize_marker(subgoal_text, stamp, str(self.get_parameter("base_frame").value), "navigation", 3, Marker.TEXT_VIEW_FACING)
            subgoal_text.frame_locked = True
            subgoal_text.pose.position.x = float(self.subgoal[0])
            subgoal_text.pose.position.y = float(self.subgoal[1])
            subgoal_text.pose.position.z = 0.65
            subgoal_text.scale.z = 0.28
            subgoal_text.color.r = 1.0
            subgoal_text.color.g = 0.95
            subgoal_text.color.b = 0.2
            subgoal_text.color.a = 1.0
            distance_text = "n/a"
            if self.pose is not None and self.final_goal is not None:
                yaw = float(self.pose[2])
                subgoal_world = self.pose[:2] + np.asarray(
                    [
                        math.cos(yaw) * self.subgoal[0] - math.sin(yaw) * self.subgoal[1],
                        math.sin(yaw) * self.subgoal[0] + math.cos(yaw) * self.subgoal[1],
                    ],
                    dtype=np.float32,
                )
                distance_text = f"{np.linalg.norm(self.final_goal - subgoal_world):.2f}m"
            subgoal_text.text = (
                f"subgoal local=({self.subgoal[0]:.2f}, {self.subgoal[1]:.2f}) "
                f"subgoal-to-goal={distance_text}"
            )
            markers.markers.append(subgoal_text)

        status = Marker()
        self.initialize_marker(status, stamp, "map", "inference", 4, Marker.TEXT_VIEW_FACING)
        status.pose.position.x = -2.0
        status.pose.position.y = 12.0
        status.pose.position.z = 0.45
        status.scale.z = 0.3
        status.color.r = 1.0 if (self.front_stop or self.goal_stop or self.scan_timeout) else 0.85
        status.color.g = 0.25 if (self.front_stop or self.scan_timeout) else 1.0
        status.color.b = 0.2 if self.goal_stop else 1.0
        status.color.a = 1.0
        front_text = "inf" if not math.isfinite(self.front_min) else f"{self.front_min:.2f}m"
        status.text = (
            f"fixed-dual CNN | valid_points={len(points.points)} | "
            f"sequence={len(self.scan_history)}/{SEQ_LEN} ready={len(self.scan_history) == SEQ_LEN} frame={self.scan_count}\n"
            f"raw=({self.last_raw_linear:+.3f}, {self.last_raw_angular:+.3f}) "
            f"cmd=({self.last_cmd_linear:+.3f}, {self.last_cmd_angular:+.3f})\n"
            f"front_min={front_text} front_stop={self.front_stop} "
            f"goal_stop={self.goal_stop} scan_timeout={self.scan_timeout}"
        )
        markers.markers.append(status)
        if (
            self.debug_scan_map is not None
            and self.debug_semantic_map is not None
            and (
                self.last_grid_marker_ns is None
                or now_ns < self.last_grid_marker_ns
                or now_ns - self.last_grid_marker_ns >= int(1e9)
            )
        ):
            self.last_grid_marker_ns = now_ns
            markers.markers.extend(
                (
                    self.cnn_grid_marker(stamp, self.debug_scan_map, 5, 8.3, semantic=False),
                    self.cnn_grid_marker(stamp, self.debug_semantic_map, 6, 4.1, semantic=True),
                    self.cnn_grid_text(stamp, 7, 10.1, "CNN scan_map 80x80"),
                    self.cnn_grid_text(stamp, 8, 5.9, "CNN semantic_map 80x80"),
                )
            )
        self.debug_marker_pub.publish(markers)

    def virtualize_scan(self, scan: LaserScan):
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("base_frame").value),
                scan.header.frame_id.strip(),
                # The two lidar extrinsics are static.  Gazebo scan stamps begin
                # before the static-TF publisher's simulated clock settles, so
                # requesting the latest transform avoids rejecting valid scans
                # as TF_OLD_DATA without changing the spatial projection.
                Time(),
                timeout=Duration(seconds=float(self.get_parameter("tf_timeout").value)),
            )
        except TransformException as exc:
            raise RuntimeError(f"TF unavailable for {scan.header.frame_id}: {exc}") from exc
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        raw = np.asarray(scan.ranges, dtype=np.float32)
        raw_angles = float(scan.angle_min) + np.arange(raw.size, dtype=np.float32) * float(scan.angle_increment)
        low = max(float(scan.range_min), float(self.get_parameter("range_min").value))
        high = min(float(scan.range_max), float(self.get_parameter("range_max").value))
        valid = np.isfinite(raw) & (raw >= low) & (raw <= high)
        ranges = np.zeros(raw.size, dtype=np.float32)
        angles = np.zeros(raw.size, dtype=np.float32)
        for index in np.flatnonzero(valid):
            value = float(raw[index])
            rx, ry, _ = rotate_point((value * math.cos(float(raw_angles[index])), value * math.sin(float(raw_angles[index])), 0.0), quaternion)
            x, y = translation.x + rx, translation.y + ry
            if bool(self.get_parameter("enable_self_filter").value) and (
                float(self.get_parameter("self_filter_min_x").value) <= x <= float(self.get_parameter("self_filter_max_x").value)
                and float(self.get_parameter("self_filter_min_y").value) <= y <= float(self.get_parameter("self_filter_max_y").value)
            ):
                valid[index] = False
                continue
            virtual_range = math.hypot(x, y)
            if virtual_range < low or virtual_range > high:
                valid[index] = False
                continue
            ranges[index] = virtual_range
            angles[index] = math.atan2(y, x)
        return ranges, angles, valid

    def semantic_for_points(self, ranges, angles, valid_mask):
        height, width = self.label_img.shape[:2]
        x, y, yaw = self.pose
        world_x = x + ranges * np.cos(yaw + angles)
        world_y = y + ranges * np.sin(yaw + angles)
        cols = np.floor((world_x - self.origin_x) / self.resolution).astype(np.int64)
        rows = height - 1 - np.floor((world_y - self.origin_y) / self.resolution).astype(np.int64)
        in_map = valid_mask & (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        labels = np.full(ranges.shape, IGNORE_LABEL, dtype=np.int64)
        labels[in_map] = self.label_img[rows[in_map], cols[in_map]]
        return labels

    def make_cnn_maps(self):
        scan_rows = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
        semantic_rows = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
        sensor_01_count = len(self.scan_history[0]) // 2
        source_sensor = np.concatenate(
            (
                np.zeros(sensor_01_count, dtype=np.int8),
                np.ones(len(self.scan_history[0]) - sensor_01_count, dtype=np.int8),
            )
        )
        for index, (ranges, angles, semantic, valid_mask) in enumerate(
            zip(self.scan_history, self.angle_history, self.semantic_history, self.valid_history)
        ):
            mins, means, sem_nearest, sem_majority, _ = self.model_module._native_lidar_maps(
                ranges, angles, semantic, valid_mask, source_sensor,
                pool_mode=self.pool_mode, range_max=self.pool_range_max,
            )
            scan_rows[2 * index] = mins
            scan_rows[2 * index + 1] = means
            semantic_rows[2 * index] = sem_nearest
            semantic_rows[2 * index + 1] = sem_majority
        row_repeat = IMG_SIZE // (SEQ_LEN * 2)
        return np.repeat(scan_rows, row_repeat, axis=0), np.repeat(semantic_rows, row_repeat, axis=0)

    def scan_callback(self, scan_01: LaserScan, scan_02: LaserScan) -> None:
        now_ns = self.observe_clock()
        self.scan_count += 1
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
            self.clear_temporal_history()
            self.publish_stop()
            self.get_logger().warning(
                "dual scan timestamps are stale, future-dated, or unsynchronized",
                throttle_duration_sec=2.0,
            )
            return
        causal_subgoal_sample = latest_causal_sample(
            self.subgoal_history,
            scan_01_stamp_ns,
            float(self.get_parameter("subgoal_timeout").value),
        )
        odom_timeout_ns = int(
            float(self.get_parameter("odom_timeout").value)
            * NANOSECONDS_PER_SECOND
        )
        odom_delta_ns = (
            None
            if self.pose_stamp_ns is None
            else abs(int(scan_01_stamp_ns) - int(self.pose_stamp_ns))
        )
        if (
            self.pose is None
            or odom_delta_ns is None
            or odom_delta_ns > odom_timeout_ns
        ):
            self.clear_temporal_history()
            self.publish_stop("stale_odom", input_stamp=scan_01.header.stamp)
            self.get_logger().warning(
                "odom is missing or stale relative to the synchronized scan",
                throttle_duration_sec=2.0,
            )
            return
        if causal_subgoal_sample is None:
            self.clear_temporal_history()
            self.publish_stop()
            return
        causal_subgoal, _causal_subgoal_stamp_ns = causal_subgoal_sample
        try:
            ranges_01, angles_01, valid_01 = self.virtualize_scan(scan_01)
            ranges_02, angles_02, valid_02 = self.virtualize_scan(scan_02)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            self.clear_temporal_history()
            self.publish_stop()
            return
        ranges = np.concatenate((ranges_01, ranges_02))
        angles = np.concatenate((angles_01, angles_02))
        valid = np.concatenate((valid_01, valid_02))
        self.debug_ranges = ranges
        self.debug_angles = angles
        self.debug_valid = valid
        semantic = self.semantic_for_points(ranges, angles, valid)
        self.scan_history.append(ranges)
        self.angle_history.append(angles)
        self.semantic_history.append(semantic)
        self.valid_history.append(valid)
        self.last_scan_time = self.get_clock().now()
        self.scan_timeout = False
        front = ranges[valid & (np.abs(angles) <= 0.35)]
        # An empty *front sector* is valid in an open corridor (all returns can
        # be to the sides or behind the robot).  The fail-safe must therefore
        # distinguish that case from a completely empty dual scan; otherwise
        # the controller can never start from an open-space pose.
        if (
            not valid.any()
            and bool(self.get_parameter("stop_on_empty_front").value)
        ):
            self.clear_temporal_history()
            self.publish_stop()
            self.get_logger().warning(
                "dual scan has no valid returns; fail-safe stop",
                throttle_duration_sec=2.0,
            )
            return
        self.front_min = float(np.min(front)) if front.size else float("inf")
        self.front_stop = False
        self.goal_stop = False
        if len(self.scan_history) < SEQ_LEN:
            self.publish_stop()
            self.publish_debug_markers()
            return
        if float(np.linalg.norm(causal_subgoal)) <= float(self.get_parameter("goal_tolerance").value):
            self.goal_stop = True
            self.publish_stop()
            self.publish_debug_markers()
            return
        inference_started = time.perf_counter()
        scan_map, semantic_map = self.make_cnn_maps()
        if not (
            np.isfinite(scan_map).all()
            and np.isfinite(semantic_map).all()
        ):
            self.clear_temporal_history()
            self.publish_stop()
            self.get_logger().error(
                "non-finite SemanticCNN input; fail-safe stop"
            )
            return
        self.debug_scan_map = scan_map
        self.debug_semantic_map = semantic_map
        if self.publish_debug_images:
            stamp = self.get_clock().now().to_msg()
            self.scan_map_pub.publish(float_image(scan_map, stamp, str(self.get_parameter("base_frame").value)))
            self.semantic_map_pub.publish(float_image(semantic_map, stamp, str(self.get_parameter("base_frame").value)))
        subgoal = (causal_subgoal - self.goal_mean) / self.goal_std
        preprocessing_finished = time.perf_counter()
        policy_started = preprocessing_finished
        with torch.no_grad():
            prediction = self.model(
                torch.from_numpy(scan_map).float().unsqueeze(0).to(self.device),
                torch.from_numpy(semantic_map).float().unsqueeze(0).to(self.device),
                torch.from_numpy(subgoal.astype(np.float32)).unsqueeze(0).to(self.device),
            ).squeeze(0).cpu().numpy()
        policy_finished = time.perf_counter()
        if prediction.shape != (2,) or not np.isfinite(prediction).all():
            self.clear_temporal_history()
            self.publish_stop()
            self.get_logger().error(
                "invalid SemanticCNN output; fail-safe stop"
            )
            return
        raw_cmd = Twist()
        raw_cmd.linear.x, raw_cmd.angular.z = float(prediction[0]), float(prediction[1])
        self.last_raw_linear = raw_cmd.linear.x
        self.last_raw_angular = raw_cmd.angular.z
        cmd = Twist()
        cmd.linear.x = float(np.clip(prediction[0], -float(self.get_parameter("max_linear").value), float(self.get_parameter("max_linear").value)))
        cmd.angular.z = float(np.clip(prediction[1], -float(self.get_parameter("max_angular").value), float(self.get_parameter("max_angular").value)))
        gate_reasons = []
        if not math.isclose(float(prediction[0]), cmd.linear.x, abs_tol=1.0e-6):
            gate_reasons.append("linear_limit")
        if not math.isclose(float(prediction[1]), cmd.angular.z, abs_tol=1.0e-6):
            gate_reasons.append("angular_limit")
        if self.front_min <= float(self.get_parameter("front_stop_distance").value):
            self.front_stop = True
            cmd.linear.x = 0.0
            gate_reasons.append("front_stop")
            if abs(cmd.angular.z) < float(self.get_parameter("front_stop_angular_deadband").value):
                direction = 1.0 if float(causal_subgoal[1]) >= 0.0 else -1.0
                cmd.angular.z = direction * min(
                    float(self.get_parameter("front_stop_min_angular").value),
                    float(self.get_parameter("max_angular").value),
                )
        self.last_cmd_linear = cmd.linear.x
        self.last_cmd_angular = cmd.angular.z
        self.inference_sequence_id += 1
        self.raw_pub.publish(raw_cmd)
        self.debug_raw_pub.publish(raw_cmd)
        try:
            self.publish_actuation_decision(
                prediction,
                np.asarray([cmd.linear.x, cmd.angular.z], dtype=np.float64),
                input_stamp=scan_01.header.stamp,
                reasons=tuple(dict.fromkeys(gate_reasons)),
                front_min=self.front_min,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"SemanticCNN actuation telemetry failed: {exc}",
                throttle_duration_sec=2.0,
            )
        self.cmd_pub.publish(cmd)
        self.debug_cmd_pub.publish(cmd)
        self.publish_debug_markers()
        postprocessing_finished = time.perf_counter()
        self.publish_inference_metrics(
            sequence_id=self.inference_sequence_id,
            input_stamp=scan_01.header.stamp,
            preprocessing_ms=(preprocessing_finished - inference_started) * 1e3,
            policy_ms=(policy_finished - policy_started) * 1e3,
            postprocessing_ms=(postprocessing_finished - policy_finished) * 1e3,
            total_ms=(postprocessing_finished - inference_started) * 1e3,
            action=prediction,
        )


def main() -> None:
    rclpy.init()
    node = FixedDualSemanticCnnInference()
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
