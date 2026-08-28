#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged, /semantic_cnn/local_subgoal, /semantic_cnn/raw_model_cmd
# 检测到的消息类型：LaserScan; Odometry; PointStamped, Twist
# 检测到的文件格式：PNG, PT, YAML
# 可能使用的关键环境变量：IGNORE_LABEL, NAVIGATION_PROJECT_ROOT, SCAN_BEAMS, TOKENNAV_SRC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo tokennav_v7_inference_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.376915220 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/tokennav_v7_inference_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜tokennav_v7_inference_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""ROS 2 TokenNav controller demo node for the v7 dual-lidar setup."""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


TOKENNAV_SRC = str(navigation_project_root() / "methods" / "tokennav" / "src")
if TOKENNAV_SRC not in sys.path:
    sys.path.insert(0, TOKENNAV_SRC)

from tokennav_inference_adapter import (  # pylint: disable=import-error,wrong-import-position
    SCAN_BEAMS,
    build_tokennav_input,
    load_tokennav_policy,
    predict_velocity,
)


IGNORE_LABEL = -1


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_map_info(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    origin = meta["origin"]
    return float(meta["resolution"]), float(origin[0]), float(origin[1])


def scan_from_msg(msg):
    ranges_raw = np.asarray(msg.ranges, dtype=np.float32).reshape(-1)
    if ranges_raw.size == 0:
        return None, None, None

    if float(msg.angle_increment) != 0.0:
        angles = float(msg.angle_min) + np.arange(ranges_raw.size, dtype=np.float32) * float(msg.angle_increment)
    else:
        angles = np.linspace(float(msg.angle_min), float(msg.angle_max), ranges_raw.size, dtype=np.float32)

    range_min = max(float(msg.range_min), 0.0)
    range_max = float(msg.range_max)
    valid_mask = np.isfinite(ranges_raw) & (ranges_raw >= range_min)
    if math.isfinite(range_max):
        valid_mask &= ranges_raw <= range_max

    ranges = np.nan_to_num(ranges_raw, nan=0.0, posinf=0.0, neginf=0.0)
    ranges[~valid_mask] = 0.0
    return ranges.astype(np.float32), angles.astype(np.float32), valid_mask


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "y", "on"):
        return True
    if normalized in ("false", "0", "no", "n", "off"):
        return False
    raise ValueError(f"invalid boolean parameter value: {value!r}")


def nearest_resample(values, target_count, dtype=None):
    arr = np.asarray(values)
    if arr.size == target_count:
        return arr.astype(dtype or arr.dtype, copy=True)
    src_x = np.linspace(0.0, 1.0, arr.size, dtype=np.float32)
    dst_x = np.linspace(0.0, 1.0, target_count, dtype=np.float32)
    indices = np.abs(src_x[:, None] - dst_x[None, :]).argmin(axis=0)
    return arr[indices].astype(dtype or arr.dtype, copy=False)


def linear_resample(values, target_count):
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    if arr.size == target_count:
        return arr.copy()
    src_x = np.linspace(0.0, 1.0, arr.size, dtype=np.float32)
    dst_x = np.linspace(0.0, 1.0, target_count, dtype=np.float32)
    return np.interp(dst_x, src_x, arr).astype(np.float32)


def resample_scan_contract(scan, angles, valid_mask, semantic, target_count, method):
    if method == "nearest":
        scan_out = nearest_resample(scan, target_count, dtype=np.float32)
    elif method in ("linear", "angle_linear"):
        scan_out = linear_resample(scan, target_count)
    else:
        raise ValueError(f"unsupported resample_method={method!r}")

    angles_out = linear_resample(angles, target_count)
    valid_out = nearest_resample(valid_mask.astype(np.int8), target_count, dtype=np.int8).astype(bool)
    semantic_out = nearest_resample(semantic, target_count, dtype=np.int64)
    scan_out[~valid_out] = 0.0
    return scan_out, angles_out, valid_out, semantic_out


def semantic_for_scan(ranges, angles, valid_mask, pose, label_img, resolution, origin_x, origin_y):
    x, y, yaw = pose
    height, width = label_img.shape[:2]
    world_x = x + ranges * np.cos(yaw + angles)
    world_y = y + ranges * np.sin(yaw + angles)
    cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
    rows = height - 1 - np.floor((world_y - origin_y) / resolution).astype(np.int64)
    valid = valid_mask & (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    labels = np.full(len(ranges), IGNORE_LABEL, dtype=np.int64)
    labels[valid] = label_img[rows[valid], cols[valid]].astype(np.int64)
    return labels


def finite_pair(values):
    return bool(np.isfinite(np.asarray(values, dtype=np.float32)).all())


def as_float(value, default=0.0):
    arr = np.asarray(value, dtype=np.float32).reshape(-1)
    if arr.size == 0 or not np.isfinite(arr[0]):
        return float(default)
    return float(arr[0])


class TokenNavV7InferenceNode(Node):
    def __init__(self):
        super().__init__("tokennav_v7_inference")
        project_root = navigation_project_root()
        maps_root = project_root / "assets" / "maps" / "ros2_workspace" / "semantic_labeling_v6"
        self.declare_parameter(
            "map_yaml",
            str(maps_root / "v6_lidar04m_20m_static_map.yaml"),
        )
        self.declare_parameter(
            "semantic_label",
            str(maps_root / "semantic2d_manual_label" / "label.png"),
        )
        self.declare_parameter(
            "checkpoint_path",
            str(
                project_root
                / "experiments"
                / "tokennav"
                / "checkpoints"
                / "exp_20260628_long_continue_risk_fast_w12_e10"
                / "last.pt"
            ),
        )
        self.declare_parameter("motion_gate_tau", 0.4)
        self.declare_parameter("instruction_id", 0)
        self.declare_parameter("scan_topic", "/scan_merged")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("local_subgoal_topic", "/semantic_cnn/local_subgoal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("front_stop_distance", 1.0)
        self.declare_parameter("front_stop_angular_deadband", 0.05)
        self.declare_parameter("front_stop_min_angular", 0.35)
        self.declare_parameter("max_linear", 0.3)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("device", "cuda" if torch.cuda.is_available() else "cpu")
        self.declare_parameter("allow_scan_resample", False)
        self.declare_parameter("target_num_beams", SCAN_BEAMS)
        self.declare_parameter("resample_method", "nearest")

        self.device = self.get_parameter("device").value
        self.checkpoint_path = str(self.get_parameter("checkpoint_path").value)
        self.motion_gate_tau = float(self.get_parameter("motion_gate_tau").value)
        self.instruction_id = int(self.get_parameter("instruction_id").value)
        self.allow_scan_resample = as_bool(self.get_parameter("allow_scan_resample").value)
        self.target_num_beams = int(self.get_parameter("target_num_beams").value)
        self.resample_method = str(self.get_parameter("resample_method").value).strip()
        if self.target_num_beams != SCAN_BEAMS:
            raise ValueError(f"target_num_beams must be {SCAN_BEAMS} for this checkpoint")
        if self.resample_method not in ("nearest", "linear", "angle_linear"):
            raise ValueError("resample_method must be nearest, linear, or angle_linear")
        self.model = load_tokennav_policy(self.checkpoint_path, self.device)

        self.resolution, self.origin_x, self.origin_y = load_map_info(Path(self.get_parameter("map_yaml").value))
        self.label_img = np.asarray(Image.open(self.get_parameter("semantic_label").value))
        if self.label_img.ndim == 3:
            self.label_img = self.label_img[:, :, 0]
        self.label_img = self.label_img.astype(np.int64)

        self.pose = None
        self.subgoal = None
        self.last_debug_log_time = 0.0
        self.last_beam_warning_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.debug_pub = self.create_publisher(Twist, "/semantic_cnn/raw_model_cmd", 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.create_subscription(PointStamped, self.get_parameter("local_subgoal_topic").value, self.subgoal_callback, 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Loaded TokenNav on "
            f"{self.device}; checkpoint={self.checkpoint_path}; "
            f"motion_gate_tau={self.motion_gate_tau}; "
            f"allow_scan_resample={self.allow_scan_resample}; "
            f"target_num_beams={self.target_num_beams}; "
            f"resample_method={self.resample_method}; "
            f"publishing {self.get_parameter('cmd_vel_topic').value}"
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = np.asarray([p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)], dtype=np.float32)

    def subgoal_callback(self, msg):
        self.subgoal = np.asarray([msg.point.x, msg.point.y], dtype=np.float32)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def warn_beam_count(self, beam_count):
        now = time.monotonic()
        if now - self.last_beam_warning_time >= 2.0:
            self.get_logger().warning(
                f"TokenNav expects {SCAN_BEAMS} scan beams, got {beam_count}; publishing zero cmd"
            )
            self.last_beam_warning_time = now

    def maybe_log_debug(self, raw_velocity, motion_prob, gated_velocity, turn_class, risk_value, cmd, front_stop):
        now = time.monotonic()
        if now - self.last_debug_log_time < 1.0:
            return
        self.get_logger().info(
            "TokenNav "
            f"raw=({float(raw_velocity[0]):.4f}, {float(raw_velocity[1]):.4f}) "
            f"motion_prob={float(motion_prob):.4f} "
            f"gated=({float(gated_velocity[0]):.4f}, {float(gated_velocity[1]):.4f}) "
            f"turn_class={turn_class} risk={float(risk_value):.4f} "
            f"front_stop={front_stop} "
            f"final=({cmd.linear.x:.4f}, {cmd.angular.z:.4f})"
        )
        self.last_debug_log_time = now

    def scan_callback(self, msg):
        if self.pose is None or self.subgoal is None:
            return
        scan, angles, valid_mask = scan_from_msg(msg)
        if scan is None:
            return
        if scan.shape[0] != SCAN_BEAMS and not self.allow_scan_resample:
            self.warn_beam_count(scan.shape[0])
            self.publish_stop()
            return

        semantic = semantic_for_scan(
            scan,
            angles,
            valid_mask,
            self.pose,
            self.label_img,
            self.resolution,
            self.origin_x,
            self.origin_y,
        )
        if scan.shape[0] != SCAN_BEAMS:
            try:
                scan, angles, valid_mask, semantic = resample_scan_contract(
                    scan,
                    angles,
                    valid_mask,
                    semantic,
                    self.target_num_beams,
                    self.resample_method,
                )
            except ValueError as exc:
                self.get_logger().warning(f"{exc}; publishing zero cmd")
                self.publish_stop()
                return
        semantic_for_input = semantic.astype(np.int64, copy=True)
        semantic_for_input[semantic_for_input < 0] = 0

        cmd = Twist()
        raw_cmd = Twist()
        goal_dist = float(np.linalg.norm(self.subgoal))
        center = len(scan) // 2
        half_width = min(40, center, max(0, len(scan) - center - 1))
        front_slice = slice(center - half_width, center + half_width + 1)
        front = scan[front_slice][valid_mask[front_slice]]
        front_min = float(np.min(front)) if front.size else float(msg.range_max)
        if goal_dist <= float(self.get_parameter("goal_tolerance").value):
            self.publish_stop()
            return

        tokennav_input = build_tokennav_input(
            scan,
            semantic_for_input,
            self.subgoal.astype(np.float32),
            instruction_id=self.instruction_id,
        )
        result = predict_velocity(self.model, tokennav_input, motion_gate_tau=self.motion_gate_tau)
        raw_velocity = np.asarray(result["raw_velocity"], dtype=np.float32).reshape(2)
        gated_velocity = np.asarray(result["gated_velocity"], dtype=np.float32).reshape(2)
        motion_prob = as_float(result.get("motion_prob", 1.0), default=1.0)
        turn_class = result.get("turn_class", None)
        risk_value = as_float(result.get("risk_value", 0.0), default=0.0)

        raw_cmd.linear.x = float(raw_velocity[0])
        raw_cmd.angular.z = float(raw_velocity[1])
        cmd.linear.x = float(
            np.clip(
                gated_velocity[0],
                -float(self.get_parameter("max_linear").value),
                float(self.get_parameter("max_linear").value),
            )
        )
        cmd.angular.z = float(
            np.clip(
                gated_velocity[1],
                -float(self.get_parameter("max_angular").value),
                float(self.get_parameter("max_angular").value),
            )
        )

        front_stop = False
        if front_min <= float(self.get_parameter("front_stop_distance").value):
            front_stop = True
            cmd.linear.x = 0.0
            deadband = float(self.get_parameter("front_stop_angular_deadband").value)
            if abs(cmd.angular.z) < deadband:
                min_angular = float(self.get_parameter("front_stop_min_angular").value)
                max_angular = float(self.get_parameter("max_angular").value)
                if abs(float(self.subgoal[1])) > 0.05:
                    turn_direction = 1.0 if float(self.subgoal[1]) > 0.0 else -1.0
                else:
                    left = scan[(angles > 0.2) & (angles < 1.57) & valid_mask]
                    right = scan[(angles < -0.2) & (angles > -1.57) & valid_mask]
                    left_clearance = float(np.median(left)) if left.size else 0.0
                    right_clearance = float(np.median(right)) if right.size else 0.0
                    turn_direction = 1.0 if left_clearance >= right_clearance else -1.0
                cmd.angular.z = turn_direction * min(abs(min_angular), abs(max_angular))

        self.maybe_log_debug(raw_velocity, motion_prob, gated_velocity, turn_class, risk_value, cmd, front_stop)
        if finite_pair([cmd.linear.x, cmd.angular.z]):
            self.debug_pub.publish(raw_cmd)
            self.cmd_pub.publish(cmd)
        else:
            self.get_logger().warning("TokenNav produced non-finite command; publishing zero cmd")
            self.publish_stop()


def main():
    rclpy.init()
    node = TokenNavV7InferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
