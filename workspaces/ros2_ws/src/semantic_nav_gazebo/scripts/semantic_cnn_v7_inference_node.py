#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged, /semantic_cnn/local_subgoal, /semantic_cnn/raw_model_cmd
# 检测到的消息类型：LaserScan; Odometry; PointStamped, Twist
# 检测到的文件格式：PNG, PT, YAML
# 可能使用的关键环境变量：GOAL_MU, GOAL_STD, IGNORE_LABEL, IMG_SIZE, NAVIGATION_PROJECT_ROOT, SCAN_MU, SCAN_STD, SEQ_LEN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo semantic_cnn_v7_inference_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.645741956 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_v7_inference_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_v7_inference_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""ROS 2 SemanticCNN controller demo node for the v7 dual-lidar setup.

The first demo version uses map-projected semantic labels instead of online
S3-Net labels so the start/goal demo isolates the navigation model behavior.
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
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


IMG_SIZE = 80
SEQ_LEN = 10
IGNORE_LABEL = -1
SCAN_MU = 4.518406
SCAN_STD = 8.2914915
GOAL_MU = 0.30655652
GOAL_STD = 0.5378557


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_map_info(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    origin = meta["origin"]
    return float(meta["resolution"]), float(origin[0]), float(origin[1])


def world_to_grid(x, y, height, resolution, origin_x, origin_y):
    col = int(math.floor((x - origin_x) / resolution))
    row = height - 1 - int(math.floor((y - origin_y) / resolution))
    return row, col


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


def majority_label(labels):
    labels = np.asarray(labels, dtype=np.int64).reshape(-1)
    labels = labels[labels >= 0]
    if labels.size == 0:
        return 0
    return int(np.bincount(labels, minlength=256).argmax())


def native_lidar_maps(scan, semantic, valid_mask):
    scan_raw = np.asarray(scan, dtype=np.float32).reshape(-1)
    semantic = np.nan_to_num(semantic, nan=IGNORE_LABEL, posinf=IGNORE_LABEL, neginf=IGNORE_LABEL)
    semantic = semantic.astype(np.int64).reshape(-1)
    valid_mask = np.asarray(valid_mask, dtype=np.bool_).reshape(-1)
    if semantic.shape[0] != scan_raw.shape[0] or valid_mask.shape[0] != scan_raw.shape[0]:
        raise ValueError("scan, semantic label, and valid mask shapes must match")

    finite_mask = valid_mask & np.isfinite(scan_raw)
    scan = np.nan_to_num(scan_raw, nan=0.0, posinf=0.0, neginf=0.0)
    semantic_for_input = semantic.copy()
    semantic_for_input[semantic_for_input < 0] = 0

    mins = np.zeros(IMG_SIZE, dtype=np.float32)
    means = np.zeros(IMG_SIZE, dtype=np.float32)
    sem_min = np.zeros(IMG_SIZE, dtype=np.float32)
    sem_mode = np.zeros(IMG_SIZE, dtype=np.float32)
    edges = np.linspace(0, scan.shape[0], IMG_SIZE + 1).astype(np.int64)

    for i in range(IMG_SIZE):
        start = int(edges[i])
        end = int(edges[i + 1])
        if end <= start:
            end = min(start + 1, scan.shape[0])
        idx = np.arange(start, end, dtype=np.int64)
        if idx.size == 0:
            continue
        valid_idx = idx[finite_mask[idx]]
        if valid_idx.size == 0:
            sem_mode[i] = majority_label(semantic_for_input[idx])
            continue
        values = scan[valid_idx]
        local_min = int(np.argmin(values))
        mins[i] = float(values[local_min])
        means[i] = float(values.mean())
        sem_min[i] = float(semantic_for_input[valid_idx[local_min]])
        sem_mode[i] = float(majority_label(semantic_for_input[valid_idx]))

    return mins, means, sem_min, sem_mode


def build_cnn_maps(scan_history, semantic_history, valid_history):
    scan_avg = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
    semantic_avg = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
    for n, (scan, semantic, valid_mask) in enumerate(zip(scan_history, semantic_history, valid_history)):
        mins, means, sem_min, sem_mode = native_lidar_maps(scan, semantic, valid_mask)
        scan_avg[2 * n] = mins
        semantic_avg[2 * n] = sem_min
        scan_avg[2 * n + 1] = means
        semantic_avg[2 * n + 1] = sem_mode

    row_repeat = IMG_SIZE // (SEQ_LEN * 2)
    if row_repeat * (SEQ_LEN * 2) != IMG_SIZE:
        raise ValueError("IMG_SIZE must be divisible by SEQ_LEN*2")
    scan_map = np.repeat(scan_avg, row_repeat, axis=0).reshape(-1)
    semantic_map = np.repeat(semantic_avg, row_repeat, axis=0).reshape(-1)
    return scan_map.astype(np.float32), semantic_map.astype(np.float32)


def load_semantic_cnn(model_code: Path, model_path: Path, device: str):
    sys.path.insert(0, str(model_code))
    from model import Bottleneck, SemanticCNN  # pylint: disable=import-error,import-outside-toplevel

    model = SemanticCNN(Bottleneck, [2, 1, 1])
    checkpoint = torch.load(model_path, map_location=device)
    model.load_state_dict(checkpoint["model"])
    model.to(device)
    model.eval()
    return model


class SemanticCnnV7InferenceNode(Node):
    def __init__(self):
        super().__init__("semantic_cnn_v7_inference")
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
            "model",
            str(
                project_root
                / "experiments"
                / "semantic_cnn"
                / "semantic_cnn_native_cmd_best_dev.pth"
            ),
        )
        self.declare_parameter(
            "model_code",
            str(project_root / "methods" / "baselines" / "semantic_cnn" / "training" / "scripts"),
        )
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

        self.device = self.get_parameter("device").value
        self.model = load_semantic_cnn(
            Path(self.get_parameter("model_code").value),
            Path(self.get_parameter("model").value),
            self.device,
        )
        self.resolution, self.origin_x, self.origin_y = load_map_info(Path(self.get_parameter("map_yaml").value))
        self.label_img = np.asarray(Image.open(self.get_parameter("semantic_label").value))
        if self.label_img.ndim == 3:
            self.label_img = self.label_img[:, :, 0]
        self.label_img = self.label_img.astype(np.int64)

        self.pose = None
        self.subgoal = None
        self.scan_history = deque(maxlen=SEQ_LEN)
        self.semantic_history = deque(maxlen=SEQ_LEN)
        self.valid_history = deque(maxlen=SEQ_LEN)
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
        self.get_logger().info(f"Loaded SemanticCNN on {self.device}; publishing {self.get_parameter('cmd_vel_topic').value}")

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = np.asarray([p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)], dtype=np.float32)

    def subgoal_callback(self, msg):
        self.subgoal = np.asarray([msg.point.x, msg.point.y], dtype=np.float32)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def scan_callback(self, msg):
        if self.pose is None or self.subgoal is None:
            return
        scan, angles, valid_mask = scan_from_msg(msg)
        if scan is None:
            return
        semantic = semantic_for_scan(scan, angles, valid_mask, self.pose, self.label_img, self.resolution, self.origin_x, self.origin_y)
        self.scan_history.append(scan)
        self.semantic_history.append(semantic)
        self.valid_history.append(valid_mask)
        if len(self.scan_history) < SEQ_LEN:
            self.publish_stop()
            return

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
        scan_map, semantic_map = build_cnn_maps(
            list(self.scan_history),
            list(self.semantic_history),
            list(self.valid_history),
        )
        scan_map = (scan_map - SCAN_MU) / SCAN_STD
        subgoal = (self.subgoal - GOAL_MU) / GOAL_STD
        with torch.no_grad():
            out = self.model(
                torch.from_numpy(scan_map).float().to(self.device),
                torch.from_numpy(semantic_map).float().to(self.device),
                torch.from_numpy(subgoal.astype(np.float32)).float().to(self.device),
            )
        pred = out.squeeze().detach().cpu().numpy().astype(np.float32)
        raw_cmd.linear.x = float(pred[0])
        raw_cmd.angular.z = float(pred[1])
        cmd.linear.x = float(np.clip(pred[0], -float(self.get_parameter("max_linear").value), float(self.get_parameter("max_linear").value)))
        cmd.angular.z = float(np.clip(pred[1], -float(self.get_parameter("max_angular").value), float(self.get_parameter("max_angular").value)))
        if front_min <= float(self.get_parameter("front_stop_distance").value):
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
        if np.isfinite([cmd.linear.x, cmd.angular.z]).all():
            self.debug_pub.publish(raw_cmd)
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SemanticCnnV7InferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
