#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /odom, /scan_01, /scan_02, /scan_merged, /tf, /tf_static
# 检测到的消息类型：Clock; LaserScan; Odometry; TFMessage; Time; TransformStamped, Twist
# 检测到的文件格式：JSON, NPY, PGM, PNG, TXT, YAML
# 可能使用的关键环境变量：ANGLE_MAX, ANGLE_MIN, FRAME_BASE, FRAME_MAP, FRAME_ODOM, FRAME_SCAN_01, FRAME_SCAN_02, FRAME_SCAN_MERGED, LABEL_NAMES, LABEL_PNG, MAP_DIR, MAP_PGM, MAP_YAML, NUM_FRAMES, ORIGIN_X, ORIGIN_Y, OUTPUT_DIR, PROJECT_ROOT, RANGE_MAX, RANGE_MIN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/generate_sim_raw_v7_semantic2d_seed.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.950217933 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/generate_sim_raw_v7_semantic2d_seed.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/generate_sim_raw_v7_semantic2d_seed.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/generate_sim_raw_v7_semantic2d_seed.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜generate_sim_raw_v7_semantic2d_seed.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import json
import math
import shutil
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image

import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


PROJECT_ROOT = Path(__file__).resolve().parents[3]
WORKSPACE = PROJECT_ROOT / "workspaces" / "ros2_ws"
MAP_DIR = PROJECT_ROOT / "assets" / "maps" / "ros2_workspace" / "semantic_labeling_v6"
OUTPUT_DIR = WORKSPACE / "_codex_generated" / "sim_raw_v7_from_semantic_labeling_v6"

MAP_YAML = MAP_DIR / "v6_lidar04m_20m_static_map.yaml"
MAP_PGM = MAP_DIR / "v6_lidar04m_20m_static_map.pgm"
LABEL_PNG = MAP_DIR / "semantic2d_manual_label" / "label.png"
LABEL_NAMES = MAP_DIR / "semantic2d_manual_label" / "label_names.txt"

FRAME_BASE = "base_link"
FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_SCAN_MERGED = "base_link"
FRAME_SCAN_01 = "mecanum730_xms5_v7_teacher_dual_scan/base_scan_01/lidar_2d_01"
FRAME_SCAN_02 = "mecanum730_xms5_v7_teacher_dual_scan/base_scan_02/lidar_2d_02"

SCAN_01_X = 0.2
SCAN_01_Y = 0.13
SCAN_01_YAW = 0.0
SCAN_02_X = -0.2
SCAN_02_Y = -0.13
SCAN_02_YAW = math.pi

RESOLUTION = 0.05
ORIGIN_X = -0.433
ORIGIN_Y = -0.0397

NUM_FRAMES = 300
SCAN_HZ = 10.0
SAMPLES = 360
ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
RANGE_MIN = 0.1
RANGE_MAX = 50.0
RAY_STEP = 0.05


def time_msg(t_sec: float) -> Time:
    msg = Time()
    msg.sec = int(t_sec)
    msg.nanosec = int(round((t_sec - msg.sec) * 1_000_000_000))
    return msg


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def load_map():
    occ_img = np.array(Image.open(MAP_PGM).convert("L"))
    labels = np.array(Image.open(LABEL_PNG).convert("L"))
    if occ_img.shape != labels.shape:
        raise RuntimeError(f"map/label shape mismatch: {occ_img.shape} vs {labels.shape}")
    return occ_img, labels


def world_to_pixel(x: float, y: float, height: int):
    col = int(round((x - ORIGIN_X) / RESOLUTION))
    row = int(round(height - 1 - (y - ORIGIN_Y) / RESOLUTION))
    return row, col


def pixel_to_world(row: int, col: int, height: int):
    x = ORIGIN_X + col * RESOLUTION
    y = ORIGIN_Y + (height - 1 - row) * RESOLUTION
    return x, y


def is_free(occ_img: np.ndarray, row: int, col: int) -> bool:
    h, w = occ_img.shape
    if row < 0 or row >= h or col < 0 or col >= w:
        return False
    return occ_img[row, col] > 240


def has_free_margin(free: np.ndarray, row: int, col: int, radius: int = 8) -> bool:
    h, w = free.shape
    r0, r1 = max(0, row - radius), min(h, row + radius + 1)
    c0, c1 = max(0, col - radius), min(w, col + radius + 1)
    patch = free[r0:r1, c0:c1]
    return patch.size > 0 and float(patch.mean()) > 0.98


def nearest_free_node(nodes, target_xy):
    tx, ty = target_xy
    return min(nodes, key=lambda n: (n[2] - tx) ** 2 + (n[3] - ty) ** 2)


def build_route(occ_img: np.ndarray):
    free = occ_img > 240
    h, w = occ_img.shape
    nodes = []
    index = {}
    step = 10

    for row in range(10, h - 10, step):
        for col in range(10, w - 10, step):
            if has_free_margin(free, row, col):
                wx, wy = pixel_to_world(row, col, h)
                index[(row, col)] = len(nodes)
                nodes.append((row, col, wx, wy))

    if not nodes:
        raise RuntimeError("no free route nodes found")

    def neighbors(node):
        row, col = node[:2]
        for dr, dc in ((step, 0), (-step, 0), (0, step), (0, -step)):
            key = (row + dr, col + dc)
            if key in index:
                yield nodes[index[key]]

    def shortest(start, goal):
        start_key = start[:2]
        goal_key = goal[:2]
        parent = {start_key: None}
        q = deque([start])
        while q:
            cur = q.popleft()
            if cur[:2] == goal_key:
                break
            for nxt in neighbors(cur):
                key = nxt[:2]
                if key not in parent:
                    parent[key] = cur[:2]
                    q.append(nxt)
        if goal_key not in parent:
            return [start]
        path_keys = []
        key = goal_key
        while key is not None:
            path_keys.append(key)
            key = parent[key]
        path_keys.reverse()
        return [nodes[index[key]] for key in path_keys]

    waypoints_xy = [
        (2.0, 2.0),
        (7.0, 4.0),
        (13.0, 5.0),
        (20.0, 6.0),
        (25.0, 10.0),
        (21.0, 16.0),
        (15.0, 18.0),
        (8.0, 15.0),
        (3.0, 9.0),
        (2.0, 2.0),
    ]
    waypoints = [nearest_free_node(nodes, xy) for xy in waypoints_xy]

    route = []
    for a, b in zip(waypoints, waypoints[1:]):
        segment = shortest(a, b)
        if route:
            segment = segment[1:]
        route.extend(segment)

    if len(route) < 2:
        raise RuntimeError("route generation failed")

    points = [(n[2], n[3]) for n in route]
    return resample_polyline(points, NUM_FRAMES)


def resample_polyline(points, count):
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = distances[-1]
    if total <= 0:
        raise RuntimeError("route length is zero")

    poses = []
    for i in range(count):
        target = total * i / max(1, count - 1)
        j = 0
        while j + 1 < len(distances) and distances[j + 1] < target:
            j += 1
        if j + 1 >= len(points):
            x, y = points[-1]
            px, py = points[-2]
        else:
            span = max(1e-9, distances[j + 1] - distances[j])
            alpha = (target - distances[j]) / span
            x = points[j][0] + alpha * (points[j + 1][0] - points[j][0])
            y = points[j][1] + alpha * (points[j + 1][1] - points[j][1])
            px, py = points[j]
        yaw = math.atan2(y - py, x - px) if math.hypot(y - py, x - px) > 1e-6 else 0.0
        poses.append((x, y, yaw))
    return poses


def raycast(occ_img: np.ndarray, labels: np.ndarray, x: float, y: float, yaw: float):
    h, w = occ_img.shape
    ranges = np.full((SAMPLES,), np.inf, dtype=np.float32)
    semantic = np.zeros((SAMPLES,), dtype=np.int64)
    angle_increment = (ANGLE_MAX - ANGLE_MIN) / (SAMPLES - 1)

    for i in range(SAMPLES):
        rel_angle = ANGLE_MIN + i * angle_increment
        world_angle = yaw + rel_angle
        hit_label = 0
        r = RANGE_MIN
        while r <= RANGE_MAX:
            px = x + r * math.cos(world_angle)
            py = y + r * math.sin(world_angle)
            row, col = world_to_pixel(px, py, h)
            if row < 0 or row >= h or col < 0 or col >= w:
                ranges[i] = r
                break
            value = int(occ_img[row, col])
            if value < 100 or value <= 240:
                ranges[i] = r
                hit_label = int(labels[row, col])
                break
            r += RAY_STEP
        semantic[i] = hit_label

    return ranges, semantic


def make_scan(stamp: Time, ranges: np.ndarray, frame_id: str) -> LaserScan:
    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.angle_min = ANGLE_MIN
    msg.angle_max = ANGLE_MAX
    msg.angle_increment = (ANGLE_MAX - ANGLE_MIN) / (SAMPLES - 1)
    msg.time_increment = 1.0 / SCAN_HZ / SAMPLES
    msg.scan_time = 1.0 / SCAN_HZ
    msg.range_min = RANGE_MIN
    msg.range_max = RANGE_MAX
    msg.ranges = [float(v) for v in ranges]
    msg.intensities = [0.0] * SAMPLES
    return msg


def make_odom(stamp: Time, pose, velocity) -> Odometry:
    x, y, yaw = pose
    vx, wz = velocity
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ODOM
    msg.child_frame_id = FRAME_BASE
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.twist.twist.linear.x = vx
    msg.twist.twist.angular.z = wz
    return msg


def make_twist(velocity) -> Twist:
    vx, wz = velocity
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz
    return msg


def make_transform(stamp: Time, parent: str, child: str, pose) -> TransformStamped:
    x, y, yaw = pose
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    qx, qy, qz, qw = yaw_to_quat(yaw)
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def make_tf(stamp: Time, parent: str, child: str, pose) -> TFMessage:
    return TFMessage(transforms=[make_transform(stamp, parent, child, pose)])


def make_static_tf(stamp: Time) -> TFMessage:
    return TFMessage(
        transforms=[
            make_transform(stamp, FRAME_BASE, FRAME_SCAN_01, (SCAN_01_X, SCAN_01_Y, 0.0)),
            make_transform(stamp, FRAME_BASE, FRAME_SCAN_02, (SCAN_02_X, SCAN_02_Y, SCAN_02_YAW)),
        ]
    )


def make_clock(stamp: Time) -> Clock:
    msg = Clock()
    msg.clock = stamp
    return msg


def topic_metadata(name: str, type_name: str):
    return rosbag2_py.TopicMetadata(
        name=name,
        type=type_name,
        serialization_format="cdr",
        offered_qos_profiles="",
    )


def write_bag(samples):
    bag_dir = OUTPUT_DIR / "rosbag2_sim_raw"
    if bag_dir.exists():
        shutil.rmtree(bag_dir)

    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    writer.open(storage_options, converter_options)

    topics = {
        "/scan_01": "sensor_msgs/msg/LaserScan",
        "/scan_02": "sensor_msgs/msg/LaserScan",
        "/scan_merged": "sensor_msgs/msg/LaserScan",
        "/odom": "nav_msgs/msg/Odometry",
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/clock": "rosgraph_msgs/msg/Clock",
    }
    for name, type_name in topics.items():
        writer.create_topic(topic_metadata(name, type_name))

    static_tf = make_static_tf(time_msg(0.0))
    writer.write("/tf_static", serialize_message(static_tf), 0)

    for sample in samples:
        stamp = sample["stamp"]
        t_ns = stamp_to_ns(stamp)
        writer.write("/clock", serialize_message(make_clock(stamp)), t_ns)
        writer.write("/scan_01", serialize_message(make_scan(stamp, sample["scan_01"], FRAME_SCAN_01)), t_ns)
        writer.write("/scan_02", serialize_message(make_scan(stamp, sample["scan_02"], FRAME_SCAN_02)), t_ns)
        writer.write("/scan_merged", serialize_message(make_scan(stamp, sample["scan_merged"], FRAME_SCAN_MERGED)), t_ns)
        writer.write("/odom", serialize_message(make_odom(stamp, sample["pose"], sample["velocity"])), t_ns)
        writer.write("/cmd_vel", serialize_message(make_twist(sample["velocity"])), t_ns)
        writer.write("/tf", serialize_message(make_tf(stamp, FRAME_ODOM, FRAME_BASE, sample["pose"])), t_ns)

    return bag_dir


def stamp_to_ns(stamp: Time) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def write_preview_dataset(samples):
    preview = OUTPUT_DIR / "semantic2d_preview"
    if preview.exists():
        shutil.rmtree(preview)
    for sub in ["scans_lidar", "semantic_label", "positions", "velocities"]:
        (preview / sub).mkdir(parents=True, exist_ok=True)

    train_lines = []
    for idx, sample in enumerate(samples):
        stem = f"{idx:06d}"
        np.save(preview / "scans_lidar" / f"{stem}.npy", sample["scan_merged"].astype(np.float32))
        np.save(preview / "semantic_label" / f"{stem}.npy", sample["semantic_merged"].astype(np.int64))
        np.save(preview / "positions" / f"{stem}.npy", np.array(sample["pose"], dtype=np.float32))
        np.save(preview / "velocities" / f"{stem}.npy", np.array(sample["velocity"], dtype=np.float32))
        train_lines.append(stem)

    (preview / "train.txt").write_text("\n".join(train_lines) + "\n")
    (preview / "dev.txt").write_text("\n".join(train_lines[::5]) + "\n")
    shutil.copy2(LABEL_NAMES, preview / "label_names.txt")
    shutil.copy2(MAP_YAML, preview / "source_map.yaml")
    return preview


def compute_velocity(prev_pose, pose, dt):
    if prev_pose is None:
        return 0.0, 0.0
    dx = pose[0] - prev_pose[0]
    dy = pose[1] - prev_pose[1]
    vx = math.hypot(dx, dy) / dt
    dyaw = math.atan2(math.sin(pose[2] - prev_pose[2]), math.cos(pose[2] - prev_pose[2]))
    wz = dyaw / dt
    return vx, wz


def sensor_pose(base_pose, offset_x, offset_y, offset_yaw):
    x, y, yaw = base_pose
    sx = x + math.cos(yaw) * offset_x - math.sin(yaw) * offset_y
    sy = y + math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
    syaw = yaw + offset_yaw
    return sx, sy, syaw


def main():
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    occ_img, labels = load_map()
    poses = build_route(occ_img)
    dt = 1.0 / SCAN_HZ

    samples = []
    prev_pose = None
    for idx, pose in enumerate(poses):
        stamp = time_msg(idx * dt)
        scan_merged, semantic_merged = raycast(occ_img, labels, pose[0], pose[1], pose[2])
        pose_01 = sensor_pose(pose, SCAN_01_X, SCAN_01_Y, SCAN_01_YAW)
        pose_02 = sensor_pose(pose, SCAN_02_X, SCAN_02_Y, SCAN_02_YAW)
        scan_01, semantic_01 = raycast(occ_img, labels, pose_01[0], pose_01[1], pose_01[2])
        scan_02, semantic_02 = raycast(occ_img, labels, pose_02[0], pose_02[1], pose_02[2])
        velocity = compute_velocity(prev_pose, pose, dt)
        samples.append(
            {
                "stamp": stamp,
                "pose": pose,
                "velocity": velocity,
                "scan_merged": scan_merged,
                "semantic_merged": semantic_merged,
                "scan_01": scan_01,
                "semantic_01": semantic_01,
                "scan_02": scan_02,
                "semantic_02": semantic_02,
            }
        )
        prev_pose = pose

    bag_dir = write_bag(samples)
    preview_dir = write_preview_dataset(samples)

    finite_counts = [int(np.isfinite(s["scan_merged"]).sum()) for s in samples]
    label_hist = np.zeros((10,), dtype=np.int64)
    for sample in samples:
        values, counts = np.unique(sample["semantic_merged"], return_counts=True)
        for value, count in zip(values, counts):
            if 0 <= int(value) < len(label_hist):
                label_hist[int(value)] += int(count)

    metadata = {
        "description": "Synthetic raw ROS 2 data generated from the real v6 semantic_labeling_v6 map and label.png.",
        "source_map_yaml": str(MAP_YAML),
        "source_map_pgm": str(MAP_PGM),
        "source_label_png": str(LABEL_PNG),
        "label_names": LABEL_NAMES.read_text().splitlines(),
        "num_frames": NUM_FRAMES,
        "hz": SCAN_HZ,
        "scan_topic": "/scan_merged",
        "scan_01_topic": "/scan_01",
        "scan_02_topic": "/scan_02",
        "scan_frame": FRAME_SCAN_MERGED,
        "scan_01_frame": FRAME_SCAN_01,
        "scan_02_frame": FRAME_SCAN_02,
        "samples_per_scan": SAMPLES,
        "angle_min": ANGLE_MIN,
        "angle_max": ANGLE_MAX,
        "range_min": RANGE_MIN,
        "range_max": RANGE_MAX,
        "finite_count_min": min(finite_counts),
        "finite_count_max": max(finite_counts),
        "finite_count_mean": float(np.mean(finite_counts)),
        "semantic_label_histogram": label_hist.tolist(),
        "rosbag2_dir": str(bag_dir),
        "semantic2d_preview_dir": str(preview_dir),
    }
    (OUTPUT_DIR / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")

    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    rclpy.init(args=None)
    try:
        main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
