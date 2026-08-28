#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/points_merged_after_dedup, /points_merged_before_dedup, /scan_01, /scan_02, /scan_merged
# 检测到的消息类型：LaserScan, PointCloud2, PointField; Time as TimeMsg
# 检测到的文件格式：CSV, JSON, NPZ, PNG
# 可能使用的关键环境变量：FLOAT32, INT32, JSON, UINT32, UINT8
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo v7_dual_laser_scan_merger.py
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/v7_dual_laser_scan_merger.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜v7_dual_laser_scan_merger.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import csv
import json
import math
import struct
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_to_ns(stamp: TimeMsg) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def newer_stamp(first: TimeMsg, second: TimeMsg) -> TimeMsg:
    return first if stamp_to_ns(first) >= stamp_to_ns(second) else second


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "y", "on"):
        return True
    if normalized in ("false", "0", "no", "n", "off"):
        return False
    raise ValueError(f"invalid boolean parameter value: {value!r}")


def rotate_point(
    point: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
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


class V7DualLaserScanMerger(Node):
    """Preserves the 360-slot navigation scan and optionally probes native points."""

    probe_epsilon_values = (0.02, 0.03, 0.05)

    def __init__(self) -> None:
        super().__init__("v7_dual_laser_scan_merger")
        self.input_scan_01_topic = self.declare_parameter(
            "input_scan_01_topic", "/scan_01"
        ).value
        self.input_scan_02_topic = self.declare_parameter(
            "input_scan_02_topic", "/scan_02"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/scan_merged"
        ).value
        self.output_frame = self.declare_parameter("output_frame", "base_link").value
        self.output_samples = int(self.declare_parameter("output_samples", 360).value)
        self.angle_min = float(self.declare_parameter("angle_min", -math.pi).value)
        self.angle_max = float(self.declare_parameter("angle_max", math.pi).value)
        self.range_min = float(self.declare_parameter("range_min", 0.1).value)
        self.range_max = float(self.declare_parameter("range_max", 50.0).value)
        self.sync_slop = float(self.declare_parameter("sync_slop", 0.05).value)
        self.queue_size = int(self.declare_parameter("queue_size", 10).value)
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.05).value)
        self.enable_self_filter = as_bool(
            self.declare_parameter("enable_self_filter", True).value
        )
        self.self_filter_min_x = float(
            self.declare_parameter("self_filter_min_x", -0.36).value
        )
        self.self_filter_max_x = float(
            self.declare_parameter("self_filter_max_x", 0.36).value
        )
        self.self_filter_min_y = float(
            self.declare_parameter("self_filter_min_y", -0.32).value
        )
        self.self_filter_max_y = float(
            self.declare_parameter("self_filter_max_y", 0.32).value
        )

        # Probe defaults are deliberately inert so /scan_merged is unchanged.
        self.enable_high_fidelity_probe = as_bool(
            self.declare_parameter("enable_high_fidelity_probe", False).value
        )
        self.duplicate_epsilon = float(
            self.declare_parameter("duplicate_epsilon", 0.03).value
        )
        self.duplicate_mode = str(
            self.declare_parameter("duplicate_mode", "fixed_xy").value
        ).strip()
        self.publish_probe_pointclouds = as_bool(
            self.declare_parameter("publish_probe_pointclouds", True).value
        )
        self.save_probe_files = as_bool(
            self.declare_parameter("save_probe_files", False).value
        )
        self.probe_output_dir = str(
            self.declare_parameter("probe_output_dir", "").value
        )
        if (
            self.output_samples < 2
            or self.angle_max <= self.angle_min
            or self.range_max <= self.range_min
        ):
            raise ValueError("invalid output scan geometry or range limits")
        if self.duplicate_epsilon <= 0.0:
            raise ValueError("duplicate_epsilon must be positive")
        if self.duplicate_mode not in ("fixed_xy", "adaptive_multi_gate"):
            raise ValueError(
                "duplicate_mode must be one of: fixed_xy, adaptive_multi_gate"
            )

        self.angle_increment = (self.angle_max - self.angle_min) / float(
            self.output_samples - 1
        )
        self.last_publish_ns: Optional[int] = None
        self.publish_count = 0
        self.dropped_tf_count = 0
        self.probe_frame_count = 0
        self.probe_history: List[Dict[str, object]] = []
        self.probe_gaps_deg: List[float] = []
        self.probe_root: Optional[Path] = None
        if self.save_probe_files:
            self.probe_root = (
                Path(self.probe_output_dir)
                if self.probe_output_dir
                else Path.cwd() / "fusion_angle_probe"
            )
            for directory in ("before_dedup", "after_dedup", "figures"):
                (self.probe_root / directory).mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            LaserScan, self.output_topic, qos_profile_sensor_data
        )
        self.before_probe_publisher = None
        self.after_probe_publisher = None
        if self.enable_high_fidelity_probe and self.publish_probe_pointclouds:
            self.before_probe_publisher = self.create_publisher(
                PointCloud2,
                "/points_merged_before_dedup",
                qos_profile_sensor_data,
            )
            self.after_probe_publisher = self.create_publisher(
                PointCloud2,
                "/points_merged_after_dedup",
                qos_profile_sensor_data,
            )
        self.scan_01_sub = message_filters.Subscriber(
            self,
            LaserScan,
            self.input_scan_01_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.scan_02_sub = message_filters.Subscriber(
            self,
            LaserScan,
            self.input_scan_02_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.scan_01_sub, self.scan_02_sub],
            queue_size=self.queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_scan_callback)
        self.get_logger().info(
            f"v7 dual laser merger ready: {self.input_scan_01_topic} + "
            f"{self.input_scan_02_topic} -> {self.output_topic}, "
            f"frame={self.output_frame}, samples={self.output_samples}, "
            f"probe={self.enable_high_fidelity_probe}, "
            f"duplicate_mode={self.duplicate_mode}; "
            f"self filter={'rectangle' if self.enable_self_filter else 'disabled'} "
            f"x=[{self.self_filter_min_x:.2f},{self.self_filter_max_x:.2f}] "
            f"y=[{self.self_filter_min_y:.2f},{self.self_filter_max_y:.2f}]"
        )

    def lookup_transform_for_scan(self, scan: LaserScan):
        if not scan.header.frame_id.strip():
            raise TransformException("input LaserScan has an empty frame_id")
        return self.tf_buffer.lookup_transform(
            self.output_frame,
            scan.header.frame_id.strip(),
            Time.from_msg(scan.header.stamp),
            timeout=Duration(seconds=self.tf_timeout),
        )

    def point_is_self(self, x: float, y: float) -> bool:
        return (
            self.enable_self_filter
            and self.self_filter_min_x <= x <= self.self_filter_max_x
            and self.self_filter_min_y <= y <= self.self_filter_max_y
        )

    def valid_raw_count(self, scan: LaserScan) -> int:
        low = max(float(scan.range_min), self.range_min)
        high = min(float(scan.range_max), self.range_max)
        return sum(
            math.isfinite(float(value)) and low <= float(value) <= high
            for value in scan.ranges
        )

    def collect_scan_points(
        self, scan: LaserScan, sensor: int
    ) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
        stats = {
            "raw": len(scan.ranges),
            "valid": 0,
            "self_filtered": 0,
            "range_filtered_after_tf": 0,
            "tf_failed": 0,
        }
        try:
            transform = self.lookup_transform_for_scan(scan)
        except TransformException:
            stats["tf_failed"] = self.valid_raw_count(scan)
            raise
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        low = max(float(scan.range_min), self.range_min)
        high = min(float(scan.range_max), self.range_max)
        has_intensities = len(scan.intensities) == len(scan.ranges)
        points: List[Dict[str, object]] = []
        for beam_index, measured_range in enumerate(scan.ranges):
            value = float(measured_range)
            if not math.isfinite(value) or value < low or value > high:
                continue
            input_angle = float(scan.angle_min) + beam_index * float(scan.angle_increment)
            rx, ry, _ = rotate_point(
                (
                    value * math.cos(input_angle),
                    value * math.sin(input_angle),
                    0.0,
                ),
                quaternion,
            )
            x, y = translation.x + rx, translation.y + ry
            if self.point_is_self(x, y):
                stats["self_filtered"] += 1
                continue
            virtual_range = math.hypot(x, y)
            if virtual_range < self.range_min or virtual_range > self.range_max:
                stats["range_filtered_after_tf"] += 1
                continue
            intensity = (
                float(scan.intensities[beam_index]) if has_intensities else 0.0
            )
            if not math.isfinite(intensity) or intensity < 0.0:
                intensity = 0.0
            points.append(
                {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "source_sensor": sensor,
                    "raw_beam_index": beam_index,
                    "virtual_range": virtual_range,
                    "virtual_angle": math.atan2(y, x),
                    "intensity": intensity,
                    "cluster_id": 0,
                    "cluster_size": 1,
                    "source_sensor_mask": sensor,
                }
            )
        stats["valid"] = len(points)
        return points, stats

    def add_to_360_slots(
        self,
        points: List[Dict[str, object]],
        ranges: List[float],
        intensities: List[float],
        sources: List[int],
        source_masks: List[int],
    ) -> int:
        discarded = 0
        for point in points:
            index = int(
                round(
                    (float(point["virtual_angle"]) - self.angle_min)
                    / self.angle_increment
                )
            )
            if not 0 <= index < self.output_samples:
                continue
            candidate_range = float(point["virtual_range"])
            source_masks[index] |= int(point["source_sensor"])
            if math.isfinite(ranges[index]):
                discarded += 1
            if candidate_range < ranges[index]:
                ranges[index] = candidate_range
                intensities[index] = float(point["intensity"])
                sources[index] = int(point["source_sensor"])
        return discarded

    def count_slot_competition(
        self, points: List[Dict[str, object]], sample_count: int
    ) -> int:
        """Count hypothetical slot conflicts without changing the navigation scan."""
        ranges = [float("inf")] * sample_count
        angle_increment = (self.angle_max - self.angle_min) / float(sample_count - 1)
        discarded = 0
        for point in points:
            index = int(
                round((float(point["virtual_angle"]) - self.angle_min) / angle_increment)
            )
            if not 0 <= index < sample_count:
                continue
            if math.isfinite(ranges[index]):
                discarded += 1
            ranges[index] = min(ranges[index], float(point["virtual_range"]))
        return discarded

    def make_pointcloud(
        self, stamp: TimeMsg, points: List[Dict[str, object]]
    ) -> PointCloud2:
        field_specs = (
            ("x", 0, PointField.FLOAT32),
            ("y", 4, PointField.FLOAT32),
            ("z", 8, PointField.FLOAT32),
            ("source_sensor", 12, PointField.UINT8),
            ("raw_beam_index", 16, PointField.INT32),
            ("virtual_range", 20, PointField.FLOAT32),
            ("virtual_angle", 24, PointField.FLOAT32),
            ("cluster_id", 28, PointField.UINT32),
            ("cluster_size", 32, PointField.UINT32),
            ("source_sensor_mask", 36, PointField.UINT8),
        )
        fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in field_specs
        ]
        data = b"".join(
            struct.pack(
                "<fffB3xiffIIB3x",
                float(point["x"]),
                float(point["y"]),
                float(point["z"]),
                int(point["source_sensor"]),
                int(point["raw_beam_index"]),
                float(point["virtual_range"]),
                float(point["virtual_angle"]),
                int(point["cluster_id"]),
                int(point["cluster_size"]),
                int(point["source_sensor_mask"]),
            )
            for point in points
        )
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp, self.output_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 40
        msg.row_step = 40 * len(points)
        msg.is_dense = True
        msg.data = data
        return msg

    def deduplicate_cross_sensor(
        self, before: List[Dict[str, object]], epsilon: float
    ) -> Tuple[List[Dict[str, object]], int]:
        scan01 = [p for p in before if p["source_sensor"] == 1]
        scan02 = [p for p in before if p["source_sensor"] == 2]
        grid: Dict[Tuple[int, int], List[int]] = {}
        for index, point in enumerate(scan02):
            cell = (
                math.floor(float(point["x"]) / epsilon),
                math.floor(float(point["y"]) / epsilon),
            )
            grid.setdefault(cell, []).append(index)
        used_02, pairs = set(), []
        for first in scan01:
            cx = math.floor(float(first["x"]) / epsilon)
            cy = math.floor(float(first["y"]) / epsilon)
            best = None
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for second_index in grid.get((cx + dx, cy + dy), []):
                        if second_index in used_02:
                            continue
                        second = scan02[second_index]
                        distance = math.hypot(
                            float(first["x"]) - float(second["x"]),
                            float(first["y"]) - float(second["y"]),
                        )
                        if distance <= epsilon and (best is None or distance < best[0]):
                            best = (distance, second_index)
            if best is not None:
                used_02.add(best[1])
                pairs.append((first, scan02[best[1]]))
        after: List[Dict[str, object]] = []
        cluster_id = 1
        paired_ids = {id(point) for pair in pairs for point in pair}
        for first, second in pairs:
            x = (float(first["x"]) + float(second["x"])) / 2.0
            y = (float(first["y"]) + float(second["y"])) / 2.0
            after.append(
                {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "source_sensor": 1,
                    "raw_beam_index": -1,
                    "virtual_range": math.hypot(x, y),
                    "virtual_angle": math.atan2(y, x),
                    "cluster_id": cluster_id,
                    "cluster_size": 2,
                    "source_sensor_mask": 3,
                }
            )
            cluster_id += 1
        for point in before:
            if id(point) not in paired_ids:
                copied = dict(point)
                copied["cluster_id"] = cluster_id
                after.append(copied)
                cluster_id += 1
        return after, len(pairs)

    @staticmethod
    def circular_angle_difference(first: float, second: float) -> float:
        return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)

    @staticmethod
    def merge_pairs(
        before: List[Dict[str, object]],
        pairs: List[Tuple[Dict[str, object], Dict[str, object]]],
    ) -> List[Dict[str, object]]:
        after: List[Dict[str, object]] = []
        cluster_id = 1
        paired_ids = {id(point) for pair in pairs for point in pair}
        for first, second in pairs:
            x = (float(first["x"]) + float(second["x"])) / 2.0
            y = (float(first["y"]) + float(second["y"])) / 2.0
            after.append(
                {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "source_sensor": 1,
                    "raw_beam_index": -1,
                    "virtual_range": math.hypot(x, y),
                    "virtual_angle": math.atan2(y, x),
                    "cluster_id": cluster_id,
                    "cluster_size": 2,
                    "source_sensor_mask": 3,
                }
            )
            cluster_id += 1
        for point in before:
            if id(point) not in paired_ids:
                copied = dict(point)
                copied["cluster_id"] = cluster_id
                after.append(copied)
                cluster_id += 1
        return after

    def deduplicate_adaptive_multi_gate(
        self, before: List[Dict[str, object]], scan_01: LaserScan, scan_02: LaserScan
    ) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
        """Deduplicate only cross-sensor points using reciprocal gated nearest matches."""
        points_01 = [point for point in before if point["source_sensor"] == 1]
        points_02 = [point for point in before if point["source_sensor"] == 2]
        max_epsilon = 0.08
        grid: Dict[Tuple[int, int], List[int]] = {}
        for index, point in enumerate(points_02):
            cell = (
                math.floor(float(point["x"]) / max_epsilon),
                math.floor(float(point["y"]) / max_epsilon),
            )
            grid.setdefault(cell, []).append(index)

        rejected = {"xy": 0, "range": 0, "angle": 0, "non_mutual": 0}
        candidates: List[Tuple[int, int, float, float, float, float]] = []
        for first_index, first in enumerate(points_01):
            cx = math.floor(float(first["x"]) / max_epsilon)
            cy = math.floor(float(first["y"]) / max_epsilon)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for second_index in grid.get((cx + dx, cy + dy), []):
                        second = points_02[second_index]
                        xy_distance = math.hypot(
                            float(first["x"]) - float(second["x"]),
                            float(first["y"]) - float(second["y"]),
                        )
                        mean_range = 0.5 * (
                            float(first["virtual_range"])
                            + float(second["virtual_range"])
                        )
                        epsilon_xy = float(
                            np.clip(
                                0.01
                                + 0.25 * mean_range
                                + 0.5
                                * (
                                    abs(float(scan_01.angle_increment))
                                    + abs(float(scan_02.angle_increment))
                                ),
                                0.02,
                                0.08,
                            )
                        )
                        if xy_distance >= epsilon_xy:
                            rejected["xy"] += 1
                            continue
                        range_difference = abs(
                            float(first["virtual_range"])
                            - float(second["virtual_range"])
                        )
                        if range_difference >= 0.04:
                            rejected["range"] += 1
                            continue
                        angle_difference = self.circular_angle_difference(
                            float(first["virtual_angle"]),
                            float(second["virtual_angle"]),
                        )
                        if angle_difference >= math.radians(0.5):
                            rejected["angle"] += 1
                            continue
                        candidates.append(
                            (
                                first_index,
                                second_index,
                                xy_distance,
                                epsilon_xy,
                                angle_difference,
                                range_difference,
                            )
                        )

        nearest_01: Dict[int, Tuple[int, float]] = {}
        nearest_02: Dict[int, Tuple[int, float]] = {}
        for first_index, second_index, distance, *_ in candidates:
            if (
                first_index not in nearest_01
                or distance < nearest_01[first_index][1]
            ):
                nearest_01[first_index] = (second_index, distance)
            if (
                second_index not in nearest_02
                or distance < nearest_02[second_index][1]
            ):
                nearest_02[second_index] = (first_index, distance)

        accepted_indices = set()
        for candidate in candidates:
            first_index, second_index, distance, *_ = candidate
            if (
                nearest_01.get(first_index) == (second_index, distance)
                and nearest_02.get(second_index) == (first_index, distance)
            ):
                accepted_indices.add((first_index, second_index))
            else:
                rejected["non_mutual"] += 1
        accepted = [
            candidate
            for candidate in candidates
            if (candidate[0], candidate[1]) in accepted_indices
        ]
        pairs = [
            (points_01[first_index], points_02[second_index])
            for first_index, second_index, *_ in accepted
        ]
        after = self.merge_pairs(before, pairs)

        def mean_or_nan(values: List[float]) -> float:
            return float(np.mean(values)) if values else float("nan")

        return after, {
            "cross_sensor_duplicate_pairs": len(pairs),
            "adaptive_epsilon_mean_m": mean_or_nan([item[3] for item in accepted]),
            "adaptive_xy_distance_mean_m": mean_or_nan([item[2] for item in accepted]),
            "adaptive_angle_difference_mean_deg": mean_or_nan(
                [math.degrees(item[4]) for item in accepted]
            ),
            "adaptive_range_difference_mean_m": mean_or_nan([item[5] for item in accepted]),
            "adaptive_rejected_xy": rejected["xy"],
            "adaptive_rejected_range": rejected["range"],
            "adaptive_rejected_angle": rejected["angle"],
            "adaptive_rejected_non_mutual": rejected["non_mutual"],
        }

    def angle_stats(self, points: List[Dict[str, object]]) -> Dict[str, float]:
        angles = np.sort(
            np.asarray(
                [float(point["virtual_angle"]) for point in points], dtype=float
            )
        )
        if len(angles) < 2:
            keys = (
                "min_gap_rad",
                "max_gap_rad",
                "mean_gap_rad",
                "median_gap_rad",
                "std_gap_rad",
                "cv",
                "small_gap_count",
                "large_gap_count",
                "near_one_degree_ratio",
            )
            return {key: float("nan") for key in keys}
        gaps = np.concatenate(
            (np.diff(angles), [angles[0] + 2.0 * math.pi - angles[-1]])
        )
        mean = float(np.mean(gaps))
        return {
            "min_gap_rad": float(np.min(gaps)),
            "max_gap_rad": float(np.max(gaps)),
            "mean_gap_rad": mean,
            "median_gap_rad": float(np.median(gaps)),
            "std_gap_rad": float(np.std(gaps)),
            "cv": float(np.std(gaps) / mean) if mean else float("nan"),
            "small_gap_count": int(np.sum(gaps < math.radians(0.1))),
            "large_gap_count": int(np.sum(gaps > math.radians(2.0))),
            "near_one_degree_ratio": float(
                np.mean(
                    (gaps >= math.radians(0.8))
                    & (gaps <= math.radians(1.2))
                )
            ),
        }

    def angle_gaps_degrees(self, points: List[Dict[str, object]]) -> List[float]:
        angles = np.sort(
            np.asarray(
                [float(point["virtual_angle"]) for point in points], dtype=float
            )
        )
        if len(angles) < 2:
            return []
        gaps = np.concatenate(
            (np.diff(angles), [angles[0] + 2.0 * math.pi - angles[-1]])
        )
        return np.degrees(gaps).tolist()

    def save_probe(
        self,
        frame_name: str,
        before: List[Dict[str, object]],
        after: List[Dict[str, object]],
        stats: Dict[str, object],
    ) -> None:
        assert self.probe_root is not None

        def values(points, key, dtype=float):
            return np.asarray([point[key] for point in points], dtype=dtype)

        np.savez_compressed(
            self.probe_root / "before_dedup" / f"{frame_name}.npz",
            x=values(before, "x"),
            y=values(before, "y"),
            ranges=values(before, "virtual_range"),
            angles=values(before, "virtual_angle"),
            source_sensor=values(before, "source_sensor", np.uint8),
            raw_beam_index=values(before, "raw_beam_index", np.int32),
        )
        np.savez_compressed(
            self.probe_root / "after_dedup" / f"{frame_name}.npz",
            x=values(after, "x"),
            y=values(after, "y"),
            ranges=values(after, "virtual_range"),
            angles=values(after, "virtual_angle"),
            source_sensor_mask=values(after, "source_sensor_mask", np.uint8),
            cluster_id=values(after, "cluster_id", np.uint32),
            cluster_size=values(after, "cluster_size", np.uint32),
        )
        self.probe_history.append(stats)
        generate_figures = (
            self.probe_frame_count == 1 or self.probe_frame_count % 25 == 0
        )
        self.write_probe_reports(generate_figures=generate_figures)

    def write_probe_reports(self, generate_figures: bool = False) -> None:
        assert self.probe_root is not None
        with (self.probe_root / "frame_stats.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.probe_history[0]))
            writer.writeheader()
            writer.writerows(self.probe_history)
        epsilon_summary = {}
        for epsilon in self.probe_epsilon_values:
            pair_key = f"duplicate_pairs_eps_{epsilon:.2f}"
            after_key = f"after_dedup_eps_{epsilon:.2f}"
            epsilon_summary[str(epsilon)] = {
                "mean_duplicate_pairs": float(
                    np.mean([row[pair_key] for row in self.probe_history])
                ),
                "mean_after_dedup_points": float(
                    np.mean([row[after_key] for row in self.probe_history])
                ),
            }
        with (self.probe_root / "summary.json").open("w") as handle:
            json.dump(
                {
                    "frame_count": len(self.probe_history),
                    "duplicate_epsilon": self.duplicate_epsilon,
                    "duplicate_mode": self.duplicate_mode,
                    "self_filter": {
                        "shape": "axis-aligned rectangle",
                        "x": [self.self_filter_min_x, self.self_filter_max_x],
                        "y": [self.self_filter_min_y, self.self_filter_max_y],
                        "safety_margin": "not separately parameterized",
                    },
                    "epsilon_comparison": epsilon_summary,
                    "latest_frame": self.probe_history[-1],
                },
                handle,
                indent=2,
                allow_nan=True,
            )
        if not generate_figures:
            return
        try:
            import matplotlib.pyplot as plt

            frames = [row["frame"] for row in self.probe_history]

            def plot_series(key: str, filename: str, ylabel: str) -> None:
                plt.figure()
                plt.plot(frames, [row[key] for row in self.probe_history])
                plt.xlabel("frame")
                plt.ylabel(ylabel)
                plt.tight_layout()
                plt.savefig(self.probe_root / "figures" / filename)
                plt.close()

            plot_series("before_dedup_points", "before_dedup_points.png", "points")
            plot_series("after_dedup_points", "after_dedup_points.png", "points")
            plot_series("max_gap_deg", "max_angle_gap_deg.png", "degrees")
            if self.probe_gaps_deg:
                plt.figure()
                plt.hist(self.probe_gaps_deg, bins="auto")
                plt.xlabel("angular gap (degrees)")
                plt.ylabel("count")
                plt.tight_layout()
                plt.savefig(
                    self.probe_root / "figures" / "angle_gap_histogram.png"
                )
                plt.close()
            plt.figure()
            for epsilon in self.probe_epsilon_values:
                plt.plot(
                    frames,
                    [
                        row[f"after_dedup_eps_{epsilon:.2f}"]
                        for row in self.probe_history
                    ],
                    label=f"epsilon={epsilon:.2f}m",
                )
            plt.xlabel("frame")
            plt.ylabel("after-dedup points")
            plt.legend()
            plt.tight_layout()
            plt.savefig(self.probe_root / "figures" / "epsilon_comparison.png")
            plt.close()
        except ImportError:
            self.get_logger().warn(
                "matplotlib unavailable; wrote NPZ/CSV/JSON but skipped probe figures"
            )

    def estimate_scan_time(self, stamp: TimeMsg, scans: Iterable[LaserScan]) -> float:
        stamp_ns = stamp_to_ns(stamp)
        if self.last_publish_ns is not None and stamp_ns > self.last_publish_ns:
            measured = (stamp_ns - self.last_publish_ns) / 1_000_000_000.0
            if 0.02 <= measured <= 1.0:
                return measured
        return next(
            (
                float(scan.scan_time)
                for scan in scans
                if float(scan.scan_time) > 0.0
            ),
            0.1,
        )

    def synced_scan_callback(self, scan_01: LaserScan, scan_02: LaserScan) -> None:
        try:
            points_01, stats_01 = self.collect_scan_points(scan_01, 1)
            points_02, stats_02 = self.collect_scan_points(scan_02, 2)
        except TransformException as exc:
            self.dropped_tf_count += 1
            if self.dropped_tf_count <= 5 or self.dropped_tf_count % 50 == 0:
                self.get_logger().warn(
                    f"Skipping merged scan; TF unavailable: {exc}"
                )
            return
        output_ranges = [float("inf")] * self.output_samples
        output_intensities = [0.0] * self.output_samples
        slot_sources = [0] * self.output_samples
        slot_source_masks = [0] * self.output_samples
        discarded = self.add_to_360_slots(
            points_01,
            output_ranges,
            output_intensities,
            slot_sources,
            slot_source_masks,
        )
        discarded += self.add_to_360_slots(
            points_02,
            output_ranges,
            output_intensities,
            slot_sources,
            slot_source_masks,
        )
        output_stamp = newer_stamp(scan_01.header.stamp, scan_02.header.stamp)
        scan_time = self.estimate_scan_time(output_stamp, (scan_01, scan_02))
        output = LaserScan()
        output.header.stamp = output_stamp
        output.header.frame_id = self.output_frame
        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment
        output.time_increment = scan_time / float(self.output_samples)
        output.scan_time = scan_time
        output.range_min = self.range_min
        output.range_max = self.range_max
        output.ranges = output_ranges
        input_intensities_are_complete = all(
            len(scan.intensities) == len(scan.ranges)
            for scan in (scan_01, scan_02)
        )
        output.intensities = (
            output_intensities if input_intensities_are_complete else []
        )
        self.publisher.publish(output)
        self.last_publish_ns = stamp_to_ns(output_stamp)
        self.publish_count += 1

        before = points_01 + points_02
        if self.enable_high_fidelity_probe:
            adaptive_stats: Dict[str, object] = {}
            if self.duplicate_mode == "adaptive_multi_gate":
                after, adaptive_stats = self.deduplicate_adaptive_multi_gate(
                    before, scan_01, scan_02
                )
                pair_count = int(adaptive_stats["cross_sensor_duplicate_pairs"])
            else:
                after, pair_count = self.deduplicate_cross_sensor(
                    before, self.duplicate_epsilon
                )
            epsilon_results = {
                epsilon: self.deduplicate_cross_sensor(before, epsilon)[1]
                for epsilon in self.probe_epsilon_values
            }
            gaps = self.angle_stats(after)
            self.probe_gaps_deg.extend(self.angle_gaps_degrees(after))
            if self.publish_probe_pointclouds:
                assert self.before_probe_publisher is not None
                assert self.after_probe_publisher is not None
                self.before_probe_publisher.publish(self.make_pointcloud(output_stamp, before))
                self.after_probe_publisher.publish(self.make_pointcloud(output_stamp, after))
            self.probe_frame_count += 1
            stats: Dict[str, object] = {
                "frame": self.probe_frame_count,
                "stamp_ns": stamp_to_ns(output_stamp),
                "sync_delta_ms": abs(
                    stamp_to_ns(scan_01.header.stamp)
                    - stamp_to_ns(scan_02.header.stamp)
                )
                / 1e6,
                "scan_01_raw_beams": stats_01["raw"],
                "scan_02_raw_beams": stats_02["raw"],
                "scan_01_valid_points": stats_01["valid"],
                "scan_02_valid_points": stats_02["valid"],
                "tf_failed_points": stats_01["tf_failed"]
                + stats_02["tf_failed"],
                "self_filtered_points": stats_01["self_filtered"]
                + stats_02["self_filtered"],
                "before_dedup_points": len(before),
                "duplicate_mode": self.duplicate_mode,
                "cross_sensor_duplicate_pairs": pair_count,
                "after_dedup_points": len(after),
                "slot_nonempty": sum(
                    math.isfinite(value) for value in output_ranges
                ),
                "slot_empty": sum(
                    not math.isfinite(value) for value in output_ranges
                ),
                "slot_inf": sum(
                    not math.isfinite(value) for value in output_ranges
                ),
                "slot_scan_01_only": sum(
                    mask == 1 for mask in slot_source_masks
                ),
                "slot_scan_02_only": sum(
                    mask == 2 for mask in slot_source_masks
                ),
                "slot_both_candidates": sum(
                    mask == 3 for mask in slot_source_masks
                ),
                "slot_discarded_candidates": discarded,
                "slot_360_discarded_candidates": discarded,
                "slot_540_discarded_candidates": self.count_slot_competition(
                    before, 540
                ),
                "endpoint_duplicate_direction": math.isclose(
                    self.angle_max - self.angle_min,
                    2.0 * math.pi,
                    abs_tol=1e-9,
                ),
            }
            stats.update(adaptive_stats)
            stats.update(gaps)
            for name in ("min_gap", "max_gap", "mean_gap", "median_gap", "std_gap"):
                value = float(stats[f"{name}_rad"])
                stats[f"{name}_deg"] = (
                    math.degrees(value) if math.isfinite(value) else value
                )
            for epsilon, pairs in epsilon_results.items():
                stats[f"duplicate_pairs_eps_{epsilon:.2f}"] = pairs
                stats[f"after_dedup_eps_{epsilon:.2f}"] = len(before) - pairs
            if self.save_probe_files:
                frame_name = (
                    f"frame_{self.probe_frame_count:06d}_"
                    f"{stamp_to_ns(output_stamp)}"
                )
                self.save_probe(frame_name, before, after, stats)
            if self.probe_frame_count <= 3 or self.probe_frame_count % 50 == 0:
                self.get_logger().info(
                    f"probe frame={self.probe_frame_count}: before={len(before)}, "
                    f"pairs={pair_count}, after={len(after)}, "
                    f"gap mean={stats['mean_gap_deg']:.3f}deg "
                    f"max={stats['max_gap_deg']:.3f}deg, "
                    f"slots={stats['slot_nonempty']}/{self.output_samples}"
                )
        if self.publish_count % 50 == 0:
            finite_count = sum(math.isfinite(value) for value in output_ranges)
            self.get_logger().info(
                f"published {self.publish_count} merged scans; "
                f"finite={finite_count}/{self.output_samples}, "
                f"candidates={len(before)}, discarded_by_slots={discarded}, "
                f"scan_time={scan_time:.3f}s"
            )

    def destroy_node(self):
        if self.save_probe_files and self.probe_history:
            self.write_probe_reports(generate_figures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V7DualLaserScanMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
