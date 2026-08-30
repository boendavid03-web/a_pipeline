#!/usr/bin/env python3
"""Evaluate scored DR-SPAAM detections against side-channel pedestrian GT."""

from __future__ import annotations

import json
import math
import os
import struct
from pathlib import Path
from typing import Iterable

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from semantic_nav_gazebo.msg import PedestrianStateArray
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def greedy_one_to_one(
    detections: list[tuple[float, float, float]],
    ground_truth: list[tuple[str, float, float]],
    threshold: float,
) -> list[tuple[int, int, float]]:
    candidates = []
    for det_index, (_, det_x, det_y) in enumerate(detections):
        for gt_index, (_, gt_x, gt_y) in enumerate(ground_truth):
            distance = math.hypot(det_x - gt_x, det_y - gt_y)
            if distance <= threshold:
                candidates.append((distance, det_index, gt_index))
    matches = []
    used_detections, used_ground_truth = set(), set()
    for distance, det_index, gt_index in sorted(candidates):
        if det_index in used_detections or gt_index in used_ground_truth:
            continue
        used_detections.add(det_index)
        used_ground_truth.add(gt_index)
        matches.append((det_index, gt_index, distance))
    return matches


def transform_xy(transform, x: float, y: float) -> tuple[float, float]:
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        cosine * x - sine * y + translation.x,
        sine * x + cosine * y + translation.y,
    )


class StreamStats:
    def __init__(self) -> None:
        self.count = 0
        self.first_ns = None
        self.last_ns = None
        self.metadata = {}

    def observe(self, timestamp_ns: int) -> None:
        self.count += 1
        if self.first_ns is None:
            self.first_ns = timestamp_ns
        self.last_ns = timestamp_ns

    def frequency(self):
        if self.count < 2 or self.last_ns <= self.first_ns:
            return None
        return (self.count - 1) * 1.0e9 / (self.last_ns - self.first_ns)

    def report(self) -> dict:
        return {
            "message_count": self.count,
            "frequency_hz": self.frequency(),
            **self.metadata,
        }


class DrSpaamDetectionEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("dr_spaam_detection_evaluator")
        output_dir = str(self.declare_parameter("output_dir", "").value)
        self.output_dir = Path(output_dir or "runs/dr_spaam_isaac_smoke/latest")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trace_path = self.output_dir / "detector_trace.jsonl"
        self.summary_path = self.output_dir / "summary.json"
        self.match_threshold = float(
            self.declare_parameter("match_threshold", 0.5).value
        )
        self.max_gt_age = float(self.declare_parameter("max_gt_age", 0.25).value)
        self.target_frame = str(
            self.declare_parameter("target_frame", "base_link").value
        )
        if self.match_threshold <= 0.0 or self.max_gt_age <= 0.0:
            raise ValueError("matching parameters must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_gt = None
        self.streams = {
            "scan_01": StreamStats(),
            "scan_02": StreamStats(),
            "scan_merged": StreamStats(),
            "detections": StreamStats(),
        }
        self.frames = 0
        self.gt_count = 0
        self.detection_count = 0
        self.tp = 0
        self.fp = 0
        self.fn = 0
        self.errors = []
        self.finite = True
        self.skipped_without_gt = 0
        self.skipped_tf = 0
        self.cmd_vel_message_count = 0
        self.nonzero_cmd_vel_count = 0
        self.odom_message_count = 0
        self.max_reported_planar_speed = 0.0
        self.distance_bins = {
            "0-2": {"low": 0.0, "high": 2.0, "gt_count": 0, "tp": 0},
            "2-4": {"low": 2.0, "high": 4.0, "gt_count": 0, "tp": 0},
            "4-6": {"low": 4.0, "high": 6.0, "gt_count": 0, "tp": 0},
            "6-8": {"low": 6.0, "high": 8.0, "gt_count": 0, "tp": 0},
        }

        self.create_subscription(
            LaserScan, "/scan_01", lambda msg: self.scan(msg, "scan_01"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan_02", lambda msg: self.scan(msg, "scan_02"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan_merged", lambda msg: self.scan(msg, "scan_merged"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/dr_spaam_detections_scored",
            self.detections,
            20,
        )
        self.create_subscription(
            PedestrianStateArray, "/pedestrian_ground_truth", self.ground_truth, 20
        )
        self.create_subscription(Odometry, "/odom", self.odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel, 20)
        self.get_logger().info(
            f"DR-SPAAM evaluator ready: target={self.target_frame}, "
            f"threshold={self.match_threshold:.3f} m, output={self.output_dir}"
        )

    def scan(self, message: LaserScan, name: str) -> None:
        stream = self.streams[name]
        stream.observe(stamp_ns(message.header.stamp))
        values = [float(value) for value in message.ranges]
        finite_count = sum(math.isfinite(value) for value in values)
        stream.metadata = {
            "message_type": "sensor_msgs/msg/LaserScan",
            "frame": message.header.frame_id,
            "beams": len(values),
            "angle_min": float(message.angle_min),
            "angle_max": float(message.angle_max),
            "fov_rad": float(message.angle_max - message.angle_min),
            "angle_increment": float(message.angle_increment),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
            "finite_ranges": finite_count,
            "inf_ranges": sum(math.isinf(value) for value in values),
            "nan_ranges": sum(math.isnan(value) for value in values),
        }

    def ground_truth(self, message: PedestrianStateArray) -> None:
        self.latest_gt = message

    def odom(self, message: Odometry) -> None:
        self.odom_message_count += 1
        twist = message.twist.twist
        speed = math.hypot(float(twist.linear.x), float(twist.linear.y))
        if math.isfinite(speed):
            self.max_reported_planar_speed = max(self.max_reported_planar_speed, speed)
        else:
            self.finite = False

    def cmd_vel(self, message: Twist) -> None:
        self.cmd_vel_message_count += 1
        values = (message.linear.x, message.linear.y, message.angular.z)
        if any(abs(float(value)) > 1.0e-6 for value in values):
            self.nonzero_cmd_vel_count += 1

    @staticmethod
    def read_detections(message: PointCloud2) -> list[tuple[float, float, float]]:
        offsets = {field.name: field for field in message.fields}
        required = ("x", "y", "confidence")
        if any(name not in offsets for name in required):
            raise ValueError("scored point cloud lacks x/y/confidence fields")
        if any(offsets[name].datatype != PointField.FLOAT32 for name in required):
            raise ValueError("scored point cloud fields must be FLOAT32")
        endian = ">" if message.is_bigendian else "<"
        result = []
        for index in range(int(message.width) * int(message.height)):
            base = index * int(message.point_step)
            values = tuple(
                struct.unpack_from(
                    endian + "f", message.data, base + int(offsets[name].offset)
                )[0]
                for name in required
            )
            result.append((values[2], values[0], values[1]))
        return result

    def transform_points(
        self,
        source_frame: str,
        timestamp,
        points: Iterable[tuple[str, float, float]],
    ) -> list[tuple[str, float, float]]:
        points = list(points)
        if source_frame == self.target_frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            Time.from_msg(timestamp),
            timeout=Duration(seconds=0.05),
        )
        return [
            (identity, *transform_xy(transform, x, y))
            for identity, x, y in points
        ]

    def detections(self, message: PointCloud2) -> None:
        timestamp_ns = stamp_ns(message.header.stamp)
        self.streams["detections"].observe(timestamp_ns)
        self.streams["detections"].metadata = {
            "topic": "/dr_spaam_detections_scored",
            "message_type": "sensor_msgs/msg/PointCloud2",
            "frame": message.header.frame_id,
            "fields": [field.name for field in message.fields],
        }
        if self.latest_gt is None:
            self.skipped_without_gt += 1
            return
        gt_age = abs(timestamp_ns - stamp_ns(self.latest_gt.header.stamp)) * 1.0e-9
        if gt_age > self.max_gt_age:
            self.skipped_without_gt += 1
            return
        try:
            detections = self.read_detections(message)
            det_tagged = [
                (str(index), x, y) for index, (_, x, y) in enumerate(detections)
            ]
            det_transformed = self.transform_points(
                message.header.frame_id, message.header.stamp, det_tagged
            )
            detections = [
                (detections[int(identity)][0], x, y)
                for identity, x, y in det_transformed
            ]
            gt_points = [
                (person.id, float(person.pose.position.x), float(person.pose.position.y))
                for person in self.latest_gt.pedestrians
            ]
            ground_truth = self.transform_points(
                self.latest_gt.header.frame_id,
                self.latest_gt.header.stamp,
                gt_points,
            )
        except (TransformException, ValueError, struct.error) as error:
            self.skipped_tf += 1
            self.get_logger().warning(f"skipping detection frame: {error}")
            return

        all_values = [value for detection in detections for value in detection]
        all_values.extend(value for gt in ground_truth for value in gt[1:])
        frame_finite = all(math.isfinite(float(value)) for value in all_values)
        self.finite = self.finite and frame_finite
        if not frame_finite:
            return
        matches = greedy_one_to_one(detections, ground_truth, self.match_threshold)
        matched_gt = {gt_index for _, gt_index, _ in matches}
        self.frames += 1
        self.gt_count += len(ground_truth)
        self.detection_count += len(detections)
        self.tp += len(matches)
        self.fp += len(detections) - len(matches)
        self.fn += len(ground_truth) - len(matches)
        self.errors.extend(distance for _, _, distance in matches)

        for gt_index, (_, x, y) in enumerate(ground_truth):
            distance = math.hypot(x, y)
            for bucket in self.distance_bins.values():
                if bucket["low"] <= distance < bucket["high"]:
                    bucket["gt_count"] += 1
                    bucket["tp"] += int(gt_index in matched_gt)
                    break

        trace = {
            "timestamp_ns": timestamp_ns,
            "detection_frame": self.target_frame,
            "gt_age_sec": gt_age,
            "scan_metadata": self.streams["scan_merged"].metadata,
            "detections": [
                {"x": x, "y": y, "confidence": confidence}
                for confidence, x, y in detections
            ],
            "ground_truth": [
                {"id": identity, "x": x, "y": y}
                for identity, x, y in ground_truth
            ],
            "matches": [
                {
                    "detection_index": det_index,
                    "gt_index": gt_index,
                    "gt_id": ground_truth[gt_index][0],
                    "distance_error": distance,
                }
                for det_index, gt_index, distance in matches
            ],
        }
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(trace, separators=(",", ":")) + "\n")
        self.write_summary()

    def write_summary(self) -> None:
        errors = sorted(self.errors)
        median = None
        if errors:
            middle = len(errors) // 2
            median = (
                errors[middle]
                if len(errors) % 2
                else 0.5 * (errors[middle - 1] + errors[middle])
            )
        distance_bins = {}
        for name, bucket in self.distance_bins.items():
            gt_count = bucket["gt_count"]
            distance_bins[name] = {
                "gt_count": gt_count,
                "tp": bucket["tp"],
                "recall": bucket["tp"] / gt_count if gt_count else None,
            }
        summary = {
            "schema": "dr_spaam_isaac_detection_smoke/v1",
            "matching": {
                "method": "greedy_nearest_one_to_one",
                "threshold_m": self.match_threshold,
                "target_frame": self.target_frame,
                "ground_truth_role": "evaluation_only",
            },
            "streams": {name: stream.report() for name, stream in self.streams.items()},
            "metrics": {
                "evaluated_frames": self.frames,
                "gt_count": self.gt_count,
                "detection_count": self.detection_count,
                "tp": self.tp,
                "fp": self.fp,
                "fn": self.fn,
                "precision": self.tp / (self.tp + self.fp) if self.tp + self.fp else None,
                "recall": self.tp / (self.tp + self.fn) if self.tp + self.fn else None,
                "mean_position_error_m": sum(errors) / len(errors) if errors else None,
                "median_position_error_m": median,
                "finite": self.finite,
                "distance_bins_m": distance_bins,
            },
            "quality": {
                "skipped_without_synchronized_gt": self.skipped_without_gt,
                "skipped_transform_or_format": self.skipped_tf,
                "cmd_vel_message_count": self.cmd_vel_message_count,
                "nonzero_cmd_vel_count": self.nonzero_cmd_vel_count,
                "odom_message_count": self.odom_message_count,
                "max_reported_planar_speed_mps": self.max_reported_planar_speed,
            },
        }
        temporary = self.summary_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.summary_path)

    def destroy_node(self):
        self.write_summary()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DrSpaamDetectionEvaluator()
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
