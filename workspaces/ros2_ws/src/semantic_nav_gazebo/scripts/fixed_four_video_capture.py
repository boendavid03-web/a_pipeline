#!/usr/bin/env python3
"""Capture real-time-only inputs needed by the fixed-four video renderer.

The evaluator already persists odometry, commands, pedestrians, inference and
actuation telemetry.  This node deliberately records only data that cannot be
    reconstructed from those files: the two synchronized raw LaserScan messages,
    the merged base-frame LaserScan used for the map overlay, the actually
    published global path, accepted goals, and episode events.
"""

from __future__ import annotations

import gzip
import json
import math
import struct
import sys
from array import array
from pathlib import Path

import message_filters
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


CAPTURE_SCHEMA = "fixed_four_video_capture/v1"
LIDAR_MAGIC = b"FFVLIDAR1\n"
LIDAR_RECORD = struct.Struct("<qqII8f")
MERGED_LIDAR_MAGIC = b"FFVMERGED1\n"
MERGED_LIDAR_RECORD = struct.Struct("<qI4f")


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite_or_none(value: float):
    value = float(value)
    return value if math.isfinite(value) else None


class FixedFourVideoCapture(Node):
    def __init__(self) -> None:
        super().__init__("fixed_four_video_capture")
        for name, default in (
            ("video_output_dir", ""),
            ("scan_01_topic", "/scan_01"),
            ("scan_02_topic", "/scan_02"),
            ("scan_merged_topic", "/scan_merged"),
            ("global_path_topic", "/semantic_cnn/global_path"),
            ("goal_accepted_topic", "/data_collection/goal_accepted"),
            ("episode_event_topic", "/data_collection/episode_event"),
            ("method_name", ""),
            ("simulator_name", ""),
            ("sync_queue_size", 20),
            ("sync_slop_sec", 0.05),
        ):
            self.declare_parameter(name, default)

        output_text = str(self.get_parameter("video_output_dir").value).strip()
        if not output_text:
            raise ValueError("video_output_dir must be non-empty")
        self.video_dir = Path(output_text).expanduser().resolve()
        self.sync_dir = self.video_dir / "sync"
        self.sync_dir.mkdir(parents=True, exist_ok=True)
        self.lidar_path = self.sync_dir / "dual_lidar.bin.gz"
        self.merged_lidar_path = self.sync_dir / "merged_lidar.bin.gz"
        self.events_path = self.sync_dir / "navigation_events.jsonl"
        self.summary_path = self.sync_dir / "capture_summary.json"
        for path in (
            self.lidar_path,
            self.merged_lidar_path,
            self.events_path,
            self.summary_path,
        ):
            if path.exists():
                raise FileExistsError(f"refusing to overwrite video capture artifact: {path}")

        self.lidar_file = gzip.open(self.lidar_path, "xb", compresslevel=4)
        self.lidar_file.write(LIDAR_MAGIC)
        self.merged_lidar_file = gzip.open(
            self.merged_lidar_path, "xb", compresslevel=4
        )
        self.merged_lidar_file.write(MERGED_LIDAR_MAGIC)
        self.events_file = self.events_path.open("x", encoding="utf-8")
        self.counts = {
            "dual_lidar": 0,
            "merged_lidar": 0,
            "global_path": 0,
            "accepted_goal": 0,
            "episode_event": 0,
        }
        self.first_time_sec = None
        self.last_time_sec = None
        self.frames = {
            "scan_01": "",
            "scan_02": "",
            "scan_merged": "",
            "global_path": "",
            "goal": "",
        }
        self.last_path_signature = None
        self.closed = False

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
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [scan_01, scan_02],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        synchronizer.registerCallback(self.lidar_callback)
        self._scan_01 = scan_01
        self._scan_02 = scan_02
        self._synchronizer = synchronizer
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_merged_topic").value),
            self.merged_lidar_callback,
            qos_profile_sensor_data,
        )

        accepted_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PathMsg,
            str(self.get_parameter("global_path_topic").value),
            self.path_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("goal_accepted_topic").value),
            self.goal_callback,
            accepted_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("episode_event_topic").value),
            self.episode_event_callback,
            20,
        )
        self.get_logger().info(
            f"Fixed-four video capture writing real sensor/path data to {self.sync_dir}"
        )

    def update_time_bounds(self, timestamp_ns: int) -> None:
        timestamp = timestamp_ns * 1.0e-9
        if self.first_time_sec is None or timestamp < self.first_time_sec:
            self.first_time_sec = timestamp
        if self.last_time_sec is None or timestamp > self.last_time_sec:
            self.last_time_sec = timestamp

    def write_event(self, payload: dict) -> None:
        self.events_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.events_file.flush()

    def lidar_callback(self, scan_01: LaserScan, scan_02: LaserScan) -> None:
        time_01 = stamp_ns(scan_01.header.stamp)
        time_02 = stamp_ns(scan_02.header.stamp)
        ranges_01 = array("f", (float(value) for value in scan_01.ranges))
        ranges_02 = array("f", (float(value) for value in scan_02.ranges))
        if sys.byteorder != "little":
            ranges_01.byteswap()
            ranges_02.byteswap()
        header = LIDAR_RECORD.pack(
            time_01,
            time_02,
            len(ranges_01),
            len(ranges_02),
            float(scan_01.angle_min),
            float(scan_01.angle_increment),
            float(scan_01.range_min),
            float(scan_01.range_max),
            float(scan_02.angle_min),
            float(scan_02.angle_increment),
            float(scan_02.range_min),
            float(scan_02.range_max),
        )
        self.lidar_file.write(header)
        self.lidar_file.write(ranges_01.tobytes())
        self.lidar_file.write(ranges_02.tobytes())
        self.counts["dual_lidar"] += 1
        self.frames["scan_01"] = str(scan_01.header.frame_id)
        self.frames["scan_02"] = str(scan_02.header.frame_id)
        self.update_time_bounds(min(time_01, time_02))
        if self.counts["dual_lidar"] % 30 == 0:
            self.lidar_file.flush()

    def merged_lidar_callback(self, scan: LaserScan) -> None:
        timestamp_ns = stamp_ns(scan.header.stamp)
        ranges = array("f", (float(value) for value in scan.ranges))
        if sys.byteorder != "little":
            ranges.byteswap()
        self.merged_lidar_file.write(
            MERGED_LIDAR_RECORD.pack(
                timestamp_ns,
                len(ranges),
                float(scan.angle_min),
                float(scan.angle_increment),
                float(scan.range_min),
                float(scan.range_max),
            )
        )
        self.merged_lidar_file.write(ranges.tobytes())
        self.counts["merged_lidar"] += 1
        self.frames["scan_merged"] = str(scan.header.frame_id)
        self.update_time_bounds(timestamp_ns)
        if self.counts["merged_lidar"] % 30 == 0:
            self.merged_lidar_file.flush()

    def path_callback(self, message: PathMsg) -> None:
        timestamp = stamp_ns(message.header.stamp)
        points = []
        for pose in message.poses:
            x = finite_or_none(pose.pose.position.x)
            y = finite_or_none(pose.pose.position.y)
            if x is not None and y is not None:
                points.append([x, y])
        if not points:
            return
        signature = (str(message.header.frame_id), tuple((round(x, 5), round(y, 5)) for x, y in points))
        if signature == self.last_path_signature:
            return
        self.last_path_signature = signature
        self.frames["global_path"] = str(message.header.frame_id)
        self.counts["global_path"] += 1
        self.update_time_bounds(timestamp)
        self.write_event(
            {
                "schema": CAPTURE_SCHEMA,
                "type": "global_path",
                "simulation_time_sec": timestamp * 1.0e-9,
                "frame_id": str(message.header.frame_id),
                "points": points,
            }
        )

    def goal_callback(self, message: PointStamped) -> None:
        timestamp = stamp_ns(message.header.stamp)
        x = finite_or_none(message.point.x)
        y = finite_or_none(message.point.y)
        if x is None or y is None:
            return
        self.frames["goal"] = str(message.header.frame_id)
        self.counts["accepted_goal"] += 1
        self.update_time_bounds(timestamp)
        self.write_event(
            {
                "schema": CAPTURE_SCHEMA,
                "type": "accepted_goal",
                "simulation_time_sec": timestamp * 1.0e-9,
                "frame_id": str(message.header.frame_id),
                "goal": [x, y],
            }
        )

    def episode_event_callback(self, message: String) -> None:
        timestamp = self.get_clock().now().nanoseconds
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            event = {"raw": message.data}
        self.counts["episode_event"] += 1
        self.update_time_bounds(timestamp)
        self.write_event(
            {
                "schema": CAPTURE_SCHEMA,
                "type": "episode_event",
                "simulation_time_sec": timestamp * 1.0e-9,
                "event": event,
            }
        )

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.lidar_file.close()
        self.merged_lidar_file.close()
        self.events_file.close()
        summary = {
            "schema": CAPTURE_SCHEMA,
            "method_name": str(self.get_parameter("method_name").value),
            "simulator_name": str(self.get_parameter("simulator_name").value),
            "time_basis": "ROS simulation time from message headers",
            "first_simulation_time_sec": self.first_time_sec,
            "last_simulation_time_sec": self.last_time_sec,
            "counts": self.counts,
            "frames": self.frames,
            "artifacts": {
                "dual_lidar": str(self.lidar_path),
                "merged_lidar": str(self.merged_lidar_path),
                "navigation_events": str(self.events_path),
            },
            "model_predicted_path": {
                "status": "unavailable",
                "reason": "Both current policies publish velocity actions, not predicted path geometry.",
            },
        }
        self.summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FixedFourVideoCapture()
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
