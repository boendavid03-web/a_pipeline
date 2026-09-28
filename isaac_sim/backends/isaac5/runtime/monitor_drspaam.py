#!/usr/bin/env python3
"""External ROS evidence monitor for the detector-only Gate 7.1."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate71_drspaam_monitor")
        self.scan_rows = []
        self.detection_rows = []
        self.create_subscription(LaserScan, "/scan_merged", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/dr_spaam_detections_scored", self.on_detection, 20)

    def on_scan(self, message: LaserScan) -> None:
        self.scan_rows.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "beams": len(message.ranges),
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
            }
        )

    def on_detection(self, message: PointCloud2) -> None:
        self.detection_rows.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "point_count": int(message.width) * int(message.height),
                "fields": [field.name for field in message.fields],
            }
        )


def rate(rows) -> float | None:
    if len(rows) < 2:
        return None
    span = rows[-1]["receive_wall"] - rows[0]["receive_wall"]
    return (len(rows) - 1) / span if span > 0.0 else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    scan_stamps = {row["stamp_ns"] for row in node.scan_rows}
    matched_stamps = sum(row["stamp_ns"] in scan_stamps for row in node.detection_rows)
    positive_frames = sum(row["point_count"] > 0 for row in node.detection_rows)
    checks = {
        "merged_scan_observed": len(node.scan_rows) >= 20,
        "merged_input_shape": bool(node.scan_rows)
        and all(row["beams"] == 360 and row["frame_id"] == "base_link" for row in node.scan_rows),
        "merged_delivery_rate": rate(node.scan_rows) is not None and 13.5 <= rate(node.scan_rows) <= 16.5,
        "detector_output_observed": len(node.detection_rows) >= 2,
        "detector_schema": bool(node.detection_rows)
        and all(row["fields"] == ["x", "y", "confidence"] for row in node.detection_rows),
        "detector_stamp_causal": bool(node.detection_rows) and matched_stamps == len(node.detection_rows),
        "nonempty_detection_observed": positive_frames > 0,
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "7.1",
        "component": "DR-SPAAM detector only",
        "scan_count": len(node.scan_rows),
        "scan_wall_rate_hz": rate(node.scan_rows),
        "scan_first": node.scan_rows[0] if node.scan_rows else None,
        "scan_last": node.scan_rows[-1] if node.scan_rows else None,
        "detection_count": len(node.detection_rows),
        "detection_wall_rate_hz": rate(node.detection_rows),
        "positive_detection_frames": positive_frames,
        "max_detections_per_frame": max((row["point_count"] for row in node.detection_rows), default=0),
        "matched_input_stamps": matched_stamps,
        "checks": checks,
    }
    print("GATE71_DRSPAAM_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
