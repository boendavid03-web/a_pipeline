#!/usr/bin/env python3
"""External ROS evidence monitor for Gate 7.2 DR-SPAAM tracking."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from semantic_nav_gazebo.msg import TrackedPedestrianArray
from sensor_msgs.msg import PointCloud2


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Monitor(Node):
    def __init__(self):
        super().__init__("isaac5_gate72_tracking_monitor")
        self.detections = []
        self.tracks = []
        self.create_subscription(PointCloud2, "/dr_spaam_detections_scored", self.on_detection, 20)
        self.create_subscription(TrackedPedestrianArray, "/pedestrian_tracks", self.on_tracks, 20)

    def on_detection(self, message: PointCloud2) -> None:
        self.detections.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "point_count": int(message.width) * int(message.height),
            }
        )

    def on_tracks(self, message: TrackedPedestrianArray) -> None:
        rows = []
        for track in message.tracks:
            rows.append(
                {
                    "track_id": int(track.track_id),
                    "position": [float(track.position.x), float(track.position.y)],
                    "velocity": [float(track.velocity.x), float(track.velocity.y)],
                    "confidence": float(track.confidence),
                    "state": str(track.state),
                    "age": int(track.age),
                    "hits": int(track.hits),
                    "misses": int(track.misses),
                }
            )
        self.tracks.append(
            {
                "receive_wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame_id": message.header.frame_id,
                "tracks": rows,
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

    detection_stamps = {row["stamp_ns"] for row in node.detections}
    nonempty = [row for row in node.tracks if row["tracks"]]
    all_tracks = [track for row in nonempty for track in row["tracks"]]
    unique_ids = sorted({track["track_id"] for track in all_tracks})
    matched_stamps = sum(row["stamp_ns"] in detection_stamps for row in node.tracks)
    finite = all(
        all(math.isfinite(value) for value in track["position"] + track["velocity"])
        and math.isfinite(track["confidence"])
        for track in all_tracks
    )
    confirmed = sum(track["state"] == "CONFIRMED" for track in all_tracks)
    checks = {
        "detector_input_observed": len(node.detections) >= 10,
        "tracker_output_observed": len(node.tracks) >= 10,
        "nonempty_tracks_observed": bool(nonempty),
        "stable_track_ids_observed": bool(unique_ids),
        "odom_frame": bool(node.tracks) and all(row["frame_id"] == "odom" for row in node.tracks),
        "causal_stamp_passthrough": bool(node.tracks) and matched_stamps == len(node.tracks),
        "finite_state": bool(all_tracks) and finite,
        "confirmed_tracks": confirmed > 0,
        "delivery_rate": rate(node.tracks) is not None and 10.0 <= rate(node.tracks) <= 16.5,
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "gate": "7.2",
        "component": "DR-SPAAM plus PointCVKalmanTracker",
        "detection_frames": len(node.detections),
        "track_frames": len(node.tracks),
        "nonempty_track_frames": len(nonempty),
        "track_wall_rate_hz": rate(node.tracks),
        "unique_track_ids": unique_ids,
        "maximum_tracks_per_frame": max((len(row["tracks"]) for row in node.tracks), default=0),
        "confirmed_track_samples": confirmed,
        "matched_detection_stamps": matched_stamps,
        "checks": checks,
        "sample": nonempty[-1] if nonempty else None,
    }
    print("GATE72_TRACKING_MONITOR_RESULT=" + json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
