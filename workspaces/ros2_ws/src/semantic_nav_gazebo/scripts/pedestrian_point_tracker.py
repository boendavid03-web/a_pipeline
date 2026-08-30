#!/usr/bin/env python3
"""Track scored DR-SPAAM point detections in a stable world frame."""

from __future__ import annotations

import math
import struct

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from semantic_nav_gazebo.msg import TrackedPedestrian, TrackedPedestrianArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from pedestrian_point_tracker_core import PointCVKalmanTracker, PointDetection


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_xy(transform, x: float, y: float) -> tuple[float, float]:
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        cosine * x - sine * y + float(translation.x),
        sine * x + cosine * y + float(translation.y),
    )


def read_scored_points(message: PointCloud2) -> list[PointDetection]:
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "confidence")
    if any(name not in fields for name in required):
        raise ValueError("scored PointCloud2 must contain x/y/confidence")
    if any(fields[name].datatype != PointField.FLOAT32 for name in required):
        raise ValueError("x/y/confidence fields must use FLOAT32")
    endian = ">" if message.is_bigendian else "<"
    detections = []
    for row in range(int(message.height)):
        for column in range(int(message.width)):
            base = row * int(message.row_step) + column * int(message.point_step)
            x, y, confidence = (
                struct.unpack_from(
                    endian + "f",
                    message.data,
                    base + int(fields[name].offset),
                )[0]
                for name in required
            )
            detections.append(
                PointDetection(float(x), float(y), float(confidence))
            )
    return detections


class PedestrianPointTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("pedestrian_point_tracker")
        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/dr_spaam_detections_scored"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/pedestrian_tracks").value
        )
        self.reset_topic = str(
            self.declare_parameter("reset_topic", "/isaac/reset_event").value
        )
        self.tracking_frame = str(
            self.declare_parameter("tracking_frame", "odom").value
        ).lstrip("/")
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.05).value)
        self.tracker = PointCVKalmanTracker(
            association_threshold=float(
                self.declare_parameter("association_threshold", 0.8).value
            ),
            min_hits=int(self.declare_parameter("min_hits", 3).value),
            max_age=int(self.declare_parameter("max_age", 8).value),
            max_coast_time=float(
                self.declare_parameter("max_coast_time", 0.75).value
            ),
            acceleration_sigma=float(
                self.declare_parameter("acceleration_sigma", 2.0).value
            ),
            measurement_sigma=float(
                self.declare_parameter("measurement_sigma", 0.10).value
            ),
            initial_position_sigma=float(
                self.declare_parameter("initial_position_sigma", 0.20).value
            ),
            initial_velocity_sigma=float(
                self.declare_parameter("initial_velocity_sigma", 1.0).value
            ),
            max_prediction_dt=float(
                self.declare_parameter("max_prediction_dt", 0.50).value
            ),
            confidence_alpha=float(
                self.declare_parameter("confidence_alpha", 0.35).value
            ),
        )
        if not self.tracking_frame:
            raise ValueError("tracking_frame cannot be empty")
        if self.tf_timeout <= 0.0:
            raise ValueError("tf_timeout must be positive")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            TrackedPedestrianArray, self.output_topic, 20
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.detections_callback, 20
        )
        self.reset_subscription = self.create_subscription(
            String, self.reset_topic, self.reset_callback, 10
        )
        self.received_frames = 0
        self.published_frames = 0
        self.skipped_format = 0
        self.skipped_tf = 0
        self.skipped_timestamp = 0
        self.reset_count = 0
        self.get_logger().info(
            f"point tracker ready: {self.input_topic} -> {self.output_topic}; "
            f"frame={self.tracking_frame}, gate={self.tracker.association_threshold:.3f} m, "
            f"min_hits={self.tracker.min_hits}, max_age={self.tracker.max_age}"
        )

    def reset_callback(self, _message: String) -> None:
        self.tracker.reset()
        self.reset_count += 1
        self.get_logger().info(
            f"tracker reset from {self.reset_topic}; reset_count={self.reset_count}"
        )

    def detections_callback(self, message: PointCloud2) -> None:
        self.received_frames += 1
        timestamp_ns = stamp_ns(message.header.stamp)
        if timestamp_ns <= 0:
            self.skipped_timestamp += 1
            self.get_logger().warning("discarding zero-stamped detection frame")
            return
        if (
            self.tracker.last_timestamp_ns is not None
            and timestamp_ns <= self.tracker.last_timestamp_ns
        ):
            self.skipped_timestamp += 1
            self.get_logger().warning(
                "discarding duplicate or non-increasing detection timestamp"
            )
            return
        try:
            detections = read_scored_points(message)
        except (ValueError, struct.error) as error:
            self.skipped_format += 1
            self.get_logger().warning(f"discarding malformed detection frame: {error}")
            return
        source_frame = str(message.header.frame_id).lstrip("/")
        if not source_frame:
            self.skipped_format += 1
            self.get_logger().warning("discarding detection frame without frame_id")
            return
        if source_frame != self.tracking_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.tracking_frame,
                    source_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self.tf_timeout),
                )
                detections = [
                    PointDetection(
                        *transform_xy(transform, detection.x, detection.y),
                        detection.confidence,
                    )
                    for detection in detections
                ]
            except TransformException as error:
                self.skipped_tf += 1
                self.get_logger().warning(
                    f"discarding detection frame without {source_frame} -> "
                    f"{self.tracking_frame} TF: {error}"
                )
                return
        try:
            estimates = self.tracker.update(detections, timestamp_ns)
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            self.skipped_format += 1
            self.get_logger().error(f"tracker update rejected: {error}")
            return

        output = TrackedPedestrianArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.tracking_frame
        for estimate in estimates:
            track = TrackedPedestrian()
            track.track_id = estimate.track_id
            track.position.x = estimate.x
            track.position.y = estimate.y
            track.velocity.x = estimate.vx
            track.velocity.y = estimate.vy
            track.confidence = estimate.confidence
            track.age = estimate.age
            track.hits = estimate.hits
            track.misses = estimate.misses
            track.state = estimate.state
            track.time_since_update = estimate.time_since_update
            output.tracks.append(track)
        self.publisher.publish(output)
        self.published_frames += 1
        if self.published_frames % 100 == 0:
            states = {"TENTATIVE": 0, "CONFIRMED": 0, "COASTING": 0}
            for estimate in estimates:
                states[estimate.state] = states.get(estimate.state, 0) + 1
            self.get_logger().info(
                f"published {self.published_frames} track frames; "
                f"tracks={len(estimates)}, states={states}, "
                f"skipped_tf={self.skipped_tf}, skipped_stamp={self.skipped_timestamp}"
            )


def main() -> None:
    rclpy.init()
    node = PedestrianPointTrackerNode()
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
