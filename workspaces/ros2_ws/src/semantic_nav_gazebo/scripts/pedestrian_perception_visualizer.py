#!/usr/bin/env python3
"""Visualize DR-SPAAM detections, tracked pedestrians, and evaluation GT.

This node is intentionally display-only.  It never feeds ground truth back to
the detector, tracker, predictor, or navigation stack.
"""

from __future__ import annotations

import math
import struct
import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from semantic_nav_gazebo.msg import PedestrianStateArray, TrackedPedestrianArray
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_scored_points(message: PointCloud2) -> list[tuple[float, float, float]]:
    """Read the project's scored PointCloud2 contract without changing it."""

    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "confidence")
    if any(name not in fields for name in required):
        raise ValueError("scored PointCloud2 must contain x/y/confidence")
    if any(fields[name].datatype != PointField.FLOAT32 for name in required):
        raise ValueError("x/y/confidence fields must use FLOAT32")
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if point_step <= 0 or row_step < point_step * int(message.width):
        raise ValueError("scored PointCloud2 has invalid point/row step")
    if any(int(fields[name].offset) + 4 > point_step for name in required):
        raise ValueError("scored PointCloud2 field offset exceeds point step")

    endian = ">" if message.is_bigendian else "<"
    points: list[tuple[float, float, float]] = []
    for row in range(int(message.height)):
        for column in range(int(message.width)):
            base = row * row_step + column * point_step
            values = tuple(
                struct.unpack_from(
                    endian + "f", message.data, base + int(fields[name].offset)
                )[0]
                for name in required
            )
            if all(math.isfinite(value) for value in values):
                points.append((float(values[0]), float(values[1]), float(values[2])))
    return points


def transform_xy(transform, x: float, y: float) -> tuple[float, float]:
    """Apply a display-only 2D TF transform to one point."""

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


class PedestrianPerceptionVisualizer(Node):
    """Publish a single RViz MarkerArray assembled from perception streams."""

    def __init__(self) -> None:
        super().__init__("pedestrian_perception_visualizer")
        self.tracks_topic = str(
            self.declare_parameter("tracks_topic", "/pedestrian_tracks").value
        )
        self.detections_topic = str(
            self.declare_parameter(
                "detections_topic", "/dr_spaam_detections_scored"
            ).value
        )
        self.ground_truth_topic = str(
            self.declare_parameter(
                "ground_truth_topic", "/pedestrian_ground_truth"
            ).value
        )
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/pedestrian_visualization").value
        )
        self.display_frame = str(
            self.declare_parameter("display_frame", "odom").value
        ).lstrip("/")
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self.stale_timeout = float(
            self.declare_parameter("stale_timeout", 1.0).value
        )
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.05).value)
        if not self.display_frame:
            raise ValueError("display_frame cannot be empty")
        if self.publish_rate_hz <= 0.0 or self.stale_timeout <= 0.0:
            raise ValueError("publish_rate_hz and stale_timeout must be positive")
        if self.tf_timeout <= 0.0:
            raise ValueError("tf_timeout must be positive")

        self.marker_publisher = self.create_publisher(MarkerArray, self.marker_topic, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_tracks: tuple[TrackedPedestrianArray, float] | None = None
        self.latest_detections: tuple[PointCloud2, float] | None = None
        self.latest_ground_truth: tuple[PedestrianStateArray, float] | None = None
        self.format_warning_count = 0
        self.tf_warning_count = 0

        self.create_subscription(
            TrackedPedestrianArray, self.tracks_topic, self.tracks_callback, 20
        )
        self.create_subscription(
            PointCloud2, self.detections_topic, self.detections_callback, 20
        )
        self.create_subscription(
            PedestrianStateArray,
            self.ground_truth_topic,
            self.ground_truth_callback,
            20,
        )
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_markers)
        self.get_logger().info(
            "display-only pedestrian visualization ready: "
            f"tracks={self.tracks_topic}, detections={self.detections_topic}, "
            f"ground_truth={self.ground_truth_topic}, markers={self.marker_topic}, "
            f"frame={self.display_frame}"
        )

    def tracks_callback(self, message: TrackedPedestrianArray) -> None:
        self.latest_tracks = (message, time.monotonic())

    def detections_callback(self, message: PointCloud2) -> None:
        try:
            read_scored_points(message)
        except (ValueError, struct.error) as error:
            self.format_warning_count += 1
            if self.format_warning_count <= 3:
                self.get_logger().warning(f"ignoring malformed scored detections: {error}")
            return
        self.latest_detections = (message, time.monotonic())

    def ground_truth_callback(self, message: PedestrianStateArray) -> None:
        self.latest_ground_truth = (message, time.monotonic())

    def _is_fresh(self, sample: tuple[object, float] | None, now: float) -> bool:
        return sample is not None and now - sample[1] <= self.stale_timeout

    def _transform_point(
        self, source_frame: str, stamp, x: float, y: float
    ) -> tuple[float, float, str] | None:
        source_frame = str(source_frame).lstrip("/") or self.display_frame
        if source_frame == self.display_frame:
            return x, y, self.display_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.display_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as error:
            self.tf_warning_count += 1
            if self.tf_warning_count <= 3:
                self.get_logger().warning(
                    f"cannot transform display point {source_frame}->{self.display_frame}: {error}"
                )
            return None
        transformed_x, transformed_y = transform_xy(transform, x, y)
        return transformed_x, transformed_y, self.display_frame

    def _base_marker(
        self, namespace: str, marker_id: int, marker_type: int, stamp, frame: str
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=self.stale_timeout).to_msg()
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x, point.y, point.z = float(x), float(y), float(z)
        return point

    def _tracker_markers(
        self, message: TrackedPedestrianArray, markers: list[Marker]
    ) -> None:
        source_frame = message.header.frame_id
        for index, track in enumerate(sorted(message.tracks, key=lambda item: item.track_id)):
            transformed = self._transform_point(
                source_frame, message.header.stamp, track.position.x, track.position.y
            )
            if transformed is None:
                continue
            x, y, frame = transformed
            marker_id = index * 3
            sphere = self._base_marker(
                "tracker_position", marker_id, Marker.SPHERE, message.header.stamp, frame
            )
            sphere.pose.position = self._point(x, y, 0.0)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
                0.10,
                1.0,
                0.35,
                0.95,
            )
            markers.append(sphere)

            text = self._base_marker(
                "tracker_id", marker_id, Marker.TEXT_VIEW_FACING, message.header.stamp, frame
            )
            text.pose.position = self._point(x, y, 1.8)
            text.scale.z = 0.28
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"ID: {int(track.track_id)}"
            markers.append(text)

            vx = float(track.velocity.x) if math.isfinite(track.velocity.x) else 0.0
            vy = float(track.velocity.y) if math.isfinite(track.velocity.y) else 0.0
            arrow = self._base_marker(
                "tracker_velocity", marker_id, Marker.ARROW, message.header.stamp, frame
            )
            arrow.points = [
                self._point(x, y, 0.1),
                self._point(x + vx, y + vy, 0.1),
            ]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.14, 0.18
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (
                0.10,
                1.0,
                0.35,
                1.0,
            )
            markers.append(arrow)

    def _detection_markers(
        self, message: PointCloud2, markers: list[Marker]
    ) -> None:
        try:
            detections = read_scored_points(message)
        except (ValueError, struct.error):
            return
        source_frame = message.header.frame_id
        for index, (raw_x, raw_y, _confidence) in enumerate(detections):
            transformed = self._transform_point(
                source_frame, message.header.stamp, raw_x, raw_y
            )
            if transformed is None:
                continue
            x, y, frame = transformed
            marker = self._base_marker(
                "detector", index, Marker.SPHERE, message.header.stamp, frame
            )
            marker.pose.position = self._point(x, y, 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                1.0,
                0.15,
                0.10,
                0.90,
            )
            markers.append(marker)

    def _ground_truth_markers(
        self, message: PedestrianStateArray, markers: list[Marker]
    ) -> None:
        source_frame = message.header.frame_id
        for index, pedestrian in enumerate(message.pedestrians):
            transformed = self._transform_point(
                source_frame,
                message.header.stamp,
                pedestrian.pose.position.x,
                pedestrian.pose.position.y,
            )
            if transformed is None:
                continue
            x, y, frame = transformed
            marker = self._base_marker(
                "ground_truth", index, Marker.SPHERE, message.header.stamp, frame
            )
            marker.pose.position = self._point(x, y, 0.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.40
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.10,
                0.35,
                1.0,
                0.55,
            )
            markers.append(marker)

    def publish_markers(self) -> None:
        now = time.monotonic()
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.display_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = "pedestrian_perception_visualization"
        clear.id = 0
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        marker_list: list[Marker] = []
        if self._is_fresh(self.latest_detections, now):
            self._detection_markers(self.latest_detections[0], marker_list)
        if self._is_fresh(self.latest_tracks, now):
            self._tracker_markers(self.latest_tracks[0], marker_list)
        if self._is_fresh(self.latest_ground_truth, now):
            self._ground_truth_markers(self.latest_ground_truth[0], marker_list)
        array.markers.extend(marker_list)
        self.marker_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PedestrianPerceptionVisualizer()
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
