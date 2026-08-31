#!/usr/bin/env python3
"""Evaluate crowded DR-SPAAM detections and point tracks without tuning either.

Frames are synchronized on the tracker/detector scan timestamp.  Ground truth
is used only by this evaluator, never by the detector or tracker.  The output
keeps detector misses/merges separate from association-native identity errors.
"""

from __future__ import annotations

import csv
import json
import math
import struct
import sys
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from semantic_nav_gazebo.msg import PedestrianStateArray, TrackedPedestrianArray
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from pedestrian_point_tracker_core import linear_sum_assignment
from pedestrian_crowded_tracking_analysis_core import (
    assign_scan_support,
    compute_static_line_of_sight,
    validate_world_scene_contract,
)


DISTANCE_BINS = (
    (">=1.50", 1.50, math.inf),
    ("1.00-1.50", 1.00, 1.50),
    ("0.75-1.00", 0.75, 1.00),
    ("0.50-0.75", 0.50, 0.75),
    ("<0.50", -math.inf, 0.50),
)
# v1 label retained for immutable artifact cross-checks; v2 uses the explicit
# closed lower bound ``>=1.50`` in the analysis core.
LEGACY_DISTANCE_BIN_LABEL = ">1.50"


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def distance_bin(distance: float) -> str:
    for label, low, high in DISTANCE_BINS:
        if low <= distance < high:
            return label
    raise ValueError(f"non-finite inter-person distance: {distance}")


def match_positions(
    observations: list[dict], truths: list[dict], threshold: float
) -> list[tuple[int, int, float]]:
    if not observations or not truths:
        return []
    invalid = 1.0e9
    costs = np.full((len(observations), len(truths)), invalid, dtype=np.float64)
    for row, observation in enumerate(observations):
        for column, truth in enumerate(truths):
            distance = math.hypot(
                observation["x"] - truth["x"], observation["y"] - truth["y"]
            )
            if distance <= threshold:
                costs[row, column] = distance + 1.0e-9 * (
                    row * len(truths) + column
                )
    rows, columns = linear_sum_assignment(costs)
    return [
        (int(row), int(column), float(costs[row, column]))
        for row, column in zip(rows.tolist(), columns.tolist())
        if costs[row, column] < invalid
    ]


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


def read_scored_detections(message: PointCloud2) -> list[dict]:
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "confidence")
    if any(name not in fields for name in required):
        raise ValueError("scored detections lack x/y/confidence fields")
    if any(fields[name].datatype != PointField.FLOAT32 for name in required):
        raise ValueError("scored detection fields must use FLOAT32")
    endian = ">" if message.is_bigendian else "<"
    detections = []
    for row in range(int(message.height)):
        for column in range(int(message.width)):
            base = row * int(message.row_step) + column * int(message.point_step)
            values = {
                name: float(
                    struct.unpack_from(
                        endian + "f", message.data, base + int(fields[name].offset)
                    )[0]
                )
                for name in required
            }
            if all(math.isfinite(value) for value in values.values()):
                detections.append(values)
    return detections


def pairwise_truth_distances(truths: list[dict]) -> list[dict]:
    pairs = []
    for index, first in enumerate(truths):
        for second in truths[index + 1 :]:
            pairs.append(
                {
                    "ids": [first["id"], second["id"]],
                    "distance_m": math.hypot(
                        first["x"] - second["x"], first["y"] - second["y"]
                    ),
                    "midpoint": [
                        0.5 * (first["x"] + second["x"]),
                        0.5 * (first["y"] + second["y"]),
                    ],
                }
            )
    return pairs


class CrowdedTrackingEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("pedestrian_crowded_tracking_evaluator")
        self.scenario = str(self.declare_parameter("scenario", "C").value).upper()
        self.stress_ids = tuple(
            str(value)
            for value in self.declare_parameter(
                "stress_ids", ["stress_a", "stress_b"]
            ).value
        )
        self.target_frame = str(
            self.declare_parameter("target_frame", "odom").value
        ).lstrip("/")
        self.max_sync_offset = float(
            self.declare_parameter("max_sync_offset", 0.08).value
        )
        self.match_threshold = float(
            self.declare_parameter("match_threshold", 0.5).value
        )
        self.visible_distance = float(
            self.declare_parameter("visible_distance", 8.0).value
        )
        self.close_distance = float(
            self.declare_parameter("close_encounter_distance", 1.5).value
        )
        self.crossing_window = float(
            self.declare_parameter("crossing_window", 1.0).value
        )
        self.failure_lookback = float(
            self.declare_parameter("failure_lookback", 0.5).value
        )
        self.requested_spacing = float(
            self.declare_parameter("requested_spacing", -1.0).value
        )
        project_root = Path(__file__).resolve().parents[5]
        self.world_contract = validate_world_scene_contract(
            project_root / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world",
            project_root / "isaac_sim/scenes/a_pipeline_eng_lobby.usda",
        )
        self.static_boxes = self._load_static_boxes() if self.world_contract["valid"] else []
        output_dir = str(self.declare_parameter("output_dir", "").value)
        self.output_dir = Path(
            output_dir or "runs/dr_spaam_isaac_crowded_tracking/latest/evaluation"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.summary_path = self.output_dir / "summary.json"
        self.trace_path = self.output_dir / "crowded_tracking_trace.jsonl"
        self.crossing_trace_path = self.output_dir / "crossing_trace.jsonl"
        self.trajectory_path = self.output_dir / "ground_truth_trajectory.csv"
        if (
            not self.stress_ids
            or not self.target_frame
            or min(
                self.max_sync_offset,
                self.match_threshold,
                self.visible_distance,
                self.close_distance,
                self.crossing_window,
                self.failure_lookback,
            )
            <= 0.0
        ):
            raise ValueError("crowded evaluator parameters must be positive")
        self.max_sync_ns = int(round(self.max_sync_offset * 1.0e9))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gt_frames: deque[PedestrianStateArray] = deque(maxlen=500)
        self.odom_frames: deque[Odometry] = deque(maxlen=500)
        self.pending_tracks: deque[TrackedPedestrianArray] = deque(maxlen=500)
        self.detections_by_stamp: dict[int, tuple[str, list[dict]]] = {}
        self.scans_by_stamp: dict[int, LaserScan] = {}
        self.source_scans_by_topic: dict[str, dict[int, LaserScan]] = {
            "/scan_01": {}, "/scan_02": {}
        }
        self.clock_stamps: list[int] = []
        self.clock_regressions = 0
        self.tf_static_received = False
        self.evaluated_before_tf_static_frames = 0
        self.evaluated_before_clock_frames = 0
        self.scan_stamps: dict[str, list[int]] = {
            "scan_01": [], "scan_02": [], "scan_merged": []
        }
        self.scan_beam_counts: dict[str, set[int]] = {
            "scan_01": set(), "scan_02": set(), "scan_merged": set()
        }
        self.latest_detection_ns: int | None = None

        self.frames: list[dict] = []
        self.identity_state: dict[str, dict] = {}
        self.id_events: list[dict] = []
        self.unique_gt_ids: set[str] = set()
        self.confirmed_track_ids: set[int] = set()
        self.trajectory: dict[str, list[tuple[int, float, float]]] = {}
        self.detector_tp = 0
        self.detector_fp = 0
        self.detector_fn = 0
        self.track_matches = 0
        self.continuous_id_switches = 0
        self.reacquisition_id_changes = 0
        self.fragmentations = 0
        self.detector_induced_failures = 0
        self.tracker_native_failures = 0
        self.two_gt_one_detection_frames = 0
        self.two_gt_two_detection_frames = 0
        self.detector_merge_events = 0
        self.detector_split_events = 0
        self.previous_merge_frame = False
        self.cmd_vel_publisher_count = None
        self.dropped_unsynchronized_frames = 0
        self.dropped_wrong_frame_frames = 0
        self.dropped_tf_frames = 0
        self.malformed_detection_frames = 0
        self.distance_metrics = {
            label: {
                "frames": 0,
                "gt_observations": 0,
                "detector_tp": 0,
                "track_matches": 0,
                "continuous_id_switches": 0,
            }
            for label, _, _ in DISTANCE_BINS
        }
        self.topic_counts = {
            "scan_merged": 0,
            "scan_01": 0,
            "scan_02": 0,
            "detections": 0,
            "tracks": 0,
            "ground_truth": 0,
            "odom": 0,
        }

        self.create_subscription(
            LaserScan,
            "/scan_merged",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        for topic in ("/scan_01", "/scan_02"):
            self.create_subscription(
                LaserScan, topic,
                lambda message, topic=topic: self.source_scan_callback(topic, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(Clock, "/clock", self.clock_callback, qos_profile_sensor_data)
        tf_static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self.tf_static_callback, tf_static_qos)
        self.create_subscription(
            PointCloud2, "/dr_spaam_detections_scored", self.detection_callback, 30
        )
        self.create_subscription(
            TrackedPedestrianArray, "/pedestrian_tracks", self.track_callback, 30
        )
        self.create_subscription(
            PedestrianStateArray,
            "/pedestrian_ground_truth",
            self.gt_callback,
            30,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 30)
        self.get_logger().info(
            f"CROWDED_EVALUATOR_READY scenario={self.scenario}, "
            f"stress_ids={list(self.stress_ids)}, match={self.match_threshold:.2f} m"
        )

    @staticmethod
    def _nearest(frames, timestamp_ns: int):
        if not frames:
            return None, None
        return min(
            (
                (abs(timestamp_ns - stamp_ns(message.header.stamp)), message)
                for message in frames
            ),
            key=lambda item: item[0],
        )

    def scan_callback(self, message: LaserScan) -> None:
        self.topic_counts["scan_merged"] += 1
        timestamp_ns = stamp_ns(message.header.stamp)
        self.scan_stamps["scan_merged"].append(timestamp_ns)
        self.scan_beam_counts["scan_merged"].add(len(message.ranges))
        self.scans_by_stamp[timestamp_ns] = message
        self._trim_timestamp_dict(self.scans_by_stamp)

    def source_scan_callback(self, topic: str, message: LaserScan) -> None:
        name = topic.lstrip("/")
        self.topic_counts[name] += 1
        self.scan_stamps[name].append(stamp_ns(message.header.stamp))
        self.scan_beam_counts[name].add(len(message.ranges))
        values = self.source_scans_by_topic[topic]
        values[stamp_ns(message.header.stamp)] = message
        self._trim_timestamp_dict(values)

    def clock_callback(self, message: Clock) -> None:
        timestamp_ns = stamp_ns(message.clock)
        if self.clock_stamps and timestamp_ns < self.clock_stamps[-1]:
            self.clock_regressions += 1
        self.clock_stamps.append(timestamp_ns)
        if len(self.clock_stamps) > 1000:
            del self.clock_stamps[:-1000]

    def tf_static_callback(self, _message: TFMessage) -> None:
        self.tf_static_received = True

    def detection_callback(self, message: PointCloud2) -> None:
        self.topic_counts["detections"] += 1
        timestamp_ns = stamp_ns(message.header.stamp)
        try:
            detections = read_scored_detections(message)
        except (ValueError, struct.error):
            self.malformed_detection_frames += 1
            return
        self.detections_by_stamp[timestamp_ns] = (
            str(message.header.frame_id).lstrip("/"),
            detections,
        )
        self.latest_detection_ns = timestamp_ns
        self._trim_timestamp_dict(self.detections_by_stamp)
        self._drain()

    def track_callback(self, message: TrackedPedestrianArray) -> None:
        self.topic_counts["tracks"] += 1
        self.pending_tracks.append(message)
        self._drain()

    def gt_callback(self, message: PedestrianStateArray) -> None:
        self.topic_counts["ground_truth"] += 1
        self.gt_frames.append(message)
        self._drain()

    def odom_callback(self, message: Odometry) -> None:
        self.topic_counts["odom"] += 1
        self.odom_frames.append(message)

    @staticmethod
    def _trim_timestamp_dict(values: dict, keep: int = 500) -> None:
        if len(values) > keep:
            for key in sorted(values)[:-keep]:
                values.pop(key, None)

    def _transform_records(
        self, records: list[dict], source_frame: str, timestamp
    ) -> list[dict]:
        if source_frame == self.target_frame:
            return [dict(record) for record in records]
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            Time.from_msg(timestamp),
            timeout=Duration(seconds=0.05),
        )
        transformed = []
        for record in records:
            x, y = transform_xy(transform, record["x"], record["y"])
            transformed.append({**record, "x": x, "y": y})
        return transformed

    def _drain(self, force: bool = False) -> None:
        if not self.gt_frames:
            return
        latest_gt_ns = stamp_ns(self.gt_frames[-1].header.stamp)
        while self.pending_tracks:
            track_frame = self.pending_tracks[0]
            timestamp_ns = stamp_ns(track_frame.header.stamp)
            if not force and latest_gt_ns < timestamp_ns + self.max_sync_ns:
                return
            if timestamp_ns not in self.detections_by_stamp:
                if not force and (
                    self.latest_detection_ns is None
                    or self.latest_detection_ns <= timestamp_ns
                ):
                    return
                self.pending_tracks.popleft()
                self.dropped_unsynchronized_frames += 1
                continue
            gt_offset_ns, gt_frame = self._nearest(self.gt_frames, timestamp_ns)
            if gt_offset_ns is None or gt_offset_ns > self.max_sync_ns:
                if not force and latest_gt_ns <= timestamp_ns + self.max_sync_ns:
                    return
                self.pending_tracks.popleft()
                self.dropped_unsynchronized_frames += 1
                continue
            self.pending_tracks.popleft()
            source_frame, detections = self.detections_by_stamp.pop(timestamp_ns)
            self._evaluate(track_frame, gt_frame, source_frame, detections)

    def _robot_position(self, timestamp_ns: int) -> tuple[float, float] | None:
        offset_ns, odom = self._nearest(self.odom_frames, timestamp_ns)
        if odom is None or offset_ns is None or offset_ns > self.max_sync_ns:
            return None
        return float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)

    def _sensor_origins(self, timestamp) -> dict[str, list[float] | None]:
        origins = {}
        for name, frame in (("scan_01", "base_scan_01"), ("scan_02", "base_scan_02")):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, frame, Time.from_msg(timestamp), timeout=Duration(seconds=0.05)
                )
                translation = transform.transform.translation
                origins[name] = [float(translation.x), float(translation.y), float(translation.z)]
            except TransformException:
                origins[name] = None
        return origins

    @staticmethod
    def _load_static_boxes() -> list[dict]:
        """Load the project converter's boxes for post-hoc LOS only.

        Failure is deliberately represented by an empty list: the trace then
        records TF_OR_WORLD_UNKNOWN and the offline validity gate rejects it.
        """
        project_root = Path(__file__).resolve().parents[5]
        world = project_root / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
        converter_dir = project_root / "isaac_sim/scripts"
        if not world.is_file():
            world = Path.cwd() / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
            converter_dir = Path.cwd() / "isaac_sim/scripts"
        try:
            sys.path.insert(0, str(converter_dir))
            from convert_gazebo_boxes_to_usda import load_static_boxes
            boxes, _ = load_static_boxes(world)
            return [{"pose": {"x": box.pose.x, "y": box.pose.y, "z": box.pose.z, "yaw": box.pose.yaw}, "size": list(box.size)} for box in boxes]
        except (ImportError, OSError, ValueError):
            return []

    def _scan_points(self, timestamp_ns: int, timestamp) -> list[list[float]]:
        scan = self.scans_by_stamp.get(timestamp_ns)
        if scan is None:
            return []
        records = []
        for index, value in enumerate(scan.ranges):
            distance = float(value)
            if not math.isfinite(distance) or distance > self.visible_distance:
                continue
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            records.append(
                {"x": distance * math.cos(angle), "y": distance * math.sin(angle)}
            )
        try:
            transformed = self._transform_records(
                records, str(scan.header.frame_id).lstrip("/"), timestamp
            )
        except TransformException:
            self.dropped_tf_frames += 1
            return []
        return [[record["x"], record["y"]] for record in transformed]

    def _classify_identity_event(self, timestamp_ns: int) -> str:
        start_ns = timestamp_ns - int(round(self.failure_lookback * 1.0e9))
        history = [
            frame
            for frame in self.frames
            if start_ns <= frame["timestamp_ns"] <= timestamp_ns
        ]
        detector_continuous = bool(history) and all(
            frame["detector_tp"] == frame["expected_visible_gt_count"]
            and not frame["merge_frame"]
            for frame in history
        )
        return (
            "TRACKER_ASSOCIATION_FAILURE"
            if detector_continuous
            else "DETECTOR_MERGE_INDUCED_TRACK_BREAK"
        )

    def _evaluate(
        self,
        track_frame: TrackedPedestrianArray,
        gt_frame: PedestrianStateArray,
        detection_frame: str,
        raw_detections: list[dict],
    ) -> None:
        timestamp_ns = stamp_ns(track_frame.header.stamp)
        if not self.tf_static_received:
            self.evaluated_before_tf_static_frames += 1
        if not self.clock_stamps:
            self.evaluated_before_clock_frames += 1
        if (
            str(track_frame.header.frame_id).lstrip("/") != self.target_frame
            or str(gt_frame.header.frame_id).lstrip("/") != self.target_frame
        ):
            self.dropped_wrong_frame_frames += 1
            return
        try:
            detections = self._transform_records(
                raw_detections, detection_frame, track_frame.header.stamp
            )
        except TransformException:
            self.dropped_tf_frames += 1
            return
        robot = self._robot_position(timestamp_ns)
        truths = []
        for item in gt_frame.pedestrians:
            identity = str(item.id)
            if identity not in self.stress_ids:
                continue
            record = {
                "id": identity,
                "x": float(item.pose.position.x),
                "y": float(item.pose.position.y),
                "vx": float(item.velocity.linear.x),
                "vy": float(item.velocity.linear.y),
            }
            if not all(math.isfinite(record[key]) for key in ("x", "y", "vx", "vy")):
                continue
            if robot is not None and math.hypot(
                record["x"] - robot[0], record["y"] - robot[1]
            ) > self.visible_distance:
                continue
            truths.append(record)
            self.unique_gt_ids.add(identity)
            self.trajectory.setdefault(identity, []).append(
                (timestamp_ns, record["x"], record["y"])
            )
        if robot is not None:
            detections = [
                detection
                for detection in detections
                if math.hypot(detection["x"] - robot[0], detection["y"] - robot[1])
                <= self.visible_distance
            ]
        tracks = []
        all_tracks = []
        for item in track_frame.tracks:
            record = {
                "track_id": int(item.track_id),
                "x": float(item.position.x),
                "y": float(item.position.y),
                "vx": float(item.velocity.x),
                "vy": float(item.velocity.y),
                "confidence": float(item.confidence),
                "state": str(item.state).upper(),
                "hits": int(item.hits),
                "misses": int(item.misses),
                "time_since_update": float(item.time_since_update),
            }
            if not all(
                math.isfinite(record[key])
                for key in ("x", "y", "vx", "vy", "confidence", "time_since_update")
            ):
                continue
            all_tracks.append(record)
            if record["state"] in {"CONFIRMED", "COASTING"}:
                tracks.append(record)
                self.confirmed_track_ids.add(record["track_id"])

        detector_matches = match_positions(detections, truths, self.match_threshold)
        track_matches = match_positions(tracks, truths, self.match_threshold)
        detector_by_gt = {
            gt_index: detection_index
            for detection_index, gt_index, _ in detector_matches
        }
        track_by_gt = {
            gt_index: track_index for track_index, gt_index, _ in track_matches
        }
        pairs = pairwise_truth_distances(truths)
        minimum_distance = min(
            (pair["distance_m"] for pair in pairs), default=None
        )
        merge_pairs = []
        for pair in pairs:
            if pair["distance_m"] >= self.close_distance:
                continue
            radius = 0.5 * pair["distance_m"] + self.match_threshold
            local_detections = [
                index
                for index, detection in enumerate(detections)
                if math.hypot(
                    detection["x"] - pair["midpoint"][0],
                    detection["y"] - pair["midpoint"][1],
                )
                <= radius
            ]
            if len(local_detections) == 1:
                merge_pairs.append({**pair, "detection_index": local_detections[0]})
        merge_frame = bool(merge_pairs)
        if merge_frame and not self.previous_merge_frame:
            self.detector_merge_events += 1
        if not merge_frame and self.previous_merge_frame:
            self.detector_split_events += 1
        self.previous_merge_frame = merge_frame
        if len(truths) == 2:
            relevant_detection_count = sum(
                any(
                    math.hypot(
                        detection["x"] - truth["x"],
                        detection["y"] - truth["y"],
                    )
                    <= self.close_distance
                    for truth in truths
                )
                for detection in detections
            )
            if relevant_detection_count == 1:
                self.two_gt_one_detection_frames += 1
            if len(detector_matches) == 2:
                self.two_gt_two_detection_frames += 1

        detector_tp = len(detector_matches)
        detector_fp = len(detections) - detector_tp
        detector_fn = len(truths) - detector_tp
        self.detector_tp += detector_tp
        self.detector_fp += detector_fp
        self.detector_fn += detector_fn
        self.track_matches += len(track_matches)

        identity_matches = []
        frame_id_events = []
        for gt_index, truth in enumerate(truths):
            identity = truth["id"]
            state = self.identity_state.setdefault(
                identity,
                {
                    "ever_matched": False,
                    "currently_matched": False,
                    "last_track": None,
                    "initial_track": None,
                    "final_track": None,
                },
            )
            if gt_index not in track_by_gt:
                if state["ever_matched"]:
                    state["currently_matched"] = False
                continue
            track = tracks[track_by_gt[gt_index]]
            track_id = track["track_id"]
            event_kind = None
            if state["ever_matched"] and not state["currently_matched"]:
                self.fragmentations += 1
                event_kind = "fragmentation"
            if state["last_track"] is not None and state["last_track"] != track_id:
                if state["currently_matched"]:
                    self.continuous_id_switches += 1
                    event_kind = "continuous_id_switch"
                else:
                    self.reacquisition_id_changes += 1
                    event_kind = "reacquisition_id_change"
            if event_kind is not None:
                classification = self._classify_identity_event(timestamp_ns)
                event = {
                    "timestamp_ns": timestamp_ns,
                    "gt_id": identity,
                    "event": event_kind,
                    "previous_track_id": state["last_track"],
                    "current_track_id": track_id,
                    "classification": classification,
                }
                frame_id_events.append(event)
                self.id_events.append(event)
                if classification == "TRACKER_ASSOCIATION_FAILURE":
                    self.tracker_native_failures += 1
                else:
                    self.detector_induced_failures += 1
            if state["initial_track"] is None:
                state["initial_track"] = track_id
            state["ever_matched"] = True
            state["currently_matched"] = True
            state["last_track"] = track_id
            state["final_track"] = track_id
            identity_matches.append(
                {
                    "gt_id": identity,
                    "track_id": track_id,
                    "position_error_m": next(
                        distance
                        for track_index, match_gt_index, distance in track_matches
                        if match_gt_index == gt_index
                    ),
                }
            )

        if minimum_distance is not None:
            label = distance_bin(minimum_distance)
            bucket = self.distance_metrics[label]
            bucket["frames"] += 1
            bucket["gt_observations"] += len(truths)
            bucket["detector_tp"] += detector_tp
            bucket["track_matches"] += len(track_matches)
            bucket["continuous_id_switches"] += sum(
                event["event"] == "continuous_id_switch" for event in frame_id_events
            )

        scan_points = self._scan_points(timestamp_ns, track_frame.header.stamp)
        scan_support = assign_scan_support(scan_points, truths)
        sensor_origins = self._sensor_origins(track_frame.header.stamp)
        observability = {}
        for truth in truths:
            identity = truth["id"]
            in_roi = robot is None or math.hypot(truth["x"] - robot[0], truth["y"] - robot[1]) <= self.visible_distance
            los = (
                compute_static_line_of_sight(
                    sensor_origins, (truth["x"], truth["y"]), self.static_boxes
                )
                if self.world_contract["valid"]
                and self.static_boxes
                and all(origin is not None for origin in sensor_origins.values())
                else None
            )
            supported = bool(scan_support.get(identity, {}).get("supported", False))
            if not in_roi:
                reason = "OUT_OF_ROI"
            elif los is None:
                reason = "TF_OR_WORLD_UNKNOWN"
            elif los is False:
                reason = "STATIC_OCCLUDED"
            elif not supported:
                reason = "NO_SCAN_SUPPORT"
            else:
                reason = "OBSERVABLE"
            observability[identity] = {
                "in_roi": in_roi,
                "scan_support": supported,
                "line_of_sight": los,
                "observable": bool(in_roi and supported and los is True),
                "reason": reason,
            }
        source_stamps = {
            "scan_01": timestamp_ns if timestamp_ns in self.source_scans_by_topic["/scan_01"] else None,
            "scan_02": timestamp_ns if timestamp_ns in self.source_scans_by_topic["/scan_02"] else None,
            "merged": timestamp_ns if timestamp_ns in self.scans_by_stamp else None,
            "detections": timestamp_ns,
            "tracks": timestamp_ns,
        }
        trace = {
            "schema": "pedestrian_crowded_tracking_trace/v2",
            "legacy_schema": "pedestrian_crowded_tracking_trace/v1",
            "timestamp_ns": timestamp_ns,
            "clock": {
                "stamp_ns": self.clock_stamps[-1] if self.clock_stamps else None,
                "nondecreasing": all(a <= b for a, b in zip(self.clock_stamps, self.clock_stamps[1:])),
            },
            "replay_sequence": len(self.frames),
            "source_stamps": source_stamps,
            "sensor_origins": sensor_origins,
            "scenario": self.scenario,
            "frame": self.target_frame,
            "robot": list(robot) if robot is not None else None,
            "expected_visible_gt_count": len(truths),
            "ground_truth": truths,
            "pairwise_gt": pairs,
            "minimum_inter_person_distance_m": minimum_distance,
            "detections": detections,
            "tracks": all_tracks,
            "gt_detection_matches": [
                {
                    "gt_id": truths[gt_index]["id"],
                    "detection_index": detection_index,
                    "position_error_m": distance,
                }
                for detection_index, gt_index, distance in detector_matches
            ],
            "gt_track_matches": identity_matches,
            "detector_tp": detector_tp,
            "detector_fp": detector_fp,
            "detector_fn": detector_fn,
            "merge_frame": merge_frame,
            "merge_pairs": merge_pairs,
            "id_events": frame_id_events,
            "scan_points": scan_points,
            "scan_support": scan_support,
            "observability": observability,
        }
        self.frames.append(trace)
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({k: v for k, v in trace.items() if k != "scan_points"}, separators=(",", ":")) + "\n")
        if minimum_distance is not None and minimum_distance <= self.close_distance:
            with self.crossing_trace_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({k: v for k, v in trace.items() if k != "scan_points"}, separators=(",", ":")) + "\n")
        self.write_summary()

    def _trajectory_summary(self) -> dict:
        result = {}
        for identity, samples in sorted(self.trajectory.items()):
            path_length = sum(
                math.hypot(current[1] - previous[1], current[2] - previous[2])
                for previous, current in zip(samples, samples[1:])
            )
            result[identity] = {
                "samples": len(samples),
                "start_xy_m": list(samples[0][1:]),
                "end_xy_m": list(samples[-1][1:]),
                "path_length_m": path_length,
                "x_range_m": [min(value[1] for value in samples), max(value[1] for value in samples)],
                "y_range_m": [min(value[2] for value in samples), max(value[2] for value in samples)],
            }
        return result

    def write_summary(self, keyframes: list[str] | None = None) -> None:
        first_ns = self.frames[0]["timestamp_ns"] if self.frames else None
        last_ns = self.frames[-1]["timestamp_ns"] if self.frames else None
        minimum_frame = min(
            (
                frame
                for frame in self.frames
                if frame["minimum_inter_person_distance_m"] is not None
            ),
            key=lambda frame: frame["minimum_inter_person_distance_m"],
            default=None,
        )
        crossing_ns = minimum_frame["timestamp_ns"] if minimum_frame else None
        crossing_window_ns = int(round(self.crossing_window * 1.0e9))
        crossing_switches = sum(
            event["event"] == "continuous_id_switch"
            and crossing_ns is not None
            and abs(event["timestamp_ns"] - crossing_ns) <= crossing_window_ns
            for event in self.id_events
        )
        distance_summary = {}
        for label, metrics in self.distance_metrics.items():
            gt_count = metrics["gt_observations"]
            distance_summary[label] = {
                **metrics,
                "detector_recall": metrics["detector_tp"] / gt_count if gt_count else None,
                "track_coverage": metrics["track_matches"] / gt_count if gt_count else None,
                "detector_separation_success": (
                    metrics["detector_tp"] == gt_count if gt_count else None
                ),
                "track_id_stable": (
                    metrics["continuous_id_switches"] == 0 if metrics["frames"] else None
                ),
            }
        identity_recovery = {
            identity: {
                "initial_track_id": state["initial_track"],
                "final_track_id": state["final_track"],
                "final_matches_initial": (
                    state["initial_track"] == state["final_track"]
                    if state["initial_track"] is not None and state["final_track"] is not None
                    else None
                ),
            }
            for identity, state in sorted(self.identity_state.items())
        }
        def observed_rate(stamps: list[int]) -> float | None:
            if len(stamps) < 2 or stamps[-1] <= stamps[0]:
                return None
            return (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1.0e9)

        summary = {
            "schema": "isaac_crowded_tracking_evaluation/v2",
            "legacy_schema": "isaac_crowded_tracking_evaluation/v1",
            "scenario": self.scenario,
            "stress_ids": list(self.stress_ids),
            "requested_spacing_m": (
                self.requested_spacing if self.requested_spacing >= 0.0 else None
            ),
            "duration_sec": (
                (last_ns - first_ns) / 1.0e9
                if first_ns is not None and last_ns is not None
                else 0.0
            ),
            "actual_pedestrian_count": len(self.unique_gt_ids),
            "actual_trajectories": self._trajectory_summary(),
            "mean_inter_person_distance_m": mean(
                [
                    frame["minimum_inter_person_distance_m"]
                    for frame in self.frames
                    if frame["minimum_inter_person_distance_m"] is not None
                ]
            ),
            "minimum_inter_person_distance_m": (
                minimum_frame["minimum_inter_person_distance_m"]
                if minimum_frame is not None
                else None
            ),
            "crossing_timestamp_ns": crossing_ns,
            "detector": {
                "tp": self.detector_tp,
                "fp": self.detector_fp,
                "fn": self.detector_fn,
                "precision": (
                    self.detector_tp / (self.detector_tp + self.detector_fp)
                    if self.detector_tp + self.detector_fp
                    else None
                ),
                "recall": (
                    self.detector_tp / (self.detector_tp + self.detector_fn)
                    if self.detector_tp + self.detector_fn
                    else None
                ),
                "two_gt_one_detection_frames": self.two_gt_one_detection_frames,
                "two_gt_two_detection_frames": self.two_gt_two_detection_frames,
                "detector_merge_events": self.detector_merge_events,
                "detector_split_events": self.detector_split_events,
            },
            "tracker": {
                "confirmed_track_count": len(self.confirmed_track_ids),
                "matched_observations": self.track_matches,
                "continuous_id_switches": self.continuous_id_switches,
                "crossing_id_switches": crossing_switches,
                "reacquisition_id_changes": self.reacquisition_id_changes,
                "fragmentations": self.fragmentations,
                "detector_induced_failures": self.detector_induced_failures,
                "tracker_native_association_failures": self.tracker_native_failures,
                "identity_recovery": identity_recovery,
            },
            "distance_conditioned": distance_summary,
            "contracts": {
                "detector_match_threshold_m": self.match_threshold,
                "visible_distance_m": self.visible_distance,
                "tracking_frame": self.target_frame,
                "ground_truth_used_by_detector_or_tracker": False,
                "velocity_is_primary_gate": False,
                "trace_schema": "pedestrian_crowded_tracking_trace/v2",
                "legacy_trace_schema": "pedestrian_crowded_tracking_trace/v1",
            },
            "replay_contract": {
                "authoritative": "raw_bag_replay",
                "use_sim_time": bool(self.get_parameter("use_sim_time").value),
                "tf_static_received": self.tf_static_received,
                "clock_received": bool(self.clock_stamps),
                "clock_monotonic": self.clock_regressions == 0,
                "world_scene_contract": self.world_contract,
                "required_exact_source_stamps": ["scan_01", "scan_02", "merged", "detections", "tracks"],
                "beams_per_sensor": 2000,
                "acceptable_scan_rate_hz": [13.5, 16.5],
            },
            "quality": {
                "evaluated_frames": len(self.frames),
                "topic_message_counts": self.topic_counts,
                "dropped_unsynchronized_frames": self.dropped_unsynchronized_frames,
                "dropped_wrong_frame_frames": self.dropped_wrong_frame_frames,
                "dropped_tf_frames": self.dropped_tf_frames,
                "malformed_detection_frames": self.malformed_detection_frames,
                "pending_track_frames": len(self.pending_tracks),
                "keyframes": keyframes or [],
                "clock_stamps": len(self.clock_stamps),
                "clock_regressions": self.clock_regressions,
                "evaluated_before_tf_static_frames": self.evaluated_before_tf_static_frames,
                "evaluated_before_clock_frames": self.evaluated_before_clock_frames,
                "scan_beam_counts": {
                    name: sorted(values) for name, values in self.scan_beam_counts.items()
                },
                "scan_observed_rate_hz": {
                    name: observed_rate(values) for name, values in self.scan_stamps.items()
                },
                "exact_source_stamp_counters": {
                    name: sum(frame.get("source_stamps", {}).get(name) == frame["timestamp_ns"] for frame in self.frames)
                    for name in ("scan_01", "scan_02", "merged", "detections", "tracks")
                },
            },
        }
        self.summary_path.write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

    def _write_trajectory_csv(self) -> None:
        with self.trajectory_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("timestamp_ns", "pedestrian_id", "x_m", "y_m"))
            for identity, samples in sorted(self.trajectory.items()):
                for timestamp_ns, x, y in samples:
                    writer.writerow((timestamp_ns, identity, f"{x:.9f}", f"{y:.9f}"))

    def _render_keyframes(self) -> list[str]:
        candidates = [
            frame
            for frame in self.frames
            if frame["minimum_inter_person_distance_m"] is not None
        ]
        if not candidates:
            return []
        minimum = min(candidates, key=lambda frame: frame["minimum_inter_person_distance_m"])
        target_times = {
            "before": minimum["timestamp_ns"] - int(round(self.crossing_window * 1.0e9)),
            "minimum": minimum["timestamp_ns"],
            "after": minimum["timestamp_ns"] + int(round(self.crossing_window * 1.0e9)),
        }
        selected = {
            label: min(candidates, key=lambda frame: abs(frame["timestamp_ns"] - target))
            for label, target in target_times.items()
        }
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning("matplotlib unavailable; keyframes not rendered")
            return []
        keyframe_dir = self.output_dir / "keyframes"
        keyframe_dir.mkdir(parents=True, exist_ok=True)
        outputs = []
        for label, frame in selected.items():
            figure, axis = plt.subplots(figsize=(8, 8), dpi=150)
            scan = np.asarray(frame["scan_points"], dtype=float)
            if scan.size:
                axis.scatter(scan[:, 0], scan[:, 1], s=2, c="#777777", alpha=0.45, label="LiDAR")
            if frame["robot"] is not None:
                axis.scatter(*frame["robot"], s=90, c="#1f77b4", marker="o", label="robot")
            for index, truth in enumerate(frame["ground_truth"]):
                axis.scatter(truth["x"], truth["y"], s=120, facecolors="none", edgecolors="#2ca02c", linewidths=2, label="GT" if index == 0 else None)
                axis.text(truth["x"] + 0.05, truth["y"] + 0.05, f"GT {truth['id']}", color="#207520")
            for index, detection in enumerate(frame["detections"]):
                axis.scatter(detection["x"], detection["y"], s=90, c="#d62728", marker="x", linewidths=2, label="DR-SPAAM" if index == 0 else None)
            for index, track in enumerate(frame["tracks"]):
                axis.scatter(track["x"], track["y"], s=70, facecolors="none", edgecolors="#9467bd", marker="s", linewidths=1.5, label="track" if index == 0 else None)
                axis.text(track["x"] + 0.05, track["y"] - 0.12, f"T{track['track_id']} {track['state']}", color="#6b4385", fontsize=8)
            minimum_distance = frame["minimum_inter_person_distance_m"]
            axis.set_title(
                f"Scenario {self.scenario} {label}: t={frame['timestamp_ns'] / 1e9:.3f}s, "
                f"min GT distance={minimum_distance:.3f} m\n"
                f"det TP/FP/FN={frame['detector_tp']}/{frame['detector_fp']}/{frame['detector_fn']}, merge={frame['merge_frame']}"
            )
            axis.set_xlabel("odom x (m)")
            axis.set_ylabel("odom y (m)")
            axis.set_aspect("equal", adjustable="box")
            axis.grid(True, alpha=0.25)
            axis.legend(loc="best")
            if frame["robot"] is not None:
                axis.set_xlim(frame["robot"][0] - self.visible_distance, frame["robot"][0] + self.visible_distance)
                axis.set_ylim(frame["robot"][1] - self.visible_distance, frame["robot"][1] + self.visible_distance)
            output = keyframe_dir / f"scenario_{self.scenario}_{label}.png"
            figure.tight_layout()
            figure.savefig(output)
            plt.close(figure)
            outputs.append(str(output))
        return outputs

    def finalize(self) -> None:
        self._drain(force=True)
        if self.pending_tracks:
            self.dropped_unsynchronized_frames += len(self.pending_tracks)
            self.pending_tracks.clear()
        self._write_trajectory_csv()
        keyframes = self._render_keyframes()
        self.write_summary(keyframes)


def main() -> None:
    rclpy.init()
    node = CrowdedTrackingEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
