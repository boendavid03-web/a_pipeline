#!/usr/bin/env python3
"""Evaluate lidar-only pedestrian tracks against timestamp-synchronized GT."""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from semantic_nav_gazebo.msg import PedestrianStateArray, TrackedPedestrianArray

from pedestrian_point_tracker_core import linear_sum_assignment


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def median(values: list[float]) -> float | None:
    return float(np.median(values)) if values else None


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def match_positions(
    tracks: list[dict], ground_truth: list[dict], threshold: float
) -> list[tuple[int, int, float]]:
    if not tracks or not ground_truth:
        return []
    invalid = 1.0e9
    costs = np.full((len(tracks), len(ground_truth)), invalid, dtype=np.float64)
    for row, track in enumerate(tracks):
        for column, truth in enumerate(ground_truth):
            distance = math.hypot(track["x"] - truth["x"], track["y"] - truth["y"])
            if distance <= threshold:
                costs[row, column] = distance + 1.0e-9 * (
                    row * len(ground_truth) + column
                )
    rows, columns = linear_sum_assignment(costs)
    return [
        (row, column, float(costs[row, column]))
        for row, column in zip(rows.tolist(), columns.tolist())
        if costs[row, column] < invalid
    ]


class PedestrianTrackingEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("pedestrian_tracking_evaluator")
        self.track_topic = str(
            self.declare_parameter("track_topic", "/pedestrian_tracks").value
        )
        self.gt_topic = str(
            self.declare_parameter(
                "ground_truth_topic", "/pedestrian_ground_truth"
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
        self.gt_velocity_fit_half_window = float(
            self.declare_parameter("gt_velocity_fit_half_window", 0.30).value
        )
        self.gt_velocity_fit_min_samples = int(
            self.declare_parameter("gt_velocity_fit_min_samples", 5).value
        )
        output_dir = str(self.declare_parameter("output_dir", "").value)
        self.output_dir = Path(
            output_dir or "runs/dr_spaam_isaac_tracking_smoke/latest/evaluation"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.summary_path = self.output_dir / "summary.json"
        self.trace_path = self.output_dir / "tracking_trace.jsonl"
        if (
            self.max_sync_offset <= 0.0
            or self.match_threshold <= 0.0
            or self.gt_velocity_fit_half_window <= 0.0
            or self.gt_velocity_fit_min_samples < 3
        ):
            raise ValueError("sync and match thresholds must be positive")
        self.max_sync_ns = int(round(self.max_sync_offset * 1.0e9))
        self.gt_velocity_fit_half_window_ns = int(
            round(self.gt_velocity_fit_half_window * 1.0e9)
        )
        self.gt_frames: deque[PedestrianStateArray] = deque(maxlen=500)
        self.pending_tracks: deque[TrackedPedestrianArray] = deque(maxlen=500)

        self.track_message_count = 0
        self.gt_message_count = 0
        self.evaluated_frames = 0
        self.gt_observations = 0
        self.track_observations = 0
        self.eligible_track_observations = 0
        self.matched_observations = 0
        self.false_positive_observations = 0
        self.false_negative_observations = 0
        self.position_errors: list[float] = []
        self.vx_errors: list[float] = []
        self.vy_errors: list[float] = []
        self.velocity_vector_errors: list[float] = []
        self.speed_errors: list[float] = []
        self.gt_raw_speeds: list[float] = []
        self.gt_fitted_speeds: list[float] = []
        self.gt_velocity_missing_observations = 0
        self.sync_offsets: list[float] = []
        self.dropped_unsynchronized_frames = 0
        self.dropped_wrong_frame_frames = 0
        self.nonfinite_observations = 0
        self.id_switches = 0
        self.fragmentations = 0
        self.gt_identity_state: dict[str, dict] = {}
        self.track_spans: dict[int, list[int]] = {}
        self.confirmed_track_ids: set[int] = set()

        self.create_subscription(
            TrackedPedestrianArray, self.track_topic, self.tracks_callback, 30
        )
        self.create_subscription(
            PedestrianStateArray, self.gt_topic, self.gt_callback, 30
        )
        self.get_logger().info(
            f"tracking evaluator ready: tracks={self.track_topic}, gt={self.gt_topic}, "
            f"frame={self.target_frame}, sync<={self.max_sync_offset:.3f} s, "
            f"match<={self.match_threshold:.3f} m"
        )

    def gt_callback(self, message: PedestrianStateArray) -> None:
        self.gt_message_count += 1
        self.gt_frames.append(message)
        self._drain_ready()

    def tracks_callback(self, message: TrackedPedestrianArray) -> None:
        self.track_message_count += 1
        self.pending_tracks.append(message)
        self._drain_ready()

    def _nearest_gt(self, timestamp_ns: int):
        if not self.gt_frames:
            return None, None
        candidates = [
            (abs(timestamp_ns - stamp_ns(message.header.stamp)), message)
            for message in self.gt_frames
        ]
        return min(candidates, key=lambda item: item[0])

    def _drain_ready(self, force: bool = False) -> None:
        if not self.gt_frames:
            return
        latest_gt_ns = stamp_ns(self.gt_frames[-1].header.stamp)
        while self.pending_tracks:
            track_frame = self.pending_tracks[0]
            track_ns = stamp_ns(track_frame.header.stamp)
            offset_ns, gt_frame = self._nearest_gt(track_ns)
            ready_ns = track_ns + max(
                self.max_sync_ns, self.gt_velocity_fit_half_window_ns
            )
            if not force and latest_gt_ns < ready_ns:
                break
            self.pending_tracks.popleft()
            if offset_ns is None or offset_ns > self.max_sync_ns:
                self.dropped_unsynchronized_frames += 1
                continue
            self._evaluate(track_frame, gt_frame, offset_ns)
            oldest_needed = track_ns - max(
                self.max_sync_ns, self.gt_velocity_fit_half_window_ns
            )
            while (
                len(self.gt_frames) > 2
                and stamp_ns(self.gt_frames[1].header.stamp) < oldest_needed
            ):
                self.gt_frames.popleft()

    def _fitted_gt_velocity(
        self, identity: str, center_ns: int
    ) -> tuple[float, float] | None:
        samples = []
        for frame in self.gt_frames:
            frame_ns = stamp_ns(frame.header.stamp)
            if abs(frame_ns - center_ns) > self.gt_velocity_fit_half_window_ns:
                continue
            for person in frame.pedestrians:
                if str(person.id) != identity:
                    continue
                x = float(person.pose.position.x)
                y = float(person.pose.position.y)
                if math.isfinite(x) and math.isfinite(y):
                    samples.append(((frame_ns - center_ns) / 1.0e9, x, y))
                break
        if len(samples) < self.gt_velocity_fit_min_samples:
            return None
        times = np.asarray([sample[0] for sample in samples], dtype=np.float64)
        design = np.stack((times, np.ones_like(times)), axis=1)
        positions = np.asarray(
            [[sample[1], sample[2]] for sample in samples], dtype=np.float64
        )
        velocity = np.linalg.lstsq(design, positions, rcond=None)[0][0]
        if velocity.shape != (2,) or not np.isfinite(velocity).all():
            return None
        return float(velocity[0]), float(velocity[1])

    def _evaluate(
        self,
        track_frame: TrackedPedestrianArray,
        gt_frame: PedestrianStateArray,
        offset_ns: int,
    ) -> None:
        track_frame_id = str(track_frame.header.frame_id).lstrip("/")
        gt_frame_id = str(gt_frame.header.frame_id).lstrip("/")
        if track_frame_id != self.target_frame or gt_frame_id != self.target_frame:
            self.dropped_wrong_frame_frames += 1
            return
        timestamp_ns = stamp_ns(track_frame.header.stamp)
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
            values = (
                record["x"], record["y"], record["vx"], record["vy"],
                record["confidence"], record["time_since_update"],
            )
            if not all(math.isfinite(value) for value in values):
                self.nonfinite_observations += 1
                continue
            all_tracks.append(record)
            if record["state"] in {"CONFIRMED", "COASTING"}:
                tracks.append(record)
                self.confirmed_track_ids.add(record["track_id"])
                span = self.track_spans.setdefault(
                    record["track_id"], [timestamp_ns, timestamp_ns]
                )
                span[1] = timestamp_ns
        ground_truth = []
        gt_timestamp_ns = stamp_ns(gt_frame.header.stamp)
        for item in gt_frame.pedestrians:
            fitted_velocity = self._fitted_gt_velocity(str(item.id), gt_timestamp_ns)
            record = {
                "id": str(item.id),
                "x": float(item.pose.position.x),
                "y": float(item.pose.position.y),
                "raw_vx": float(item.velocity.linear.x),
                "raw_vy": float(item.velocity.linear.y),
                "vx": fitted_velocity[0] if fitted_velocity is not None else None,
                "vy": fitted_velocity[1] if fitted_velocity is not None else None,
            }
            if not all(
                math.isfinite(record[name])
                for name in ("x", "y", "raw_vx", "raw_vy")
            ):
                self.nonfinite_observations += 1
                continue
            self.gt_raw_speeds.append(math.hypot(record["raw_vx"], record["raw_vy"]))
            if fitted_velocity is None:
                self.gt_velocity_missing_observations += 1
            else:
                self.gt_fitted_speeds.append(math.hypot(*fitted_velocity))
            ground_truth.append(record)
        matches = match_positions(tracks, ground_truth, self.match_threshold)
        match_by_gt = {gt_index: track_index for track_index, gt_index, _ in matches}

        self.evaluated_frames += 1
        self.gt_observations += len(ground_truth)
        self.track_observations += len(all_tracks)
        self.eligible_track_observations += len(tracks)
        self.matched_observations += len(matches)
        self.false_positive_observations += len(tracks) - len(matches)
        self.false_negative_observations += len(ground_truth) - len(matches)
        self.sync_offsets.append(offset_ns / 1.0e9)
        for track_index, gt_index, distance in matches:
            track = tracks[track_index]
            truth = ground_truth[gt_index]
            if truth["vx"] is None or truth["vy"] is None:
                continue
            vx_error = abs(track["vx"] - truth["vx"])
            vy_error = abs(track["vy"] - truth["vy"])
            vector_error = math.hypot(
                track["vx"] - truth["vx"], track["vy"] - truth["vy"]
            )
            speed_error = abs(
                math.hypot(track["vx"], track["vy"])
                - math.hypot(truth["vx"], truth["vy"])
            )
            self.position_errors.append(float(distance))
            self.vx_errors.append(vx_error)
            self.vy_errors.append(vy_error)
            self.velocity_vector_errors.append(vector_error)
            self.speed_errors.append(speed_error)

        for gt_index, truth in enumerate(ground_truth):
            identity = truth["id"]
            state = self.gt_identity_state.setdefault(
                identity,
                {"ever_matched": False, "currently_matched": False, "last_track": None},
            )
            if gt_index not in match_by_gt:
                if state["ever_matched"]:
                    state["currently_matched"] = False
                continue
            track_id = tracks[match_by_gt[gt_index]]["track_id"]
            if state["ever_matched"] and not state["currently_matched"]:
                self.fragmentations += 1
            if state["last_track"] is not None and state["last_track"] != track_id:
                self.id_switches += 1
            state["ever_matched"] = True
            state["currently_matched"] = True
            state["last_track"] = track_id

        trace = {
            "track_timestamp_ns": timestamp_ns,
            "gt_timestamp_ns": gt_timestamp_ns,
            "timestamp_offset_sec": offset_ns / 1.0e9,
            "frame": self.target_frame,
            "tracks": all_tracks,
            "ground_truth": ground_truth,
            "matches": [
                {
                    "track_id": tracks[track_index]["track_id"],
                    "gt_id": ground_truth[gt_index]["id"],
                    "position_error_m": distance,
                    "velocity_vector_error_mps": (
                        math.hypot(
                            tracks[track_index]["vx"]
                            - ground_truth[gt_index]["vx"],
                            tracks[track_index]["vy"]
                            - ground_truth[gt_index]["vy"],
                        )
                        if ground_truth[gt_index]["vx"] is not None
                        and ground_truth[gt_index]["vy"] is not None
                        else None
                    ),
                }
                for track_index, gt_index, distance in matches
            ],
        }
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(trace, separators=(",", ":")) + "\n")
        self.write_summary()

    def write_summary(self) -> None:
        durations = sorted(
            (last_ns - first_ns) / 1.0e9
            for first_ns, last_ns in self.track_spans.values()
        )
        summary = {
            "schema": "dr_spaam_isaac_tracking_smoke/v1",
            "tracker_input": {
                "topic": "/dr_spaam_detections_scored",
                "message_type": "sensor_msgs/msg/PointCloud2",
                "fields": ["x", "y", "confidence"],
            },
            "tracker_output": {
                "topic": self.track_topic,
                "message_type": "semantic_nav_gazebo/msg/TrackedPedestrianArray",
                "frame": self.target_frame,
                "evaluated_states": ["CONFIRMED", "COASTING"],
            },
            "matching": {
                "method": "hungarian_euclidean_position",
                "threshold_m": self.match_threshold,
                "ground_truth_role": "evaluation_only",
            },
            "time_sync": {
                "method": "nearest_timestamp_buffered",
                "max_accepted_offset_sec": self.max_sync_offset,
                "mean_offset_sec": mean(self.sync_offsets),
                "median_offset_sec": median(self.sync_offsets),
                "maximum_observed_offset_sec": max(self.sync_offsets, default=None),
                "dropped_unsynchronized_frames": self.dropped_unsynchronized_frames,
            },
            "ground_truth_velocity": {
                "source": "short_window_linear_fit_of_PedestrianStateArray_position",
                "fit_half_window_sec": self.gt_velocity_fit_half_window,
                "minimum_samples": self.gt_velocity_fit_min_samples,
                "raw_message_velocity_source": (
                    "adjacent published Isaac character positions divided by elapsed time"
                ),
                "raw_speed_mean_mps": mean(self.gt_raw_speeds),
                "raw_speed_p95_mps": (
                    float(np.percentile(self.gt_raw_speeds, 95))
                    if self.gt_raw_speeds
                    else None
                ),
                "raw_speed_max_mps": max(self.gt_raw_speeds, default=None),
                "fitted_speed_mean_mps": mean(self.gt_fitted_speeds),
                "fitted_speed_p95_mps": (
                    float(np.percentile(self.gt_fitted_speeds, 95))
                    if self.gt_fitted_speeds
                    else None
                ),
                "fitted_speed_max_mps": max(self.gt_fitted_speeds, default=None),
                "missing_fitted_velocity_observations": (
                    self.gt_velocity_missing_observations
                ),
                "used_by_tracker": False,
            },
            "metrics": {
                "evaluated_frames": self.evaluated_frames,
                "gt_observations": self.gt_observations,
                "track_observations_all_states": self.track_observations,
                "track_observations_confirmed_or_coasting": self.eligible_track_observations,
                "matched_tracks": self.matched_observations,
                "false_positive_track_observations": self.false_positive_observations,
                "false_negative_gt_observations": self.false_negative_observations,
                "position_mean_error_m": mean(self.position_errors),
                "position_median_error_m": median(self.position_errors),
                "vx_mae_mps": mean(self.vx_errors),
                "vy_mae_mps": mean(self.vy_errors),
                "velocity_vector_mae_mps": mean(self.velocity_vector_errors),
                "speed_mae_mps": mean(self.speed_errors),
                "id_switches": self.id_switches,
                "fragmentation": self.fragmentations,
                "confirmed_track_count": len(self.confirmed_track_ids),
                "mean_track_duration_sec": mean(durations),
                "median_track_duration_sec": median(durations),
                "finite": self.nonfinite_observations == 0,
            },
            "quality": {
                "track_message_count": self.track_message_count,
                "ground_truth_message_count": self.gt_message_count,
                "dropped_wrong_frame_frames": self.dropped_wrong_frame_frames,
                "nonfinite_observations": self.nonfinite_observations,
                "pending_track_frames": len(self.pending_tracks),
            },
        }
        self.summary_path.write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

    def finalize(self) -> None:
        self._drain_ready(force=True)
        if self.pending_tracks:
            self.dropped_unsynchronized_frames += len(self.pending_tracks)
            self.pending_tracks.clear()
        self.write_summary()


def main() -> None:
    rclpy.init()
    node = PedestrianTrackingEvaluator()
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
