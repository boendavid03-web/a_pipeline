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
from std_msgs.msg import String

from pedestrian_point_tracker_core import linear_sum_assignment


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def median(values: list[float]) -> float | None:
    return float(np.median(values)) if values else None


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def percentile(values: list[float], value: float) -> float | None:
    return float(np.percentile(values, value)) if values else None


def new_velocity_metrics() -> dict:
    return {
        "eligible": 0,
        "valid": 0,
        "vx_errors": [],
        "vy_errors": [],
        "vector_errors": [],
        "speed_errors": [],
        "direction_cosines": [],
    }


def summarize_velocity_metrics(metrics: dict) -> dict:
    eligible = int(metrics["eligible"])
    valid = int(metrics["valid"])
    return {
        "eligible_observations": eligible,
        "valid_observations": valid,
        "valid_ratio": valid / eligible if eligible else None,
        "vx_mae_mps": mean(metrics["vx_errors"]),
        "vy_mae_mps": mean(metrics["vy_errors"]),
        "velocity_vector_mae_mps": mean(metrics["vector_errors"]),
        "velocity_vector_median_error_mps": median(metrics["vector_errors"]),
        "velocity_vector_p90_error_mps": percentile(
            metrics["vector_errors"], 90
        ),
        "speed_mae_mps": mean(metrics["speed_errors"]),
        "direction_cosine_mean": mean(metrics["direction_cosines"]),
        "direction_cosine_median": median(metrics["direction_cosines"]),
        "direction_cosine_observations": len(metrics["direction_cosines"]),
    }


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
        self.velocity_diagnostics_topic = str(
            self.declare_parameter(
                "velocity_diagnostics_topic",
                "/pedestrian_track_velocity_diagnostics",
            ).value
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
        self.gt_velocity_fit_half_windows = tuple(
            float(value)
            for value in self.declare_parameter(
                "gt_velocity_fit_half_windows", [0.20, 0.30, 0.40]
            ).value
        )
        self.gt_velocity_fit_min_samples = int(
            self.declare_parameter("gt_velocity_fit_min_samples", 5).value
        )
        self.direction_min_speed = float(
            self.declare_parameter("direction_min_speed", 0.05).value
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
            or not self.gt_velocity_fit_half_windows
            or min(self.gt_velocity_fit_half_windows) <= 0.0
            or self.gt_velocity_fit_min_samples < 3
            or self.direction_min_speed < 0.0
        ):
            raise ValueError("sync and match thresholds must be positive")
        if not any(
            math.isclose(value, self.gt_velocity_fit_half_window, abs_tol=1.0e-9)
            for value in self.gt_velocity_fit_half_windows
        ):
            raise ValueError(
                "gt_velocity_fit_half_window must appear in "
                "gt_velocity_fit_half_windows"
            )
        self.max_sync_ns = int(round(self.max_sync_offset * 1.0e9))
        self.gt_velocity_fit_window_ns = {
            value: int(round(value * 1.0e9))
            for value in self.gt_velocity_fit_half_windows
        }
        self.max_gt_velocity_fit_half_window_ns = max(
            self.gt_velocity_fit_window_ns.values()
        )
        self.gt_frames: deque[PedestrianStateArray] = deque(maxlen=500)
        self.pending_tracks: deque[TrackedPedestrianArray] = deque(maxlen=500)
        self.velocity_diagnostics: dict[int, dict] = {}
        self.latest_velocity_diagnostics_ns: int | None = None

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
        self.velocity_metrics = {
            half_window: {
                method: new_velocity_metrics()
                for method in ("kalman", "fit5", "fit8")
            }
            for half_window in self.gt_velocity_fit_half_windows
        }
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
        self.continuous_id_switches = 0
        self.reacquisition_id_changes = 0
        self.velocity_diagnostics_message_count = 0
        self.velocity_diagnostics_invalid_count = 0
        self.velocity_diagnostics_missing_frames = 0
        self.kalman_diagnostics_mismatches = 0
        self.detection_count = 0

        self.create_subscription(
            TrackedPedestrianArray, self.track_topic, self.tracks_callback, 30
        )
        self.create_subscription(
            String,
            self.velocity_diagnostics_topic,
            self.velocity_diagnostics_callback,
            30,
        )
        self.create_subscription(
            PedestrianStateArray, self.gt_topic, self.gt_callback, 30
        )
        self.get_logger().info(
            f"tracking evaluator ready: tracks={self.track_topic}, gt={self.gt_topic}, "
            f"velocity_diagnostics={self.velocity_diagnostics_topic}, "
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

    def velocity_diagnostics_callback(self, message: String) -> None:
        self.velocity_diagnostics_message_count += 1
        try:
            diagnostics = json.loads(message.data)
            timestamp_ns = int(diagnostics["timestamp_ns"])
            if (
                diagnostics.get("schema")
                != "pedestrian_track_velocity_diagnostics/v1"
                or timestamp_ns <= 0
                or not isinstance(diagnostics.get("tracks"), list)
            ):
                raise ValueError("invalid velocity diagnostics contract")
            self.detection_count += int(diagnostics.get("detection_count", 0))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.velocity_diagnostics_invalid_count += 1
            return
        self.velocity_diagnostics[timestamp_ns] = diagnostics
        self.latest_velocity_diagnostics_ns = timestamp_ns
        if len(self.velocity_diagnostics) > 600:
            for old_timestamp_ns in sorted(self.velocity_diagnostics)[:-500]:
                self.velocity_diagnostics.pop(old_timestamp_ns, None)
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
                self.max_sync_ns, self.max_gt_velocity_fit_half_window_ns
            )
            if not force and latest_gt_ns < ready_ns:
                break
            diagnostics_ready = track_ns in self.velocity_diagnostics or (
                self.latest_velocity_diagnostics_ns is not None
                and self.latest_velocity_diagnostics_ns > track_ns
            )
            if not force and not diagnostics_ready:
                break
            self.pending_tracks.popleft()
            if offset_ns is None or offset_ns > self.max_sync_ns:
                self.dropped_unsynchronized_frames += 1
                continue
            velocity_diagnostics = self.velocity_diagnostics.pop(track_ns, None)
            if velocity_diagnostics is None:
                self.velocity_diagnostics_missing_frames += 1
            self._evaluate(
                track_frame, gt_frame, offset_ns, velocity_diagnostics
            )
            oldest_needed = track_ns - max(
                self.max_sync_ns, self.max_gt_velocity_fit_half_window_ns
            )
            while (
                len(self.gt_frames) > 2
                and stamp_ns(self.gt_frames[1].header.stamp) < oldest_needed
            ):
                self.gt_frames.popleft()

    def _fitted_gt_velocity(
        self, identity: str, center_ns: int, half_window: float
    ) -> tuple[float, float] | None:
        samples = []
        half_window_ns = self.gt_velocity_fit_window_ns[half_window]
        for frame in self.gt_frames:
            frame_ns = stamp_ns(frame.header.stamp)
            if abs(frame_ns - center_ns) > half_window_ns:
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

    def _record_velocity(
        self,
        metrics: dict,
        estimate: dict,
        truth_velocity: tuple[float, float],
    ) -> dict | None:
        metrics["eligible"] += 1
        if not estimate["valid"]:
            return None
        vx = estimate["vx"]
        vy = estimate["vy"]
        if vx is None or vy is None or not all(
            math.isfinite(value) for value in (vx, vy)
        ):
            self.nonfinite_observations += 1
            return None
        truth_vx, truth_vy = truth_velocity
        vx_error = abs(vx - truth_vx)
        vy_error = abs(vy - truth_vy)
        vector_error = math.hypot(vx - truth_vx, vy - truth_vy)
        estimated_speed = math.hypot(vx, vy)
        truth_speed = math.hypot(truth_vx, truth_vy)
        speed_error = abs(estimated_speed - truth_speed)
        metrics["valid"] += 1
        metrics["vx_errors"].append(vx_error)
        metrics["vy_errors"].append(vy_error)
        metrics["vector_errors"].append(vector_error)
        metrics["speed_errors"].append(speed_error)
        direction_cosine = None
        if (
            estimated_speed >= self.direction_min_speed
            and truth_speed >= self.direction_min_speed
        ):
            direction_cosine = float(
                np.clip(
                    (vx * truth_vx + vy * truth_vy)
                    / (estimated_speed * truth_speed),
                    -1.0,
                    1.0,
                )
            )
            metrics["direction_cosines"].append(direction_cosine)
        return {
            "vx_error_mps": vx_error,
            "vy_error_mps": vy_error,
            "velocity_vector_error_mps": vector_error,
            "speed_error_mps": speed_error,
            "direction_cosine": direction_cosine,
        }

    def _evaluate(
        self,
        track_frame: TrackedPedestrianArray,
        gt_frame: PedestrianStateArray,
        offset_ns: int,
        velocity_diagnostics: dict | None,
    ) -> None:
        track_frame_id = str(track_frame.header.frame_id).lstrip("/")
        gt_frame_id = str(gt_frame.header.frame_id).lstrip("/")
        if track_frame_id != self.target_frame or gt_frame_id != self.target_frame:
            self.dropped_wrong_frame_frames += 1
            return
        timestamp_ns = stamp_ns(track_frame.header.stamp)
        diagnostics_by_track = {}
        if velocity_diagnostics is not None:
            if str(velocity_diagnostics.get("frame", "")).lstrip("/") != self.target_frame:
                self.velocity_diagnostics_invalid_count += 1
            else:
                for record in velocity_diagnostics["tracks"]:
                    try:
                        diagnostics_by_track[int(record["track_id"])] = record
                    except (KeyError, TypeError, ValueError):
                        self.velocity_diagnostics_invalid_count += 1
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
            record["velocity_estimators"] = {
                "kalman": {
                    "vx": record["vx"],
                    "vy": record["vy"],
                    "valid": True,
                },
                "fit5": {"vx": None, "vy": None, "valid": False},
                "fit8": {"vx": None, "vy": None, "valid": False},
            }
            diagnostic = diagnostics_by_track.get(record["track_id"])
            if diagnostic is not None:
                try:
                    diagnostic_kalman = (
                        float(diagnostic["kalman_vx"]),
                        float(diagnostic["kalman_vy"]),
                    )
                    if not np.allclose(
                        diagnostic_kalman,
                        (record["vx"], record["vy"]),
                        rtol=0.0,
                        atol=1.0e-9,
                    ):
                        self.kalman_diagnostics_mismatches += 1
                    for method in ("fit5", "fit8"):
                        fit = diagnostic[method]
                        valid = bool(fit["valid"])
                        fit_record = dict(fit)
                        fit_record["valid"] = valid
                        if valid:
                            fit_record["vx"] = float(fit["vx"])
                            fit_record["vy"] = float(fit["vy"])
                        else:
                            fit_record["vx"] = None
                            fit_record["vy"] = None
                        record["velocity_estimators"][method] = fit_record
                except (KeyError, TypeError, ValueError):
                    self.velocity_diagnostics_invalid_count += 1
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
            fitted_velocities = {
                f"{half_window:.2f}": self._fitted_gt_velocity(
                    str(item.id), gt_timestamp_ns, half_window
                )
                for half_window in self.gt_velocity_fit_half_windows
            }
            fitted_velocity = fitted_velocities[
                f"{self.gt_velocity_fit_half_window:.2f}"
            ]
            record = {
                "id": str(item.id),
                "x": float(item.pose.position.x),
                "y": float(item.pose.position.y),
                "raw_vx": float(item.velocity.linear.x),
                "raw_vy": float(item.velocity.linear.y),
                "vx": fitted_velocity[0] if fitted_velocity is not None else None,
                "vy": fitted_velocity[1] if fitted_velocity is not None else None,
                "velocity_by_half_window": {
                    key: (
                        {"vx": value[0], "vy": value[1]}
                        if value is not None
                        else None
                    )
                    for key, value in fitted_velocities.items()
                },
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
        trace_matches = []
        for track_index, gt_index, distance in matches:
            track = tracks[track_index]
            truth = ground_truth[gt_index]
            self.position_errors.append(float(distance))
            velocity_errors = {}
            for half_window in self.gt_velocity_fit_half_windows:
                window_key = f"{half_window:.2f}"
                truth_record = truth["velocity_by_half_window"][window_key]
                if truth_record is None:
                    continue
                truth_velocity = (truth_record["vx"], truth_record["vy"])
                velocity_errors[window_key] = {
                    method: self._record_velocity(
                        self.velocity_metrics[half_window][method],
                        track["velocity_estimators"][method],
                        truth_velocity,
                    )
                    for method in ("kalman", "fit5", "fit8")
                }
            trace_matches.append(
                {
                    "track_id": track["track_id"],
                    "gt_id": truth["id"],
                    "position_error_m": distance,
                    "velocity_errors_by_half_window": velocity_errors,
                }
            )

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
                if state["currently_matched"]:
                    self.continuous_id_switches += 1
                else:
                    self.reacquisition_id_changes += 1
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
            "matches": trace_matches,
        }
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(trace, separators=(",", ":")) + "\n")
        self.write_summary()

    def write_summary(self) -> None:
        durations = sorted(
            (last_ns - first_ns) / 1.0e9
            for first_ns, last_ns in self.track_spans.values()
        )
        velocity_ab = {
            f"{half_window:.2f}": {
                method: summarize_velocity_metrics(
                    self.velocity_metrics[half_window][method]
                )
                for method in ("kalman", "fit5", "fit8")
            }
            for half_window in self.gt_velocity_fit_half_windows
        }
        primary_velocity = velocity_ab[f"{self.gt_velocity_fit_half_window:.2f}"][
            "kalman"
        ]
        summary = {
            "schema": "dr_spaam_isaac_tracking_smoke/v2",
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
                "public_velocity_source": "point_cv_kalman_state_vx_vy",
                "velocity_diagnostics": {
                    "topic": self.velocity_diagnostics_topic,
                    "message_type": "std_msgs/msg/String JSON",
                    "schema": "pedestrian_track_velocity_diagnostics/v1",
                    "public_message_modified": False,
                },
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
                "sensitivity_half_windows_sec": list(
                    self.gt_velocity_fit_half_windows
                ),
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
            "velocity_ab": velocity_ab,
            "metrics": {
                "evaluated_frames": self.evaluated_frames,
                "gt_observations": self.gt_observations,
                "track_observations_all_states": self.track_observations,
                "track_observations_confirmed_or_coasting": self.eligible_track_observations,
                "matched_tracks": self.matched_observations,
                "track_coverage": (
                    self.matched_observations / self.gt_observations
                    if self.gt_observations
                    else None
                ),
                "false_positive_track_observations": self.false_positive_observations,
                "false_negative_gt_observations": self.false_negative_observations,
                "position_mean_error_m": mean(self.position_errors),
                "position_median_error_m": median(self.position_errors),
                "vx_mae_mps": primary_velocity["vx_mae_mps"],
                "vy_mae_mps": primary_velocity["vy_mae_mps"],
                "velocity_vector_mae_mps": primary_velocity[
                    "velocity_vector_mae_mps"
                ],
                "velocity_vector_median_error_mps": primary_velocity[
                    "velocity_vector_median_error_mps"
                ],
                "velocity_vector_p90_error_mps": primary_velocity[
                    "velocity_vector_p90_error_mps"
                ],
                "speed_mae_mps": primary_velocity["speed_mae_mps"],
                "direction_cosine_mean": primary_velocity[
                    "direction_cosine_mean"
                ],
                "direction_cosine_median": primary_velocity[
                    "direction_cosine_median"
                ],
                "id_switches": self.id_switches,
                "continuous_id_switches": self.continuous_id_switches,
                "reacquisition_id_changes": self.reacquisition_id_changes,
                "fragmentation": self.fragmentations,
                "confirmed_track_count": len(self.confirmed_track_ids),
                "mean_track_duration_sec": mean(durations),
                "median_track_duration_sec": median(durations),
                "finite": self.nonfinite_observations == 0,
            },
            "quality": {
                "track_message_count": self.track_message_count,
                "ground_truth_message_count": self.gt_message_count,
                "velocity_diagnostics_message_count": (
                    self.velocity_diagnostics_message_count
                ),
                "velocity_diagnostics_invalid_count": (
                    self.velocity_diagnostics_invalid_count
                ),
                "velocity_diagnostics_missing_frames": (
                    self.velocity_diagnostics_missing_frames
                ),
                "kalman_diagnostics_mismatches": (
                    self.kalman_diagnostics_mismatches
                ),
                "detection_count": self.detection_count,
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
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
