
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .dataset import base_to_map, _rotation
from .model import DecodedDetection


@dataclass(frozen=True)
class MapDetection:
    position_xy_map: np.ndarray
    velocity_xy_map_absolute: np.ndarray
    confidence: float


@dataclass(frozen=True)
class TrackedPedestrian:
    track_id: int
    position_xy_map: np.ndarray
    velocity_xy_map_absolute: np.ndarray
    confidence: float
    track_state: str
    time_since_update_s: float


def detections_base_to_map(
    detections: Sequence[DecodedDetection], robot_pose_map: np.ndarray
) -> List[MapDetection]:
    pose = np.asarray(robot_pose_map, dtype=np.float64).reshape(3)
    if not np.isfinite(pose).all():
        raise ValueError("robot pose must be finite")
    rotation = _rotation(float(pose[2]))
    converted = []
    for detection in detections:
        position = base_to_map(
            np.asarray(detection.position_xy_base).reshape(1, 2), pose
        )[0]
        velocity = (
            np.asarray(detection.velocity_xy_robot_axes_absolute, dtype=np.float64)
            .reshape(1, 2)
            @ rotation.T
        )[0]
        converted.append(
            MapDetection(
                position_xy_map=position,
                velocity_xy_map_absolute=velocity,
                confidence=float(detection.confidence),
            )
        )
    return converted


def linear_sum_assignment(cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rectangular Hungarian assignment without a SciPy runtime dependency."""

    costs = np.asarray(cost_matrix, dtype=np.float64)
    if costs.ndim != 2:
        raise ValueError("cost matrix must be two-dimensional")
    original_rows, original_cols = costs.shape
    if original_rows == 0 or original_cols == 0:
        return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
    transposed = original_rows > original_cols
    if transposed:
        costs = costs.T
    rows, cols = costs.shape
    u = np.zeros(rows + 1, dtype=np.float64)
    v = np.zeros(cols + 1, dtype=np.float64)
    p = np.zeros(cols + 1, dtype=np.int64)
    way = np.zeros(cols + 1, dtype=np.int64)
    for row in range(1, rows + 1):
        p[0] = row
        min_values = np.full(cols + 1, np.inf, dtype=np.float64)
        used = np.zeros(cols + 1, dtype=bool)
        column0 = 0
        while True:
            used[column0] = True
            row0 = p[column0]
            delta = np.inf
            column1 = 0
            for column in range(1, cols + 1):
                if used[column]:
                    continue
                current = costs[row0 - 1, column - 1] - u[row0] - v[column]
                if current < min_values[column]:
                    min_values[column] = current
                    way[column] = column0
                if min_values[column] < delta:
                    delta = min_values[column]
                    column1 = column
            if not np.isfinite(delta):
                raise ValueError("assignment cost matrix contains no finite solution")
            for column in range(cols + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    min_values[column] -= delta
            column0 = column1
            if p[column0] == 0:
                break
        while True:
            column1 = way[column0]
            p[column0] = p[column1]
            column0 = column1
            if column0 == 0:
                break
    row_indices = []
    col_indices = []
    for column in range(1, cols + 1):
        if p[column] != 0:
            row_indices.append(int(p[column] - 1))
            col_indices.append(int(column - 1))
    row_array = np.asarray(row_indices, dtype=np.int64)
    col_array = np.asarray(col_indices, dtype=np.int64)
    order = np.argsort(row_array, kind="stable")
    row_array = row_array[order]
    col_array = col_array[order]
    if transposed:
        return col_array, row_array
    return row_array, col_array


class _Track:
    def __init__(
        self, track_id: int, detection: MapDetection, timestamp_ns: int
    ) -> None:
        self.track_id = int(track_id)
        self.state = np.concatenate(
            (
                np.asarray(detection.position_xy_map, dtype=np.float64),
                np.asarray(detection.velocity_xy_map_absolute, dtype=np.float64),
            )
        )
        self.covariance = np.diag([0.15**2, 0.15**2, 0.5**2, 0.5**2])
        self.created_ns = int(timestamp_ns)
        self.last_predict_ns = int(timestamp_ns)
        self.last_update_ns = int(timestamp_ns)
        self.hit_count = 1
        self.track_state = "TENTATIVE"
        self.confidence = float(detection.confidence)

    def predict(self, timestamp_ns: int, acceleration_sigma: float) -> None:
        dt = max(0.0, (int(timestamp_ns) - self.last_predict_ns) / 1e9)
        transition = np.asarray(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        process = acceleration_sigma**2 * np.asarray(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=np.float64,
        )
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_predict_ns = int(timestamp_ns)

    def update(
        self,
        detection: MapDetection,
        timestamp_ns: int,
        *,
        position_measurement_scale: float,
        velocity_measurement_scale: float,
    ) -> None:
        measurement = np.concatenate(
            (
                np.asarray(detection.position_xy_map, dtype=np.float64),
                np.asarray(detection.velocity_xy_map_absolute, dtype=np.float64),
            )
        )
        confidence = float(np.clip(detection.confidence, 0.05, 1.0))
        measurement_covariance = np.diag(
            [
                (
                    position_measurement_scale
                    * (0.08 + 0.20 * (1.0 - confidence))
                )
                ** 2,
                (
                    position_measurement_scale
                    * (0.08 + 0.20 * (1.0 - confidence))
                )
                ** 2,
                (
                    velocity_measurement_scale
                    * (0.20 + 0.60 * (1.0 - confidence))
                )
                ** 2,
                (
                    velocity_measurement_scale
                    * (0.20 + 0.60 * (1.0 - confidence))
                )
                ** 2,
            ]
        )
        innovation = measurement - self.state
        innovation_covariance = self.covariance + measurement_covariance
        gain = self.covariance @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(4, dtype=np.float64)
        self.covariance = (
            (identity - gain)
            @ self.covariance
            @ (identity - gain).T
            + gain @ measurement_covariance @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_update_ns = int(timestamp_ns)
        self.hit_count += 1
        if self.hit_count >= 3:
            self.track_state = "CONFIRMED"
        self.confidence = float(
            np.clip(0.65 * self.confidence + 0.35 * detection.confidence, 0.0, 1.0)
        )

    def snapshot(self, timestamp_ns: int) -> TrackedPedestrian:
        time_since_update = (int(timestamp_ns) - self.last_update_ns) / 1e9
        return TrackedPedestrian(
            track_id=self.track_id,
            position_xy_map=self.state[:2].copy(),
            velocity_xy_map_absolute=self.state[2:].copy(),
            confidence=float(self.confidence),
            track_state=self.track_state,
            time_since_update_s=float(time_since_update),
        )


class PedestrianTracker:
    def __init__(
        self,
        *,
        position_gate_m: float = 0.5,
        velocity_gate_mps: float = 1.5,
        tentative_timeout_s: float = 0.33,
        confirmed_timeout_s: float = 1.0,
        acceleration_sigma_mps2: float = 3.0,
        position_measurement_scale: float = 0.75,
        velocity_measurement_scale: float = 2.0,
        association_velocity_weight: float = 0.4,
    ) -> None:
        self.position_gate_m = float(position_gate_m)
        self.velocity_gate_mps = float(velocity_gate_mps)
        self.tentative_timeout_s = float(tentative_timeout_s)
        self.confirmed_timeout_s = float(confirmed_timeout_s)
        self.acceleration_sigma_mps2 = float(acceleration_sigma_mps2)
        self.position_measurement_scale = float(position_measurement_scale)
        self.velocity_measurement_scale = float(velocity_measurement_scale)
        self.association_velocity_weight = float(association_velocity_weight)
        if self.position_measurement_scale <= 0.0:
            raise ValueError("position_measurement_scale must be positive")
        if self.velocity_measurement_scale <= 0.0:
            raise ValueError("velocity_measurement_scale must be positive")
        if self.association_velocity_weight < 0.0:
            raise ValueError("association_velocity_weight cannot be negative")
        self.tracks: List[_Track] = []
        self.next_track_id = 1
        self.last_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.tracks = []
        self.next_track_id = 1
        self.last_timestamp_ns = None

    def update(
        self, detections: Sequence[MapDetection], timestamp_ns: int
    ) -> List[TrackedPedestrian]:
        timestamp_ns = int(timestamp_ns)
        if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
            raise ValueError("tracker timestamps must be strictly increasing")
        self.last_timestamp_ns = timestamp_ns
        detections = list(detections)
        for detection in detections:
            position = np.asarray(detection.position_xy_map, dtype=np.float64)
            velocity = np.asarray(
                detection.velocity_xy_map_absolute, dtype=np.float64
            )
            if (
                position.shape != (2,)
                or velocity.shape != (2,)
                or not np.isfinite(position).all()
                or not np.isfinite(velocity).all()
                or not math.isfinite(float(detection.confidence))
            ):
                raise ValueError("tracker detections must be finite 2D states")
        detections.sort(
            key=lambda item: (
                -float(item.confidence),
                float(item.position_xy_map[0]),
                float(item.position_xy_map[1]),
                float(item.velocity_xy_map_absolute[0]),
                float(item.velocity_xy_map_absolute[1]),
            )
        )
        for track in self.tracks:
            track.predict(timestamp_ns, self.acceleration_sigma_mps2)

        matched_tracks = set()
        matched_detections = set()
        if self.tracks and detections:
            invalid = 1e9
            costs = np.full((len(self.tracks), len(detections)), invalid)
            for row, track in enumerate(self.tracks):
                for col, detection in enumerate(detections):
                    position_distance = float(
                        np.linalg.norm(
                            track.state[:2]
                            - np.asarray(detection.position_xy_map, dtype=np.float64)
                        )
                    )
                    velocity_distance = float(
                        np.linalg.norm(
                            track.state[2:]
                            - np.asarray(
                                detection.velocity_xy_map_absolute,
                                dtype=np.float64,
                            )
                        )
                    )
                    if (
                        position_distance <= self.position_gate_m
                        and velocity_distance <= self.velocity_gate_mps
                    ):
                        costs[row, col] = (
                            position_distance
                            + self.association_velocity_weight
                            * velocity_distance
                            - 0.05 * float(detection.confidence)
                            + 1e-9 * (row * len(detections) + col)
                        )
            rows, cols = linear_sum_assignment(costs)
            for row, col in zip(rows.tolist(), cols.tolist()):
                if costs[row, col] >= invalid:
                    continue
                self.tracks[row].update(
                    detections[col],
                    timestamp_ns,
                    position_measurement_scale=self.position_measurement_scale,
                    velocity_measurement_scale=self.velocity_measurement_scale,
                )
                matched_tracks.add(row)
                matched_detections.add(col)

        for col, detection in enumerate(detections):
            if col in matched_detections:
                continue
            self.tracks.append(
                _Track(self.next_track_id, detection, timestamp_ns)
            )
            self.next_track_id += 1

        survivors = []
        for row, track in enumerate(self.tracks):
            time_since_update = (timestamp_ns - track.last_update_ns) / 1e9
            if row not in matched_tracks and time_since_update > 0.0:
                track.track_state = (
                    "COASTING"
                    if track.track_state in {"CONFIRMED", "COASTING"}
                    else "TENTATIVE"
                )
                track.confidence *= math.exp(
                    -time_since_update
                    / (
                        self.confirmed_timeout_s
                        if track.track_state == "COASTING"
                        else self.tentative_timeout_s
                    )
                )
            timeout = (
                self.confirmed_timeout_s
                if track.track_state == "COASTING"
                else self.tentative_timeout_s
            )
            if time_since_update <= timeout:
                survivors.append(track)
        self.tracks = survivors
        return [
            track.snapshot(timestamp_ns)
            for track in sorted(self.tracks, key=lambda item: item.track_id)
        ]
