#!/usr/bin/env python3
"""Point-target constant-velocity Kalman multi-object tracking."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class PointDetection:
    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class MeasurementSample:
    timestamp_ns: int
    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class CausalVelocityFit:
    vx: float | None
    vy: float | None
    valid: bool
    samples: int
    time_span: float
    fit_rmse: float | None
    mean_detection_confidence: float | None


@dataclass(frozen=True)
class TrackEstimate:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    age: int
    hits: int
    misses: int
    state: str
    time_since_update: float
    fit5: CausalVelocityFit
    fit8: CausalVelocityFit


def causal_linear_velocity_fit(
    measurements: Sequence[MeasurementSample],
    *,
    window_size: int,
    min_samples: int = 3,
    min_time_span: float = 0.15,
) -> CausalVelocityFit:
    """Fit x(t), y(t) using only the newest available real measurements."""

    if window_size < 1:
        raise ValueError("window_size must be positive")
    if min_samples < 2 or min_samples > window_size:
        raise ValueError("min_samples must be in [2, window_size]")
    if min_time_span <= 0.0:
        raise ValueError("min_time_span must be positive")
    selected = list(measurements)[-window_size:]
    sample_count = len(selected)
    if sample_count == 0:
        return CausalVelocityFit(None, None, False, 0, 0.0, None, None)
    timestamps = np.asarray(
        [sample.timestamp_ns for sample in selected], dtype=np.int64
    )
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError("measurement timestamps must be strictly increasing")
    values = np.asarray(
        [
            [sample.x, sample.y, sample.confidence]
            for sample in selected
        ],
        dtype=np.float64,
    )
    if not np.isfinite(values).all():
        raise ValueError("measurements must contain finite x/y/confidence")
    time_span = float((timestamps[-1] - timestamps[0]) / 1.0e9)
    mean_confidence = float(np.mean(values[:, 2]))
    if sample_count < min_samples or time_span < min_time_span:
        return CausalVelocityFit(
            None,
            None,
            False,
            sample_count,
            time_span,
            None,
            mean_confidence,
        )
    times = (timestamps - timestamps[-1]).astype(np.float64) / 1.0e9
    design = np.stack((times, np.ones_like(times)), axis=1)
    velocity_and_intercept = np.linalg.lstsq(
        design, values[:, :2], rcond=None
    )[0]
    velocity = velocity_and_intercept[0]
    fitted_positions = design @ velocity_and_intercept
    residuals = values[:, :2] - fitted_positions
    fit_rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if (
        velocity.shape != (2,)
        or not np.isfinite(velocity).all()
        or not math.isfinite(fit_rmse)
    ):
        return CausalVelocityFit(
            None,
            None,
            False,
            sample_count,
            time_span,
            None,
            mean_confidence,
        )
    return CausalVelocityFit(
        float(velocity[0]),
        float(velocity[1]),
        True,
        sample_count,
        time_span,
        fit_rmse,
        mean_confidence,
    )


def linear_sum_assignment(cost_matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rectangular Hungarian assignment without a SciPy runtime dependency."""

    costs = np.asarray(cost_matrix, dtype=np.float64)
    if costs.ndim != 2:
        raise ValueError("cost matrix must be two-dimensional")
    original_rows, original_cols = costs.shape
    if original_rows == 0 or original_cols == 0:
        return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
    if not np.isfinite(costs).all():
        raise ValueError("cost matrix must be finite")
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
        minimum = np.full(cols + 1, np.inf, dtype=np.float64)
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
                if current < minimum[column]:
                    minimum[column] = current
                    way[column] = column0
                if minimum[column] < delta:
                    delta = minimum[column]
                    column1 = column
            if not math.isfinite(float(delta)):
                raise ValueError("cost matrix has no finite assignment")
            for column in range(cols + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    minimum[column] -= delta
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
    column_indices = []
    for column in range(1, cols + 1):
        if p[column] != 0:
            row_indices.append(int(p[column] - 1))
            column_indices.append(int(column - 1))
    row_array = np.asarray(row_indices, dtype=np.int64)
    column_array = np.asarray(column_indices, dtype=np.int64)
    order = np.argsort(row_array, kind="stable")
    row_array = row_array[order]
    column_array = column_array[order]
    if transposed:
        return column_array, row_array
    return row_array, column_array


class _Track:
    def __init__(
        self,
        track_id: int,
        detection: PointDetection,
        timestamp_ns: int,
        initial_position_sigma: float,
        initial_velocity_sigma: float,
        measurement_history_size: int,
        velocity_fit_min_samples: int,
        velocity_fit_min_span: float,
    ) -> None:
        self.track_id = int(track_id)
        self.state = np.asarray(
            [detection.x, detection.y, 0.0, 0.0], dtype=np.float64
        )
        self.covariance = np.diag(
            [
                initial_position_sigma**2,
                initial_position_sigma**2,
                initial_velocity_sigma**2,
                initial_velocity_sigma**2,
            ]
        )
        self.created_ns = int(timestamp_ns)
        self.last_predict_ns = int(timestamp_ns)
        self.last_update_ns = int(timestamp_ns)
        self.confidence = float(detection.confidence)
        self.age = 1
        self.hits = 1
        self.consecutive_hits = 1
        self.misses = 0
        self.confirmed = False
        self.measurement_history: deque[MeasurementSample] = deque(
            [
                MeasurementSample(
                    int(timestamp_ns),
                    float(detection.x),
                    float(detection.y),
                    float(detection.confidence),
                )
            ],
            maxlen=measurement_history_size,
        )
        self.velocity_fit_min_samples = int(velocity_fit_min_samples)
        self.velocity_fit_min_span = float(velocity_fit_min_span)

    def predict(
        self,
        timestamp_ns: int,
        acceleration_sigma: float,
        max_prediction_dt: float,
    ) -> None:
        elapsed = (int(timestamp_ns) - self.last_predict_ns) / 1.0e9
        dt = min(max(0.0, elapsed), max_prediction_dt)
        transition = np.asarray(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dt2, dt3, dt4 = dt * dt, dt**3, dt**4
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
        self.age += 1

    def update(
        self,
        detection: PointDetection,
        timestamp_ns: int,
        measurement_sigma: float,
        confidence_alpha: float,
        min_hits: int,
    ) -> None:
        measurement = np.asarray([detection.x, detection.y], dtype=np.float64)
        observation = np.asarray(
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]],
            dtype=np.float64,
        )
        measurement_covariance = np.eye(2, dtype=np.float64) * measurement_sigma**2
        innovation = measurement - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T
            + measurement_covariance
        )
        gain = (
            self.covariance
            @ observation.T
            @ np.linalg.inv(innovation_covariance)
        )
        self.state = self.state + gain @ innovation
        identity = np.eye(4, dtype=np.float64)
        residual = identity - gain @ observation
        self.covariance = (
            residual @ self.covariance @ residual.T
            + gain @ measurement_covariance @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_update_ns = int(timestamp_ns)
        self.hits += 1
        self.consecutive_hits += 1
        self.misses = 0
        self.confirmed = self.confirmed or self.consecutive_hits >= min_hits
        self.confidence = float(
            np.clip(
                (1.0 - confidence_alpha) * self.confidence
                + confidence_alpha * detection.confidence,
                0.0,
                1.0,
            )
        )
        self.measurement_history.append(
            MeasurementSample(
                int(timestamp_ns),
                float(detection.x),
                float(detection.y),
                float(detection.confidence),
            )
        )

    def miss(self) -> None:
        self.misses += 1
        self.consecutive_hits = 0

    def snapshot(self, timestamp_ns: int) -> TrackEstimate:
        time_since_update = max(
            0.0, (int(timestamp_ns) - self.last_update_ns) / 1.0e9
        )
        state = (
            "COASTING"
            if self.confirmed and self.misses > 0
            else "CONFIRMED"
            if self.confirmed
            else "TENTATIVE"
        )
        fit5 = causal_linear_velocity_fit(
            self.measurement_history,
            window_size=5,
            min_samples=self.velocity_fit_min_samples,
            min_time_span=self.velocity_fit_min_span,
        )
        fit8 = causal_linear_velocity_fit(
            self.measurement_history,
            window_size=8,
            min_samples=self.velocity_fit_min_samples,
            min_time_span=self.velocity_fit_min_span,
        )
        return TrackEstimate(
            track_id=self.track_id,
            x=float(self.state[0]),
            y=float(self.state[1]),
            vx=float(self.state[2]),
            vy=float(self.state[3]),
            confidence=float(self.confidence),
            age=self.age,
            hits=self.hits,
            misses=self.misses,
            state=state,
            time_since_update=time_since_update,
            fit5=fit5,
            fit8=fit8,
        )


class PointCVKalmanTracker:
    """Hungarian-associated 2D point tracker with a four-state CV Kalman filter."""

    def __init__(
        self,
        *,
        association_threshold: float = 0.8,
        min_hits: int = 3,
        max_age: int = 8,
        max_coast_time: float = 0.75,
        acceleration_sigma: float = 2.0,
        measurement_sigma: float = 0.10,
        initial_position_sigma: float = 0.20,
        initial_velocity_sigma: float = 1.0,
        max_prediction_dt: float = 0.50,
        confidence_alpha: float = 0.35,
        measurement_history_size: int = 8,
        velocity_fit_min_samples: int = 3,
        velocity_fit_min_span: float = 0.15,
    ) -> None:
        self.association_threshold = float(association_threshold)
        self.min_hits = int(min_hits)
        self.max_age = int(max_age)
        self.max_coast_time = float(max_coast_time)
        self.acceleration_sigma = float(acceleration_sigma)
        self.measurement_sigma = float(measurement_sigma)
        self.initial_position_sigma = float(initial_position_sigma)
        self.initial_velocity_sigma = float(initial_velocity_sigma)
        self.max_prediction_dt = float(max_prediction_dt)
        self.confidence_alpha = float(confidence_alpha)
        self.measurement_history_size = int(measurement_history_size)
        self.velocity_fit_min_samples = int(velocity_fit_min_samples)
        self.velocity_fit_min_span = float(velocity_fit_min_span)
        if self.association_threshold <= 0.0:
            raise ValueError("association_threshold must be positive")
        if self.min_hits < 1 or self.max_age < 0:
            raise ValueError("min_hits must be positive and max_age non-negative")
        if min(
            self.max_coast_time,
            self.acceleration_sigma,
            self.measurement_sigma,
            self.initial_position_sigma,
            self.initial_velocity_sigma,
            self.max_prediction_dt,
        ) <= 0.0:
            raise ValueError("tracker noise and time parameters must be positive")
        if not 0.0 <= self.confidence_alpha <= 1.0:
            raise ValueError("confidence_alpha must be in [0,1]")
        if self.measurement_history_size < 8:
            raise ValueError("measurement_history_size must be at least 8")
        if not 2 <= self.velocity_fit_min_samples <= 5:
            raise ValueError("velocity_fit_min_samples must be in [2,5]")
        if self.velocity_fit_min_span <= 0.0:
            raise ValueError("velocity_fit_min_span must be positive")
        self.tracks: list[_Track] = []
        self.next_track_id = 1
        self.last_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.tracks = []
        self.next_track_id = 1
        self.last_timestamp_ns = None

    def update(
        self,
        detections: Sequence[PointDetection],
        timestamp_ns: int,
    ) -> list[TrackEstimate]:
        timestamp_ns = int(timestamp_ns)
        if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
            raise ValueError("tracker timestamps must be strictly increasing")
        self.last_timestamp_ns = timestamp_ns
        detections = list(detections)
        for detection in detections:
            values = (detection.x, detection.y, detection.confidence)
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("detections must contain finite x/y/confidence")
            if not 0.0 <= float(detection.confidence) <= 1.0:
                raise ValueError("detection confidence must be in [0,1]")
        detections.sort(key=lambda item: (-item.confidence, item.x, item.y))

        for track in self.tracks:
            track.predict(
                timestamp_ns,
                self.acceleration_sigma,
                self.max_prediction_dt,
            )

        matched_tracks: set[int] = set()
        matched_detections: set[int] = set()
        invalid_cost = 1.0e9

        def associate(track_indices: list[int], detection_indices: list[int]) -> None:
            if not track_indices or not detection_indices:
                return
            costs = np.full(
                (len(track_indices), len(detection_indices)),
                invalid_cost,
                dtype=np.float64,
            )
            for row, track_index in enumerate(track_indices):
                track = self.tracks[track_index]
                for column, detection_index in enumerate(detection_indices):
                    detection = detections[detection_index]
                    distance = math.hypot(
                        float(track.state[0]) - detection.x,
                        float(track.state[1]) - detection.y,
                    )
                    if distance <= self.association_threshold:
                        costs[row, column] = distance + 1.0e-9 * (
                            track_index * len(detections) + detection_index
                        )
            rows, columns = linear_sum_assignment(costs)
            for row, column in zip(rows.tolist(), columns.tolist()):
                if costs[row, column] >= invalid_cost:
                    continue
                track_index = track_indices[row]
                detection_index = detection_indices[column]
                self.tracks[track_index].update(
                    detections[detection_index],
                    timestamp_ns,
                    self.measurement_sigma,
                    self.confidence_alpha,
                    self.min_hits,
                )
                matched_tracks.add(track_index)
                matched_detections.add(detection_index)

        # Confirmed/coasting tracks retain ownership before tentative tracks.
        # This prevents a duplicate point from birthing a tentative track that
        # steals the next measurement from a long-lived confirmed identity.
        confirmed_indices = [
            index for index, track in enumerate(self.tracks) if track.confirmed
        ]
        tentative_indices = [
            index for index, track in enumerate(self.tracks) if not track.confirmed
        ]
        all_detection_indices = list(range(len(detections)))
        associate(confirmed_indices, all_detection_indices)
        associate(
            tentative_indices,
            [
                index
                for index in all_detection_indices
                if index not in matched_detections
            ],
        )

        for row, track in enumerate(self.tracks):
            if row not in matched_tracks:
                track.miss()

        for column, detection in enumerate(detections):
            if column in matched_detections:
                continue
            new_track = _Track(
                self.next_track_id,
                detection,
                timestamp_ns,
                self.initial_position_sigma,
                self.initial_velocity_sigma,
                self.measurement_history_size,
                self.velocity_fit_min_samples,
                self.velocity_fit_min_span,
            )
            new_track.confirmed = self.min_hits == 1
            self.tracks.append(new_track)
            self.next_track_id += 1

        survivors = []
        for track in self.tracks:
            coast_time = (timestamp_ns - track.last_update_ns) / 1.0e9
            if track.misses <= self.max_age and coast_time <= self.max_coast_time:
                survivors.append(track)
        self.tracks = survivors

        estimates = [
            track.snapshot(timestamp_ns)
            for track in sorted(self.tracks, key=lambda item: item.track_id)
        ]
        if any(
            not all(math.isfinite(value) for value in (item.x, item.y, item.vx, item.vy))
            for item in estimates
        ):
            raise FloatingPointError("tracker produced a non-finite state")
        return estimates
