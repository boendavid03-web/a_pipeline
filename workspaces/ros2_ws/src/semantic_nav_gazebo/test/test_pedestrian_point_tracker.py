import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from pedestrian_point_tracker_core import (  # noqa: E402
    PointCVKalmanTracker,
    PointDetection,
    linear_sum_assignment,
)


def test_smoke_a_two_people_variable_dt_and_single_miss():
    tracker = PointCVKalmanTracker(
        association_threshold=0.8,
        min_hits=3,
        max_age=5,
        measurement_sigma=0.05,
        acceleration_sigma=1.0,
    )
    rng = np.random.default_rng(7)
    timestamp_ns = 1_000_000_000
    elapsed = 0.0
    identity_history = {"a": [], "b": []}
    latest = {}
    for frame in range(18):
        dt = (0.07, 0.08, 0.075)[frame % 3]
        if frame > 0:
            elapsed += dt
            timestamp_ns += int(round(dt * 1.0e9))
        a_xy = np.asarray([1.0 + elapsed, 2.0])
        b_xy = np.asarray([3.0, 1.0 + elapsed])
        detections = []
        if frame != 8:
            noisy_a = a_xy + rng.normal(0.0, 0.008, size=2)
            detections.append(PointDetection(*noisy_a, 0.98))
        noisy_b = b_xy + rng.normal(0.0, 0.008, size=2)
        detections.append(PointDetection(*noisy_b, 0.99))
        estimates = tracker.update(detections, timestamp_ns)
        if frame < 2:
            continue
        for name, expected in (("a", a_xy), ("b", b_xy)):
            estimate = min(
                estimates,
                key=lambda item: np.linalg.norm(
                    np.asarray([item.x, item.y]) - expected
                ),
            )
            identity_history[name].append(estimate.track_id)
            latest[name] = estimate

    assert len(set(identity_history["a"])) == 1
    assert len(set(identity_history["b"])) == 1
    assert identity_history["a"][0] != identity_history["b"][0]
    assert latest["a"].state == "CONFIRMED"
    assert latest["b"].state == "CONFIRMED"
    assert latest["a"].vx > 0.7
    assert abs(latest["a"].vy) < 0.25
    assert latest["b"].vy > 0.7
    assert abs(latest["b"].vx) < 0.25
    assert all(
        np.isfinite([item.x, item.y, item.vx, item.vy]).all()
        for item in latest.values()
    )


def test_single_miss_is_coasting_and_not_new_id():
    tracker = PointCVKalmanTracker(min_hits=2, max_age=2)
    first = tracker.update([PointDetection(1.0, 0.0, 0.99)], 1_000_000_000)
    second = tracker.update([PointDetection(1.1, 0.0, 0.99)], 1_100_000_000)
    missed = tracker.update([], 1_200_000_000)
    resumed = tracker.update([PointDetection(1.3, 0.0, 0.99)], 1_300_000_000)
    assert first[0].track_id == second[0].track_id == missed[0].track_id
    assert missed[0].state == "COASTING"
    assert resumed[0].track_id == first[0].track_id
    assert resumed[0].state == "CONFIRMED"


def test_timestamp_must_increase_and_large_dt_stays_finite():
    tracker = PointCVKalmanTracker(max_prediction_dt=0.5, max_coast_time=5.0)
    tracker.update([PointDetection(1.0, 2.0, 0.99)], 1_000_000_000)
    with pytest.raises(ValueError, match="strictly increasing"):
        tracker.update([PointDetection(1.1, 2.0, 0.99)], 1_000_000_000)
    estimate = tracker.update([], 4_000_000_000)[0]
    assert np.isfinite([estimate.x, estimate.y, estimate.vx, estimate.vy]).all()


def test_hungarian_is_one_to_one():
    rows, columns = linear_sum_assignment(
        np.asarray([[0.1, 0.2], [0.11, 10.0]], dtype=np.float64)
    )
    assert sorted(rows.tolist()) == [0, 1]
    assert sorted(columns.tolist()) == [0, 1]
    assert len(set(columns.tolist())) == len(columns)


def test_confirmed_track_has_priority_over_nearby_tentative_duplicate():
    tracker = PointCVKalmanTracker(min_hits=3, association_threshold=0.8)
    tracker.update([PointDetection(1.0, 0.0, 0.99)], 1_000_000_000)
    tracker.update([PointDetection(1.1, 0.0, 0.99)], 1_100_000_000)
    third = tracker.update(
        [PointDetection(1.2, 0.0, 0.99), PointDetection(1.7, 0.0, 0.96)],
        1_200_000_000,
    )
    confirmed_id = next(item.track_id for item in third if item.state == "CONFIRMED")
    fourth = tracker.update([PointDetection(1.3, 0.0, 0.99)], 1_300_000_000)
    confirmed = next(item for item in fourth if item.track_id == confirmed_id)
    assert confirmed.state == "CONFIRMED"
    assert confirmed.misses == 0
    assert abs(confirmed.x - 1.3) < 0.2
