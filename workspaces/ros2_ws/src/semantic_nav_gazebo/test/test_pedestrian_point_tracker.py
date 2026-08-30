import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from pedestrian_point_tracker_core import (  # noqa: E402
    MeasurementSample,
    PointCVKalmanTracker,
    PointDetection,
    causal_linear_velocity_fit,
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


def measurement(timestamp: float, x: float, y: float, confidence: float = 0.95):
    return MeasurementSample(int(round(timestamp * 1.0e9)), x, y, confidence)


def test_causal_fit_exact_constant_velocity_fit5_and_fit8():
    samples = [
        measurement(t, 1.2 + 0.7 * t, -0.4 - 0.3 * t)
        for t in np.arange(0.0, 0.8, 0.1)
    ]
    fit5 = causal_linear_velocity_fit(samples, window_size=5)
    fit8 = causal_linear_velocity_fit(samples, window_size=8)
    assert fit5.valid and fit8.valid
    assert fit5.vx == pytest.approx(0.7, abs=1.0e-10)
    assert fit5.vy == pytest.approx(-0.3, abs=1.0e-10)
    assert fit8.vx == pytest.approx(0.7, abs=1.0e-10)
    assert fit8.vy == pytest.approx(-0.3, abs=1.0e-10)


def test_causal_fit_uses_irregular_real_timestamps():
    times = [0.0, 0.071, 0.155, 0.239, 0.402, 0.491]
    samples = [measurement(t, -0.2 + 1.1 * t, 2.0 + 0.25 * t) for t in times]
    fit = causal_linear_velocity_fit(samples, window_size=5)
    assert fit.valid
    assert fit.samples == 5
    assert fit.time_span == pytest.approx(times[-1] - times[-5])
    assert fit.vx == pytest.approx(1.1, abs=1.0e-9)
    assert fit.vy == pytest.approx(0.25, abs=1.0e-9)


def test_causal_fit_reduces_zero_mean_position_noise():
    times = np.arange(0.0, 0.8, 0.1)
    x_noise = [0.01, -0.01, 0.006, -0.006, 0.008, -0.008, 0.004, -0.004]
    y_noise = [-0.006, 0.006, -0.004, 0.004, -0.008, 0.008, -0.002, 0.002]
    samples = [
        measurement(t, 0.6 * t + x_noise[index], -0.2 * t + y_noise[index])
        for index, t in enumerate(times)
    ]
    fit = causal_linear_velocity_fit(samples, window_size=8)
    assert fit.valid
    assert abs(fit.vx - 0.6) < 0.03
    assert abs(fit.vy + 0.2) < 0.03
    assert fit.fit_rmse is not None and fit.fit_rmse < 0.02


def test_one_or_two_misses_do_not_add_coast_predictions_to_history():
    tracker = PointCVKalmanTracker(min_hits=2, max_coast_time=1.0)
    tracker.update([PointDetection(0.0, 0.0, 0.99)], 1_000_000_000)
    tracker.update([PointDetection(0.1, 0.0, 0.99)], 1_100_000_000)
    one_miss = tracker.update([], 1_200_000_000)[0]
    two_misses = tracker.update([], 1_300_000_000)[0]
    resumed = tracker.update([PointDetection(0.4, 0.0, 0.99)], 1_400_000_000)[0]
    assert one_miss.fit5.samples == 2
    assert two_misses.fit5.samples == 2
    assert resumed.fit5.samples == 3
    assert resumed.fit5.time_span == pytest.approx(0.4)
    assert resumed.fit5.valid
    assert resumed.fit5.vx == pytest.approx(1.0, abs=1.0e-9)


def test_causal_fit_insufficient_samples_is_invalid():
    fit = causal_linear_velocity_fit(
        [measurement(0.0, 0.0, 0.0), measurement(0.2, 0.2, 0.0)],
        window_size=5,
    )
    assert not fit.valid
    assert fit.samples == 2
    assert fit.vx is None and fit.vy is None


def test_causal_fit_enforces_minimum_time_span():
    samples = [
        measurement(0.00, 0.0, 0.0),
        measurement(0.02, 0.2, 0.0),
        measurement(0.04, 0.4, 0.0),
    ]
    fit = causal_linear_velocity_fit(
        samples, window_size=5, min_samples=3, min_time_span=0.15
    )
    assert not fit.valid
    assert fit.samples == 3
    assert fit.time_span == pytest.approx(0.04)


def test_plain_fit_exposes_single_outlier_sensitivity():
    samples = [measurement(t, t, 0.0) for t in np.arange(0.0, 0.8, 0.1)]
    samples[-2] = measurement(0.6, 1.6, 0.0)
    fit = causal_linear_velocity_fit(samples, window_size=8)
    assert fit.valid
    assert abs(fit.vx - 1.0) > 0.1
    assert fit.fit_rmse is not None and fit.fit_rmse > 0.25


def test_reset_clears_velocity_measurement_history():
    tracker = PointCVKalmanTracker(min_hits=1)
    tracker.update([PointDetection(0.0, 0.0, 0.99)], 1_000_000_000)
    tracker.update([PointDetection(0.2, 0.0, 0.99)], 1_200_000_000)
    before_reset = tracker.update(
        [PointDetection(0.4, 0.0, 0.99)], 1_400_000_000
    )[0]
    assert before_reset.fit5.valid and before_reset.fit5.samples == 3
    tracker.reset()
    after_reset = tracker.update(
        [PointDetection(10.0, 4.0, 0.99)], 2_000_000_000
    )[0]
    assert after_reset.track_id == 1
    assert after_reset.fit5.samples == 1
    assert not after_reset.fit5.valid


def test_causal_output_at_k_is_independent_of_future_measurements():
    history = [measurement(t, 0.5 * t, -0.1 * t) for t in np.arange(0.0, 0.8, 0.1)]
    prefix = history[:5]
    at_k_before_future = causal_linear_velocity_fit(prefix, window_size=5)
    history.append(measurement(0.8, 100.0, -100.0))
    at_k_after_future_exists = causal_linear_velocity_fit(prefix, window_size=5)
    assert at_k_before_future == at_k_after_future_exists
    assert at_k_before_future.vx == pytest.approx(0.5, abs=1.0e-10)
    assert at_k_before_future.vy == pytest.approx(-0.1, abs=1.0e-10)
