import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from navigation_evaluation_core import (
    align_actuation_series,
    constant_velocity_ttc,
    derivative_summary,
    distribution_summary,
    failure_to_progress_summary,
    goal_spl,
    path_length,
    path_irregularity_summary,
    personal_space_integral,
    planner_reference_metadata,
    time_milestones,
    ttc_statistics,
    threshold_exposure,
)


def test_actuation_alignment_is_causal_and_reports_gates_and_low_variance():
    decisions = [
        {"time": 0.0, "raw": 1.0, "final": 0.8, "raw_angular": 0.3, "final_angular": 0.2, "gated": True, "sequence": 1},
        {"time": 1.0 / 15.0, "raw": 0.5, "final": 0.5, "raw_angular": 0.2, "final_angular": 0.2, "gated": False, "sequence": 2},
        {"time": 2.0 / 15.0, "raw": 0.4, "final": 0.4, "raw_angular": 0.1, "final_angular": 0.1, "gated": False, "sequence": 4},
    ]
    states = [
        {"time": 0.0, "received": 0.8, "applied": 0.8, "actual": 0.7, "received_angular": 0.2, "applied_angular": 0.2, "actual_angular": 0.15, "sequence": 10, "command_sequence": 20},
        {"time": 1.0 / 15.0, "received": 0.5, "applied": 0.5, "actual": 0.4, "received_angular": 0.2, "applied_angular": 0.2, "actual_angular": 0.18, "sequence": 11, "command_sequence": 21},
        {"time": 2.0 / 15.0, "received": 0.4, "applied": 0.4, "actual": 0.3, "received_angular": 0.1, "applied_angular": 0.1, "actual_angular": 0.08, "sequence": 13, "command_sequence": 23},
    ]
    result = align_actuation_series(decisions, states)
    assert len(result["rows"]) == 3
    assert math.isclose(result["raw_to_final"]["bias"], -0.06666666666666665)
    assert result["gated"]["sample_count"] == 1
    assert result["ungated"]["sample_count"] == 2
    assert math.isclose(result["final_to_actual"]["mae"], 0.1)
    assert result["best_causal_delay_sec"] is not None
    assert result["final_to_actual_angular"]["sample_count"] == 3
    assert result["diagnostics"]["decision_sequence_gaps"] == 1
    assert result["diagnostics"]["telemetry_sequence_gaps"] == 1
    assert result["diagnostics"]["command_sequence_gaps"] == 1
    low = align_actuation_series(
        [{"time": t / 15.0, "raw": 0.0, "final": 0.0, "raw_angular": 0.0, "final_angular": 0.0, "gated": False} for t in range(3)],
        [{"time": t / 15.0, "received": 0.0, "applied": 0.0, "actual": 0.0, "received_angular": 0.0, "applied_angular": 0.0, "actual_angular": 0.0} for t in range(3)],
    )
    assert low["delay_reason"] == "insufficient_or_low_variation_command"
    rollback = align_actuation_series(decisions + [{"time": 0.01, "raw": 1.0, "final": 1.0, "raw_angular": 0.0, "final_angular": 0.0, "gated": False}], states)
    assert rollback["diagnostics"]["decision_duplicate_or_nonpositive"] == 1

    sparse_decisions = [dict(decisions[0]), {**decisions[-1], "time": 0.2}]
    extended_states = states + [{**states[-1], "time": 0.2, "sequence": 14, "command_sequence": 24}]
    stale = align_actuation_series(sparse_decisions, extended_states, freshness_sec=0.01)
    assert stale["diagnostics"]["decision_stale"] > 0

    stop_decisions = [
        {"time": 0.0, "raw": None, "final": 0.0, "raw_angular": None,
         "final_angular": 0.0, "gated": True}
    ]
    stop_states = [
        {"time": 0.0, "received": 0.0, "applied": 0.0, "actual": 0.03,
         "received_angular": 0.0, "applied_angular": 0.0, "actual_angular": 0.01}
    ]
    stop = align_actuation_series(stop_decisions, stop_states)
    assert stop["raw_to_final"]["valid"] is False
    assert stop["final_to_actual"]["zero_command_hold_error"] == 0.03


def test_path_length_and_goal_spl():
    assert math.isclose(path_length([(0, 0), (3, 4), (3, 8)]), 9.0)
    assert math.isclose(goal_spl(True, 6.0, 9.0), 2.0 / 3.0)
    assert goal_spl(False, 6.0, 9.0) == 0.0
    assert goal_spl(True, None, 9.0) is None
    assert goal_spl(None, 6.0, 9.0) is None
    assert goal_spl(False, None, None) == 0.0
    assert goal_spl(False, 6.0, 0.0) == 0.0


def test_ttc_approaching_and_receding():
    assert math.isclose(constant_velocity_ttc((0, 0), (0, 0), (3, 0), (-1, 0), 1.0), 2.0)
    assert constant_velocity_ttc((0, 0), (0, 0), (3, 0), (1, 0), 1.0) is None
    assert constant_velocity_ttc((0, 0), (0, 0), (0.5, 0), (1, 0), 1.0) == 0.0


def test_personal_space_integral_and_missing_values():
    seconds, ratio = personal_space_integral([(0.0, 0.5), (2.0, 1.0), (5.0, float("nan"))], 0.8)
    assert seconds == 2.0
    assert math.isclose(ratio, 0.4)


def test_jerk_is_second_difference_of_velocity_and_audits_timestamps():
    acceleration = derivative_summary([(0.0, 0.0), (1.0, 1.0), (2.0, 4.0)], order=1)
    jerk = derivative_summary([(0.0, 0.0), (1.0, 1.0), (2.0, 4.0)], order=2)
    assert acceleration["sample_count"] == 2
    assert math.isclose(acceleration["rms"], math.sqrt(5.0))
    assert jerk["sample_count"] == 1
    assert math.isclose(jerk["rms"], 2.0)
    assert math.isclose(jerk["maximum"], 2.0)

    audited = derivative_summary(
        [(0.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (1.0, 2.0), (5.0, 10.0)],
        order=1,
        max_dt=2.0,
    )
    assert audited["sample_count"] == 1
    assert audited["duplicate_timestamp_pairs"] == 1
    assert audited["nonpositive_dt_pairs"] == 2
    assert audited["abnormal_dt_pairs"] == 1


def test_ttc_statistics_counts_none_as_infinite_and_uses_episode_time():
    summary = ttc_statistics(
        [(0.0, None), (1.0, 4.0), (2.0, 1.0), (3.0, None), (5.0, 1.0)],
        episode_time_sec=5.0,
        threshold_sec=2.0,
    )
    assert summary["minimum_finite_ttc_sec"] == 1.0
    assert math.isclose(summary["finite_ttc_sample_fraction"], 3.0 / 5.0)
    assert math.isclose(summary["finite_ttc_time_fraction"], 2.0 / 5.0)
    assert summary["time_below_threshold_sec"] == 1.0
    assert math.isclose(summary["episode_time_fraction_below_threshold_sec"], 1.0 / 5.0)
    assert summary["threshold_sec"] == 2.0


def test_time_milestones_preserve_zero_goal_stamp():
    summary = time_milestones(0.0, 10.0, 1.0, 2.0, 3.0)
    assert summary == {
        "goal_to_reach_sec": 10.0,
        "first_odom_to_reach_sec": 9.0,
        "first_policy_action_to_reach_sec": 8.0,
        "first_nonzero_cmd_to_reach_sec": 7.0,
    }
    assert time_milestones(0.0, 3.0, 4.0, None, 2.0)["first_odom_to_reach_sec"] is None


def test_planner_reference_metadata_is_explicitly_astar_based():
    metadata = planner_reference_metadata(0.05, 0.5, 0.8)
    assert metadata["algorithm"] == "astar_8_connected"
    assert metadata["resolution"] == 0.05
    assert metadata["inflation_radius"] == 0.5
    assert metadata["snap_radius"] == 0.8
    assert "snap_to_nearest_free_cell" in metadata["endpoint_convention"]


def test_new_metric_helpers_are_json_safe_and_semantically_correct():
    assert distribution_summary([])["p95"] is None
    exposure = threshold_exposure([(0.0, 0.4), (1.0, 0.9), (2.0, 0.3)], 0.8)
    assert exposure["entry_count"] == 2
    assert math.isclose(exposure["max_penetration"], 0.5)
    assert failure_to_progress_summary([(0.0, 2.0), (5.0, 1.0)], 5.0, 0.2)["failed"] is False
    assert failure_to_progress_summary([(0.0, 2.0), (5.0, 2.0)], 5.0, 0.2)["failed"] is True
    assert failure_to_progress_summary([(0.0, 2.0), (5.0, 3.0)], 5.0, 0.2)["failed"] is True
    dense_stall = failure_to_progress_summary([(float(t), 2.0) for t in range(16)], 5.0, 0.2)
    assert dense_stall["event_count"] == 1
    assert math.isclose(dense_stall["stalled_duration_sec"], 10.0)
    assert math.isclose(dense_stall["stalled_duration_ratio"], 1.0)
    recovery_second_stall = [(float(t), 2.0 if t <= 10 or t >= 16 else 1.0) for t in range(26)]
    recovered = failure_to_progress_summary(recovery_second_stall, 5.0, 0.2)
    assert recovered["event_count"] == 2
    assert path_irregularity_summary([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])[
        "turning_rad_per_m"
    ] is not None
    assert path_irregularity_summary([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])[ 
        "unnecessary_turn_rad"
    ] == 0.0
    oscillation = path_irregularity_summary([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)])
    assert oscillation["unnecessary_turn_rad"] > 0.0
    micro = path_irregularity_summary([(0.0, 0.0), (1e-10, 0.0), (2e-10, 0.0)])
    assert micro["valid"] is False
    assert micro["turning_rad_per_m"] is None
    import json
    json.dumps({"distribution": distribution_summary([]), "exposure": exposure}, allow_nan=False)
