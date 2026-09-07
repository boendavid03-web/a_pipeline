import importlib.util
import json
import sys
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "evaluate_scenario_topology_ab.py"
)
SPEC = importlib.util.spec_from_file_location("evaluate_scenario_topology_ab", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_episode_accumulator_integrates_long_interval_without_splitting():
    accumulator = MODULE.EpisodeAccumulator()
    accumulator.update(True, 0.1, False)
    accumulator.update(True, 0.2, False)
    accumulator.update(True, 0.6, True)
    accumulator.update(True, 0.1, False)
    accumulator.update(False, 0.1, False)

    assert accumulator.count == 1
    assert accumulator.total_sec == pytest.approx(1.0)
    assert accumulator.maximum_sec == pytest.approx(1.0)


def test_percentile_and_person_id_helpers():
    assert MODULE.percentile([0.0, 10.0], 0.5) == 5.0
    assert MODULE.short_person_id(
        "/World/Characters/gazebo_a_001/gazebo_a_001_0/ManRoot/model"
    ) == "gazebo_a_001"


def test_exact_free_space_target_precedes_legacy_written_target():
    legacy = {"applied_target_m": [1.0, 2.0]}
    target, exact = MODULE.free_space_selected_target(legacy)
    assert target == [1.0, 2.0]
    assert exact is False

    current = {
        "applied_target_m": [1.0, 2.0],
        "free_space_selected_target_m": [0.0, 0.0],
    }
    target, exact = MODULE.free_space_selected_target(current)
    assert target == [0.0, 0.0]
    assert exact is True


def test_trace_metrics_supports_single_pedestrian(tmp_path):
    trace = tmp_path / "single.jsonl"
    person = {
        "position_m": [1.0, 2.0],
        "preferred_speed_mps": 1.0,
        "actual_navigation_velocity_mps": [0.5, 0.0],
        "isaac_adapter_output_velocity_mps": [0.5, 0.0],
        "emergency_inhibited": False,
        "free_space_constrained": False,
        "free_space_selected_target_m": [2.0, 2.0],
        "free_space_decision": "requested_safe",
        "gazebo_raw_lateral_component_mps": 0.0,
        "adapter_lateral_component_mps": 0.0,
        "actual_navigation_lateral_mps": 0.0,
    }
    records = [
        {"schema": MODULE.TRACE_SCHEMA, "type": "header"},
        {
            "schema": MODULE.TRACE_SCHEMA,
            "type": "sample",
            "sim_time": 1.0,
            "people": {"/World/Characters/solo/model": person},
        },
        {
            "schema": MODULE.TRACE_SCHEMA,
            "type": "sample",
            "sim_time": 1.1,
            "people": {"/World/Characters/solo/model": person},
        },
    ]
    trace.write_text("".join(json.dumps(record) + "\n" for record in records))

    metrics = MODULE.trace_metrics(trace, {"solo"})

    assert metrics["sample_frames"] == 2
    assert metrics["geometry"]["pair_observations"] == 0
    assert metrics["geometry"]["global_minimum_human_distance_m"] is None
    assert metrics["geometry"]["average_nearest_neighbor_distance_m"] is None


def test_compare_seed7_gate_requires_all_primary_improvements():
    def summary(congestion, constraint, freeze, cycles, normalized):
        hotspot = {
            "person_time_sec": 10.0,
            "overlap_lt_0_45_person_sec": 1.0,
            "personal_space_lt_1_0_person_sec": 2.0,
            "congestion_person_sec": 1.0,
            "constraint_person_sec": 1.0,
            "freeze_person_sec": 1.0,
        }
        return {
            "run_name": "run",
            "valid": True,
            "runtime": {
                "parameters": {"weight": 5.1},
                "producer_source_sha256": "same",
                "launcher_sha256": "same",
                "rtf": 1.0,
                "average_fps": 5.0,
            },
            "trace": {
                "valid_integration_coverage": 1.0,
                "sample_frames": 10,
                "missing_interval_count": 0,
                "hotspots": {
                    name: dict(hotspot) for name in MODULE.HOTSPOTS
                },
                "geometry": {
                    "global_minimum_human_distance_m": 0.5,
                    "per_frame_minimum_p01_m": 0.5,
                    "per_frame_minimum_p05_m": 0.5,
                    "per_frame_minimum_p50_m": 0.8,
                    "average_nearest_neighbor_distance_m": 1.2,
                    "visual_overlap_pair_ratio": 0.01,
                    "personal_space_pair_ratio": 0.02,
                },
                "congestion": {
                    "total_person_seconds": congestion,
                    "maximum_continuous_sec": congestion,
                },
                "freeze": {
                    "inclusive": {
                        "total_person_seconds": freeze,
                        "maximum_continuous_sec": freeze,
                        "episode_count": 1,
                    },
                    "emergency_inhibited": {"total_person_seconds": 0.0},
                },
                "free_space_constraint": {
                    "sample_ratio": 0.1,
                    "episode_count": constraint,
                    "total_person_seconds": float(constraint),
                    "target_current_fallback_count": 0,
                },
                "lateral_execution": {
                    "mean_abs_raw_lateral_mps": 0.1,
                    "mean_abs_adapter_lateral_mps": 0.1,
                    "mean_abs_actual_lateral_mps": 0.1,
                    "raw_to_adapter_realization_ratio": 1.0,
                    "adapter_to_actual_realization_ratio": 1.0,
                },
            },
            "route": {
                "people_with_completed_cycle": cycles,
                "completed_semantic_cycles": cycles,
                "completed_semantic_legs_per_person_minute": 1.0,
                "normalized_completed_cycle_distance": normalized,
            },
        }

    a = summary(10.0, 10, 5.0, 2, 0.2)
    b = summary(8.0, 8, 4.0, 2, 0.2)
    assert MODULE.compare_runs(a, b)["seed7_pass"] is True

    b["trace"]["freeze"]["inclusive"]["total_person_seconds"] = 6.0
    assert MODULE.compare_runs(a, b)["seed7_pass"] is False
