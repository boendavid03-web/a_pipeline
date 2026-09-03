from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"


def load_module(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / filename)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


generator = load_module("single_motion_generator", "generate_single_motion_config.py")
analyzer = load_module("single_motion_analyzer", "analyze_single_motion_benchmark.py")


@pytest.mark.parametrize("scenario", tuple(generator.SCENARIOS))
def test_scenario_geometry_is_eight_metres_and_speed_is_frozen(scenario):
    spec = generator.scenario_spec(scenario, 0.8)
    assert spec["route_length"] == pytest.approx(8.0)
    assert math.hypot(*spec["target_local_velocity"]) == pytest.approx(0.8)


def test_front_routes_are_exact_reversals():
    approach = generator.scenario_spec("front_approach", 0.8)
    leave = generator.scenario_spec("front_leave", 0.8)
    assert approach["robot_pose"] == leave["robot_pose"]
    assert approach["local_route"] == tuple(reversed(leave["local_route"]))
    assert approach["odom_route"] == tuple(reversed(leave["odom_route"]))


@pytest.mark.parametrize("scenario", tuple(generator.SCENARIOS))
def test_generated_config_has_one_person_and_passes_route_visibility(tmp_path, scenario):
    spec = generator.scenario_spec(scenario, 0.8)
    template = yaml.safe_load(generator.DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    config = generator.build_config(template, spec, 0.8, 7)
    generator.validate_geometry(config, spec, generator.DEFAULT_WORLD, 0.55)
    root = config["isaacsim.replicator.agent"]
    assert root["seed"] == 7
    assert list(root["character"]["groups"]) == ["benchmark_person"]
    group = root["character"]["groups"]["benchmark_person"]
    assert group["num"] == 1
    patrol = next(item["patrol"] for item in group["routines"] if "patrol" in item)
    assert patrol["speed_range"] == [0.8, 0.8]
    assert len(patrol["path_points"]) == 2


def test_inverse_transform_round_trip_for_all_scenarios():
    for scenario in generator.SCENARIOS:
        spec = generator.scenario_spec(scenario, 0.8)
        for local, odom in zip(spec["local_route"], spec["odom_route"]):
            assert analyzer.inverse_transform(*odom, spec["robot_pose"]) == pytest.approx(local)


def test_percentile_and_position_statistics():
    values = [0.1, 0.2, 0.3, 0.4]
    assert analyzer.mean(values) == pytest.approx(0.25)
    assert analyzer.median(values) == pytest.approx(0.25)
    assert max(values) == pytest.approx(0.4)
    assert analyzer.percentile(values, 0.95) == pytest.approx(0.385)


def scenario_metadata(scenario="front_approach"):
    spec = generator.scenario_spec(scenario, 0.8)
    return generator.metadata_for(
        spec,
        speed=0.8,
        seed=7,
        clearance=0.55,
        template=generator.DEFAULT_TEMPLATE,
        map_yaml=generator.DEFAULT_MAP_YAML,
        world=generator.DEFAULT_WORLD,
    )


def trace_record(timestamp_ns, *, pred_velocity=(-0.8, 0.0), matched=True):
    track = {
        "track_id": 4,
        "x": 7.0,
        "y": 9.0,
        "vx": pred_velocity[0],
        "vy": pred_velocity[1],
        "state": "CONFIRMED",
    }
    return {
        "track_timestamp_ns": timestamp_ns,
        "gt_timestamp_ns": timestamp_ns,
        "timestamp_offset_sec": 0.0,
        "frame": "odom",
        "tracks": [track],
        "ground_truth": [{
            "id": "benchmark_person_0",
            "x": 7.0,
            "y": 9.0,
            "raw_vx": -0.8,
            "raw_vy": 0.0,
            "vx": -0.8,
            "vy": 0.0,
            "velocity_by_half_window": {"0.30": {"vx": -0.8, "vy": 0.0}},
        }],
        "matches": ([{"track_id": 4, "gt_id": "benchmark_person_0", "position_error_m": 0.1}] if matched else []),
        "target_ground_truth": {
            "id": "benchmark_person_0", "x": 7.0, "y": 9.0,
            "raw_vx": -0.8, "raw_vy": 0.0, "vx": -0.8, "vy": 0.0,
        },
    }


def test_velocity_components_speed_and_angle():
    metadata = scenario_metadata()
    trajectory, velocity = analyzer.row_from_trace(
        trace_record(1_000_000_000, pred_velocity=(-0.6, 0.2)), metadata, "steady"
    )
    assert trajectory["position_error_m"] == pytest.approx(0.1)
    assert velocity["vx_error_mps"] == pytest.approx(0.2)
    assert velocity["vy_error_mps"] == pytest.approx(0.2)
    assert velocity["velocity_vector_error_mps"] == pytest.approx(math.sqrt(0.08))
    assert velocity["speed_error_mps"] == pytest.approx(abs(math.sqrt(0.4) - 0.8))
    assert velocity["angle_error_deg"] == pytest.approx(18.4349488229)
    assert velocity["angle_valid"] is True


def test_angle_wrap_and_low_prediction_are_auditable():
    metadata = scenario_metadata()
    record = trace_record(1_000_000_000, pred_velocity=(-0.1, 0.0))
    _, velocity = analyzer.row_from_trace(record, metadata, "steady")
    assert velocity["angle_error_deg"] is None
    assert velocity["angle_valid"] is False
    assert velocity["direction_unavailable"] is True

    difference = math.radians(179.0) - math.radians(-179.0)
    wrapped = abs(math.degrees(math.atan2(math.sin(difference), math.cos(difference))))
    assert wrapped == pytest.approx(2.0)


def test_phase_segmentation_trims_edges_and_excludes_reverse():
    metadata = scenario_metadata()
    samples = []
    for index in range(101):
        timestamp_ns = index * 100_000_000
        x = 10.0 - 0.08 * index
        samples.append({
            "timestamp_ns": timestamp_ns,
            "x_local": x,
            "y_local": 0.0,
            "velocity": (-0.8, 0.0),
        })
    phases = analyzer.assign_phases(samples, metadata)
    assert phases[0] == "desired_transient"
    assert phases[500_000_000] == "steady"
    assert phases[9_500_000_000] == "steady"
    assert phases[10_000_000_000] == "desired_transient"
    samples[50]["velocity"] = (0.8, 0.0)
    phases = analyzer.assign_phases(samples, metadata)
    assert phases[5_000_000_000] == "reverse_motion"


def test_unmatched_row_is_preserved():
    metadata = scenario_metadata()
    trajectory, velocity = analyzer.row_from_trace(
        trace_record(1_000_000_000, matched=False), metadata, "steady"
    )
    assert trajectory["matched"] is False
    assert trajectory["track_id"] is None
    assert velocity["vx_pred_mps"] is None
    assert velocity["invalid_reason"] == "unmatched"


def test_report_requires_explicit_four_summaries(tmp_path):
    paths = []
    for scenario in analyzer.SCENARIOS:
        path = tmp_path / f"{scenario}.json"
        path.write_text(json.dumps({
            "scenario": scenario,
            "episode": {"valid": True, "algorithm_status": "MEASURED"},
            "denominators": {"n_gt_eval": 100, "n_sync": 100, "n_matched": 90, "angle_coverage": 0.8},
            "position": {"mean_error_m": 0.1, "median_error_m": 0.09, "max_error_m": 0.3},
            "velocity": {"speed_mae_mps": 0.2, "angle_error_deg": {"mean": 5.0}},
        }), encoding="utf-8")
        paths.append(path)
    args = type("Args", (), {"summary": paths, "output": tmp_path / "single_person_motion_report.md"})
    assert analyzer.report(args) == 0
    text = args.output.read_text(encoding="utf-8")
    assert "| scenario | position error | velocity MAE | direction error |" in text
    primary_table = text.split("Position cells are", 1)[0]
    assert [line.split("|")[1].strip() for line in primary_table.splitlines() if line.startswith("| front") or line.startswith("| lateral") or line.startswith("| diagonal")] == list(analyzer.SCENARIOS)


def test_runner_freezes_baseline_and_has_no_crowded_dependency():
    runner = (SCRIPT_DIR / "run_single_person_motion_benchmark.sh").read_text(encoding="utf-8")
    for token in (
        "ckpt_jrdb_ann_ft_dr_spaam_e20.pth", "conf_thresh:=0.95", "stride:=5",
        "ISAAC_LIDAR_RATE_HZ=15", "ISAAC_LIDAR_SAMPLE_COUNT=2000",
        "association_threshold:=0.8", "measurement_sigma:=0.10",
        "ros2 bag record --use-sim-time", "CAPTURE_SEC",
        "ISAAC_ROBOT_PHYSICS=0", "ISAAC_PEDESTRIAN_COUNT=1",
    ):
        assert token in runner
    assert "run_custom_people_dr_spaam_crowded_tracking_stress.sh" not in runner
    assert "pedestrian_crowded_tracking_evaluator.py" not in runner
    assert "pkill" not in runner
    detector_block = runner[runner.index('"$TRAIN_PYTHON" "$DR_SPAAM_NODE"'):runner.index("ros2 bag record")]
    tracker_block = runner[runner.index('/usr/bin/python3 "$TRACKER"'):runner.index('/usr/bin/python3 "$EVALUATOR"')]
    assert "/pedestrian_ground_truth" not in detector_block
    assert "/pedestrian_ground_truth" not in tracker_block
