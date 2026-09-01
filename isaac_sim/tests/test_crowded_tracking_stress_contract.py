from __future__ import annotations

import sys
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from generate_crowded_tracking_config import build_config, route_specs  # noqa: E402


def template_config():
    path = SCRIPT_DIR / "ira_people_demo/custom_eng_lobby_people.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_all_scenarios_have_named_independent_people_and_expected_counts():
    for scenario, expected_count in {"A": 2, "B": 2, "C": 2, "D": 2, "E": 3}.items():
        config, metadata = build_config(
            template_config(), scenario, 0.75, speed=0.8, seed=7
        )
        groups = config["isaacsim.replicator.agent"]["character"]["groups"]
        scene = config["isaacsim.replicator.agent"]["environment"][
            "base_stage_asset_path"
        ]
        assert Path(scene).is_absolute()
        assert Path(scene).is_file()
        assert metadata["robot_spawn_ros_m"] == [13.5, 6.5, 0.01]
        assert [f"{name}_0" for name in groups] == metadata["stress_ids"]
        assert len(groups) == expected_count
        assert all(group["num"] == 1 for group in groups.values())
        assert all(
            routine["patrol"]["speed_range"] == [0.8, 0.8]
            for group in groups.values()
            for routine in group["routines"]
            if "patrol" in routine
        )


def test_parallel_spacing_is_geometry_only_and_exact():
    for scenario in ("A", "D"):
        for spacing in (1.5, 1.0, 0.75, 0.50):
            routes = dict(route_specs(scenario, spacing))
            first_x = routes["stress_a"][0][0]
            second_x = routes["stress_b"][0][0]
            assert abs(second_x - first_x) == spacing
    assert route_specs("A", 0.75)[1][1][0][1] < route_specs("A", 0.75)[1][1][1][1]
    assert route_specs("D", 0.75)[1][1][0][1] > route_specs("D", 0.75)[1][1][1][1]


def test_runner_keeps_baseline_parameters_and_records_every_required_layer():
    source = (
        SCRIPT_DIR / "run_custom_people_dr_spaam_crowded_tracking_stress.sh"
    ).read_text(encoding="utf-8")
    assert "ckpt_jrdb_ann_ft_dr_spaam_e20.pth" in source
    assert "-p conf_thresh:=0.95" in source
    assert "-p association_threshold:=0.8" in source
    assert "-p min_hits:=3" in source
    assert "-p max_age:=8" in source
    assert "-p max_coast_time:=0.75" in source
    assert "export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off" in source
    assert "export ISAAC_ROBOT_PHYSICS=0" in source
    assert "export ISAAC_LIDAR_MODE=physx" in source
    for topic in (
        "/scan_01",
        "/scan_02",
        "/scan_merged",
        "/dr_spaam_detections_scored",
        "/pedestrian_tracks",
        "/pedestrian_ground_truth",
        "/odom",
        "/tf",
        "/tf_static",
        "/clock",
    ):
        assert topic in source
    assert "STATIONARY_GUARD=PASS" in source


def test_normal_and_crowded_runners_share_sensor_detector_tracker_contract():
    normal_detector = (SCRIPT_DIR / "run_custom_people_dr_spaam_smoke.sh").read_text(
        encoding="utf-8"
    )
    normal_tracking = (
        SCRIPT_DIR / "run_custom_people_dr_spaam_tracking_smoke.sh"
    ).read_text(encoding="utf-8")
    crowded = (
        SCRIPT_DIR / "run_custom_people_dr_spaam_crowded_tracking_stress.sh"
    ).read_text(encoding="utf-8")

    shared_sensor_contract = (
        "export ISAAC_LIDAR_MODE=physx",
        "export ISAAC_LIDAR_RATE_HZ=15",
        "export ISAAC_LIDAR_SAMPLE_COUNT=2000",
        '"$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh"',
    )
    shared_detector_contract = (
        "ckpt_jrdb_ann_ft_dr_spaam_e20.pth",
        "-p detector_model:=DR-SPAAM",
        "-p conf_thresh:=0.95",
        "-p stride:=5",
        "-p panoramic_scan:=true",
        "-p reverse_scan:=true",
        "-p drow_to_ros:=true",
        "-p target_frame:=base_link",
        "-p subscriber.scan.topic:=/scan_merged",
    )
    shared_tracker_contract = (
        "pedestrian_point_tracker.py",
        "-p tracking_frame:=odom",
        "-p association_threshold:=0.8",
        "-p min_hits:=3",
        "-p max_age:=8",
        "-p max_coast_time:=0.75",
        "-p acceleration_sigma:=2.0",
        "-p measurement_sigma:=0.10",
        "-p max_prediction_dt:=0.50",
        "-p measurement_history_size:=8",
        "-p velocity_fit_min_samples:=3",
        "-p velocity_fit_min_span:=0.15",
    )
    for runner in (normal_detector, normal_tracking, crowded):
        for contract_line in shared_sensor_contract + shared_detector_contract:
            assert contract_line in runner
    for runner in (normal_tracking, crowded):
        for contract_line in shared_tracker_contract:
            assert contract_line in runner


def test_evaluator_exposes_detector_tracker_failure_split_and_distance_bins():
    source = (
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts"
        / "pedestrian_crowded_tracking_evaluator.py"
    ).read_text(encoding="utf-8")
    for label in ("<0.50", "0.50-0.75", "0.75-1.00", "1.00-1.50", ">1.50"):
        assert label in source
    assert "TRACKER_ASSOCIATION_FAILURE" in source
    assert "DETECTOR_MERGE_INDUCED_TRACK_BREAK" in source
    assert '"detector_merge_events"' in source
    assert '"crossing_id_switches"' in source
    assert '"ground_truth_used_by_detector_or_tracker": False' in source


def test_isaac_duration_starts_after_scene_and_people_initialization():
    source = (
        SCRIPT_DIR / "show_warehouse_people_robot_6_0.py"
    ).read_text(encoding="utf-8")
    assert "sim_time - started_sim_time >= ARGS.duration" in source


def test_official_physx_backend_uses_native_experimental_sensor():
    source = (
        SCRIPT_DIR / "show_warehouse_people_robot_6_0.py"
    ).read_text(encoding="utf-8")
    helper = (SCRIPT_DIR / "physx_lidar_people.py").read_text(encoding="utf-8")
    relay = (SCRIPT_DIR / "cmd_vel_udp_relay.py").read_text(encoding="utf-8")

    assert '"physx_raycast_sensor" if LIDAR_MODE == "physx"' in source
    assert '"isaacsim.sensors.experimental.physics.RaycastSensor"' in source
    assert "class PhysxDualLidarScheduler" in source
    assert "RaycastSensor" in source
    assert "def merge_native_physx_scan(" in source
    assert "native_ranges_m" in source
    assert "LIDAR_RANGE_MIN_M + candidate_distance_stage * scale" in source
    assert "self.physics_steps % self._capture_period_steps" in source
    assert "self._capture_sim_time = float(" in source
    assert "SimulationManager.get_simulation_time()" in source
    assert "self.ros.send_lidar_telemetry(" in source
    assert '"robot_pose": robot_pose' in source
    assert '"isaacsim.sensors.experimental.physics"' in source
    assert "ray_start_offsets_outside_box" in helper
    assert "is_ignored_robot_query_collider" in helper
    assert "LIDAR_TELEMETRY_SCHEMA" in relay
    assert "self.clock_pub.publish(clock)" in relay
    assert "self.publish_odometry(pose, self.actual_velocity)" in relay
