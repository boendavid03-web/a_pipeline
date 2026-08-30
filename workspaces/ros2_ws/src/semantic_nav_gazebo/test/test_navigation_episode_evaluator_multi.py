import json
import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import ActuationDecision, SimulatorActuationState
from semantic_nav_gazebo.msg import PedestrianStateArray
from std_msgs.msg import Empty

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from navigation_episode_evaluator import NavigationEpisodeEvaluator


def set_stamp(message, seconds):
    whole = int(seconds)
    message.header.stamp.sec = whole
    message.header.stamp.nanosec = int(round((seconds - whole) * 1e9))


def goal_message(x, y, seconds, frame_id="map"):
    message = PointStamped()
    set_stamp(message, seconds)
    message.header.frame_id = frame_id
    message.point.x = x
    message.point.y = y
    return message


def odom_message(x, y, seconds, frame_id="map"):
    message = Odometry()
    set_stamp(message, seconds)
    message.header.frame_id = frame_id
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    return message


def pedestrian_message(x=None, y=None, seconds=0.0, frame_id="map"):
    message = PedestrianStateArray()
    set_stamp(message, seconds)
    message.header.frame_id = frame_id
    if x is not None and y is not None:
        person = message.pedestrians.add() if hasattr(message.pedestrians, "add") else None
        if person is None:
            from semantic_nav_gazebo.msg import PedestrianState
            person = PedestrianState()
            message.pedestrians.append(person)
        person.id = "p1"
        person.pose.position.x = x
        person.pose.position.y = y
    return message


def decision_message(seconds, sequence, raw, final, gated=False):
    message = ActuationDecision()
    set_stamp(message, seconds)
    message.has_raw_action = True
    message.decision_sequence_id = sequence
    message.inference_sequence_id = sequence
    message.raw_physical_action.linear.x = raw
    message.raw_physical_action.angular.z = 0.1 * raw
    message.final_command.linear.x = final
    message.final_command.angular.z = 0.1 * final
    message.gated = gated
    message.gate_reasons = ["linear_limit"] if gated else []
    return message


def actuation_state_message(seconds, sequence, command, actual):
    message = SimulatorActuationState()
    set_stamp(message, seconds)
    message.telemetry_sequence_id = sequence
    message.command_received = True
    message.command_sequence_id = sequence
    message.bridge_receive_stamp = message.header.stamp
    message.received_command.linear.x = command
    message.applied_command.linear.x = command
    message.actual_velocity.linear.x = actual
    message.received_command.angular.z = 0.1 * command
    message.applied_command.angular.z = 0.1 * command
    message.actual_velocity.angular.z = 0.1 * actual
    message.actual_velocity_source = "physx_rigid_body_api"
    message.command_age_sec = 0.01
    return message


def test_multi_episode_rolls_over_and_indexes_outputs(tmp_path, monkeypatch):
    output_root = tmp_path / "evaluation_session"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"evaluation_output_dir:={output_root}",
            "-p",
            "evaluation_multi_episode:=true",
        ]
    )
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(3.0, 0.0, 10.0))
        node.odom_callback(odom_message(0.0, 0.0, 10.5))

        node.goal_callback(goal_message(0.0, 0.0, 11.0))
        node.odom_callback(odom_message(0.0, 0.0, 11.5))

        first = json.loads(
            (output_root / "episode_0001" / "episode_summary.json").read_text()
        )
        second = json.loads(
            (output_root / "episode_0002" / "episode_summary.json").read_text()
        )
        session = json.loads((output_root / "session_summary.json").read_text())

        assert first["episode_sequence"] == {
            "index": 1,
            "multi_episode": True,
        }
        assert first["episode"]["termination_reason"] == "superseded_by_new_goal"
        assert first["schema"]["version"] == 5
        assert first["episode"]["strict_success_proxy"] is False
        assert first["episode"]["success"] is False
        assert set(("goal_reached", "termination_reason", "navigation_time_sec")) <= set(first["episode"])
        assert len(first["metric_catalog"]) == 26
        assert {item["status"] for item in first["metric_catalog"].values()} == {
            "available", "provisional", "proxy", "unavailable", "not_applicable"
        }
        assert set(first["metric_catalog"]) == {
            "Success", "Timeout", "Path Length", "Run Time", "SPL",
            "Static Obstacle Collision", "Velocity Metrics", "Speed Efficiency",
            "Acceleration Metrics", "Jerk Metrics", "Obstacle Distance",
            "Path Irregularity", "Topological Complexity", "Path Efficiency",
            "Failure To Progress", "Human Collision", "Social Distance",
            "Min Time To Collide", "Crowd Density", "Virtual Collision",
            "Personal Space", "Legibility", "Predictability", "Projected Path",
            "Following Rate", "SPS",
        }
        def present(payload, path):
            value = payload
            for part in path.split("."):
                assert isinstance(value, dict) and part in value
                value = value[part]
        for entry in first["metric_catalog"].values():
            paths = entry["output_path"]
            if isinstance(paths, str):
                paths = [paths]
            for path in paths or []:
                present(first, path)
            assert entry["output_paths"] == (paths or [])
        assert first["collision"]["occurred"] is None
        assert first["collision"]["status"] == "proxy"
        assert first["collision"]["human"] is None
        assert first["static_clearance"]["threshold_exposure"]["threshold_m"] == 0.0
        assert first["human_clearance"]["threshold_exposure"]["threshold_m"] == 0.0
        assert "virtual_collision_proxy" in first
        json.dumps(first, allow_nan=False)
        assert first["experiment"]["accepted_goal"] == [3.0, 0.0]
        assert second["episode_sequence"]["index"] == 2
        assert second["episode"]["termination_reason"] == "goal_tolerance_reached"
        assert second["experiment"]["accepted_goal"] == [0.0, 0.0]
        assert session["episode_count"] == 2
        assert [item["directory"] for item in session["episodes"]] == [
            "episode_0001",
            "episode_0002",
        ]
        assert "aggregate_metrics" in session
        assert "aggregate_inference_p95_ms" in session["aggregate_metrics"]
        assert "blocks" in session["aggregate_metrics"]["velocity_tracking"]
        assert "angular_final_command_to_actual_velocity" in session["aggregate_metrics"]["velocity_tracking"]["blocks"]
        metrics = session["aggregate_metrics"]["metrics"]
        computed = {name for name, entry in first["metric_catalog"].items()
                    if entry["status"] in {"available", "proxy", "provisional"}}
        assert set(metrics) == computed
        for name in computed:
            for path in first["metric_catalog"][name]["output_paths"]:
                output = metrics[name]["outputs"][path]
                assert {"available", "missing", "coverage", "by_success"} <= set(output)
                assert set(output["by_success"]) == {"success", "failure", "unknown"}
        assert metrics["Success"]["outputs"]["episode.success"]["by_success"]["failure"]["available"] == 1
        assert metrics["Success"]["outputs"]["episode.success"]["by_success"]["unknown"]["missing"] == 1
        assert metrics["Static Obstacle Collision"]["outputs"]["collision.static_proxy"]["coverage"] == 0.0
        node._completed_latency_samples[:] = [1.0, 2.0, 100.0]
        node._write_session_summary()
        session = json.loads((output_root / "session_summary.json").read_text())
        assert session["aggregate_metrics"]["aggregate_inference_p95_ms"] == pytest.approx(90.2)
        assert session["aggregate_metrics"]["inference_latency_ms"]["p95"] == pytest.approx(90.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_crowd_density_is_people_per_square_meter(tmp_path, monkeypatch):
    output_root = tmp_path / "density_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "density_ros_logs"))
    rclpy.init(args=["--ros-args", "-p", f"evaluation_output_dir:={output_root}"])
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(5.0, 0.0, 1.0))
        node.pedestrian_callback(pedestrian_message(1.0, 0.0, 1.5))
        node.odom_callback(odom_message(0.0, 0.0, 2.0))
        assert node.crowd_density_samples[-1][1] == pytest.approx(1.0 / (4.0 * 3.141592653589793))
        node.pedestrian_callback(pedestrian_message(seconds=2.5))
        node.odom_callback(odom_message(0.1, 0.0, 3.0))
        assert node.crowd_density_samples[-1][1] == 0.0
        node.odom_callback(odom_message(0.2, 0.0, 4.0))
        node.finish("test_complete", 4.0)
        summary = json.loads((output_root / "episode_summary.json").read_text())
        assert summary["virtual_collision_proxy"]["entry_count"] == 0
        assert summary["collision"]["virtual_proxy"] is False
        assert summary["personal_space"]["violation_time_ratio"] == 0.0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_pedestrian_sync_quality_and_nonfinite_rejection(tmp_path, monkeypatch):
    output_root = tmp_path / "sync_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "sync_ros_logs"))
    rclpy.init(args=["--ros-args", "-p", f"evaluation_output_dir:={output_root}"])
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(5.0, 0.0, 10.0))
        node.pedestrian_callback(pedestrian_message(1.0, 0.0, 10.0, "map"))
        node.odom_callback(odom_message(0.0, 0.0, 10.1, "map"))
        node.odom_callback(odom_message(0.1, 0.0, 10.7, "map"))
        node.pedestrian_callback(pedestrian_message(seconds=12.0, frame_id="map"))
        node.odom_callback(odom_message(0.2, 0.0, 11.5, "map"))
        node.pedestrian_callback(pedestrian_message(seconds=11.4, frame_id="other"))
        node.odom_callback(odom_message(0.3, 0.0, 11.5, "map"))
        node.pedestrian_callback(pedestrian_message(seconds=11.5, frame_id="map"))
        node.odom_callback(odom_message(0.4, 0.0, 11.6, "map"))
        bad_odom = odom_message(0.5, 0.0, 11.7, "map")
        bad_odom.pose.pose.position.x = float("nan")
        node.odom_callback(bad_odom)
        bad_cmd = Twist()
        bad_cmd.linear.x = float("nan")
        node.cmd_callback(bad_cmd)
        node.finish("test_complete", 12.0)
        summary = json.loads((output_root / "episode_summary.json").read_text())
        quality = summary["data_quality"]
        # Duplicate timestamps are invalid for derivatives/path integration.
        assert quality["accepted_odom"] == 4
        assert quality["rejected_nonpositive_odom_timestamp"] == 1
        assert quality["synchronized_pedestrian_snapshots"] == 2
        assert quality["stale_pedestrian_snapshots"] == 1
        assert quality["future_pedestrian_snapshots"] == 1
        # The same callback has a duplicate timestamp and is rejected before
        # any secondary pedestrian-frame accounting.
        assert quality["frame_mismatch_pedestrian_snapshots"] == 0
        assert quality["rejected_nonfinite_odom"] == 1
        assert quality["rejected_nonfinite_cmd"] == 1
        assert quality["pedestrian_sync_coverage"] == pytest.approx(2.0 / 4.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_single_episode_preserves_flat_output_and_ignores_later_goal(
    tmp_path, monkeypatch
):
    output_root = tmp_path / "single_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "single_ros_logs"))
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"evaluation_output_dir:={output_root}",
        ]
    )
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(1.0, 0.0, 20.0))
        node.goal_callback(goal_message(2.0, 0.0, 20.5))
        node.odom_callback(odom_message(1.0, 0.0, 21.0))

        summary = json.loads((output_root / "episode_summary.json").read_text())
        assert summary["experiment"]["accepted_goal"] == [1.0, 0.0]
        assert summary["episode_sequence"] == {
            "index": 1,
            "multi_episode": False,
        }
        assert not (output_root / "episode_0001").exists()
        assert not (output_root / "session_summary.json").exists()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_episode_reset_is_an_explicit_episode_boundary(tmp_path, monkeypatch):
    output_root = tmp_path / "reset_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "reset_ros_logs"))
    rclpy.init(args=["--ros-args", "-p", f"evaluation_output_dir:={output_root}"])
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(5.0, 0.0, 1.0))
        node.odom_callback(odom_message(0.0, 0.0, 1.1))
        node.episode_reset_callback(Empty())
        summary = json.loads((output_root / "episode_summary.json").read_text())
        assert summary["episode"]["termination_reason"] == "episode_reset"
        assert summary["data_quality"]["episode_reset_events"] == 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_reset_jump_is_not_integrated_as_extreme_speed_or_path(tmp_path, monkeypatch):
    output_root = tmp_path / "jump_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "jump_ros_logs"))
    rclpy.init(args=["--ros-args", "-p", f"evaluation_output_dir:={output_root}"])
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(100.0, 0.0, 1.0))
        node.odom_callback(odom_message(0.0, 0.0, 1.1))
        jumped = odom_message(23.6018, 0.0, 1.135)
        jumped.twist.twist.linear.x = 472.0
        node.odom_callback(jumped)
        summary = json.loads((output_root / "episode_summary.json").read_text())
        assert summary["episode"]["termination_reason"] == "sim_reset_detected"
        assert summary["data_quality"]["reset_jump_detected"] == 1
        assert summary["navigation"]["path_length_m"] == 0.0
        assert summary["navigation"]["path_irregularity"]["turning_rad_per_m"] is None
        assert summary["speed"]["max_mps"] is None
        assert summary["data_quality"]["pose_derived_invalid_reset_or_teleport"] == 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_actuation_episode_and_session_schema_contains_three_stage_metrics(
    tmp_path, monkeypatch
):
    output_root = tmp_path / "actuation_evaluation"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "actuation_ros_logs"))
    rclpy.init(args=["--ros-args", "-p", f"evaluation_output_dir:={output_root}"])
    node = NavigationEpisodeEvaluator()
    try:
        node.goal_callback(goal_message(5.0, 0.0, 1.0))
        position_x = 0.0
        node.odom_callback(odom_message(position_x, 0.0, 1.0))
        for index, (raw, command, actual) in enumerate(
            ((0.8, 0.7, 0.6), (0.5, 0.5, 0.45), (0.2, 0.2, 0.18)), start=1
        ):
            stamp = 1.0 + index / 15.0
            node.actuation_decision_callback(
                decision_message(stamp, index, raw, command, gated=index == 1)
            )
            node.simulator_actuation_callback(
                actuation_state_message(stamp, index, command, actual)
            )
            position_x += actual / 15.0
            node.odom_callback(odom_message(position_x, 0.0, stamp))
        node.finish("test_complete", 2.0)
        summary = json.loads((output_root / "episode_summary.json").read_text())
        tracking = summary["velocity_tracking"]
        assert tracking["actual_velocity_sources"] == ["pose_derived_velocity"]
        assert tracking["raw_model_to_final_command"]["sample_count"] == 3
        assert tracking["final_command_to_actual_velocity"]["sample_count"] == 3
        assert tracking["angular_final_command_to_actual_velocity"]["sample_count"] == 3
        assert tracking["safety_gated"]["sample_count"] == 1
        assert tracking["safety_ungated"]["sample_count"] == 2
        assert tracking["physx_reported_velocity_diagnostic"][
            "actual_velocity_sources"
        ] == ["physx_rigid_body_api"]
        assert tracking["physx_reported_velocity_diagnostic"][
            "final_command_to_physx_reported_velocity"
        ]["sample_count"] == 3
        assert summary["schema"]["version"] == 5
        alignment = (output_root / "actuation_alignment.csv").read_text()
        assert "received_command_angular_z_radps" in alignment
        assert "simulator_gated" in alignment
        assert (output_root / "physx_actuation_alignment.csv").is_file()
        assert "pose_derived_velocity" in summary["velocity_tracking"]["source"]
        assert summary["navigation"]["pose_integration_consistency"]["consistent"]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
