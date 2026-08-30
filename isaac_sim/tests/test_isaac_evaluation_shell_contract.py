from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "isaac_sim/scripts/run_custom_people_drlvo_demo.sh"
ISAAC_RUNNER = ROOT / "isaac_sim/scripts/show_warehouse_people_robot_6_0.py"


def test_launcher_records_complete_evaluation_contract_and_never_mass_kills():
    source = LAUNCHER.read_text(encoding="utf-8")
    for topic in (
        "/drl_vo/actuation_decision",
        "/isaac/actuation_state",
        "/isaac/reset_pose",
        "/isaac/reset_event",
        "/data_collection/goal_accepted",
        "/semantic_cnn/global_path",
        "/data_collection/sensor_config",
        "/clock",
        "/tf_static",
    ):
        assert topic in source
    assert "relocation_backend:=isaac_pose_topic" in source
    assert "pkill " not in source
    assert 'if ! mkdir "$log_dir"' in source
    assert "actual_velocity_source" in source
    assert "export ISAAC_MANUAL_EPISODE_EVENTS=0" in source
    assert 'ISAAC_DEMO_ACTUATION_SOURCE_TIMEOUT:-180.0' in source
    assert 'timeout "$demo_actuation_source_timeout" ros2 topic echo /isaac/actuation_state' in source
    assert '| tr -d "\'\\\"" \\\n        || true' in source
    assert 'printf "%.9f", value' in source
    assert '-p capture_duration_sec:="$demo_capture_duration_sec"' in source
    scheduler_error = source.index("automatic capture scheduler exited with status")
    visualization = source.index('"$TRAJECTORY_VISUALIZER" "${visualization_args[@]}"')
    assert scheduler_error < visualization
    assert 'kill -INT -- "-$bag_pid"' in source
    assert 'wait "$bag_pid"' in source
    assert '"$TRAJECTORY_VISUALIZER" "${visualization_args[@]}"' in source
    for artifact in (
        "visualization/episode_index.html",
        "visualization.log",
    ):
        assert artifact in source


def test_dual_lidar_policy_contract_remains_2000_beams_at_15_hz():
    source = LAUNCHER.read_text(encoding="utf-8")
    assert "export ISAAC_LIDAR_SAMPLE_COUNT=2000" in source
    assert 'export ISAAC_LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-15}"' in source


def test_dynamic_velocity_command_is_applied_at_physx_rate():
    source = ISAAC_RUNNER.read_text(encoding="utf-8")
    assert "event=IsaacEvents.PRE_PHYSICS_STEP" in source
    assert "collision_proxy.set_dynamic_command(command)" in source
    assert "collision_proxy.apply_dynamic_command(command, dynamic_target_yaw)" not in source
    assert "collision_proxy.close_dynamic_controller()" in source
    assert '"test_command_velocity_tracking": test_velocity_tracking' in source
    assert "physics_material.CreateStaticFrictionAttr().Set(0.0)" in source
    assert "physics_material.CreateDynamicFrictionAttr().Set(0.0)" in source
    assert 'CreateFrictionCombineModeAttr("min")' in source


def test_timing_catchup_and_pose_truth_are_explicitly_instrumented():
    source = ISAAC_RUNNER.read_text(encoding="utf-8")
    assert '"ISAAC_MIN_SIMULATION_FRAME_RATE_HZ"' in source
    assert '"/persistent/simulation/minFrameRate"' in source
    assert '"true" if ARGS.deterministic else "false"' in source
    assert '"physics_steps_per_expected_timeline_step"' in source
    assert '"physics_steps_main_loop"' in source
    assert '"timeline_elapsed_sec"' in source
    assert '"pose_derived_velocity"' in source
    assert '"physx_reported_velocity"' in source
    assert '"source": "pose_derived_velocity"' in source
    assert '"--app-update-rate-limit-hz"' in source
    assert "navigation_yaw_unwrapped += math.atan2(" in source
    assert '"robot_final_yaw_unwrapped_ros_rad": navigation_yaw_unwrapped' in source


def test_gazebo_social_mode_is_opt_in_and_does_not_replace_patrol_tasks():
    launcher = LAUNCHER.read_text(encoding="utf-8")
    source = ISAAC_RUNNER.read_text(encoding="utf-8")
    adapter = source[
        source.index("class BehaviorAgentSocialMotion") : source.index(
            "class PedestrianRobotAvoidance"
        )
    ]

    assert 'ISAAC_PEDESTRIAN_SOCIAL_MODE:-legacy' in launcher
    assert 'legacy|gazebo_social' in launcher
    assert 'PEDESTRIAN_SOCIAL_MODE == "gazebo_social"' in source
    assert "agent.get_linear_velocity(True)" in adapter
    assert ".get_speed()" in adapter
    assert "agent.get_target_location()" in adapter
    assert ".set_speed(" in adapter
    assert ".move_to(" not in adapter
    assert ".move_along(" not in adapter
    assert ".dodge(" not in adapter
    assert '"patrol_task_replacement": False' in adapter
    assert 'agent.set_auto_avoidance_enabled(social_mode == "legacy")' in source
