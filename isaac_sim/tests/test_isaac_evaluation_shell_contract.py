from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "isaac_sim/scripts/run_custom_people_drlvo_demo.sh"
ISAAC_RUNNER = ROOT / "isaac_sim/scripts/show_warehouse_people_robot_6_0.py"
WAREHOUSE_LAUNCHER = (
    ROOT / "isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh"
)
ROS_BRIDGE = ROOT / "isaac_sim/scripts/cmd_vel_udp_relay.py"


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
    assert "Isaac failed while waiting for actual velocity telemetry" in source
    assert "grep '\\[WAREHOUSE-ROBOT\\] ERROR:'" in source
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
    assert "if (( ISAAC_LIDAR_RATE_HZ != 15 )); then" in source
    assert "DRL-VO dual-LiDAR input is fixed at 15 Hz" in source
    assert "if (( ISAAC_LIDAR_SAMPLE_COUNT != 2000 )); then" in source
    assert "fixed at 2000 beams per sensor" in source
    assert "--verify-lidar-rate --require-realtime-lidar" in source


def test_physx_lidar_uses_physics_clock_and_dedicated_timestamped_messages():
    source = ISAAC_RUNNER.read_text(encoding="utf-8")
    bridge = ROS_BRIDGE.read_text(encoding="utf-8")

    assert "class PhysxDualLidarScheduler" in source
    assert 'enable_extension("isaacsim.sensors.experimental.physics")' in source
    assert "Raycast.create(" in source
    assert "RaycastSensor(authoring)" in source
    assert "ray_origins=ray_origins" in source
    assert "ray_directions=ray_directions" in source
    assert "report_hit_prim_paths=True" in source
    assert 'prim.GetAttribute("enabled").Set(enabled)' in source
    assert "self.physics_steps % self.steps_per_capture == 0" in source
    assert "reading.depths" in source
    assert "reading.hit_prim_paths" in source
    assert "native PhysX raycast reading was stale/replayed" in source
    assert "native PhysX raycast cadence is not 15 Hz" in source
    assert "native PhysX front/rear readings were not paired" in source
    assert "native PhysX reading time is not current" not in source
    assert "merge_native_physx_scan(" in source
    assert "ThreadPoolExecutor" not in source
    assert "raycast_closest" not in source
    assert "raycast_all" in source
    assert "event=IsaacEvents.PRE_PHYSICS_STEP" in source
    assert "event=IsaacEvents.POST_PHYSICS_STEP" in source
    assert "self.physics_sim_time += step_dt" in source
    assert "self.ros.send_lidar_telemetry(sim_time, scans)" in source
    assert 'LIDAR_TELEMETRY_SCHEMA = "isaac_6_lidar_telemetry/v1"' in source
    assert "missed_periods" in source
    assert "Dual-lidar simulation/wall rate validation failed" in source
    assert 'LIDAR_TELEMETRY_SCHEMA = "isaac_6_lidar_telemetry/v1"' in bridge
    assert "self.handle_lidar_telemetry(payload)" in bridge
    assert "self.pending_lidar.append((sim_time, scans))" in bridge
    assert "self.publish_ready_lidar(sim_time)" in bridge


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


def test_gazebo_social_mode_uses_persistent_full_2d_follow_target():
    launcher = LAUNCHER.read_text(encoding="utf-8")
    warehouse_launcher = WAREHOUSE_LAUNCHER.read_text(encoding="utf-8")
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
    assert '"actual_navigation_velocity_source": "pose_derived_position_delta"' in adapter
    assert '"behavior_agent_reported_navigation_velocity_mps"' in adapter
    assert "authored_midpoint_mps" in adapter
    assert ".get_target_location()" in adapter
    assert ".set_speed(" in adapter
    assert ".follow(" in adapter
    assert "steering_target_from_velocity(" in adapter
    assert "guard.segment_world_free(" in adapter
    assert "steering_free_space_guard = free_space_guard" in source
    assert "ISAAC_PEDESTRIAN_OPPOSED_PAIR_TEST" in warehouse_launcher
    assert "--opposed-pair-test" in warehouse_launcher
    assert "CUSTOM_FREE_SPACE_CLEARANCE_M" in source
    assert '"locomotion_target_free_space_constrained"' in adapter
    assert '"free_space_constrained_target_count"' in adapter
    assert "output.final_desired_velocity_mps" in adapter
    assert "[float(point[0]), float(point[1]), float(point[2])]" in adapter
    assert '"lateral_vector_applied_directly": True' in adapter
    assert '"patrol_execution": "persistent_follow_moving_target"' in adapter
    assert '"stock_per_waypoint_autobrake": False' in adapter
    assert '"trace_path": PEDESTRIAN_SOCIAL_TRACE_PATH or None' in adapter
    assert ".move_along(" not in adapter
    assert ".dodge(" not in adapter
    assert '"patrol_task_replacement": True' in adapter
    assert 'agent.set_auto_avoidance_enabled(social_mode == "legacy")' in source
    assert '"behavior_agent_persistent_follow_target_2d"' in source
    assert 'social_motion["target_write_count"] <= result["people"]' in source
    assert 'result["pedestrian_robot_dodge_count"] != 0' in source
