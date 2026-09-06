#!/usr/bin/env python3
"""Isaac Sim DR-SPAAM pedestrian perception visualization demo.

The launch starts the existing Isaac scene/dual-LiDAR runner, the existing
DR-SPAAM detector and tracker, and the new display-only visualizer.  It does
not start DRL-VO, navigation, or any controller.
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def project_root() -> Path:
    return Path(os.environ.get("NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5]))


def validate_configuration(context):
    root = project_root()
    paths = {
        "isaac_runner": root / "isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh",
        "dr_spaam": root
        / "github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2/dr_spaam_ros2/dr_spaam_w_score_ros.py",
        "checkpoint": root
        / "github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth",
        "train_python": root / ".venvs/train/bin/python",
    }
    start_map_server = str(LaunchConfiguration("start_map_server").perform(context)).lower()
    if start_map_server in {"1", "true", "yes", "on"}:
        map_yaml_value = str(LaunchConfiguration("map_yaml").perform(context)).strip()
        paths["map_yaml"] = Path(map_yaml_value).expanduser()
    scenario = str(LaunchConfiguration("scenario").perform(context))
    allowed = {"front_approach", "lateral", "C", "default"}
    if scenario not in allowed:
        raise ValueError(f"scenario must be one of {sorted(allowed)}, got {scenario!r}")
    scene = str(LaunchConfiguration("scene").perform(context)).strip().lower()
    if scene not in {"custom", "empty"}:
        raise ValueError(f"scene must be 'custom' or 'empty', got {scene!r}")
    if scene == "empty":
        paths["empty_scene"] = root / "isaac_sim/scenes/a_pipeline_empty_people.usda"
        paths["empty_route_generator"] = (
            root / "isaac_sim/scripts/generate_empty_field_people_config.py"
        )
    missing = [f"{name}={path}" for name, path in paths.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "pedestrian perception demo is missing required inputs: " + ", ".join(missing)
        )
    ira_config_value = str(LaunchConfiguration("ira_config").perform(context)).strip()
    ira_config = Path(ira_config_value).expanduser() if ira_config_value else None
    if ira_config is not None and not ira_config.is_file():
        raise FileNotFoundError(f"ira_config does not exist: {ira_config}")
    return [
        LogInfo(
            msg=(
                "[perception visualization only] scenario="
                f"{scenario} scene={scene}; no DRL-VO/navigation/controller is launched"
            )
        )
    ]


def clean_rviz_environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in tuple(environment):
        if name == "GTK_PATH" or name.startswith("GIO_") or name.startswith("SNAP"):
            environment.pop(name)
    # The Isaac bridge and the ROS nodes in this demo use loopback-only Fast DDS.
    # An explicit env dictionary otherwise preserves the shell's default
    # ROS_LOCALHOST_ONLY=0 and prevents RViz from discovering /map/markers.
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    return environment


def generate_launch_description() -> LaunchDescription:
    root = project_root()
    package_share = Path(get_package_share_directory("semantic_nav_gazebo"))
    rviz_config = package_share / "rviz" / "pedestrian_perception_visualization_demo.rviz"
    default_map_yaml = package_share / "maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml"
    train_python = root / ".venvs/train/bin/python"
    dr_spaam_root = root / "github_src/drl_vo_nav-drl_vo/2D_lidar_person_detection/dr_spaam"
    dr_spaam_ros_root = root / "github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2"
    dr_spaam_node = dr_spaam_ros_root / "dr_spaam_ros2/dr_spaam_w_score_ros.py"
    checkpoint = dr_spaam_ros_root / "model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
    isaac_runner = root / "isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh"
    python_path = ":".join(
        part
        for part in (str(dr_spaam_root), str(dr_spaam_ros_root), os.environ.get("PYTHONPATH", ""))
        if part
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene", default_value="empty"),
            DeclareLaunchArgument("scenario", default_value="front_approach"),
            DeclareLaunchArgument("ira_config", default_value=""),
            DeclareLaunchArgument("people_count", default_value="8"),
            DeclareLaunchArgument("pedestrian_speed", default_value="0.8"),
            DeclareLaunchArgument("pedestrian_seed", default_value="21"),
            # Use Isaac's stock warehouse-style reciprocal avoidance without
            # Social Force steering or the legacy stop/yield controller.
            DeclareLaunchArgument(
                "pedestrian_social_mode", default_value="native_avoidance"
            ),
            DeclareLaunchArgument("pedestrian_avoidance_mode", default_value="native"),
            DeclareLaunchArgument("fixed_speed", default_value="true"),
            DeclareLaunchArgument("ensure_all_directions", default_value="true"),
            DeclareLaunchArgument("isaac_duration", default_value="0"),
            DeclareLaunchArgument("ros_domain_id", default_value="81"),
            DeclareLaunchArgument("start_map_server", default_value="false"),
            DeclareLaunchArgument("map_yaml", default_value=str(default_map_yaml)),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=validate_configuration),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            Node(
                condition=IfCondition(LaunchConfiguration("start_map_server")),
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "yaml_filename": LaunchConfiguration("map_yaml"),
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(LaunchConfiguration("start_map_server")),
                        cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(LaunchConfiguration("start_map_server")),
                        cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                        output="screen",
                    )
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_map_server")),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
            ),
            ExecuteProcess(
                cmd=["bash", str(isaac_runner), "--duration", LaunchConfiguration("isaac_duration")],
                name="isaac_perception_scene",
                output="screen",
                additional_env={
                    "ISAAC_ROS_DOMAIN_ID": LaunchConfiguration("ros_domain_id"),
                    "ISAAC_SCENE": LaunchConfiguration("scene"),
                    "ISAAC_ENABLE_PEOPLE": "1",
                    "ISAAC_PEDESTRIAN_COUNT": LaunchConfiguration("people_count"),
                    "ISAAC_PEDESTRIAN_SEED": LaunchConfiguration("pedestrian_seed"),
                    "ISAAC_PEDESTRIAN_SPEED": LaunchConfiguration("pedestrian_speed"),
                    "ISAAC_PEDESTRIAN_SOCIAL_MODE": LaunchConfiguration(
                        "pedestrian_social_mode"
                    ),
                    "ISAAC_PEDESTRIAN_AVOIDANCE_MODE": LaunchConfiguration(
                        "pedestrian_avoidance_mode"
                    ),
                    "ISAAC_PEDESTRIAN_FIXED_SPEED": LaunchConfiguration(
                        "fixed_speed"
                    ),
                    "ISAAC_PEDESTRIAN_ENSURE_ALL_DIRECTIONS": LaunchConfiguration(
                        "ensure_all_directions"
                    ),
                    "ISAAC_ROBOT_PHYSICS": "0",
                    "ISAAC_LIDAR_MODE": "physx",
                    "ISAAC_LIDAR_RATE_HZ": "15",
                    "ISAAC_LIDAR_SAMPLE_COUNT": "2000",
                    "ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG": LaunchConfiguration("ira_config"),
                    "PYTHONPATH": python_path,
                },
            ),
            ExecuteProcess(
                cmd=[
                    str(train_python),
                    str(dr_spaam_node),
                    "--ros-args",
                    "-p",
                    f"weight_file:={checkpoint}",
                    "-p",
                    "detector_model:=DR-SPAAM",
                    "-p",
                    "conf_thresh:=0.95",
                    "-p",
                    "stride:=5",
                    "-p",
                    "panoramic_scan:=true",
                    "-p",
                    "reverse_scan:=true",
                    "-p",
                    "drow_to_ros:=true",
                    "-p",
                    "target_frame:=base_link",
                    "-p",
                    "subscriber.scan.topic:=/scan_merged",
                ],
                name="dr_spaam_detector",
                output="screen",
                additional_env={"PYTHONPATH": python_path},
            ),
            Node(
                package="semantic_nav_gazebo",
                executable="pedestrian_point_tracker.py",
                name="pedestrian_point_tracker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "tracking_frame": "odom",
                        "input_topic": "/dr_spaam_detections_scored",
                        "output_topic": "/pedestrian_tracks",
                        "association_threshold": 0.8,
                        "min_hits": 3,
                        "max_age": 8,
                        "max_coast_time": 0.75,
                        "acceleration_sigma": 2.0,
                        "measurement_sigma": 0.10,
                        "max_prediction_dt": 0.50,
                        "measurement_history_size": 8,
                        "velocity_fit_min_samples": 3,
                        "velocity_fit_min_span": 0.15,
                    }
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2_pedestrian_perception",
                output="screen",
                arguments=["-d", str(rviz_config), "-f", "odom"],
                env=clean_rviz_environment(),
            ),
            Node(
                package="semantic_nav_gazebo",
                executable="pedestrian_perception_visualizer.py",
                name="pedestrian_perception_visualizer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "tracks_topic": "/pedestrian_tracks",
                        "detections_topic": "/dr_spaam_detections_scored",
                        "ground_truth_topic": "/pedestrian_ground_truth",
                        "marker_topic": "/pedestrian_visualization",
                        "display_frame": "odom",
                    }
                ],
            ),
        ]
    )
