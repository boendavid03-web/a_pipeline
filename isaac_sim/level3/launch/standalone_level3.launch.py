#!/usr/bin/env python3
"""Standalone existing-map Nav2 bringup for the Isaac custom lobby.

This launch intentionally contains no AMCL, SLAM, Arena, RViz, simulator, or
robot-asset process.  Isaac and its ROS/UDP bridge must already be running.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LEVEL3_ROOT = Path(__file__).resolve().parents[1]
PARAMS_FILE = LEVEL3_ROOT / "config/nav2_level3.yaml"
ALIGNMENT_FILE = LEVEL3_ROOT / "config/map_alignment.yaml"


def fail_if_missing(path: Path) -> None:
    if not path.is_file():
        raise RuntimeError(f"required Level 3 file is missing: {path}")


def generate_launch_description() -> LaunchDescription:
    fail_if_missing(PARAMS_FILE)
    fail_if_missing(ALIGNMENT_FILE)
    alignment = yaml.safe_load(ALIGNMENT_FILE.read_text(encoding="utf-8"))
    transform = alignment["map_to_odom"]
    log_level = LaunchConfiguration("log_level")
    def stop_stack() -> Shutdown:
        return Shutdown(reason="a required standalone Level 3 process exited")
    common_arguments = ["--ros-args", "--log-level", log_level]

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="level3_ground_truth_map_to_odom",
            output="screen",
            arguments=[
                "--x", str(transform["x_m"]),
                "--y", str(transform["y_m"]),
                "--z", str(transform["z_m"]),
                "--yaw", str(transform["yaw_rad"]),
                "--pitch", str(transform["pitch_rad"]),
                "--roll", str(transform["roll_rad"]),
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            remappings=[("cmd_vel", "cmd_vel_nav")],
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            remappings=[("cmd_vel", "cmd_vel_nav")],
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[str(PARAMS_FILE)],
            arguments=common_arguments,
            remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            on_exit=stop_stack(),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_level3",
            output="screen",
            arguments=common_arguments,
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "bond_timeout": 4.0,
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                        "behavior_server",
                        "bt_navigator",
                        "velocity_smoother",
                    ],
                }
            ],
            on_exit=stop_stack(),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            *nodes,
        ]
    )
