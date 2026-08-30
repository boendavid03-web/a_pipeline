#!/usr/bin/env python3
"""Isaac/Gazebo navigation infrastructure with CALF as the sole policy."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_root = Path(
        os.environ.get(
            "NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5]
        )
    )
    package_share = Path(get_package_share_directory("semantic_nav_gazebo"))
    infrastructure_launch = package_share / "launch" / "drl_vo_fixed_dual_start_goal_demo.launch.py"
    calf_root = project_root / "github_src/drl_vo_nav-drl_vo/LegNav-Sim-master"
    default_checkpoint = calf_root / "checkpoints/ppo/ppo_legs_best.msgpack"
    default_python = project_root / ".venvs/calf_ros2/bin/python"
    default_map = (
        project_root
        / "runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml"
    )

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_simulator", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("map_yaml", default_value=str(default_map)),
        DeclareLaunchArgument("robot_x", default_value="2.0"),
        DeclareLaunchArgument("robot_y", default_value="2.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="6.0"),
        DeclareLaunchArgument("goal_y", default_value="4.0"),
        DeclareLaunchArgument("auto_set_initial_goal", default_value="true"),
        DeclareLaunchArgument("enable_goal_picker", default_value="false"),
        DeclareLaunchArgument("fixed_test", default_value="false"),
        DeclareLaunchArgument("lookahead", default_value="1.0"),
        DeclareLaunchArgument("inflate_radius", default_value="0.45"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.35"),
        DeclareLaunchArgument("show_actual_trajectory", default_value="true"),
        DeclareLaunchArgument("record_trace", default_value="true"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("trace_timeout_sec", default_value="300.0"),
        DeclareLaunchArgument("evaluate_episode", default_value="false"),
        DeclareLaunchArgument("evaluation_output_dir", default_value=""),
        DeclareLaunchArgument("evaluation_timeout_sec", default_value="300.0"),
        DeclareLaunchArgument("calf_checkpoint", default_value=str(default_checkpoint)),
        DeclareLaunchArgument("calf_python", default_value=str(default_python)),
        DeclareLaunchArgument("calf_trace_path", default_value=""),
        DeclareLaunchArgument("max_linear", default_value="0.8"),
        DeclareLaunchArgument("scan_timeout", default_value="0.75"),
        DeclareLaunchArgument("odom_timeout", default_value="0.5"),
        DeclareLaunchArgument("subgoal_timeout", default_value="0.5"),
    ]

    infrastructure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(infrastructure_launch)),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_simulator": LaunchConfiguration("start_simulator"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "start_drl_vo_policy": "false",
            "start_online_ppo_training": "false",
            "publish_policy_actions": "false",
            "require_pedestrian_truth": "false",
            "pedestrian_source": "zero",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "robot_x": LaunchConfiguration("robot_x"),
            "robot_y": LaunchConfiguration("robot_y"),
            "robot_yaw": LaunchConfiguration("robot_yaw"),
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "auto_set_initial_goal": LaunchConfiguration("auto_set_initial_goal"),
            "enable_goal_picker": LaunchConfiguration("enable_goal_picker"),
            "fixed_test": LaunchConfiguration("fixed_test"),
            "lookahead": LaunchConfiguration("lookahead"),
            "inflate_radius": LaunchConfiguration("inflate_radius"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            "show_actual_trajectory": LaunchConfiguration("show_actual_trajectory"),
            "record_trace": LaunchConfiguration("record_trace"),
            "trace_path": LaunchConfiguration("trace_path"),
            "trace_timeout_sec": LaunchConfiguration("trace_timeout_sec"),
            "evaluate_episode": LaunchConfiguration("evaluate_episode"),
            "evaluation_output_dir": LaunchConfiguration("evaluation_output_dir"),
            "evaluation_timeout_sec": LaunchConfiguration("evaluation_timeout_sec"),
            "cmd_vel_topic": "/cmd_vel",
        }.items(),
    )

    policy = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="semantic_nav_gazebo",
                executable="calf_policy_node.py",
                name="calf_policy",
                output="screen",
                prefix=[LaunchConfiguration("calf_python")],
                additional_env={
                    "JAX_PLATFORMS": "cpu",
                    "PYTHONPATH": str(calf_root)
                    + os.pathsep
                    + os.environ.get("PYTHONPATH", ""),
                },
                on_exit=[Shutdown(reason="CALF policy node exited")],
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "checkpoint": LaunchConfiguration("calf_checkpoint"),
                        "max_linear": ParameterValue(
                            LaunchConfiguration("max_linear"), value_type=float
                        ),
                        "scan_timeout": ParameterValue(
                            LaunchConfiguration("scan_timeout"), value_type=float
                        ),
                        "odom_timeout": ParameterValue(
                            LaunchConfiguration("odom_timeout"), value_type=float
                        ),
                        "subgoal_timeout": ParameterValue(
                            LaunchConfiguration("subgoal_timeout"), value_type=float
                        ),
                        "trace_path": LaunchConfiguration("calf_trace_path"),
                    }
                ],
            )
        ],
    )
    return LaunchDescription(arguments + [infrastructure, policy])
