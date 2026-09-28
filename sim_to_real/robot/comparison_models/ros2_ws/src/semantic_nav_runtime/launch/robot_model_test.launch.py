"""One external robot interface with a selectable policy backend."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def selected(name: str) -> IfCondition:
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration("method"), "' == '", name, "'"])
    )


def generate_launch_description():
    root_text = os.environ.get("ROBOT_COMPARISON_MODELS_ROOT")
    if not root_text:
        raise RuntimeError("use real_robot_model_test.py to set the model root")
    checkpoints = Path(root_text).resolve() / "checkpoints"

    return LaunchDescription([
        DeclareLaunchArgument("method"),
        DeclareLaunchArgument("scan_01_topic", default_value="/scan_01"),
        DeclareLaunchArgument("scan_02_topic", default_value="/scan_02"),
        DeclareLaunchArgument("tf_pose_topic", default_value="/sim_to_real/tf_odom"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("goal_x", default_value="-0.671"),
        DeclareLaunchArgument("goal_y", default_value="1.778"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        Node(
            package="semantic_nav_runtime",
            executable="fixed_global_goal_bridge.py",
            name="fixed_global_goal",
            output="screen",
            parameters=[{
                "global_frame": LaunchConfiguration("global_frame"),
                "world_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "publish_rate": 10.0,
                "pose_topic": LaunchConfiguration("tf_pose_topic"),
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="fixed_dual_scan_adapter.py",
            name="semantic_cnn_dual_scan_adapter",
            output="screen",
            condition=selected("semantic_cnn"),
            parameters=[{
                "input_scan_01_topic": LaunchConfiguration("scan_01_topic"),
                "input_scan_02_topic": LaunchConfiguration("scan_02_topic"),
                "output_scan_01_topic": "/sim_to_real/aligned/scan_01",
                "output_scan_02_topic": "/sim_to_real/aligned/scan_02",
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="s3net_fixed_dual_inference_node.py",
            name="s3net_backend",
            output="screen",
            condition=selected("semantic_cnn"),
            parameters=[{
                "model": str(checkpoints / "s3net" / "s3net_native_stats_best_dev.pth"),
                "model_code": str(checkpoints / "s3net"),
                "stats_json": str(checkpoints / "s3net" / "s3net_native_lidar_train_stats.json"),
                "scan_01_topic": "/sim_to_real/aligned/scan_01",
                "scan_02_topic": "/sim_to_real/aligned/scan_02",
                "base_frame": LaunchConfiguration("base_frame"),
                "enforce_message_layout": True,
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="semantic_cnn_fixed_dual_inference_node.py",
            name="semantic_cnn_backend",
            output="screen",
            condition=selected("semantic_cnn"),
            parameters=[{
                "model": str(checkpoints / "semantic_cnn" / "semantic_cnn_native_cmd_best_dev.pth"),
                "model_code": str(checkpoints / "semantic_cnn"),
                "scan_01_topic": "/sim_to_real/aligned/scan_01",
                "scan_02_topic": "/sim_to_real/aligned/scan_02",
                "use_online_s3net": True,
                "s3net_labels_topic": "/s3net/labels",
                "odom_topic": LaunchConfiguration("tf_pose_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="v7_dual_laser_scan_merger.py",
            name="drl_vo_scan_merger",
            output="screen",
            condition=selected("drl_vo"),
            parameters=[{
                "input_scan_01_topic": LaunchConfiguration("scan_01_topic"),
                "input_scan_02_topic": LaunchConfiguration("scan_02_topic"),
                "output_topic": "/scan_merged",
                "output_frame": LaunchConfiguration("base_frame"),
                "enable_self_filter": True,
            }],
        ),
        Node(
            package="dr_spaam_ros2",
            executable="dr_spaam_w_score_ros",
            name="dr_spaam_backend",
            output="screen",
            condition=selected("drl_vo"),
            parameters=[{
                "weight_file": str(checkpoints / "dr_spaam" / "ckpt_jrdb_ann_ft_dr_spaam_e20.pth"),
                "detector_model": "DR-SPAAM",
                "device": "auto",
                "panoramic_scan": True,
                "reverse_scan": True,
                "drow_to_ros": True,
                "target_frame": LaunchConfiguration("odom_frame"),
                "transform_use_latest": True,
                "subscriber.scan.topic": "/scan_merged",
                "publisher.scored.topic": "/dr_spaam_detections_scored",
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="pedestrian_point_tracker.py",
            name="tracker_backend",
            output="screen",
            condition=selected("drl_vo"),
            parameters=[{
                "input_topic": "/dr_spaam_detections_scored",
                "output_topic": "/pedestrian_tracks",
                "tracking_frame": LaunchConfiguration("odom_frame"),
            }],
        ),
        Node(
            package="semantic_nav_runtime",
            executable="drl_vo_fixed_dual_inference_node.py",
            name="drl_vo_backend",
            output="screen",
            condition=selected("drl_vo"),
            parameters=[{
                "mode": "base",
                "model": str(checkpoints / "drl_vo" / "base_bc_best.pt"),
                "device": "auto",
                "scan_01_topic": LaunchConfiguration("scan_01_topic"),
                "scan_02_topic": LaunchConfiguration("scan_02_topic"),
                "odom_topic": LaunchConfiguration("tf_pose_topic"),
                "odom_timeout": 0.8,
                "subgoal_timeout": 0.8,
                "final_goal_timeout": 0.8,
                "pedestrian_source": "dr_spaam",
                "require_pedestrian_truth": False,
                "pedestrian_tracks_topic": "/pedestrian_tracks",
                # Sensor stamps remain causal-alignment timestamps. The policy
                # measures this timeout from local track-message receipt instead.
                "pedestrian_track_timeout": 0.8,
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "map_frame": LaunchConfiguration("odom_frame"),
                "pedestrian_track_frame": LaunchConfiguration("odom_frame"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }],
        ),
    ])
