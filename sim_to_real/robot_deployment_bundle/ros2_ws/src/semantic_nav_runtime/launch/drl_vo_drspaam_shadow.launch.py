"""Dual LiDAR merger -> DR-SPAAM -> tracker -> base DRL-VO, shadow-only."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ.get('SIM_TO_REAL_BUNDLE_ROOT', Path.cwd())).resolve()
    model = root / 'checkpoints'
    return LaunchDescription([
        DeclareLaunchArgument('scan_01_topic', default_value='/scan_01'), DeclareLaunchArgument('scan_02_topic', default_value='/scan_02'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'), DeclareLaunchArgument('path_topic', default_value='/plan'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'), DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'), DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='semantic_nav_runtime', executable='v7_dual_laser_scan_merger.py', name='dual_lidar_scan_merger', output='screen', parameters=[{
            'input_scan_01_topic': LaunchConfiguration('scan_01_topic'), 'input_scan_02_topic': LaunchConfiguration('scan_02_topic'),
            'output_topic': '/scan_merged', 'output_frame': LaunchConfiguration('base_frame'), 'enable_self_filter': True,
        }]),
        Node(package='semantic_nav_runtime', executable='global_path_to_local_subgoal.py', name='global_path_to_local_subgoal', output='screen', parameters=[{
            'path_topic': LaunchConfiguration('path_topic'), 'odom_topic': LaunchConfiguration('odom_topic'), 'goal_topic': LaunchConfiguration('goal_topic'),
            'map_frame': LaunchConfiguration('map_frame'), 'odom_frame': LaunchConfiguration('odom_frame'), 'base_frame': LaunchConfiguration('base_frame'),
            'lookahead_distance': 1.2, 'publish_rate': 10.0,
        }]),
        Node(package='dr_spaam_ros2', executable='dr_spaam_w_score_ros', name='dr_spaam_ros2', output='screen', parameters=[{
            'weight_file': str(model / 'dr_spaam' / 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth'), 'detector_model': 'DR-SPAAM', 'device': 'auto',
            'panoramic_scan': True, 'reverse_scan': True, 'drow_to_ros': True, 'target_frame': LaunchConfiguration('odom_frame'),
            'subscriber.scan.topic': '/scan_merged', 'publisher.detections.topic': '/dr_spaam_detections',
            'publisher.scored.topic': '/dr_spaam_detections_scored',
        }]),
        Node(package='semantic_nav_runtime', executable='pedestrian_point_tracker.py', name='pedestrian_point_tracker', output='screen', parameters=[{
            'input_topic': '/dr_spaam_detections_scored', 'output_topic': '/pedestrian_tracks', 'tracking_frame': LaunchConfiguration('odom_frame'),
        }]),
        Node(package='semantic_nav_runtime', executable='drl_vo_fixed_dual_inference_node.py', name='drl_vo_fixed_dual_inference', output='screen', parameters=[{
            'mode': 'base', 'model': str(model / 'drl_vo' / 'base_bc_best.pt'), 'device': 'auto',
            'scan_01_topic': LaunchConfiguration('scan_01_topic'), 'scan_02_topic': LaunchConfiguration('scan_02_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'), 'local_subgoal_topic': '/semantic_cnn/local_subgoal',
            'final_goal_topic': '/semantic_cnn/final_goal', 'pedestrian_source': 'dr_spaam', 'require_pedestrian_truth': False,
            'pedestrian_tracks_topic': '/pedestrian_tracks', 'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'), 'map_frame': LaunchConfiguration('map_frame'),
            'pedestrian_track_frame': LaunchConfiguration('odom_frame'), 'cmd_vel_topic': '/sim_to_real/drl_vo/cmd_vel_shadow',
        }]),
    ])
