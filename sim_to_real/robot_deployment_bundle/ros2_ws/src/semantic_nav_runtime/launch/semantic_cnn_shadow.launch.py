"""Online S3-Net + SemanticCNN, shadow-only hardware launch."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ.get('SIM_TO_REAL_BUNDLE_ROOT', Path.cwd())).resolve()
    model = root / 'checkpoints'
    topics = {
        'input_scan_01_topic': LaunchConfiguration('scan_01_topic'),
        'input_scan_02_topic': LaunchConfiguration('scan_02_topic'),
        'output_scan_01_topic': '/sim_to_real/aligned/scan_01',
        'output_scan_02_topic': '/sim_to_real/aligned/scan_02',
    }
    return LaunchDescription([
        DeclareLaunchArgument('scan_01_topic', default_value='/scan_01'),
        DeclareLaunchArgument('scan_02_topic', default_value='/scan_02'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('path_topic', default_value='/plan'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='semantic_nav_runtime', executable='fixed_dual_scan_adapter.py', name='fixed_dual_scan_adapter', output='screen', parameters=[topics]),
        Node(package='semantic_nav_runtime', executable='global_path_to_local_subgoal.py', name='global_path_to_local_subgoal', output='screen', parameters=[{
            'path_topic': LaunchConfiguration('path_topic'), 'odom_topic': LaunchConfiguration('odom_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'), 'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'), 'base_frame': LaunchConfiguration('base_frame'),
            'lookahead_distance': 1.2, 'publish_rate': 10.0,
        }]),
        Node(package='semantic_nav_runtime', executable='s3net_fixed_dual_inference_node.py', name='s3net_fixed_dual_inference', output='screen', parameters=[{
            'model': str(model / 's3net' / 's3net_native_stats_best_dev.pth'),
            'model_code': str(model / 's3net'), 'stats_json': str(model / 's3net' / 's3net_native_lidar_train_stats.json'),
            'scan_01_topic': '/sim_to_real/aligned/scan_01', 'scan_02_topic': '/sim_to_real/aligned/scan_02',
            'base_frame': LaunchConfiguration('base_frame'), 'enforce_message_layout': True,
        }]),
        Node(package='semantic_nav_runtime', executable='semantic_cnn_fixed_dual_inference_node.py', name='semantic_cnn_fixed_dual_inference', output='screen', parameters=[{
            'model': str(model / 'semantic_cnn' / 'semantic_cnn_native_cmd_best_dev.pth'), 'model_code': str(model / 'semantic_cnn'),
            'scan_01_topic': '/sim_to_real/aligned/scan_01', 'scan_02_topic': '/sim_to_real/aligned/scan_02',
            'use_online_s3net': True, 's3net_labels_topic': '/s3net/labels', 'odom_topic': LaunchConfiguration('odom_topic'),
            'local_subgoal_topic': '/semantic_cnn/local_subgoal', 'final_goal_topic': '/semantic_cnn/final_goal',
            'base_frame': LaunchConfiguration('base_frame'), 'cmd_vel_topic': '/sim_to_real/semantic_cnn/cmd_vel_shadow',
        }]),
    ])
