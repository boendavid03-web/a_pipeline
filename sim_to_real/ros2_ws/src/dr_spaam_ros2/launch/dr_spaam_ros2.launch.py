from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('dr_spaam_ros2')

    # Paths to the parameter files
    dr_spaam_params = os.path.join(package_dir, 'config', 'dr_spaam_ros2.yaml')
    topics_params = os.path.join(package_dir, 'config', 'topics.yaml')

    return LaunchDescription([
        Node(
            package='dr_spaam_ros2',
            executable='dr_spaam_w_score_ros',  # Remove '.py' extension
            name='dr_spaam_ros2',
            output='screen',
            parameters=[dr_spaam_params, topics_params]
        )
    ])

