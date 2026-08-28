#!/usr/bin/env python3
"""Start the Isaac Sim scan merger and pedestrian-message adapter."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("semantic_nav_gazebo"))
    return LaunchDescription(
        [
            Node(
                package="semantic_nav_gazebo",
                executable="v7_dual_laser_scan_merger.py",
                name="v7_dual_laser_scan_merger",
                output="screen",
                parameters=[str(package_share / "config" / "v7_dual_laser_scan_merger.yaml")],
            ),
            Node(
                package="semantic_nav_gazebo",
                executable="isaac_pedestrian_ground_truth_adapter.py",
                name="isaac_pedestrian_ground_truth_adapter",
                output="screen",
            ),
        ]
    )
