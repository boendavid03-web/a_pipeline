import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from odom_path_node import OdomPathNode


def odom_message(x, y, seconds):
    message = Odometry()
    message.header.stamp.sec = int(seconds)
    message.header.frame_id = "odom"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    return message


def goal_message(seconds):
    message = PointStamped()
    message.header.stamp.sec = int(seconds)
    message.header.frame_id = "map"
    return message


def test_path_waits_for_goal_and_clears_for_each_new_goal(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "start_on_goal:=true",
            "-p",
            "enabled:=true",
            "-p",
            "path_topic:=/test/actual_trajectory",
        ]
    )
    node = OdomPathNode()
    try:
        node.odom_callback(odom_message(0.0, 0.0, 1.0))
        assert node.active is False
        assert node.path.poses == []

        node.goal_callback(goal_message(2.0))
        node.odom_callback(odom_message(0.0, 0.0, 2.0))
        node.odom_callback(odom_message(1.0, 0.0, 3.0))
        assert node.active is True
        assert len(node.path.poses) == 2

        node.goal_callback(goal_message(4.0))
        assert node.path.poses == []
        assert node.last_xy is None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
