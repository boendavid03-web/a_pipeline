#!/usr/bin/env python3
"""Verify that an Isaac teleop bag contains motion and ends at zero velocity."""

from __future__ import annotations

import argparse
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message


EPSILON = 1.0e-6


def is_zero(values: tuple[float, float, float]) -> bool:
    return all(abs(value) <= EPSILON for value in values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    expected = "geometry_msgs/msg/Twist"
    if topic_types.get("/cmd_vel") != expected:
        print(
            f"ERROR: /cmd_vel type is {topic_types.get('/cmd_vel')!r}, expected {expected}"
        )
        return 1

    commands: list[tuple[float, float, float]] = []
    while reader.has_next():
        topic, data, _storage_time = reader.read_next()
        if topic != "/cmd_vel":
            continue
        message = deserialize_message(data, Twist)
        commands.append(
            (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )
        )

    moving_count = sum(not is_zero(command) for command in commands)
    final_zero = bool(commands and is_zero(commands[-1]))
    linear_x = [command[0] for command in commands]
    linear_y = [command[1] for command in commands]
    angular_z = [command[2] for command in commands]
    print(f"cmd_vel_count: {len(commands)}")
    print(f"cmd_vel_moving_count: {moving_count}")
    print(f"cmd_vel_final_zero: {final_zero}")
    if commands:
        print(f"linear_x_range: [{min(linear_x):.6f}, {max(linear_x):.6f}]")
        print(f"linear_y_range: [{min(linear_y):.6f}, {max(linear_y):.6f}]")
        print(f"angular_z_range: [{min(angular_z):.6f}, {max(angular_z):.6f}]")
    if not commands:
        print("ERROR: no /cmd_vel messages were recorded")
        return 1
    if moving_count == 0:
        print("ERROR: the bag contains only stop commands; no driving data was captured")
        return 1
    if not final_zero:
        print("ERROR: the final /cmd_vel is not zero; press k before stopping future recordings")
        return 1
    print("teleop_command_semantics: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
