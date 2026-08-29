#!/usr/bin/env python3
"""Verify aligned, non-zero, non-constant LaserScan intensities in a ROS 2 bag."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = ("/scan", "/scan_01", "/scan_02", "/scan_merged")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    args = parser.parse_args()
    if not args.bag.is_dir():
        parser.error(f"bag directory not found: {args.bag}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    stats = {
        topic: {
            "messages": 0,
            "aligned": 0,
            "empty": 0,
            "nonzero": 0,
            "minimum": math.inf,
            "maximum": -math.inf,
            "unique": set(),
        }
        for topic in TOPICS
    }
    while reader.has_next():
        topic, serialized, _timestamp = reader.read_next()
        if topic not in stats:
            continue
        message = deserialize_message(serialized, get_message(topic_types[topic]))
        current = stats[topic]
        current["messages"] += 1
        if not message.intensities:
            current["empty"] += 1
            continue
        if len(message.intensities) != len(message.ranges):
            continue
        current["aligned"] += 1
        for raw_value in message.intensities:
            value = float(raw_value)
            if not math.isfinite(value) or value <= 0.0:
                continue
            current["nonzero"] += 1
            current["minimum"] = min(current["minimum"], value)
            current["maximum"] = max(current["maximum"], value)
            if len(current["unique"]) < 1024:
                current["unique"].add(round(value, 6))

    errors = []
    for topic in TOPICS:
        current = stats[topic]
        if current["messages"] <= 0:
            errors.append(f"{topic}: no messages")
        if current["aligned"] != current["messages"]:
            errors.append(
                f"{topic}: aligned intensity frames={current['aligned']}/"
                f"{current['messages']}, empty={current['empty']}"
            )
        if current["nonzero"] <= 0:
            errors.append(f"{topic}: no positive intensity returns")
        if len(current["unique"]) < 2:
            errors.append(
                f"{topic}: intensity is missing or constant; "
                f"unique_positive_values={len(current['unique'])}"
            )
        minimum = current["minimum"] if current["minimum"] != math.inf else 0.0
        maximum = current["maximum"] if current["maximum"] != -math.inf else 0.0
        print(
            f"lidar_intensity {topic}: messages={current['messages']} "
            f"aligned={current['aligned']} nonzero_values={current['nonzero']} "
            f"unique_positive_values={len(current['unique'])} "
            f"min={minimum:.6f} max={maximum:.6f}"
        )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("lidar_intensity_contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

