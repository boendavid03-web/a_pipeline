#!/usr/bin/env python3
"""Verify that a rosbag preserves the fixed dual-LiDAR self-occlusion mask."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_ROOT / "isaac_sim/scripts"))
from physx_lidar_people import fixed_dual_self_occlusion_mask  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--samples-per-sensor", type=int, default=30)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()
    if args.samples_per_sensor <= 0:
        raise SystemExit("ERROR: --samples-per-sensor must be positive")

    bag = args.bag.expanduser().resolve()
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=str(metadata["storage_identifier"])
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {"/scan_01": 1, "/scan_02": 2}
    counts = {topic: 0 for topic in topics}
    failures = {topic: 0 for topic in topics}
    masked_nan_min = {topic: 2000 for topic in topics}
    masked_nan_max = {topic: 0 for topic in topics}
    while reader.has_next() and any(
        count < args.samples_per_sensor for count in counts.values()
    ):
        topic, data, _ = reader.read_next()
        if topic not in topics or counts[topic] >= args.samples_per_sensor:
            continue
        scan = deserialize_message(data, LaserScan)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        mask = fixed_dual_self_occlusion_mask(topics[topic], len(ranges))
        masked_nan = int(np.count_nonzero(np.isnan(ranges[mask])))
        expected = int(mask.sum())
        counts[topic] += 1
        masked_nan_min[topic] = min(masked_nan_min[topic], masked_nan)
        masked_nan_max[topic] = max(masked_nan_max[topic], masked_nan)
        failures[topic] += int(masked_nan != expected)

    expected_counts = {
        topic: int(fixed_dual_self_occlusion_mask(sensor, 2000).sum())
        for topic, sensor in topics.items()
    }
    passed = all(
        counts[topic] == args.samples_per_sensor and failures[topic] == 0
        for topic in topics
    )
    report = {
        "schema": "dual_lidar_self_occlusion_bag_validation/v1",
        "status": "PASS" if passed else "FAIL",
        "bag": str(bag),
        "samples_requested_per_sensor": args.samples_per_sensor,
        "samples_checked": counts,
        "expected_masked_beams": expected_counts,
        "masked_nan_min": masked_nan_min,
        "masked_nan_max": masked_nan_max,
        "frames_failing_exact_mask": failures,
    }
    text = json.dumps(report, indent=2) + "\n"
    if args.output_json:
        output = args.output_json.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text)
    print(text, end="")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
