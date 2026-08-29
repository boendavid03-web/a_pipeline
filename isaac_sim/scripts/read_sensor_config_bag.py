#!/usr/bin/env python3
"""Read the self-describing Isaac sensor configuration from a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


TOPIC = "/data_collection/sensor_config"
SCHEMA = "isaac_sensor_config/v1"
FIELDS = {
    "bridge_source_sha256",
    "launcher_sha256",
    "lidar_mode",
    "lidar_profile",
    "lidar_profile_asset",
    "lidar_profile_asset_sha256",
    "lidar_pairing_timestamp_domain",
    "lidar_rate_basis",
    "lidar_rate_hz",
    "lidar_samples",
    "lidar_timestamp_domain",
    "producer_source_sha256",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--field", required=True, choices=sorted(FIELDS))
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
    if topic_types.get(TOPIC) != "std_msgs/msg/String":
        return 4

    while reader.has_next():
        topic, serialized, _timestamp = reader.read_next()
        if topic != TOPIC:
            continue
        message = deserialize_message(serialized, String)
        config = json.loads(message.data)
        if config.get("schema") != SCHEMA:
            raise ValueError(f"unexpected sensor config schema: {config.get('schema')!r}")
        rate_hz = int(config["lidar_rate_hz"])
        samples = int(config["lidar_samples"])
        mode = str(config["lidar_mode"])
        profile = config.get("lidar_profile")
        expected_pairing_timestamp_domain = {
            "rtx": "isaac_rtx_gmo_timestamp_ns",
            "physx": "isaac_telemetry_sim_time",
        }.get(mode)
        if not 1 <= rate_hz <= 30 or not 90 <= samples <= 4096:
            raise ValueError("sensor config contains invalid lidar settings")
        pairing_timestamp_domain = config.get("lidar_pairing_timestamp_domain")
        current_timing_contract = bool(
            config.get("lidar_timestamp_domain") == "isaac_telemetry_sim_time"
            and pairing_timestamp_domain == expected_pairing_timestamp_domain
        )
        # Preserve read-only inspection of bags created before the ROS/RTX
        # time-domain split. The live readiness gate intentionally does not
        # accept this legacy contract.
        legacy_timing_contract = bool(
            pairing_timestamp_domain is None
            and config.get("lidar_timestamp_domain")
            == expected_pairing_timestamp_domain
        )
        current_profile_contract = (
            profile in {"example_dense", "navigation_2d_32k", "rplidar_s2e"}
            if mode == "rtx"
            else profile == "physx_raycast"
        )
        legacy_profile_contract = profile is None
        if (
            config.get("lidar_rate_basis") != "simulation_time"
            or not (current_timing_contract or legacy_timing_contract)
            or not (current_profile_contract or legacy_profile_contract)
        ):
            raise ValueError("sensor config contains inconsistent timing metadata")
        if (
            args.field == "lidar_pairing_timestamp_domain"
            and pairing_timestamp_domain is None
        ):
            print(expected_pairing_timestamp_domain)
        else:
            print(config[args.field])
        return 0
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
