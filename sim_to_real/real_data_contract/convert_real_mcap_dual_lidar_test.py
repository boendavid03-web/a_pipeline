#!/usr/bin/env python3
"""Convert one hardware dual-LiDAR MCAP bag to non-semantic NumPy arrays.

This is a read-only adapter for a hardware test capture.  It preserves the two
raw sensor slots and writes no semantic labels or map projections.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import struct
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


SELF_FOOTPRINT_HALF_EXTENTS_M = (0.36, 0.32)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scan-01-urdf", required=True, type=Path)
    parser.add_argument("--scan-02-urdf", required=True, type=Path)
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    return parser.parse_args()


def read_string(data, offset):
    length = struct.unpack_from("<I", data, offset)[0]
    start = offset + 4
    return data[start : start + length].decode("utf-8"), start + length


def records(data):
    offset = 0
    while offset + 9 <= len(data):
        opcode = data[offset]
        length = struct.unpack_from("<Q", data, offset + 1)[0]
        offset += 9
        end = offset + length
        if end > len(data):
            raise RuntimeError("Malformed MCAP record length")
        yield opcode, data[offset:end]
        offset = end


def mcap_messages(path):
    """Yield (topic, ROS type, serialized CDR, storage timestamp) from MCAP."""
    raw = path.read_bytes()
    if raw[:8] != b"\x89MCAP0\r\n" or raw[-8:] != b"\x89MCAP0\r\n":
        raise RuntimeError(f"Not an MCAP file: {path}")

    schemas = {}
    channels = {}
    chunks = []
    for opcode, data in records(raw[8:-8]):
        if opcode == 3:  # Schema
            schema_id = struct.unpack_from("<H", data)[0]
            name, offset = read_string(data, 2)
            _, offset = read_string(data, offset)
            schemas[schema_id] = name
        elif opcode == 4:  # Channel
            channel_id, schema_id = struct.unpack_from("<HH", data)
            topic, offset = read_string(data, 4)
            _, offset = read_string(data, offset)
            channels[channel_id] = (topic, schemas[schema_id])
        elif opcode == 6:  # Chunk
            chunks.append(data)

    for chunk in chunks:
        compression_length = struct.unpack_from("<I", chunk, 28)[0]
        if compression_length:
            compression, _ = read_string(chunk, 28)
            raise RuntimeError(
                f"Compressed MCAP chunks are not supported by this test adapter: {compression}"
            )
        records_offset = 32
        records_length = struct.unpack_from("<Q", chunk, records_offset)[0]
        payload_start = records_offset + 8
        payload = chunk[payload_start : payload_start + records_length]
        if len(payload) != records_length:
            raise RuntimeError("Malformed MCAP chunk payload")
        for opcode, data in records(payload):
            if opcode != 5:  # Message
                continue
            channel_id = struct.unpack_from("<H", data)[0]
            storage_stamp_ns = struct.unpack_from("<Q", data, 6)[0]
            topic, message_type = channels[channel_id]
            yield topic, message_type, data[22:], storage_stamp_ns


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def compose_planar(first, second):
    x, y, yaw = first
    other_x, other_y, other_yaw = second
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        x + cosine * other_x - sine * other_y,
        y + sine * other_x + cosine * other_y,
        math.atan2(math.sin(yaw + other_yaw), math.cos(yaw + other_yaw)),
    )


def nearest_by_time(items, target_ns):
    times = [item[0] for item in items]
    index = bisect.bisect_left(times, target_ns)
    if index == 0:
        return items[0]
    if index == len(items):
        return items[-1]
    before, after = items[index - 1], items[index]
    return before if target_ns - before[0] <= after[0] - target_ns else after


def hold_last_by_time(items, target_ns):
    times = [item[0] for item in items]
    index = bisect.bisect_right(times, target_ns)
    return None if index == 0 else items[index - 1]


def pair_scans(scans_01, scans_02, tolerance_ns):
    pairs = []
    index_02 = 0
    skipped_01 = 0
    skipped_02 = 0
    for stamp_01, scan_01 in scans_01:
        while index_02 < len(scans_02) and scans_02[index_02][0] < stamp_01 - tolerance_ns:
            skipped_02 += 1
            index_02 += 1
        if index_02 == len(scans_02):
            skipped_01 += 1
            continue
        end = index_02
        while end < len(scans_02) and scans_02[end][0] <= stamp_01 + tolerance_ns:
            end += 1
        if end == index_02:
            skipped_01 += 1
            continue
        match = min(
            range(index_02, end),
            key=lambda candidate: abs(scans_02[candidate][0] - stamp_01),
        )
        skipped_02 += match - index_02
        stamp_02, scan_02 = scans_02[match]
        pairs.append((stamp_01, scan_01, stamp_02, scan_02))
        index_02 = match + 1
    skipped_02 += len(scans_02) - index_02
    return pairs, skipped_01, skipped_02


def load_extrinsic(path):
    root = ElementTree.parse(path).getroot()
    joint = root.find("joint")
    if joint is None or joint.get("type") != "fixed":
        raise RuntimeError(f"Expected one fixed joint in {path}")
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    if parent is None or child is None or origin is None:
        raise RuntimeError(f"Missing parent, child, or origin in {path}")
    xyz = np.fromstring(origin.get("xyz", ""), sep=" ", dtype=np.float64)
    rpy = np.fromstring(origin.get("rpy", ""), sep=" ", dtype=np.float64)
    if xyz.shape != (3,) or rpy.shape != (3,):
        raise RuntimeError(f"Invalid xyz/rpy in {path}")
    roll, pitch, yaw = rpy
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=np.float64,
    )
    return {
        "parent": parent.get("link"),
        "child": child.get("link"),
        "xyz": xyz,
        "rpy": rpy,
        "rotation": rotation,
    }


def scan_arrays(scan, extrinsic):
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    intensities = np.asarray(scan.intensities, dtype=np.float32)
    if ranges.shape != (2000,) or intensities.shape != (2000,):
        raise RuntimeError(
            f"Expected 2000 ranges and intensities from {scan.header.frame_id}, "
            f"got {ranges.shape} and {intensities.shape}"
        )
    angles = float(scan.angle_min) + np.arange(2000, dtype=np.float32) * float(scan.angle_increment)
    range_valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    points = np.full((2000, 2), np.nan, dtype=np.float32)
    indices = np.flatnonzero(range_valid)
    sensor_points = np.column_stack(
        (
            ranges[indices] * np.cos(angles[indices]),
            ranges[indices] * np.sin(angles[indices]),
            np.zeros(len(indices), dtype=np.float32),
        )
    )
    base_points = sensor_points @ extrinsic["rotation"].T + extrinsic["xyz"]
    points[indices] = base_points[:, :2].astype(np.float32)
    self_mask = np.zeros(2000, dtype=np.bool_)
    self_mask[indices] = (
        (np.abs(points[indices, 0]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[0])
        & (np.abs(points[indices, 1]) <= SELF_FOOTPRINT_HALF_EXTENTS_M[1])
    )
    valid = range_valid & ~self_mask
    features = np.zeros((2000, 3), dtype=np.float32)
    features[:, 0] = np.where(valid, np.clip(ranges, scan.range_min, scan.range_max) / scan.range_max, 0.0)
    features[:, 1] = np.clip(intensities, 0.0, 255.0) / 255.0
    features[:, 2] = valid.astype(np.float32)
    return ranges, intensities, valid, points, features


def main():
    args = parse_args()
    if args.sync_tolerance_ms < 0.0:
        raise ValueError("--sync-tolerance-ms must be non-negative")
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {args.output}")
    if not args.bag.is_file():
        raise FileNotFoundError(args.bag)

    extrinsics = (load_extrinsic(args.scan_01_urdf), load_extrinsic(args.scan_02_urdf))
    if [(item["parent"], item["child"]) for item in extrinsics] != [
        ("base_footprint", "base_scan_01"),
        ("base_footprint", "base_scan_02"),
    ]:
        raise RuntimeError("URDF extrinsics do not match base_footprint -> base_scan_01/02")

    scans_01, scans_02, map_to_odom, odom_to_base, commands = [], [], [], [], []
    for topic, message_type, data, storage_stamp_ns in mcap_messages(args.bag):
        if topic in ("/scan_01", "/scan_02"):
            if message_type != "sensor_msgs/msg/LaserScan":
                raise RuntimeError(f"Unexpected type for {topic}: {message_type}")
            scan = deserialize_message(data, LaserScan)
            (scans_01 if topic == "/scan_01" else scans_02).append((stamp_ns(scan.header.stamp), scan))
        elif topic == "/tf":
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                value = (
                    stamp_ns(transform.header.stamp),
                    (
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y),
                        yaw_from_quaternion(transform.transform.rotation),
                    ),
                )
                if (parent, child) == ("map", "odom"):
                    map_to_odom.append(value)
                elif (parent, child) == ("odom", "base_footprint"):
                    odom_to_base.append(value)
        elif topic == "/cmd_vel":
            command = deserialize_message(data, Twist)
            commands.append(
                (
                    int(storage_stamp_ns),
                    (float(command.linear.x), float(command.linear.y), float(command.angular.z)),
                )
            )

    for values in (scans_01, scans_02, map_to_odom, odom_to_base, commands):
        values.sort(key=lambda item: item[0])
    if not scans_01 or not scans_02 or not map_to_odom or not odom_to_base or not commands:
        raise RuntimeError("Missing scan, TF pose, or cmd_vel data in MCAP")

    pairs, skipped_01, skipped_02 = pair_scans(
        scans_01, scans_02, round(args.sync_tolerance_ms * 1_000_000.0)
    )
    rows = []
    for scan_01_stamp, scan_01, scan_02_stamp, scan_02 in pairs:
        command = hold_last_by_time(commands, scan_01_stamp)
        if command is None:
            continue
        map_odom = nearest_by_time(map_to_odom, scan_01_stamp)
        odom_base = nearest_by_time(odom_to_base, scan_01_stamp)
        pose = compose_planar(map_odom[1], odom_base[1])
        first = scan_arrays(scan_01, extrinsics[0])
        second = scan_arrays(scan_02, extrinsics[1])
        rows.append(
            (
                scan_01_stamp,
                scan_02_stamp,
                command[0],
                pose,
                abs(map_odom[0] - scan_01_stamp),
                abs(odom_base[0] - scan_01_stamp),
                first,
                second,
                command[1],
            )
        )
    if not rows:
        raise RuntimeError("No paired scans had a causal cmd_vel message")

    args.output.mkdir(parents=True)
    np.save(args.output / "features.npy", np.asarray([[row[6][4], row[7][4]] for row in rows], dtype=np.float32))
    np.save(args.output / "raw_ranges_m.npy", np.asarray([[row[6][0], row[7][0]] for row in rows], dtype=np.float32))
    np.save(args.output / "intensities.npy", np.asarray([[row[6][1], row[7][1]] for row in rows], dtype=np.float32))
    np.save(args.output / "valid_mask.npy", np.asarray([[row[6][2], row[7][2]] for row in rows], dtype=np.bool_))
    np.save(args.output / "points_base_xy_m.npy", np.asarray([[row[6][3], row[7][3]] for row in rows], dtype=np.float32))
    np.save(args.output / "pose_map_base_footprint.npy", np.asarray([row[3] for row in rows], dtype=np.float32))
    np.save(args.output / "actions_cmd_vel.npy", np.asarray([row[8] for row in rows], dtype=np.float32))
    np.save(args.output / "scan_timestamps_ns.npy", np.asarray([[row[0], row[1]] for row in rows], dtype=np.int64))
    np.save(args.output / "command_timestamps_ns.npy", np.asarray([row[2] for row in rows], dtype=np.int64))
    np.save(args.output / "command_age_ms.npy", np.asarray([(row[0] - row[2]) / 1_000_000.0 for row in rows], dtype=np.float32))
    np.save(args.output / "pose_nearest_delta_ms.npy", np.asarray([[row[4] / 1_000_000.0, row[5] / 1_000_000.0] for row in rows], dtype=np.float32))

    metadata = {
        "purpose": "non-semantic hardware dual-LiDAR conversion test",
        "source_mcap": str(args.bag.resolve()),
        "sample_count": len(rows),
        "layout": "[sample, sensor_01_or_02, beam, feature]",
        "features": ["range_normalized_by_sensor_range_max", "intensity_normalized_by_255", "valid_mask"],
        "actions_cmd_vel": ["linear_x_mps", "linear_y_mps", "angular_z_radps"],
        "pose": "map -> base_footprint: x_m, y_m, yaw_rad",
        "command_time_source": "MCAP storage timestamp; original /cmd_vel has no header stamp",
        "sync_tolerance_ms": args.sync_tolerance_ms,
        "scan_pairs_before_command_filter": len(pairs),
        "skipped_scan_01": skipped_01,
        "skipped_scan_02": skipped_02,
        "extrinsics": [
            {
                "parent": item["parent"],
                "child": item["child"],
                "xyz_m": item["xyz"].tolist(),
                "rpy_rad": item["rpy"].tolist(),
            }
            for item in extrinsics
        ],
    }
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
