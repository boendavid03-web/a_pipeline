#!/usr/bin/env python3
"""Convert synchronized dual LaserScan topics into the project's virtual 360 scan.

The geometry matches ``v7_dual_laser_scan_merger.py``: valid beam endpoints are
transformed into the output frame, points inside the robot self-filter are
removed, and the nearest endpoint wins each angular slot.  The converter keeps
the validity and source provenance that a LaserScan alone would discard.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scan-01-topic", default="/scan_01")
    parser.add_argument("--scan-02-topic", default="/scan_02")
    parser.add_argument(
        "--self-occlusion-calibration",
        type=Path,
        help="metadata JSON containing fixed scan_01/scan_02 self-mask runs",
    )
    parser.add_argument("--output-frame", default="base_link")
    parser.add_argument("--output-samples", type=int, default=360)
    parser.add_argument("--angle-min", type=float, default=-math.pi)
    parser.add_argument("--angle-max", type=float, default=math.pi)
    parser.add_argument("--range-min", type=float, default=0.1)
    parser.add_argument("--range-max", type=float, default=50.0)
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    parser.add_argument("--self-filter-min-x", type=float, default=-0.36)
    parser.add_argument("--self-filter-max-x", type=float, default=0.36)
    parser.add_argument("--self-filter-min-y", type=float, default=-0.32)
    parser.add_argument("--self-filter-max-y", type=float, default=0.32)
    return parser.parse_args()


def stamp_ns(scan: LaserScan) -> int:
    return int(scan.header.stamp.sec) * 1_000_000_000 + int(
        scan.header.stamp.nanosec
    )


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(np.float64).eps:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def transform_matrix(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix(
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    matrix[:3, 3] = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
    )
    return matrix


def normalize_frame(frame: str) -> str:
    return frame.strip().lstrip("/")


def build_static_graph(messages: list[TFMessage]):
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    edges: list[dict[str, object]] = []
    for message in messages:
        for stamped in message.transforms:
            parent = normalize_frame(stamped.header.frame_id)
            child = normalize_frame(stamped.child_frame_id)
            if not parent or not child:
                continue
            child_to_parent = transform_matrix(stamped.transform)
            graph.setdefault(child, []).append((parent, child_to_parent))
            graph.setdefault(parent, []).append((child, np.linalg.inv(child_to_parent)))
            edges.append(
                {
                    "parent": parent,
                    "child": child,
                    "child_to_parent": child_to_parent.tolist(),
                }
            )
    return graph, edges


def lookup_static_transform(
    graph: dict[str, list[tuple[str, np.ndarray]]], source: str, target: str
) -> np.ndarray:
    source, target = normalize_frame(source), normalize_frame(target)
    if source == target:
        return np.eye(4, dtype=np.float64)
    queue = deque([(source, np.eye(4, dtype=np.float64))])
    visited = {source}
    while queue:
        frame, source_to_frame = queue.popleft()
        for neighbor, frame_to_neighbor in graph.get(frame, []):
            if neighbor in visited:
                continue
            source_to_neighbor = frame_to_neighbor @ source_to_frame
            if neighbor == target:
                return source_to_neighbor
            visited.add(neighbor)
            queue.append((neighbor, source_to_neighbor))
    raise RuntimeError(f"No static TF path from {source!r} to {target!r}")


def pair_scans(
    first: list[tuple[int, LaserScan]],
    second: list[tuple[int, LaserScan]],
    tolerance_ns: int,
):
    second_times = [item[0] for item in second]
    used_second: set[int] = set()
    pairs = []
    skipped_first = 0
    for first_stamp, first_scan in first:
        begin = bisect.bisect_left(second_times, first_stamp - tolerance_ns)
        end = bisect.bisect_right(second_times, first_stamp + tolerance_ns)
        candidates = [index for index in range(begin, end) if index not in used_second]
        if not candidates:
            skipped_first += 1
            continue
        match = min(candidates, key=lambda index: abs(second[index][0] - first_stamp))
        used_second.add(match)
        pairs.append((first_stamp, first_scan, second[match][0], second[match][1]))
    skipped_second = len(second) - len(used_second)
    return pairs, skipped_first, skipped_second


def load_fixed_self_masks(path: Path | None, beam_count: int = 2000):
    if path is None:
        return {
            1: np.zeros(beam_count, dtype=bool),
            2: np.zeros(beam_count, dtype=bool),
        }
    metadata = json.loads(path.expanduser().resolve().read_text())
    calibration = metadata.get("self_mask_calibration")
    if not isinstance(calibration, dict):
        raise RuntimeError(f"{path}: missing self_mask_calibration")
    masks = {}
    for sensor in (1, 2):
        key = f"scan_0{sensor}_masked_beam_runs"
        runs = calibration.get(key)
        if not isinstance(runs, list):
            raise RuntimeError(f"{path}: missing {key}")
        mask = np.zeros(beam_count, dtype=bool)
        for start, end in runs:
            start, end = int(start), int(end)
            if start < 0 or end < start or end >= beam_count:
                raise RuntimeError(f"{path}: invalid {key} run {[start, end]}")
            mask[start : end + 1] = True
        expected = calibration.get(f"scan_0{sensor}_masked_beam_count")
        if expected is not None and int(mask.sum()) != int(expected):
            raise RuntimeError(f"{path}: {key} count mismatch")
        masks[sensor] = mask
    return masks


def project_scan(
    scan: LaserScan,
    source_to_output: np.ndarray,
    *,
    sensor_id: int,
    range_min: float,
    range_max: float,
    self_filter: tuple[float, float, float, float],
    fixed_self_mask: np.ndarray | None = None,
):
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = float(scan.angle_min) + np.arange(ranges.size) * float(
        scan.angle_increment
    )
    low = max(float(scan.range_min), range_min)
    high = min(float(scan.range_max), range_max)
    raw_valid = np.isfinite(ranges) & (ranges >= low) & (ranges <= high)
    fixed_self_filtered = 0
    if fixed_self_mask is not None:
        fixed_self_mask = np.asarray(fixed_self_mask, dtype=bool)
        if fixed_self_mask.shape != ranges.shape:
            raise RuntimeError(
                f"fixed self mask shape {fixed_self_mask.shape} does not match scan {ranges.shape}"
            )
        fixed_self_filtered = int(np.count_nonzero(raw_valid & fixed_self_mask))
        raw_valid &= ~fixed_self_mask
    beam_indices = np.flatnonzero(raw_valid)
    points = np.column_stack(
        (
            ranges[beam_indices] * np.cos(angles[beam_indices]),
            ranges[beam_indices] * np.sin(angles[beam_indices]),
            np.zeros(beam_indices.size, dtype=np.float64),
            np.ones(beam_indices.size, dtype=np.float64),
        )
    )
    output_points = points @ source_to_output.T
    x, y = output_points[:, 0], output_points[:, 1]
    min_x, max_x, min_y, max_y = self_filter
    self_hits = (x >= min_x) & (x <= max_x) & (y >= min_y) & (y <= max_y)
    virtual_ranges = np.hypot(x, y)
    after_tf = (
        ~self_hits
        & (virtual_ranges >= range_min)
        & (virtual_ranges <= range_max)
    )
    return {
        "ranges": virtual_ranges[after_tf],
        "angles": np.arctan2(y[after_tf], x[after_tf]),
        "beams": beam_indices[after_tf].astype(np.int32),
        "sensors": np.full(np.count_nonzero(after_tf), sensor_id, dtype=np.uint8),
        "raw": int(ranges.size),
        "raw_valid": int(beam_indices.size),
        "fixed_self_filtered": fixed_self_filtered,
        "self_filtered": int(np.count_nonzero(self_hits)),
        "range_filtered_after_tf": int(np.count_nonzero(~self_hits & ~after_tf)),
    }


def merge_projected(
    projected: list[dict[str, object]],
    *,
    output_samples: int,
    angle_min: float,
    angle_max: float,
):
    angle_increment = (angle_max - angle_min) / float(output_samples - 1)
    ranges = np.concatenate([item["ranges"] for item in projected])
    angles = np.concatenate([item["angles"] for item in projected])
    beams = np.concatenate([item["beams"] for item in projected])
    sensors = np.concatenate([item["sensors"] for item in projected])
    slots = np.rint((angles - angle_min) / angle_increment).astype(np.int64)
    inside = (slots >= 0) & (slots < output_samples)
    slots, ranges, beams, sensors = (
        value[inside] for value in (slots, ranges, beams, sensors)
    )

    output_ranges = np.full(output_samples, np.inf, dtype=np.float32)
    output_sensors = np.zeros(output_samples, dtype=np.uint8)
    output_beams = np.full(output_samples, -1, dtype=np.int32)
    source_masks = np.zeros(output_samples, dtype=np.uint8)
    for slot, sensor in zip(slots, sensors):
        source_masks[slot] |= sensor
    if slots.size:
        order = np.lexsort((beams, sensors, ranges, slots))
        sorted_slots = slots[order]
        first = np.concatenate(([True], sorted_slots[1:] != sorted_slots[:-1]))
        selected = order[first]
        output_ranges[slots[selected]] = ranges[selected].astype(np.float32)
        output_sensors[slots[selected]] = sensors[selected]
        output_beams[slots[selected]] = beams[selected]
    return output_ranges, output_sensors, output_beams, source_masks, int(
        slots.size - np.unique(slots).size
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_merged_rosbag(
    path: Path,
    stamps: np.ndarray,
    ranges: np.ndarray,
    *,
    frame_id: str,
    angle_min: float,
    angle_max: float,
    range_min: float,
    range_max: float,
) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic = "/scan_merged"
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic,
            type="sensor_msgs/msg/LaserScan",
            serialization_format="cdr",
        )
    )
    positive_periods = np.diff(stamps)
    positive_periods = positive_periods[positive_periods > 0]
    default_scan_time = (
        float(np.median(positive_periods) / 1_000_000_000.0)
        if positive_periods.size
        else 0.1
    )
    angle_increment = (angle_max - angle_min) / float(ranges.shape[1] - 1)
    for index, timestamp in enumerate(stamps):
        scan_time = default_scan_time
        if index > 0 and stamps[index] > stamps[index - 1]:
            scan_time = float((stamps[index] - stamps[index - 1]) / 1_000_000_000.0)
        message = LaserScan()
        message.header.stamp.sec = int(timestamp // 1_000_000_000)
        message.header.stamp.nanosec = int(timestamp % 1_000_000_000)
        message.header.frame_id = frame_id
        message.angle_min = angle_min
        message.angle_max = angle_max
        message.angle_increment = angle_increment
        message.time_increment = scan_time / float(ranges.shape[1])
        message.scan_time = scan_time
        message.range_min = range_min
        message.range_max = range_max
        message.ranges = ranges[index].tolist()
        message.intensities = []
        writer.write(topic, serialize_message(message), int(timestamp))


def main() -> None:
    args = parse_args()
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite output: {args.output}")
    if args.output_samples < 2 or args.angle_max <= args.angle_min:
        raise ValueError("Invalid output angular geometry")
    if args.range_max <= args.range_min or args.sync_tolerance_ms < 0.0:
        raise ValueError("Invalid range or synchronization limits")

    storage = rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, rosbag2_py.ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic in (args.scan_01_topic, args.scan_02_topic):
        if topic_types.get(topic) != "sensor_msgs/msg/LaserScan":
            raise RuntimeError(f"Missing LaserScan topic {topic!r}")

    scans_01: list[tuple[int, LaserScan]] = []
    scans_02: list[tuple[int, LaserScan]] = []
    static_messages: list[TFMessage] = []
    while reader.has_next():
        topic, data, _storage_stamp = reader.read_next()
        if topic == args.scan_01_topic:
            message = deserialize_message(data, LaserScan)
            scans_01.append((stamp_ns(message), message))
        elif topic == args.scan_02_topic:
            message = deserialize_message(data, LaserScan)
            scans_02.append((stamp_ns(message), message))
        elif topic == "/tf_static":
            static_messages.append(deserialize_message(data, TFMessage))
    if not scans_01 or not scans_02:
        raise RuntimeError("Both scan topics must contain at least one message")

    tolerance_ns = round(args.sync_tolerance_ms * 1_000_000.0)
    pairs, skipped_01, skipped_02 = pair_scans(scans_01, scans_02, tolerance_ns)
    if not pairs:
        raise RuntimeError("No synchronized scan pairs found")
    graph, static_edges = build_static_graph(static_messages)
    frames = {
        normalize_frame(scan.header.frame_id)
        for pair in pairs
        for scan in (pair[1], pair[3])
    }
    transforms = {
        frame: lookup_static_transform(graph, frame, args.output_frame)
        for frame in frames
    }
    fixed_self_masks = load_fixed_self_masks(args.self_occlusion_calibration)

    output_ranges = np.full((len(pairs), args.output_samples), np.inf, np.float32)
    output_sensors = np.zeros((len(pairs), args.output_samples), np.uint8)
    output_beams = np.full((len(pairs), args.output_samples), -1, np.int32)
    output_masks = np.zeros((len(pairs), args.output_samples), np.uint8)
    stamps = np.empty(len(pairs), np.int64)
    sync_delta_ns = np.empty(len(pairs), np.int64)
    frame_rows = []
    self_filter = (
        args.self_filter_min_x,
        args.self_filter_max_x,
        args.self_filter_min_y,
        args.self_filter_max_y,
    )
    for frame_index, (stamp_01, scan_01, stamp_02, scan_02) in enumerate(pairs):
        first = project_scan(
            scan_01,
            transforms[normalize_frame(scan_01.header.frame_id)],
            sensor_id=1,
            range_min=args.range_min,
            range_max=args.range_max,
            self_filter=self_filter,
            fixed_self_mask=fixed_self_masks[1],
        )
        second = project_scan(
            scan_02,
            transforms[normalize_frame(scan_02.header.frame_id)],
            sensor_id=2,
            range_min=args.range_min,
            range_max=args.range_max,
            self_filter=self_filter,
            fixed_self_mask=fixed_self_masks[2],
        )
        merged = merge_projected(
            [first, second],
            output_samples=args.output_samples,
            angle_min=args.angle_min,
            angle_max=args.angle_max,
        )
        output_ranges[frame_index], output_sensors[frame_index], output_beams[
            frame_index
        ], output_masks[frame_index], discarded = merged
        stamps[frame_index] = max(stamp_01, stamp_02)
        sync_delta_ns[frame_index] = abs(stamp_01 - stamp_02)
        frame_rows.append(
            {
                "frame_index": frame_index,
                "stamp_ns": int(stamps[frame_index]),
                "sync_delta_ms": sync_delta_ns[frame_index] / 1_000_000.0,
                "scan_01_valid_points": first["ranges"].size,
                "scan_02_valid_points": second["ranges"].size,
                "scan_01_fixed_self_filtered": first["fixed_self_filtered"],
                "scan_02_fixed_self_filtered": second["fixed_self_filtered"],
                "finite_slots": int(np.isfinite(output_ranges[frame_index]).sum()),
                "slot_competition": discarded,
                "dual_source_slots": int((output_masks[frame_index] == 3).sum()),
            }
        )

    args.output.mkdir(parents=True)
    npz_path = args.output / "virtual360.npz"
    np.savez_compressed(
        npz_path,
        ranges_m=output_ranges,
        valid_mask=np.isfinite(output_ranges),
        stamp_ns=stamps,
        sync_delta_ns=sync_delta_ns,
        source_sensor=output_sensors,
        source_beam_index=output_beams,
        source_sensor_mask=output_masks,
        angles_rad=np.linspace(
            args.angle_min, args.angle_max, args.output_samples, dtype=np.float32
        ),
    )
    rosbag_path = args.output / "rosbag"
    write_merged_rosbag(
        rosbag_path,
        stamps,
        output_ranges,
        frame_id=normalize_frame(args.output_frame),
        angle_min=args.angle_min,
        angle_max=args.angle_max,
        range_min=args.range_min,
        range_max=args.range_max,
    )
    with (args.output / "frame_stats.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frame_rows[0]))
        writer.writeheader()
        writer.writerows(frame_rows)

    finite_per_frame = np.isfinite(output_ranges).sum(axis=1)
    metadata = {
        "schema": "a_pipeline.dual_lidar_virtual360.v1",
        "source_bag": str(args.bag.resolve()),
        "topics": [args.scan_01_topic, args.scan_02_topic],
        "output_frame": normalize_frame(args.output_frame),
        "algorithm": "transform endpoints to output frame; self-filter; nearest range per angular slot",
        "matches_v7_dual_laser_scan_merger": args.self_occlusion_calibration is None,
        "self_occlusion_calibration": (
            str(args.self_occlusion_calibration.expanduser().resolve())
            if args.self_occlusion_calibration
            else None
        ),
        "fixed_self_masked_beams": {
            "scan_01": int(fixed_self_masks[1].sum()),
            "scan_02": int(fixed_self_masks[2].sum()),
        },
        "output": {
            "samples": args.output_samples,
            "angle_min_rad": args.angle_min,
            "angle_max_rad": args.angle_max,
            "angle_increment_rad": (args.angle_max - args.angle_min)
            / (args.output_samples - 1),
            "range_min_m": args.range_min,
            "range_max_m": args.range_max,
            "empty_slot_value": "+inf",
        },
        "self_filter_xy_m": {
            "min_x": args.self_filter_min_x,
            "max_x": args.self_filter_max_x,
            "min_y": args.self_filter_min_y,
            "max_y": args.self_filter_max_y,
        },
        "pairing": {
            "scan_01_messages": len(scans_01),
            "scan_02_messages": len(scans_02),
            "pairs": len(pairs),
            "skipped_scan_01": skipped_01,
            "skipped_scan_02": skipped_02,
            "tolerance_ms": args.sync_tolerance_ms,
            "max_observed_delta_ms": float(sync_delta_ns.max() / 1_000_000.0),
        },
        "coverage": {
            "finite_slots_mean": float(finite_per_frame.mean()),
            "finite_slots_min": int(finite_per_frame.min()),
            "finite_slots_max": int(finite_per_frame.max()),
            "finite_fraction_mean": float(finite_per_frame.mean() / args.output_samples),
        },
        "input_frames": sorted(frames),
        "source_to_output_transforms": {
            frame: matrix.tolist() for frame, matrix in sorted(transforms.items())
        },
        "static_tf_edges": static_edges,
        "arrays": {
            "ranges_m": list(output_ranges.shape),
            "valid_mask": list(output_ranges.shape),
            "stamp_ns": list(stamps.shape),
            "sync_delta_ns": list(sync_delta_ns.shape),
            "source_sensor": list(output_sensors.shape),
            "source_beam_index": list(output_beams.shape),
            "source_sensor_mask": list(output_masks.shape),
            "angles_rad": [args.output_samples],
        },
        "derived_rosbag": {
            "path": "rosbag",
            "topic": "/scan_merged",
            "type": "sensor_msgs/msg/LaserScan",
            "messages": len(pairs),
        },
        "npz_sha256": sha256(npz_path),
    }
    (args.output / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n"
    )
    (args.output / "README.md").write_text(
        "# 双雷达虚拟 360 数据\n\n"
        "该目录由 `convert_rosbag2_dual_lidar_to_virtual360.py` 生成。算法与项目"
        " `v7_dual_laser_scan_merger.py` 的默认 360 束 `/scan_merged` 路径一致："
        "将两路有效端点变换到 `base_link`，删除底盘包络内回波，按角度落入 360"
        " 个槽，并在同槽保留最近回波。\n\n"
        "`virtual360.npz` 保留 `ranges_m`、`valid_mask`、时间戳、获胜传感器/原始"
        "束号及每槽来源掩码。空槽使用 `+inf`；训练时应联合使用 `valid_mask`，"
        "不能直接把 `+inf` 输入网络。`frame_stats.csv` 是逐帧覆盖和槽竞争统计。\n\n"
        "`rosbag/` 只包含同一批 2459 帧 `/scan_merged`，可直接回放给现有 ROS 2"
        " 节点或在 RViz 中检查。\n\n"
        "这份虚拟扫描适合表示实验和 `/scan_merged` 回放，不替代 S3-Net/DRL-VO"
        " 当前各 2000 束的双路输入合同。\n"
    )
    print(json.dumps(metadata["pairing"], ensure_ascii=False))
    print(json.dumps(metadata["coverage"], ensure_ascii=False))
    print(npz_path)


if __name__ == "__main__":
    main()
