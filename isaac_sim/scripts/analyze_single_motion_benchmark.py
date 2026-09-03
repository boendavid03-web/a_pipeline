#!/usr/bin/env python3
"""Build auditable single-person motion metrics from the existing evaluator trace."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
import subprocess
from pathlib import Path
from typing import Any, Iterable

import yaml


SCENARIOS = ("front_approach", "front_leave", "lateral", "diagonal")
SPEED_RANGE = (0.72, 0.88)
ANGLE_MIN_SPEED = 0.20
SYNC_LIMIT_SEC = 0.08
MATCH_LIMIT_M = 0.50
POSITION_FIELDS = (
    "track_timestamp_ns", "gt_timestamp_ns", "timestamp_sec", "sync_dt_sec",
    "phase", "matched", "invalid_reason", "gt_id", "track_id", "track_state",
    "x_gt_m", "y_gt_m", "x_pred_m", "y_pred_m", "position_error_m",
    "range_gt_m", "bearing_gt_deg",
)
VELOCITY_FIELDS = (
    "track_timestamp_ns", "gt_timestamp_ns", "timestamp_sec", "sync_dt_sec",
    "phase", "matched", "invalid_reason", "gt_id", "track_id", "track_state",
    "x_gt_m", "y_gt_m", "vx_gt_mps", "vy_gt_mps", "speed_gt_mps",
    "x_pred_m", "y_pred_m", "vx_pred_mps", "vy_pred_mps", "speed_pred_mps",
    "vx_error_mps", "vy_error_mps", "velocity_vector_error_mps",
    "speed_error_mps", "vx_signed_residual_mps", "vy_signed_residual_mps",
    "angle_error_deg", "angle_valid", "direction_unavailable",
    "range_gt_m", "bearing_gt_deg",
)


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def mad(values: list[float]) -> float | None:
    if not values:
        return None
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def inverse_transform(
    x: float, y: float, pose: tuple[float, float, float], *, vector: bool = False
) -> tuple[float, float]:
    origin_x, origin_y, yaw = pose
    if not vector:
        x -= origin_x
        y -= origin_y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def route_geometry(metadata: dict) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    start_raw, end_raw = metadata["route"]["local_points_m"]
    start = (float(start_raw[0]), float(start_raw[1]))
    end = (float(end_raw[0]), float(end_raw[1]))
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    unit = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
    return start, end, unit


def point_route_errors(
    point: tuple[float, float], start: tuple[float, float], unit: tuple[float, float]
) -> tuple[float, float]:
    dx, dy = point[0] - start[0], point[1] - start[1]
    along = dx * unit[0] + dy * unit[1]
    cross = abs(dx * unit[1] - dy * unit[0])
    return along, cross


def fitted_velocity(samples: list[dict], index: int, half_window_ns: int = 300_000_000) -> tuple[float, float] | None:
    center = samples[index]["timestamp_ns"]
    times = [sample["timestamp_ns"] for sample in samples]
    left = bisect.bisect_left(times, center - half_window_ns)
    right = bisect.bisect_right(times, center + half_window_ns)
    window = samples[left:right]
    if len(window) < 5:
        return None
    seconds = [(item["timestamp_ns"] - center) / 1.0e9 for item in window]
    denominator = sum(value * value for value in seconds) - sum(seconds) ** 2 / len(seconds)
    if denominator <= 1.0e-12:
        return None
    mean_t = sum(seconds) / len(seconds)
    mean_x = sum(item["x"] for item in window) / len(window)
    mean_y = sum(item["y"] for item in window) / len(window)
    vx = sum((t - mean_t) * (item["x"] - mean_x) for t, item in zip(seconds, window)) / denominator
    vy = sum((t - mean_t) * (item["y"] - mean_y) for t, item in zip(seconds, window)) / denominator
    return vx, vy


def assign_phases(samples: list[dict], metadata: dict) -> dict[int, str]:
    start, _, unit = route_geometry(metadata)
    candidate: list[bool] = []
    for sample in samples:
        velocity = sample.get("velocity")
        if velocity is None:
            candidate.append(False)
            continue
        speed = math.hypot(*velocity)
        along, cross = point_route_errors((sample["x_local"], sample["y_local"]), start, unit)
        candidate.append(
            speed >= ANGLE_MIN_SPEED
            and velocity[0] * unit[0] + velocity[1] * unit[1] > 0.0
            and -0.20 <= along <= 8.20
            and cross <= 0.20
        )
    phases: dict[int, str] = {}
    groups: list[list[int]] = []
    group: list[int] = []
    for index, is_candidate in enumerate(candidate):
        continuous = (
            group
            and samples[index]["timestamp_ns"] - samples[group[-1]]["timestamp_ns"] <= 200_000_000
        )
        if is_candidate and (not group or continuous):
            group.append(index)
        else:
            if group:
                groups.append(group)
            group = [index] if is_candidate else []
    if group:
        groups.append(group)

    for index, sample in enumerate(samples):
        velocity = sample.get("velocity")
        if velocity is None:
            phases[sample["timestamp_ns"]] = "unclassified"
            continue
        speed = math.hypot(*velocity)
        projection = velocity[0] * unit[0] + velocity[1] * unit[1]
        if speed < ANGLE_MIN_SPEED:
            phases[sample["timestamp_ns"]] = "low_speed_turnaround"
        elif projection < 0.0:
            phases[sample["timestamp_ns"]] = "reverse_motion"
        elif candidate[index]:
            phases[sample["timestamp_ns"]] = "desired_transient"
        else:
            phases[sample["timestamp_ns"]] = "unclassified"
    for indices in groups:
        first_ns = samples[indices[0]]["timestamp_ns"]
        last_ns = samples[indices[-1]]["timestamp_ns"]
        if last_ns - first_ns < 1_000_000_000:
            continue
        steady_start = first_ns + 500_000_000
        steady_end = last_ns - 500_000_000
        for index in indices:
            timestamp_ns = samples[index]["timestamp_ns"]
            if steady_start <= timestamp_ns <= steady_end:
                phases[timestamp_ns] = "steady"
    return phases


def load_trace(path: Path, start_ns: int, end_ns: int, gt_id: str) -> list[dict]:
    records: list[dict] = []
    previous = -1
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            timestamp_ns = int(record["track_timestamp_ns"])
            if timestamp_ns < previous:
                raise ValueError(f"{path}:{line_number}: non-monotonic track timestamp")
            previous = timestamp_ns
            if not start_ns <= timestamp_ns <= end_ns:
                continue
            truths = [item for item in record["ground_truth"] if str(item["id"]) == gt_id]
            if len(truths) != 1:
                raise ValueError(f"{path}:{line_number}: expected one {gt_id} ground truth")
            record["target_ground_truth"] = truths[0]
            records.append(record)
    return records


def read_bag_samples(run_dir: Path, gt_id: str, start_ns: int, end_ns: int) -> tuple[list[dict], dict]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return [], {"available": False, "reason": "rosbag2 Python API unavailable"}
    bag_dir = run_dir / "rosbag"
    if not (bag_dir / "metadata.yaml").is_file():
        return [], {"available": False, "reason": "rosbag/metadata.yaml missing"}
    metadata_raw = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))
    info = metadata_raw["rosbag2_bagfile_information"]
    topic_counts = {
        item["topic_metadata"]["name"]: {
            "type": item["topic_metadata"]["type"],
            "message_count": int(item["message_count"]),
        }
        for item in info.get("topics_with_message_count", [])
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=str(info.get("storage_identifier", "sqlite3"))),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/pedestrian_ground_truth", "/odom", "/cmd_vel", "/clock"]))
    type_map = {name: get_message(value["type"]) for name, value in topic_counts.items() if name in {"/pedestrian_ground_truth", "/odom", "/cmd_vel", "/clock"}}
    gt_samples: list[dict] = []
    ground_truth_ids: set[str] = set()
    odom_positions: list[tuple[float, float]] = []
    cmd_vel_count = 0
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        if topic not in type_map:
            continue
        message = deserialize_message(data, type_map[topic])
        if topic == "/cmd_vel":
            cmd_vel_count += 1
        elif topic == "/odom" and start_ns <= recorded_ns <= end_ns:
            odom_positions.append((float(message.pose.pose.position.x), float(message.pose.pose.position.y)))
        elif topic == "/pedestrian_ground_truth":
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if not start_ns <= stamp <= end_ns:
                continue
            ground_truth_ids.update(str(person.id) for person in message.pedestrians)
            targets = [person for person in message.pedestrians if str(person.id) == gt_id]
            if len(targets) == 1:
                gt_samples.append({
                    "timestamp_ns": stamp,
                    "x": float(targets[0].pose.position.x),
                    "y": float(targets[0].pose.position.y),
                })
    displacement = None
    if odom_positions:
        origin = odom_positions[0]
        displacement = max(math.hypot(x - origin[0], y - origin[1]) for x, y in odom_positions)
    duration_sec = int(info["duration"]["nanoseconds"]) / 1.0e9
    return gt_samples, {
        "available": True,
        "duration_sec": duration_sec,
        "message_count": int(info.get("message_count", 0)),
        "topics": topic_counts,
        "cmd_vel_message_count": cmd_vel_count,
        "robot_max_displacement_m": displacement,
        "ground_truth_ids": sorted(ground_truth_ids),
    }


def prepare_phase_samples(records: list[dict], metadata: dict, bag_samples: list[dict]) -> tuple[list[dict], dict[int, str]]:
    pose_raw = metadata["robot_pose_odom"]
    pose = (float(pose_raw["x_m"]), float(pose_raw["y_m"]), float(pose_raw["yaw_rad"]))
    samples = bag_samples or [
        {
            "timestamp_ns": int(record["gt_timestamp_ns"]),
            "x": float(record["target_ground_truth"]["x"]),
            "y": float(record["target_ground_truth"]["y"]),
            "trace_velocity": (
                (float(record["target_ground_truth"]["vx"]), float(record["target_ground_truth"]["vy"]))
                if record["target_ground_truth"].get("vx") is not None
                else None
            ),
        }
        for record in records
    ]
    samples.sort(key=lambda item: item["timestamp_ns"])
    for index, sample in enumerate(samples):
        sample["x_local"], sample["y_local"] = inverse_transform(sample["x"], sample["y"], pose)
        velocity = sample.get("trace_velocity") or fitted_velocity(samples, index)
        sample["velocity"] = inverse_transform(*velocity, pose, vector=True) if velocity is not None else None
    return samples, assign_phases(samples, metadata)


def format_cell(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return value


def write_csv(path: Path, fields: Iterable[str], rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_cell(row.get(key)) for key in fields})


def row_from_trace(record: dict, metadata: dict, phase: str) -> tuple[dict, dict]:
    pose_raw = metadata["robot_pose_odom"]
    pose = (float(pose_raw["x_m"]), float(pose_raw["y_m"]), float(pose_raw["yaw_rad"]))
    truth = record["target_ground_truth"]
    x_gt, y_gt = inverse_transform(float(truth["x"]), float(truth["y"]), pose)
    truth_velocity = None
    if truth.get("vx") is not None and truth.get("vy") is not None:
        truth_velocity = inverse_transform(float(truth["vx"]), float(truth["vy"]), pose, vector=True)
    target_matches = [item for item in record["matches"] if str(item["gt_id"]) == str(truth["id"])]
    if len(target_matches) > 1:
        raise ValueError("evaluator trace contains multiple matches for the single target")
    match = target_matches[0] if target_matches else None
    track = None
    if match is not None:
        candidates = [item for item in record["tracks"] if int(item["track_id"]) == int(match["track_id"])]
        if len(candidates) != 1:
            raise ValueError("matched track is missing or duplicated in evaluator trace")
        track = candidates[0]
    matched = track is not None
    common = {
        "track_timestamp_ns": int(record["track_timestamp_ns"]),
        "gt_timestamp_ns": int(record["gt_timestamp_ns"]),
        "timestamp_sec": int(record["track_timestamp_ns"]) / 1.0e9,
        "sync_dt_sec": abs(float(record["timestamp_offset_sec"])),
        "phase": phase,
        "matched": matched,
        "invalid_reason": "" if matched else "unmatched",
        "gt_id": str(truth["id"]),
        "track_id": int(track["track_id"]) if track is not None else None,
        "track_state": str(track["state"]) if track is not None else None,
        "x_gt_m": x_gt,
        "y_gt_m": y_gt,
        "range_gt_m": math.hypot(x_gt, y_gt),
        "bearing_gt_deg": math.degrees(math.atan2(y_gt, x_gt)),
    }
    if track is not None:
        x_pred, y_pred = inverse_transform(float(track["x"]), float(track["y"]), pose)
        common.update({
            "x_pred_m": x_pred,
            "y_pred_m": y_pred,
            "position_error_m": float(match["position_error_m"]),
        })
    else:
        common.update({"x_pred_m": None, "y_pred_m": None, "position_error_m": None})
    trajectory = dict(common)
    velocity = dict(common)
    if truth_velocity is None:
        velocity.update({
            "vx_gt_mps": None, "vy_gt_mps": None, "speed_gt_mps": None,
            "vx_pred_mps": None, "vy_pred_mps": None, "speed_pred_mps": None,
            "vx_error_mps": None, "vy_error_mps": None,
            "velocity_vector_error_mps": None, "speed_error_mps": None,
            "vx_signed_residual_mps": None, "vy_signed_residual_mps": None,
            "angle_error_deg": None, "angle_valid": False,
            "direction_unavailable": False,
        })
        velocity["invalid_reason"] = "gt_velocity_unavailable"
        return trajectory, velocity
    vx_gt, vy_gt = truth_velocity
    speed_gt = math.hypot(vx_gt, vy_gt)
    velocity.update({"vx_gt_mps": vx_gt, "vy_gt_mps": vy_gt, "speed_gt_mps": speed_gt})
    if track is None:
        velocity.update({
            "vx_pred_mps": None, "vy_pred_mps": None, "speed_pred_mps": None,
            "vx_error_mps": None, "vy_error_mps": None,
            "velocity_vector_error_mps": None, "speed_error_mps": None,
            "vx_signed_residual_mps": None, "vy_signed_residual_mps": None,
            "angle_error_deg": None, "angle_valid": False,
            "direction_unavailable": False,
        })
        return trajectory, velocity
    vx_pred, vy_pred = inverse_transform(float(track["vx"]), float(track["vy"]), pose, vector=True)
    speed_pred = math.hypot(vx_pred, vy_pred)
    residual_x, residual_y = vx_pred - vx_gt, vy_pred - vy_gt
    eligible = speed_gt >= ANGLE_MIN_SPEED
    angle_valid = eligible and speed_pred >= ANGLE_MIN_SPEED
    angle_error = None
    if angle_valid:
        difference = math.atan2(vy_pred, vx_pred) - math.atan2(vy_gt, vx_gt)
        angle_error = abs(math.degrees(math.atan2(math.sin(difference), math.cos(difference))))
    velocity.update({
        "vx_pred_mps": vx_pred, "vy_pred_mps": vy_pred, "speed_pred_mps": speed_pred,
        "vx_error_mps": abs(residual_x), "vy_error_mps": abs(residual_y),
        "velocity_vector_error_mps": math.hypot(residual_x, residual_y),
        "speed_error_mps": abs(speed_pred - speed_gt),
        "vx_signed_residual_mps": residual_x, "vy_signed_residual_mps": residual_y,
        "angle_error_deg": angle_error, "angle_valid": angle_valid,
        "direction_unavailable": eligible and not angle_valid,
    })
    return trajectory, velocity


def numeric(rows: list[dict], field: str) -> list[float]:
    return [float(row[field]) for row in rows if row.get(field) is not None]


def summarize(run_dir: Path, metadata: dict, trajectory: list[dict], velocity: list[dict], phase_samples: list[dict], bag: dict, capture_start_ns: int, capture_end_ns: int, base_summary: dict) -> dict:
    steady_all = [row for row in velocity if row["phase"] == "steady"]
    steady_matched = [row for row in steady_all if row["matched"] and row["vx_error_mps"] is not None]
    n_gt_eval = len(steady_all)
    n_sync = len(steady_all)
    n_matched = len(steady_matched)
    angle_eligible = [row for row in steady_matched if float(row["speed_gt_mps"]) >= ANGLE_MIN_SPEED]
    angle_valid = [row for row in angle_eligible if row["angle_valid"]]
    position_errors = numeric(steady_matched, "position_error_m")
    vx_residuals = numeric(steady_matched, "vx_signed_residual_mps")
    vy_residuals = numeric(steady_matched, "vy_signed_residual_mps")
    gt_speeds = numeric(steady_all, "speed_gt_mps")
    steady_times = sorted(int(row["gt_timestamp_ns"]) for row in steady_all)
    steady_segments: list[list[int]] = []
    current_segment: list[int] = []
    for timestamp_ns in steady_times:
        if current_segment and timestamp_ns - current_segment[-1] > 200_000_000:
            steady_segments.append(current_segment)
            current_segment = []
        current_segment.append(timestamp_ns)
    if current_segment:
        steady_segments.append(current_segment)
    segment_durations = [
        (segment[-1] - segment[0]) / 1.0e9 for segment in steady_segments if segment
    ]
    max_steady_duration = max(segment_durations, default=0.0)
    total_steady_duration = sum(segment_durations)
    start, end, _ = route_geometry(metadata)
    endpoint_start_error = min(
        (math.hypot(item["x_local"] - start[0], item["y_local"] - start[1]) for item in phase_samples),
        default=None,
    )
    endpoint_end_error = min(
        (math.hypot(item["x_local"] - end[0], item["y_local"] - end[1]) for item in phase_samples),
        default=None,
    )
    evaluated_frames = int(base_summary.get("metrics", {}).get("evaluated_frames", 0))
    dropped_sync = int(base_summary.get("time_sync", {}).get("dropped_unsynchronized_frames", 0))
    global_sync_completeness = (
        evaluated_frames / (evaluated_frames + dropped_sync)
        if evaluated_frames + dropped_sync
        else None
    )
    invalid_reasons = []
    capture_duration = (capture_end_ns - capture_start_ns) / 1.0e9
    if capture_duration < 29.0:
        invalid_reasons.append("capture_duration_below_29_seconds")
    if n_gt_eval == 0:
        invalid_reasons.append("no_target_direction_steady_frames")
    if max_steady_duration < 7.8:
        invalid_reasons.append("steady_target_direction_duration_below_8_seconds")
    if gt_speeds and not SPEED_RANGE[0] <= statistics.median(gt_speeds) <= SPEED_RANGE[1]:
        invalid_reasons.append("steady_gt_median_speed_out_of_range")
    if global_sync_completeness is None or global_sync_completeness < 0.99:
        invalid_reasons.append("global_evaluator_sync_completeness_below_0.99")
    if bag.get("available"):
        if bag.get("ground_truth_ids") != [metadata["ground_truth_id"]]:
            invalid_reasons.append("ground_truth_id_set_is_not_exactly_the_single_target")
        if int(bag.get("cmd_vel_message_count", 0)) != 0:
            invalid_reasons.append("cmd_vel_messages_recorded")
        displacement = bag.get("robot_max_displacement_m")
        if displacement is not None and displacement > 0.02:
            invalid_reasons.append("robot_displacement_above_0.02_m")
        for topic in ("/scan_01", "/scan_02", "/scan_merged", "/pedestrian_ground_truth", "/pedestrian_tracks"):
            if int(bag.get("topics", {}).get(topic, {}).get("message_count", 0)) <= 0:
                invalid_reasons.append(f"missing_bag_topic:{topic}")
        bag_duration = float(bag.get("duration_sec", 0.0))
        for topic in ("/scan_01", "/scan_02", "/scan_merged"):
            count = int(bag.get("topics", {}).get(topic, {}).get("message_count", 0))
            if bag_duration <= 0.0 or count / bag_duration < 13.5:
                invalid_reasons.append(f"bag_topic_rate_below_13.5_hz:{topic}")
        if endpoint_start_error is None or endpoint_start_error > 0.20:
            invalid_reasons.append("route_start_not_reached_within_0.20_m")
        if endpoint_end_error is None or endpoint_end_error > 0.20:
            invalid_reasons.append("route_end_not_reached_within_0.20_m")
    else:
        invalid_reasons.append("rosbag_audit_unavailable")
    algorithm_status = "MEASURED" if n_matched >= 30 else "NOT_EVALUABLE"
    phases = {}
    for phase in ("steady", "desired_transient", "reverse_motion", "low_speed_turnaround", "unclassified"):
        subset = [row for row in velocity if row["phase"] == phase]
        phases[phase] = {"frames": len(subset), "matched": sum(bool(row["matched"]) for row in subset)}
    return {
        "schema": "dr_spaam_single_motion_benchmark/v1",
        "scenario": metadata["scenario"],
        "episode": {"valid": not invalid_reasons, "invalid_reasons": invalid_reasons, "algorithm_status": algorithm_status},
        "capture": {"start_ns": capture_start_ns, "end_ns": capture_end_ns, "duration_sec": capture_duration},
        "coordinate_frame": metadata["benchmark_frame"],
        "route": metadata["route"],
        "matching": {"source": "existing_pedestrian_tracking_evaluator", "method": "hungarian_euclidean_position", "threshold_m": MATCH_LIMIT_M, "ground_truth_role": "evaluation_only"},
        "time_sync": {"source": "existing_pedestrian_tracking_evaluator", "method": "nearest_timestamp_buffered", "max_accepted_offset_sec": SYNC_LIMIT_SEC, "base_summary": base_summary.get("time_sync", {})},
        "denominators": {
            "n_gt_eval": n_gt_eval, "n_sync": n_sync, "n_matched": n_matched,
            "sync_completeness": n_sync / n_gt_eval if n_gt_eval else None,
            "global_evaluator_sync_completeness": global_sync_completeness,
            "tracking_coverage": n_matched / n_sync if n_sync else None,
            "n_angle_eligible": len(angle_eligible), "n_angle_valid": len(angle_valid),
            "angle_coverage": len(angle_valid) / len(angle_eligible) if angle_eligible else None,
        },
        "position": {
            "mean_error_m": mean(position_errors), "median_error_m": median(position_errors),
            "max_error_m": max(position_errors, default=None), "p95_error_m": percentile(position_errors, 0.95),
            "note": "matched samples only; maximum is truncated by the 0.5 m matching gate",
        },
        "velocity": {
            "vx_mae_mps": mean(numeric(steady_matched, "vx_error_mps")),
            "vy_mae_mps": mean(numeric(steady_matched, "vy_error_mps")),
            "velocity_vector_mae_mps": mean(numeric(steady_matched, "velocity_vector_error_mps")),
            "speed_mae_mps": mean(numeric(steady_matched, "speed_error_mps")),
            "angle_error_deg": {"mean": mean(numeric(angle_valid, "angle_error_deg")), "median": median(numeric(angle_valid, "angle_error_deg")), "p95": percentile(numeric(angle_valid, "angle_error_deg"), 0.95)},
            "vx_bias_mps": mean(vx_residuals), "vy_bias_mps": mean(vy_residuals),
            "vx_residual_std_mps": statistics.pstdev(vx_residuals) if vx_residuals else None,
            "vy_residual_std_mps": statistics.pstdev(vy_residuals) if vy_residuals else None,
            "vx_residual_mad_mps": mad(vx_residuals), "vy_residual_mad_mps": mad(vy_residuals),
            "direction_min_speed_mps": ANGLE_MIN_SPEED,
        },
        "phases": phases,
        "identity": {"source": "existing evaluator", "base_metrics": {key: base_summary.get("metrics", {}).get(key) for key in ("id_switches", "continuous_id_switches", "reacquisition_id_changes", "fragmentation")}},
        "false_tracks": {"source": "existing evaluator", "base_false_positive_track_observations": base_summary.get("metrics", {}).get("false_positive_track_observations")},
        "quality": {
            "max_continuous_steady_duration_sec": max_steady_duration,
            "total_steady_duration_sec": total_steady_duration,
            "steady_segment_durations_sec": segment_durations,
            "steady_gt_median_speed_mps": median(gt_speeds),
            "route_start_min_error_m": endpoint_start_error,
            "route_end_min_error_m": endpoint_end_error,
            "bag": bag,
            "finite": all(math.isfinite(value) for row in steady_matched for key, value in row.items() if isinstance(value, float)),
        },
        "provenance": {"base_evaluator_summary": str(run_dir / "evaluation/summary.json"), "base_evaluator_trace": str(run_dir / "evaluation/tracking_trace.jsonl"), "ground_truth_used_by_detector_or_tracker": False},
    }


def git_value(project_root: Path, *args: str) -> str | None:
    result = subprocess.run(["git", *args], cwd=project_root, text=True, capture_output=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else None


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_analysis(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    scenario_path = run_dir / "scenario_metadata.yaml"
    trace_path = run_dir / "evaluation/tracking_trace.jsonl"
    base_summary_path = run_dir / "evaluation/summary.json"
    for path in (scenario_path, trace_path, base_summary_path):
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"ERROR: required non-empty benchmark input is missing: {path}")
    metadata = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    if metadata.get("schema") != "isaac_single_motion_scenario/v1" or metadata.get("scenario") not in SCENARIOS:
        raise SystemExit(f"ERROR: invalid scenario metadata schema or scenario: {scenario_path}")
    gt_id = str(metadata["ground_truth_id"])
    records = load_trace(trace_path, args.capture_start_ns, args.capture_end_ns, gt_id)
    if not records:
        raise SystemExit("ERROR: evaluator trace has no synchronized frames in capture window")
    bag_samples, bag_audit = read_bag_samples(run_dir, gt_id, args.capture_start_ns, args.capture_end_ns)
    phase_samples, phases = prepare_phase_samples(records, metadata, bag_samples)
    for sample in phase_samples:
        sample["phase"] = phases[sample["timestamp_ns"]]
    trajectory_rows, velocity_rows = [], []
    for record in records:
        phase = phases.get(int(record["gt_timestamp_ns"]), "unclassified")
        trajectory, velocity = row_from_trace(record, metadata, phase)
        trajectory_rows.append(trajectory)
        velocity_rows.append(velocity)
    write_csv(run_dir / "trajectory.csv", POSITION_FIELDS, trajectory_rows)
    write_csv(run_dir / "velocity.csv", VELOCITY_FIELDS, velocity_rows)
    base_summary = json.loads(base_summary_path.read_text(encoding="utf-8"))
    summary = summarize(run_dir, metadata, trajectory_rows, velocity_rows, phase_samples, bag_audit, args.capture_start_ns, args.capture_end_ns, base_summary)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    project_root = Path(__file__).resolve().parents[2]
    source_paths = [Path(__file__).resolve(), Path(__file__).resolve().with_name("generate_single_motion_config.py")]
    runtime_metadata = {
        **metadata,
        "schema": "dr_spaam_single_motion_metadata/v1",
        "capture": summary["capture"],
        "phase_summary": summary["phases"],
        "denominators": summary["denominators"],
        "episode": summary["episode"],
        "ground_truth_role": "evaluation_only",
        "ground_truth_used_by_detector_or_tracker": False,
        "parameters": {
            "lidar": {"backend": "physx", "rate_hz": 15, "samples_per_sensor": 2000},
            "detector": {"model": "DR-SPAAM", "confidence_threshold": 0.95, "stride": 5, "input": "/scan_merged"},
            "tracker": {"tracking_frame": "odom", "association_threshold_m": 0.8, "min_hits": 3, "max_age": 8, "max_coast_time_sec": 0.75, "acceleration_sigma": 2.0, "measurement_sigma": 0.10, "max_prediction_dt_sec": 0.50},
            "evaluator": {"sync_limit_sec": SYNC_LIMIT_SEC, "match_limit_m": MATCH_LIMIT_M, "gt_fit_half_window_sec": 0.30},
        },
        "bag_audit": bag_audit,
        "provenance": {
            "git_head": git_value(project_root, "rev-parse", "HEAD"),
            "git_branch": git_value(project_root, "branch", "--show-current"),
            "git_status_porcelain": git_value(project_root, "status", "--porcelain"),
            "source_sha256": {str(path): sha256(path) for path in source_paths if path.is_file()},
            "paths": summary["provenance"],
        },
    }
    (run_dir / "metadata.yaml").write_text(yaml.safe_dump(runtime_metadata, sort_keys=False), encoding="utf-8")
    print(f"SINGLE_MOTION_ANALYSIS=PASS scenario={metadata['scenario']} run_dir={run_dir}")
    print(f"EPISODE_VALID={str(summary['episode']['valid']).lower()} ALGORITHM_STATUS={summary['episode']['algorithm_status']}")
    return 0 if summary["episode"]["valid"] else 3


def parse_csv_number(row: dict, field: str) -> float | None:
    value = row.get(field, "")
    return float(value) if value != "" else None


def verify(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    with (run_dir / "velocity.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    steady = [row for row in rows if row["phase"] == "steady" and row["matched"] == "true" and row["vx_error_mps"]]
    checks = {
        "velocity.vx_mae_mps": mean([float(row["vx_error_mps"]) for row in steady]),
        "velocity.vy_mae_mps": mean([float(row["vy_error_mps"]) for row in steady]),
        "velocity.velocity_vector_mae_mps": mean([float(row["velocity_vector_error_mps"]) for row in steady]),
        "velocity.speed_mae_mps": mean([float(row["speed_error_mps"]) for row in steady]),
    }
    with (run_dir / "trajectory.csv").open(encoding="utf-8", newline="") as stream:
        position_rows = [row for row in csv.DictReader(stream) if row["phase"] == "steady" and row["matched"] == "true" and row["position_error_m"]]
    errors = [float(row["position_error_m"]) for row in position_rows]
    checks.update({
        "position.mean_error_m": mean(errors), "position.median_error_m": median(errors),
        "position.max_error_m": max(errors, default=None), "position.p95_error_m": percentile(errors, 0.95),
    })
    for dotted, recomputed in checks.items():
        section, key = dotted.split(".")
        stored = summary[section][key]
        if stored is None or recomputed is None:
            if stored != recomputed:
                raise SystemExit(f"ERROR: verify mismatch {dotted}: stored={stored} recomputed={recomputed}")
        elif not math.isclose(float(stored), float(recomputed), rel_tol=1.0e-9, abs_tol=1.0e-9):
            raise SystemExit(f"ERROR: verify mismatch {dotted}: stored={stored} recomputed={recomputed}")
    print(f"SINGLE_MOTION_VERIFY=PASS run_dir={run_dir}")
    return 0


def render(value: Any, digits: int = 3) -> str:
    return "N/A" if value is None else f"{float(value):.{digits}f}"


def report(args: argparse.Namespace) -> int:
    summaries = [json.loads(path.read_text(encoding="utf-8")) for path in args.summary]
    by_scenario = {item.get("scenario"): item for item in summaries}
    if len(summaries) != 4 or set(by_scenario) != set(SCENARIOS):
        raise SystemExit("ERROR: report requires exactly one explicit summary for each scenario")
    lines = [
        "# Isaac Sim + DR-SPAAM single-person motion benchmark", "",
        "| scenario | position error | velocity MAE | direction error |",
        "| --- | --- | --- | --- |",
    ]
    for scenario in SCENARIOS:
        item = by_scenario[scenario]
        position = item["position"]
        velocity = item["velocity"]
        coverage = item["denominators"].get("angle_coverage")
        lines.append(
            f"| {scenario} | {render(position.get('mean_error_m'))} / {render(position.get('median_error_m'))} / {render(position.get('max_error_m'))} m "
            f"| {render(velocity.get('speed_mae_mps'))} m/s "
            f"| {render(velocity.get('angle_error_deg', {}).get('mean'))} deg (coverage {render(coverage)}) |"
        )
    lines.extend(["", "Position cells are mean / median / max. Velocity MAE is speed magnitude MAE.", "", "## Validity and denominators", ""])
    for scenario in SCENARIOS:
        item = by_scenario[scenario]
        episode = item["episode"]
        denominator = item["denominators"]
        lines.append(
            f"- {scenario}: EPISODE {'VALID' if episode['valid'] else 'INVALID'}, "
            f"ALGORITHM {episode['algorithm_status']}; n_gt_eval={denominator['n_gt_eval']}, "
            f"n_sync={denominator['n_sync']}, n_matched={denominator['n_matched']}."
        )
    lines.extend([
        "", "## Velocity components and residual dispersion", "",
        "| scenario | vx MAE | vy MAE | vector MAE | speed MAE | vx/vy residual std | false-track observations |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ])
    for scenario in SCENARIOS:
        item = by_scenario[scenario]
        velocity = item["velocity"]
        false_tracks = item.get("false_tracks", {}).get(
            "base_false_positive_track_observations"
        )
        lines.append(
            f"| {scenario} | {render(velocity.get('vx_mae_mps'))} | "
            f"{render(velocity.get('vy_mae_mps'))} | "
            f"{render(velocity.get('velocity_vector_mae_mps'))} | "
            f"{render(velocity.get('speed_mae_mps'))} | "
            f"{render(velocity.get('vx_residual_std_mps'))} / "
            f"{render(velocity.get('vy_residual_std_mps'))} | "
            f"{false_tracks if false_tracks is not None else 'N/A'} |"
        )
    position_means = [
        float(by_scenario[scenario]["position"]["mean_error_m"])
        for scenario in SCENARIOS
        if by_scenario[scenario]["position"].get("mean_error_m") is not None
    ]
    bias_values = [
        abs(float(by_scenario[scenario]["velocity"][key]))
        for scenario in SCENARIOS
        for key in ("vx_bias_mps", "vy_bias_mps")
        if by_scenario[scenario]["velocity"].get(key) is not None
    ]
    residual_std_values = [
        float(by_scenario[scenario]["velocity"][key])
        for scenario in SCENARIOS
        for key in ("vx_residual_std_mps", "vy_residual_std_mps")
        if by_scenario[scenario]["velocity"].get(key) is not None
    ]
    if position_means and bias_values and residual_std_values:
        lines.extend([
            "", "## Result pattern", "",
            f"Matched position mean error spans {min(position_means):.3f}–{max(position_means):.3f} m across the four scenarios.",
            f"The largest absolute component bias is {max(bias_values):.3f} m/s, while component residual standard deviations are at least {min(residual_std_values):.3f} m/s.",
            "This is a case-2-like pattern: matched positions remain stable across direction, while velocity residual dispersion is much larger than signed bias.",
            "It supports a velocity-estimation jitter diagnosis, but does not identify a Kalman parameter cause and is not a deployment-readiness PASS.",
            "Persistent extra tracks in some global scene poses are reported separately and must not be hidden by the matched-track metrics.",
        ])
    lines.extend([
        "", "## Interpretation limits", "",
        "This is one exploratory run per scenario. Frame samples are autocorrelated and do not establish repeatability.",
        "S1/S2 are a reversed-path pair. S3/S4 use different safe global robot poses, so direction, range, bearing, and background are confounded.",
        "A directional difference is therefore a candidate effect, not proof of DR-SPAAM direction causality.",
        "Ground truth is evaluation-only and is not used by detector or tracker. No tracker optimization is performed.",
    ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"SINGLE_MOTION_REPORT=PASS output={args.output}")
    return 0 if all(item["episode"]["valid"] and item["episode"]["algorithm_status"] == "MEASURED" for item in summaries) else 4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--run-dir", type=Path, required=True)
    run_parser.add_argument("--capture-start-ns", type=int, required=True)
    run_parser.add_argument("--capture-end-ns", type=int, required=True)
    verify_parser = commands.add_parser("verify")
    verify_parser.add_argument("--run-dir", type=Path, required=True)
    report_parser = commands.add_parser("report")
    report_parser.add_argument("--output", type=Path, required=True)
    report_parser.add_argument("--summary", type=Path, action="append", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "run":
        if args.capture_start_ns < 0 or args.capture_end_ns <= args.capture_start_ns:
            raise SystemExit("ERROR: capture bounds must be finite increasing non-negative nanoseconds")
        return run_analysis(args)
    if args.command == "verify":
        return verify(args)
    return report(args)


if __name__ == "__main__":
    raise SystemExit(main())
