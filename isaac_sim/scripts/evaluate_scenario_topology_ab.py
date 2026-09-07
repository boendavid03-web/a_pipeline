#!/usr/bin/env python3
"""Offline evaluator for matched scenario-topology A/B Isaac runs.

This tool reads an Isaac run directory, its steering JSONL, final runtime result,
and the authored people configuration.  It never imports or controls Isaac.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


TRACE_SCHEMA = "isaac_pedestrian_social_steering/v2"
MAX_VALID_GAP_SEC = 0.5
VISUAL_OVERLAP_M = 0.45
PERSONAL_SPACE_M = 1.0
INTENT_MIN_MPS = 0.2
FREEZE_ACTUAL_MAX_MPS = 0.05
CONGESTION_PREFERRED_FRACTION = 0.25
TARGET_CURRENT_TOLERANCE_M = 1.0e-6
GRID_SIZE_M = 2.0
HOTSPOTS = {
    "northeast_22_20": (20.0, 24.0, 18.0, 22.0),
    "west_4_6_10_13": (4.0, 6.0, 10.0, 13.0),
    "north_center_13_15_18_20": (13.0, 15.0, 18.0, 20.0),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: Iterable[float], q: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def safe_ratio(numerator: float, denominator: float) -> float | None:
    return numerator / denominator if denominator else None


def vector_norm(value: Any) -> float:
    if not isinstance(value, list) or len(value) < 2:
        return 0.0
    return math.hypot(float(value[0]), float(value[1]))


def free_space_selected_target(person: dict[str, Any]) -> tuple[Any, bool]:
    """Return the exact guard output when available, else the legacy proxy.

    Older v2 traces exposed only ``applied_target_m``, which is the last target
    actually written after the minimum-shift gate.  It can therefore hide a
    current-position fallback.  New traces retain that field for compatibility
    and add ``free_space_selected_target_m`` before the write gate.
    """

    exact = person.get("free_space_selected_target_m")
    if exact is not None:
        return exact, True
    return person.get("applied_target_m"), False


def short_person_id(runtime_path: str) -> str:
    parts = runtime_path.split("/")
    try:
        return parts[parts.index("Characters") + 1]
    except (ValueError, IndexError):
        return runtime_path


def load_runtime_result(log_path: Path) -> dict[str, Any]:
    marker = "WAREHOUSE_PEOPLE_ROBOT_RESULT="
    result = None
    with log_path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith(marker):
                result = json.loads(line[len(marker) :])
    if result is None:
        raise ValueError(f"missing final RESULT marker in {log_path}")
    return result


def load_config(config_path: Path) -> dict[str, dict[str, Any]]:
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    groups = document["isaacsim.replicator.agent"]["character"]["groups"]
    people: dict[str, dict[str, Any]] = {}
    for person_id, group in groups.items():
        patrol = group["routines"][0]["patrol"]
        points = [tuple(map(float, point[:2])) for point in patrol["path_points"]]
        if len(points) < 2:
            raise ValueError(f"{person_id} has fewer than two patrol points")
        loop_length = sum(
            math.dist(points[index], points[index + 1])
            for index in range(len(points) - 1)
        )
        if points[-1] != points[0]:
            loop_length += math.dist(points[-1], points[0])
        people[person_id] = {
            "preferred_speed_mps": float(patrol["speed_range"][0]),
            "path_point_count": len(points),
            "authored_loop_length_m": loop_length,
            "spawn_position_m": list(points[0]),
        }
    return people


def load_semantic_waypoint_counts(manifest_path: Path) -> dict[str, int]:
    document = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    return {
        cluster["cluster_id"]: len(cluster["waypoints"])
        for cluster in document["logical_topology_snapshot"]["waypoint_clusters"]
    }


@dataclass
class EpisodeAccumulator:
    total_sec: float = 0.0
    count: int = 0
    maximum_sec: float = 0.0
    current_sec: float = 0.0
    active: bool = False

    def update(self, condition: bool, dt: float, gap: bool) -> None:
        # A long app-update interval is observable, but it is not proof that
        # simulator state was missing.  Match the runtime accumulator by
        # integrating every positive monotonic dt and report long intervals
        # separately instead of discarding or splitting them.
        del gap
        if condition:
            if not self.active:
                self.count += 1
                self.active = True
                self.current_sec = 0.0
            self.total_sec += dt
            self.current_sec += dt
            self.maximum_sec = max(self.maximum_sec, self.current_sec)
        else:
            self.active = False
            self.current_sec = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "total_sec": self.total_sec,
            "episode_count": self.count,
            "maximum_continuous_sec": self.maximum_sec,
        }


def new_spatial_bucket() -> dict[str, float]:
    return {
        "person_samples": 0,
        "person_time_sec": 0.0,
        "overlap_lt_0_45_samples": 0,
        "overlap_lt_0_45_person_sec": 0.0,
        "personal_space_lt_1_0_samples": 0,
        "personal_space_lt_1_0_person_sec": 0.0,
        "congestion_samples": 0,
        "congestion_person_sec": 0.0,
        "constraint_samples": 0,
        "constraint_person_sec": 0.0,
        "freeze_samples": 0,
        "freeze_person_sec": 0.0,
    }


def update_spatial_bucket(
    bucket: dict[str, float],
    dt: float,
    nearest: float,
    congestion: bool,
    constrained: bool,
    frozen: bool,
) -> None:
    bucket["person_samples"] += 1
    bucket["person_time_sec"] += dt
    conditions = {
        "overlap_lt_0_45": nearest < VISUAL_OVERLAP_M,
        "personal_space_lt_1_0": nearest < PERSONAL_SPACE_M,
        "congestion": congestion,
        "constraint": constrained,
        "freeze": frozen,
    }
    for name, condition in conditions.items():
        if condition:
            bucket[f"{name}_samples"] += 1
            bucket[f"{name}_person_sec"] += dt


def trace_metrics(trace_path: Path, expected_ids: set[str]) -> dict[str, Any]:
    frame_minima: list[float] = []
    nearest_values: list[float] = []
    pair_observations = 0
    visual_pair_observations = 0
    personal_pair_observations = 0
    sample_count = 0
    person_sample_count = 0
    first_time = None
    previous_time = None
    valid_integrated_sec = 0.0
    missing_interval_count = 0
    missing_interval_sec = 0.0
    maximum_gap_sec = 0.0
    observed_ids: set[str] = set()

    trackers: dict[str, dict[str, EpisodeAccumulator]] = defaultdict(
        lambda: {
            name: EpisodeAccumulator()
            for name in (
                "congestion",
                "freeze_inclusive",
                "freeze_exclusive",
                "constraint",
                "emergency_inhibited",
            )
        }
    )
    target_current_fallback_count: dict[str, int] = defaultdict(int)
    free_space_decision_count: dict[str, int] = defaultdict(int)
    exact_selected_target_samples = 0
    lateral_weighted_sum: dict[str, float] = defaultdict(float)
    lateral_weight_sec = 0.0
    hotspots = {name: new_spatial_bucket() for name in HOTSPOTS}
    grid: dict[str, dict[str, float]] = defaultdict(new_spatial_bucket)
    latest_people: dict[str, Any] = {}

    with trace_path.open(encoding="utf-8") as stream:
        header = json.loads(next(stream))
        if header.get("schema") != TRACE_SCHEMA or header.get("type") != "header":
            raise ValueError(f"unexpected steering header in {trace_path}")
        for line in stream:
            record = json.loads(line)
            if record.get("schema") != TRACE_SCHEMA or record.get("type") != "sample":
                raise ValueError(f"unexpected steering record in {trace_path}")
            sim_time = float(record["sim_time"])
            if first_time is None:
                first_time = sim_time
            raw_dt = 0.0 if previous_time is None else sim_time - previous_time
            if raw_dt < 0.0:
                raise ValueError(f"non-monotonic steering time in {trace_path}")
            maximum_gap_sec = max(maximum_gap_sec, raw_dt)
            gap = raw_dt > MAX_VALID_GAP_SEC
            if gap:
                missing_interval_count += 1
                missing_interval_sec += raw_dt
            dt = raw_dt if raw_dt > 0.0 else 0.0
            valid_integrated_sec += dt
            previous_time = sim_time

            people = record["people"]
            latest_people = people
            ids = {short_person_id(path) for path in people}
            observed_ids.update(ids)
            if ids != expected_ids:
                raise ValueError(
                    f"steering IDs differ from config in {trace_path}: "
                    f"missing={sorted(expected_ids - ids)} extra={sorted(ids - expected_ids)}"
                )
            sample_count += 1
            person_sample_count += len(people)
            paths = list(people)
            positions = {
                path: tuple(map(float, people[path]["position_m"][:2]))
                for path in paths
            }
            nearest_by_path = {path: math.inf for path in paths}
            frame_pairs: list[float] = []
            for index, left in enumerate(paths):
                for right in paths[index + 1 :]:
                    distance = math.dist(positions[left], positions[right])
                    frame_pairs.append(distance)
                    nearest_by_path[left] = min(nearest_by_path[left], distance)
                    nearest_by_path[right] = min(nearest_by_path[right], distance)
            pair_observations += len(frame_pairs)
            visual_pair_observations += sum(
                distance < VISUAL_OVERLAP_M for distance in frame_pairs
            )
            personal_pair_observations += sum(
                distance < PERSONAL_SPACE_M for distance in frame_pairs
            )
            if frame_pairs:
                frame_minima.append(min(frame_pairs))
                nearest_values.extend(nearest_by_path.values())

            for path, person in people.items():
                person_id = short_person_id(path)
                nearest = nearest_by_path[path]
                preferred = float(person["preferred_speed_mps"])
                actual_speed = vector_norm(person["actual_navigation_velocity_mps"])
                intent_speed = vector_norm(person["isaac_adapter_output_velocity_mps"])
                inhibited = bool(person.get("emergency_inhibited", False))
                constrained = bool(person.get("free_space_constrained", False))
                congestion = (
                    nearest < PERSONAL_SPACE_M
                    and actual_speed < CONGESTION_PREFERRED_FRACTION * preferred
                    and intent_speed >= INTENT_MIN_MPS
                    and not inhibited
                )
                freeze_inclusive = (
                    intent_speed >= INTENT_MIN_MPS
                    and actual_speed < FREEZE_ACTUAL_MAX_MPS
                )
                freeze_exclusive = freeze_inclusive and not inhibited
                states = {
                    "congestion": congestion,
                    "freeze_inclusive": freeze_inclusive,
                    "freeze_exclusive": freeze_exclusive,
                    "constraint": constrained,
                    "emergency_inhibited": inhibited,
                }
                for name, condition in states.items():
                    trackers[person_id][name].update(condition, dt, gap)

                selected_target, selected_target_is_exact = (
                    free_space_selected_target(person)
                )
                if selected_target_is_exact:
                    exact_selected_target_samples += 1
                decision = person.get("free_space_decision")
                if isinstance(decision, str) and decision:
                    free_space_decision_count[decision] += 1
                if (
                    constrained
                    and isinstance(selected_target, list)
                    and len(selected_target) >= 2
                    and math.dist(
                        positions[path], tuple(map(float, selected_target[:2]))
                    )
                    <= TARGET_CURRENT_TOLERANCE_M
                ):
                    target_current_fallback_count[person_id] += 1

                lateral_weighted_sum["raw"] += (
                    abs(float(person["gazebo_raw_lateral_component_mps"])) * dt
                )
                lateral_weighted_sum["adapter"] += (
                    abs(float(person["adapter_lateral_component_mps"])) * dt
                )
                lateral_weighted_sum["actual"] += (
                    abs(float(person["actual_navigation_lateral_mps"])) * dt
                )

                x, y = positions[path]
                grid_key = f"x[{math.floor(x / GRID_SIZE_M) * GRID_SIZE_M:.0f},{(math.floor(x / GRID_SIZE_M) + 1) * GRID_SIZE_M:.0f})_y[{math.floor(y / GRID_SIZE_M) * GRID_SIZE_M:.0f},{(math.floor(y / GRID_SIZE_M) + 1) * GRID_SIZE_M:.0f})"
                update_spatial_bucket(
                    grid[grid_key], dt, nearest, congestion, constrained, freeze_inclusive
                )
                for hotspot_name, (xmin, xmax, ymin, ymax) in HOTSPOTS.items():
                    if xmin <= x < xmax and ymin <= y < ymax:
                        update_spatial_bucket(
                            hotspots[hotspot_name],
                            dt,
                            nearest,
                            congestion,
                            constrained,
                            freeze_inclusive,
                        )
            lateral_weight_sec += dt * len(people)

    if sample_count == 0 or first_time is None or previous_time is None:
        raise ValueError(f"no steering samples in {trace_path}")

    def aggregate_tracker(name: str) -> dict[str, Any]:
        per_person = {
            person_id: person_trackers[name].as_dict()
            for person_id, person_trackers in sorted(trackers.items())
        }
        return {
            "total_person_seconds": sum(
                value["total_sec"] for value in per_person.values()
            ),
            "episode_count": sum(
                value["episode_count"] for value in per_person.values()
            ),
            "maximum_continuous_sec": max(
                (value["maximum_continuous_sec"] for value in per_person.values()),
                default=0.0,
            ),
            "per_person": per_person,
        }

    congestion = aggregate_tracker("congestion")
    freeze_inclusive = aggregate_tracker("freeze_inclusive")
    freeze_exclusive = aggregate_tracker("freeze_exclusive")
    constraint = aggregate_tracker("constraint")
    emergency = aggregate_tracker("emergency_inhibited")
    total_valid_person_sec = valid_integrated_sec * len(expected_ids)
    constrained_samples = sum(
        bucket["constraint_samples"] for bucket in grid.values()
    )
    constraint["sample_count"] = int(constrained_samples)
    constraint["sample_ratio"] = safe_ratio(constrained_samples, person_sample_count)
    constraint["duration_ratio"] = safe_ratio(
        constraint["total_person_seconds"], total_valid_person_sec
    )
    constraint["target_current_fallback_count"] = sum(
        target_current_fallback_count.values()
    )
    constraint["target_current_fallback_count_by_person"] = dict(
        sorted(target_current_fallback_count.items())
    )
    constraint["target_current_fallback_count_is_exact"] = (
        exact_selected_target_samples == person_sample_count
    )
    constraint["exact_selected_target_sample_count"] = (
        exact_selected_target_samples
    )
    constraint["free_space_decision_count"] = dict(
        sorted(free_space_decision_count.items())
    )
    congestion["duration_ratio"] = safe_ratio(
        congestion["total_person_seconds"], total_valid_person_sec
    )
    freeze_inclusive["duration_ratio"] = safe_ratio(
        freeze_inclusive["total_person_seconds"], total_valid_person_sec
    )
    freeze_exclusive["duration_ratio"] = safe_ratio(
        freeze_exclusive["total_person_seconds"], total_valid_person_sec
    )
    emergency["duration_ratio"] = safe_ratio(
        emergency["total_person_seconds"], total_valid_person_sec
    )

    lateral = {
        f"mean_abs_{name}_lateral_mps": safe_ratio(total, lateral_weight_sec)
        for name, total in lateral_weighted_sum.items()
    }
    lateral["raw_to_adapter_realization_ratio"] = safe_ratio(
        lateral["mean_abs_adapter_lateral_mps"],
        lateral["mean_abs_raw_lateral_mps"],
    )
    lateral["adapter_to_actual_realization_ratio"] = safe_ratio(
        lateral["mean_abs_actual_lateral_mps"],
        lateral["mean_abs_adapter_lateral_mps"],
    )

    top_grid: dict[str, list[dict[str, Any]]] = {}
    for metric in (
        "overlap_lt_0_45_person_sec",
        "personal_space_lt_1_0_person_sec",
        "congestion_person_sec",
        "constraint_person_sec",
        "freeze_person_sec",
    ):
        top_grid[metric] = [
            {"cell": key, metric: value[metric]}
            for key, value in sorted(
                grid.items(), key=lambda item: item[1][metric], reverse=True
            )[:10]
            if value[metric] > 0.0
        ]

    return {
        "schema": TRACE_SCHEMA,
        "sample_frames": sample_count,
        "person_samples": person_sample_count,
        "observed_ids": sorted(observed_ids),
        "trace_start_sim_time": first_time,
        "trace_end_sim_time": previous_time,
        "trace_span_sec": previous_time - first_time,
        "valid_integrated_sec": valid_integrated_sec,
        "valid_integration_coverage": safe_ratio(
            valid_integrated_sec, previous_time - first_time
        ),
        "missing_interval_threshold_sec": MAX_VALID_GAP_SEC,
        "missing_interval_interpretation": (
            "observed inter-frame gap above threshold; diagnostic only, not "
            "proof of missing simulator state and not excluded from integration"
        ),
        "missing_interval_count": missing_interval_count,
        "missing_interval_total_sec": missing_interval_sec,
        "maximum_interframe_gap_sec": maximum_gap_sec,
        "geometry": {
            "global_minimum_human_distance_m": (
                min(frame_minima) if frame_minima else None
            ),
            "per_frame_minimum_p01_m": percentile(frame_minima, 0.01),
            "per_frame_minimum_p05_m": percentile(frame_minima, 0.05),
            "per_frame_minimum_p50_m": percentile(frame_minima, 0.50),
            "average_nearest_neighbor_distance_m": (
                safe_ratio(sum(nearest_values), len(nearest_values))
            ),
            "pair_observations": pair_observations,
            "visual_overlap_pair_observations": visual_pair_observations,
            "visual_overlap_pair_ratio": safe_ratio(
                visual_pair_observations, pair_observations
            ),
            "personal_space_pair_observations": personal_pair_observations,
            "personal_space_pair_ratio": safe_ratio(
                personal_pair_observations, pair_observations
            ),
        },
        "congestion": congestion,
        "freeze": {
            "inclusive": freeze_inclusive,
            "exclusive_non_emergency": freeze_exclusive,
            "emergency_inhibited": emergency,
        },
        "free_space_constraint": constraint,
        "lateral_execution": lateral,
        "hotspots": hotspots,
        "top_grid_cells": top_grid,
        "latest_people_count": len(latest_people),
    }


def route_metrics(
    runtime_result: dict[str, Any],
    config_people: dict[str, dict[str, Any]],
    waypoint_counts: dict[str, int],
) -> dict[str, Any]:
    cursors = runtime_result["pedestrian_social_motion"]["patrol_cursors"]
    by_person: dict[str, Any] = {}
    completed_distance = 0.0
    possible_distance = 0.0
    completed_semantic_legs = 0
    duration = float(runtime_result["timeline_elapsed_sec"])
    for runtime_path, cursor in cursors.items():
        person_id = short_person_id(runtime_path)
        authored = config_people[person_id]
        laps = int(cursor["lap_count"])
        cluster = person_id.rsplit("_", 1)[0]
        semantic_waypoints = waypoint_counts[cluster]
        lap_distance = laps * authored["authored_loop_length_m"]
        completed_distance += lap_distance
        possible_distance += authored["preferred_speed_mps"] * duration
        completed_semantic_legs += laps * semantic_waypoints
        by_person[person_id] = {
            **authored,
            "lap_count": laps,
            "advance_count_dense_diagnostic_only": int(cursor["advance_count"]),
            "runtime_point_count": int(cursor["point_count"]),
            "semantic_waypoints_per_cycle": semantic_waypoints,
            "completed_cycle_distance_m": lap_distance,
        }
    people_count = len(by_person)
    person_minutes = people_count * duration / 60.0
    return {
        "people_count": people_count,
        "people_with_completed_cycle": sum(
            value["lap_count"] >= 1 for value in by_person.values()
        ),
        "route_completion_rate": safe_ratio(
            sum(value["lap_count"] >= 1 for value in by_person.values()),
            people_count,
        ),
        "completed_semantic_cycles": sum(
            value["lap_count"] for value in by_person.values()
        ),
        "completed_semantic_legs": completed_semantic_legs,
        "completed_semantic_legs_per_person_minute": safe_ratio(
            completed_semantic_legs, person_minutes
        ),
        "normalized_completed_cycle_distance": safe_ratio(
            completed_distance, possible_distance
        ),
        "dense_advance_sum_diagnostic_only": sum(
            value["advance_count_dense_diagnostic_only"]
            for value in by_person.values()
        ),
        "by_person": dict(sorted(by_person.items())),
    }


def navigation_metrics(run_dir: Path) -> dict[str, Any]:
    episode_path = run_dir / "evaluation" / "episode_0001" / "episode_summary.json"
    session_path = run_dir / "evaluation" / "session_summary.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    session = json.loads(session_path.read_text(encoding="utf-8"))
    aggregate = session["aggregate_metrics"]
    return {
        "episode_count": session["episode_count"],
        "goal_reached": bool(episode["episode"]["goal_reached"]),
        "timeout": bool(episode["episode"]["timeout"]),
        "strict_success": episode["episode"]["success"],
        "strict_success_coverage": aggregate["success_coverage"],
        "termination_reason": episode["episode"]["termination_reason"],
        "navigation_time_sec": episode["episode"]["navigation_time_sec"],
        "path_length_m": episode["navigation"]["path_length_m"],
        "failure_to_progress": episode["failure_to_progress"],
        "collision_proxy": episode["collision"],
        "virtual_collision_proxy": episode["virtual_collision_proxy"],
        "human_clearance": episode["human_clearance"],
        "static_clearance": episode["static_clearance"],
        "ttc": episode["ttc"],
        "data_quality": episode["data_quality"],
    }


def evaluate_run(
    run_name: str,
    run_dir: Path,
    config_path: Path,
    topology_manifest: Path,
) -> dict[str, Any]:
    runtime_result = load_runtime_result(run_dir / "isaac.log")
    config_people = load_config(config_path)
    waypoint_counts = load_semantic_waypoint_counts(topology_manifest)
    trace = trace_metrics(
        run_dir / "pedestrian_social_steering.jsonl", set(config_people)
    )
    validity = {
        "runtime_status_pass": runtime_result.get("status") == "PASS",
        "duration_reached": runtime_result.get("exit_reason") == "duration_reached",
        "people_15": runtime_result.get("people") == 15,
        "people_moving_15": runtime_result.get("people_moving") == 15,
        "gazebo_social": runtime_result.get("pedestrian_social_mode")
        == "gazebo_social",
        "trace_people_15": trace["latest_people_count"] == 15,
        "trace_schema_v2": trace["schema"] == TRACE_SCHEMA,
        "no_runtime_recovery": runtime_result.get(
            "pedestrian_free_space_recovery_count"
        )
        == 0,
        "no_inside_robot_frames": runtime_result.get(
            "pedestrian_inside_robot_frames"
        )
        == 0,
        "route_cursor_15": len(
            runtime_result["pedestrian_social_motion"]["patrol_cursors"]
        )
        == 15,
    }
    return {
        "schema": "scenario_topology_ab_run_summary/v1",
        "run_name": run_name,
        "run_dir": str(run_dir),
        "config_path": str(config_path),
        "config_sha256": sha256_file(config_path),
        "validity": validity,
        "valid": all(validity.values()),
        "runtime": {
            "timeline_elapsed_sec": runtime_result["timeline_elapsed_sec"],
            "wall_elapsed_sec": runtime_result["wall_elapsed_sec"],
            "rtf": safe_ratio(
                runtime_result["timeline_elapsed_sec"],
                runtime_result["wall_elapsed_sec"],
            ),
            "average_fps": runtime_result["average_fps"],
            "physics_sim_hz": safe_ratio(
                runtime_result["physics_steps_main_loop"],
                runtime_result["timeline_elapsed_sec"],
            ),
            "physics_wall_hz": safe_ratio(
                runtime_result["physics_steps_main_loop"],
                runtime_result["wall_elapsed_sec"],
            ),
            "physics_steps_per_expected_timeline_step": runtime_result[
                "physics_steps_per_expected_timeline_step"
            ],
            "lidar_sim_hz": runtime_result["lidar_measured_pair_rate_hz"],
            "lidar_wall_hz": runtime_result["lidar_measured_pair_wall_rate_hz"],
            "collision_blocked_count": runtime_result["collision_blocked_count"],
            "follow_restart_count": runtime_result["pedestrian_social_motion"][
                "follow_restart_count"
            ],
            "free_space_recovery_count": runtime_result[
                "pedestrian_free_space_recovery_count"
            ],
            "pedestrian_robot_dodge_count": runtime_result[
                "pedestrian_robot_dodge_count"
            ],
            "parameters": runtime_result["pedestrian_social_motion"]["parameters"],
            "producer_source_sha256": runtime_result["producer_source_sha256"],
            "launcher_sha256": runtime_result["launcher_sha256"],
        },
        "trace": trace,
        "route": route_metrics(runtime_result, config_people, waypoint_counts),
        "navigation": navigation_metrics(run_dir),
        "runtime_social_quality": runtime_result["pedestrian_social_quality"],
    }


METRICS = [
    ("geometry.global_minimum_human_distance_m", "higher"),
    ("geometry.per_frame_minimum_p01_m", "higher"),
    ("geometry.per_frame_minimum_p05_m", "higher"),
    ("geometry.per_frame_minimum_p50_m", "higher"),
    ("geometry.average_nearest_neighbor_distance_m", "higher"),
    ("geometry.visual_overlap_pair_ratio", "lower"),
    ("geometry.personal_space_pair_ratio", "lower"),
    ("congestion.total_person_seconds", "lower"),
    ("congestion.maximum_continuous_sec", "lower"),
    ("freeze.inclusive.total_person_seconds", "lower"),
    ("freeze.inclusive.maximum_continuous_sec", "lower"),
    ("freeze.inclusive.episode_count", "lower"),
    ("freeze.emergency_inhibited.total_person_seconds", "lower"),
    ("free_space_constraint.sample_ratio", "lower"),
    ("free_space_constraint.episode_count", "lower"),
    ("free_space_constraint.total_person_seconds", "lower"),
    ("free_space_constraint.target_current_fallback_count", "lower"),
    ("lateral_execution.mean_abs_raw_lateral_mps", "diagnostic"),
    ("lateral_execution.mean_abs_adapter_lateral_mps", "diagnostic"),
    ("lateral_execution.mean_abs_actual_lateral_mps", "diagnostic"),
    ("lateral_execution.raw_to_adapter_realization_ratio", "diagnostic"),
    ("lateral_execution.adapter_to_actual_realization_ratio", "diagnostic"),
    ("route.people_with_completed_cycle", "higher"),
    ("route.completed_semantic_cycles", "higher"),
    ("route.completed_semantic_legs_per_person_minute", "higher"),
    ("route.normalized_completed_cycle_distance", "higher"),
    ("runtime.rtf", "diagnostic"),
    ("runtime.average_fps", "diagnostic"),
    ("trace.sample_frames", "diagnostic"),
    ("trace.missing_interval_count", "lower"),
]


def nested_get(document: dict[str, Any], dotted: str) -> Any:
    value: Any = document
    for part in dotted.split("."):
        value = value[part]
    return value


def compare_runs(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    if not a["valid"] or not b["valid"]:
        raise ValueError("cannot compare invalid runs")
    runtime_parameters_identical = a["runtime"]["parameters"] == b["runtime"][
        "parameters"
    ]
    runtime_hashes_identical = all(
        a["runtime"][key] == b["runtime"][key]
        for key in ("producer_source_sha256", "launcher_sha256")
    )
    rows = []
    for path, direction in METRICS:
        a_value = nested_get(a["trace"], path) if path.startswith(
            ("geometry.", "congestion.", "freeze.", "free_space_constraint.", "lateral_execution.")
        ) else nested_get(a, path)
        b_value = nested_get(b["trace"], path) if path.startswith(
            ("geometry.", "congestion.", "freeze.", "free_space_constraint.", "lateral_execution.")
        ) else nested_get(b, path)
        delta = b_value - a_value
        delta_percent = safe_ratio(delta * 100.0, abs(a_value))
        if direction == "diagnostic" or math.isclose(delta, 0.0, abs_tol=1.0e-12):
            judgement = "neutral"
        elif (direction == "lower" and delta < 0.0) or (
            direction == "higher" and delta > 0.0
        ):
            judgement = "better"
        else:
            judgement = "worse"
        rows.append(
            {
                "metric": path,
                "direction": direction,
                "A": a_value,
                "B": b_value,
                "delta_B_minus_A": delta,
                "delta_percent_of_abs_A": delta_percent,
                "judgement": judgement,
            }
        )

    a_trace = a["trace"]
    b_trace = b["trace"]
    seed7_conditions = {
        "congestion_decreased": b_trace["congestion"]["total_person_seconds"]
        < a_trace["congestion"]["total_person_seconds"],
        "constraint_episodes_decreased": b_trace["free_space_constraint"][
            "episode_count"
        ]
        < a_trace["free_space_constraint"]["episode_count"],
        "freeze_decreased": b_trace["freeze"]["inclusive"][
            "total_person_seconds"
        ]
        < a_trace["freeze"]["inclusive"]["total_person_seconds"],
        "route_completion_noninferior": (
            b["route"]["people_with_completed_cycle"]
            >= a["route"]["people_with_completed_cycle"]
            and b["route"]["normalized_completed_cycle_distance"]
            >= a["route"]["normalized_completed_cycle_distance"]
        ),
        "raw_social_layer_active": (
            a_trace["lateral_execution"]["mean_abs_raw_lateral_mps"] > 0.0
            and b_trace["lateral_execution"]["mean_abs_raw_lateral_mps"] > 0.0
        ),
        "runtime_parameters_identical": runtime_parameters_identical,
        "runtime_hashes_identical": runtime_hashes_identical,
    }
    rtf_relative_difference = abs(b["runtime"]["rtf"] - a["runtime"]["rtf"]) / abs(
        a["runtime"]["rtf"]
    )
    coverage_relative_difference = abs(
        b_trace["valid_integration_coverage"]
        - a_trace["valid_integration_coverage"]
    )
    fairness = {
        "rtf_relative_difference": rtf_relative_difference,
        "trace_coverage_absolute_difference": coverage_relative_difference,
        "rtf_comparable_within_5_percent": rtf_relative_difference <= 0.05,
        "trace_coverage_comparable_within_2_percent": coverage_relative_difference
        <= 0.02,
    }
    hotspot_comparison: dict[str, Any] = {}
    hotspot_metrics = (
        "overlap_lt_0_45_person_sec",
        "personal_space_lt_1_0_person_sec",
        "congestion_person_sec",
        "constraint_person_sec",
        "freeze_person_sec",
    )
    for name in HOTSPOTS:
        a_hotspot = a_trace["hotspots"][name]
        b_hotspot = b_trace["hotspots"][name]
        metrics = {}
        for metric in hotspot_metrics:
            a_rate = safe_ratio(a_hotspot[metric], a_hotspot["person_time_sec"])
            b_rate = safe_ratio(b_hotspot[metric], b_hotspot["person_time_sec"])
            metrics[metric.replace("_person_sec", "_person_time_ratio")] = {
                "A": a_rate,
                "B": b_rate,
                "delta_B_minus_A": (
                    b_rate - a_rate if a_rate is not None and b_rate is not None else None
                ),
            }
        hotspot_comparison[name] = {
            "bounds": HOTSPOTS[name],
            "A_person_time_sec": a_hotspot["person_time_sec"],
            "B_person_time_sec": b_hotspot["person_time_sec"],
            "metrics": metrics,
        }
    seed7_pass = all(seed7_conditions.values()) and fairness[
        "rtf_comparable_within_5_percent"
    ] and fairness["trace_coverage_comparable_within_2_percent"]
    return {
        "schema": "scenario_topology_ab_pair_summary/v1",
        "A_run": a["run_name"],
        "B_run": b["run_name"],
        "runtime_parameters_identical": runtime_parameters_identical,
        "runtime_hashes_identical": runtime_hashes_identical,
        "fairness": fairness,
        "hotspots": hotspot_comparison,
        "metrics": rows,
        "seed7_conditions": seed7_conditions,
        "seed7_pass": seed7_pass,
        "multi_seed_authorized_by_seed7_gate": seed7_pass,
        "scenario_decision": (
            "B_adopt_spread_radius" if seed7_pass else "A_retain_baseline"
        ),
    }


def flatten_scalars(value: Any, prefix: str = "") -> Iterable[tuple[str, Any]]:
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            yield from flatten_scalars(child, child_prefix)
    elif not isinstance(value, (list, tuple)):
        yield prefix, value


def write_run_outputs(
    run_dir: Path, config_path: Path, summary: dict[str, Any]
) -> None:
    json_path = run_dir / "evaluation_summary.json"
    csv_path = run_dir / "evaluation_summary.csv"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "value"])
        for key, value in flatten_scalars(summary):
            writer.writerow([key, value])
    input_copy = run_dir / "input_people_config.yaml"
    shutil.copyfile(config_path, input_copy)
    runtime_result = load_runtime_result(run_dir / "isaac.log")
    warehouse_path = run_dir / "warehouse_metrics.json"
    warehouse_path.write_text(
        json.dumps(runtime_result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    project_root = Path(__file__).resolve().parents[2]
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    manifest = {
        "schema": "scenario_topology_ab_experiment_manifest/v1",
        "run_name": summary["run_name"],
        "git_commit": git_commit,
        "launch_working_directory": str(
            project_root / "isaac_sim" / "scripts" / "ira_people_demo"
        ),
        "launcher_absolute_path": str(
            project_root / "isaac_sim" / "scripts" / "run_custom_people_drlvo_demo.sh"
        ),
        "duration_sec": summary["runtime"]["timeline_elapsed_sec"],
        "config_path": str(config_path.resolve()),
        "config_sha256": summary["config_sha256"],
        "producer_source_sha256": summary["runtime"]["producer_source_sha256"],
        "launcher_sha256": summary["runtime"]["launcher_sha256"],
        "social_parameters": summary["runtime"]["parameters"],
        "validity": summary["validity"],
        "valid": summary["valid"],
    }
    manifest_path = run_dir / "experiment_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    checksum_paths = [
        input_copy,
        manifest_path,
        run_dir / "isaac.log",
        run_dir / "pedestrian_social_steering.jsonl",
        warehouse_path,
        json_path,
        csv_path,
        run_dir / "evaluation" / "session_summary.json",
        run_dir / "evaluation" / "episode_0001" / "episode_summary.json",
        run_dir / "evaluation" / "episode_0001" / "pedestrian_trace.csv",
    ]
    checksum_text = "".join(
        f"{sha256_file(path)}  {path.relative_to(run_dir)}\n"
        for path in checksum_paths
    )
    (run_dir / "SHA256SUMS").write_text(checksum_text, encoding="utf-8")


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def write_pair_outputs(output_dir: Path, pair: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "A_vs_B_seed7_summary.json"
    csv_path = output_dir / "A_vs_B_seed7_summary.csv"
    report_path = output_dir / "A_vs_B_seed7_report.md"
    json_path.write_text(json.dumps(pair, indent=2, sort_keys=True) + "\n")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(pair["metrics"][0]))
        writer.writeheader()
        writer.writerows(pair["metrics"])
    lines = [
        "# Scenario topology A/B seed-7 report",
        "",
        f"- A: `{pair['A_run']}`",
        f"- B: `{pair['B_run']}`",
        f"- Seed-7 continuation gate: **{'PASS' if pair['seed7_pass'] else 'FAIL'}**",
        f"- Scenario decision: **{pair['scenario_decision']}**",
        f"- Runtime parameters identical: `{pair['runtime_parameters_identical']}`",
        f"- Runtime source hashes identical: `{pair['runtime_hashes_identical']}`",
        "- Collision evidence is geometric proxy only; physical-contact truth is unavailable.",
        "",
        "## Metric definitions",
        "",
        "- Congestion: nearest human `<1.0 m`, actual speed `<0.25 * preferred`, intended adapter speed `>=0.2 m/s`, and not emergency-inhibited.",
        "- Inclusive freeze: intended adapter speed `>=0.2 m/s` and actual speed `<0.05 m/s`; emergency-inhibited time remains included and is also reported separately.",
        "- Constraint episode: per-person `false -> true` transition of `free_space_constrained`.",
        "- Hotspot ratios divide condition person-seconds by observed person-seconds inside the stated bounds.",
        "- Inter-frame gaps above `0.5 s` are diagnostics only; all positive monotonic simulation-time intervals are integrated to match runtime accounting.",
        "",
        "## Gate conditions",
        "",
    ]
    for key, value in pair["seed7_conditions"].items():
        lines.append(f"- {key}: `{'PASS' if value else 'FAIL'}`")
    lines.extend(
        [
            "",
            "## Metric table",
            "",
            "| Metric | Direction | A | B | Delta | Delta % | Judgement |",
            "|---|---:|---:|---:|---:|---:|---|",
        ]
    )
    for row in pair["metrics"]:
        lines.append(
            "| {metric} | {direction} | {a} | {b} | {delta} | {percent} | {judgement} |".format(
                metric=row["metric"],
                direction=row["direction"],
                a=format_value(row["A"]),
                b=format_value(row["B"]),
                delta=format_value(row["delta_B_minus_A"]),
                percent=format_value(row["delta_percent_of_abs_A"]),
                judgement=row["judgement"],
            )
        )
    lines.extend(["", "## Spatial hotspots", ""])
    for name, hotspot in pair["hotspots"].items():
        lines.extend(
            [
                f"### {name}",
                "",
                f"- A person-time: `{format_value(hotspot['A_person_time_sec'])} s`",
                f"- B person-time: `{format_value(hotspot['B_person_time_sec'])} s`",
                "",
                "| Condition | A ratio | B ratio | Delta |",
                "|---|---:|---:|---:|",
            ]
        )
        for metric, values in hotspot["metrics"].items():
            lines.append(
                f"| {metric} | {format_value(values['A'])} | "
                f"{format_value(values['B'])} | "
                f"{format_value(values['delta_B_minus_A'])} |"
            )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    checksum_paths = [json_path, csv_path, report_path]
    (output_dir / "SHA256SUMS").write_text(
        "".join(
            f"{sha256_file(path)}  {path.name}\n" for path in checksum_paths
        ),
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--a-run", type=Path, required=True)
    parser.add_argument("--b-run", type=Path, required=True)
    parser.add_argument("--a-config", type=Path, required=True)
    parser.add_argument("--b-config", type=Path, required=True)
    parser.add_argument("--topology-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    a = evaluate_run("A_seed7", args.a_run, args.a_config, args.topology_manifest)
    b = evaluate_run("B_seed7", args.b_run, args.b_config, args.topology_manifest)
    write_run_outputs(args.a_run, args.a_config, a)
    write_run_outputs(args.b_run, args.b_config, b)
    pair = compare_runs(a, b)
    write_pair_outputs(args.output_dir, pair)
    print(json.dumps(pair, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
