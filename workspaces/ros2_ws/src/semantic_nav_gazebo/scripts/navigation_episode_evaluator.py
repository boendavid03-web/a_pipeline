#!/usr/bin/env python3
"""Model-agnostic ROS-time navigation evaluator.

The node is deliberately passive: it never publishes a command and stores the
raw receive-time traces needed to revise metric definitions after an experiment.
It can either preserve the original single-episode output layout or record a
sequence of manually selected goals into independent episode subdirectories.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from navigation_evaluation_msgs.msg import (
    ActuationDecision,
    InferenceMetrics,
    SimulatorActuationState,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from semantic_nav_gazebo.msg import PedestrianStateArray
from std_msgs.msg import Empty

from navigation_evaluation_core import (
    constant_velocity_ttc,
    align_actuation_series,
    derivative_summary,
    distribution_summary,
    failure_to_progress_summary,
    free_space_mask,
    goal_spl,
    load_map,
    map_asset_paths,
    path_length,
    planner_reference_metadata,
    planner_reference_path_length,
    personal_space_integral,
    path_irregularity_summary,
    threshold_exposure,
    time_milestones,
    ttc_statistics,
)

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # Static clearance remains explicitly unavailable.
    distance_transform_edt = None


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def normalized_frame_id(value):
    return str(value or "").strip().lstrip("/")


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def sha256_or_none(value: str):
    path = Path(value).expanduser()
    if not value or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stats(values):
    values = np.asarray([value for value in values if value is not None and math.isfinite(value)], dtype=float)
    if not len(values):
        return {"minimum": None, "mean": None, "maximum": None}
    return {"minimum": float(np.min(values)), "mean": float(np.mean(values)), "maximum": float(np.max(values))}


def percentile_summary(values):
    values = np.asarray([value for value in values if value is not None and math.isfinite(value)], dtype=float)
    if not len(values):
        return {"samples": 0, "mean": None, "p50": None, "p95": None, "max": None}
    return {"samples": int(len(values)), "mean": float(np.mean(values)), "p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)), "max": float(np.max(values))}


def motion_derivative_block(odom_linear, cmd_linear, cmd_angular, unit_suffix):
    """Format derivative summaries while retaining usable sample/audit counts."""
    signals = {
        "odom_linear": odom_linear,
        "cmd_linear": cmd_linear,
        "cmd_angular": cmd_angular,
    }
    result = {}
    for name, summary in signals.items():
        suffix = "radps" if name == "cmd_angular" else "mps"
        result[f"{name}_rms_{suffix}{unit_suffix}"] = summary["rms"]
        result[f"{name}_max_{suffix}{unit_suffix}"] = summary["maximum"]
        result[f"{name}_minimum_{suffix}{unit_suffix}"] = summary["minimum"]
        result[f"{name}_mean_{suffix}{unit_suffix}"] = summary["mean"]
        result[f"{name}_maximum_signed_{suffix}{unit_suffix}"] = summary["maximum_signed"]
        result[f"{name}_mean_abs_{suffix}{unit_suffix}"] = summary["mean_abs"]
        result[f"{name}_p95_abs_{suffix}{unit_suffix}"] = summary["p95_abs"]
        result[f"{name}_max_abs_{suffix}{unit_suffix}"] = summary["max_abs"]
        result[f"{name}_integrated_squared_{suffix}{unit_suffix}"] = summary["integrated_squared"]
    result["derivative_sample_count"] = {
        name: summary["sample_count"] for name, summary in signals.items()
    }
    result["discarded_interval_count"] = {
        name: {
            "duplicate_timestamp": summary["duplicate_timestamp_pairs"],
            "nonpositive_dt": summary["nonpositive_dt_pairs"],
            "abnormal_dt": summary["abnormal_dt_pairs"],
            "nonfinite_value": summary["nonfinite_value_pairs"],
            "nonfinite_timestamp": summary["nonfinite_timestamp_pairs"],
        }
        for name, summary in signals.items()
    }
    return result


def metric_catalog():
    """Exact inventory of the 26 requested metrics."""
    entries = [
        ("Success", "available", "episode.success", "Strict success requires evaluable static and human proxies."),
        ("Timeout", "available", "episode.timeout", "Evaluator timeout termination."),
        ("Path Length", "available", "navigation.path_length_m", "Integrated accepted odometry trajectory."),
        ("Run Time", "available", "episode.navigation_time_sec", "ROS simulation-time duration."),
        ("SPL", "provisional", "navigation.goal_spl.value", "Uses the configured A* reference, not an asserted strict shortest path."),
        ("Static Obstacle Collision", "proxy", "collision.static_proxy", "Nullable static geometry-overlap proxy, not a physical contact event."),
        ("Velocity Metrics", "available", ["speed.mean_mps", "speed.max_mps", "speed.stopped_time_ratio", "angular_velocity.mean_absolute_radps", "angular_velocity.maximum_absolute_radps"], "Odometry velocity summaries."),
        ("Speed Efficiency", "unavailable", None, "Nominal speed limit/efficiency source is not recorded."),
        ("Acceleration Metrics", "available", ["motion_quality.acceleration.odom_linear_rms_mps2", "motion_quality.acceleration.odom_linear_max_mps2"], "Finite differences of accepted odometry velocity."),
        ("Jerk Metrics", "available", ["motion_quality.jerk.odom_linear_rms_mps3", "motion_quality.jerk.odom_linear_max_mps3"], "Second finite differences of accepted odometry velocity."),
        ("Obstacle Distance", "available", ["static_clearance.minimum_m", "static_clearance.mean_m"], "Occupancy-map clearance when map distance is evaluable."),
        ("Path Irregularity", "proxy", "navigation.path_irregularity.turning_rad_per_m", "Geometric necessary/unnecessary course-change proxy."),
        ("Topological Complexity", "unavailable", None, "No fixed braid/topological convention is recorded."),
        ("Path Efficiency", "provisional", "navigation.planner_reference_path_efficiency", "A* reference efficiency is provisional."),
        ("Failure To Progress", "available", ["failure_to_progress.failed", "failure_to_progress.event_count", "failure_to_progress.evaluated_duration_sec", "failure_to_progress.stalled_duration_sec", "failure_to_progress.stalled_duration_ratio"], "Five-second goal-distance progress events and left-held stalled-state duration."),
        ("Human Collision", "proxy", "collision.human_proxy", "Nullable synchronized human geometry-overlap proxy, not a physical contact event."),
        ("Social Distance", "available", ["human_clearance.minimum_center_distance_m", "human_clearance.minimum_body_clearance_m"], "Synchronized ground-truth human clearance."),
        ("Min Time To Collide", "available", "ttc.minimum_finite_ttc_sec", "Synchronized constant-velocity TTC proxy."),
        ("Crowd Density", "available", "crowd_density.statistics.mean", "Synchronized people per square metre within 2 m."),
        ("Virtual Collision", "proxy", ["virtual_collision_proxy.entry_count", "virtual_collision_proxy.exposure_time_sec", "virtual_collision_proxy.exposure_ratio", "virtual_collision_proxy.max_penetration"], "Synchronized social-space intrusion proxy, not physical contact."),
        ("Personal Space", "available", ["personal_space.violation_time_sec", "personal_space.violation_time_ratio"], "Synchronized time below configured personal-space threshold."),
        ("Legibility", "unavailable", None, "No observer-intent model is recorded."),
        ("Predictability", "unavailable", None, "No observer-prediction model is recorded."),
        ("Projected Path", "unavailable", None, "No projected-path geometry trace is recorded."),
        ("Following Rate", "not_applicable", None, "Point-goal episodes do not define a followed person."),
        ("SPS", "not_applicable", None, "Point-goal episodes do not define this task-specific score."),
    ]
    catalog = {}
    for name, status, path, reason in entries:
        paths = list(path) if isinstance(path, list) else ([path] if path else [])
        catalog[name] = {"status": status, "output_paths": paths,
                         "output_path": path, "reason": reason}
    return catalog


def resolve_output_path(payload, path):
    value = payload
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def aggregate_scalar(values):
    known = [value for value in values if value is not None and (isinstance(value, bool) or (isinstance(value, (int, float)) and math.isfinite(float(value))))]
    result = {"available": len(known), "missing": len(values) - len(known),
              "coverage": len(known) / len(values) if values else None}
    if known and all(isinstance(value, bool) for value in known):
        result["binary"] = {"true_count": sum(known), "false_count": len(known) - sum(known),
                             "true_rate": sum(known) / len(known)}
    elif known:
        array = np.asarray([float(value) for value in known], dtype=float)
        result["numeric"] = {"mean": float(np.mean(array)), "std": float(np.std(array)),
                              "min": float(np.min(array)), "p05": float(np.percentile(array, 5)),
                              "p50": float(np.percentile(array, 50)), "p95": float(np.percentile(array, 95)),
                              "max": float(np.max(array))}
    else:
        result["binary"] = None
        result["numeric"] = None
    return result


class NavigationEpisodeEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("navigation_episode_evaluator")
        self._declare_parameters()
        output_text = str(self.get_parameter("evaluation_output_dir").value).strip()
        if not output_text:
            raise ValueError("evaluation_output_dir must be non-empty")
        self.output_root = Path(output_text).expanduser().resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.output_dir = self.output_root
        self.multi_episode = bool(
            self.get_parameter("evaluation_multi_episode").value
        )
        self.episode_index = 0
        self.completed_episodes = []
        self.trace_files, self.writers = {}, {}
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.timeout_sec = float(self.get_parameter("evaluation_timeout_sec").value)
        if self.goal_tolerance <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError("goal_tolerance and evaluation_timeout_sec must be positive")
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.pedestrian_radius = float(self.get_parameter("pedestrian_radius").value)
        self.personal_space_radius = float(self.get_parameter("personal_space_radius").value)
        self.stopped_threshold = float(self.get_parameter("stopped_speed_threshold").value)
        self.derivative_max_dt = float(self.get_parameter("derivative_max_dt_sec").value)
        self.ttc_threshold = float(self.get_parameter("ttc_threshold_sec").value)
        self.nonzero_cmd_epsilon = float(self.get_parameter("nonzero_cmd_epsilon").value)
        if min(self.robot_radius, self.pedestrian_radius, self.personal_space_radius, self.stopped_threshold, self.nonzero_cmd_epsilon) < 0.0:
            raise ValueError("evaluation radii and stopped_speed_threshold must be non-negative")
        if self.derivative_max_dt <= 0.0 or self.ttc_threshold < 0.0:
            raise ValueError("derivative_max_dt_sec must be positive and ttc_threshold_sec non-negative")
        self.map_loaded = False
        self.free = self.clearance_field = None
        self.map_resolution = self.map_origin_x = self.map_origin_y = None
        self._load_map()
        self.active = self.finished = False
        self.goal_start_time = self.end_time = None
        self.first_odom_time = self.first_policy_action_time = None
        self.first_nonzero_cmd_time = None
        self.accepted_goal = self.actual_start = None
        self.trajectory, self.commands, self.human_samples, self.human_body_samples, self.ttc_samples = [], [], [], [], []
        self.actuation_decisions, self.simulator_actuation = [], []
        self.crowd_density_samples = []
        self.inference_latencies, self.inference_timestamps = [], []
        self.latest_inference_metadata = None
        self.current_pedestrians = []
        self.current_pedestrian_timestamp = None
        self.current_pedestrian_frame_id = ""
        self.last_odom_frame_id = ""
        self.odom_proxy_validity = []
        self.sync_counters = {}
        self.source_received = {"odom": False, "goal": False, "pedestrian_ground_truth": False, "inference_metrics": False, "global_path": False, "actuation_decision": False, "simulator_actuation": False}
        self.global_path = None
        self._completed_summaries = []
        self._completed_latency_samples = []
        accepted_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointStamped, str(self.get_parameter("goal_accepted_topic").value), self.goal_callback, accepted_qos)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 100)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.cmd_callback, 100)
        self.create_subscription(PathMsg, str(self.get_parameter("global_path_topic").value), self.path_callback, 10)
        self.create_subscription(PedestrianStateArray, str(self.get_parameter("pedestrian_ground_truth_topic").value), self.pedestrian_callback, 100)
        self.create_subscription(InferenceMetrics, str(self.get_parameter("inference_metrics_topic").value), self.inference_callback, 100)
        self.create_subscription(ActuationDecision, str(self.get_parameter("actuation_decision_topic").value), self.actuation_decision_callback, 100)
        self.create_subscription(SimulatorActuationState, str(self.get_parameter("simulator_actuation_topic").value), self.simulator_actuation_callback, 100)
        self.create_subscription(
            Empty,
            str(self.get_parameter("episode_reset_topic").value),
            self.episode_reset_callback,
            10,
        )
        self.create_timer(0.2, self.timeout_callback)
        mode = "multi-episode" if self.multi_episode else "single-episode"
        self.get_logger().info(
            f"Episode evaluator ({mode}) will write to {self.output_root}"
        )

    def _declare_parameters(self):
        for name, default in (
            ("evaluation_output_dir", ""), ("evaluation_timeout_sec", 360.0),
            ("evaluation_multi_episode", False),
            ("odom_topic", "/odom"), ("cmd_vel_topic", "/cmd_vel"),
            ("goal_accepted_topic", "/data_collection/goal_accepted"),
            ("global_path_topic", "/semantic_cnn/global_path"),
            ("pedestrian_ground_truth_topic", "/pedestrian_ground_truth"),
            ("inference_metrics_topic", "/navigation_evaluation/inference_metrics"),
            ("actuation_decision_topic", "/drl_vo/actuation_decision"),
            ("simulator_actuation_topic", "/isaac/actuation_state"),
            ("episode_reset_topic", "/drl_vo/episode_reset"),
            ("map_yaml", ""), ("semantic_label_path", ""), ("inflate_radius", 0.5), ("snap_radius", 0.8),
            ("goal_tolerance", 0.35), ("robot_radius", 0.34), ("pedestrian_radius", 0.125),
            ("personal_space_radius", 0.8), ("stopped_speed_threshold", 0.02),
            ("derivative_max_dt_sec", 2.0), ("ttc_threshold_sec", 2.0),
            ("reset_jump_distance_m", 2.0),
            ("alignment_rate_hz", 15.0),
            ("alignment_freshness_sec", 0.20),
            ("alignment_max_delay_sec", 0.50),
            ("maximum_actual_linear_speed_mps", 5.0),
            ("maximum_actual_angular_speed_radps", 10.0),
            ("nonzero_cmd_epsilon", 1e-6),
            ("experiment_scene_id", ""), ("scene_file", ""), ("robot_x", float("nan")),
            ("robot_y", float("nan")), ("robot_yaw", float("nan")), ("goal_x", float("nan")),
            ("goal_y", float("nan")), ("pedestrian_seed", -1), ("pedestrian_count", -1),
            ("method_name", ""), ("producer_id", ""), ("policy_mode", ""),
            ("checkpoint", ""), ("device", ""), ("pedestrian_source", ""),
            ("oracle_pedestrian_velocity", False),
        ):
            self.declare_parameter(name, default)

    def _open_traces(self):
        self.trace_files, self.writers = {}, {}
        fields = {
            "trajectory.csv": ("simulation_time_sec", "x", "y", "yaw", "odom_linear_x_mps", "odom_linear_y_mps", "odom_angular_z_radps", "goal_distance_m", "static_clearance_m", "nearest_human_center_distance_m", "nearest_human_body_clearance_m", "ttc_sec"),
            "commands.csv": ("simulation_time_sec", "linear_x_mps", "linear_y_mps", "angular_z_radps"),
            "pedestrian_trace.csv": ("simulation_time_sec", "pedestrian_id", "x", "y", "yaw", "vx_mps", "vy_mps"),
            "inference_trace.csv": ("header_time_sec", "input_time_sec", "producer_id", "sequence_id", "success", "preprocessing_ms", "policy_ms", "postprocessing_ms", "total_ms", "action_encoding", "action", "device", "model_parameters", "cuda_memory_allocated_bytes", "cuda_peak_memory_bytes"),
            "actuation_decisions.csv": ("simulation_time_sec", "decision_sequence_id", "inference_sequence_id", "has_raw_action", "raw_linear_x_mps", "raw_angular_z_radps", "final_linear_x_mps", "final_angular_z_radps", "gated", "gate_reasons", "front_min_range_m"),
            "simulator_actuation.csv": ("simulation_time_sec", "telemetry_sequence_id", "command_received", "command_sequence_id", "bridge_receive_time_sec", "received_linear_x_mps", "received_angular_z_radps", "applied_linear_x_mps", "applied_angular_z_radps", "actual_linear_x_mps", "actual_angular_z_radps", "actual_velocity_source", "command_age_sec", "watchdog_active", "collision_protection_active", "control_reasons"),
            "actuation_alignment.csv": ("simulation_time_sec", "raw_model_linear_x_mps", "raw_model_angular_z_radps", "final_command_linear_x_mps", "final_command_angular_z_radps", "received_command_linear_x_mps", "received_command_angular_z_radps", "applied_command_linear_x_mps", "applied_command_angular_z_radps", "actual_linear_x_mps", "actual_angular_z_radps", "policy_gated", "simulator_gated", "gated", "decision_age_sec", "state_age_sec"),
        }
        for name, fieldnames in fields.items():
            stream = (self.output_dir / name).open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            self.trace_files[name], self.writers[name] = stream, writer

    def _reset_episode_state(self):
        self.goal_start_time = self.end_time = None
        self.first_odom_time = self.first_policy_action_time = None
        self.first_nonzero_cmd_time = None
        self.accepted_goal = self.actual_start = None
        self.trajectory, self.commands, self.actuation_decisions, self.simulator_actuation = [], [], [], []
        self.last_accepted_odom = None
        self.human_samples, self.human_body_samples, self.ttc_samples = [], [], []
        self.crowd_density_samples = []
        self.current_pedestrians = []
        self.current_pedestrian_timestamp = None
        self.current_pedestrian_frame_id = ""
        self.last_odom_frame_id = ""
        self.odom_proxy_validity = []
        self.sync_counters = {
            "accepted_odom": 0,
            "synchronized_pedestrian_snapshots": 0,
            "stale_pedestrian_snapshots": 0,
            "future_pedestrian_snapshots": 0,
            "frame_mismatch_pedestrian_snapshots": 0,
            "missing_pedestrian_snapshots": 0,
            "rejected_pedestrian_snapshots": 0,
            "rejected_nonfinite_odom": 0,
            "rejected_nonfinite_cmd": 0,
            "rejected_nonfinite_pedestrian_snapshots": 0,
        }
        self.inference_latencies, self.inference_timestamps = [], []
        self.latest_inference_metadata = None
        self.source_received = {
            "odom": False,
            "goal": False,
            "pedestrian_ground_truth": False,
            "inference_metrics": False,
            "global_path": False,
            "actuation_decision": False,
            "simulator_actuation": False,
        }
        self.global_path = None

    def _start_episode(self, goal, start_time):
        self.episode_index += 1
        if self.multi_episode:
            self.output_dir = self.output_root / f"episode_{self.episode_index:04d}"
            if self.output_dir.exists():
                raise FileExistsError(
                    f"Refusing to overwrite existing episode directory: {self.output_dir}"
                )
            self.output_dir.mkdir(parents=False)
        else:
            self.output_dir = self.output_root

        self._reset_episode_state()
        self.accepted_goal = goal
        self.goal_start_time = start_time
        self.source_received["goal"] = True
        self.finished = False
        self.active = True
        self._open_traces()
        self.get_logger().info(
            f"Episode {self.episode_index} started for accepted goal "
            f"({goal[0]:.3f}, {goal[1]:.3f}); output={self.output_dir}"
        )

    def _write_session_summary(self):
        if not self.multi_episode:
            return
        summaries = self._completed_summaries
        def mean(values):
            values = [float(v) for v in values if v is not None and math.isfinite(float(v))]
            return float(np.mean(values)) if values else None
        def proxy_rate(key):
            known = [s["collision"].get(key) for s in summaries
                     if isinstance(s.get("collision", {}).get(key), bool)]
            return (sum(known) / len(known) if known else None), len(known)
        human_rate, human_coverage = proxy_rate("human_proxy")
        static_rate, static_coverage = proxy_rate("static_proxy")
        successful = [s for s in summaries if s.get("episode", {}).get("success") is True]
        spl = [s.get("navigation", {}).get("goal_spl", {}).get("value") for s in summaries]
        ftp = [s.get("failure_to_progress", {}).get("failed") for s in summaries
               if isinstance(s.get("failure_to_progress", {}).get("failed"), bool)]
        strict_values = [s.get("episode", {}).get("success") for s in summaries]
        strict_known = [value for value in strict_values if isinstance(value, bool)]
        goal_values = [bool(s.get("episode", {}).get("goal_reached")) for s in summaries]
        aggregate = {
            "episode_count": len(summaries),
            "success_count": sum(strict_known),
            "success_rate": (sum(strict_known) / len(strict_known)) if strict_known else None,
            "success_coverage": len(strict_known) / len(summaries) if summaries else None,
            "goal_reached_count": sum(goal_values),
            "goal_reached_rate": (sum(goal_values) / len(goal_values)) if goal_values else None,
            "timeout_count": sum(bool(s.get("episode", {}).get("timeout")) for s in summaries),
            "timeout_rate": (sum(bool(s.get("episode", {}).get("timeout")) for s in summaries) / len(summaries)) if summaries else None,
            "human_proxy_collision_rate": human_rate,
            "human_proxy_collision_coverage": human_coverage / len(summaries) if summaries else None,
            "static_proxy_collision_rate": static_rate,
            "static_proxy_collision_coverage": static_coverage / len(summaries) if summaries else None,
            "mean_provisional_spl": mean(spl),
            "successful_path_length_mean_m": mean([s.get("navigation", {}).get("path_length_m") for s in successful]),
            "successful_navigation_time_mean_sec": mean([s.get("episode", {}).get("navigation_time_sec") for s in successful]),
            "failure_to_progress_episode_rate": mean([bool(v) for v in ftp]) if ftp else None,
            "aggregate_inference_p95_ms": float(np.percentile(self._completed_latency_samples, 95)) if self._completed_latency_samples else None,
        }
        catalog = metric_catalog()
        metrics = {}
        for name, entry in catalog.items():
            if entry["status"] not in {"available", "proxy", "provisional"}:
                continue
            paths = entry.get("output_paths", [])
            outputs = {}
            for path in paths:
                values = [resolve_output_path(summary, path) for summary in summaries]
                output_summary = aggregate_scalar(values)
                groups = {"success": [], "failure": [], "unknown": []}
                for summary, value in zip(summaries, values):
                    status = summary.get("episode", {}).get("success")
                    groups["success" if status is True else "failure" if status is False else "unknown"].append(value)
                output_summary["by_success"] = {group: aggregate_scalar(values) for group, values in groups.items()}
                outputs[path] = output_summary
            metrics[name] = {"status": entry["status"], "outputs": outputs}
        aggregate["goal_reached"] = aggregate_scalar(goal_values)
        aggregate["metrics"] = metrics
        pooled_latency = distribution_summary(self._completed_latency_samples)
        pooled_latency["p95"] = pooled_latency.get("p95")
        aggregate["inference_latency_ms"] = pooled_latency
        tracking_block_names = (
            "raw_model_to_final_command",
            "final_command_to_actual_velocity",
            "angular_raw_model_to_final_command",
            "angular_final_command_to_actual_velocity",
            "safety_gated",
            "safety_ungated",
            "raw_model_to_final_safety_gated",
            "raw_model_to_final_safety_ungated",
            "angular_safety_gated",
            "angular_safety_ungated",
            "stable_straight_ungated",
        )
        scalar_metrics = (
            "bias", "mae", "rmse", "p50_absolute_error",
            "p95_absolute_error", "maximum_absolute_error", "correlation",
            "actual_to_command_ratio", "zero_command_hold_error",
        )
        tracking_blocks = {}
        for block_name in tracking_block_names:
            items = [
                summary.get("velocity_tracking", {}).get(block_name, {})
                for summary in summaries
            ]
            valid = [item for item in items if item.get("valid")]
            tracking_blocks[block_name] = {
                "episode_coverage": len(valid) / len(summaries) if summaries else None,
                "valid_episode_count": len(valid),
                "metrics": {
                    metric: distribution_summary(
                        [item.get(metric) for item in valid]
                    )
                    for metric in scalar_metrics
                },
                "sample_count_total": sum(
                    int(item.get("sample_count", 0)) for item in valid
                ),
            }
        primary = tracking_blocks["final_command_to_actual_velocity"]
        delay_values = [
            summary.get("velocity_tracking", {}).get("best_causal_delay_sec")
            for summary in summaries
        ]
        alignment_diagnostic_totals = {}
        for summary in summaries:
            diagnostics = summary.get("data_quality", {}).get(
                "actuation_alignment_diagnostics", {}
            )
            for key, value in diagnostics.items():
                if isinstance(value, int):
                    alignment_diagnostic_totals[key] = (
                        alignment_diagnostic_totals.get(key, 0) + value
                    )
        aggregate["velocity_tracking"] = {
            "episode_coverage": primary["episode_coverage"],
            "valid_episode_count": primary["valid_episode_count"],
            "mean_mae": primary["metrics"]["mae"]["mean"],
            "mean_rmse": primary["metrics"]["rmse"]["mean"],
            "blocks": tracking_blocks,
            "best_causal_delay_sec": distribution_summary(delay_values),
            "alignment_diagnostic_totals": alignment_diagnostic_totals,
        }
        payload = {
            "schema": {
                "name": "navigation_evaluation_session",
                "version": 2,
            },
            "episode_count": len(self.completed_episodes),
            "episodes": self.completed_episodes,
            "aggregate_metrics": aggregate,
        }
        summary_path = self.output_root / "session_summary.json"
        temporary_path = self.output_root / ".session_summary.json.tmp"
        temporary_path.write_text(
            json.dumps(payload, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary_path, summary_path)

    def _write(self, name, row):
        self.writers[name].writerow(row)
        self.trace_files[name].flush()

    def _load_map(self):
        map_yaml = str(self.get_parameter("map_yaml").value).strip()
        if not map_yaml:
            return
        try:
            occupancy, resolution, origin_x, origin_y = load_map(Path(map_yaml))
            self.free = free_space_mask(occupancy, resolution, float(self.get_parameter("inflate_radius").value))
            self.map_resolution, self.map_origin_x, self.map_origin_y = resolution, origin_x, origin_y
            if distance_transform_edt is not None:
                self.clearance_field = distance_transform_edt(occupancy >= 250) * resolution
            self.map_loaded = True
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(f"Evaluation map unavailable: {exc}")

    def goal_callback(self, message):
        x, y = float(message.point.x), float(message.point.y)
        if not math.isfinite(x) or not math.isfinite(y):
            self.get_logger().error("Ignoring non-finite accepted goal")
            return
        start_time = stamp_sec(message.header.stamp)
        if self.active:
            if not self.multi_episode:
                self.get_logger().warning(
                    "Ignoring additional goal_accepted in single-episode evaluator"
                )
                return
            self.finish("superseded_by_new_goal", start_time)
        elif self.finished and not self.multi_episode:
            self.get_logger().warning(
                "Ignoring additional goal_accepted in single-episode evaluator"
            )
            return
        self._start_episode((x, y), start_time)

    def path_callback(self, message):
        self.source_received["global_path"] = True
        self.global_path = message

    def _count(self, name, amount=1):
        self.sync_counters[name] = self.sync_counters.get(name, 0) + amount

    def pedestrian_callback(self, message):
        timestamp = stamp_sec(message.header.stamp)
        frame_id = normalized_frame_id(message.header.frame_id)
        if not math.isfinite(timestamp):
            self._count("rejected_pedestrian_snapshots")
            self._count("rejected_nonfinite_pedestrian_snapshots")
            return
        people = []
        for person in message.pedestrians:
            values = (float(person.pose.position.x), float(person.pose.position.y),
                      float(person.pose.orientation.x), float(person.pose.orientation.y),
                      float(person.pose.orientation.z), float(person.pose.orientation.w),
                      float(person.velocity.linear.x), float(person.velocity.linear.y))
            if not all(math.isfinite(value) for value in values):
                self._count("rejected_pedestrian_snapshots")
                self._count("rejected_nonfinite_pedestrian_snapshots")
                return
            record = (str(person.id), values[0], values[1], yaw_from_quaternion(person.pose.orientation), values[6], values[7])
            people.append(record)
        if self.active and not self.finished:
            for record in people:
                self._write("pedestrian_trace.csv", dict(zip(self.writers["pedestrian_trace.csv"].fieldnames, (timestamp,) + record)))
        self.source_received["pedestrian_ground_truth"] = True
        self.current_pedestrians = people
        self.current_pedestrian_timestamp = timestamp
        self.current_pedestrian_frame_id = frame_id

    def inference_callback(self, message):
        self.source_received["inference_metrics"] = True
        if not self.active or self.finished:
            return
        total = finite_or_none(message.total_ms)
        timestamp = stamp_sec(message.header.stamp)
        self.inference_timestamps.append(timestamp)
        self.latest_inference_metadata = {
            "producer_id": message.producer_id or None,
            "device": message.device or None,
            "model_parameters": int(message.model_parameters) or None,
            "cuda_peak_memory_bytes": int(message.cuda_peak_memory_bytes) or None,
        }
        if message.success and total is not None:
            self.inference_latencies.append(total)
        if (
            self.first_policy_action_time is None
            and message.success
            and len(message.action) > 0
            and all(math.isfinite(float(value)) for value in message.action)
        ):
            # This is deliberately telemetry-only.  A policy-action milestone
            # must never be reconstructed from cmd_vel, which can be altered
            # by safety gates or another controller.
            self.first_policy_action_time = timestamp
        self._write("inference_trace.csv", {
            "header_time_sec": timestamp, "input_time_sec": stamp_sec(message.input_stamp),
            "producer_id": message.producer_id, "sequence_id": message.sequence_id, "success": message.success,
            "preprocessing_ms": message.preprocessing_ms, "policy_ms": message.policy_ms,
            "postprocessing_ms": message.postprocessing_ms, "total_ms": message.total_ms,
            "action_encoding": message.action_encoding, "action": json.dumps(list(message.action)),
            "device": message.device, "model_parameters": message.model_parameters,
            "cuda_memory_allocated_bytes": message.cuda_memory_allocated_bytes,
            "cuda_peak_memory_bytes": message.cuda_peak_memory_bytes,
        })

    def actuation_decision_callback(self, message):
        self.source_received["actuation_decision"] = True
        if not self.active or self.finished:
            return
        timestamp = stamp_sec(message.header.stamp)
        required_values = (
            timestamp,
            float(message.final_command.linear.x),
            float(message.final_command.angular.z),
        )
        raw_linear = (
            float(message.raw_physical_action.linear.x)
            if message.has_raw_action else None
        )
        raw_angular = (
            float(message.raw_physical_action.angular.z)
            if message.has_raw_action else None
        )
        if not all(math.isfinite(value) for value in required_values) or (
            message.has_raw_action
            and not all(math.isfinite(value) for value in (raw_linear, raw_angular))
        ):
            self._count("rejected_nonfinite_actuation_decision")
            return
        row = {
            "simulation_time_sec": timestamp,
            "decision_sequence_id": int(message.decision_sequence_id),
            "inference_sequence_id": int(message.inference_sequence_id),
            "has_raw_action": bool(message.has_raw_action),
            "raw_linear_x_mps": raw_linear,
            "raw_angular_z_radps": raw_angular,
            "final_linear_x_mps": required_values[1],
            "final_angular_z_radps": required_values[2],
            "gated": bool(message.gated),
            "gate_reasons": json.dumps(list(message.gate_reasons)),
            "front_min_range_m": (float(message.front_min_range_m)
                                  if message.has_front_min_range else None),
        }
        self._write("actuation_decisions.csv", row)
        self.actuation_decisions.append({"time": timestamp, "raw": raw_linear,
                                         "final": required_values[1],
                                         "raw_angular": row["raw_angular_z_radps"],
                                         "final_angular": row["final_angular_z_radps"],
                                         "gated": row["gated"],
                                         "sequence": row["decision_sequence_id"]})

    def simulator_actuation_callback(self, message):
        self.source_received["simulator_actuation"] = True
        if not self.active or self.finished:
            return
        timestamp = stamp_sec(message.header.stamp)
        values = (
            timestamp,
            float(message.received_command.linear.x),
            float(message.applied_command.linear.x),
            float(message.actual_velocity.linear.x),
            float(message.received_command.angular.z),
            float(message.applied_command.angular.z),
            float(message.actual_velocity.angular.z),
        )
        source = str(message.actual_velocity_source)
        if not all(math.isfinite(value) for value in values) or source not in {
            "physx_rigid_body_api", "fixed_tick_pose_difference"
        }:
            self._count("rejected_invalid_actual_velocity")
            return
        if (
            math.hypot(
                float(message.actual_velocity.linear.x),
                float(message.actual_velocity.linear.y),
            ) > float(self.get_parameter("maximum_actual_linear_speed_mps").value)
            or abs(float(message.actual_velocity.angular.z))
            > float(self.get_parameter("maximum_actual_angular_speed_radps").value)
        ):
            self._count("rejected_implausible_actual_velocity")
            return
        row = {
            "simulation_time_sec": timestamp, "telemetry_sequence_id": int(message.telemetry_sequence_id),
            "command_received": bool(message.command_received),
            "command_sequence_id": int(message.command_sequence_id),
            "bridge_receive_time_sec": (stamp_sec(message.bridge_receive_stamp)
                                        if message.command_received else None),
            "received_linear_x_mps": values[1], "applied_linear_x_mps": values[2],
            "received_angular_z_radps": values[4],
            "applied_angular_z_radps": values[5],
            "actual_linear_x_mps": values[3],
            "actual_angular_z_radps": values[6],
            "actual_velocity_source": source, "command_age_sec": finite_or_none(message.command_age_sec),
            "watchdog_active": bool(message.watchdog_active),
            "collision_protection_active": bool(message.collision_protection_active),
            "control_reasons": json.dumps(list(message.control_reasons)),
        }
        self._write("simulator_actuation.csv", row)
        self.simulator_actuation.append({"time": timestamp, "received": values[1],
                                         "applied": values[2], "actual": values[3],
                                         "received_angular": row["received_angular_z_radps"],
                                         "applied_angular": row["applied_angular_z_radps"],
                                         "actual_angular": row["actual_angular_z_radps"],
                                         "command_received": row["command_received"],
                                         "actual_velocity_source": row["actual_velocity_source"],
                                         "command_age_sec": row["command_age_sec"],
                                         "watchdog_active": row["watchdog_active"],
                                         "collision_protection_active": row["collision_protection_active"],
                                         "control_reasons": list(message.control_reasons),
                                         "sequence": row["telemetry_sequence_id"],
                                         "command_sequence": row["command_sequence_id"]})

    def _static_clearance(self, x, y):
        if self.clearance_field is None:
            return None
        row = self.clearance_field.shape[0] - 1 - int(math.floor((y - self.map_origin_y) / self.map_resolution))
        col = int(math.floor((x - self.map_origin_x) / self.map_resolution))
        if not (0 <= row < self.clearance_field.shape[0] and 0 <= col < self.clearance_field.shape[1]):
            return None
        return finite_or_none(self.clearance_field[row, col] - self.robot_radius)

    def odom_callback(self, message):
        if not self.active or self.finished:
            return
        timestamp = stamp_sec(message.header.stamp)
        x, y = float(message.pose.pose.position.x), float(message.pose.pose.position.y)
        orientation = message.pose.pose.orientation
        linear_x, linear_y = float(message.twist.twist.linear.x), float(message.twist.twist.linear.y)
        angular_z = float(message.twist.twist.angular.z)
        raw_values = (timestamp, x, y, float(orientation.x), float(orientation.y),
                      float(orientation.z), float(orientation.w), linear_x, linear_y, angular_z)
        if not all(math.isfinite(value) for value in raw_values):
            self._count("rejected_nonfinite_odom")
            return
        if self.last_accepted_odom is not None:
            previous_time, previous_x, previous_y = self.last_accepted_odom
            if timestamp <= previous_time:
                self._count("rejected_nonpositive_odom_timestamp")
                return
            if math.hypot(x - previous_x, y - previous_y) > float(
                self.get_parameter("reset_jump_distance_m").value
            ):
                # A reset is an episode boundary, never a high-speed path
                # segment.  Preserve the existing trace and report why it is
                # incomplete instead of integrating the teleport.
                self._count("reset_jump_detected")
                self.finish("sim_reset_detected", timestamp)
                return
        if (
            math.hypot(linear_x, linear_y)
            > float(self.get_parameter("maximum_actual_linear_speed_mps").value)
            or abs(angular_z)
            > float(self.get_parameter("maximum_actual_angular_speed_radps").value)
        ):
            self._count("rejected_implausible_odom_twist")
            return
        self.last_accepted_odom = (timestamp, x, y)
        self.source_received["odom"] = True
        self._count("accepted_odom")
        self.last_odom_frame_id = normalized_frame_id(message.header.frame_id)
        yaw = yaw_from_quaternion(orientation)
        if self.actual_start is None:
            self.actual_start = (x, y, yaw)
            self.first_odom_time = timestamp
        goal_distance = math.hypot(x - self.accepted_goal[0], y - self.accepted_goal[1])
        synchronized = False
        sync_people = []
        if self.current_pedestrian_timestamp is None:
            self._count("missing_pedestrian_snapshots")
        else:
            delta = timestamp - self.current_pedestrian_timestamp
            ped_frame = self.current_pedestrian_frame_id
            odom_frame = self.last_odom_frame_id
            if not ped_frame or not odom_frame or ped_frame != odom_frame:
                self._count("frame_mismatch_pedestrian_snapshots")
            elif delta < 0.0:
                self._count("future_pedestrian_snapshots")
            elif delta > 0.5:
                self._count("stale_pedestrian_snapshots")
            else:
                synchronized = True
                sync_people = list(self.current_pedestrians)
                self._count("synchronized_pedestrian_snapshots")
        nearest_center = nearest_body = minimum_ttc = None
        if synchronized and sync_people:
            distances = [math.hypot(px - x, py - y) for _, px, py, _, _, _ in sync_people]
            nearest_center = min(distances)
            nearest_body = nearest_center - self.robot_radius - self.pedestrian_radius
            robot_world_velocity = (math.cos(yaw) * linear_x - math.sin(yaw) * linear_y, math.sin(yaw) * linear_x + math.cos(yaw) * linear_y)
            candidates = [constant_velocity_ttc((x, y), robot_world_velocity, (px, py), (vx, vy), self.robot_radius + self.pedestrian_radius) for _, px, py, _, vx, vy in sync_people]
            future = [candidate for candidate in candidates if candidate is not None]
            minimum_ttc = min(future) if future else None
        static_clearance = self._static_clearance(x, y)
        row = {"simulation_time_sec": timestamp, "x": x, "y": y, "yaw": yaw, "odom_linear_x_mps": linear_x, "odom_linear_y_mps": linear_y, "odom_angular_z_radps": angular_z, "goal_distance_m": goal_distance, "static_clearance_m": static_clearance, "nearest_human_center_distance_m": nearest_center, "nearest_human_body_clearance_m": nearest_body, "ttc_sec": minimum_ttc}
        self._write("trajectory.csv", row)
        self.trajectory.append(row)
        # A received empty ground-truth crowd is known-safe for social-space
        # exposure; keep trajectory nearest-human fields null while using +inf
        # only in the internal exposure trace.
        internal_human_distance = nearest_center
        if internal_human_distance is None and synchronized:
            internal_human_distance = float("inf")
        self.human_samples.append((timestamp, internal_human_distance if synchronized else None))
        self.human_body_samples.append((timestamp, nearest_body if synchronized else None))
        self.ttc_samples.append((timestamp, minimum_ttc if synchronized else None))
        if synchronized:
            # Fixed, documented 2 m sensing radius; this is a local density
            # sample, not a claim about the full scene population.
            crowd_count = sum(1 for _, px, py, _, _, _ in sync_people
                              if math.hypot(px - x, py - y) <= 2.0)
            self.crowd_density_samples.append((timestamp, crowd_count / (math.pi * 2.0 ** 2)))
        self.odom_proxy_validity.append((static_clearance is not None,
                                         synchronized))
        if goal_distance <= self.goal_tolerance:
            self.finish("goal_tolerance_reached", timestamp)

    def cmd_callback(self, message):
        if not self.active or self.finished:
            return
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        row = {"simulation_time_sec": timestamp, "linear_x_mps": float(message.linear.x), "linear_y_mps": float(message.linear.y), "angular_z_radps": float(message.angular.z)}
        if not all(math.isfinite(float(value)) for value in row.values()):
            self._count("rejected_nonfinite_cmd")
            return
        self._write("commands.csv", row)
        self.commands.append(row)
        if (
            self.first_nonzero_cmd_time is None
            and max(abs(row["linear_x_mps"]), abs(row["linear_y_mps"]), abs(row["angular_z_radps"])) > self.nonzero_cmd_epsilon
        ):
            self.first_nonzero_cmd_time = timestamp

    def timeout_callback(self):
        if self.active and not self.finished:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now >= self.goal_start_time + self.timeout_sec:
                self.finish("sim_timeout", now)

    def episode_reset_callback(self, _message):
        if not self.active or self.finished:
            return
        self._count("episode_reset_events")
        now = self.get_clock().now().nanoseconds * 1e-9
        self.finish("episode_reset", now)

    @staticmethod
    def _rate(timestamps):
        timestamps = [timestamp for timestamp in timestamps if timestamp is not None and math.isfinite(timestamp)]
        if len(timestamps) < 2:
            return None
        duration = max(timestamps) - min(timestamps)
        return (len(timestamps) - 1) / duration if duration > 0.0 else None

    def _planner_reference_path(self):
        if self.actual_start is None or self.accepted_goal is None or self.free is None:
            return None
        return planner_reference_path_length(self.actual_start[:2], self.accepted_goal, self.free, self.map_resolution, self.map_origin_x, self.map_origin_y, float(self.get_parameter("snap_radius").value))

    def _summary(self, reason):
        end = self.end_time
        navigation_time = end - self.goal_start_time if end is not None and self.goal_start_time is not None and end >= self.goal_start_time else None
        path = path_length([(row["x"], row["y"]) for row in self.trajectory]) if self.trajectory else None
        planner_reference_length = self._planner_reference_path()
        reached = reason == "goal_tolerance_reached"
        timeout = reason == "sim_timeout"
        start_goal_euclidean = (
            math.hypot(self.accepted_goal[0] - self.actual_start[0], self.accepted_goal[1] - self.actual_start[1])
            if self.accepted_goal is not None and self.actual_start is not None else None
        )
        linear_speed_samples = [(row["simulation_time_sec"], math.hypot(row["odom_linear_x_mps"], row["odom_linear_y_mps"])) for row in self.trajectory]
        speed_values = [value for _, value in linear_speed_samples]
        angular_values = [row["odom_angular_z_radps"] for row in self.trajectory]
        stopped_seconds = total_seconds = weighted_speed = 0.0
        for (t0, speed), (t1, _) in zip(linear_speed_samples, linear_speed_samples[1:]):
            if 0.0 < t1 - t0 <= self.derivative_max_dt:
                total_seconds += t1 - t0
                weighted_speed += speed * (t1 - t0)
                if speed < self.stopped_threshold:
                    stopped_seconds += t1 - t0
        odom_linear_samples = [(row["simulation_time_sec"], row["odom_linear_x_mps"]) for row in self.trajectory]
        cmd_linear_samples = [(row["simulation_time_sec"], row["linear_x_mps"]) for row in self.commands]
        cmd_angular_samples = [(row["simulation_time_sec"], row["angular_z_radps"]) for row in self.commands]
        acceleration = motion_derivative_block(
            derivative_summary(odom_linear_samples, order=1, max_dt=self.derivative_max_dt),
            derivative_summary(cmd_linear_samples, order=1, max_dt=self.derivative_max_dt),
            derivative_summary(cmd_angular_samples, order=1, max_dt=self.derivative_max_dt),
            "2",
        )
        jerk = motion_derivative_block(
            derivative_summary(odom_linear_samples, order=2, max_dt=self.derivative_max_dt),
            derivative_summary(cmd_linear_samples, order=2, max_dt=self.derivative_max_dt),
            derivative_summary(cmd_angular_samples, order=2, max_dt=self.derivative_max_dt),
            "3",
        )
        static = stats([row["static_clearance_m"] for row in self.trajectory])
        human_center = stats([row["nearest_human_center_distance_m"] for row in self.trajectory])
        human_body = stats([row["nearest_human_body_clearance_m"] for row in self.trajectory])
        violation_seconds, violation_ratio = personal_space_integral(self.human_samples, self.personal_space_radius)
        personal_spaces = {}
        for threshold in (0.5, 0.8, 1.2):
            seconds, ratio = personal_space_integral(self.human_samples, threshold)
            personal_spaces[f"{threshold:.1f}m"] = {"threshold_m": threshold,
                                                       "violation_time_sec": seconds if self.human_samples else None,
                                                       "violation_time_ratio": ratio}
        ftp = failure_to_progress_summary(
            [(row["simulation_time_sec"], row["goal_distance_m"]) for row in self.trajectory],
            window=5.0, progress=0.2,
        )
        crowd_values = [density for _, density in self.crowd_density_samples]
        static_values = [row["static_clearance_m"] for row in self.trajectory
                         if row["static_clearance_m"] is not None and math.isfinite(row["static_clearance_m"])]
        human_body_values = [row["nearest_human_body_clearance_m"] for row in self.trajectory
                             if row["nearest_human_body_clearance_m"] is not None and math.isfinite(row["nearest_human_body_clearance_m"])]
        static_known = bool(self.odom_proxy_validity) and all(static for static, _human in self.odom_proxy_validity)
        human_known = bool(self.odom_proxy_validity) and all(human for _static, human in self.odom_proxy_validity)
        static_proxy = any(value <= 0.0 for value in static_values) if static_known else None
        human_proxy = any(value <= 0.0 for value in human_body_values) if human_known else None
        static_exposure = threshold_exposure(
            [(row["simulation_time_sec"], row["static_clearance_m"]) for row in self.trajectory], 0.0
        )
        human_exposure = threshold_exposure(
            self.human_body_samples, 0.0
        )
        virtual_exposure = threshold_exposure(self.human_samples, self.personal_space_radius)
        virtual_proxy = virtual_exposure["entry_count"] > 0 if virtual_exposure["entry_count"] is not None else None
        path_irregularity = path_irregularity_summary([(row["x"], row["y"]) for row in self.trajectory])
        if not reached:
            strict_success = False
        elif static_known and human_known:
            strict_success = static_proxy is False and human_proxy is False
        else:
            strict_success = None
        params = self.get_parameter
        requested_start = [finite_or_none(params("robot_x").value), finite_or_none(params("robot_y").value), finite_or_none(params("robot_yaw").value)]
        requested_goal = [finite_or_none(params("goal_x").value), finite_or_none(params("goal_y").value)]
        checkpoint = str(params("checkpoint").value)
        producer_id = str(params("producer_id").value) or None
        telemetry = self.latest_inference_metadata or {}
        map_yaml = str(params("map_yaml").value).strip()
        semantic_label = str(params("semantic_label_path").value).strip()
        try:
            map_paths = map_asset_paths(map_yaml, semantic_label)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Map provenance is incomplete: {exc}")
            map_paths = {
                "map_yaml_path": str(Path(map_yaml).expanduser().resolve()) if map_yaml else None,
                "occupancy_map_path": None,
                "semantic_label_path": str(Path(semantic_label).expanduser().resolve()) if semantic_label else None,
            }
        planner_reference = planner_reference_metadata(
            self.map_resolution,
            float(params("inflate_radius").value),
            float(params("snap_radius").value),
        )
        reach_time = end if reached else None
        quality = {"odom_received": self.source_received["odom"], "goal_received": self.source_received["goal"], "map_loaded": self.map_loaded, "pedestrian_ground_truth_received": self.source_received["pedestrian_ground_truth"], "inference_metrics_received": self.source_received["inference_metrics"], "missing_sources": [name for name, received in self.source_received.items() if not received], **self.sync_counters}
        quality["pedestrian_sync_coverage"] = (self.sync_counters.get("synchronized_pedestrian_snapshots", 0) / self.sync_counters.get("accepted_odom", 0)
                                                if self.sync_counters.get("accepted_odom", 0) else None)
        alignment = align_actuation_series(
            self.actuation_decisions,
            self.simulator_actuation,
            rate_hz=float(params("alignment_rate_hz").value),
            freshness_sec=float(params("alignment_freshness_sec").value),
            max_delay_sec=float(params("alignment_max_delay_sec").value),
        )
        for row in alignment["rows"]:
            self._write("actuation_alignment.csv", row)
        quality["actuation_alignment_coverage"] = alignment["coverage"]
        quality["actuation_alignment_diagnostics"] = alignment["diagnostics"]
        advisory = alignment["stable_straight_ungated"]
        advisory_criteria = {
            "absolute_bias_max_mps": 0.05,
            "mae_max_mps": 0.08,
            "p95_absolute_error_max_mps": 0.15,
            "correlation_minimum": 0.90,
        }
        advisory_checks = None
        if advisory.get("valid") and advisory.get("correlation") is not None:
            advisory_checks = {
                "absolute_bias": abs(advisory["bias"]) <= 0.05,
                "mae": advisory["mae"] <= 0.08,
                "p95_absolute_error": advisory["p95_absolute_error"] <= 0.15,
                "correlation": advisory["correlation"] >= 0.90,
            }
        return {
            "schema": {"name": "navigation_episode_evaluation", "version": 4},
            "episode_sequence": {
                "index": self.episode_index,
                "multi_episode": self.multi_episode,
            },
            "experiment": {"scene_id": str(params("experiment_scene_id").value) or None, "scene_file": str(params("scene_file").value) or None, "requested_start": requested_start, "actual_start": list(self.actual_start) if self.actual_start else None, "requested_goal": requested_goal, "accepted_goal": list(self.accepted_goal) if self.accepted_goal else None, "pedestrian_seed": int(params("pedestrian_seed").value) if int(params("pedestrian_seed").value) >= 0 else None, "pedestrian_count": int(params("pedestrian_count").value) if int(params("pedestrian_count").value) >= 0 else None, "simulation_time_start": self.goal_start_time, "simulation_time_end": end},
            "map_provenance": {"map_yaml_path": map_paths["map_yaml_path"], "map_yaml_sha256": sha256_or_none(map_paths["map_yaml_path"] or ""), "occupancy_map_path": map_paths["occupancy_map_path"], "occupancy_map_sha256": sha256_or_none(map_paths["occupancy_map_path"] or ""), "semantic_label_path": map_paths["semantic_label_path"], "semantic_label_sha256": sha256_or_none(map_paths["semantic_label_path"] or "")},
            "method": {"name": str(params("method_name").value) or None, "producer_id": telemetry.get("producer_id") or producer_id, "policy_mode": str(params("policy_mode").value) or None, "checkpoint": checkpoint or None, "checkpoint_sha256": sha256_or_none(checkpoint), "device": telemetry.get("device") or str(params("device").value) or None, "pedestrian_source": str(params("pedestrian_source").value) or None, "oracle_pedestrian_velocity": bool(params("oracle_pedestrian_velocity").value), "model_parameters": telemetry.get("model_parameters"), "cuda_peak_memory_bytes": telemetry.get("cuda_peak_memory_bytes")},
            "episode": {"goal_reached": reached, "success": strict_success, "timeout": timeout,
                        "strict_success_proxy": strict_success, "termination_reason": reason,
                        "navigation_time_sec": navigation_time,
                        "samples": {"odom": len(self.trajectory), "cmd_vel": len(self.commands), "inference": len(self.inference_latencies), "actuation_decisions": len(self.actuation_decisions), "simulator_actuation": len(self.simulator_actuation)}},
            "time_metrics": time_milestones(self.goal_start_time, reach_time, self.first_odom_time, self.first_policy_action_time, self.first_nonzero_cmd_time),
            "navigation": {"path_length_m": path, "euclidean_start_goal_distance_m": start_goal_euclidean,
                           "planner_reference_path_length_m": planner_reference_length, "planner_reference": planner_reference,
                           "goal_spl": {"value": goal_spl(strict_success, planner_reference_length, path), "status": "provisional", "reason": "A* reference is not asserted to be a strict shortest path; strict success may be unknown"},
                           "planner_reference_path_efficiency": planner_reference_length / path if planner_reference_length is not None and path is not None and path >= 1.0e-3 else None,
                           "minimum_goal_distance_m": min((row["goal_distance_m"] for row in self.trajectory), default=None),
                           "path_irregularity": path_irregularity},
            "speed": {"mean_mps": weighted_speed / total_seconds if total_seconds > 0.0 else None, "max_mps": float(np.max(speed_values)) if speed_values else None,
                      "statistics": distribution_summary(speed_values), "stopped_time_ratio": stopped_seconds / total_seconds if total_seconds > 0.0 else None},
            "angular_velocity": {"statistics_radps": distribution_summary(angular_values),
                                  "mean_absolute_radps": float(np.mean(np.abs(angular_values))) if angular_values else None,
                                  "maximum_absolute_radps": float(np.max(np.abs(angular_values))) if angular_values else None},
            "motion_quality": {"acceleration": acceleration, "jerk": jerk},
            "static_clearance": {"minimum_m": static["minimum"], "mean_m": static["mean"],
                                 "statistics": distribution_summary([row["static_clearance_m"] for row in self.trajectory]),
                                 "threshold_exposure": {"threshold_m": 0.0, "meaning": "static occupancy geometry overlap proxy", **static_exposure}},
            "human_clearance": {"minimum_center_distance_m": human_center["minimum"], "minimum_body_clearance_m": human_body["minimum"],
                                 "mean_nearest_center_distance_m": human_center["mean"],
                                 "center_statistics": distribution_summary([row["nearest_human_center_distance_m"] for row in self.trajectory]),
                                 "body_statistics": distribution_summary([row["nearest_human_body_clearance_m"] for row in self.trajectory]),
                                 "threshold_exposure": {"threshold_m": 0.0, "meaning": "human body-geometry overlap proxy", **human_exposure}},
            "ttc": ttc_statistics(self.ttc_samples, navigation_time, threshold_sec=self.ttc_threshold, max_dt=self.derivative_max_dt),
            "personal_space": {"threshold_m": self.personal_space_radius, "violation_time_sec": violation_seconds if self.human_samples else None, "violation_time_ratio": violation_ratio, "thresholds": personal_spaces},
            "failure_to_progress": ftp,
            "crowd_density": {"radius_m": 2.0, "samples": len(crowd_values), "statistics": distribution_summary(crowd_values), "source": "pedestrian_ground_truth" if self.source_received["pedestrian_ground_truth"] else None},
            "virtual_collision_proxy": {"threshold_m": self.personal_space_radius,
                                         "meaning": "nearest human center inside personal-space radius; not a physical contact event",
                                         **virtual_exposure},
            "frequency": {"cmd_vel_hz": self._rate([row["simulation_time_sec"] for row in self.commands]), "policy_inference_hz": self._rate(self.inference_timestamps)},
            "inference_latency_ms": percentile_summary(self.inference_latencies),
            "velocity_tracking": {
                "source": "simulator_actuation.actual_velocity; /odom twist is actual velocity when this source is present",
                "actual_velocity_sources": sorted({
                    row["actual_velocity_source"]
                    for row in self.simulator_actuation
                    if row.get("actual_velocity_source")
                }),
                "watchdog_sample_count": sum(
                    bool(row.get("watchdog_active"))
                    for row in self.simulator_actuation
                ),
                "collision_protection_sample_count": sum(
                    bool(row.get("collision_protection_active"))
                    for row in self.simulator_actuation
                ),
                "stale_command_sample_count": sum(
                    row.get("command_age_sec") is not None
                    and row["command_age_sec"]
                    > float(params("alignment_freshness_sec").value)
                    for row in self.simulator_actuation
                ),
                "resample_rate_hz": float(params("alignment_rate_hz").value),
                "freshness_sec": float(params("alignment_freshness_sec").value),
                "maximum_causal_delay_sec": float(params("alignment_max_delay_sec").value),
                "raw_model_to_final_command": alignment["raw_to_final"],
                "final_command_to_actual_velocity": alignment["final_to_actual"],
                "angular_raw_model_to_final_command": alignment["raw_to_final_angular"],
                "angular_final_command_to_actual_velocity": alignment["final_to_actual_angular"],
                "safety_gated": alignment["gated"], "safety_ungated": alignment["ungated"],
                "raw_model_to_final_safety_gated": alignment["raw_to_final_gated"],
                "raw_model_to_final_safety_ungated": alignment["raw_to_final_ungated"],
                "angular_safety_gated": alignment["gated_angular"],
                "angular_safety_ungated": alignment["ungated_angular"],
                "stable_straight_ungated": advisory,
                "stable_straight_ungated_advisory": {
                    "statistics": advisory,
                    "criteria": advisory_criteria,
                    "checks": advisory_checks,
                    "passed": (all(advisory_checks.values())
                               if advisory_checks is not None else None),
                    "status": ("evaluated" if advisory_checks is not None
                               else "unavailable"),
                    "reason": (None if advisory_checks is not None else
                               advisory.get("correlation_reason")
                               or advisory.get("reason")
                               or "insufficient_qualified_samples"),
                },
                "best_causal_delay_sec": alignment["best_causal_delay_sec"],
                "delay_reason": alignment["delay_reason"], "coverage": alignment["coverage"],
            },
            "metric_validity": {"velocity_tracking": {
                "status": "available" if alignment["final_to_actual"].get("valid") else "unavailable",
                "reason": alignment["final_to_actual"].get("reason"),
            }},
            "collision": {"occurred": None, "human": None, "static": None, "human_proxy": human_proxy, "static_proxy": static_proxy, "virtual_proxy": virtual_proxy,
                          "status": "proxy", "reason": "Physical contact truth is unavailable; human/static values are geometry proxies, not contact events.",
                          "sensor_configured": False, "publisher_discovered": False, "messages_received": False},
            "metric_catalog": metric_catalog(),
            "data_quality": quality,
        }

    def finish(self, reason, end_time=None):
        if self.finished or not self.active:
            return
        self.finished, self.active = True, False
        self.end_time = end_time if end_time is not None else self.get_clock().now().nanoseconds * 1e-9
        summary = self._summary(reason)
        summary_path = self.output_dir / "episode_summary.json"
        temporary_path = self.output_dir / ".episode_summary.json.tmp"
        temporary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        os.replace(temporary_path, summary_path)
        for stream in self.trace_files.values():
            stream.close()
        if self.multi_episode:
            self._completed_summaries.append(summary)
            self._completed_latency_samples.extend(self.inference_latencies)
            self.completed_episodes.append(
                {
                    "index": self.episode_index,
                    "directory": self.output_dir.name,
                    "accepted_goal": list(self.accepted_goal),
                    "simulation_time_start": self.goal_start_time,
                    "simulation_time_end": self.end_time,
                    "termination_reason": reason,
                    "goal_reached": reason == "goal_tolerance_reached",
                }
            )
            self._write_session_summary()
        self.get_logger().info(f"Episode evaluation finished: {reason}; summary={summary_path}")

    def destroy_node(self):
        if self.active and not self.finished:
            self.finish("interrupted")
        for stream in self.trace_files.values():
            if not stream.closed:
                stream.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = NavigationEpisodeEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
