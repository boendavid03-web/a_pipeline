#!/usr/bin/env python3
"""Select reachable goals and delimit unattended teacher-recording episodes."""

from __future__ import annotations

import json
import math
import os
import random
from collections import Counter, deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from semantic_nav_gazebo.msg import PedestrianStateArray
from std_msgs.msg import Empty, String

from navigation_evaluation_core import (
    astar,
    free_space_mask,
    grid_to_world,
    load_map,
    snap_to_free,
    world_to_grid,
)


EVENT_SCHEMA = "semantic_nav_episode_event/v1"
STATUS_SCHEMA = "semantic_nav_auto_capture_status/v1"
RELOCATION_TARGET = (2.0, 2.0, 0.0)
RELOCATION_YAW_TOLERANCE_RAD = 0.35
SUCCESS_REASON = "goal_reached_and_stopped"
ACTIVE_EPISODE_STATES = frozenset(
    ("waiting_accept", "waiting_goal_sync", "armed", "recording")
)


def quality_quota_met(
    success_count,
    successful_duration_sec,
    minimum_successful_episodes,
    minimum_successful_duration_sec,
):
    """Return whether enough successful supervision has been accumulated."""

    return (
        int(success_count) >= int(minimum_successful_episodes)
        and float(successful_duration_sec)
        >= float(minimum_successful_duration_sec)
    )


def progress_window_is_stuck(
    samples,
    displacement_threshold_m,
    goal_progress_threshold_m,
    yaw_progress_threshold_rad,
    minimum_command_ratio,
):
    """Classify a full progress window without mislabeling real rotation."""

    history = list(samples)
    if len(history) < 2:
        return False
    first, last = history[0], history[-1]
    displacement = math.hypot(last[1] - first[1], last[2] - first[2])
    improvement = first[4] - last[4]
    yaw_progress = max(
        abs(
            math.atan2(
                math.sin(item[3] - first[3]),
                math.cos(item[3] - first[3]),
            )
        )
        for item in history
    )
    moving_ratio = sum(item[5] for item in history) / len(history)
    return (
        displacement < float(displacement_threshold_m)
        and improvement < float(goal_progress_threshold_m)
        and yaw_progress < float(yaw_progress_threshold_rad)
        and moving_ratio >= float(minimum_command_ratio)
    )


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def path_length_m(cells, resolution):
    return float(
        sum(
            math.hypot(a[0] - b[0], a[1] - b[1])
            for a, b in zip(cells, cells[1:])
        )
        * resolution
    )


def weighted_bucket(rng, weights):
    names = ("short", "medium", "long")
    positive = [max(0.0, float(weights[name])) for name in names]
    if sum(positive) <= 0.0:
        raise ValueError("at least one distance-bucket weight must be positive")
    return rng.choices(names, weights=positive, k=1)[0]


class AutoGoalRosbagScheduler(Node):
    def __init__(self):
        super().__init__("auto_goal_rosbag_scheduler")
        self._declare_parameters()
        self._load_parameters()

        occupancy, self.resolution, self.origin_x, self.origin_y = load_map(
            Path(self.map_yaml).expanduser().resolve()
        )
        self.height, self.width = occupancy.shape
        self.route_free = free_space_mask(
            occupancy, self.resolution, self.route_inflation_radius
        )
        self.goal_free = free_space_mask(
            occupancy, self.resolution, self.goal_inflation_radius
        )
        self.collision_free = free_space_mask(
            occupancy, self.resolution, self.robot_radius
        )
        self.goal_cells = np.argwhere(self.goal_free)
        if not len(self.goal_cells):
            raise RuntimeError("the 0.5 m-safe goal mask contains no free cells")

        accepted_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 20)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.reset_pub = self.create_publisher(
            Empty, self.episode_reset_topic, 10
        )
        self.reset_client = (
            self.create_client(SetEntityPose, self.robot_reset_service)
            if self.relocation_backend == "gazebo_set_entity_pose"
            else None
        )
        self.isaac_reset_pub = (
            self.create_publisher(PoseStamped, self.isaac_reset_pose_topic, 10)
            if self.relocation_backend == "isaac_pose_topic"
            else None
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(
            TwistStamped, self.cmd_topic, self._on_cmd, 20
        )
        self.create_subscription(
            PointStamped,
            self.accepted_goal_topic,
            self._on_goal_accepted,
            accepted_qos,
        )
        self.create_subscription(
            PointStamped,
            self.final_goal_topic,
            self._on_final_goal,
            10,
        )
        self.create_subscription(
            String,
            self.control_event_topic,
            self._on_control_event,
            10,
        )
        self.create_subscription(
            PedestrianStateArray,
            self.pedestrian_topic,
            self._on_pedestrians,
            10,
        )
        self.create_timer(0.05, self._on_timer)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            0.10, self._on_relocation_watchdog, clock=self.steady_clock
        )

        self.state = "waiting_odom"
        self.pose = None
        self.odom_velocity = (0.0, 0.0)
        self.cmd_velocity = (0.0, 0.0)
        self.pedestrians = []
        self.pedestrian_stamp_ns = None
        self.session_start_ns = None
        self.capture_start_ns = None
        self.state_start_ns = None
        self.cooldown_delay_sec = self.next_goal_delay_sec
        self.recovery_stop_since_ns = None
        self.last_reset_publish_ns = None
        self.arrival_since_ns = None
        self.static_collision_since_ns = None
        self.human_collision_since_ns = None
        self.pending_goal = None
        self.goal = None
        self.goal_path_length = None
        self.goal_bucket = None
        self.goal_history = []
        self.consecutive_episode_failures = 0
        self.relocation_count = 0
        self.relocation_attempt_count = 0
        self.relocation_failure_count = 0
        self.relocation_state = None
        self.relocation_target = None
        self.relocation_origin = None
        self.relocation_future = None
        self.relocation_response_odom_sequence = None
        self.relocation_deadline_ns = None
        self.relocation_stopped_since_ns = None
        self.odom_sequence = 0
        self.episode_id = 0
        self.success_count = 0
        self.failure_count = 0
        self.successful_duration_sec = 0.0
        self.discarded_duration_sec = 0.0
        self.failure_reasons = Counter()
        self.setup_recovery_count = 0
        self.setup_recovery_reasons = Counter()
        self.capture_deadline_reached = False
        self.quality_wait_logged = False
        self.done = False
        self.exit_code = 0
        self.progress = deque()
        self.rng = random.Random(self.seed)
        self.get_logger().info(
            "AUTO_CAPTURE_READY: waiting for odometry; goals use %.2f m "
            "inflation and A* reachability" % self.goal_inflation_radius
        )

    def _declare_parameters(self):
        declarations = {
            "map_yaml": "",
            "status_path": "",
            "goal_topic": "/goal_pose",
            "accepted_goal_topic": "/data_collection/goal_accepted",
            "final_goal_topic": "/semantic_cnn/final_goal",
            "event_topic": "/data_collection/episode_event",
            "status_topic": "/data_collection/auto_capture_status",
            "episode_reset_topic": "/drl_vo/episode_reset",
            "robot_reset_service": "/world/default/set_pose",
            "relocation_backend": "gazebo_set_entity_pose",
            "isaac_reset_pose_topic": "/isaac/reset_pose",
            "robot_entity_name": "mecanum730_xms5_v7_teacher_dual_scan",
            "control_event_topic": "/drl_vo/control_event",
            "odom_topic": "/odom",
            "cmd_vel_stamped_topic": "/cmd_vel_stamped",
            "pedestrian_topic": "/pedestrian_ground_truth",
            "frame_id": "map",
            "capture_enabled": True,
            "seed": 7001,
            "capture_duration_sec": 1800.0,
            "initial_goal_delay_sec": 8.0,
            "goal_inflation_radius": 0.5,
            "route_inflation_radius": 0.4,
            "robot_radius": 0.34,
            "pedestrian_radius": 0.125,
            "enable_static_map_collision_proxy": False,
            "short_min_m": 3.0,
            "short_max_m": 7.0,
            "medium_min_m": 7.0,
            "medium_max_m": 14.0,
            "long_min_m": 14.0,
            "long_max_m": 26.0,
            "short_weight": 0.15,
            "medium_weight": 0.45,
            "long_weight": 0.40,
            "candidate_attempts": 160,
            "repeat_goal_separation_m": 1.0,
            "goal_tolerance": 0.35,
            "arrival_dwell_sec": 0.5,
            "next_goal_delay_sec": 1.0,
            "goal_accept_timeout_sec": 12.0,
            "episode_timeout_sec": 240.0,
            "stuck_window_sec": 20.0,
            "stuck_displacement_m": 0.20,
            "stuck_goal_progress_m": 0.15,
            "stuck_yaw_progress_rad": 0.15,
            "stuck_min_command_ratio": 0.5,
            "continue_after_episode_failure": False,
            "minimum_successful_episodes": 1,
            "minimum_successful_duration_sec": 0.0,
            "failure_next_goal_delay_sec": 0.25,
            "recovery_stop_dwell_sec": 0.5,
            "relocation_after_failures": 3,
            "relocation_service_timeout_sec": 10.0,
            "relocation_odom_timeout_sec": 5.0,
            "relocation_odom_tolerance_m": 0.50,
            "reset_repeat_period_sec": 0.2,
            "goal_sync_timeout_sec": 5.0,
            "motion_linear_threshold": 0.02,
            "motion_angular_threshold": 0.05,
            "stop_linear_threshold": 0.02,
            "stop_angular_threshold": 0.05,
            "collision_grace_sec": 1.0,
            "static_collision_stall_sec": 4.0,
            "human_collision_confirmation_sec": 0.20,
            "human_collision_penetration_m": 0.0,
            "pedestrian_truth_timeout_sec": 0.5,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self.map_yaml = str(value("map_yaml"))
        self.status_path = str(value("status_path"))
        if not self.map_yaml or not self.status_path:
            raise ValueError("map_yaml and status_path must be non-empty")
        self.goal_topic = str(value("goal_topic"))
        self.accepted_goal_topic = str(value("accepted_goal_topic"))
        self.final_goal_topic = str(value("final_goal_topic"))
        self.event_topic = str(value("event_topic"))
        self.status_topic = str(value("status_topic"))
        self.episode_reset_topic = str(value("episode_reset_topic"))
        self.robot_reset_service = str(value("robot_reset_service"))
        self.robot_entity_name = str(value("robot_entity_name"))
        self.relocation_backend = str(value("relocation_backend"))
        self.isaac_reset_pose_topic = str(value("isaac_reset_pose_topic"))
        if self.relocation_backend not in {
            "gazebo_set_entity_pose", "isaac_pose_topic"
        }:
            raise ValueError(
                "relocation_backend must be gazebo_set_entity_pose or isaac_pose_topic"
            )
        if not self.robot_entity_name or (
            self.relocation_backend == "gazebo_set_entity_pose"
            and not self.robot_reset_service
        ) or (
            self.relocation_backend == "isaac_pose_topic"
            and not self.isaac_reset_pose_topic
        ):
            raise ValueError("robot_reset_service and robot_entity_name must be non-empty")
        self.control_event_topic = str(value("control_event_topic"))
        self.odom_topic = str(value("odom_topic"))
        self.cmd_topic = str(value("cmd_vel_stamped_topic"))
        self.pedestrian_topic = str(value("pedestrian_topic"))
        self.frame_id = str(value("frame_id"))
        self.seed = int(value("seed"))
        float_names = (
            "capture_duration_sec", "initial_goal_delay_sec", "goal_inflation_radius",
            "route_inflation_radius", "robot_radius", "pedestrian_radius",
            "short_min_m", "short_max_m", "medium_min_m", "medium_max_m",
            "long_min_m", "long_max_m", "short_weight", "medium_weight",
            "long_weight", "repeat_goal_separation_m", "goal_tolerance",
            "arrival_dwell_sec", "next_goal_delay_sec",
            "goal_accept_timeout_sec", "episode_timeout_sec",
            "stuck_window_sec", "stuck_displacement_m",
            "stuck_goal_progress_m", "stuck_yaw_progress_rad",
            "stuck_min_command_ratio",
            "minimum_successful_duration_sec", "failure_next_goal_delay_sec",
            "recovery_stop_dwell_sec", "reset_repeat_period_sec",
            "relocation_service_timeout_sec", "relocation_odom_timeout_sec",
            "relocation_odom_tolerance_m",
            "goal_sync_timeout_sec",
            "motion_linear_threshold", "motion_angular_threshold",
            "stop_linear_threshold", "stop_angular_threshold",
            "collision_grace_sec", "static_collision_stall_sec",
            "human_collision_confirmation_sec",
            "human_collision_penetration_m",
            "pedestrian_truth_timeout_sec",
        )
        for name in float_names:
            setattr(self, name, float(value(name)))
        self.candidate_attempts = int(value("candidate_attempts"))
        self.relocation_after_failures = int(value("relocation_after_failures"))
        self.continue_after_episode_failure = bool(
            value("continue_after_episode_failure")
        )
        self.minimum_successful_episodes = int(
            value("minimum_successful_episodes")
        )
        self.enable_static_map_collision_proxy = bool(
            value("enable_static_map_collision_proxy")
        )
        if min(self.capture_duration_sec, self.goal_inflation_radius,
               self.route_inflation_radius, self.goal_tolerance,
               self.episode_timeout_sec, self.stuck_window_sec,
               self.stuck_displacement_m, self.stuck_goal_progress_m,
               self.stuck_yaw_progress_rad) <= 0.0:
            raise ValueError("duration, inflation, tolerance and timeout must be positive")
        if not 0.0 <= self.stuck_min_command_ratio <= 1.0:
            raise ValueError("stuck_min_command_ratio must be in [0,1]")
        if self.minimum_successful_episodes < 1:
            raise ValueError("minimum_successful_episodes must be positive")
        if self.relocation_after_failures < 1:
            raise ValueError("relocation_after_failures must be positive")
        if min(
            self.minimum_successful_duration_sec,
            self.failure_next_goal_delay_sec,
            self.recovery_stop_dwell_sec,
            self.reset_repeat_period_sec,
            self.relocation_service_timeout_sec,
            self.relocation_odom_timeout_sec,
            self.goal_sync_timeout_sec,
            self.human_collision_confirmation_sec,
            self.human_collision_penetration_m,
        ) < 0.0:
            raise ValueError(
                "quality duration, recovery timing and collision confirmation "
                "parameters must be non-negative"
            )
        if self.relocation_odom_tolerance_m <= 0.0:
            raise ValueError("relocation_odom_tolerance_m must be positive")
        collision_threshold = self.robot_radius + self.pedestrian_radius
        if self.human_collision_penetration_m >= collision_threshold:
            raise ValueError(
                "human_collision_penetration_m must be smaller than the "
                "combined robot and pedestrian radii"
            )
        self.bounds = {
            "short": (self.short_min_m, self.short_max_m),
            "medium": (self.medium_min_m, self.medium_max_m),
            "long": (self.long_min_m, self.long_max_m),
        }
        if any(lo < 0.0 or hi <= lo for lo, hi in self.bounds.values()):
            raise ValueError("every distance bucket must have 0 <= min < max")
        self.weights = {
            "short": self.short_weight,
            "medium": self.medium_weight,
            "long": self.long_weight,
        }

    def _now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def _steady_now_ns(self):
        return int(self.steady_clock.now().nanoseconds)

    def _moving(self, velocity):
        return (
            abs(velocity[0]) >= self.motion_linear_threshold
            or abs(velocity[1]) >= self.motion_angular_threshold
        )

    def _stopped(self, velocity):
        return (
            abs(velocity[0]) <= self.stop_linear_threshold
            and abs(velocity[1]) <= self.stop_angular_threshold
        )

    def _publish_json(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        publisher.publish(msg)

    def _publish_event(self, kind, reason=None):
        payload = {
            "schema": EVENT_SCHEMA,
            "event": kind,
            "episode_id": self.episode_id,
            "stamp_ns": self._now_ns(),
            "goal": list(self.goal) if self.goal is not None else None,
            "pose": list(self.pose) if self.pose is not None else None,
        }
        if reason is not None:
            payload["reason"] = reason
        if self.goal_bucket is not None:
            payload["distance_bucket"] = self.goal_bucket
            payload["planned_path_length_m"] = self.goal_path_length
        self._publish_json(self.event_pub, payload)

    def _quality_quota_met(self):
        return quality_quota_met(
            self.success_count,
            self.successful_duration_sec,
            self.minimum_successful_episodes,
            self.minimum_successful_duration_sec,
        )

    def _enter_cooldown(self, delay_sec):
        self.state = "cooldown"
        self.state_start_ns = self._now_ns()
        self.cooldown_delay_sec = float(delay_sec)
        self.recovery_stop_since_ns = None
        self.pending_goal = None
        self.goal = None
        self.goal_path_length = None
        self.goal_bucket = None
        self.arrival_since_ns = None
        self.static_collision_since_ns = None
        self.human_collision_since_ns = None
        self.progress.clear()
        self._publish_episode_reset()

    def _publish_episode_reset(self):
        self.reset_pub.publish(Empty())
        self.last_reset_publish_ns = self._now_ns()

    def _relocation_failed(self, reason):
        self.relocation_state = None
        self.relocation_target = None
        self.relocation_origin = None
        self.relocation_future = None
        self.relocation_deadline_ns = None
        self.relocation_response_odom_sequence = None
        self.relocation_stopped_since_ns = None
        self.relocation_failure_count += 1
        self.get_logger().error(f"AUTO_RELOCATION_FAILED reason={reason}")
        self._complete("failed", reason)

    def _relocation_stopped_dwell_met(self):
        if not (
            self._stopped(self.odom_velocity)
            and self._stopped(self.cmd_velocity)
        ):
            self.relocation_stopped_since_ns = None
            return False
        if self.relocation_stopped_since_ns is None:
            self.relocation_stopped_since_ns = self._steady_now_ns()
        return (
            self._steady_now_ns() - self.relocation_stopped_since_ns
            >= int(self.recovery_stop_dwell_sec * 1e9)
        )

    def _begin_relocation(self, reason):
        """Relocate only after the configured run of completed failures."""

        if self.relocation_state is not None:
            return
        self.relocation_attempt_count += 1
        self.relocation_state = "waiting_service"
        self.relocation_target = RELOCATION_TARGET
        self.relocation_origin = self.pose
        self.relocation_future = None
        self.relocation_deadline_ns = self._steady_now_ns() + int(
            self.relocation_service_timeout_sec * 1e9
        )
        self.relocation_response_odom_sequence = None
        self.relocation_stopped_since_ns = None
        self.state = "relocating"
        self.state_start_ns = self._now_ns()
        self.pending_goal = None
        self.goal = None
        self.goal_path_length = None
        self.goal_bucket = None
        self.arrival_since_ns = None
        self.progress.clear()
        self._publish_episode_reset()
        self.get_logger().warning(
            f"AUTO_RELOCATION_REQUESTED after={self.consecutive_episode_failures} "
            f"failures reason={reason} target=(2.00,2.00)"
        )

    def _send_relocation_request(self):
        if self.relocation_state != "waiting_service":
            return
        if not self._relocation_stopped_dwell_met():
            return
        backend = getattr(self, "relocation_backend", "gazebo_set_entity_pose")
        if backend == "isaac_pose_topic":
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_id
            message.pose.position.x = float(self.relocation_target[0])
            message.pose.position.y = float(self.relocation_target[1])
            yaw = float(self.relocation_target[2])
            message.pose.orientation.z = math.sin(yaw / 2.0)
            message.pose.orientation.w = math.cos(yaw / 2.0)
            self.isaac_reset_pub.publish(message)
            self.relocation_state = "waiting_odom"
            self.relocation_response_odom_sequence = self.odom_sequence
            self.relocation_stopped_since_ns = None
            self.relocation_deadline_ns = self._steady_now_ns() + int(
                self.relocation_odom_timeout_sec * 1e9
            )
            self.get_logger().info(
                "AUTO_RELOCATION_ISAAC_RESET_PUBLISHED: waiting for fresh odometry"
            )
            return
        if not self.reset_client.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.robot_entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(self.relocation_target[0])
        request.pose.position.y = float(self.relocation_target[1])
        request.pose.position.z = 0.0
        yaw = float(self.relocation_target[2])
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self.relocation_state = "service_pending"
        future = self.reset_client.call_async(request)
        self.relocation_future = future
        future.add_done_callback(
            lambda completed, expected=future: self._on_relocation_response(
                completed, expected
            )
        )

    def _on_relocation_response(self, future, expected_future=None):
        if (
            self.relocation_state != "service_pending"
            or expected_future is not self.relocation_future
            or future is not self.relocation_future
        ):
            return
        try:
            response = future.result()
        except Exception as exc:
            self._relocation_failed(f"relocation_service_error:{exc}")
            return
        if response is None or not response.success:
            self._relocation_failed("relocation_service_rejected")
            return
        self.relocation_state = "waiting_odom"
        self.relocation_response_odom_sequence = self.odom_sequence
        self.relocation_stopped_since_ns = None
        self.relocation_deadline_ns = self._steady_now_ns() + int(
            self.relocation_odom_timeout_sec * 1e9
        )
        self.get_logger().info(
            "AUTO_RELOCATION_SERVICE_SUCCEEDED: waiting for fresh odometry"
        )

    def _relocation_pose_confirmed(self):
        if (
            self.relocation_response_odom_sequence is None
            or self.odom_sequence <= self.relocation_response_odom_sequence
            or self.pose is None
        ):
            return False
        yaw_error = math.atan2(
            math.sin(self.pose[2] - RELOCATION_TARGET[2]),
            math.cos(self.pose[2] - RELOCATION_TARGET[2]),
        )
        return (
            math.hypot(
                self.pose[0] - RELOCATION_TARGET[0],
                self.pose[1] - RELOCATION_TARGET[1],
            ) <= self.relocation_odom_tolerance_m
            and abs(yaw_error) <= RELOCATION_YAW_TOLERANCE_RAD
            and self._stopped(self.odom_velocity)
            and self._stopped(self.cmd_velocity)
        )

    def _complete_relocation(self):
        if self.relocation_state != "settling" or not self._relocation_pose_confirmed():
            return
        self.relocation_count += 1
        self.consecutive_episode_failures = 0
        self.relocation_state = None
        self.relocation_target = None
        self.relocation_origin = None
        self.relocation_future = None
        self.relocation_deadline_ns = None
        self.relocation_response_odom_sequence = None
        self.relocation_stopped_since_ns = None
        self.get_logger().info(
            f"AUTO_RELOCATION_CONFIRMED count={self.relocation_count}"
        )
        self._enter_cooldown(self.failure_next_goal_delay_sec)

    def _on_relocation_watchdog(self):
        if self.done or self.relocation_state is None:
            return
        now_ns = self._steady_now_ns()
        if self.relocation_state == "waiting_service":
            stopped = self._relocation_stopped_dwell_met()
            if stopped:
                self._send_relocation_request()
        elif self.relocation_state == "waiting_odom":
            if self._relocation_pose_confirmed():
                self.relocation_state = "settling"
                self.relocation_stopped_since_ns = now_ns
        elif self.relocation_state == "settling":
            if not self._relocation_pose_confirmed():
                self.relocation_state = "waiting_odom"
                self.relocation_stopped_since_ns = None
            elif (
                self.relocation_stopped_since_ns is not None
                and now_ns - self.relocation_stopped_since_ns
                >= int(self.recovery_stop_dwell_sec * 1e9)
            ):
                self._complete_relocation()
        if self.relocation_deadline_ns is not None and now_ns >= self.relocation_deadline_ns:
            if self.relocation_state == "waiting_service":
                reason = (
                    "relocation_precondition_timeout"
                    if not self._relocation_stopped_dwell_met()
                    else "relocation_service_timeout"
                )
            elif self.relocation_state == "service_pending":
                reason = "relocation_service_timeout"
            else:
                reason = "relocation_odom_timeout"
            self._relocation_failed(
                reason
            )

    def _recover_goal_setup(self, reason):
        """Retry transient goal/control handshakes without ending the bag."""

        if not self.continue_after_episode_failure:
            self._complete("failed", reason)
            return
        self.setup_recovery_count += 1
        self.setup_recovery_reasons[reason] += 1
        self.get_logger().warning(
            f"AUTO_SETUP_RECOVERY reason={reason}; stopping the teacher and "
            "retrying a fresh goal in the same bag"
        )
        self._enter_cooldown(self.failure_next_goal_delay_sec)

    def _write_status(self, outcome, reason):
        actual_duration_sec = 0.0
        if self.capture_start_ns is not None:
            actual_duration_sec = max(
                0.0, (self._now_ns() - self.capture_start_ns) / 1e9
            )
        payload = {
            "schema": STATUS_SCHEMA,
            "outcome": outcome,
            "reason": reason,
            "stamp_ns": self._now_ns(),
            "success_count": self.success_count,
            "failure_count": self.failure_count,
            "consecutive_episode_failures": self.consecutive_episode_failures,
            "relocation_count": self.relocation_count,
            "relocation_attempt_count": self.relocation_attempt_count,
            "relocation_failure_count": self.relocation_failure_count,
            "relocation": {
                "after_failures": self.relocation_after_failures,
                "count": self.relocation_count,
                "attempt_count": self.relocation_attempt_count,
                "failure_count": self.relocation_failure_count,
                "target": list(RELOCATION_TARGET),
                "robot_reset_service": self.robot_reset_service,
                "backend": self.relocation_backend,
                "isaac_reset_pose_topic": self.isaac_reset_pose_topic,
                "robot_entity_name": self.robot_entity_name,
            },
            "successful_episode_duration_sec": self.successful_duration_sec,
            "discarded_episode_duration_sec": self.discarded_duration_sec,
            "failure_reasons": dict(sorted(self.failure_reasons.items())),
            "setup_recovery_count": self.setup_recovery_count,
            "setup_recovery_reasons": dict(
                sorted(self.setup_recovery_reasons.items())
            ),
            "last_episode_id": self.episode_id,
            "requested_duration_sec": self.capture_duration_sec,
            "robot_reset_service": self.robot_reset_service,
            "relocation_backend": self.relocation_backend,
            "isaac_reset_pose_topic": self.isaac_reset_pose_topic,
            "robot_entity_name": self.robot_entity_name,
            "actual_duration_sec": actual_duration_sec,
            "duration_deadline_reached": self.capture_deadline_reached,
            "quality_quota_met": self._quality_quota_met(),
            "quality_requirements": {
                "minimum_successful_episodes": self.minimum_successful_episodes,
                "minimum_successful_duration_sec": (
                    self.minimum_successful_duration_sec
                ),
                "continue_after_episode_failure": (
                    self.continue_after_episode_failure
                ),
                "recovery_stop_dwell_sec": self.recovery_stop_dwell_sec,
                "relocation_after_failures": self.relocation_after_failures,
                "relocation_target": list(RELOCATION_TARGET),
                "robot_reset_service": self.robot_reset_service,
                "robot_entity_name": self.robot_entity_name,
            },
            "goal_inflation_radius_m": self.goal_inflation_radius,
            "route_inflation_radius_m": self.route_inflation_radius,
            "distance_weights": self.weights,
            "collision_detection": {
                "human": "pedestrian_ground_truth_geometry_proxy",
                "static": (
                    "occupancy_map_geometry_proxy"
                    if self.enable_static_map_collision_proxy
                    else (
                        "robot-radius map proximity plus commanded-motion/"
                        "odom-stall persistence; no contact sensor available"
                    )
                ),
                "human_confirmation_sec": (
                    self.human_collision_confirmation_sec
                ),
                "human_penetration_margin_m": (
                    self.human_collision_penetration_m
                ),
            },
        }
        path = Path(self.status_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, path)
        self._publish_json(self.status_pub, payload)

    def _complete(self, outcome, reason):
        if self.done:
            return
        self.done = True
        self.state = "done"
        self.exit_code = 0 if outcome == "complete" else 4
        self._write_status(outcome, reason)
        self.get_logger().info(
            f"AUTO_CAPTURE_FINISHED outcome={outcome} reason={reason} "
            f"successes={self.success_count} failures={self.failure_count}"
        )

    def _finish_episode(self, success, reason):
        if self.state != "recording":
            self._complete("failed", reason)
            return
        now_ns = self._now_ns()
        episode_duration_sec = max(
            0.0, (now_ns - self.state_start_ns) / 1e9
        )
        self._publish_event("end", reason)
        self._publish_episode_reset()
        self.static_collision_since_ns = None
        self.human_collision_since_ns = None
        if success:
            self.consecutive_episode_failures = 0
            self.success_count += 1
            self.successful_duration_sec += episode_duration_sec
            self.get_logger().info(
                f"AUTO_EPISODE_SAVED id={self.episode_id} "
                f"bucket={self.goal_bucket} path={self.goal_path_length:.2f}m "
                f"duration={episode_duration_sec:.2f}s"
            )
            if self.capture_deadline_reached and self._quality_quota_met():
                self._complete(
                    "complete", "capture_quality_target_reached_after_episode"
                )
                return
            self._enter_cooldown(self.next_goal_delay_sec)
        else:
            self.consecutive_episode_failures = getattr(
                self, "consecutive_episode_failures", 0
            ) + 1
            self.failure_count += 1
            self.discarded_duration_sec += episode_duration_sec
            self.failure_reasons[reason] += 1
            if not self.continue_after_episode_failure:
                self._complete("failed", reason)
                return
            self.get_logger().warning(
                f"AUTO_EPISODE_DISCARDED id={self.episode_id} reason={reason} "
                f"duration={episode_duration_sec:.2f}s; continuing in same bag"
            )
            if self.capture_deadline_reached and self._quality_quota_met():
                self._complete(
                    "complete", "capture_quality_target_reached_after_discard"
                )
                return
            if self.consecutive_episode_failures >= getattr(
                self, "relocation_after_failures", math.inf
            ):
                self._begin_relocation(reason)
                return
            self._enter_cooldown(self.failure_next_goal_delay_sec)

    def _select_goal(self):
        assert self.pose is not None
        start_raw = world_to_grid(
            self.pose[0], self.pose[1], self.height, self.resolution,
            self.origin_x, self.origin_y,
        )
        start = snap_to_free(
            start_raw, self.route_free, self.resolution, 1.2
        )
        preferred = weighted_bucket(self.rng, self.weights)
        order = [preferred] + [
            name for name in ("long", "medium", "short") if name != preferred
        ]
        cells = [tuple(map(int, item)) for item in self.goal_cells]
        self.rng.shuffle(cells)
        for bucket in order:
            lower, upper = self.bounds[bucket]
            tested = 0
            for cell in cells:
                xy = grid_to_world(
                    cell[0], cell[1], self.height, self.resolution,
                    self.origin_x, self.origin_y,
                )
                straight = math.hypot(xy[0] - self.pose[0], xy[1] - self.pose[1])
                if straight < lower or straight > upper:
                    continue
                if any(
                    math.hypot(xy[0] - old[0], xy[1] - old[1])
                    < self.repeat_goal_separation_m
                    for old in self.goal_history
                ):
                    continue
                tested += 1
                try:
                    route = astar(start, cell, self.route_free)
                except RuntimeError:
                    if tested >= self.candidate_attempts:
                        break
                    continue
                length = path_length_m(route, self.resolution)
                if lower <= length <= upper:
                    return xy, bucket, length
                if tested >= self.candidate_attempts:
                    break
        raise RuntimeError("no reachable goal satisfies any configured distance bucket")

    def _request_goal(self):
        try:
            goal, bucket, path_length = self._select_goal()
        except RuntimeError as exc:
            self._recover_goal_setup(f"goal_selection_failed:{exc}")
            return
        self.pending_goal = goal
        self.goal_bucket = bucket
        self.goal_path_length = path_length
        self.state = "waiting_accept"
        self.state_start_ns = self._now_ns()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"AUTO_GOAL_REQUESTED bucket={bucket} path={path_length:.2f}m "
            f"goal=({goal[0]:.2f},{goal[1]:.2f})"
        )

    def _on_goal_accepted(self, msg):
        if self.state != "waiting_accept" or self.pending_goal is None:
            return
        if msg.header.frame_id.lstrip("/") not in ("", self.frame_id.lstrip("/")):
            return
        accepted = (float(msg.point.x), float(msg.point.y))
        if math.hypot(
            accepted[0] - self.pending_goal[0],
            accepted[1] - self.pending_goal[1],
        ) > max(0.20, 2.0 * self.resolution):
            return
        cell = world_to_grid(
            accepted[0], accepted[1], self.height, self.resolution,
            self.origin_x, self.origin_y,
        )
        if not (0 <= cell[0] < self.height and 0 <= cell[1] < self.width
                and self.goal_free[cell]):
            self._recover_goal_setup("accepted_goal_outside_safe_mask")
            return
        self.episode_id += 1
        self.goal = accepted
        self.goal_history.append(accepted)
        self.pending_goal = None
        self.state = "waiting_goal_sync"
        self.state_start_ns = self._now_ns()
        self.human_collision_since_ns = None
        self.get_logger().info(
            f"AUTO_PATH_ACCEPTED id={self.episode_id}; waiting for matching "
            "final-goal publication"
        )

    def _on_final_goal(self, msg):
        if self.state != "waiting_goal_sync" or self.goal is None:
            return
        if msg.header.frame_id.lstrip("/") not in ("", self.frame_id.lstrip("/")):
            return
        published = (float(msg.point.x), float(msg.point.y))
        if not all(math.isfinite(value) for value in published):
            return
        if math.hypot(
            published[0] - self.goal[0], published[1] - self.goal[1]
        ) > max(0.20, 2.0 * self.resolution):
            return
        self.state = "armed"
        self.state_start_ns = self._now_ns()
        self._publish_event("armed", "new_final_goal_synchronized")
        self.get_logger().info(
            f"AUTO_GOAL_ACCEPTED id={self.episode_id}; new final goal is "
            "active, waiting for fresh teacher motion"
        )

    def _on_control_event(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().error("Ignoring invalid DRL-VO control event JSON")
            return
        if not isinstance(payload, dict):
            return
        if (
            payload.get("schema") != "drl_vo_control_event/v1"
            or payload.get("event") != "actuation_deadlock"
        ):
            return
        event_goal = payload.get("final_goal")
        goal_matches = (
            self.goal is not None
            and isinstance(event_goal, list)
            and len(event_goal) == 2
            and all(isinstance(value, (int, float)) for value in event_goal)
            and all(math.isfinite(float(value)) for value in event_goal)
            and math.hypot(
                float(event_goal[0]) - self.goal[0],
                float(event_goal[1]) - self.goal[1],
            ) <= max(0.20, 2.0 * self.resolution)
        )
        if self.state == "recording" and goal_matches:
            self._finish_episode(False, "actuation_deadlock")

    def _on_odom(self, msg):
        self.odom_sequence += 1
        pose = msg.pose.pose
        self.pose = (
            float(pose.position.x), float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self.odom_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )
        if self.session_start_ns is None:
            self.session_start_ns = self._now_ns()
        if self.relocation_state == "waiting_odom" and self._relocation_pose_confirmed():
            self.relocation_state = "settling"
            self.relocation_stopped_since_ns = self._steady_now_ns()
        elif self.relocation_state == "settling" and not self._relocation_pose_confirmed():
            self.relocation_state = "waiting_odom"
            self.relocation_stopped_since_ns = None

    def _on_cmd(self, msg):
        self.cmd_velocity = (
            float(msg.twist.linear.x), float(msg.twist.angular.z)
        )
        if self.state == "armed" and self._moving(self.cmd_velocity):
            self.state = "recording"
            self.state_start_ns = self._now_ns()
            if self.capture_start_ns is None:
                self.capture_start_ns = self.state_start_ns
            self.progress.clear()
            self.human_collision_since_ns = None
            self._publish_event("start", "first_nonzero_teacher_command")
            self.get_logger().info(f"AUTO_EPISODE_STARTED id={self.episode_id}")

    def _on_pedestrians(self, msg):
        self.pedestrian_stamp_ns = self._now_ns()
        self.pedestrians = [
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in msg.pedestrians
        ]

    def _collision_reason(self, now_ns):
        if self.pose is None:
            return None
        cell = world_to_grid(
            self.pose[0], self.pose[1], self.height, self.resolution,
            self.origin_x, self.origin_y,
        )
        if self.enable_static_map_collision_proxy and not (
            0 <= cell[0] < self.height
            and 0 <= cell[1] < self.width
            and self.collision_free[cell]
        ):
            return "static_collision_geometry_proxy"
        fresh = (
            self.pedestrian_stamp_ns is not None
            and now_ns - self.pedestrian_stamp_ns
            <= int(self.pedestrian_truth_timeout_sec * 1e9)
        )
        if fresh and self.pedestrians:
            threshold = (
                self.robot_radius
                + self.pedestrian_radius
                - self.human_collision_penetration_m
            )
            candidate = min(
                math.hypot(self.pose[0] - x, self.pose[1] - y)
                for x, y in self.pedestrians
            ) <= threshold
            if candidate:
                if self.human_collision_since_ns is None:
                    self.human_collision_since_ns = now_ns
                if now_ns - self.human_collision_since_ns >= int(
                    self.human_collision_confirmation_sec * 1e9
                ):
                    return "human_collision_geometry_proxy"
                return None
        self.human_collision_since_ns = None
        return None

    def _static_collision_stalled(self, now_ns):
        """Combine map proximity with persistent command/odometry disagreement."""
        cell = world_to_grid(
            self.pose[0], self.pose[1], self.height, self.resolution,
            self.origin_x, self.origin_y,
        )
        near_static = not (
            0 <= cell[0] < self.height
            and 0 <= cell[1] < self.width
            and self.collision_free[cell]
        )
        stalled = self._moving(self.cmd_velocity) and self._stopped(
            self.odom_velocity
        )
        if near_static and stalled:
            if self.static_collision_since_ns is None:
                self.static_collision_since_ns = now_ns
            return now_ns - self.static_collision_since_ns >= int(
                self.static_collision_stall_sec * 1e9
            )
        self.static_collision_since_ns = None
        return False

    def _check_stuck(self, now_ns, goal_distance):
        self.progress.append(
            (now_ns, self.pose[0], self.pose[1], self.pose[2], goal_distance,
             self._moving(self.cmd_velocity))
        )
        cutoff = now_ns - int(self.stuck_window_sec * 1e9)
        while len(self.progress) > 1 and self.progress[1][0] <= cutoff:
            self.progress.popleft()
        if len(self.progress) < 2 or now_ns - self.progress[0][0] < int(
            0.95 * self.stuck_window_sec * 1e9
        ):
            return False
        return progress_window_is_stuck(
            self.progress,
            self.stuck_displacement_m,
            self.stuck_goal_progress_m,
            self.stuck_yaw_progress_rad,
            self.stuck_min_command_ratio,
        )

    def _on_timer(self):
        if self.done or self.pose is None or self.session_start_ns is None:
            return
        if not bool(self.get_parameter("capture_enabled").value):
            return
        now_ns = self._now_ns()
        if self.relocation_state is not None:
            return
        duration_reached = (
            self.capture_start_ns is not None
            and now_ns - self.capture_start_ns
            >= int(self.capture_duration_sec * 1e9)
        )
        if duration_reached:
            if not self.capture_deadline_reached:
                self.capture_deadline_reached = True
                if self.state in ACTIVE_EPISODE_STATES:
                    self.get_logger().info(
                        "AUTO_CAPTURE_DURATION_REACHED: waiting for active "
                        f"episode {self.episode_id} to finish"
                    )
            if self.state not in ACTIVE_EPISODE_STATES:
                if self._quality_quota_met():
                    self._complete(
                        "complete", "capture_quality_target_reached"
                    )
                    return
                if not self.quality_wait_logged:
                    self.quality_wait_logged = True
                    self.get_logger().warning(
                        "AUTO_CAPTURE_QUALITY_PENDING: duration reached but "
                        f"successes={self.success_count}/"
                        f"{self.minimum_successful_episodes}, "
                        "successful_duration="
                        f"{self.successful_duration_sec:.2f}/"
                        f"{self.minimum_successful_duration_sec:.2f}s; "
                        "continuing in the same bag"
                    )
        if self.state == "waiting_odom":
            if now_ns - self.session_start_ns >= int(self.initial_goal_delay_sec * 1e9):
                self._request_goal()
            return
        if self.state == "cooldown":
            stopped = self._stopped(self.odom_velocity) and self._stopped(
                self.cmd_velocity
            )
            if stopped:
                if self.recovery_stop_since_ns is None:
                    self.recovery_stop_since_ns = now_ns
            else:
                self.recovery_stop_since_ns = None
            if (
                self.last_reset_publish_ns is None
                or now_ns - self.last_reset_publish_ns
                >= int(self.reset_repeat_period_sec * 1e9)
            ):
                self._publish_episode_reset()
            cooldown_elapsed = now_ns - self.state_start_ns >= int(
                self.cooldown_delay_sec * 1e9
            )
            stop_dwell_met = (
                self.recovery_stop_since_ns is not None
                and now_ns - self.recovery_stop_since_ns
                >= int(self.recovery_stop_dwell_sec * 1e9)
            )
            if cooldown_elapsed and stop_dwell_met:
                self._request_goal()
            return
        if self.state == "waiting_accept":
            if now_ns - self.state_start_ns >= int(self.goal_accept_timeout_sec * 1e9):
                self._recover_goal_setup("goal_accept_timeout")
            return
        if self.state == "waiting_goal_sync":
            if now_ns - self.state_start_ns >= int(
                self.goal_sync_timeout_sec * 1e9
            ):
                self._recover_goal_setup("final_goal_sync_timeout")
            return
        if self.state == "armed":
            if now_ns - self.state_start_ns >= int(self.goal_accept_timeout_sec * 1e9):
                self._recover_goal_setup("teacher_did_not_start")
            return
        if self.state != "recording" or self.goal is None:
            return
        elapsed = now_ns - self.state_start_ns
        if elapsed >= int(self.episode_timeout_sec * 1e9):
            self._finish_episode(False, "episode_timeout")
            return
        if elapsed >= int(self.collision_grace_sec * 1e9):
            if self._static_collision_stalled(now_ns):
                self._finish_episode(
                    False, "static_collision_map_proximity_and_stall_proxy"
                )
                return
            collision = self._collision_reason(now_ns)
            if collision:
                self._finish_episode(False, collision)
                return
        distance = math.hypot(self.pose[0] - self.goal[0], self.pose[1] - self.goal[1])
        if self._check_stuck(now_ns, distance):
            self._finish_episode(False, "stuck_no_progress")
            return
        stopped = self._stopped(self.odom_velocity) and self._stopped(self.cmd_velocity)
        if distance <= self.goal_tolerance and stopped:
            if self.arrival_since_ns is None:
                self.arrival_since_ns = now_ns
            elif now_ns - self.arrival_since_ns >= int(self.arrival_dwell_sec * 1e9):
                self._finish_episode(True, SUCCESS_REASON)
        else:
            self.arrival_since_ns = None


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutoGoalRosbagScheduler()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code
    except KeyboardInterrupt:
        if node.state == "recording":
            node._finish_episode(False, "scheduler_interrupted")
        if not node.done:
            node._complete("failed", "scheduler_interrupted")
        code = node.exit_code
    finally:
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
