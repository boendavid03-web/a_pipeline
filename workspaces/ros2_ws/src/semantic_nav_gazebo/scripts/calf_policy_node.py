#!/usr/bin/env python3
"""Minimal ROS 2 deployment adapter for the current LegNav CALF PPO policy.

Inputs are deliberately limited to the two raw lidars, odometry, and the
robot-local SemanticCNN subgoal.  In particular, pedestrian ground truth is
never subscribed to or included in the observation.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Iterable

os.environ.setdefault("JAX_PLATFORMS", "cpu")

import flax.serialization
import jax
import jax.numpy as jnp
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from navigation_evaluation_msgs.msg import InferenceMetrics
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from legnav.config import RobotConfig
from legnav.core.jax_env import MAX_LIDAR_DIST, ROBOT_RADIUS
from legnav.core.jax_network import EndToEndActorCritic, scale_action_to_env
from legnav.core.jax_wrappers import _ego_deltas, assemble_stacked_obs, strided_indices


NUM_RAYS = 216
STACK_DIM = 3
STACK_STRIDE = int(RobotConfig.LIDAR_STACK_STRIDE)
BUFFER_LEN = (STACK_DIM - 1) * STACK_STRIDE + 1
OBS_DIM = 668
ACTION_DIM = 2
MAX_GOAL_DIST = math.sqrt(12.0**2 + 12.0**2)
TARGET_ANGLES = np.linspace(-math.pi, math.pi, NUM_RAYS, dtype=np.float32)
TARGET_INCREMENT = float(2.0 * math.pi / (NUM_RAYS - 1))


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate_point(point, quaternion):
    """Quaternion rotation matching v7_dual_laser_scan_merger.py."""
    px, py, pz = point
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    return (
        px + qw * tx + (qy * tz - qz * ty),
        py + qw * ty + (qz * tx - qx * tz),
        pz + qw * tz + (qx * ty - qy * tx),
    )


def scan_points_in_base(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    sensor_range_min: float,
    sensor_range_max: float,
    translation,
    quaternion,
    *,
    input_range_min: float = 0.1,
    max_lidar_dist: float = MAX_LIDAR_DIST,
    self_filter=(-0.36, 0.36, -0.32, 0.32),
) -> np.ndarray:
    """Convert valid native beams to hit points in base_link.

    This is the existing virtual geometry: native polar beam -> sensor-frame
    Cartesian point -> TF/extrinsic -> base_link Cartesian point.  It does not
    create an intermediate 360-slot scan.
    """
    raw = np.asarray(tuple(ranges), dtype=np.float32)
    angles = float(angle_min) + np.arange(raw.size, dtype=np.float32) * float(
        angle_increment
    )
    low = max(float(sensor_range_min), float(input_range_min))
    high = min(float(sensor_range_max), float(max_lidar_dist))
    valid = np.isfinite(raw) & (raw >= low) & (raw <= high)
    points = []
    min_x, max_x, min_y, max_y = self_filter
    tx, ty, tz = (float(value) for value in translation)
    for index in np.flatnonzero(valid):
        measured = float(raw[index])
        angle = float(angles[index])
        rx, ry, rz = rotate_point(
            (measured * math.cos(angle), measured * math.sin(angle), 0.0),
            quaternion,
        )
        x, y = tx + rx, ty + ry
        if min_x <= x <= max_x and min_y <= y <= max_y:
            continue
        virtual_range = math.hypot(x, y)
        if low <= virtual_range <= max_lidar_dist:
            points.append((x, y, tz + rz))
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def uniform_calf_scan(points: np.ndarray) -> np.ndarray:
    """Nearest-return projection into CALF's fixed uniform 216-ray ring."""
    scan = np.full(NUM_RAYS, MAX_LIDAR_DIST, dtype=np.float32)
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not points.size:
        return scan
    ranges = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    valid = np.isfinite(ranges) & np.isfinite(angles) & (ranges > 0.0)
    ranges = np.clip(ranges[valid], 0.0, MAX_LIDAR_DIST)
    indices = np.rint((angles[valid] + math.pi) / TARGET_INCREMENT).astype(np.int64)
    indices = np.clip(indices, 0, NUM_RAYS - 1)
    np.minimum.at(scan, indices, ranges)
    return scan


def normalize_calf_scan(scan_m: np.ndarray) -> np.ndarray:
    """Exact LegNav training-side inverted range preprocessing."""
    values = np.asarray(scan_m, dtype=np.float32)
    values = np.clip(values, 0.0, MAX_LIDAR_DIST)
    return np.clip(
        (MAX_LIDAR_DIST - values) / (MAX_LIDAR_DIST - ROBOT_RADIUS),
        0.0,
        1.0,
    ).astype(np.float32)


def goal_and_kinematics(local_goal, linear_x: float, angular_z: float, max_v: float):
    """Exact feature order and scaling from legnav.core.jax_env.get_obs."""
    goal = np.asarray(local_goal, dtype=np.float32).reshape(2)
    distance = float(np.linalg.norm(goal))
    bearing = math.atan2(float(goal[1]), float(goal[0]))
    goal_vec = goal / np.float32(MAX_GOAL_DIST)
    kin_vec = np.asarray(
        [
            float(linear_x) / max(float(max_v), 1.0e-3),
            float(angular_z),
            (float(max_v) - 0.2) / 1.8,
            distance / MAX_GOAL_DIST,
            bearing / math.pi,
        ],
        dtype=np.float32,
    )
    return goal_vec, kin_vec


def tree_shapes(tree) -> dict[str, tuple[int, ...]]:
    result = {}
    for path, leaf in jax.tree_util.tree_flatten_with_path(tree)[0]:
        name = "/".join(
            str(getattr(item, "key", getattr(item, "idx", item))) for item in path
        )
        result[name] = tuple(np.asarray(leaf).shape)
    return result


def load_calf_ppo(checkpoint: Path, max_v: float):
    """Load only a shape-exact current-network PPO checkpoint."""
    network = EndToEndActorCritic(
        action_dim=ACTION_DIM, stack_dim=STACK_DIM, num_rays=NUM_RAYS
    )
    template = network.init(
        jax.random.PRNGKey(0), jnp.zeros((1, OBS_DIM), dtype=jnp.float32)
    )["params"]
    with checkpoint.open("rb") as stream:
        bundle = flax.serialization.msgpack_restore(stream.read())
    params = bundle.get("params", bundle)
    expected = tree_shapes(template)
    actual = tree_shapes(params)
    problems = []
    for name in sorted(set(expected) | set(actual)):
        if name not in expected:
            problems.append(f"extra {name}: actual={actual[name]}")
        elif name not in actual:
            problems.append(f"missing {name}: expected={expected[name]}")
        elif expected[name] != actual[name]:
            problems.append(
                f"{name}: expected={expected[name]}, actual={actual[name]}"
            )
    if problems:
        raise ValueError("checkpoint parameter shape mismatch: " + "; ".join(problems))

    def infer(observation):
        mean, _, _ = network.apply({"params": params}, observation[None])
        return scale_action_to_env(jnp.squeeze(mean, axis=0), float(max_v))

    compiled = jax.jit(infer)
    warmup = np.asarray(compiled(jnp.zeros((OBS_DIM,), dtype=jnp.float32)))
    if warmup.shape != (ACTION_DIM,) or not np.isfinite(warmup).all():
        raise ValueError(f"invalid checkpoint warmup action: {warmup!r}")
    parameter_count = sum(int(np.asarray(leaf).size) for leaf in jax.tree_util.tree_leaves(params))
    return compiled, parameter_count


def observation_components(kin_vec, goal_buffer, pose_buffer, lidar_buffer):
    selection = np.asarray(strided_indices(len(goal_buffer), STACK_STRIDE))
    selected_goal = np.asarray(goal_buffer, dtype=np.float32)[selection]
    selected_pose = jnp.asarray(np.asarray(pose_buffer, dtype=np.float32)[selection])
    ego_delta = np.asarray(_ego_deltas(selected_pose), dtype=np.float32)
    selected_lidar = np.asarray(lidar_buffer, dtype=np.float32)[selection]
    observation = np.asarray(
        assemble_stacked_obs(
            jnp.asarray(kin_vec),
            jnp.asarray(np.asarray(goal_buffer, dtype=np.float32)),
            jnp.asarray(np.asarray(pose_buffer, dtype=np.float32)),
            jnp.asarray(np.asarray(lidar_buffer, dtype=np.float32)),
            STACK_STRIDE,
        ),
        dtype=np.float32,
    )
    return selected_lidar, selected_goal, ego_delta, observation


def smoke_observation(checkpoint: Path, max_v: float) -> None:
    infer, _ = load_calf_ppo(checkpoint, max_v)
    raw_scan = np.full(NUM_RAYS, MAX_LIDAR_DIST, dtype=np.float32)
    raw_scan[NUM_RAYS // 2] = 2.0
    lidar = normalize_calf_scan(raw_scan)
    goal, kin = goal_and_kinematics((2.0, 0.4), 0.0, 0.0, max_v)
    lidar_buffer = [lidar.copy() for _ in range(BUFFER_LEN)]
    goal_buffer = [goal.copy() for _ in range(BUFFER_LEN)]
    pose_buffer = [np.zeros(3, dtype=np.float32) for _ in range(BUFFER_LEN)]
    lidar_history, goal_history, ego_motion, observation = observation_components(
        kin, goal_buffer, pose_buffer, lidar_buffer
    )
    action = np.asarray(infer(jnp.asarray(observation)))
    print(f"checkpoint path: {checkpoint.resolve()}")
    print(f"LiDAR history shape: {lidar_history.shape}")
    print(f"goal history shape: {goal_history.shape}")
    print(f"ego-motion shape: {ego_motion.shape}")
    print(f"kinematic shape: {kin.shape}")
    print(f"final observation shape: {observation.shape}")
    print(f"action shape: {action.shape}")
    print(f"observation finite: {bool(np.isfinite(observation).all())}")
    print(f"action finite: {bool(np.isfinite(action).all())}")
    print(f"action: {action.tolist()}")
    if observation.shape != (OBS_DIM,) or action.shape != (ACTION_DIM,):
        raise SystemExit(1)
    if not np.isfinite(observation).all() or not np.isfinite(action).all():
        raise SystemExit(1)


class CalfPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("calf_policy")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("scan_01_topic", "/scan_01")
        self.declare_parameter("scan_02_topic", "/scan_02")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("local_subgoal_topic", "/semantic_cnn/local_subgoal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_linear", 0.8)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("scan_timeout", 0.75)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("subgoal_timeout", 0.5)
        self.declare_parameter("command_timeout", 0.75)
        self.declare_parameter("odom_jump_reset_distance", 1.0)
        self.declare_parameter("odom_jump_reset_yaw", 1.0)
        self.declare_parameter("trace_path", "")
        self.max_v = float(self.get_parameter("max_linear").value)
        if not 0.0 < self.max_v <= 2.0:
            raise ValueError("max_linear must be in (0, 2.0]")
        checkpoint = Path(str(self.get_parameter("checkpoint").value))
        if not checkpoint.is_file():
            raise FileNotFoundError(f"CALF checkpoint not found: {checkpoint}")
        self.infer, self.parameter_count = load_calf_ppo(checkpoint, self.max_v)
        self.checkpoint = checkpoint.resolve()

        self.pose = None
        self.odom_velocity = None
        self.odom_stamp_ns = None
        self.goal = None
        self.goal_stamp_ns = None
        self.lidar_buffer = deque(maxlen=BUFFER_LEN)
        self.goal_buffer = deque(maxlen=BUFFER_LEN)
        self.pose_buffer = deque(maxlen=BUFFER_LEN)
        self.last_scan_stamp_ns = None
        self.last_scan_wall_time = None
        self.last_command_time = None
        self.inference_count = 0
        self.last_clock_ns = None

        trace_path = str(self.get_parameter("trace_path").value).strip()
        self.trace_stream = None
        if trace_path:
            path = Path(trace_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists():
                raise FileExistsError(f"refusing to overwrite CALF trace: {path}")
            self.trace_stream = path.open("x", encoding="utf-8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.raw_action_pub = self.create_publisher(Twist, "/calf/raw_action", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/calf/scan_216", qos_profile_sensor_data
        )
        self.metrics_pub = self.create_publisher(
            InferenceMetrics, "/navigation_evaluation/inference_metrics", 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("local_subgoal_topic").value),
            self.goal_callback,
            10,
        )
        scan_01 = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_01_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        scan_02 = message_filters.Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("scan_02_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [scan_01, scan_02], queue_size=20, slop=float(self.get_parameter("sync_slop").value)
        )
        self.sync.registerCallback(self.scan_callback)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            f"CALF PPO ready checkpoint={self.checkpoint} obs={OBS_DIM} "
            f"rays={NUM_RAYS} stack={STACK_DIM} stride={STACK_STRIDE} max_v={self.max_v:.3f}; "
            "inputs=/scan_01,/scan_02,/odom,/semantic_cnn/local_subgoal only"
        )

    def reset_history(self) -> None:
        self.lidar_buffer.clear()
        self.goal_buffer.clear()
        self.pose_buffer.clear()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        pose = np.asarray(
            [position.x, position.y, yaw_from_quaternion(message.pose.pose.orientation)],
            dtype=np.float32,
        )
        velocity = np.asarray(
            [message.twist.twist.linear.x, message.twist.twist.angular.z],
            dtype=np.float32,
        )
        if not np.isfinite(pose).all() or not np.isfinite(velocity).all():
            self.pose = self.odom_velocity = None
            self.odom_stamp_ns = None
            self.reset_history()
            self.publish_stop()
            return
        if self.pose is not None:
            translation = float(np.linalg.norm(pose[:2] - self.pose[:2]))
            yaw_difference = math.atan2(
                math.sin(float(pose[2] - self.pose[2])),
                math.cos(float(pose[2] - self.pose[2])),
            )
            if (
                translation > float(self.get_parameter("odom_jump_reset_distance").value)
                or abs(yaw_difference) > float(self.get_parameter("odom_jump_reset_yaw").value)
            ):
                self.get_logger().warning("odom jump; resetting CALF temporal history")
                self.reset_history()
                self.publish_stop()
        self.pose = pose
        self.odom_velocity = velocity
        self.odom_stamp_ns = stamp_ns(message.header.stamp)

    def goal_callback(self, message: PointStamped) -> None:
        expected = str(self.get_parameter("base_frame").value).lstrip("/")
        actual = str(message.header.frame_id).lstrip("/")
        if actual and actual != expected:
            self.get_logger().error(
                f"rejecting local subgoal frame={actual!r}; expected {expected!r}"
            )
            self.goal = None
            self.goal_stamp_ns = None
            self.reset_history()
            self.publish_stop()
            return
        goal = np.asarray([message.point.x, message.point.y], dtype=np.float32)
        if not np.isfinite(goal).all():
            self.goal = None
            self.goal_stamp_ns = None
            self.reset_history()
            self.publish_stop()
            return
        self.goal = goal
        self.goal_stamp_ns = stamp_ns(message.header.stamp)

    def lookup_transform(self, scan: LaserScan):
        return self.tf_buffer.lookup_transform(
            str(self.get_parameter("base_frame").value),
            scan.header.frame_id.strip(),
            Time(),
            timeout=Duration(seconds=float(self.get_parameter("tf_timeout").value)),
        )

    def points_for_scan(self, scan: LaserScan) -> np.ndarray:
        transform = self.lookup_transform(scan)
        t = transform.transform.translation
        q = transform.transform.rotation
        return scan_points_in_base(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            (t.x, t.y, t.z),
            (q.x, q.y, q.z, q.w),
        )

    def inputs_fresh(self, scan_stamp: int) -> bool:
        if self.pose is None or self.odom_velocity is None or self.goal is None:
            return False
        odom_limit = int(float(self.get_parameter("odom_timeout").value) * 1.0e9)
        goal_limit = int(float(self.get_parameter("subgoal_timeout").value) * 1.0e9)
        return (
            self.odom_stamp_ns is not None
            and self.goal_stamp_ns is not None
            and abs(scan_stamp - self.odom_stamp_ns) <= odom_limit
            and abs(scan_stamp - self.goal_stamp_ns) <= goal_limit
        )

    def publish_scan(self, source: LaserScan, scan_m: np.ndarray) -> None:
        message = LaserScan()
        message.header.stamp = source.header.stamp
        message.header.frame_id = str(self.get_parameter("base_frame").value)
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = TARGET_INCREMENT
        message.time_increment = 0.0
        message.scan_time = source.scan_time
        message.range_min = 0.0
        message.range_max = MAX_LIDAR_DIST
        message.ranges = scan_m.astype(float).tolist()
        self.scan_pub.publish(message)

    def publish_metrics(self, source: LaserScan, durations, action) -> None:
        message = InferenceMetrics()
        message.header.stamp = self.get_clock().now().to_msg()
        message.input_stamp = source.header.stamp
        message.sequence_id = self.inference_count
        message.producer_id = "calf_ppo_policy"
        message.success = True
        message.preprocessing_ms = float(durations[0])
        message.policy_ms = float(durations[1])
        message.postprocessing_ms = float(durations[2])
        message.total_ms = float(sum(durations))
        message.device = "jax_cpu"
        message.model_parameters = self.parameter_count
        message.action_encoding = "twist_linear_x_mps_angular_z_radps"
        message.action = [float(action[0]), float(action[1])]
        self.metrics_pub.publish(message)

    def write_trace(self, scan_stamp_value, scan_m, normalized, goal, observation, action, hz):
        if self.trace_stream is None:
            return
        payload = {
            "schema": "calf_isaac_inference/v1",
            "sequence": self.inference_count,
            "scan_stamp_ns": int(scan_stamp_value),
            "checkpoint": str(self.checkpoint),
            "observation_shape": list(observation.shape),
            "scan_216_m": scan_m.astype(float).tolist(),
            "scan_216_normalized": normalized.astype(float).tolist(),
            "local_goal_base_link": goal.astype(float).tolist(),
            "odom_pose": self.pose.astype(float).tolist(),
            "odom_velocity": self.odom_velocity.astype(float).tolist(),
            "action": action.astype(float).tolist(),
            "inference_frequency_hz": None if hz is None else float(hz),
        }
        self.trace_stream.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.trace_stream.flush()

    def scan_callback(self, scan_01: LaserScan, scan_02: LaserScan) -> None:
        started = time.perf_counter()
        first_stamp = stamp_ns(scan_01.header.stamp)
        second_stamp = stamp_ns(scan_02.header.stamp)
        slop_ns = int(float(self.get_parameter("sync_slop").value) * 1.0e9)
        if abs(first_stamp - second_stamp) > slop_ns or not self.inputs_fresh(first_stamp):
            self.reset_history()
            self.publish_stop()
            return
        try:
            points = np.concatenate(
                (self.points_for_scan(scan_01), self.points_for_scan(scan_02)), axis=0
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f"CALF scan projection failed: {error}", throttle_duration_sec=2.0
            )
            self.reset_history()
            self.publish_stop()
            return
        scan_m = uniform_calf_scan(points)
        normalized = normalize_calf_scan(scan_m)
        goal_vec, kin_vec = goal_and_kinematics(
            self.goal, self.odom_velocity[0], self.odom_velocity[1], self.max_v
        )
        if not self.lidar_buffer:
            for _ in range(BUFFER_LEN):
                self.lidar_buffer.append(normalized.copy())
                self.goal_buffer.append(goal_vec.copy())
                self.pose_buffer.append(self.pose.copy())
        else:
            self.lidar_buffer.append(normalized.copy())
            self.goal_buffer.append(goal_vec.copy())
            self.pose_buffer.append(self.pose.copy())
        lidar_history, goal_history, ego_motion, observation = observation_components(
            kin_vec, self.goal_buffer, self.pose_buffer, self.lidar_buffer
        )
        preprocessing_done = time.perf_counter()
        if observation.shape != (OBS_DIM,) or not np.isfinite(observation).all():
            self.get_logger().error("non-finite or wrong-shape CALF observation")
            self.reset_history()
            self.publish_stop()
            return
        action = np.asarray(self.infer(jnp.asarray(observation)), dtype=np.float32)
        policy_done = time.perf_counter()
        if action.shape != (ACTION_DIM,) or not np.isfinite(action).all():
            self.get_logger().error("non-finite or wrong-shape CALF action")
            self.reset_history()
            self.publish_stop()
            return
        command = Twist()
        command.linear.x = float(action[0])
        command.linear.y = 0.0
        command.angular.z = float(action[1])
        self.cmd_pub.publish(command)
        self.raw_action_pub.publish(command)
        self.publish_scan(scan_01, scan_m)
        finished = time.perf_counter()
        self.inference_count += 1
        now_wall = time.monotonic()
        hz = None
        if self.last_scan_wall_time is not None and now_wall > self.last_scan_wall_time:
            hz = 1.0 / (now_wall - self.last_scan_wall_time)
        self.last_scan_wall_time = now_wall
        self.last_scan_stamp_ns = first_stamp
        self.last_command_time = self.get_clock().now()
        durations = (
            (preprocessing_done - started) * 1.0e3,
            (policy_done - preprocessing_done) * 1.0e3,
            (finished - policy_done) * 1.0e3,
        )
        self.publish_metrics(scan_01, durations, action)
        self.write_trace(first_stamp, scan_m, normalized, self.goal, observation, action, hz)
        if self.inference_count == 1:
            self.get_logger().info(
                "CALF first inference PASS "
                f"lidar_history={lidar_history.shape} goal_history={goal_history.shape} "
                f"ego_motion={ego_motion.shape} kinematic={kin_vec.shape} "
                f"observation={observation.shape} action={action.shape}"
            )

    def watchdog(self) -> None:
        if self.last_command_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1.0e9
        if elapsed > float(self.get_parameter("command_timeout").value):
            self.publish_stop()

    def destroy_node(self):
        if self.trace_stream is not None:
            self.trace_stream.close()
            self.trace_stream = None
        return super().destroy_node()


def default_checkpoint() -> Path:
    project_root = Path(
        os.environ.get("NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5])
    )
    return (
        project_root
        / "github_src"
        / "drl_vo_nav-drl_vo"
        / "LegNav-Sim-master"
        / "checkpoints"
        / "ppo"
        / "ppo_legs_best.msgpack"
    )


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--smoke-observation", action="store_true")
    parser.add_argument("--checkpoint", default=str(default_checkpoint()))
    known, _ = parser.parse_known_args()
    if known.smoke_observation:
        smoke_observation(Path(known.checkpoint), 0.8)
        return
    rclpy.init(args=sys.argv)
    node = CalfPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
