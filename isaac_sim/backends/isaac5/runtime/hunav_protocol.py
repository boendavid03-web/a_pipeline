#!/usr/bin/env python3
"""Versioned JSON boundary and Isaac-owned HuNav command execution primitives."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Callable, Mapping, Sequence

import numpy as np


STATE_SCHEMA = "a_pipeline.isaac_hunav.state/v1"
COMMAND_SCHEMA = "a_pipeline.isaac_hunav.command/v1"
STATUS_SCHEMA = "a_pipeline.isaac_hunav.status/v1"


class ProtocolError(ValueError):
    pass


def _finite(value: object, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError(f"{label} must be finite")
    return number


def _vec2(value: object, label: str) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ProtocolError(f"{label} must be a two-element array")
    return np.asarray([_finite(value[0], label), _finite(value[1], label)], dtype=float)


def validate_state(payload: object) -> dict:
    if not isinstance(payload, dict) or payload.get("schema") != STATE_SCHEMA:
        raise ProtocolError("invalid state schema")
    if not isinstance(payload.get("session_id"), str) or not payload["session_id"]:
        raise ProtocolError("state session_id is required")
    sequence = int(payload.get("sequence", -1))
    sim_time = _finite(payload.get("sim_time"), "sim_time")
    if sequence < 0 or sim_time < 0.0:
        raise ProtocolError("sequence and sim_time must be non-negative")
    agents = payload.get("agents")
    if not isinstance(agents, list) or not agents:
        raise ProtocolError("state must contain agents")
    ids: set[int] = set()
    stable_ids: set[str] = set()
    for row in agents:
        if not isinstance(row, dict):
            raise ProtocolError("agent must be an object")
        agent_id = int(row.get("id", -1))
        if agent_id < 0 or agent_id in ids:
            raise ProtocolError("agent ids must be unique and non-negative")
        ids.add(agent_id)
        stable_id = str(row.get("stable_id", "")).strip()
        track_id = int(row.get("track_id", -1))
        if not stable_id or stable_id in stable_ids or track_id != agent_id:
            raise ProtocolError("agent stable_id must be unique and track_id must equal id")
        stable_ids.add(stable_id)
        _vec2(row.get("position"), "agent.position")
        _vec2(row.get("velocity"), "agent.velocity")
        _finite(row.get("yaw", 0.0), "agent.yaw")
        goals = row.get("goals", [])
        if not isinstance(goals, list) or not goals:
            raise ProtocolError("agent goals must be non-empty")
        for goal in goals:
            _vec2(goal, "agent.goal")
        behavior = row.get("behavior")
        if not isinstance(behavior, dict) or str(behavior.get("type", "")) not in {
            "regular", "impassive", "surprised", "scared", "curious", "threatening"
        }:
            raise ProtocolError("agent behavior must name a supported HuNav behavior")
        for label in (
            "desired_velocity", "radius", "goal_radius",
        ):
            if _finite(row.get(label), f"agent.{label}") < 0.0:
                raise ProtocolError(f"agent.{label} must be non-negative")
        obstacles = row.get("closest_obstacles")
        if not isinstance(obstacles, list):
            raise ProtocolError("agent.closest_obstacles must be an array")
        for obstacle in obstacles:
            _vec2(obstacle, "agent.closest_obstacle")
    robot = payload.get("robot")
    if not isinstance(robot, dict):
        raise ProtocolError("robot must be an object")
    _vec2(robot.get("position"), "robot.position")
    _vec2(robot.get("velocity"), "robot.velocity")
    _finite(robot.get("yaw"), "robot.yaw")
    _finite(robot.get("angular_velocity", 0.0), "robot.angular_velocity")
    footprint = robot.get("footprint")
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise ProtocolError("robot.footprint must contain at least three points")
    for point in footprint:
        _vec2(point, "robot.footprint point")
    return payload


def validate_command(payload: object) -> dict:
    if not isinstance(payload, dict) or payload.get("schema") != COMMAND_SCHEMA:
        raise ProtocolError("invalid command schema")
    if not isinstance(payload.get("session_id"), str) or not payload["session_id"]:
        raise ProtocolError("command session_id is required")
    sequence = int(payload.get("sequence", -1))
    state_sequence = int(payload.get("state_sequence", -1))
    sim_time = _finite(payload.get("sim_time"), "sim_time")
    if min(sequence, state_sequence) < 0 or sim_time < 0.0:
        raise ProtocolError("command sequence and time must be non-negative")
    commands = payload.get("commands")
    if not isinstance(commands, list):
        raise ProtocolError("commands must be an array")
    ids: set[int] = set()
    for row in commands:
        agent_id = int(row.get("id", -1))
        if agent_id < 0 or agent_id in ids:
            raise ProtocolError("command ids must be unique and non-negative")
        ids.add(agent_id)
        _vec2(row.get("desired_velocity"), "desired_velocity")
        _vec2(row.get("source_position"), "source_position")
        _vec2(row.get("target_position"), "target_position")
        _finite(row.get("yaw", 0.0), "yaw")
        if "requested_goal" in row:
            _vec2(row.get("requested_goal"), "requested_goal")
        if "active_goal" in row:
            _vec2(row.get("active_goal"), "active_goal")
        if "active_goal_count" in row and int(row["active_goal_count"]) < 1:
            raise ProtocolError("active_goal_count must be positive")
        if "goal_sync_error_m" in row:
            error = _finite(row.get("goal_sync_error_m"), "goal_sync_error_m")
            if error < 0.0:
                raise ProtocolError("goal_sync_error_m must be non-negative")
    return payload


@dataclass
class CommandInbox:
    timeout_sec: float = 0.35
    last_sequence: int = -1
    last_state_sequence: int = -1
    accepted: int = 0
    rejected_duplicate: int = 0
    rejected_out_of_order: int = 0
    rejected_session: int = 0
    rejected_command_sequence: int = 0
    rejected_state_sequence: int = 0
    rejected_future: int = 0
    rejected_invalid: int = 0
    _payload: dict | None = None
    _received_wall: float = -math.inf

    def accept(self, payload: object, current_sim_time: float, expected_session_id: str | None = None) -> bool:
        try:
            command = validate_command(payload)
        except (ProtocolError, TypeError, ValueError):
            self.rejected_invalid += 1
            return False
        if expected_session_id is not None and command["session_id"] != expected_session_id:
            self.rejected_out_of_order += 1
            self.rejected_session += 1
            return False
        sequence = int(command["sequence"])
        state_sequence = int(command["state_sequence"])
        if sequence == self.last_sequence:
            self.rejected_duplicate += 1
            return False
        if sequence < self.last_sequence or state_sequence < self.last_state_sequence:
            self.rejected_out_of_order += 1
            self.rejected_command_sequence += int(sequence < self.last_sequence)
            self.rejected_state_sequence += int(state_sequence < self.last_state_sequence)
            return False
        if float(command["sim_time"]) > current_sim_time + 0.10:
            self.rejected_future += 1
            return False
        self.last_sequence = sequence
        self.last_state_sequence = state_sequence
        self._payload = command
        self._received_wall = time.monotonic()
        self.accepted += 1
        return True

    def current(self, current_sim_time: float) -> tuple[dict[int, dict], str | None]:
        if self._payload is None:
            return {}, "no_command"
        sim_age = current_sim_time - float(self._payload["sim_time"])
        wall_age = time.monotonic() - self._received_wall
        if sim_age < -1.0e-6:
            return {}, "future_command"
        if sim_age > self.timeout_sec or wall_age > max(1.0, 3.0 * self.timeout_sec):
            return {}, "command_timeout"
        return {int(row["id"]): row for row in self._payload["commands"]}, None


@dataclass(frozen=True)
class SafetyConfig:
    human_radius_m: float = 0.25
    personal_space_m: float = 0.70
    time_horizon_sec: float = 2.0
    max_speed_mps: float = 1.5
    max_acceleration_mps2: float = 1.2
    max_deceleration_mps2: float = 2.2
    max_yaw_rate_radps: float = 2.2
    robot_margin_m: float = 0.12
    enforce_pair_separation: bool = True
    pair_projection_iterations: int = 8
    pair_unilateral_hold_fallback: bool = False
    pair_kinematic_feasibility: bool = False
    static_lookahead_sec: float = 0.0


@dataclass
class SafetyMetrics:
    projection_count: int = 0
    static_hold_count: int = 0
    timeout_stop_count: int = 0
    minimum_predicted_separation_m: float = math.inf
    nonfinite_stop_count: int = 0
    speed_limit_count: int = 0
    acceleration_limit_count: int = 0
    penetration_hold_count: int = 0
    static_projection_count: int = 0
    pair_projection_count: int = 0
    robot_rectangle_projection_count: int = 0


class SafeVelocityProjector:
    """Hard execution limits only; HuNav owns normal social avoidance."""

    def __init__(self, config: SafetyConfig = SafetyConfig()):
        self.config = config
        self.metrics = SafetyMetrics()
        self.last_reasons: dict[int, tuple[str, ...]] = {}

    def _bounded_velocity(
        self,
        current_velocity: np.ndarray,
        proposed_velocity: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Project one velocity into the configured speed/acceleration set."""
        cfg = self.config
        current = np.asarray(current_velocity, dtype=float)
        proposed = np.asarray(proposed_velocity, dtype=float).copy()
        speed = float(np.linalg.norm(proposed))
        if speed > cfg.max_speed_mps:
            proposed *= cfg.max_speed_mps / speed
        delta = proposed - current
        accelerating = float(np.dot(delta, proposed)) >= 0.0
        limit = (
            cfg.max_acceleration_mps2 if accelerating else cfg.max_deceleration_mps2
        ) * max(float(dt), 1.0e-6)
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > limit:
            proposed = current + delta * (limit / delta_norm)
        return proposed

    @staticmethod
    def _closest_distance(relative_position: np.ndarray, relative_velocity: np.ndarray, horizon: float) -> float:
        speed2 = float(np.dot(relative_velocity, relative_velocity))
        when = 0.0 if speed2 < 1.0e-12 else float(
            np.clip(-np.dot(relative_position, relative_velocity) / speed2, 0.0, horizon)
        )
        return float(np.linalg.norm(relative_position + relative_velocity * when))

    @staticmethod
    def _robot_support_radius(direction: np.ndarray, footprint: Sequence[Sequence[float]]) -> float:
        points = np.asarray(footprint, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-9:
            return float(np.max(np.linalg.norm(points, axis=1)))
        unit = direction / norm
        return float(np.max(np.abs(points @ unit)))

    @staticmethod
    def _robot_rectangle_projection(
        person_xy: np.ndarray,
        robot_xy: np.ndarray,
        robot_yaw_rad: float,
        footprint: Sequence[Sequence[float]],
        clearance_m: float,
    ) -> tuple[np.ndarray, float]:
        """Project a circle centre outside an oriented rectangular footprint."""
        points = np.asarray(footprint, dtype=float)
        half_x = float(np.max(np.abs(points[:, 0])))
        half_y = float(np.max(np.abs(points[:, 1])))
        cosine, sine = math.cos(robot_yaw_rad), math.sin(robot_yaw_rad)
        delta = np.asarray(person_xy, dtype=float) - np.asarray(robot_xy, dtype=float)
        local = np.asarray([
            cosine * delta[0] + sine * delta[1],
            -sine * delta[0] + cosine * delta[1],
        ])
        closest = np.clip(local, [-half_x, -half_y], [half_x, half_y])
        away = local - closest
        distance = float(np.linalg.norm(away))
        if distance >= clearance_m:
            return np.asarray(person_xy, dtype=float), distance
        if distance > 1.0e-9:
            corrected_local = closest + away * (clearance_m / distance)
        else:
            x_depth = half_x - abs(float(local[0]))
            y_depth = half_y - abs(float(local[1]))
            corrected_local = local.copy()
            if x_depth <= y_depth:
                sign = 1.0 if local[0] >= 0.0 else -1.0
                corrected_local[0] = sign * (half_x + clearance_m)
            else:
                sign = 1.0 if local[1] >= 0.0 else -1.0
                corrected_local[1] = sign * (half_y + clearance_m)
        corrected_delta = np.asarray([
            cosine * corrected_local[0] - sine * corrected_local[1],
            sine * corrected_local[0] + cosine * corrected_local[1],
        ])
        return np.asarray(robot_xy, dtype=float) + corrected_delta, distance

    def project(
        self,
        positions: Mapping[int, np.ndarray],
        current_velocities: Mapping[int, np.ndarray],
        desired_velocities: Mapping[int, np.ndarray],
        dt: float,
        robot_position: np.ndarray,
        robot_velocity: np.ndarray,
        robot_footprint: Sequence[Sequence[float]],
        segment_is_safe: Callable[[np.ndarray, np.ndarray], bool],
        segment_projector: Callable[[int, np.ndarray, np.ndarray], Sequence[float] | None] | None = None,
        robot_yaw_rad: float | None = None,
        fixed_pair_agent_ids: set[int] | None = None,
    ) -> dict[int, np.ndarray]:
        cfg = self.config
        output: dict[int, np.ndarray] = {}
        self.last_reasons = {}
        dt = max(float(dt), 1.0e-6)
        changed_by_agent: dict[int, bool] = {}
        reasons_by_agent: dict[int, list[str]] = {}
        for agent_id in sorted(positions):
            position = np.asarray(positions[agent_id], dtype=float)
            current = np.asarray(current_velocities[agent_id], dtype=float)
            desired = np.asarray(desired_velocities.get(agent_id, np.zeros(2)), dtype=float)
            changed = False
            reasons: list[str] = []
            if desired.shape != (2,) or not np.all(np.isfinite(desired)):
                desired = np.zeros(2, dtype=float)
                self.metrics.nonfinite_stop_count += 1
                changed = True
                reasons.append("nonfinite")
            speed = float(np.linalg.norm(desired))
            if speed > cfg.max_speed_mps:
                desired *= cfg.max_speed_mps / speed
                self.metrics.speed_limit_count += 1
                changed = True
                reasons.append("speed_limit")
            projected = desired.copy()
            delta = projected - current
            accelerating = float(np.dot(delta, projected)) >= 0.0
            limit = (cfg.max_acceleration_mps2 if accelerating else cfg.max_deceleration_mps2) * dt
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > limit:
                projected = current + delta * (limit / delta_norm)
                self.metrics.acceleration_limit_count += 1
                changed = True
                reasons.append("acceleration_limit")
            output[agent_id] = projected
            changed_by_agent[agent_id] = changed
            reasons_by_agent[agent_id] = reasons

        ids = sorted(positions)
        fixed_pair_ids = set() if fixed_pair_agent_ids is None else set(fixed_pair_agent_ids)

        def constrain_endpoint(agent_id: int, position: np.ndarray, end: np.ndarray):
            if segment_projector is not None:
                value = segment_projector(agent_id, position, end)
                if value is None:
                    return None
                value = np.asarray(value, dtype=float)
                if value.shape != (2,) or not np.all(np.isfinite(value)):
                    return None
                return value
            return end if segment_is_safe(position, end) else None

        def bounded_endpoint_is_safe(
            agent_id: int,
            position: np.ndarray,
            velocity: np.ndarray,
        ) -> bool:
            """Check the endpoint after kinematic clipping, not before it."""
            end = position + np.asarray(velocity, dtype=float) * dt
            constrained = constrain_endpoint(agent_id, position, end)
            if constrained is None:
                return False
            # The committed-pose audit allows 5 cm of closest-point error.
            # Trigger substantially before that acceptance edge so one control
            # interval of pair correction cannot turn a marginally safe pose
            # into an audit violation.
            return float(np.linalg.norm(np.asarray(constrained) - end)) <= 0.020

        for agent_id in ids:
            position = np.asarray(positions[agent_id], dtype=float)
            lookahead = max(float(cfg.static_lookahead_sec), dt)
            if lookahead > dt + 1.0e-9:
                lookahead_end = position + output[agent_id] * lookahead
                constrained_lookahead = constrain_endpoint(
                    agent_id, position, lookahead_end
                )
                if constrained_lookahead is None:
                    output[agent_id] = self._bounded_velocity(
                        current_velocities[agent_id], np.zeros(2), dt
                    )
                    changed_by_agent[agent_id] = True
                    reasons_by_agent[agent_id].append("static_boundary_lookahead_braking")
                elif not np.allclose(
                    constrained_lookahead, lookahead_end, atol=1.0e-6, rtol=0.0
                ):
                    lookahead_velocity = (
                        np.asarray(constrained_lookahead, dtype=float) - position
                    ) / lookahead
                    output[agent_id] = self._bounded_velocity(
                        current_velocities[agent_id], lookahead_velocity, dt
                    )
                    changed_by_agent[agent_id] = True
                    reasons_by_agent[agent_id].append("static_boundary_lookahead_steering")
            end = position + output[agent_id] * dt
            constrained_end = constrain_endpoint(agent_id, position, end)
            if constrained_end is not None:
                if not np.allclose(constrained_end, end, atol=1.0e-6, rtol=0.0):
                    constrained_velocity = (constrained_end - position) / dt
                    candidate = self._bounded_velocity(
                        current_velocities[agent_id], constrained_velocity, dt
                    )
                    if cfg.pair_kinematic_feasibility:
                        if not bounded_endpoint_is_safe(agent_id, position, candidate):
                            alternatives = (
                                np.asarray(current_velocities[agent_id], dtype=float),
                                self._bounded_velocity(
                                    current_velocities[agent_id], np.zeros(2), dt
                                ),
                            )
                            candidate = next(
                                (
                                    value for value in alternatives
                                    if bounded_endpoint_is_safe(
                                        agent_id, position, value
                                    )
                                ),
                                candidate,
                            )
                    output[agent_id] = candidate
                    self.metrics.static_projection_count += 1
                    changed_by_agent[agent_id] = True
                    reasons_by_agent[agent_id].append("static_boundary_projection")
            if constrained_end is None:
                output[agent_id] = self._bounded_velocity(
                    current_velocities[agent_id], np.zeros(2), dt
                )
                self.metrics.static_hold_count += 1
                changed_by_agent[agent_id] = True
                reasons_by_agent[agent_id].append("static_boundary_hold")

        # LightSFM's social force is intentionally soft.  Enforce only the
        # hard disc non-penetration invariant with a small position-based
        # projection.  Unlike the old bilateral hold, this preserves tangent
        # motion and cannot freeze an entire contact chain by setting every
        # involved velocity to zero.
        endpoints = {
            agent_id: np.asarray(positions[agent_id], dtype=float) + output[agent_id] * dt
            for agent_id in ids
        }
        minimum_separation = 2.0 * cfg.human_radius_m + 1.0e-3
        corrected_ids: set[int] = set()
        if cfg.enforce_pair_separation and cfg.pair_kinematic_feasibility:
            # Keep pair correction inside each agent's speed/acceleration set.
            # The stopping-distance inequality activates before contact, so a
            # later exact separation correction never needs an instantaneous
            # multi-metre-per-second pose jump.
            for _ in range(max(int(cfg.pair_projection_iterations), 1)):
                changed = False
                for index, first_id in enumerate(ids):
                    for second_id in ids[index + 1 :]:
                        first_position = np.asarray(positions[first_id], dtype=float)
                        second_position = np.asarray(positions[second_id], dtype=float)
                        offset = second_position - first_position
                        distance = float(np.linalg.norm(offset))
                        if distance <= 1.0e-9:
                            normal = np.asarray(
                                [1.0, 0.0]
                                if (first_id + second_id) % 2 == 0
                                else [0.0, 1.0],
                                dtype=float,
                            )
                        else:
                            normal = offset / distance
                        relative_normal_speed = float(np.dot(
                            output[second_id] - output[first_id], normal
                        ))
                        first_fixed = first_id in fixed_pair_ids
                        second_fixed = second_id in fixed_pair_ids
                        # A normal-direction correction is not guaranteed to
                        # classify as pure deceleration in the 2-D velocity
                        # ball (especially after another pair constraint has
                        # consumed some of the budget).  Use the smaller
                        # acceleration/deceleration limit for the stopping
                        # look-ahead; otherwise the solver can begin braking
                        # later than _bounded_velocity can physically apply.
                        conservative_rate = min(
                            cfg.max_acceleration_mps2,
                            cfg.max_deceleration_mps2,
                        )
                        relative_deceleration = conservative_rate * (
                            1.0 if first_fixed or second_fixed else 2.0
                        )
                        # Start reducing closing speed at the social-space
                        # boundary, but keep the endpoint hard constraint at
                        # the physical disc separation below.  The reserve is
                        # consumed only while agents are converging; parallel
                        # walkers may remain closer than personal_space_m.
                        # This leaves acceleration budget for additional pair
                        # and NavMesh constraints before contact becomes
                        # physically infeasible.
                        braking_separation = max(
                            minimum_separation,
                            cfg.personal_space_m,
                        )
                        clearance = max(distance - braking_separation, 0.0)
                        # Discrete-time stopping invariant.  The selected
                        # closing speed is applied for this whole control
                        # interval before another deceleration step is
                        # available, so reserve ``q * dt`` in addition to the
                        # continuous stopping distance ``q^2 / (2a)``.
                        closing_limit = (
                            -relative_deceleration * dt
                            + math.sqrt(
                                (relative_deceleration * dt) ** 2
                                + 2.0 * relative_deceleration * clearance
                            )
                        )
                        braking_floor = -closing_limit
                        endpoint_floor = (minimum_separation - distance) / dt
                        required_relative_speed = max(braking_floor, endpoint_floor)
                        deficit = required_relative_speed - relative_normal_speed
                        if deficit <= 1.0e-9:
                            continue

                        first_before = output[first_id].copy()
                        second_before = output[second_id].copy()
                        if first_fixed and not second_fixed:
                            first_target = first_before
                            second_target = second_before + deficit * normal
                        elif second_fixed and not first_fixed:
                            first_target = first_before - deficit * normal
                            second_target = second_before
                        else:
                            first_target = first_before - 0.5 * deficit * normal
                            second_target = second_before + 0.5 * deficit * normal
                        output[first_id] = self._bounded_velocity(
                            current_velocities[first_id], first_target, dt
                        )
                        output[second_id] = self._bounded_velocity(
                            current_velocities[second_id], second_target, dt
                        )

                        for agent_id, before in (
                            (first_id, first_before),
                            (second_id, second_before),
                        ):
                            position = np.asarray(positions[agent_id], dtype=float)
                            end = position + output[agent_id] * dt
                            constrained = constrain_endpoint(agent_id, position, end)
                            if constrained is None:
                                candidate = before
                            elif not np.allclose(
                                constrained, end, atol=1.0e-6, rtol=0.0
                            ):
                                candidate = self._bounded_velocity(
                                    current_velocities[agent_id],
                                    (np.asarray(constrained) - position) / dt,
                                    dt,
                                )
                                reasons_by_agent[agent_id].append(
                                    "static_boundary_projection"
                                )
                            else:
                                candidate = output[agent_id]
                            if not bounded_endpoint_is_safe(
                                agent_id, position, candidate
                            ):
                                if bounded_endpoint_is_safe(
                                    agent_id, position, before
                                ):
                                    candidate = before
                            output[agent_id] = candidate
                        if not np.allclose(output[first_id], first_before) or not np.allclose(
                            output[second_id], second_before
                        ):
                            corrected_ids.update((first_id, second_id))
                            self.metrics.pair_projection_count += 1
                            changed = True
                            reasons_by_agent[first_id].append("kinematic_pair_braking")
                            reasons_by_agent[second_id].append("kinematic_pair_braking")
                if not changed:
                    break
            endpoints = {
                agent_id: np.asarray(positions[agent_id], dtype=float)
                + output[agent_id] * dt
                for agent_id in ids
            }
        elif cfg.enforce_pair_separation:
            for _ in range(max(int(cfg.pair_projection_iterations), 1)):
                changed = False
                for index, first_id in enumerate(ids):
                    for second_id in ids[index + 1 :]:
                        first_position = np.asarray(positions[first_id], dtype=float)
                        second_position = np.asarray(positions[second_id], dtype=float)
                        first_fixed = first_id in fixed_pair_ids
                        second_fixed = second_id in fixed_pair_ids
                        if cfg.pair_unilateral_hold_fallback:
                            if first_fixed:
                                endpoints[first_id] = first_position.copy()
                                corrected_ids.add(first_id)
                                changed_by_agent[first_id] = True
                                reasons_by_agent[first_id].append("pair_yielder_hold")
                            if second_fixed:
                                endpoints[second_id] = second_position.copy()
                                corrected_ids.add(second_id)
                                changed_by_agent[second_id] = True
                                reasons_by_agent[second_id].append("pair_yielder_hold")
                        offset = endpoints[second_id] - endpoints[first_id]
                        distance = float(np.linalg.norm(offset))
                        if distance + 1.0e-9 >= minimum_separation:
                            continue
                        if distance > 1.0e-9:
                            normal = offset / distance
                        else:
                            normal = np.asarray(
                                [1.0, 0.0] if (first_id + second_id) % 2 == 0 else [0.0, 1.0],
                                dtype=float,
                            )
                        correction = normal * (minimum_separation - distance)
                        if first_fixed and not second_fixed:
                            second = constrain_endpoint(
                                second_id, second_position, endpoints[second_id] + correction
                            )
                            if second is None:
                                continue
                            endpoints[second_id] = second
                        elif second_fixed and not first_fixed:
                            first = constrain_endpoint(
                                first_id, first_position, endpoints[first_id] - correction
                            )
                            if first is None:
                                continue
                            endpoints[first_id] = first
                        else:
                            first = constrain_endpoint(
                                first_id, first_position, endpoints[first_id] - 0.5 * correction
                            )
                            second = constrain_endpoint(
                                second_id, second_position, endpoints[second_id] + 0.5 * correction
                            )
                            if first is not None and second is not None:
                                endpoints[first_id], endpoints[second_id] = first, second
                            elif first is not None:
                                full = constrain_endpoint(
                                    first_id, first_position, endpoints[first_id] - correction
                                )
                                if full is None:
                                    continue
                                endpoints[first_id] = full
                            elif second is not None:
                                full = constrain_endpoint(
                                    second_id, second_position, endpoints[second_id] + correction
                                )
                                if full is None:
                                    continue
                                endpoints[second_id] = full
                            else:
                                continue
                        if cfg.pair_unilateral_hold_fallback:
                            resolved_distance = float(np.linalg.norm(
                                endpoints[second_id] - endpoints[first_id]
                            ))
                            if resolved_distance + 1.0e-9 < minimum_separation:
                                candidates = []
                                for first, second in (
                                    (first_position, endpoints[second_id]),
                                    (endpoints[first_id], second_position),
                                    (first_position, second_position),
                                ):
                                    candidate_distance = float(np.linalg.norm(second - first))
                                    if candidate_distance + 1.0e-9 >= minimum_separation:
                                        retained_motion = float(
                                            np.linalg.norm(first - first_position)
                                            + np.linalg.norm(second - second_position)
                                        )
                                        candidates.append((retained_motion, first.copy(), second.copy()))
                                if candidates:
                                    _motion, first, second = max(candidates, key=lambda item: item[0])
                                    endpoints[first_id], endpoints[second_id] = first, second
                                    reasons_by_agent[first_id].append("pair_unilateral_hold")
                                    reasons_by_agent[second_id].append("pair_unilateral_hold")
                        corrected_ids.update((first_id, second_id))
                        self.metrics.pair_projection_count += 1
                        changed = True
                if not changed:
                    break


        for agent_id in ids:
            if agent_id in corrected_ids:
                output[agent_id] = (
                    endpoints[agent_id] - np.asarray(positions[agent_id], dtype=float)
                ) / dt
                changed_by_agent[agent_id] = True
                reasons_by_agent[agent_id].append("physical_pair_projection")

        if robot_yaw_rad is not None:
            predicted_robot = np.asarray(robot_position, dtype=float) + np.asarray(
                robot_velocity, dtype=float
            ) * dt
            required_clearance = cfg.human_radius_m + cfg.robot_margin_m
            for agent_id in ids:
                position = np.asarray(positions[agent_id], dtype=float)
                endpoint = endpoints[agent_id]
                corrected, distance = self._robot_rectangle_projection(
                    endpoint,
                    predicted_robot,
                    float(robot_yaw_rad),
                    robot_footprint,
                    required_clearance,
                )
                if distance + 1.0e-9 >= required_clearance:
                    continue
                constrained = constrain_endpoint(agent_id, position, corrected)
                if constrained is not None:
                    _checked, constrained_distance = self._robot_rectangle_projection(
                        np.asarray(constrained, dtype=float),
                        predicted_robot,
                        float(robot_yaw_rad),
                        robot_footprint,
                        required_clearance,
                    )
                else:
                    constrained_distance = -math.inf
                if constrained is None or constrained_distance + 1.0e-9 < required_clearance:
                    # Waiting is the only safe action when the static/NavMesh
                    # constraint does not admit the rectangle-boundary point.
                    constrained = position.copy()
                endpoints[agent_id] = np.asarray(constrained, dtype=float)
                output[agent_id] = (endpoints[agent_id] - position) / dt
                self.metrics.robot_rectangle_projection_count += 1
                changed_by_agent[agent_id] = True
                reasons_by_agent[agent_id].append("physical_robot_rectangle_projection")

        for index, first_id in enumerate(ids):
            for second_id in ids[index + 1 :]:
                separation = float(np.linalg.norm(endpoints[second_id] - endpoints[first_id]))
                self.metrics.minimum_predicted_separation_m = min(
                    self.metrics.minimum_predicted_separation_m, separation
                )

        for agent_id in ids:
            if changed_by_agent[agent_id]:
                self.metrics.projection_count += 1
            self.last_reasons[agent_id] = tuple(reasons_by_agent[agent_id])
        return output


class IndependentYieldPlanner:
    """Pairwise hysteresis for dense-crowd liveness recovery.

    Active yielders are kept mutually separated by ``resume_distance_m`` so
    adjacent agents are never both deliberately paused.  Every yielder tracks
    the peer that triggered it and is released only after that peer clears and
    nobody remains inside the trigger radius.  This is the integer-ID version
    of the Isaac 6 social-yield invariant used by this project.
    """

    def __init__(
        self,
        trigger_distance_m: float = 0.90,
        resume_distance_m: float = 1.10,
        maximum_hold_sec: float = 1.0,
        cooldown_sec: float = 3.0,
    ) -> None:
        if not 0.0 < trigger_distance_m < resume_distance_m:
            raise ValueError("yield distances must satisfy 0 < trigger < resume")
        if maximum_hold_sec <= 0.0 or cooldown_sec < 0.0:
            raise ValueError("yield timing must be positive with non-negative cooldown")
        self.trigger_distance_m = float(trigger_distance_m)
        self.resume_distance_m = float(resume_distance_m)
        self.maximum_hold_sec = float(maximum_hold_sec)
        self.cooldown_sec = float(cooldown_sec)
        self.active: set[int] = set()
        self.trigger_peers: dict[int, int] = {}
        self.started_at: dict[int, float] = {}
        self.cooldown_until: dict[tuple[int, int], float] = {}
        self.agent_cooldown_until: dict[int, float] = {}
        self.yield_counts: dict[int, int] = {}
        self.start_count = 0
        self.end_count = 0

    def update(
        self,
        positions: Mapping[int, np.ndarray],
        stalled_ids: set[int],
        current_time_sec: float,
    ) -> set[int]:
        points = {
            int(agent_id): np.asarray(position, dtype=float)[:2]
            for agent_id, position in positions.items()
        }
        present = set(points)
        now = float(current_time_sec)
        removed = self.active - present
        self.active.intersection_update(present)
        for agent_id in removed:
            self.trigger_peers.pop(agent_id, None)
            self.started_at.pop(agent_id, None)
        self.cooldown_until = {
            pair: until for pair, until in self.cooldown_until.items() if until > now
        }
        self.agent_cooldown_until = {
            agent_id: until
            for agent_id, until in self.agent_cooldown_until.items()
            if agent_id in present and until > now
        }

        forced_resumes: set[int] = set()

        def release(agent_id: int) -> None:
            peer = self.trigger_peers.pop(agent_id, None)
            self.started_at.pop(agent_id, None)
            self.active.discard(agent_id)
            forced_resumes.add(agent_id)
            self.end_count += 1
            self.agent_cooldown_until[agent_id] = now + self.cooldown_sec
            if peer is not None:
                # Protect the released agent, not the whole conflict pair.
                # The still-stalled peer must be allowed to take the next
                # bounded yield turn immediately so the released walker gets
                # an explicit right-of-way window.
                self.cooldown_until[tuple(sorted((agent_id, peer)))] = now

        for agent_id in sorted(self.active):
            distances = {
                other_id: float(np.linalg.norm(points[other_id] - points[agent_id]))
                for other_id in points
                if other_id != agent_id
            }
            peer = self.trigger_peers.get(agent_id)
            peer_clear = peer not in distances or distances[peer] >= self.resume_distance_m
            trigger_clear = all(
                distance >= self.trigger_distance_m for distance in distances.values()
            )
            held_too_long = now - self.started_at.get(agent_id, now) >= self.maximum_hold_sec
            # Dense contact graphs may never become completely clear.  Make
            # yielding a bounded, rotating turn rather than letting one person
            # become a permanent obstacle while waiting for global clearance.
            if held_too_long or (trigger_clear and peer_clear):
                release(agent_id)

        active_ids = sorted(self.active)
        for left_index, left_id in enumerate(active_ids):
            if left_id not in self.active:
                continue
            for right_id in active_ids[left_index + 1 :]:
                if right_id not in self.active:
                    continue
                if float(np.linalg.norm(points[right_id] - points[left_id])) < self.resume_distance_m:
                    release(right_id)

        ids = sorted(points)
        stalled = set(stalled_ids) & present
        for left_index, left_id in enumerate(ids):
            for right_id in ids[left_index + 1 :]:
                distance = float(np.linalg.norm(points[right_id] - points[left_id]))
                if distance >= self.trigger_distance_m:
                    continue
                if left_id in self.active or right_id in self.active:
                    continue
                candidates = stalled & {left_id, right_id}
                candidates = {
                    agent_id
                    for agent_id in candidates
                    if self.agent_cooldown_until.get(agent_id, -math.inf) <= now
                }
                if not candidates:
                    continue
                yielder = min(
                    candidates,
                    key=lambda agent_id: (
                        self.yield_counts.get(agent_id, 0),
                        -agent_id,
                    ),
                )
                if yielder in forced_resumes:
                    continue
                pair = (left_id, right_id)
                if self.cooldown_until.get(pair, -math.inf) > now:
                    continue
                if any(
                    float(np.linalg.norm(points[active_id] - points[yielder]))
                    < self.resume_distance_m
                    for active_id in self.active
                ):
                    continue
                peer = left_id if yielder == right_id else right_id
                self.active.add(yielder)
                self.trigger_peers[yielder] = peer
                self.started_at[yielder] = now
                self.yield_counts[yielder] = self.yield_counts.get(yielder, 0) + 1
                self.start_count += 1

        return set(self.active)


def bounded_route_social_velocity(
    route_direction: np.ndarray,
    preferred_speed_mps: float,
    hunav_velocity: np.ndarray,
    maximum_social_correction_mps: float = 0.35,
) -> np.ndarray:
    """Keep route progress while retaining a bounded HuNav steering residual."""
    direction = np.asarray(route_direction, dtype=float)
    social = np.asarray(hunav_velocity, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-9 or social.shape != (2,) or not np.all(np.isfinite(social)):
        return social.copy()
    base = direction / norm * max(float(preferred_speed_mps), 0.0)
    correction = social - base
    correction_norm = float(np.linalg.norm(correction))
    limit = max(float(maximum_social_correction_mps), 0.0)
    if correction_norm > limit and correction_norm > 1.0e-9:
        correction *= limit / correction_norm
    return base + correction


def forward_recovery_goal(
    position: np.ndarray,
    remaining_goals: Sequence[Sequence[float]],
    maximum_lookahead_m: float = 2.0,
) -> np.ndarray:
    """Choose recovery lookahead without crossing a route reversal.

    Generated cyclic routes can contain a nearby turn-around.  Selecting a
    fixed goal index beyond that turn makes normal intent and recovery intent
    point in opposite directions, producing a bounded but permanent oscillation.
    """
    current = np.asarray(position, dtype=float)[:2]
    goals = [np.asarray(goal, dtype=float)[:2] for goal in remaining_goals]
    if not goals:
        return current.copy()
    initial = goals[0] - current
    initial_norm = float(np.linalg.norm(initial))
    if initial_norm <= 1.0e-9:
        return goals[0].copy()
    initial_direction = initial / initial_norm
    selected = goals[0]
    travelled = initial_norm
    previous = current
    for index, goal in enumerate(goals):
        segment = goal - previous
        segment_length = float(np.linalg.norm(segment))
        if segment_length <= 1.0e-9:
            previous = goal
            continue
        if index > 0 and float(np.dot(segment / segment_length, initial_direction)) <= 0.0:
            break
        if index > 0 and travelled + segment_length > maximum_lookahead_m:
            break
        selected = goal
        travelled += segment_length if index > 0 else 0.0
        previous = goal
    return selected.copy()


@dataclass
class RecoverySupervisor:
    """No-teleport recovery state machine; route replanning is requested, never faked."""

    low_speed_mps: float = 0.08
    stuck_after_sec: float = 2.0
    state: str = "FOLLOW_ROUTE"
    low_speed_sec: float = 0.0
    no_progress_sec: float = 0.0
    last_route_progress: float | None = None
    state_age_sec: float = 0.0
    counts: dict[str, int] = field(default_factory=lambda: {
        "YIELD": 0, "LATERAL_ESCAPE": 0, "BACKTRACK": 0,
        "REPLAN": 0, "REPLAN_FAILED": 0,
    })

    def update(
        self,
        speed: float,
        dt: float,
        route_progress: float | None = None,
        intentional_stop: bool = False,
    ) -> str:
        self.state_age_sec += dt
        made_progress = (
            route_progress is not None
            and self.last_route_progress is not None
            and route_progress > self.last_route_progress + 1.0e-4
        )
        if route_progress is not None:
            self.last_route_progress = route_progress
        low_speed = speed < self.low_speed_mps and not intentional_stop
        self.low_speed_sec = self.low_speed_sec + dt if low_speed else 0.0
        self.no_progress_sec = 0.0 if made_progress or intentional_stop else self.no_progress_sec + dt
        if (
            self.state == "FOLLOW_ROUTE"
            and self.low_speed_sec >= self.stuck_after_sec
            and self.no_progress_sec >= self.stuck_after_sec
        ):
            self.state, self.state_age_sec = "YIELD", 0.0
        elif self.state == "YIELD" and self.state_age_sec >= 1.0:
            self.state, self.state_age_sec = "LATERAL_ESCAPE", 0.0
        elif self.state == "LATERAL_ESCAPE" and self.state_age_sec >= 1.5:
            self.state, self.state_age_sec = "BACKTRACK", 0.0
        elif self.state == "BACKTRACK" and self.state_age_sec >= 1.0:
            self.state, self.state_age_sec = "REPLAN", 0.0
        if self.state in self.counts and self.state_age_sec == 0.0:
            self.counts[self.state] += 1
        return self.state

    def replan_result(self, succeeded: bool) -> None:
        if self.state != "REPLAN":
            raise RuntimeError("replan_result is only valid in REPLAN")
        if succeeded:
            self.state = "FOLLOW_ROUTE"
        else:
            self.state = "WAIT_AND_RETRY"
            self.counts["REPLAN_FAILED"] += 1
        self.state_age_sec = self.low_speed_sec = self.no_progress_sec = 0.0

    def retry_ready(self, delay_sec: float = 2.0) -> bool:
        if self.state == "WAIT_AND_RETRY" and self.state_age_sec >= delay_sec:
            self.state, self.state_age_sec = "REPLAN", 0.0
            self.counts["REPLAN"] += 1
            return True
        return False
