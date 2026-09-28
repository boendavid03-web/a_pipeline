#!/usr/bin/env python3
"""Backend-local, deterministic social crowd motion for Isaac Sim 5.

This is deliberately pure Python.  It adapts the project's Gazebo social
interaction and the Isaac 6 patrol-cursor semantics to the Isaac 5 coloured
character + kinematic-capsule adapter.  It owns neither USD nor PhysX: the
caller supplies a conservative free-space predicate and is the sole writer of
the resulting pose.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Sequence

import numpy as np


Vector2 = tuple[float, float]
SegmentIsSafe = Callable[[Vector2, Vector2], bool]


def _unit(value: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(value))
    return np.zeros(2, dtype=float) if length <= 1.0e-9 else value / length


def _limit(value: np.ndarray, maximum: float) -> np.ndarray:
    length = float(np.linalg.norm(value))
    return value if length <= maximum else value * (maximum / length)


def _angle_delta(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class CrowdSocialConfig:
    """Limits retained from the shared Gazebo/Isaac social-controller contract."""

    agent_radius_m: float = 0.25
    robot_radius_m: float = 0.45
    human_influence_m: float = 1.45
    robot_influence_m: float = 1.75
    personal_space_m: float = 0.75
    human_repulsion_mps2: float = 1.35
    robot_repulsion_mps2: float = 1.8
    max_acceleration_mps2: float = 1.20
    max_deceleration_mps2: float = 1.80
    max_yaw_rate_radps: float = math.radians(100.0)
    collision_prediction_sec: float = 1.60
    yield_trigger_m: float = 1.60
    route_lookahead_m: float = 0.90
    waypoint_reach_m: float = 0.38
    stall_speed_mps: float = 0.06
    stall_recovery_sec: float = 2.0


@dataclass
class PatrolCursor:
    """Ping-pong look-ahead cursor adapted from ``pedestrian_steering.py``.

    Lobby routes are open polylines.  Treating them as cyclic would request an
    unvalidated last-to-first segment and make the free-space gate hard-stop at
    an endpoint.  The cursor therefore reverses along the cleared polyline.
    """

    route: tuple[Vector2, ...]
    target_index: int = 1
    route_progress: int = 0
    lap_count: int = 0
    travel_direction: int = 1
    lookahead_index: int | None = None
    lookahead_checkpoint_advance_count: int = 0

    def __post_init__(self) -> None:
        if len(self.route) < 2:
            raise ValueError("a crowd patrol route requires at least two points")
        self.target_index = min(max(self.target_index, 0), len(self.route) - 1)
        self.travel_direction = 1 if self.travel_direction >= 0 else -1
        self.lookahead_index = self.target_index

    def _advance(self) -> None:
        """Advance one point on an open ping-pong route without a seam jump."""
        if self.target_index == len(self.route) - 1 and self.travel_direction > 0:
            self.travel_direction = -1
            self.target_index -= 1
        elif self.target_index == 0 and self.travel_direction < 0:
            self.travel_direction = 1
            self.target_index += 1
            self.lap_count += 1
        else:
            self.target_index += self.travel_direction
        self.route_progress += 1

    def _waypoint_passed_at(
        self,
        index: int,
        position: np.ndarray,
        config: CrowdSocialConfig,
    ) -> bool:
        """Isaac6 cursor contract: accept a point that was reached or passed."""
        target = np.asarray(self.route[index], dtype=float)
        if float(np.linalg.norm(position - target)) <= config.waypoint_reach_m:
            return True
        previous_index = index - self.travel_direction
        if previous_index < 0 or previous_index >= len(self.route):
            return False
        previous = np.asarray(self.route[previous_index], dtype=float)
        segment = target - previous
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1.0e-12:
            return True
        progress = float(np.dot(position - previous, segment) / length_squared)
        return progress >= 1.0 and float(np.linalg.norm(position - target)) <= max(
            2.0 * config.route_lookahead_m,
            3.0 * config.waypoint_reach_m,
        )

    def _advance_through(self, checkpoint_index: int) -> None:
        for _ in range(len(self.route)):
            previous = self.target_index
            self._advance()
            if previous == checkpoint_index:
                return

    def direction(self, position: np.ndarray, config: CrowdSocialConfig) -> np.ndarray:
        # This is adapted directly from the verified Isaac6
        # PatrolPolylineCursor.  Lookahead can command a point beyond the
        # current semantic corner; acknowledge that commanded checkpoint once
        # it is reached or passed, otherwise a sharp corner can remain current
        # forever while the walker circles roughly one metre away from it.
        previous_lookahead = int(self.lookahead_index)
        if (
            previous_lookahead != self.target_index
            and self._waypoint_passed_at(previous_lookahead, position, config)
        ):
            self._advance_through(previous_lookahead)
            self.lookahead_checkpoint_advance_count += 1

        # Consume reached or passed points.  The bounded loop handles dense
        # points and makes progress monotonic without an endpoint teleport.
        for _ in range(len(self.route)):
            if not self._waypoint_passed_at(self.target_index, position, config):
                break
            self._advance()

        target = np.asarray(self.route[self.target_index], dtype=float)
        lookahead = self.target_index
        distance = float(np.linalg.norm(target - position))
        while len(self.route) > 2 and distance < config.route_lookahead_m:
            candidate = lookahead + self.travel_direction
            if candidate < 0 or candidate >= len(self.route):
                break
            candidate_point = np.asarray(self.route[candidate], dtype=float)
            distance += float(
                np.linalg.norm(candidate_point - np.asarray(self.route[lookahead]))
            )
            lookahead = candidate
        self.lookahead_index = lookahead
        return _unit(np.asarray(self.route[lookahead], dtype=float) - position)


@dataclass
class CrowdAgent:
    stable_id: str
    track_id: int
    cursor: PatrolCursor
    preferred_speed_mps: float
    position_xy: np.ndarray
    yaw_rad: float
    velocity_xy: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    desired_velocity_xy: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    animation_phase: float = 0.0
    stalled_seconds: float = 0.0
    recovery_count: int = 0
    free_space_emergency_hold_count: int = 0
    control_steps: int = 0
    stalled_control_steps: int = 0
    speed_sum_mps: float = 0.0
    avoidance_state_counts: dict[str, int] = field(default_factory=dict)
    last_avoidance_state: str = "route"
    max_speed_mps: float = 0.0
    max_acceleration_mps2: float = 0.0
    max_yaw_rate_radps: float = 0.0
    cumulative_path_m: float = 0.0
    maximum_displacement_m: float = 0.0
    initial_position_xy: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

    def __post_init__(self) -> None:
        self.position_xy = np.asarray(self.position_xy, dtype=float)
        self.initial_position_xy = self.position_xy.copy()


class CrowdSocialController:
    """One authoritative, deterministic position/yaw integrator for a crowd."""

    def __init__(self, agents: Sequence[CrowdAgent], config: CrowdSocialConfig | None = None) -> None:
        if not agents:
            raise ValueError("crowd must contain at least one agent")
        ids = [agent.stable_id for agent in agents]
        tracks = [agent.track_id for agent in agents]
        if len(set(ids)) != len(ids) or len(set(tracks)) != len(tracks):
            raise ValueError("stable IDs and track IDs must be unique")
        self.agents = list(agents)
        self.config = config or CrowdSocialConfig()

    def _repulsion(
        self, position: np.ndarray, velocity: np.ndarray, other_position: np.ndarray,
        other_velocity: np.ndarray, influence: float, weight: float,
    ) -> np.ndarray:
        """Gazebo-style distance/relative-velocity social force, bounded later."""
        offset = position - other_position
        distance = float(np.linalg.norm(offset))
        if distance <= 1.0e-6 or distance >= influence:
            return np.zeros(2, dtype=float)
        direction = offset / distance
        closeness = (influence - distance) / influence
        closing = max(0.0, float(np.dot(other_velocity - velocity, direction)))
        # Preserve the shared Gazebo kernel's relative-motion idea, but make
        # the near-field term decisive enough for a kinematic adapter: unlike
        # BehaviorAgent/PhysX, this layer has no native reciprocal avoidance.
        near_field = 1.0 + (0.75 / max(distance, 0.20)) ** 2
        return direction * weight * closeness * near_field * (1.0 + 0.35 * closing)

    def _predictive_right_pass(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        other_position: np.ndarray,
        other_velocity: np.ndarray,
        route_direction: np.ndarray,
        weight: float,
    ) -> np.ndarray:
        """Return an early, route-relative passing acceleration.

        Radial repulsion alone has no lateral component for head-on traffic.
        A world-axis bias is also wrong for diagonal corridors.  Predict the
        closest approach under current velocity and, when personal space is at
        risk, have every walker pass on its own right.  Opposite headings then
        produce reciprocal, opposite lateral accelerations deterministically.
        """
        relative_position = other_position - position
        relative_velocity = other_velocity - velocity
        relative_speed_sq = float(np.dot(relative_velocity, relative_velocity))
        if relative_speed_sq <= 1.0e-8:
            return np.zeros(2, dtype=float)
        time_to_closest = float(np.clip(
            -np.dot(relative_position, relative_velocity) / relative_speed_sq,
            0.0,
            self.config.collision_prediction_sec,
        ))
        if time_to_closest <= 1.0e-6:
            return np.zeros(2, dtype=float)
        closest_offset = relative_position + relative_velocity * time_to_closest
        closest_distance = float(np.linalg.norm(closest_offset))
        if closest_distance >= self.config.personal_space_m:
            return np.zeros(2, dtype=float)
        urgency = 1.0 - time_to_closest / self.config.collision_prediction_sec
        intrusion = 1.0 - closest_distance / self.config.personal_space_m
        right = np.asarray([route_direction[1], -route_direction[0]], dtype=float)
        return right * weight * (0.35 + 0.65 * urgency) * (0.50 + intrusion)

    def step(
        self,
        dt: float,
        robot_position_xy: Sequence[float],
        robot_velocity_xy: Sequence[float],
        segment_is_safe: SegmentIsSafe,
    ) -> None:
        """Advance all agents once; a rejected free-space step stops, never teleports."""
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        robot_position = np.asarray(robot_position_xy, dtype=float)
        robot_velocity = np.asarray(robot_velocity_xy, dtype=float)
        snapshot = [(a.position_xy.copy(), a.velocity_xy.copy()) for a in self.agents]
        planned: list[tuple[np.ndarray, np.ndarray, float, str]] = []
        for index, agent in enumerate(self.agents):
            position, velocity = snapshot[index]
            route_direction = agent.cursor.direction(position, self.config)
            desired = route_direction * agent.preferred_speed_mps
            acceleration = (desired - velocity) / 0.55
            avoidance = "route"
            for other_index, (other_position, other_velocity) in enumerate(snapshot):
                if index == other_index:
                    continue
                force = self._repulsion(
                    position, velocity, other_position, other_velocity,
                    self.config.human_influence_m, self.config.human_repulsion_mps2,
                )
                if float(np.linalg.norm(force)) > 0.0:
                    avoidance = "human"
                acceleration += force
                passing = self._predictive_right_pass(
                    position,
                    velocity,
                    other_position,
                    other_velocity,
                    route_direction,
                    self.config.human_repulsion_mps2,
                )
                if float(np.linalg.norm(passing)) > 0.0:
                    avoidance = "human_predictive"
                acceleration += passing
                separation = float(np.linalg.norm(position - other_position))
                if (
                    float(np.linalg.norm(passing)) > 0.0
                    and separation < self.config.yield_trigger_m
                    and agent.track_id > self.agents[other_index].track_id
                ):
                    # Adapt the Isaac6 SocialYieldPlanner's deterministic
                    # right-of-way rule for narrow shared corridors: one agent
                    # brakes before the conflict point while the lower stable
                    # track ID continues.  Keeping this asymmetric avoids the
                    # reciprocal stop/start oscillation of pure radial forces.
                    acceleration -= (
                        route_direction * self.config.human_repulsion_mps2 * 1.50
                    )
                    avoidance = "human_yield"
                if separation < self.config.personal_space_m:
                    # Exact head-on configurations have no lateral component
                    # in a radial force.  Assign a stable, deterministic side
                    # preference so the pair passes rather than deadlocks or
                    # overlaps.  This is the Isaac5 adaptation of the shared
                    # controller's steering-angle bound.
                    right = np.asarray([route_direction[1], -route_direction[0]], dtype=float)
                    acceleration += right * self.config.human_repulsion_mps2 * 0.70
            robot_force = self._repulsion(
                position, velocity, robot_position, robot_velocity,
                self.config.robot_influence_m, self.config.robot_repulsion_mps2,
            )
            if float(np.linalg.norm(robot_force)) > 0.0:
                avoidance = "robot"
            acceleration += robot_force
            robot_distance = float(np.linalg.norm(position - robot_position))
            if robot_distance < self.config.personal_space_m + self.config.robot_radius_m:
                right = np.asarray([route_direction[1], -route_direction[0]], dtype=float)
                acceleration += right * self.config.robot_repulsion_mps2 * 0.70
            acceleration = _limit(acceleration, self.config.max_acceleration_mps2)
            candidate_velocity = velocity + acceleration * dt
            delta = candidate_velocity - velocity
            delta_limit = (
                self.config.max_acceleration_mps2 if float(np.linalg.norm(candidate_velocity)) >= float(np.linalg.norm(velocity))
                else self.config.max_deceleration_mps2
            ) * dt
            candidate_velocity = velocity + _limit(delta, delta_limit)
            candidate_velocity = _limit(candidate_velocity, agent.preferred_speed_mps)
            # Social forces may request braking, but a kinematic walker must
            # not reverse through a passing manoeuvre.  Stop and reorient; the
            # route cursor then resumes forward progress once clear.
            if float(np.dot(candidate_velocity, route_direction)) < 0.0:
                candidate_velocity = velocity - _limit(
                    velocity, self.config.max_deceleration_mps2 * dt
                )
            speed = float(np.linalg.norm(candidate_velocity))
            # At a ping-pong endpoint the walker can be fully stopped while it
            # turns toward the reverse leg.  Use route intent at very low speed
            # so the low-speed heading does not deadlock on the old direction.
            yaw_vector = (
                candidate_velocity
                if speed >= self.config.stall_speed_mps
                else route_direction
            )
            target_yaw = (
                agent.yaw_rad
                if float(np.linalg.norm(yaw_vector)) <= 1.0e-9
                else math.atan2(yaw_vector[1], yaw_vector[0])
            )
            yaw_delta = float(np.clip(
                _angle_delta(target_yaw, agent.yaw_rad),
                -self.config.max_yaw_rate_radps * dt, self.config.max_yaw_rate_radps * dt,
            ))
            yaw = agent.yaw_rad + yaw_delta
            # Couple translation to the bounded yaw: this prevents a character
            # moving sideways while the skeleton rotates after a sharp turn.
            heading = np.asarray([math.cos(yaw), math.sin(yaw)])
            if speed > self.config.stall_speed_mps and float(np.dot(heading, _unit(candidate_velocity))) < 0.0:
                # Reorient in place rather than walking backward while a
                # capped yaw catches up after an avoidance turn.
                candidate_velocity = velocity - _limit(
                    velocity, self.config.max_deceleration_mps2 * dt
                )
            else:
                candidate_velocity = heading * speed
            # Heading coupling changes both scalar speed and vector direction.
            # Re-apply the vector acceleration/deceleration bound so a legal
            # yaw step cannot add a small unaccounted lateral acceleration.
            coupled_delta_limit = (
                self.config.max_acceleration_mps2
                if float(np.linalg.norm(candidate_velocity)) >= float(np.linalg.norm(velocity))
                else self.config.max_deceleration_mps2
            ) * dt
            candidate_velocity = velocity + _limit(
                candidate_velocity - velocity, coupled_delta_limit
            )
            coupled_speed = float(np.linalg.norm(candidate_velocity))
            if coupled_speed >= self.config.stall_speed_mps:
                yaw = math.atan2(candidate_velocity[1], candidate_velocity[0])
            candidate_position = position + candidate_velocity * dt
            # Qualify enough corridor to stop under the configured maximum
            # deceleration, not just the next 16.7 ms endpoint.  This prevents
            # a lateral proposal from reaching a boundary where no bounded
            # braking step remains available.
            candidate_speed = float(np.linalg.norm(candidate_velocity))
            candidate_stop = candidate_position + candidate_velocity * (
                candidate_speed / max(2.0 * self.config.max_deceleration_mps2, 1.0e-9)
            )
            if not segment_is_safe(tuple(position), tuple(candidate_stop)):
                # Reject the unsafe social/turning proposal, but preserve the
                # kinematic contract by trying one bounded braking step along
                # the previously accepted velocity before an emergency hold.
                braked_velocity = velocity - _limit(
                    velocity, self.config.max_deceleration_mps2 * dt
                )
                braked_position = position + braked_velocity * dt
                braked_speed = float(np.linalg.norm(braked_velocity))
                braked_stop = braked_position + braked_velocity * (
                    braked_speed / max(2.0 * self.config.max_deceleration_mps2, 1.0e-9)
                )
                if segment_is_safe(tuple(position), tuple(braked_stop)):
                    candidate_velocity = braked_velocity
                    candidate_position = braked_position
                    avoidance = "free_space_brake"
                else:
                    candidate_velocity = np.zeros(2, dtype=float)
                    candidate_position = position
                    avoidance = "free_space_emergency_hold"
            planned.append((candidate_position, candidate_velocity, yaw, avoidance))

        for agent, (position, velocity, yaw, avoidance) in zip(self.agents, planned):
            distance = float(np.linalg.norm(position - agent.position_xy))
            speed = float(np.linalg.norm(velocity))
            acceleration = float(np.linalg.norm(velocity - agent.velocity_xy)) / dt
            yaw_rate = abs(_angle_delta(yaw, agent.yaw_rad)) / dt
            agent.position_xy = position
            agent.velocity_xy = velocity
            agent.desired_velocity_xy = velocity.copy()
            agent.yaw_rad = yaw
            agent.last_avoidance_state = avoidance
            agent.control_steps += 1
            agent.speed_sum_mps += speed
            agent.avoidance_state_counts[avoidance] = (
                agent.avoidance_state_counts.get(avoidance, 0) + 1
            )
            if avoidance == "free_space_emergency_hold":
                agent.free_space_emergency_hold_count += 1
            agent.cumulative_path_m += distance
            agent.maximum_displacement_m = max(
                agent.maximum_displacement_m,
                float(np.linalg.norm(position - agent.initial_position_xy)),
            )
            agent.max_speed_mps = max(agent.max_speed_mps, speed)
            agent.max_acceleration_mps2 = max(agent.max_acceleration_mps2, acceleration)
            agent.max_yaw_rate_radps = max(agent.max_yaw_rate_radps, yaw_rate)
            agent.animation_phase += speed * dt
            agent.stalled_seconds = agent.stalled_seconds + dt if speed < self.config.stall_speed_mps else 0.0
            if speed < self.config.stall_speed_mps:
                agent.stalled_control_steps += 1
            if agent.stalled_seconds >= self.config.stall_recovery_sec:
                # A recovery records an explicit controller state and retries
                # the same forward target after reorientation.  It never
                # teleports or flips a two-point patrol backward.
                agent.recovery_count += 1
                agent.stalled_seconds = 0.0

    def summary(self) -> dict[str, object]:
        return {
            "pedestrian_count": len(self.agents),
            "unique_stable_ids": len({a.stable_id for a in self.agents}),
            "maximum_displacement_m": [a.maximum_displacement_m for a in self.agents],
            "cumulative_path_length_m": [a.cumulative_path_m for a in self.agents],
            "route_progress": [a.cursor.route_progress for a in self.agents],
            "route_laps": [a.cursor.lap_count for a in self.agents],
            "stall_seconds": [a.stalled_seconds for a in self.agents],
            "stall_ratio": [
                a.stalled_control_steps / max(a.control_steps, 1) for a in self.agents
            ],
            "recovery_count": [a.recovery_count for a in self.agents],
            "free_space_emergency_hold_count": [
                a.free_space_emergency_hold_count for a in self.agents
            ],
            "maximum_speed_mps": [a.max_speed_mps for a in self.agents],
            "average_speed_mps": [
                a.speed_sum_mps / max(a.control_steps, 1) for a in self.agents
            ],
            "maximum_acceleration_mps2": [a.max_acceleration_mps2 for a in self.agents],
            "maximum_yaw_rate_radps": [a.max_yaw_rate_radps for a in self.agents],
            "last_avoidance_state": [a.last_avoidance_state for a in self.agents],
            "avoidance_state_counts": [a.avoidance_state_counts for a in self.agents],
            "control_authority": "CrowdSocialController is the only movement writer",
            "physical_truth_scope": "kinematic capsules are LiDAR/occupancy geometry; no mesh-contact truth",
        }
