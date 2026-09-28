"""Pure geometry for continuous BehaviorAgent social steering.

Isaac Sim owns NavMesh legality, locomotion, and animation.  This module only
maps a complete planar desired velocity to a continuously moving local target
and keeps the underlying patrol polyline progressing without per-waypoint
BehaviorAgent task replacement.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence


Vector2 = tuple[float, float]


def _vector2(name: str, value: Sequence[float]) -> Vector2:
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly two values")
    result = float(value[0]), float(value[1])
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _unit(vector: Vector2) -> Vector2:
    length = math.hypot(*vector)
    if length <= 1.0e-12:
        return 0.0, 0.0
    return vector[0] / length, vector[1] / length


@dataclass(frozen=True)
class SteeringTargetCommand:
    """One full-2D command sent to the moving BehaviorAgent target."""

    desired_velocity_mps: Vector2
    speed_mps: float
    direction: Vector2
    target_position_m: Vector2
    target_offset_m: Vector2


@dataclass(frozen=True)
class PredictiveBoundaryCommand:
    """Velocity correction for a finite-response locomotion boundary guard."""

    velocity_mps: Vector2
    predicted_stop_position_m: Vector2
    stopping_distance_m: float
    constrained: bool
    inward_direction: Vector2


class PredictiveBoundaryLatch:
    """Hold a boundary correction through brief pose-velocity reversals."""

    def __init__(self, *, release_safe_samples: int) -> None:
        if isinstance(release_safe_samples, bool) or not isinstance(
            release_safe_samples, int
        ):
            raise ValueError("release_safe_samples must be a positive integer")
        if release_safe_samples <= 0:
            raise ValueError("release_safe_samples must be a positive integer")
        self.release_safe_samples = release_safe_samples
        self._inward_by_person: dict[str, Vector2] = {}
        self._safe_samples_by_person: dict[str, int] = {}
        self.latched_update_count = 0

    def update(
        self,
        person: str,
        command: PredictiveBoundaryCommand,
        *,
        current_position_safe: bool,
    ) -> tuple[Vector2, bool, str]:
        if not isinstance(person, str) or not person:
            raise ValueError("person must be a non-empty string")
        if not isinstance(current_position_safe, bool):
            raise ValueError("current_position_safe must be boolean")
        if not current_position_safe:
            self._inward_by_person.pop(person, None)
            self._safe_samples_by_person.pop(person, None)
            return command.velocity_mps, False, "outside_recovery_owned"
        if command.constrained:
            self._inward_by_person[person] = command.inward_direction
            self._safe_samples_by_person[person] = 0
            return command.velocity_mps, True, "prediction"
        inward = self._inward_by_person.get(person)
        if inward is None:
            return command.velocity_mps, False, "inactive"
        safe_samples = self._safe_samples_by_person.get(person, 0) + 1
        if safe_samples >= self.release_safe_samples:
            self._inward_by_person.pop(person, None)
            self._safe_samples_by_person.pop(person, None)
            return command.velocity_mps, False, "released"
        self._safe_samples_by_person[person] = safe_samples
        speed = math.hypot(*command.velocity_mps)
        self.latched_update_count += 1
        return (
            (inward[0] * speed, inward[1] * speed),
            True,
            "latched_until_consecutive_safe",
        )

    @property
    def active_people(self) -> tuple[str, ...]:
        return tuple(sorted(self._inward_by_person))


@dataclass(frozen=True)
class PedestrianRobotEmergencyStopDecision:
    """One edge-aware decision from the pedestrian/robot clearance latch."""

    active: bool
    entering: bool
    leaving: bool
    reason: str
    net_clearance_m: float
    closing_speed_mps: float
    consecutive_safe_samples: int


@dataclass(frozen=True)
class PedestrianRobotEmergencyStopOwnership:
    """Execution ownership after combining emergency stop and yield/dodge."""

    motion_inhibited: bool
    action: str


def pedestrian_robot_emergency_stop_ownership(
    decision: PedestrianRobotEmergencyStopDecision,
    *,
    externally_inhibited: bool,
) -> PedestrianRobotEmergencyStopOwnership:
    """Give emergency stop priority without stealing a yield/dodge release."""

    if decision.active:
        return PedestrianRobotEmergencyStopOwnership(
            True,
            "idle_and_set_speed_zero" if decision.entering else "set_speed_zero_hold",
        )
    if externally_inhibited:
        return PedestrianRobotEmergencyStopOwnership(
            True,
            (
                "release_deferred_to_external_owner"
                if decision.leaving
                else "external_owner"
            ),
        )
    return PedestrianRobotEmergencyStopOwnership(
        False,
        "resume_follow" if decision.leaving else "none",
    )


def circular_body_net_clearance_m(
    center_to_outline_clearance_m: float,
    body_radius_m: float,
) -> float:
    """Convert root-to-outline distance to circle-edge net clearance.

    The pedestrian body is approximated as a planar circle centered at its
    observed root.  Negative output means that circle overlaps the outline.
    """

    center_clearance = float(center_to_outline_clearance_m)
    body_radius = float(body_radius_m)
    if not math.isfinite(center_clearance):
        raise ValueError("center_to_outline_clearance_m must be finite")
    if not math.isfinite(body_radius) or body_radius <= 0.0:
        raise ValueError("body_radius_m must be finite and positive")
    return center_clearance - body_radius


class PedestrianRobotEmergencyStopLatch:
    """Latch a pedestrian stop across clearance hysteresis and safe samples."""

    def __init__(
        self,
        *,
        final_trigger_clearance_m: float,
        early_trigger_clearance_m: float,
        resume_clearance_m: float,
        release_safe_samples: int,
        minimum_closing_speed_mps: float,
    ) -> None:
        values = {
            "final_trigger_clearance_m": final_trigger_clearance_m,
            "early_trigger_clearance_m": early_trigger_clearance_m,
            "resume_clearance_m": resume_clearance_m,
            "minimum_closing_speed_mps": minimum_closing_speed_mps,
        }
        for name, value in values.items():
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if early_trigger_clearance_m < final_trigger_clearance_m:
            raise ValueError(
                "early_trigger_clearance_m must be at least the final trigger"
            )
        if resume_clearance_m <= early_trigger_clearance_m:
            raise ValueError(
                "resume_clearance_m must be greater than the early trigger"
            )
        if isinstance(release_safe_samples, bool) or not isinstance(
            release_safe_samples, int
        ):
            raise ValueError("release_safe_samples must be a positive integer")
        if release_safe_samples <= 0:
            raise ValueError("release_safe_samples must be a positive integer")
        self.final_trigger_clearance_m = float(final_trigger_clearance_m)
        self.early_trigger_clearance_m = float(early_trigger_clearance_m)
        self.resume_clearance_m = float(resume_clearance_m)
        self.release_safe_samples = release_safe_samples
        self.minimum_closing_speed_mps = float(minimum_closing_speed_mps)
        self._active_people: set[str] = set()
        self._safe_samples_by_person: dict[str, int] = {}
        self._previous_clearance_by_person: dict[str, float] = {}

    def update(
        self,
        person: str,
        net_clearance_m: float,
        dt_sec: float,
    ) -> PedestrianRobotEmergencyStopDecision:
        if not isinstance(person, str) or not person:
            raise ValueError("person must be a non-empty string")
        clearance = float(net_clearance_m)
        dt = float(dt_sec)
        if not math.isfinite(clearance):
            raise ValueError("net_clearance_m must be finite")
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_sec must be finite and positive")
        previous = self._previous_clearance_by_person.get(person)
        closing_speed = (
            max(0.0, (previous - clearance) / dt)
            if previous is not None
            else 0.0
        )
        self._previous_clearance_by_person[person] = clearance
        was_active = person in self._active_people

        if not was_active:
            if clearance <= self.final_trigger_clearance_m:
                self._active_people.add(person)
                self._safe_samples_by_person[person] = 0
                return PedestrianRobotEmergencyStopDecision(
                    True,
                    True,
                    False,
                    "final_clearance_threshold",
                    clearance,
                    closing_speed,
                    0,
                )
            if (
                clearance <= self.early_trigger_clearance_m
                and closing_speed >= self.minimum_closing_speed_mps
            ):
                self._active_people.add(person)
                self._safe_samples_by_person[person] = 0
                return PedestrianRobotEmergencyStopDecision(
                    True,
                    True,
                    False,
                    "evidence_braking_margin_approach",
                    clearance,
                    closing_speed,
                    0,
                )
            return PedestrianRobotEmergencyStopDecision(
                False,
                False,
                False,
                "inactive",
                clearance,
                closing_speed,
                0,
            )

        if clearance < self.resume_clearance_m:
            self._safe_samples_by_person[person] = 0
            return PedestrianRobotEmergencyStopDecision(
                True,
                False,
                False,
                "held_below_resume_clearance",
                clearance,
                closing_speed,
                0,
            )
        safe_samples = self._safe_samples_by_person.get(person, 0) + 1
        if safe_samples < self.release_safe_samples:
            self._safe_samples_by_person[person] = safe_samples
            return PedestrianRobotEmergencyStopDecision(
                True,
                False,
                False,
                "held_for_consecutive_safe_confirmation",
                clearance,
                closing_speed,
                safe_samples,
            )
        self._active_people.remove(person)
        self._safe_samples_by_person.pop(person, None)
        return PedestrianRobotEmergencyStopDecision(
            False,
            False,
            True,
            "released_after_consecutive_safe",
            clearance,
            closing_speed,
            safe_samples,
        )

    @property
    def active_people(self) -> tuple[str, ...]:
        return tuple(sorted(self._active_people))


def steering_target_from_velocity(
    position_m: Sequence[float],
    desired_velocity_mps: Sequence[float],
    lookahead_m: float,
) -> SteeringTargetCommand:
    """Map both velocity components to a local target without projection."""

    position = _vector2("position_m", position_m)
    velocity = _vector2("desired_velocity_mps", desired_velocity_mps)
    lookahead = _positive("lookahead_m", lookahead_m)
    speed = math.hypot(*velocity)
    direction = _unit(velocity)
    offset = direction[0] * lookahead, direction[1] * lookahead
    target = position[0] + offset[0], position[1] + offset[1]
    return SteeringTargetCommand(
        desired_velocity_mps=velocity,
        speed_mps=speed,
        direction=direction,
        target_position_m=target,
        target_offset_m=offset,
    )


def predictive_boundary_velocity(
    position_m: Sequence[float],
    desired_velocity_mps: Sequence[float],
    actual_velocity_mps: Sequence[float],
    *,
    response_time_sec: float,
    maximum_deceleration_mps2: float,
    boundary_probe_m: float,
    segment_is_safe: Callable[[Vector2, Vector2], bool],
    nearest_safe_point: Callable[[Vector2], Vector2],
) -> PredictiveBoundaryCommand:
    """Counter-steer before the measured root can coast across a boundary.

    The predicted travel is the current root speed over the locomotion response
    interval plus its constant-deceleration stopping distance.  If that segment
    is unsafe, the desired vector is projected away from the first crossed
    boundary and biased inward by the measured outward root speed.  Magnitude is
    preserved: this is a continuous steering constraint, not a stop or teleport.
    """

    position = _vector2("position_m", position_m)
    desired = _vector2("desired_velocity_mps", desired_velocity_mps)
    actual = _vector2("actual_velocity_mps", actual_velocity_mps)
    response_time = _positive("response_time_sec", response_time_sec)
    maximum_deceleration = _positive(
        "maximum_deceleration_mps2", maximum_deceleration_mps2
    )
    boundary_probe = _positive("boundary_probe_m", boundary_probe_m)
    actual_speed = math.hypot(*actual)
    desired_speed = math.hypot(*desired)
    stopping_distance = (
        actual_speed * response_time
        + actual_speed * actual_speed / (2.0 * maximum_deceleration)
    )
    actual_direction = _unit(actual)
    predicted_stop = (
        position[0] + actual_direction[0] * stopping_distance,
        position[1] + actual_direction[1] * stopping_distance,
    )
    if (
        actual_speed <= 1.0e-12
        or desired_speed <= 1.0e-12
        or segment_is_safe(position, predicted_stop)
    ):
        return PredictiveBoundaryCommand(
            velocity_mps=desired,
            predicted_stop_position_m=predicted_stop,
            stopping_distance_m=stopping_distance,
            constrained=False,
            inward_direction=(0.0, 0.0),
        )

    # Find the first unsafe prefix rather than using the final projection.  A
    # long lookahead can cross a thin blocked corner and end in a different free
    # region, whose nearest point would give the wrong recovery normal.
    safe_fraction = 0.0
    unsafe_fraction = 1.0
    for _ in range(18):
        fraction = 0.5 * (safe_fraction + unsafe_fraction)
        candidate = (
            position[0]
            + actual_direction[0] * stopping_distance * fraction,
            position[1]
            + actual_direction[1] * stopping_distance * fraction,
        )
        if segment_is_safe(position, candidate):
            safe_fraction = fraction
        else:
            unsafe_fraction = fraction
    probe_distance = min(
        stopping_distance,
        stopping_distance * unsafe_fraction + boundary_probe,
    )
    boundary_probe_position = (
        position[0] + actual_direction[0] * probe_distance,
        position[1] + actual_direction[1] * probe_distance,
    )
    safe_point = _vector2(
        "nearest_safe_point", nearest_safe_point(boundary_probe_position)
    )
    inward = _unit(
        (
            safe_point[0] - boundary_probe_position[0],
            safe_point[1] - boundary_probe_position[1],
        )
    )
    if inward == (0.0, 0.0):
        inward = (-actual_direction[0], -actual_direction[1])

    # Positive projection on ``outward`` is unsafe.  Remove it from the desired
    # vector, then counter the root's measured outward momentum with an equal
    # inward component.  Renormalization keeps the requested walking speed.
    outward = -inward[0], -inward[1]
    desired_outward = max(
        0.0, desired[0] * outward[0] + desired[1] * outward[1]
    )
    actual_outward = max(
        0.0, actual[0] * outward[0] + actual[1] * outward[1]
    )
    corrected = (
        desired[0] - desired_outward * outward[0] + actual_outward * inward[0],
        desired[1] - desired_outward * outward[1] + actual_outward * inward[1],
    )
    corrected_direction = _unit(corrected)
    if corrected_direction == (0.0, 0.0):
        corrected_direction = inward
    guarded_velocity = (
        corrected_direction[0] * desired_speed,
        corrected_direction[1] * desired_speed,
    )
    return PredictiveBoundaryCommand(
        velocity_mps=guarded_velocity,
        predicted_stop_position_m=predicted_stop,
        stopping_distance_m=stopping_distance,
        constrained=True,
        inward_direction=inward,
    )


class PatrolPolylineCursor:
    """Track a cyclic patrol while exposing a smooth forward route intent."""

    def __init__(
        self,
        points_m: Sequence[Sequence[float]],
        initial_position_m: Sequence[float],
        *,
        waypoint_reach_m: float,
        route_lookahead_m: float,
        initial_route_window_m: float | None = None,
    ) -> None:
        self.points = tuple(_vector2("patrol point", point) for point in points_m)
        if len(self.points) < 2:
            raise ValueError("a patrol polyline requires at least two points")
        self.waypoint_reach_m = _positive("waypoint_reach_m", waypoint_reach_m)
        self.route_lookahead_m = _positive("route_lookahead_m", route_lookahead_m)
        self.initial_route_window_m = (
            None
            if initial_route_window_m is None
            else _positive("initial_route_window_m", initial_route_window_m)
        )
        initial = _vector2("initial_position_m", initial_position_m)
        candidate_indices = range(len(self.points))
        if self.initial_route_window_m is not None:
            # The cursor is attached shortly after patrol startup at point 0.
            # A cyclic route may return close to its start, so a global nearest
            # search can mistake the final segment for completed progress and
            # keep steering across the seam.  Scope only this one-time phase
            # estimate to the route prefix reachable during startup.
            startup_indices = [0]
            startup_distance_m = 0.0
            for index in range(1, len(self.points)):
                startup_distance_m += math.dist(
                    self.points[index - 1], self.points[index]
                )
                if startup_distance_m > self.initial_route_window_m:
                    break
                startup_indices.append(index)
            candidate_indices = startup_indices
        closest = min(
            candidate_indices,
            key=lambda index: math.dist(initial, self.points[index]),
        )
        self.target_index = (closest + 1) % len(self.points)
        self.lookahead_index = self.target_index
        self.advance_count = 0
        self.lap_count = 0
        self.lookahead_checkpoint_advance_count = 0
        self.visibility_limited_count = 0
        self.last_visibility_limited = False

    def _advance(self) -> None:
        previous = self.target_index
        self.target_index = (self.target_index + 1) % len(self.points)
        self.advance_count += 1
        if self.target_index <= previous:
            self.lap_count += 1

    def _waypoint_passed_at(self, index: int, position: Vector2) -> bool:
        target = self.points[index]
        if math.dist(position, target) <= self.waypoint_reach_m:
            return True
        previous = self.points[(index - 1) % len(self.points)]
        segment = target[0] - previous[0], target[1] - previous[1]
        length_squared = segment[0] * segment[0] + segment[1] * segment[1]
        if length_squared <= 1.0e-12:
            return True
        progress = (
            (position[0] - previous[0]) * segment[0]
            + (position[1] - previous[1]) * segment[1]
        ) / length_squared
        return progress >= 1.0 and math.dist(position, target) <= max(
            2.0 * self.route_lookahead_m,
            3.0 * self.waypoint_reach_m,
        )

    def _waypoint_passed(self, position: Vector2) -> bool:
        return self._waypoint_passed_at(self.target_index, position)

    def _advance_through(self, checkpoint_index: int) -> None:
        """Advance the semantic cursor through one previously commanded point."""

        for _ in range(len(self.points)):
            previous = self.target_index
            self._advance()
            if previous == checkpoint_index:
                return

    def desired_direction(
        self,
        position_m: Sequence[float],
        *,
        segment_is_safe: Callable[[Vector2, Vector2], bool] | None = None,
    ) -> Vector2:
        position = _vector2("position_m", position_m)
        # Lookahead deliberately commands a point beyond the immediate semantic
        # waypoint to avoid per-corner braking.  Once the character reaches or
        # passes that commanded checkpoint, acknowledge every skipped waypoint;
        # otherwise the cursor can remain forever on a corner it never targeted.
        # A visibility-limited predecessor is behind the cursor and must never
        # be interpreted as forward progress.
        previous_lookahead = self.lookahead_index
        if (
            not self.last_visibility_limited
            and previous_lookahead != self.target_index
            and self._waypoint_passed_at(previous_lookahead, position)
        ):
            self._advance_through(previous_lookahead)
            self.lookahead_checkpoint_advance_count += 1
        # A bounded loop also safely consumes duplicate or very dense points.
        for _ in range(len(self.points)):
            if not self._waypoint_passed(position):
                break
            self._advance()

        target = self.points[self.target_index]
        if segment_is_safe is not None and not segment_is_safe(position, target):
            # BehaviorAgent can carry the animated root laterally beyond the
            # current polyline corner.  Keep the semantic target monotonic,
            # but temporarily steer to the nearest visible predecessor so the
            # agent can rejoin the authored local corridor.  Without this
            # bounded reattachment, an unreachable current target makes the
            # free-space guard return the current position forever.
            reconnect_limit_m = max(
                2.0 * self.route_lookahead_m,
                3.0 * self.waypoint_reach_m,
            )
            reconnect_distance_m = 0.0
            reconnect_index = self.target_index
            for _ in range(len(self.points) - 1):
                previous_index = (reconnect_index - 1) % len(self.points)
                reconnect_distance_m += math.dist(
                    self.points[reconnect_index], self.points[previous_index]
                )
                if reconnect_distance_m > reconnect_limit_m:
                    break
                reconnect_index = previous_index
                if segment_is_safe(position, self.points[reconnect_index]):
                    self.lookahead_index = reconnect_index
                    self.last_visibility_limited = True
                    self.visibility_limited_count += 1
                    return _unit(
                        (
                            self.points[reconnect_index][0] - position[0],
                            self.points[reconnect_index][1] - position[1],
                        )
                    )

        accumulated = math.dist(position, target)
        lookahead_index = self.target_index
        visibility_limited = False
        while accumulated < self.route_lookahead_m:
            next_index = (lookahead_index + 1) % len(self.points)
            next_accumulated = accumulated + math.dist(
                self.points[lookahead_index], self.points[next_index]
            )
            if next_index == self.target_index:
                break
            if segment_is_safe is not None and not segment_is_safe(
                position, self.points[next_index]
            ):
                visibility_limited = True
                break
            accumulated = next_accumulated
            lookahead_index = next_index
        self.lookahead_index = lookahead_index
        self.last_visibility_limited = visibility_limited
        if visibility_limited:
            self.visibility_limited_count += 1
        direction = _unit(
            (
                self.points[lookahead_index][0] - position[0],
                self.points[lookahead_index][1] - position[1],
            )
        )
        if direction == (0.0, 0.0):
            direction = _unit(
                (target[0] - position[0], target[1] - position[1])
            )
        return direction

    def summary(self) -> dict[str, object]:
        return {
            "point_count": len(self.points),
            "target_index": self.target_index,
            "lookahead_index": self.lookahead_index,
            "advance_count": self.advance_count,
            "lap_count": self.lap_count,
            "lookahead_checkpoint_advance_count": (
                self.lookahead_checkpoint_advance_count
            ),
            "visibility_limited_count": self.visibility_limited_count,
            "last_visibility_limited": self.last_visibility_limited,
            "waypoint_reach_m": self.waypoint_reach_m,
            "route_lookahead_m": self.route_lookahead_m,
            "initial_route_window_m": self.initial_route_window_m,
        }
