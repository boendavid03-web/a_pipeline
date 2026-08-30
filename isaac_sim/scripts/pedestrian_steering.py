"""Pure geometry for continuous BehaviorAgent social steering.

Isaac Sim owns NavMesh legality, locomotion, and animation.  This module only
maps a complete planar desired velocity to a continuously moving local target
and keeps the underlying patrol polyline progressing without per-waypoint
BehaviorAgent task replacement.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


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


class PatrolPolylineCursor:
    """Track a cyclic patrol while exposing a smooth forward route intent."""

    def __init__(
        self,
        points_m: Sequence[Sequence[float]],
        initial_position_m: Sequence[float],
        *,
        waypoint_reach_m: float,
        route_lookahead_m: float,
    ) -> None:
        self.points = tuple(_vector2("patrol point", point) for point in points_m)
        if len(self.points) < 2:
            raise ValueError("a patrol polyline requires at least two points")
        self.waypoint_reach_m = _positive("waypoint_reach_m", waypoint_reach_m)
        self.route_lookahead_m = _positive("route_lookahead_m", route_lookahead_m)
        initial = _vector2("initial_position_m", initial_position_m)
        closest = min(
            range(len(self.points)),
            key=lambda index: math.dist(initial, self.points[index]),
        )
        self.target_index = (closest + 1) % len(self.points)
        self.advance_count = 0
        self.lap_count = 0

    def _advance(self) -> None:
        previous = self.target_index
        self.target_index = (self.target_index + 1) % len(self.points)
        self.advance_count += 1
        if self.target_index <= previous:
            self.lap_count += 1

    def _waypoint_passed(self, position: Vector2) -> bool:
        target = self.points[self.target_index]
        if math.dist(position, target) <= self.waypoint_reach_m:
            return True
        previous = self.points[(self.target_index - 1) % len(self.points)]
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

    def desired_direction(self, position_m: Sequence[float]) -> Vector2:
        position = _vector2("position_m", position_m)
        # A bounded loop also safely consumes duplicate or very dense points.
        for _ in range(len(self.points)):
            if not self._waypoint_passed(position):
                break
            self._advance()

        target = self.points[self.target_index]
        accumulated = math.dist(position, target)
        lookahead_index = self.target_index
        while accumulated < self.route_lookahead_m:
            next_index = (lookahead_index + 1) % len(self.points)
            accumulated += math.dist(
                self.points[lookahead_index], self.points[next_index]
            )
            lookahead_index = next_index
            if lookahead_index == self.target_index:
                break
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
            "advance_count": self.advance_count,
            "lap_count": self.lap_count,
            "waypoint_reach_m": self.waypoint_reach_m,
            "route_lookahead_m": self.route_lookahead_m,
        }
