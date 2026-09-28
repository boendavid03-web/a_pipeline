#!/usr/bin/env python3
"""Semantic waypoint and replaceable local-corner state for HuNav execution."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Sequence


Point3 = tuple[float, float, float]
Planner = Callable[[Sequence[float], Sequence[float]], Sequence[Sequence[float]] | None]


def _point3(value: Sequence[float]) -> Point3:
    if len(value) < 2:
        raise ValueError("route point needs at least x and y")
    point = (float(value[0]), float(value[1]), float(value[2]) if len(value) > 2 else 0.0)
    if not all(math.isfinite(component) for component in point):
        raise ValueError("route points must be finite")
    return point


@dataclass
class SemanticRouteRuntime:
    semantic_route: tuple[Point3, ...]
    goal_radius_m: float = 0.30
    cyclic: bool = True
    semantic_index: int = 0
    local_corners: tuple[Point3, ...] = ()
    local_index: int = 0
    semantic_progress: int = 0
    resolved_route_progress: int = 0
    loop_count: int = 0
    replan_count: int = 0
    replan_failure_count: int = 0
    skipped_blocked_corner_count: int = 0
    last_progress_time: float = 0.0
    progress_m: float = 0.0
    _last_position: Point3 | None = field(default=None, repr=False)
    _last_target_distance: float | None = field(default=None, repr=False)
    _replanned_segment: bool = field(default=False, repr=False)

    def __post_init__(self) -> None:
        self.semantic_route = tuple(_point3(point) for point in self.semantic_route)
        if not self.semantic_route:
            raise ValueError("semantic route must be non-empty")
        if not math.isfinite(self.goal_radius_m) or self.goal_radius_m <= 0.0:
            raise ValueError("goal radius must be positive and finite")

    @property
    def semantic_goal(self) -> Point3:
        return self.semantic_route[self.semantic_index]

    @property
    def route_direction(self) -> int:
        return 1

    def _install(
        self,
        position: Sequence[float],
        corners: Sequence[Sequence[float]],
        *,
        replanned_segment: bool,
    ) -> bool:
        resolved = tuple(_point3(point) for point in corners)
        if not resolved:
            return False
        current = _point3(position)
        if math.dist(current[:2], resolved[0][:2]) > 0.05:
            resolved = (current, *resolved)
        if len(resolved) < 2:
            resolved = (current, self.semantic_goal)
        self.local_corners = resolved
        self.local_index = 1
        self._last_target_distance = None
        self._replanned_segment = replanned_segment
        return True

    def initialize(self, position: Sequence[float], planner: Planner) -> bool:
        corners = planner(_point3(position), self.semantic_goal)
        return corners is not None and self._install(
            position, corners, replanned_segment=True
        )

    def initialize_resolved(
        self, position: Sequence[float], corners: Sequence[Sequence[float]]
    ) -> bool:
        """Install the generator's already validated cyclic local route."""
        return self._install(position, corners, replanned_segment=False)

    def replan(self, position: Sequence[float], planner: Planner) -> bool:
        self.replan_count += 1
        corners = planner(_point3(position), self.semantic_goal)
        if corners is None or not self._install(
            position, corners, replanned_segment=True
        ):
            self.replan_failure_count += 1
            return False
        return True

    def skip_blocked_local_corner(self) -> bool:
        """Advance one dense local corner after a validated replan failure.

        This changes only the route cursor.  It never changes the agent pose
        and never skips the semantic destination, so the next state frame
        remains authoritative for both HuNav and the execution controller.
        """
        if not self.local_corners or self.local_index + 1 >= len(self.local_corners):
            return False
        self.local_index += 1
        self.resolved_route_progress += 1
        self.skipped_blocked_corner_count += 1
        self._last_target_distance = None
        return True

    def _advance_semantic(self) -> bool:
        if self.semantic_index + 1 < len(self.semantic_route):
            self.semantic_index += 1
        elif self.cyclic:
            self.semantic_index = 0
            self.loop_count += 1
        else:
            return False
        self.semantic_progress += 1
        return True

    def update(self, position: Sequence[float], sim_time: float, planner: Planner) -> None:
        current = _point3(position)
        if not self.local_corners:
            self.initialize(current, planner)
        if not self.local_corners:
            return

        if math.dist(current[:2], self.semantic_goal[:2]) <= self.goal_radius_m:
            self._advance_semantic()

        target = self.local_corners[self.local_index]
        target_distance = math.dist(current[:2], target[:2])
        if self._last_target_distance is not None:
            gained = max(0.0, self._last_target_distance - target_distance)
            if gained > 1.0e-4:
                self.progress_m += gained
                self.last_progress_time = float(sim_time)
        self._last_target_distance = target_distance
        self._last_position = current

        while target_distance <= self.goal_radius_m:
            self.resolved_route_progress += 1
            if self.local_index + 1 < len(self.local_corners):
                self.local_index += 1
                target = self.local_corners[self.local_index]
                target_distance = math.dist(current[:2], target[:2])
                self._last_target_distance = target_distance
                continue
            if not self._replanned_segment and self.cyclic:
                self.local_index = 0
                self.loop_count += 1
                target = self.local_corners[self.local_index]
                target_distance = math.dist(current[:2], target[:2])
                self._last_target_distance = target_distance
                break
            corners = planner(current, self.semantic_goal)
            if corners is None or not self._install(
                current, corners, replanned_segment=True
            ):
                self.replan_failure_count += 1
                self.local_corners = ()
                return
            target = self.local_corners[self.local_index]
            target_distance = math.dist(current[:2], target[:2])

    def goals_xy(self) -> list[list[float]]:
        if not self.local_corners:
            return [[self.semantic_goal[0], self.semantic_goal[1]]]
        return [
            [point[0], point[1]]
            for point in self.local_corners[self.local_index :]
        ] or [[self.semantic_goal[0], self.semantic_goal[1]]]

    def snapshot(self) -> dict[str, object]:
        return {
            "semantic_waypoint_index": self.semantic_index,
            "semantic_progress": self.semantic_progress,
            "resolved_route_progress": self.resolved_route_progress,
            "local_corner_index": self.local_index,
            "route_direction": self.route_direction,
            "cyclic": self.cyclic,
            "loop_count": self.loop_count,
            "last_progress_time": self.last_progress_time,
            "progress_m": self.progress_m,
            "replan_count": self.replan_count,
            "replan_failure_count": self.replan_failure_count,
            "skipped_blocked_corner_count": self.skipped_blocked_corner_count,
        }
