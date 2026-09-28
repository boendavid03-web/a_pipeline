"""Pure-Python manual-goal controller for the first Isaac5 navigation gate."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass
class NavigationObservation:
    """Stable input envelope for the first planner boundary.

    The manual controller currently uses odom pose and goal only.  LaserScan
    and TF are carried explicitly so a later collision-aware planner can be
    substituted without changing the runtime topic contract.
    """

    odom: dict[str, Any]
    tf: dict[str, Any]
    laser_scan: Any | None
    goal: tuple[float, float, float] | None = None


@dataclass
class ManualGoalController:
    goal_x: float
    goal_y: float
    goal_yaw: float = 0.0
    max_linear: float = 0.35
    max_angular: float = 0.8
    position_tolerance: float = 0.08
    yaw_tolerance: float = 0.08

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        dx, dy = self.goal_x - x, self.goal_y - y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        distance = math.hypot(dx, dy)
        if distance <= self.position_tolerance:
            body_x, body_y = 0.0, 0.0
        else:
            scale = min(1.0, self.max_linear / max(distance, 1e-9))
            body_x *= scale
            body_y *= scale
        yaw_error = self._wrap(self.goal_yaw - yaw)
        wz = 0.0 if abs(yaw_error) <= self.yaw_tolerance else max(
            -self.max_angular, min(self.max_angular, 1.5 * yaw_error)
        )
        return body_x, body_y, wz

    def command_from_observation(self, observation: NavigationObservation) -> tuple[float, float, float]:
        pose = observation.odom["pose"]
        return self.command(float(pose[0]), float(pose[1]), float(pose[2]))

    def reached(self, x: float, y: float, yaw: float) -> bool:
        return math.hypot(self.goal_x - x, self.goal_y - y) <= self.position_tolerance and abs(
            self._wrap(self.goal_yaw - yaw)
        ) <= self.yaw_tolerance
