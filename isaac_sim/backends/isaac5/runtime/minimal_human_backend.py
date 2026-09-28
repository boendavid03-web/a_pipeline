"""Minimal kinematic human source for the DRL-VO input-contract gate."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MinimalHumanState:
    track_id: int
    position_xy_map: np.ndarray
    velocity_xy_map_absolute: np.ndarray
    goal_xy_map: np.ndarray
    speed_mps: float
    track_state: str = "CONFIRMED"
    confidence: float = 1.0
    time_since_update_s: float = 0.0


class MinimalHumanBackend:
    """Move one or more humans directly toward fixed goals.

    This is deliberately not a pedestrian behavior system. It only provides
    finite position, velocity, goal, state and confidence fields needed to
    exercise the existing DRL-VO pedestrian-map converter.
    """

    def __init__(self, humans: list[MinimalHumanState]):
        if not humans:
            raise ValueError("at least one minimal human is required")
        self.humans = humans

    def step(self, dt: float) -> None:
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        for human in self.humans:
            delta = np.asarray(human.goal_xy_map - human.position_xy_map, dtype=float)
            distance = float(np.linalg.norm(delta))
            if distance <= 1.0e-6:
                human.velocity_xy_map_absolute = np.zeros(2, dtype=np.float32)
                human.time_since_update_s = 0.0
                continue
            direction = delta / distance
            step_distance = min(float(human.speed_mps) * dt, distance)
            human.velocity_xy_map_absolute = (
                direction * (step_distance / dt)
            ).astype(np.float32)
            human.position_xy_map = (
                human.position_xy_map + direction * step_distance
            ).astype(np.float32)
            human.time_since_update_s = 0.0

    def tracks(self) -> list[MinimalHumanState]:
        return list(self.humans)

    def relative_observations(
        self, robot_position_xy: np.ndarray, robot_yaw: float
    ) -> list[dict[str, object]]:
        """Return finite ground-truth human observations in ``base_link``.

        This is an environment observation for Stage 5, not a LiDAR detector
        or tracker.  Perception remains a later Stage 7 concern.
        """

        robot_position_xy = np.asarray(robot_position_xy, dtype=float).reshape(-1)
        if robot_position_xy.shape != (2,) or not np.isfinite(robot_position_xy).all():
            raise ValueError("robot position must be a finite two-dimensional vector")
        cos_yaw, sin_yaw = np.cos(float(robot_yaw)), np.sin(float(robot_yaw))
        observations: list[dict[str, object]] = []
        for human in self.humans:
            delta = np.asarray(human.position_xy_map, dtype=float) - robot_position_xy
            relative_position = np.asarray(
                [cos_yaw * delta[0] + sin_yaw * delta[1], -sin_yaw * delta[0] + cos_yaw * delta[1]],
                dtype=np.float32,
            )
            velocity = np.asarray(human.velocity_xy_map_absolute, dtype=float)
            relative_velocity = np.asarray(
                [cos_yaw * velocity[0] + sin_yaw * velocity[1], -sin_yaw * velocity[0] + cos_yaw * velocity[1]],
                dtype=np.float32,
            )
            observations.append(
                {
                    "track_id": int(human.track_id),
                    "position_xy_map": np.asarray(human.position_xy_map, dtype=np.float32).copy(),
                    "velocity_xy_map_absolute": np.asarray(human.velocity_xy_map_absolute, dtype=np.float32).copy(),
                    "relative_position_xy_base": relative_position,
                    "relative_velocity_xy_base": relative_velocity,
                    "goal_xy_map": np.asarray(human.goal_xy_map, dtype=np.float32).copy(),
                    "confidence": float(human.confidence),
                }
            )
        return observations
