#!/usr/bin/env python3
"""Small episode contract for the Isaac5 navigation backend.

The collision check is an explicit footprint-vs-AABB geometric guard.  It is
useful for episode termination and reset validation, but it is not a claim of
measured physical contact semantics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ObstacleBox:
    name: str
    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float]


@dataclass
class EpisodeRecord:
    episode_id: int
    start_xy: tuple[float, float]
    goal_xy: tuple[float, float]
    status: str = "RUNNING"
    steps: int = 0
    sim_time_s: float = 0.0
    goal_distance_m: float = math.inf
    collision_obstacle: str | None = None
    failure_reason: str | None = None


class NavigationEpisodeEnvironment:
    """Manage reset, goal, timeout, and geometric collision state."""

    def __init__(
        self,
        goals: Iterable[tuple[float, float]],
        *,
        position_tolerance: float = 0.08,
        footprint_half_extents: tuple[float, float] = (0.36, 0.32),
        max_episode_time_s: float = 5.0,
        obstacles: Iterable[ObstacleBox] = (),
    ) -> None:
        self.goals = tuple((float(x), float(y)) for x, y in goals)
        if not self.goals:
            raise ValueError("at least one goal is required")
        if position_tolerance <= 0.0 or max_episode_time_s <= 0.0:
            raise ValueError("episode tolerances must be positive")
        self.position_tolerance = float(position_tolerance)
        self.footprint_half_extents = tuple(float(v) for v in footprint_half_extents)
        self.max_episode_time_s = float(max_episode_time_s)
        self.obstacles = tuple(obstacles)
        self.records: list[EpisodeRecord] = []
        self.current: EpisodeRecord | None = None

    def reset_episode(self, world: object, robot: object, start_pose: tuple[float, float, float]) -> EpisodeRecord:
        """Reset the simulation and return a new RUNNING episode record."""

        x, y, z = (float(value) for value in start_pose)
        world.reset()
        robot.set_world_pose(
            np.asarray([x, y, z], dtype=float),
            np.asarray([1.0, 0.0, 0.0, 0.0], dtype=float),
        )
        robot.set_linear_velocity(np.zeros(3, dtype=float))
        robot.set_angular_velocity(np.zeros(3, dtype=float))
        episode_id = len(self.records) + 1
        goal = self.goals[episode_id - 1]
        self.current = EpisodeRecord(
            episode_id=episode_id,
            start_xy=(x, y),
            goal_xy=goal,
        )
        self.records.append(self.current)
        return self.current

    def collision_obstacle(self, position_xy: Iterable[float]) -> str | None:
        """Return the first AABB intersecting the robot footprint, if any."""

        x, y = (float(value) for value in position_xy)
        hx, hy = self.footprint_half_extents
        for obstacle in self.obstacles:
            cx, cy = obstacle.center_xy
            ox, oy = obstacle.half_extents_xy
            if abs(x - cx) <= hx + ox and abs(y - cy) <= hy + oy:
                return obstacle.name
        return None

    def observe(self, position_xy: Iterable[float], sim_time_s: float) -> EpisodeRecord:
        if self.current is None:
            raise RuntimeError("episode has not been reset")
        record = self.current
        x, y = (float(value) for value in position_xy)
        record.steps += 1
        record.sim_time_s = float(sim_time_s)
        record.goal_distance_m = math.hypot(record.goal_xy[0] - x, record.goal_xy[1] - y)
        obstacle = self.collision_obstacle((x, y))
        if obstacle is not None:
            record.status = "FAILURE"
            record.collision_obstacle = obstacle
            record.failure_reason = "footprint_aabb_collision"
        elif record.goal_distance_m <= self.position_tolerance:
            record.status = "SUCCESS"
        elif record.sim_time_s >= self.max_episode_time_s:
            record.status = "FAILURE"
            record.failure_reason = "timeout"
        return record

    @property
    def done(self) -> bool:
        return self.current is not None and self.current.status != "RUNNING"
