#!/usr/bin/env python3
"""Minimal deterministic Social Force style motion adapters for Stage 6."""

from __future__ import annotations

import math

import numpy as np


class SocialForceHuman:
    """Goal-seeking human with repulsion from the robot personal space."""

    def __init__(
        self,
        position_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        speed_mps: float = 0.35,
        personal_space_m: float = 0.8,
        repulsion_gain: float = 0.45,
    ) -> None:
        self.position_xy = np.asarray(position_xy, dtype=np.float32)
        self.goal_xy = np.asarray(goal_xy, dtype=np.float32)
        self.velocity_xy = np.zeros(2, dtype=np.float32)
        self.speed_mps = float(speed_mps)
        self.personal_space_m = float(personal_space_m)
        self.repulsion_gain = float(repulsion_gain)

    def step(self, robot_position_xy: np.ndarray, dt: float) -> None:
        dt = float(dt)
        delta = self.goal_xy - self.position_xy
        distance = float(np.linalg.norm(delta))
        desired = np.zeros(2, dtype=float) if distance <= 1e-6 else delta / distance * self.speed_mps
        away = self.position_xy.astype(float) - np.asarray(robot_position_xy, dtype=float)
        robot_distance = float(np.linalg.norm(away))
        repulsion = np.zeros(2, dtype=float)
        if 1e-6 < robot_distance < self.personal_space_m:
            unit = away / robot_distance
            repulsion = unit * self.repulsion_gain * (
                1.0 / robot_distance - 1.0 / self.personal_space_m
            )
        velocity = desired + repulsion
        norm = float(np.linalg.norm(velocity))
        if norm > self.speed_mps and norm > 1e-9:
            velocity *= self.speed_mps / norm
        self.velocity_xy = velocity.astype(np.float32)
        self.position_xy = (self.position_xy + velocity * dt).astype(np.float32)


class SocialNavigationPolicy:
    """Goal controller with short-range personal-space repulsion."""

    def __init__(
        self,
        goal_xy: tuple[float, float],
        *,
        max_linear_mps: float = 0.35,
        influence_radius_m: float = 1.1,
        personal_space_m: float = 0.8,
        repulsion_gain: float = 0.65,
    ) -> None:
        self.goal_xy = np.asarray(goal_xy, dtype=float)
        self.max_linear_mps = float(max_linear_mps)
        self.influence_radius_m = float(influence_radius_m)
        self.personal_space_m = float(personal_space_m)
        self.repulsion_gain = float(repulsion_gain)

    def command(
        self,
        robot_position_xy: np.ndarray,
        robot_yaw: float,
        human_positions_xy: list[np.ndarray],
    ) -> tuple[float, float, float]:
        robot_position_xy = np.asarray(robot_position_xy, dtype=float)
        to_goal = self.goal_xy - robot_position_xy
        goal_distance = float(np.linalg.norm(to_goal))
        desired = np.zeros(2, dtype=float) if goal_distance <= 0.08 else to_goal / max(goal_distance, 1e-9) * self.max_linear_mps
        command_world = desired.copy()
        for human_position in human_positions_xy:
            away = robot_position_xy - np.asarray(human_position, dtype=float)
            distance = float(np.linalg.norm(away))
            if 1e-6 < distance < self.influence_radius_m:
                unit = away / distance
                command_world += unit * self.repulsion_gain * (
                    1.0 / distance - 1.0 / self.influence_radius_m
                )
        norm = float(np.linalg.norm(command_world))
        if norm > self.max_linear_mps and norm > 1e-9:
            command_world *= self.max_linear_mps / norm
        cos_yaw, sin_yaw = math.cos(float(robot_yaw)), math.sin(float(robot_yaw))
        body = np.asarray(
            [cos_yaw * command_world[0] + sin_yaw * command_world[1],
             -sin_yaw * command_world[0] + cos_yaw * command_world[1]],
            dtype=float,
        )
        return float(body[0]), float(body[1]), 0.0
