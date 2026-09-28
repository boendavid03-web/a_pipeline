"""Model-neutral Isaac5 navigation policy boundary.

Stage 1 intentionally uses a manual-goal policy.  Real DRL-VO and SemanticCNN
models can be attached later through ``predict`` without changing the Isaac5
sensor or controller contracts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navigation_interface import ManualGoalController, NavigationObservation


@dataclass(frozen=True)
class PolicyCommand:
    vx: float
    vy: float
    omega: float

    def as_tuple(self) -> tuple[float, float, float]:
        return self.vx, self.vy, self.omega


class NavigationPolicy(Protocol):
    def predict(self, observation: NavigationObservation) -> PolicyCommand:
        ...


def validate_observation(observation: NavigationObservation) -> None:
    if observation.goal is None:
        raise ValueError("policy observation requires goal=(x, y, yaw)")
    goal = np.asarray(observation.goal, dtype=float).reshape(-1)
    if goal.shape != (3,) or not np.isfinite(goal).all():
        raise ValueError("goal must be a finite [x, y, yaw] vector")
    pose = np.asarray(observation.odom.get("pose"), dtype=float).reshape(-1)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("odom pose must be a finite [x, y, yaw] vector")
    if not isinstance(observation.tf, dict):
        raise ValueError("tf must be a mapping")


class ManualGoalPolicy:
    """Dummy policy used only to validate the Stage 1 adapter contract."""

    def __init__(self, max_linear: float = 0.35, max_angular: float = 0.8):
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)

    def predict(self, observation: NavigationObservation) -> PolicyCommand:
        validate_observation(observation)
        goal_x, goal_y, goal_yaw = [float(value) for value in observation.goal]
        controller = ManualGoalController(
            goal_x,
            goal_y,
            goal_yaw,
            max_linear=self.max_linear,
            max_angular=self.max_angular,
        )
        x, y, yaw = [float(value) for value in observation.odom["pose"]]
        command = controller.command(x, y, yaw)
        if not np.isfinite(np.asarray(command, dtype=float)).all():
            raise ValueError("policy command contains NaN or Inf")
        return PolicyCommand(*command)


class PolicyAdapter:
    """Adapt Isaac5 observations to a model policy and standard cmd_vel."""

    def __init__(self, policy: NavigationPolicy):
        self.policy = policy
        self.calls = 0

    def predict(self, observation: NavigationObservation) -> PolicyCommand:
        validate_observation(observation)
        command = self.policy.predict(observation)
        if not isinstance(command, PolicyCommand):
            raise TypeError("policy must return PolicyCommand")
        values = np.asarray(command.as_tuple(), dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            raise ValueError("policy command must be a finite [vx, vy, omega]")
        self.calls += 1
        return command

    def cmd_vel(self, observation: NavigationObservation) -> tuple[float, float, float]:
        return self.predict(observation).as_tuple()
