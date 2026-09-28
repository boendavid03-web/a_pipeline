"""Small, testable Mecanum controller for the Isaac 5 custom experience.

The imported Mecanum730 asset has wheel DOFs, but its wheel collision model is
not a reliable drive base in a renderer-free smoke test.  The controller
therefore records and applies the wheel velocity command and also applies the
same planar command to the articulation root.  This is the same kinematic
control boundary used by the existing Isaac5 backend, kept here as an
explicit, isolated adapter for the custom experience.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
WHEEL_RADIUS_M = 0.077993
WHEEL_BASE_SUM_M = 0.1575 + 0.1725
MAX_LINEAR_MPS = 2.0
MAX_ANGULAR_RADPS = 1.5
ROOT_HEIGHT_M = 0.0


def yaw_from_quaternion(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = [float(value) for value in quat_wxyz]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clamp_twist(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    vx, vy, wz = float(vx), float(vy), float(wz)
    norm = math.hypot(vx, vy)
    if norm > MAX_LINEAR_MPS:
        scale = MAX_LINEAR_MPS / norm
        vx *= scale
        vy *= scale
    return vx, vy, max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, wz))


def mecanum_wheel_velocities(vx: float, vy: float, wz: float) -> np.ndarray:
    """Return wheel rad/s in FL, FR, RL, RR order."""

    vx, vy, wz = clamp_twist(vx, vy, wz)
    return np.asarray(
        [
            (vx - vy - WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx + vy + WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx + vy - WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx - vy + WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
        ],
        dtype=np.float32,
    )


@dataclass(frozen=True)
class WheelCommand:
    body_twist: tuple[float, float, float]
    wheel_names: tuple[str, str, str, str]
    joint_indices: np.ndarray
    wheel_velocity_rad_s: np.ndarray


class MecanumController:
    """Apply a body twist and keep the wheel command contract observable."""

    def __init__(self, robot: object):
        self.robot = robot
        names = list(robot.dof_names)
        missing = [name for name in WHEEL_NAMES if name not in names]
        if missing:
            raise RuntimeError(f"Missing expected wheel joints: {missing}")
        self.joint_names = tuple(names)
        self.wheel_indices = np.asarray([names.index(name) for name in WHEEL_NAMES], dtype=np.int32)
        self.last_command = (0.0, 0.0, 0.0)
        self.last_wheel_command = np.zeros(4, dtype=np.float32)

    def make_command(self, vx: float, vy: float, wz: float) -> WheelCommand:
        body = clamp_twist(vx, vy, wz)
        wheels = mecanum_wheel_velocities(*body)
        return WheelCommand(body, WHEEL_NAMES, self.wheel_indices.copy(), wheels)

    def apply(self, vx: float, vy: float, wz: float) -> WheelCommand:
        """Send wheel velocity targets and the equivalent root planar velocity."""

        command = self.make_command(vx, vy, wz)
        from isaacsim.core.utils.types import ArticulationAction

        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_velocities=command.wheel_velocity_rad_s,
                joint_indices=command.joint_indices,
            )
        )

        position, orientation = self.robot.get_world_pose()
        yaw = yaw_from_quaternion(np.asarray(orientation, dtype=float))
        vx_body, vy_body, wz_body = command.body_twist
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        world_velocity = np.asarray(
            [cos_yaw * vx_body - sin_yaw * vy_body, sin_yaw * vx_body + cos_yaw * vy_body, 0.0],
            dtype=float,
        )
        position = np.asarray(position, dtype=float)
        vertical = float(np.clip(8.0 * (ROOT_HEIGHT_M - position[2]), -0.35, 0.35))
        self.robot.set_linear_velocity(np.asarray([world_velocity[0], world_velocity[1], vertical]))
        self.robot.set_angular_velocity(np.asarray([0.0, 0.0, wz_body], dtype=float))
        self.last_command = command.body_twist
        self.last_wheel_command = command.wheel_velocity_rad_s.copy()
        return command

    def wheel_state(self) -> dict[str, list[float]]:
        positions = np.asarray(self.robot.get_joint_positions(), dtype=float)
        velocities = np.asarray(self.robot.get_joint_velocities(), dtype=float)
        return {
            "names": list(WHEEL_NAMES),
            "indices": self.wheel_indices.tolist(),
            "positions": positions[self.wheel_indices].tolist(),
            "velocities": velocities[self.wheel_indices].tolist(),
            "command_rad_s": self.last_wheel_command.tolist(),
        }


def yaw_delta(start: np.ndarray, end: np.ndarray) -> float:
    delta = yaw_from_quaternion(end) - yaw_from_quaternion(start)
    return math.atan2(math.sin(delta), math.cos(delta))
