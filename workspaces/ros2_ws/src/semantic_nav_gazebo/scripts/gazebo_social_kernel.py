"""Pure Gazebo social-interaction mathematics shared by simulator adapters.

This module intentionally contains no ROS, Gazebo, or Isaac integration.  Route
selection, robot-state freshness, obstacle forces, and position integration stay
with the calling adapter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


Vector2 = tuple[float, float]
ZERO_VECTOR: Vector2 = (0.0, 0.0)


@dataclass(frozen=True)
class KinematicState:
    position: Vector2
    velocity: Vector2


@dataclass(frozen=True)
class SocialInteraction:
    direction_to_neighbor: Vector2
    interaction_direction: Vector2
    theta: float
    force: Vector2


@dataclass(frozen=True)
class CoreAcceleration:
    desired_acceleration: Vector2
    desired_weighted: Vector2
    human_social_raw: Vector2
    robot_social_raw: Vector2
    social_raw: Vector2
    human_social_weighted: Vector2
    robot_social_weighted: Vector2
    social_weighted: Vector2
    robot_personal_space_raw: Vector2
    robot_personal_space_weighted: Vector2
    core_acceleration: Vector2


@dataclass(frozen=True)
class VelocityStep:
    pre_clip_velocity: Vector2
    post_clip_velocity: Vector2


def desired_acceleration(
    current_velocity: Vector2,
    desired_direction: Vector2,
    vmax: float,
    relaxation_time: float,
) -> Vector2:
    """Return the current Gazebo desired-velocity relaxation acceleration."""
    return (
        (desired_direction[0] * vmax - current_velocity[0]) / relaxation_time,
        (desired_direction[1] * vmax - current_velocity[1]) / relaxation_time,
    )


def social_force_from_neighbor(
    position: Vector2,
    velocity: Vector2,
    neighbor_position: Vector2,
    neighbor_velocity: Vector2,
    neighbor_range: float,
) -> SocialInteraction:
    """Return the Gazebo interaction model for one human or robot center."""
    lambda_importance = 2.0
    gamma = 0.35
    n = 2.0
    n_prime = 3.0

    diff_x = neighbor_position[0] - position[0]
    diff_y = neighbor_position[1] - position[1]
    distance = math.hypot(diff_x, diff_y)
    if distance < 1e-6 or distance > neighbor_range:
        return SocialInteraction(ZERO_VECTOR, ZERO_VECTOR, 0.0, ZERO_VECTOR)

    diff_dir_x = diff_x / distance
    diff_dir_y = diff_y / distance
    vel_diff_x = velocity[0] - neighbor_velocity[0]
    vel_diff_y = velocity[1] - neighbor_velocity[1]
    interaction_x = lambda_importance * vel_diff_x + diff_dir_x
    interaction_y = lambda_importance * vel_diff_y + diff_dir_y
    interaction_len = math.hypot(interaction_x, interaction_y)
    if interaction_len < 1e-6:
        return SocialInteraction(
            (diff_dir_x, diff_dir_y), ZERO_VECTOR, 0.0, ZERO_VECTOR
        )

    interaction_dir_x = interaction_x / interaction_len
    interaction_dir_y = interaction_y / interaction_len
    theta = _angle_between(
        interaction_dir_x, interaction_dir_y, diff_dir_x, diff_dir_y
    )
    b = max(1e-6, gamma * interaction_len)

    force_velocity_amount = -math.exp(
        -distance / b - (n_prime * b * theta) * (n_prime * b * theta)
    )
    force_angle_amount = -_sign(theta) * math.exp(
        -distance / b - (n * b * theta) * (n * b * theta)
    )
    force = (
        force_velocity_amount * interaction_dir_x
        + force_angle_amount * (-interaction_dir_y),
        force_velocity_amount * interaction_dir_y
        + force_angle_amount * interaction_dir_x,
    )
    return SocialInteraction(
        (diff_dir_x, diff_dir_y),
        (interaction_dir_x, interaction_dir_y),
        theta,
        force,
    )


def robot_personal_space_force(
    pedestrian_position: Vector2,
    pedestrian_yaw: float,
    robot_position: Vector2,
    robot_clearance: float,
    sigma_robot_personal_space: float,
) -> Vector2:
    """Return Gazebo's robot-center personal-space repulsion."""
    diff_x = pedestrian_position[0] - robot_position[0]
    diff_y = pedestrian_position[1] - robot_position[1]
    distance = math.hypot(diff_x, diff_y)
    if distance < 1e-6:
        direction_x = -math.cos(pedestrian_yaw)
        direction_y = -math.sin(pedestrian_yaw)
    else:
        direction_x = diff_x / distance
        direction_y = diff_y / distance

    clearance = distance - robot_clearance
    force_amount = math.exp(-clearance / sigma_robot_personal_space)
    return force_amount * direction_x, force_amount * direction_y


def compute_social_interaction_core(
    *,
    pedestrian_position: Vector2,
    pedestrian_yaw: float,
    pedestrian_velocity: Vector2,
    desired_direction: Vector2,
    vmax: float,
    relaxation_time: float,
    desired_force_factor: float,
    human_neighbors: Iterable[KinematicState],
    robot_state: KinematicState | None,
    neighbor_range: float,
    force_social: float,
    robot_clearance: float,
    sigma_robot_personal_space: float,
    force_robot_personal_space: float,
) -> CoreAcceleration:
    """Decompose desired, human, and robot contributions to core acceleration."""
    desired_acceleration_value = desired_acceleration(
        pedestrian_velocity, desired_direction, vmax, relaxation_time
    )
    human_social_x = 0.0
    human_social_y = 0.0
    for neighbor in human_neighbors:
        interaction = social_force_from_neighbor(
            pedestrian_position,
            pedestrian_velocity,
            neighbor.position,
            neighbor.velocity,
            neighbor_range,
        )
        human_social_x += interaction.force[0]
        human_social_y += interaction.force[1]

    robot_social = ZERO_VECTOR
    robot_personal = ZERO_VECTOR
    if robot_state is not None:
        robot_social = social_force_from_neighbor(
            pedestrian_position,
            pedestrian_velocity,
            robot_state.position,
            robot_state.velocity,
            neighbor_range,
        ).force
        robot_personal = robot_personal_space_force(
            pedestrian_position,
            pedestrian_yaw,
            robot_state.position,
            robot_clearance,
            sigma_robot_personal_space,
        )

    human_social = (human_social_x, human_social_y)
    social_raw = (
        human_social[0] + robot_social[0],
        human_social[1] + robot_social[1],
    )
    desired_weighted = _scale(desired_acceleration_value, desired_force_factor)
    human_social_weighted = _scale(human_social, force_social)
    robot_social_weighted = _scale(robot_social, force_social)
    # Weight the already-summed raw social force to preserve Gazebo's original
    # floating-point operation order.
    social_weighted = _scale(social_raw, force_social)
    robot_personal_weighted = _scale(
        robot_personal, force_robot_personal_space
    )
    core_acceleration = (
        desired_weighted[0] + social_weighted[0] + robot_personal_weighted[0],
        desired_weighted[1] + social_weighted[1] + robot_personal_weighted[1],
    )
    return CoreAcceleration(
        desired_acceleration=desired_acceleration_value,
        desired_weighted=desired_weighted,
        human_social_raw=human_social,
        robot_social_raw=robot_social,
        social_raw=social_raw,
        human_social_weighted=human_social_weighted,
        robot_social_weighted=robot_social_weighted,
        social_weighted=social_weighted,
        robot_personal_space_raw=robot_personal,
        robot_personal_space_weighted=robot_personal_weighted,
        core_acceleration=core_acceleration,
    )


def velocity_step(
    current_velocity: Vector2,
    acceleration: Vector2,
    dt: float,
    vmax: float,
) -> VelocityStep:
    """Integrate velocity once and apply Gazebo's magnitude-based vmax cap."""
    pre_clip_x = current_velocity[0] + dt * acceleration[0]
    pre_clip_y = current_velocity[1] + dt * acceleration[1]
    speed = math.hypot(pre_clip_x, pre_clip_y)
    if speed > vmax:
        post_clip = (
            pre_clip_x / speed * vmax,
            pre_clip_y / speed * vmax,
        )
    else:
        post_clip = (pre_clip_x, pre_clip_y)
    return VelocityStep((pre_clip_x, pre_clip_y), post_clip)


def _scale(vector: Vector2, factor: float) -> Vector2:
    return factor * vector[0], factor * vector[1]


def _angle_between(ax: float, ay: float, bx: float, by: float) -> float:
    return _normalize_angle(math.atan2(by, bx) - math.atan2(ay, ax))


def _normalize_angle(value: float) -> float:
    while value <= -math.pi:
        value += 2.0 * math.pi
    while value > math.pi:
        value -= 2.0 * math.pi
    return value


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0
