"""Regression of the shared kernel against the pre-extraction Gazebo math."""

import ast
import math
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from gazebo_social_kernel import (  # noqa: E402
    KinematicState,
    compute_social_interaction_core,
    desired_acceleration,
    robot_personal_space_force,
    social_force_from_neighbor,
    velocity_step,
)


ABS_TOL = 1e-12


def _assert_scalar_matches(old, new):
    error = abs(old - new)
    assert error <= ABS_TOL
    return error


def _assert_vector_matches(old, new):
    errors = [abs(old[index] - new[index]) for index in range(2)]
    assert max(errors) <= ABS_TOL
    return max(errors)


# Frozen characterization helpers copied from the unmodified controller object
# 5347a03197ccbeea3497b209eb00595675d7e46a before extraction.  They remain
# independent of the production kernel so these tests compare old math to new.
def _legacy_desired_force(velocity, desired_direction, vmax, relaxation_time):
    return (
        (desired_direction[0] * vmax - velocity[0]) / relaxation_time,
        (desired_direction[1] * vmax - velocity[1]) / relaxation_time,
    )


def _legacy_social_force(position, velocity, neighbor, neighbor_range):
    lambda_importance = 2.0
    gamma = 0.35
    n = 2.0
    n_prime = 3.0

    diff_x = neighbor.position[0] - position[0]
    diff_y = neighbor.position[1] - position[1]
    distance = math.hypot(diff_x, diff_y)
    if distance < 1e-6 or distance > neighbor_range:
        return (0.0, 0.0), (0.0, 0.0), 0.0, (0.0, 0.0)

    diff_direction = (diff_x / distance, diff_y / distance)
    interaction_x = (
        lambda_importance * (velocity[0] - neighbor.velocity[0])
        + diff_direction[0]
    )
    interaction_y = (
        lambda_importance * (velocity[1] - neighbor.velocity[1])
        + diff_direction[1]
    )
    interaction_len = math.hypot(interaction_x, interaction_y)
    if interaction_len < 1e-6:
        return diff_direction, (0.0, 0.0), 0.0, (0.0, 0.0)

    interaction_direction = (
        interaction_x / interaction_len,
        interaction_y / interaction_len,
    )
    theta = _legacy_normalize_angle(
        math.atan2(diff_direction[1], diff_direction[0])
        - math.atan2(interaction_direction[1], interaction_direction[0])
    )
    b = max(1e-6, gamma * interaction_len)
    force_velocity_amount = -math.exp(
        -distance / b - (n_prime * b * theta) * (n_prime * b * theta)
    )
    force_angle_amount = -_legacy_sign(theta) * math.exp(
        -distance / b - (n * b * theta) * (n * b * theta)
    )
    force = (
        force_velocity_amount * interaction_direction[0]
        + force_angle_amount * (-interaction_direction[1]),
        force_velocity_amount * interaction_direction[1]
        + force_angle_amount * interaction_direction[0],
    )
    return diff_direction, interaction_direction, theta, force


def _legacy_robot_personal_space(
    pedestrian_position,
    pedestrian_yaw,
    robot_position,
    robot_clearance,
    sigma,
):
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
    force_amount = math.exp(-clearance / sigma)
    return force_amount * direction_x, force_amount * direction_y


def _legacy_velocity_step(velocity, acceleration, dt, vmax):
    pre_clip = (
        velocity[0] + dt * acceleration[0],
        velocity[1] + dt * acceleration[1],
    )
    speed = math.hypot(pre_clip[0], pre_clip[1])
    if speed > vmax:
        post_clip = (
            pre_clip[0] / speed * vmax,
            pre_clip[1] / speed * vmax,
        )
    else:
        post_clip = pre_clip
    return pre_clip, post_clip


def _legacy_normalize_angle(value):
    while value <= -math.pi:
        value += 2.0 * math.pi
    while value > math.pi:
        value -= 2.0 * math.pi
    return value


def _legacy_sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def test_single_pedestrian_desired_and_velocity_regression():
    velocity = (0.15, -0.05)
    desired_direction = (0.6, 0.8)
    vmax = 0.8
    relaxation_time = 0.5
    desired_force_factor = 0.5
    dt = 0.1

    old_desired = _legacy_desired_force(
        velocity, desired_direction, vmax, relaxation_time
    )
    new_desired = desired_acceleration(
        velocity, desired_direction, vmax, relaxation_time
    )
    max_error = _assert_vector_matches(old_desired, new_desired)

    old_acceleration = (
        desired_force_factor * old_desired[0],
        desired_force_factor * old_desired[1],
    )
    core = compute_social_interaction_core(
        pedestrian_position=(2.0, -1.0),
        pedestrian_yaw=0.0,
        pedestrian_velocity=velocity,
        desired_direction=desired_direction,
        vmax=vmax,
        relaxation_time=relaxation_time,
        desired_force_factor=desired_force_factor,
        human_neighbors=(),
        robot_state=None,
        neighbor_range=10.0,
        force_social=5.1,
        robot_clearance=1.0,
        sigma_robot_personal_space=0.2,
        force_robot_personal_space=6.0,
    )
    max_error = max(
        max_error,
        _assert_vector_matches(old_acceleration, core.core_acceleration),
    )
    old_pre_clip, old_post_clip = _legacy_velocity_step(
        velocity, old_acceleration, dt, vmax
    )
    new_step = velocity_step(velocity, core.core_acceleration, dt, vmax)
    max_error = max(
        max_error,
        _assert_vector_matches(old_pre_clip, new_step.pre_clip_velocity),
        _assert_vector_matches(old_post_clip, new_step.post_clip_velocity),
    )
    print(f"Test 1 max_abs_error={max_error:.17g}")


def test_two_pedestrian_head_on_regression():
    position = (-1.0, 0.0)
    velocity = (0.6, 0.0)
    neighbor = KinematicState(position=(1.0, 0.0), velocity=(-0.6, 0.0))
    desired = _legacy_desired_force(velocity, (1.0, 0.0), 0.8, 0.5)
    old_direction, old_interaction, old_theta, old_force = _legacy_social_force(
        position, velocity, neighbor, 10.0
    )
    interaction = social_force_from_neighbor(
        position, velocity, neighbor.position, neighbor.velocity, 10.0
    )

    max_error = max(
        _assert_vector_matches(old_direction, interaction.direction_to_neighbor),
        _assert_vector_matches(old_interaction, interaction.interaction_direction),
        _assert_scalar_matches(old_theta, interaction.theta),
        _assert_vector_matches(old_force, interaction.force),
    )
    assert interaction.theta == 0.0
    assert interaction.force[1] == 0.0

    force_social = 5.1
    old_weighted_social = (
        force_social * old_force[0],
        force_social * old_force[1],
    )
    old_core = (
        desired[0] + old_weighted_social[0],
        desired[1] + old_weighted_social[1],
    )
    core = compute_social_interaction_core(
        pedestrian_position=position,
        pedestrian_yaw=0.0,
        pedestrian_velocity=velocity,
        desired_direction=(1.0, 0.0),
        vmax=0.8,
        relaxation_time=0.5,
        desired_force_factor=1.0,
        human_neighbors=(neighbor,),
        robot_state=None,
        neighbor_range=10.0,
        force_social=force_social,
        robot_clearance=1.0,
        sigma_robot_personal_space=0.2,
        force_robot_personal_space=6.0,
    )
    old_pre_clip, old_post_clip = _legacy_velocity_step(
        velocity, old_core, 0.1, 0.8
    )
    new_step = velocity_step(velocity, core.core_acceleration, 0.1, 0.8)
    max_error = max(
        max_error,
        _assert_vector_matches(old_force, core.human_social_raw),
        _assert_vector_matches(old_weighted_social, core.human_social_weighted),
        _assert_vector_matches(old_core, core.core_acceleration),
        _assert_vector_matches(old_pre_clip, new_step.pre_clip_velocity),
        _assert_vector_matches(old_post_clip, new_step.post_clip_velocity),
    )
    print(f"Test 2 max_abs_error={max_error:.17g}")


def test_pedestrian_robot_center_regression():
    position = (0.0, 0.0)
    velocity = (0.2, -0.05)
    yaw = 0.4
    robot = KinematicState(position=(0.8, 0.3), velocity=(0.0, 0.0))
    desired = _legacy_desired_force(velocity, (1.0, 0.0), 0.8, 0.5)
    _, _, _, old_robot_social = _legacy_social_force(
        position, velocity, robot, 10.0
    )
    old_robot_personal = _legacy_robot_personal_space(
        position, yaw, robot.position, 1.0, 0.2
    )
    force_social = 5.1
    force_robot_personal_space = 6.0
    old_robot_social_weighted = (
        force_social * old_robot_social[0],
        force_social * old_robot_social[1],
    )
    old_robot_personal_weighted = (
        force_robot_personal_space * old_robot_personal[0],
        force_robot_personal_space * old_robot_personal[1],
    )
    old_core = (
        desired[0]
        + old_robot_social_weighted[0]
        + old_robot_personal_weighted[0],
        desired[1]
        + old_robot_social_weighted[1]
        + old_robot_personal_weighted[1],
    )

    core = compute_social_interaction_core(
        pedestrian_position=position,
        pedestrian_yaw=yaw,
        pedestrian_velocity=velocity,
        desired_direction=(1.0, 0.0),
        vmax=0.8,
        relaxation_time=0.5,
        desired_force_factor=1.0,
        human_neighbors=(),
        robot_state=robot,
        neighbor_range=10.0,
        force_social=force_social,
        robot_clearance=1.0,
        sigma_robot_personal_space=0.2,
        force_robot_personal_space=force_robot_personal_space,
    )
    old_pre_clip, old_post_clip = _legacy_velocity_step(
        velocity, old_core, 0.05, 0.8
    )
    new_step = velocity_step(velocity, core.core_acceleration, 0.05, 0.8)
    max_error = max(
        _assert_vector_matches(old_robot_social, core.robot_social_raw),
        _assert_vector_matches(
            old_robot_social_weighted, core.robot_social_weighted
        ),
        _assert_vector_matches(old_robot_personal, core.robot_personal_space_raw),
        _assert_vector_matches(
            old_robot_personal_weighted,
            core.robot_personal_space_weighted,
        ),
        _assert_vector_matches(old_core, core.core_acceleration),
        _assert_vector_matches(old_pre_clip, new_step.pre_clip_velocity),
        _assert_vector_matches(old_post_clip, new_step.post_clip_velocity),
    )
    print(f"Test 3 max_abs_error={max_error:.17g}")


def test_robot_personal_space_overlap_fallback_regression():
    position = (1.25, -0.4)
    yaw = 0.73
    old_force = _legacy_robot_personal_space(position, yaw, position, 1.0, 0.2)
    new_force = robot_personal_space_force(position, yaw, position, 1.0, 0.2)
    assert _assert_vector_matches(old_force, new_force) <= ABS_TOL


def test_kernel_dependency_and_controller_delegation_boundaries():
    kernel_source = (SCRIPTS / "gazebo_social_kernel.py").read_text()
    controller_source = (SCRIPTS / "scenario_pedestrian_controller.py").read_text()

    imported_roots = set()
    for node in ast.walk(ast.parse(kernel_source)):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported_roots.add(node.module.split(".", 1)[0])
    assert imported_roots <= {"__future__", "math", "dataclasses", "typing"}
    assert "head_on_bias" not in kernel_source
    assert "smoothing" not in kernel_source
    assert "compute_social_interaction_core(" in controller_source
    assert "step = velocity_step(" in controller_source
