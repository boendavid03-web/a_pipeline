"""Exact raw-layer parity between Isaac integration and the shared kernel."""

from __future__ import annotations

import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ISAAC_SCRIPTS = PROJECT_ROOT / "isaac_sim/scripts"
GAZEBO_SCRIPTS = (
    PROJECT_ROOT / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts"
)
for directory in (ISAAC_SCRIPTS, GAZEBO_SCRIPTS):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from gazebo_social_kernel import (  # noqa: E402
    KinematicState,
    compute_social_interaction_core,
    velocity_step,
)
import pedestrian_social as isaac_social  # noqa: E402
from pedestrian_social import (  # noqa: E402
    PedestrianMotionState,
    PedestrianSocialForceController,
    RobotMotionState,
    SocialForceParameters,
    gazebo_social_kernel_source_path,
)


ABS_TOL = 1.0e-12
EXPECTED_KERNEL = (GAZEBO_SCRIPTS / "gazebo_social_kernel.py").resolve()


def _state(position, velocity, direction, speed=0.8, yaw=0.0):
    return PedestrianMotionState(position, velocity, direction, speed, yaw)


def _max_vector_error(left, right):
    error = max(abs(left[index] - right[index]) for index in range(2))
    assert error <= ABS_TOL
    return error


def _standalone(
    name: str,
    states: dict[str, PedestrianMotionState],
    robot: RobotMotionState | None,
    dt: float,
    parameters: SocialForceParameters,
):
    state = states[name]
    shared_robot = (
        KinematicState(robot.position_m, robot.velocity_mps)
        if robot is not None
        else None
    )
    core = compute_social_interaction_core(
        pedestrian_position=state.position_m,
        pedestrian_yaw=float(state.yaw_rad),
        pedestrian_velocity=state.velocity_mps,
        desired_direction=state.desired_direction,
        vmax=state.preferred_speed_mps,
        relaxation_time=parameters.relaxation_time_sec,
        desired_force_factor=1.0,
        human_neighbors=(
            KinematicState(other.position_m, other.velocity_mps)
            for other_name, other in sorted(states.items())
            if other_name != name
        ),
        robot_state=shared_robot,
        neighbor_range=parameters.neighbor_range_m,
        force_social=parameters.human_social_force_weight,
        robot_clearance=parameters.robot_clearance_m,
        sigma_robot_personal_space=parameters.robot_personal_space_sigma_m,
        force_robot_personal_space=parameters.robot_personal_space_force_weight,
    )
    step = velocity_step(
        state.velocity_mps,
        core.core_acceleration,
        dt,
        state.preferred_speed_mps,
    )
    return core, step


def test_isaac_raw_layer_imports_and_matches_the_shared_source_file():
    parameters = SocialForceParameters()
    states = {"a": _state((0.2, -0.4), (0.1, 0.05), (0.6, 0.8))}
    output = PedestrianSocialForceController(parameters).update(
        states, None, 0.073
    )["a"]
    core, step = _standalone("a", states, None, 0.073, parameters)

    assert gazebo_social_kernel_source_path() == EXPECTED_KERNEL
    imported_kernel_path = Path(
        sys.modules[
            isaac_social.compute_social_interaction_core.__module__
        ].__file__
    ).resolve()
    assert imported_kernel_path == EXPECTED_KERNEL
    max_error = max(
        _max_vector_error(
            output.gazebo_desired_acceleration_mps2,
            core.desired_acceleration,
        ),
        _max_vector_error(
            output.gazebo_core_acceleration_mps2, core.core_acceleration
        ),
        _max_vector_error(
            output.gazebo_raw_pre_clip_velocity_mps,
            step.pre_clip_velocity,
        ),
        _max_vector_error(output.gazebo_raw_velocity_mps, step.post_clip_velocity),
    )
    assert max_error == 0.0


def test_two_person_head_on_raw_parity():
    parameters = SocialForceParameters()
    states = {
        "a": _state((-1.0, 0.0), (0.6, 0.0), (1.0, 0.0)),
        "b": _state((1.0, 0.0), (-0.6, 0.0), (-1.0, 0.0), yaw=math.pi),
    }
    outputs = PedestrianSocialForceController(parameters).update(
        states, None, 0.1
    )
    max_error = 0.0
    for name in states:
        core, step = _standalone(name, states, None, 0.1, parameters)
        max_error = max(
            max_error,
            _max_vector_error(
                outputs[name].gazebo_human_social_raw_mps2,
                core.human_social_raw,
            ),
            _max_vector_error(
                outputs[name].gazebo_core_acceleration_mps2,
                core.core_acceleration,
            ),
            _max_vector_error(
                outputs[name].gazebo_raw_velocity_mps,
                step.post_clip_velocity,
            ),
        )
        assert outputs[name].gazebo_raw_velocity_mps[1] == 0.0
    assert max_error == 0.0


def test_pedestrian_robot_center_raw_parity():
    parameters = SocialForceParameters()
    states = {
        "a": _state((0.0, 0.0), (0.2, -0.05), (1.0, 0.0), yaw=0.4)
    }
    robot = RobotMotionState(
        position_m=(0.8, 0.3),
        velocity_mps=(0.0, 0.0),
        yaw_rad=1.2,
        half_extents_m=(1.7, 0.1),
    )
    output = PedestrianSocialForceController(parameters).update(
        states, robot, 0.05
    )["a"]
    core, step = _standalone("a", states, robot, 0.05, parameters)
    max_error = max(
        _max_vector_error(
            output.gazebo_robot_social_raw_mps2, core.robot_social_raw
        ),
        _max_vector_error(
            output.gazebo_robot_personal_space_raw_mps2,
            core.robot_personal_space_raw,
        ),
        _max_vector_error(
            output.gazebo_robot_social_weighted_mps2,
            core.robot_social_weighted,
        ),
        _max_vector_error(
            output.gazebo_robot_personal_space_weighted_mps2,
            core.robot_personal_space_weighted,
        ),
        _max_vector_error(
            output.gazebo_raw_velocity_mps, step.post_clip_velocity
        ),
    )
    assert max_error == 0.0


def test_adapter_separation_retains_raw_input_and_reports_changed_output():
    parameters = SocialForceParameters(smoothing_time_sec=1.0)
    states = {
        "eastbound": _state((0.0, 0.0), (0.8, 0.0), (1.0, 0.0)),
        "southbound": _state((0.7, 0.7), (0.0, -0.8), (0.0, -1.0), yaw=-math.pi / 2.0),
    }
    output = PedestrianSocialForceController(parameters).update(
        states, None, 0.1
    )["eastbound"]

    assert output.isaac_adapter_input_velocity_mps == output.gazebo_raw_velocity_mps
    assert math.dist(
        output.isaac_adapter_input_velocity_mps,
        output.isaac_adapter_output_velocity_mps,
    ) > 0.0
    assert output.adapter_smoothing_applied
    assert output.gazebo_raw_dt_sec == 0.1
    assert output.isaac_adapter_dt_sec == 0.1


def test_solver_dt_is_unclamped_while_adapter_dt_is_explicitly_limited():
    parameters = SocialForceParameters(maximum_dt_sec=0.2)
    states = {"a": _state((0.0, 0.0), (0.0, 0.0), (1.0, 0.0))}
    output = PedestrianSocialForceController(parameters).update(
        states, None, 0.35
    )["a"]
    _core, step = _standalone("a", states, None, 0.35, parameters)

    assert output.gazebo_raw_dt_sec == 0.35
    assert output.isaac_adapter_dt_sec == 0.2
    assert output.adapter_dt_limited
    assert _max_vector_error(output.gazebo_raw_velocity_mps, step.post_clip_velocity) == 0.0
