from pathlib import Path
import sys
import time

import numpy as np


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))

from hunav_protocol import (  # noqa: E402
    COMMAND_SCHEMA,
    CommandInbox,
    IndependentYieldPlanner,
    RecoverySupervisor,
    SafeVelocityProjector,
    SafetyConfig,
    bounded_route_social_velocity,
    forward_recovery_goal,
)
from isaac_navmesh import IsaacNavMeshGuard  # noqa: E402


def command(sequence=1, state_sequence=1, sim_time=1.0):
    return {
        "schema": COMMAND_SCHEMA,
        "session_id": "test-session",
        "sequence": sequence,
        "state_sequence": state_sequence,
        "sim_time": sim_time,
        "commands": [{
            "id": 1,
            "desired_velocity": [0.5, 0.0],
            "source_position": [0.95, 0.0],
            "target_position": [1.0, 0.0],
            "yaw": 0.0,
        }],
    }


def test_inbox_rejects_duplicate_out_of_order_future_and_timeout():
    inbox = CommandInbox(timeout_sec=0.01)
    assert inbox.accept(command(), 1.0)
    assert not inbox.accept(command(), 1.0)
    assert not inbox.accept(command(0, 0), 1.0)
    assert not inbox.accept(command(2, 2, 2.0), 1.0)
    time.sleep(0.02)
    assert inbox.current(1.1)[1] == "command_timeout"


def test_command_accepts_and_validates_external_goal_sync_evidence():
    payload = command()
    payload["commands"][0].update({
        "requested_goal": [2.0, 3.0],
        "active_goal": [2.0, 3.0],
        "active_goal_count": 1,
        "goal_sync_error_m": 0.0,
    })
    inbox = CommandInbox(timeout_sec=1.0)
    assert inbox.accept(payload, 1.0)

    invalid = command(sequence=2, state_sequence=2)
    invalid["commands"][0].update({
        "requested_goal": [2.0, 3.0],
        "active_goal": [2.0, 3.0],
        "active_goal_count": 0,
        "goal_sync_error_m": 0.0,
    })
    assert not inbox.accept(invalid, 1.0)


def test_projector_limits_acceleration_without_reimplementing_social_avoidance():
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=1.0))
    positions = {1: np.array([-0.4, 0.0]), 2: np.array([0.4, 0.0])}
    current = {1: np.zeros(2), 2: np.zeros(2)}
    desired = {1: np.array([1.0, 0.0]), 2: np.array([-1.0, 0.0])}
    output = projector.project(
        positions, current, desired, 0.1,
        np.array([10.0, 10.0]), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
    )
    assert all(np.linalg.norm(value) <= 0.100001 for value in output.values())
    assert projector.metrics.projection_count >= 2


def test_projector_does_not_steer_apart_safe_hunav_commands():
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=100.0))
    desired = {1: np.array([0.5, 0.0]), 2: np.array([-0.5, 0.0])}
    output = projector.project(
        {1: np.array([-0.4, 0.0]), 2: np.array([0.4, 0.0])},
        {1: desired[1].copy(), 2: desired[2].copy()}, desired, 0.1,
        np.zeros(2), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: True,
    )
    assert np.allclose(output[1], desired[1])
    assert np.allclose(output[2], desired[2])
    assert projector.metrics.projection_count == 0


def test_projector_preserves_tangent_motion_while_preventing_physical_overlap():
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=100.0))
    desired = {1: np.array([1.0, 0.2]), 2: np.array([-1.0, 0.2])}
    positions = {1: np.array([-0.3, 0.0]), 2: np.array([0.3, 0.0])}
    output = projector.project(
        positions,
        {1: desired[1].copy(), 2: desired[2].copy()}, desired, 0.1,
        np.zeros(2), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: True,
    )
    first_end = positions[1] + output[1] * 0.1
    second_end = positions[2] + output[2] * 0.1
    assert np.linalg.norm(second_end - first_end) >= 0.500999
    assert np.isclose(output[1][1], 0.2)
    assert np.isclose(output[2][1], 0.2)
    assert not np.allclose(output[1], 0.0)
    assert not np.allclose(output[2], 0.0)
    assert projector.metrics.penetration_hold_count == 0
    assert projector.metrics.pair_projection_count >= 1


def test_projector_can_leave_human_pairs_to_native_controller():
    projector = SafeVelocityProjector(SafetyConfig(
        max_acceleration_mps2=100.0,
        enforce_pair_separation=False,
    ))
    desired = {1: np.array([1.0, 0.0]), 2: np.array([-1.0, 0.0])}
    positions = {1: np.array([-0.3, 0.0]), 2: np.array([0.3, 0.0])}
    output = projector.project(
        positions,
        {1: desired[1].copy(), 2: desired[2].copy()},
        desired,
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
    )
    assert np.allclose(output[1], desired[1])
    assert np.allclose(output[2], desired[2])
    assert projector.metrics.pair_projection_count == 0
    assert all("physical_pair_projection" not in reasons for reasons in projector.last_reasons.values())


def test_pair_projection_keeps_selected_yielder_fixed():
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=100.0))
    positions = {1: np.array([-0.3, 0.0]), 2: np.array([0.3, 0.0])}
    desired = {1: np.zeros(2), 2: np.array([-1.0, 0.0])}
    output = projector.project(
        positions,
        {1: np.zeros(2), 2: desired[2].copy()},
        desired,
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        fixed_pair_agent_ids={1},
    )
    assert np.allclose(output[1], [0.0, 0.0])
    first_end = positions[1] + output[1] * 0.1
    second_end = positions[2] + output[2] * 0.1
    assert np.linalg.norm(second_end - first_end) >= 0.500999


def test_unilateral_pair_fallback_stops_moving_yielder_only_for_conflict():
    projector = SafeVelocityProjector(SafetyConfig(
        max_acceleration_mps2=100.0,
        max_deceleration_mps2=100.0,
        pair_unilateral_hold_fallback=True,
    ))
    positions = {1: np.array([-0.3, 0.0]), 2: np.array([0.3, 0.0])}
    desired = {1: np.array([0.8, 0.0]), 2: np.array([-0.2, 0.0])}
    output = projector.project(
        positions,
        desired,
        desired,
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        fixed_pair_agent_ids={1},
    )
    assert np.allclose(output[1], [0.0, 0.0])
    first_end = positions[1] + output[1] * 0.1
    second_end = positions[2] + output[2] * 0.1
    assert np.linalg.norm(second_end - first_end) >= 0.500999


def test_unilateral_pair_fallback_uses_local_hold_when_navmesh_clamps_correction():
    projector = SafeVelocityProjector(SafetyConfig(
        max_acceleration_mps2=100.0,
        pair_projection_iterations=2,
        pair_unilateral_hold_fallback=True,
    ))
    positions = {1: np.array([0.0, 0.0]), 2: np.array([0.6, 0.0])}
    desired = {1: np.array([1.0, 0.0]), 2: np.array([-1.0, 0.0])}
    blocked_endpoints = {1: np.array([0.1, 0.0]), 2: np.array([0.5, 0.0])}
    output = projector.project(
        positions,
        {1: np.zeros(2), 2: np.zeros(2)},
        desired,
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        segment_projector=lambda agent_id, _start, _end: blocked_endpoints[agent_id],
    )
    first_end = positions[1] + output[1] * 0.1
    second_end = positions[2] + output[2] * 0.1
    assert np.linalg.norm(second_end - first_end) >= 0.500999
    assert "pair_unilateral_hold" in projector.last_reasons[1]
    assert "pair_unilateral_hold" in projector.last_reasons[2]


def test_kinematic_pair_projection_preserves_limits_and_separation_over_time():
    dt = 0.05
    config = SafetyConfig(
        human_radius_m=0.25,
        max_speed_mps=1.5,
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        pair_projection_iterations=24,
        pair_kinematic_feasibility=True,
    )
    projector = SafeVelocityProjector(config)
    positions = {1: np.array([-1.0, 0.0]), 2: np.array([1.0, 0.0])}
    velocities = {1: np.zeros(2), 2: np.zeros(2)}
    desired = {1: np.array([1.0, 0.0]), 2: np.array([-1.0, 0.0])}

    minimum_distance = float("inf")
    for _ in range(160):
        previous = {agent_id: value.copy() for agent_id, value in velocities.items()}
        velocities = projector.project(
            positions,
            velocities,
            desired,
            dt,
            np.array([10.0, 10.0]),
            np.zeros(2),
            [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
            lambda _start, _end: True,
        )
        for agent_id in velocities:
            assert np.linalg.norm(velocities[agent_id]) <= 1.500001
            assert np.linalg.norm(velocities[agent_id] - previous[agent_id]) <= (
                config.max_deceleration_mps2 * dt + 1.0e-6
            )
            positions[agent_id] = positions[agent_id] + velocities[agent_id] * dt
        minimum_distance = min(
            minimum_distance, float(np.linalg.norm(positions[2] - positions[1]))
        )

    assert minimum_distance >= 0.500999
    assert projector.metrics.pair_projection_count > 0


def test_kinematic_pair_projection_brakes_diagonal_head_on_motion_early():
    dt = 0.05
    config = SafetyConfig(
        human_radius_m=0.25,
        max_speed_mps=1.5,
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        pair_projection_iterations=24,
        pair_kinematic_feasibility=True,
    )
    projector = SafeVelocityProjector(config)
    positions = {1: np.array([-0.6, 0.0]), 2: np.array([0.6, 0.0])}
    velocities = {
        1: np.array([1.0, 1.0]),
        2: np.array([-1.0, -1.0]),
    }
    desired = {agent_id: value.copy() for agent_id, value in velocities.items()}

    minimum_distance = float("inf")
    for _ in range(80):
        previous = {agent_id: value.copy() for agent_id, value in velocities.items()}
        velocities = projector.project(
            positions,
            velocities,
            desired,
            dt,
            np.array([10.0, 10.0]),
            np.zeros(2),
            [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
            lambda _start, _end: True,
        )
        for agent_id in velocities:
            assert np.linalg.norm(velocities[agent_id]) <= 1.500001
            assert np.linalg.norm(velocities[agent_id] - previous[agent_id]) <= (
                config.max_deceleration_mps2 * dt + 1.0e-6
            )
            positions[agent_id] = positions[agent_id] + velocities[agent_id] * dt
        minimum_distance = min(
            minimum_distance, float(np.linalg.norm(positions[2] - positions[1]))
        )

    assert minimum_distance >= 0.500999


def test_kinematic_static_projection_cannot_create_a_speed_jump():
    projector = SafeVelocityProjector(SafetyConfig(
        max_speed_mps=1.5,
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        enforce_pair_separation=False,
        pair_kinematic_feasibility=True,
    ))
    output = projector.project(
        {1: np.array([0.0, 0.0])},
        {1: np.zeros(2)},
        {1: np.array([0.2, 0.0])},
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        segment_projector=lambda _agent_id, _start, _end: np.array([1.0, 0.0]),
    )
    assert np.linalg.norm(output[1]) <= 0.120001


def test_static_lookahead_steers_before_current_step_reaches_boundary():
    projector = SafeVelocityProjector(SafetyConfig(
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        enforce_pair_separation=False,
        static_lookahead_sec=0.35,
    ))

    def project(_agent_id, _start, end):
        if end[0] > 0.02:
            return np.array([0.0, 0.10])
        return end

    output = projector.project(
        {1: np.zeros(2)}, {1: np.zeros(2)}, {1: np.array([1.0, 0.0])}, 0.1,
        np.array([10.0, 10.0]), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: True,
        segment_projector=project,
    )
    assert output[1][1] > 0.0
    assert np.linalg.norm(output[1]) <= 0.120001
    assert "static_boundary_lookahead_steering" in projector.last_reasons[1]


def test_kinematic_static_projection_prefers_safe_current_velocity():
    projector = SafeVelocityProjector(SafetyConfig(
        max_speed_mps=1.5,
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        enforce_pair_separation=False,
        pair_kinematic_feasibility=True,
    ))

    def clamp_to_half_plane(_agent_id, _start, end):
        return np.array([min(float(end[0]), 0.0), float(end[1])])

    output = projector.project(
        {1: np.array([0.0, 0.0])},
        {1: np.zeros(2)},
        {1: np.array([1.0, 0.0])},
        0.1,
        np.array([10.0, 10.0]),
        np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        segment_projector=clamp_to_half_plane,
    )
    assert np.allclose(output[1], np.zeros(2))


def test_independent_yield_planner_never_pauses_adjacent_agents():
    planner = IndependentYieldPlanner(cooldown_sec=0.0)
    positions = {
        1: np.array([0.0, 0.0]),
        2: np.array([0.6, 0.0]),
        3: np.array([1.2, 0.0]),
        4: np.array([1.8, 0.0]),
    }
    active = planner.update(positions, {1, 2, 3, 4}, 2.0)
    assert active
    for first in active:
        for second in active - {first}:
            assert np.linalg.norm(positions[first] - positions[second]) >= 1.10


def test_independent_yield_releases_after_trigger_peer_clears():
    planner = IndependentYieldPlanner(cooldown_sec=0.0)
    positions = {1: np.array([0.0, 0.0]), 2: np.array([0.6, 0.0])}
    assert planner.update(positions, {1, 2}, 2.0) == {2}
    separated = {1: np.array([0.0, 0.0]), 2: np.array([1.2, 0.0])}
    assert planner.update(separated, set(), 3.0) == set()
    assert planner.start_count == 1
    assert planner.end_count == 1


def test_independent_yield_rotates_after_bounded_hold_in_dense_cluster():
    planner = IndependentYieldPlanner(maximum_hold_sec=1.0, cooldown_sec=0.5)
    positions = {
        1: np.array([0.0, 0.0]),
        2: np.array([0.6, 0.0]),
        3: np.array([1.2, 0.0]),
    }
    first = planner.update(positions, {1, 2, 3}, 0.0)
    assert first
    second = planner.update(positions, {1, 2, 3}, 1.01)
    assert first.isdisjoint(second)
    assert planner.end_count >= len(first)
    assert all(planner.yield_counts.get(agent_id, 0) <= 1 for agent_id in first)


def test_independent_yield_default_cooldown_protects_released_agent_recovery_window():
    planner = IndependentYieldPlanner(maximum_hold_sec=1.0)
    positions = {1: np.array([0.0, 0.0]), 2: np.array([0.6, 0.0])}
    first = planner.update(positions, {1, 2}, 0.0)
    assert first == {2}

    # Protect the released agent for three seconds, but do not cool down the
    # whole pair: its still-stalled peer must immediately take the next yield
    # turn and give the released walker an explicit right of way.
    second = planner.update(positions, {1, 2}, 1.01)
    assert second == {1}
    third = planner.update(positions, {1, 2}, 2.02)
    assert third == set()
    assert planner.agent_cooldown_until[2] > 4.0
    fourth = planner.update(positions, {1, 2}, 4.02)
    assert fourth == {2}


def test_route_biased_social_velocity_preserves_forward_progress():
    output = bounded_route_social_velocity(
        np.array([1.0, 0.0]),
        0.8,
        np.array([-0.2, 0.3]),
        maximum_social_correction_mps=0.35,
    )
    assert output[0] > 0.0
    assert np.linalg.norm(output - np.array([0.8, 0.0])) <= 0.350001


def test_route_biased_social_velocity_keeps_small_hunav_residual():
    raw = np.array([0.65, 0.15])
    output = bounded_route_social_velocity(
        np.array([2.0, 0.0]), 0.8, raw, maximum_social_correction_mps=0.35
    )
    assert np.allclose(output, raw)


def test_forward_recovery_goal_stops_before_route_reversal():
    goal = forward_recovery_goal(
        np.array([20.575, 20.325]),
        [
            [21.575, 20.325],
            [22.425, 20.325],
            [21.425, 20.325],
            [20.425, 20.325],
            [19.425, 20.325],
        ],
    )
    assert np.allclose(goal, [22.425, 20.325])


def test_forward_recovery_goal_is_bounded_on_straight_route():
    goal = forward_recovery_goal(
        np.array([0.0, 0.0]),
        [[0.5, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]],
        maximum_lookahead_m=2.0,
    )
    assert np.allclose(goal, [2.0, 0.0])


def test_projector_prevents_oriented_robot_rectangle_overlap() -> None:
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=100.0))
    position = np.array([0.0, 0.75])
    output = projector.project(
        {1: position}, {1: np.array([0.0, -1.0])}, {1: np.array([0.0, -1.0])}, 0.1,
        np.zeros(2), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
        lambda _start, _end: True,
        robot_yaw_rad=0.0,
    )
    endpoint = position + output[1] * 0.1
    assert endpoint[1] >= 0.3 + 0.25 + 0.12 - 1.0e-9
    assert projector.metrics.robot_rectangle_projection_count == 1
    assert "physical_robot_rectangle_projection" in projector.last_reasons[1]


def test_static_guard_stops_without_pose_reset():
    projector = SafeVelocityProjector()
    output = projector.project(
        {1: np.zeros(2)}, {1: np.zeros(2)}, {1: np.ones(2)}, 0.1,
        np.array([10.0, 10.0]), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: False,
    )
    assert np.allclose(output[1], 0.0)
    assert projector.metrics.static_hold_count == 1


def test_static_guard_projects_around_boundary_instead_of_stopping():
    projector = SafeVelocityProjector(SafetyConfig(max_acceleration_mps2=100.0))
    output = projector.project(
        {1: np.zeros(2)}, {1: np.zeros(2)}, {1: np.array([0.2, 0.1])}, 0.1,
        np.array([10.0, 10.0]), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: False,
        segment_projector=lambda _agent_id, _start, _end: np.array([0.02, 0.0]),
    )
    assert np.allclose(output[1], [0.2, 0.0])
    assert projector.metrics.static_projection_count == 1
    assert projector.metrics.static_hold_count == 0


def test_static_projection_always_preserves_acceleration_bound():
    projector = SafeVelocityProjector(SafetyConfig(
        max_acceleration_mps2=1.2,
        max_deceleration_mps2=2.2,
        enforce_pair_separation=False,
    ))
    output = projector.project(
        {1: np.zeros(2)}, {1: np.zeros(2)}, {1: np.array([0.2, 0.0])}, 0.1,
        np.array([10.0, 10.0]), np.zeros(2),
        [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3]],
        lambda _start, _end: True,
        segment_projector=lambda _agent_id, _start, _end: np.array([1.0, 0.0]),
    )
    assert np.linalg.norm(output[1]) <= 0.120001
    assert projector.metrics.static_projection_count == 1


def test_recovery_supervisor_progresses_without_teleport_action():
    supervisor = RecoverySupervisor(stuck_after_sec=0.2)
    visited = {supervisor.update(0.0, 0.1) for _ in range(60)}
    assert {"YIELD", "LATERAL_ESCAPE", "BACKTRACK", "REPLAN"}.issubset(visited)


def test_recovery_requires_both_low_speed_and_stalled_route_progress():
    supervisor = RecoverySupervisor(stuck_after_sec=0.2)
    for progress in (0.0, 0.1, 0.2, 0.3, 0.4):
        assert supervisor.update(0.0, 0.1, route_progress=progress) == "FOLLOW_ROUTE"
    assert supervisor.update(0.0, 0.1, route_progress=0.4) == "FOLLOW_ROUTE"
    assert supervisor.update(0.0, 0.1, route_progress=0.4) == "YIELD"


def test_replan_failure_is_counted_instead_of_escaping_main_loop():
    class FakeCarb:
        Float3 = staticmethod(lambda *values: tuple(values))

    class DisconnectedNavMesh:
        @staticmethod
        def query_closest_point(point, agent_radius):
            return (point, 0)

        @staticmethod
        def query_shortest_path(start, end, agent_radius):
            return None

    guard = IsaacNavMeshGuard(None, DisconnectedNavMesh(), FakeCarb())
    assert not guard.replan_is_available((22.58, 12.26), (22.12, 12.52, 0.0))
    assert guard.replan_failure_count == 1


def test_navmesh_projects_corner_cut_toward_path_without_changing_step_length():
    class FakeCarb:
        Float3 = staticmethod(lambda *values: tuple(values))

    class Path:
        def __init__(self, points):
            self._points = points

        def get_points(self):
            return self._points

    class HalfPlaneNavMesh:
        @staticmethod
        def query_closest_point(point, agent_radius):
            return ((point[0], min(point[1], 0.0), point[2]), 0)

        @staticmethod
        def query_shortest_path(start, end, agent_radius):
            return Path((start, end))

    guard = IsaacNavMeshGuard(None, HalfPlaneNavMesh(), FakeCarb())
    start = np.array([0.0, 0.0])
    requested = np.array([0.02, 0.02])
    constrained = np.asarray(guard.project_step(start, requested, (1.0, 0.0)))
    assert np.allclose(constrained, [np.linalg.norm(requested), 0.0])
    assert np.isclose(np.linalg.norm(constrained - start), np.linalg.norm(requested))
    assert guard.projected_step_count == 1


def test_navmesh_accepts_quantized_shortest_path_segment_then_recovers_to_mesh():
    class FakeCarb:
        Float3 = staticmethod(lambda *values: tuple(values))

    class Path:
        def __init__(self, points):
            self._points = points

        def get_points(self):
            return self._points

    class QuantizedEdgeNavMesh:
        @staticmethod
        def query_closest_point(point, agent_radius):
            if np.allclose(point[:2], [0.0, 0.0]) or np.allclose(point[:2], [1.0, -1.0]):
                return (point, 0)
            return ((point[0] + 0.04, point[1], point[2]), 0)

        @staticmethod
        def query_shortest_path(start, end, agent_radius):
            return Path((start, end))

    guard = IsaacNavMeshGuard(None, QuantizedEdgeNavMesh(), FakeCarb())
    start = np.array([0.0, 0.0])
    requested = np.array([0.01, -0.01])
    constrained = np.asarray(guard.project_step(start, requested, (1.0, -1.0)))
    assert np.allclose(constrained, requested)
    assert guard.projected_step_failure_count == 0

    # Once the committed start has a projection error, the next bounded step
    # is directed back toward the queried mesh point instead of accumulating
    # the lateral request.
    recovered = np.asarray(
        guard.project_step(constrained, constrained + [0.0, -0.01], (1.0, -1.0))
    )
    projected_start = np.array([0.05, -0.01])
    assert np.linalg.norm(recovered - projected_start) < np.linalg.norm(
        constrained - projected_start
    )
    assert np.linalg.norm(recovered - constrained) <= 0.010001
    assert guard.projected_step_failure_count == 0
