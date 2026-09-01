from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from physx_lidar_people import (  # noqa: E402
    ENDPOINT_HIT_WORLD_TOLERANCE_M,
    endpoint_hit_world_diagnostic,
    endpoint_ranges_from_world_geometry,
    is_ignored_person_query_collider,
    is_ignored_robot_query_collider,
    native_depth_diagnostic,
    nearest_ray_capsule_intersections,
    physics_capture_due,
    ray_start_offsets_outside_box,
    ray_capsule_intersection_matrix,
    scene_query_hit_value,
    summarize_physics_time_qualification,
)


class DummyRaycastHit:
    collision = "/World/Characters/person/Physics/BodyCollider"
    distance = 1.25


@pytest.mark.parametrize(
    "hit, name, expected",
    [
        ({"collision": "/World/Floor", "distance": 2.5}, "collision", "/World/Floor"),
        (DummyRaycastHit(), "collision", DummyRaycastHit.collision),
        (DummyRaycastHit(), "distance", 1.25),
        (DummyRaycastHit(), "missing", None),
    ],
)
def test_scene_query_hit_value_supports_mapping_and_raycast_hit_object(
    hit, name, expected
):
    assert scene_query_hit_value(hit, name) == expected


def test_hits_finite_cylinder_side_and_normalizes_ray_direction():
    distances = ray_capsule_intersection_matrix(
        [[-1.0, 0.0, 0.5]],
        [[2.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0]],
        [[0.0, 0.0, 1.0]],
        0.1,
    )
    assert distances.shape == (1, 1)
    assert distances[0, 0] == pytest.approx(0.9)


def test_hits_outward_caps_and_misses_parallel_ray_outside_radius():
    distances = ray_capsule_intersection_matrix(
        [[0.0, 0.0, -1.0], [0.2, 0.0, -1.0]],
        [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
        [[0.0, 0.0, 0.0]],
        [[0.0, 0.0, 1.0]],
        0.1,
    )
    assert distances[0, 0] == pytest.approx(0.9)
    assert math.isinf(distances[1, 0])


def test_origin_inside_capsule_returns_forward_exit():
    distances = ray_capsule_intersection_matrix(
        [[0.0, 0.0, 0.5]],
        [[1.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0]],
        [[0.0, 0.0, 1.0]],
        0.1,
    )
    assert distances[0, 0] == pytest.approx(0.1)


def test_vectorized_batch_returns_nearest_leg_and_preserves_occlusion():
    origins = np.asarray([[-1.0, 0.0, 0.5], [-1.0, 0.5, 0.5]])
    directions = np.asarray([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    starts = np.asarray([[0.0, 0.0, 0.0], [0.4, 0.0, 0.0]])
    ends = np.asarray([[0.0, 0.0, 1.0], [0.4, 0.0, 1.0]])

    matrix = ray_capsule_intersection_matrix(origins, directions, starts, ends, [0.1, 0.1])
    distances, indices = nearest_ray_capsule_intersections(
        origins, directions, starts, ends, [0.1, 0.1]
    )

    assert matrix.shape == (2, 2)
    assert matrix[0].tolist() == pytest.approx([0.9, 1.3])
    assert distances[0] == pytest.approx(0.9)
    assert indices.tolist() == [0, -1]
    assert math.isinf(distances[1])


def test_tilted_capsule_uses_complete_shin_foot_axis():
    distances = ray_capsule_intersection_matrix(
        [[-1.0, 0.0, 0.5]],
        [[1.0, 0.0, 0.0]],
        [[-0.2, 0.0, 0.0]],
        [[0.2, 0.0, 1.0]],
        0.05,
    )
    # The axis crosses z=0.5 at x=0; a vertical proxy at the foot would not.
    assert distances[0, 0] == pytest.approx(0.946148, abs=1.0e-6)


@pytest.mark.parametrize(
    "path, expected",
    [
        ("/World/Characters/person/Physics/BodyCollider", True),
        ("/World/Characters/person/Physics/AvoidanceTrigger", True),
        ("/World/Characters/person/Physics/LeftLegLidarProxy", False),
        ("/World/Robot/Physics/BodyCollider", False),
        ("/World/Characters/person/BodyColliderVisual", False),
        (None, False),
    ],
)
def test_full_body_query_collider_filter_is_narrow(path, expected):
    assert is_ignored_person_query_collider(path) is expected


@pytest.mark.parametrize(
    "path, expected",
    [
        ("/World/Robot/Physics/Body", True),
        ("/World/RobotCollisionProxy", True),
        ("/World/RobotCollisionProxy/Shape", True),
        ("/World/RobotLike", False),
        ("/World/Characters/person/Physics/BodyCollider", False),
        (None, False),
    ],
)
def test_robot_self_query_filter_is_root_bounded(path, expected):
    assert is_ignored_robot_query_collider(path) is expected


def test_ray_start_offsets_preserve_minimum_and_exit_robot_box():
    offsets = ray_start_offsets_outside_box(
        [0.2, 0.0],
        [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]],
        [0.5, 0.5],
        0.5,
    )
    assert offsets.tolist() == pytest.approx([0.5, 0.51, 0.71])


def test_sixty_physics_steps_produce_fifteen_capture_events():
    next_capture = 1.0 / 15.0
    captures = 0
    missed = 0
    for step in range(1, 61):
        due, next_capture, skipped = physics_capture_due(
            step / 60.0, next_capture, 1.0 / 15.0
        )
        captures += int(due)
        missed += skipped
    assert captures == 15
    assert missed == 0
    assert next_capture == pytest.approx(1.0666666666666667)


def test_capture_timestamps_are_strictly_monotonic():
    next_capture = 1.0 / 15.0
    timestamps = []
    for step in range(1, 61):
        current = step / 60.0
        due, next_capture, _skipped = physics_capture_due(
            current, next_capture, 1.0 / 15.0
        )
        if due:
            timestamps.append(current)
    assert len(timestamps) == 15
    assert np.all(np.diff(timestamps) > 0.0)


def test_physics_time_qualification_accepts_per_step_manager_time_with_timeline_duplicates():
    samples = []
    for index in range(1, 121):
        physics_step = 1000 + index
        samples.append(
            {
                "app_update_sequence": (index - 1) // 4,
                "physics_callback_sequence": index,
                "physics_step_count": physics_step,
                "timeline_time": 20.0 + ((index - 1) // 4) / 15.0,
                "simulation_manager_time": physics_step / 60.0,
                "callback_dt": 1.0 / 60.0,
                "wall_monotonic": 100.0 + index / 20.0,
            }
        )
    summary = summarize_physics_time_qualification(samples, 60.0)
    assert summary["same_app_update_multi_physics_callbacks"] is True
    assert summary["same_app_update_max_physics_callbacks"] == 4
    assert summary["timeline_duplicate_count"] == 90
    assert summary["simulation_manager_duplicate_count"] == 0
    assert summary["simulation_manager_non_monotonic_count"] == 0
    assert summary["expected_dt_error_max"] < 1.0e-12
    assert summary["step_mapping_error"]["span"] < 1.0e-12
    assert abs(summary["long_term_drift_slope_sec_per_sim_sec"]) < 1.0e-12
    assert summary["qualification"] == "PASS"


def test_physics_time_qualification_rejects_same_app_manager_duplicates():
    samples = []
    for index in range(1, 101):
        samples.append(
            {
                "app_update_sequence": (index - 1) // 2,
                "physics_callback_sequence": index,
                "physics_step_count": index,
                "timeline_time": ((index - 1) // 2) / 30.0,
                "simulation_manager_time": ((index - 1) // 2) / 30.0,
                "callback_dt": 1.0 / 60.0,
                "wall_monotonic": 100.0 + index / 20.0,
            }
        )
    summary = summarize_physics_time_qualification(samples, 60.0)
    assert summary["simulation_manager_duplicate_count"] == 50
    assert summary["simulation_manager_strictly_increasing"] is False
    assert summary["qualification"] == "FAIL"


def test_production_physx_stamp_uses_qualified_time_and_integer_step_cadence():
    source = (
        Path(__file__).resolve().parents[1]
        / "scripts/show_warehouse_people_robot_6_0.py"
    ).read_text()
    assert "self.physics_steps % self._capture_period_steps" in source
    assert "self._capture_sim_time = float(" in source
    assert "SimulationManager.get_simulation_time()" in source
    post_callback = source[
        source.index("    def _on_physics_post_step(") : source.index(
            "    def close(", source.index("    def _on_physics_post_step(")
        )
    ]
    assert "sim_time = self._capture_sim_time" in post_callback
    assert "sim_time = self.physics_sim_time" not in post_callback
    assert "self.timeline.get_current_time()" not in post_callback
    assert '"sim_time": authoritative_ros_sim_time' in source
    assert 'LIDAR_TIMESTAMP_DOMAIN = "simulation_manager_physics_time"' in source

    relay = (
        Path(__file__).resolve().parents[1] / "scripts/cmd_vel_udp_relay.py"
    ).read_text()
    handler = relay[
        relay.index("    def handle_lidar_telemetry(") : relay.index(
            "    def publish_odometry("
        )
    ]
    assert handler.index("self.clock_pub.publish(clock)") < handler.index(
        "self.publish_odometry(pose, self.actual_velocity)"
    ) < handler.index("self.publish_scans(scans, sim_time)")


def test_bad_native_depth_is_nonfatal_and_endpoint_geometry_remains_range_source():
    origins = np.zeros((4, 3), dtype=float)
    endpoints = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0], [4.0, 0.0, 0.0]]
    )
    offsets = np.asarray([0.1, 0.2, 0.3, 0.4])
    ranges = endpoint_ranges_from_world_geometry(endpoints, origins, offsets, 1.0)
    diagnostic = native_depth_diagnostic(
        np.asarray([20.0, 20.0, 20.0, 20.0]), ranges, offsets, 1.0
    )
    assert ranges.tolist() == pytest.approx([1.1, 2.2, 3.3, 4.4])
    assert diagnostic["disagreement_count"] == 4
    assert diagnostic["error_max_m"] > 15.0
    assert diagnostic["depth_unique_count"] == 1


def test_producer_integration_has_no_fatal_depth_endpoint_disagreement():
    source = (
        Path(__file__).resolve().parents[1]
        / "scripts/show_warehouse_people_robot_6_0.py"
    ).read_text()
    assert "native PhysX depth/endpoint disagreement: sensor=" not in source
    assert "endpoint_ranges_from_world_geometry(" in source
    assert "native_depth_diagnostic(" in source


def test_dual_lidar_endpoint_ranges_preserve_two_thousand_beams_each():
    origins = np.zeros((2000, 3), dtype=float)
    endpoints = np.column_stack(
        (np.ones(2000), np.zeros(2000), np.zeros(2000))
    )
    offsets = np.full(2000, 0.5)
    front = endpoint_ranges_from_world_geometry(endpoints, origins, offsets, 1.0)
    rear = endpoint_ranges_from_world_geometry(endpoints, origins, offsets, 1.0)
    assert front.shape == rear.shape == (2000,)
    assert np.all(front == pytest.approx(1.5))


def test_endpoint_hit_world_geometry_accepts_measured_micrometre_scale_error():
    endpoints = np.asarray([[10.0, 2.0, 0.0], [11.0, 2.0, 0.0]])
    hits_sensor = np.asarray([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    transform = np.eye(4)
    transform[3, :3] = [9.0, 2.0, 0.0]
    endpoints[1, 1] += 1.1e-6
    diagnostic = endpoint_hit_world_diagnostic(
        endpoints, hits_sensor, ["/World/A", "/World/B"], transform, 1.0
    )
    assert diagnostic["error_max_m"] == pytest.approx(1.1e-6)
    assert diagnostic["tolerance_m"] == ENDPOINT_HIT_WORLD_TOLERANCE_M


def test_endpoint_hit_world_severe_inconsistency_is_a_safety_failure():
    endpoints = np.asarray([[1.0, 0.0, 0.0]])
    hits_sensor = np.asarray([[1.01, 0.0, 0.0]])
    with pytest.raises(
        RuntimeError, match="endpoint/hit WORLD geometry inconsistency"
    ):
        endpoint_hit_world_diagnostic(
            endpoints, hits_sensor, ["/World/Obstacle"], np.eye(4), 1.0
        )


def test_slow_app_frame_does_not_change_simulation_clock_capture_count():
    next_capture = 1.0 / 15.0
    captures = 0
    app_frames = 0
    for step in range(1, 61):
        due, next_capture, skipped = physics_capture_due(
            step / 60.0, next_capture, 1.0 / 15.0
        )
        captures += int(due)
        assert skipped == 0
        if step in (20, 40, 60):
            app_frames += 1
    assert app_frames == 3
    assert captures == 15


@pytest.mark.parametrize(
    "origins, directions, starts, ends, radii",
    [
        ([[0, 0]], [[1, 0, 0]], [[0, 0, 0]], [[0, 0, 1]], 0.1),
        ([[0, 0, 0]], [[0, 0, 0]], [[0, 0, 0]], [[0, 0, 1]], 0.1),
        ([[0, 0, 0]], [[1, 0, 0]], [[0, 0, 0]], [[0, 0, 0]], 0.1),
        ([[0, 0, 0]], [[1, 0, 0]], [[0, 0, 0]], [[0, 0, 1]], 0.0),
    ],
)
def test_rejects_invalid_geometry(origins, directions, starts, ends, radii):
    with pytest.raises(ValueError):
        ray_capsule_intersection_matrix(origins, directions, starts, ends, radii)
