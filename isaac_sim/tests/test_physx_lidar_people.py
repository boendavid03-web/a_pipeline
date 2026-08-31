from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from physx_lidar_people import (  # noqa: E402
    is_ignored_person_query_collider,
    nearest_ray_capsule_intersections,
    ray_capsule_intersection_matrix,
    scene_query_hit_value,
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
