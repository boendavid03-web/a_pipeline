"""Analytic pedestrian-leg geometry for the PhysX-backed 2D LiDAR.

The physical IRA/BehaviorAgent body colliders remain authoritative for
simulation and avoidance.  This module represents each animated lower leg as
an in-memory finite capsule used only while producing LaserScan ranges.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np


PERSON_QUERY_COLLIDER_SUFFIXES = (
    "/Physics/BodyCollider",
    "/Physics/AvoidanceTrigger",
)


def scene_query_hit_value(hit: object, name: str, default: object = None) -> object:
    """Read a field from either Isaac 6.0 scene-query result representation.

    ``raycast_closest()`` returns a mapping, while ``raycast_all()`` passes a
    ``RaycastHit`` object to its callback.
    """
    if isinstance(hit, Mapping):
        return hit.get(name, default)
    return getattr(hit, name, default)


def is_ignored_person_query_collider(path: object) -> bool:
    """Return whether *path* is an IRA full-body collider hidden from LiDAR."""
    if not isinstance(path, str) or "/World/Characters/" not in path:
        return False
    return path.endswith(PERSON_QUERY_COLLIDER_SUFFIXES)


def _vectors(name: str, values: object) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _radii(values: float | Sequence[float], count: int) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim == 0:
        array = np.full(count, float(array), dtype=float)
    if array.shape != (count,):
        raise ValueError(f"radii must be scalar or have shape ({count},)")
    if not np.all(np.isfinite(array)) or np.any(array <= 0.0):
        raise ValueError("radii must contain only finite positive values")
    return array


def ray_capsule_intersection_matrix(
    ray_origins: object,
    ray_directions: object,
    segment_starts: object,
    segment_ends: object,
    radii: float | Sequence[float],
) -> np.ndarray:
    """Return all forward ray/finite-capsule intersection distances.

    Rays form the rows and capsules form the columns of the returned ``(R,L)``
    matrix.  A miss is ``np.inf``.  Directions are normalized internally, so
    returned values are distances in the same units as the input coordinates.

    The implementation is fully broadcast over rays and capsules.  It tests
    the finite cylinder and the two outward hemispheres separately, avoiding a
    Python loop over either dimension.
    """
    origins = _vectors("ray_origins", ray_origins)
    directions = _vectors("ray_directions", ray_directions)
    starts = _vectors("segment_starts", segment_starts)
    ends = _vectors("segment_ends", segment_ends)
    if origins.shape[0] != directions.shape[0]:
        raise ValueError("ray_origins and ray_directions must have the same length")
    if starts.shape != ends.shape:
        raise ValueError("segment_starts and segment_ends must have the same shape")

    direction_norms = np.linalg.norm(directions, axis=1)
    if np.any(direction_norms <= 1.0e-12):
        raise ValueError("ray_directions must be nonzero")
    directions = directions / direction_norms[:, None]

    axes = ends - starts
    axis_length_sq = np.einsum("li,li->l", axes, axes)
    if np.any(axis_length_sq <= 1.0e-12):
        raise ValueError("capsule segments must be nondegenerate")
    radius = _radii(radii, starts.shape[0])

    origin = origins[:, None, :]
    direction = directions[:, None, :]
    start = starts[None, :, :]
    axis = axes[None, :, :]
    length_sq = axis_length_sq[None, :]
    radius_sq = np.square(radius)[None, :]
    origin_from_start = origin - start

    axis_dot_direction = np.sum(axis * direction, axis=2)
    axis_dot_origin = np.sum(axis * origin_from_start, axis=2)
    direction_dot_origin = np.sum(direction * origin_from_start, axis=2)
    origin_length_sq = np.sum(origin_from_start * origin_from_start, axis=2)

    quadratic_a = length_sq - np.square(axis_dot_direction)
    quadratic_b = length_sq * direction_dot_origin - axis_dot_origin * axis_dot_direction
    quadratic_c = (
        length_sq * origin_length_sq
        - np.square(axis_dot_origin)
        - radius_sq * length_sq
    )
    cylinder_discriminant = np.square(quadratic_b) - quadratic_a * quadratic_c
    cylinder_valid = (quadratic_a > 1.0e-12) & (cylinder_discriminant >= 0.0)
    cylinder_sqrt = np.sqrt(np.maximum(cylinder_discriminant, 0.0))
    safe_cylinder_a = np.where(cylinder_valid, quadratic_a, 1.0)

    candidates: list[np.ndarray] = []
    for numerator in (-quadratic_b - cylinder_sqrt, -quadratic_b + cylinder_sqrt):
        distance = numerator / safe_cylinder_a
        axial = axis_dot_origin + distance * axis_dot_direction
        valid = cylinder_valid & (distance >= 0.0) & (axial >= 0.0) & (axial <= length_sq)
        candidates.append(np.where(valid, distance, np.inf))

    for center, outward_start_cap in ((start, True), (ends[None, :, :], False)):
        origin_from_center = origin - center
        sphere_b = np.sum(direction * origin_from_center, axis=2)
        sphere_c = np.sum(origin_from_center * origin_from_center, axis=2) - radius_sq
        sphere_discriminant = np.square(sphere_b) - sphere_c
        sphere_valid = sphere_discriminant >= 0.0
        sphere_sqrt = np.sqrt(np.maximum(sphere_discriminant, 0.0))
        for distance in (-sphere_b - sphere_sqrt, -sphere_b + sphere_sqrt):
            axial = axis_dot_origin + distance * axis_dot_direction
            cap_valid = axial <= 0.0 if outward_start_cap else axial >= length_sq
            valid = sphere_valid & (distance >= 0.0) & cap_valid
            candidates.append(np.where(valid, distance, np.inf))

    return np.minimum.reduce(candidates)


def nearest_ray_capsule_intersections(
    ray_origins: object,
    ray_directions: object,
    segment_starts: object,
    segment_ends: object,
    radii: float | Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Return each ray's nearest capsule distance and capsule index.

    Misses have distance ``np.inf`` and index ``-1``.
    """
    matrix = ray_capsule_intersection_matrix(
        ray_origins,
        ray_directions,
        segment_starts,
        segment_ends,
        radii,
    )
    indices = np.argmin(matrix, axis=1)
    distances = matrix[np.arange(matrix.shape[0]), indices]
    indices = np.where(np.isfinite(distances), indices, -1)
    return distances, indices
