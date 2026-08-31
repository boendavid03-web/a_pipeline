"""Analytic pedestrian-leg geometry for the PhysX-backed 2D LiDAR.

The physical IRA/BehaviorAgent body colliders remain authoritative for
simulation and avoidance.  This module represents each animated lower leg as
an in-memory finite capsule used only while producing LaserScan ranges.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np


PERSON_QUERY_COLLIDER_SUFFIXES = (
    "/Physics/BodyCollider",
    "/Physics/AvoidanceTrigger",
)

# The current Isaac Sim 6.0.1 native RaycastSensor produces endpoint/hit WORLD
# geometry agreeing to about 1.1 um in full-frame measurements.  Fifty um leaves
# roughly 47x headroom for float and transform roundoff while remaining 20x
# tighter than the former 1 mm depth/endpoint diagnostic threshold.
ENDPOINT_HIT_WORLD_TOLERANCE_M = 5.0e-5


def endpoint_ranges_from_world_geometry(
    ray_end_points_world: object,
    ray_origins_world: object,
    per_beam_start_offset_m: object,
    stage_meters_per_unit: float,
) -> np.ndarray:
    """Return ranges from authoritative endpoint geometry plus ray offsets."""
    endpoints = np.asarray(ray_end_points_world, dtype=float)
    origins = np.asarray(ray_origins_world, dtype=float)
    offsets = np.asarray(per_beam_start_offset_m, dtype=float).reshape(-1)
    if endpoints.ndim != 2 or endpoints.shape[1] != 3:
        raise ValueError("ray_end_points_world must have shape (N, 3)")
    if origins.shape != endpoints.shape:
        raise ValueError("ray_origins_world must match ray_end_points_world")
    if offsets.shape != (endpoints.shape[0],):
        raise ValueError("per_beam_start_offset_m must have shape (N,)")
    if not math.isfinite(stage_meters_per_unit) or stage_meters_per_unit <= 0.0:
        raise ValueError("stage_meters_per_unit must be finite and positive")
    if np.any(~np.isfinite(origins)) or np.any(~np.isfinite(offsets)):
        raise ValueError("ray origins and start offsets must be finite")
    return (
        np.linalg.norm(endpoints - origins, axis=1) * stage_meters_per_unit
        + offsets
    )


def native_depth_diagnostic(
    depths: object,
    endpoint_ranges_m: object,
    per_beam_start_offset_m: object,
    stage_meters_per_unit: float,
    *,
    disagreement_threshold_m: float = 1.0e-3,
) -> dict[str, float | int | None]:
    """Summarize native depths without making disagreement a safety failure."""
    depth_values = np.asarray(depths, dtype=float).reshape(-1)
    endpoint_ranges = np.asarray(endpoint_ranges_m, dtype=float).reshape(-1)
    offsets = np.asarray(per_beam_start_offset_m, dtype=float).reshape(-1)
    if (
        endpoint_ranges.shape != depth_values.shape
        or offsets.shape != depth_values.shape
    ):
        raise ValueError("depth, endpoint range, and offset arrays must have equal length")
    if not math.isfinite(stage_meters_per_unit) or stage_meters_per_unit <= 0.0:
        raise ValueError("stage_meters_per_unit must be finite and positive")
    if not math.isfinite(disagreement_threshold_m) or disagreement_threshold_m <= 0.0:
        raise ValueError("disagreement_threshold_m must be finite and positive")

    depths_m = depth_values * stage_meters_per_unit
    depth_ranges = depths_m + offsets
    finite_depths = depths_m[np.isfinite(depths_m)]
    finite = np.isfinite(depth_ranges) & np.isfinite(endpoint_ranges)
    errors = np.abs(depth_ranges[finite] - endpoint_ranges[finite])
    return {
        "finite_count": int(errors.size),
        "disagreement_count": int(np.count_nonzero(errors > disagreement_threshold_m)),
        "disagreement_threshold_m": float(disagreement_threshold_m),
        "error_median_m": float(np.median(errors)) if errors.size else None,
        "error_p95_m": float(np.percentile(errors, 95.0)) if errors.size else None,
        "error_max_m": float(np.max(errors)) if errors.size else None,
        "depth_unique_count": int(np.unique(finite_depths).size),
        "depth_min_m": float(np.min(finite_depths)) if finite_depths.size else None,
        "depth_max_m": float(np.max(finite_depths)) if finite_depths.size else None,
        "depth_std_m": float(np.std(finite_depths)) if finite_depths.size else None,
    }


def endpoint_hit_world_diagnostic(
    ray_end_points_world: object,
    hit_positions_sensor: object,
    hit_prim_paths: object,
    sensor_local_to_world: object,
    stage_meters_per_unit: float,
    *,
    tolerance_m: float = ENDPOINT_HIT_WORLD_TOLERANCE_M,
) -> dict[str, float | int | None]:
    """Validate endpoint positions against valid native hits in WORLD space."""
    endpoints = np.asarray(ray_end_points_world, dtype=float)
    hit_positions = np.asarray(hit_positions_sensor, dtype=float)
    paths = np.asarray(hit_prim_paths, dtype=object).reshape(-1)
    transform = np.asarray(sensor_local_to_world, dtype=float)
    if endpoints.ndim != 2 or endpoints.shape[1] != 3:
        raise ValueError("ray_end_points_world must have shape (N, 3)")
    if hit_positions.shape != endpoints.shape:
        raise ValueError("hit_positions must match ray_end_points_world")
    if paths.shape != (endpoints.shape[0],):
        raise ValueError("hit_prim_paths must have shape (N,)")
    if transform.shape != (4, 4) or np.any(~np.isfinite(transform)):
        raise ValueError("sensor_local_to_world must be a finite 4x4 matrix")
    if not math.isfinite(stage_meters_per_unit) or stage_meters_per_unit <= 0.0:
        raise ValueError("stage_meters_per_unit must be finite and positive")
    if not math.isfinite(tolerance_m) or tolerance_m <= 0.0:
        raise ValueError("tolerance_m must be finite and positive")

    valid_hit = np.asarray(
        [path is not None and bool(str(path)) for path in paths], dtype=bool
    )
    if np.any(valid_hit & ~np.all(np.isfinite(hit_positions), axis=1)):
        raise RuntimeError("native PhysX valid hit has non-finite hit_position")
    world_hits = (
        hit_positions[valid_hit] @ transform[:3, :3] + transform[3, :3]
    )
    errors = (
        np.linalg.norm(endpoints[valid_hit] - world_hits, axis=1)
        * stage_meters_per_unit
    )
    if np.any(~np.isfinite(errors)):
        raise RuntimeError("native PhysX endpoint/hit WORLD error is non-finite")
    stats: dict[str, float | int | None] = {
        "valid_hit_count": int(errors.size),
        "tolerance_m": float(tolerance_m),
        "error_median_m": float(np.median(errors)) if errors.size else None,
        "error_p95_m": float(np.percentile(errors, 95.0)) if errors.size else None,
        "error_max_m": float(np.max(errors)) if errors.size else None,
    }
    if errors.size and float(np.max(errors)) > tolerance_m:
        raise RuntimeError(
            "native PhysX endpoint/hit WORLD geometry inconsistency: "
            f"max_m={float(np.max(errors)):.9f}, tolerance_m={tolerance_m:.9f}"
        )
    return stats


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


ROBOT_QUERY_COLLIDER_ROOTS = (
    "/World/Robot",
    "/World/RobotCollisionProxy",
)


def is_ignored_robot_query_collider(path: object) -> bool:
    """Return whether *path* belongs to the LiDAR-carrying robot itself."""
    if not isinstance(path, str):
        return False
    return any(
        path == root or path.startswith(root + "/")
        for root in ROBOT_QUERY_COLLIDER_ROOTS
    )


def ray_start_offsets_outside_box(
    sensor_xy: object,
    ray_directions_xy: object,
    box_half_extents_xy: object,
    minimum_offset: float,
    *,
    epsilon: float = 0.01,
) -> np.ndarray:
    """Return per-ray starts beyond an enclosing robot XY box.

    Only rays whose mount is inside the robot proxy are moved beyond the exact
    forward box exit.  This prevents native readings from reporting robot self
    hits while preserving the configured minimum range for every other ray.
    """
    sensor = np.asarray(sensor_xy, dtype=float).reshape(-1)
    directions = np.asarray(ray_directions_xy, dtype=float)
    half_extents = np.asarray(box_half_extents_xy, dtype=float).reshape(-1)
    if sensor.shape != (2,) or half_extents.shape != (2,):
        raise ValueError("sensor_xy and box_half_extents_xy must each have shape (2,)")
    if directions.ndim != 2 or directions.shape[1] != 2:
        raise ValueError("ray_directions_xy must have shape (N, 2)")
    if not np.all(np.isfinite(sensor)) or not np.all(np.isfinite(directions)):
        raise ValueError("sensor and ray directions must be finite")
    if not np.all(np.isfinite(half_extents)) or np.any(half_extents <= 0.0):
        raise ValueError("box half extents must be finite and positive")
    if not math.isfinite(minimum_offset) or minimum_offset < 0.0:
        raise ValueError("minimum_offset must be finite and non-negative")
    if not math.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError("epsilon must be finite and positive")
    norms = np.linalg.norm(directions, axis=1)
    if np.any(norms <= 1.0e-12):
        raise ValueError("ray directions must be nonzero")
    directions = directions / norms[:, None]
    if np.any(np.abs(sensor) >= half_extents):
        raise ValueError("sensor_xy must be strictly inside the box")
    boundaries = np.where(directions >= 0.0, half_extents, -half_extents)
    with np.errstate(divide="ignore", invalid="ignore"):
        axis_exit = (boundaries - sensor) / directions
    axis_exit[np.abs(directions) <= 1.0e-12] = np.inf
    exit_offsets = np.min(axis_exit, axis=1)
    if np.any(~np.isfinite(exit_offsets)) or np.any(exit_offsets < 0.0):
        raise ValueError("could not compute a finite forward box exit")
    return np.maximum(float(minimum_offset), exit_offsets + float(epsilon))


def physics_capture_due(
    current_sim_time: float,
    next_capture_sim_time: float,
    period_sec: float,
) -> tuple[bool, float, int]:
    """Advance a simulation-clock capture schedule without app-frame coupling."""
    if (
        not math.isfinite(current_sim_time)
        or current_sim_time < 0.0
        or not math.isfinite(period_sec)
        or period_sec <= 0.0
        or not math.isfinite(next_capture_sim_time)
    ):
        raise ValueError("capture schedule values must be finite")
    due_count = 0
    while next_capture_sim_time <= current_sim_time + 1.0e-9:
        due_count += 1
        next_capture_sim_time += period_sec
    return due_count > 0, next_capture_sim_time, max(0, due_count - 1)


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
