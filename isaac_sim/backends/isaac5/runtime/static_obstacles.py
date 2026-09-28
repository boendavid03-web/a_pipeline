#!/usr/bin/env python3
"""Sample world-frame static obstacle points for HuNav Agent.closest_obs."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Iterable, Sequence


DEFAULT_IGNORED_PREFIXES = (
    "/World/PedestrianColliders/",
    "/World/Pedestrians/",
    "/World/Robot",
    "/World/Mecanum",
)


def _hit_field(hit, name: str, default):
    """Read both PhysX callback objects and legacy dict-shaped test hits."""
    if isinstance(hit, Mapping):
        return hit.get(name, default)
    return getattr(hit, name, default)


def sample_static_obstacle_points(
    query,
    position_xy: Sequence[float],
    *,
    height_m: float = 0.9,
    radius_m: float = 0.25,
    maximum_distance_m: float = 3.0,
    ray_count: int = 16,
    ignored_prefixes: Iterable[str] = DEFAULT_IGNORED_PREFIXES,
) -> list[list[float]]:
    """Return the nearest non-human/non-robot hit along each planar ray."""
    if ray_count < 4 or maximum_distance_m <= 0.0 or radius_m < 0.0:
        raise ValueError("invalid static-obstacle sampling geometry")
    x, y = float(position_xy[0]), float(position_xy[1])
    prefixes = tuple(str(value) for value in ignored_prefixes)
    points: list[list[float]] = []
    start_offset = radius_m + 0.02
    for index in range(ray_count):
        angle = 2.0 * math.pi * index / ray_count
        direction = (math.cos(angle), math.sin(angle), 0.0)
        origin = (
            x + direction[0] * start_offset,
            y + direction[1] * start_offset,
            float(height_m),
        )
        nearest = math.inf

        def collect(hit) -> bool:
            nonlocal nearest
            collision = str(_hit_field(hit, "collision", ""))
            if any(collision.startswith(prefix) for prefix in prefixes):
                return True
            distance = float(_hit_field(hit, "distance", math.inf))
            if math.isfinite(distance) and 0.0 <= distance <= maximum_distance_m:
                nearest = min(nearest, distance)
            return True

        query.raycast_all(origin, direction, maximum_distance_m, collect)
        if math.isfinite(nearest):
            points.append(
                [
                    origin[0] + direction[0] * nearest,
                    origin[1] + direction[1] * nearest,
                ]
            )
    return points


def static_polyline_is_clear(
    query,
    route,
    *,
    clearance_m: float = 0.35,
    ignored_prefixes: Iterable[str] = DEFAULT_IGNORED_PREFIXES,
) -> bool:
    """PhysX clearance for runtime replans, excluding dynamic task actors."""
    prefixes = tuple(str(value) for value in ignored_prefixes)

    def nearest(origin, direction, distance):
        value = math.inf

        def collect(hit) -> bool:
            nonlocal value
            collision = str(_hit_field(hit, "collision", ""))
            if any(collision.startswith(prefix) for prefix in prefixes):
                return True
            hit_distance = float(_hit_field(hit, "distance", math.inf))
            if math.isfinite(hit_distance) and 0.0 <= hit_distance <= distance:
                value = min(value, hit_distance)
            return True

        query.raycast_all(origin, direction, distance, collect)
        return value

    points = [tuple(float(component) for component in point) for point in route]
    for start, end in zip(points[:-1], points[1:]):
        for sample_index in range(9):
            alpha = sample_index / 8.0
            x = (1.0 - alpha) * start[0] + alpha * end[0]
            y = (1.0 - alpha) * start[1] + alpha * end[1]
            if not math.isfinite(nearest((x, y, 2.0), (0.0, 0.0, -1.0), 3.0)):
                return False
            for ray_index in range(16):
                angle = 2.0 * math.pi * ray_index / 16
                if nearest(
                    (x, y, 0.9),
                    (math.cos(angle), math.sin(angle), 0.0),
                    clearance_m,
                ) < clearance_m:
                    return False
    return True
