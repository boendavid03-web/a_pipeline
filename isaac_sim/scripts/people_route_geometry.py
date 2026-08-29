"""Shared continuous geometry predicates for Isaac pedestrian routes."""

from __future__ import annotations

import math
from collections.abc import Sequence

from convert_gazebo_boxes_to_usda import Box


DEFAULT_LOBBY_BOUNDS = (0.0, 0.0, 32.0, 24.0)


def point_within_clear_bounds(
    point: tuple[float, float],
    bounds: tuple[float, float, float, float],
    clearance: float,
) -> bool:
    """Return whether a point is strictly inside bounds after clearance."""
    x, y = point
    x_min, y_min, x_max, y_max = bounds
    return (
        x_min + clearance < x < x_max - clearance
        and y_min + clearance < y < y_max - clearance
    )


def segment_intersects_expanded_box(
    start: tuple[float, float],
    end: tuple[float, float],
    box: Box,
    clearance: float,
) -> bool:
    """Return whether a segment touches an oriented box expanded by clearance."""
    cosine = math.cos(box.pose.yaw)
    sine = math.sin(box.pose.yaw)

    def local(point: tuple[float, float]) -> tuple[float, float]:
        dx, dy = point[0] - box.pose.x, point[1] - box.pose.y
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    sx, sy = local(start)
    ex, ey = local(end)
    half_x = 0.5 * box.size[0] + clearance
    half_y = 0.5 * box.size[1] + clearance
    enter, leave = 0.0, 1.0
    for origin, delta, lower, upper in (
        (sx, ex - sx, -half_x, half_x),
        (sy, ey - sy, -half_y, half_y),
    ):
        if abs(delta) < 1.0e-12:
            if origin < lower or origin > upper:
                return False
            continue
        first = (lower - origin) / delta
        second = (upper - origin) / delta
        if first > second:
            first, second = second, first
        enter = max(enter, first)
        leave = min(leave, second)
        if enter > leave:
            return False
    return True


def edge_is_continuously_safe(
    start: tuple[float, float],
    end: tuple[float, float],
    boxes: Sequence[Box],
    bounds: tuple[float, float, float, float] = DEFAULT_LOBBY_BOUNDS,
    clearance: float = 0.55,
) -> bool:
    """Check bounds and exact OBB clearance for an entire route edge."""
    return (
        point_within_clear_bounds(start, bounds, clearance)
        and point_within_clear_bounds(end, bounds, clearance)
        and not any(
            segment_intersects_expanded_box(start, end, box, clearance)
            for box in boxes
        )
    )
