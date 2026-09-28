"""Pure geometry helpers for the global-path-to-local-subgoal ROS node."""
from __future__ import annotations

import math
import numpy as np


def finite_xy(xy):
    point = np.asarray(xy, dtype=np.float64)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError("path point must contain finite x/y")
    return point


def choose_lookahead(path_xy, robot_xy, lookahead_distance):
    """Return the nearest valid point index and arc-length lookahead point."""
    if not math.isfinite(float(lookahead_distance)) or float(lookahead_distance) < 0:
        raise ValueError("lookahead_distance must be finite and non-negative")
    points = np.asarray(path_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] != 2:
        raise ValueError("path must be non-empty Nx2")
    if not np.isfinite(points).all():
        raise ValueError("path contains NaN or Inf")
    robot = finite_xy(robot_xy)
    nearest = int(np.argmin(np.sum((points - robot) ** 2, axis=1)))
    remaining = float(lookahead_distance)
    current = points[nearest].copy()
    for next_point in points[nearest + 1:]:
        segment = next_point - current
        length = float(np.linalg.norm(segment))
        if length > 1e-12 and remaining <= length:
            return nearest, current + segment * (remaining / length)
        remaining -= length
        current = next_point.copy()
    return nearest, points[-1].copy()


def map_to_base(point_map, robot_xy_yaw):
    point = finite_xy(point_map)
    pose = np.asarray(robot_xy_yaw, dtype=np.float64)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError("robot pose must be finite x/y/yaw")
    dx, dy = point - pose[:2]
    cosine, sine = math.cos(float(pose[2])), math.sin(float(pose[2]))
    return np.array([cosine * dx + sine * dy, -sine * dx + cosine * dy])
