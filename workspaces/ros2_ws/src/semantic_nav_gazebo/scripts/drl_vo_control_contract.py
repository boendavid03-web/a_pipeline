#!/usr/bin/env python3
"""Pure control-supervision safety contracts shared by DRL-VO runtime/tests."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ActuationSample:
    stamp_ns: int
    command_linear: float
    command_angular: float
    x: float
    y: float
    yaw: float


def final_goal_rearms_after_reset(
    reset_goal,
    incoming_goal,
    tolerance_m: float = 1e-4,
) -> bool:
    """Return whether a finite incoming goal is genuinely new after reset.

    Replaying the transient-local final goal during an episode reset must keep
    control inhibited.  Only a spatially different goal may re-arm actions.
    """

    tolerance = float(tolerance_m)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("tolerance_m must be finite and non-negative")
    try:
        incoming = tuple(float(value) for value in incoming_goal)
    except (TypeError, ValueError):
        return False
    if len(incoming) != 2 or not all(math.isfinite(value) for value in incoming):
        return False
    if reset_goal is None:
        return True
    try:
        previous = tuple(float(value) for value in reset_goal)
    except (TypeError, ValueError):
        return False
    if len(previous) != 2 or not all(math.isfinite(value) for value in previous):
        return False
    return math.hypot(
        incoming[0] - previous[0], incoming[1] - previous[1]
    ) > tolerance


def actuation_deadlock_detected(
    samples,
    minimum_window_sec: float,
    minimum_command_ratio: float,
    linear_command_threshold: float,
    angular_command_threshold: float,
    maximum_displacement_m: float,
    maximum_yaw_progress_rad: float,
) -> bool:
    """Detect persistent command/pose disagreement in a causal window."""

    if not 0.0 < float(minimum_command_ratio) <= 1.0:
        raise ValueError("minimum_command_ratio must be in (0,1]")

    history = list(samples)
    if len(history) < 2:
        return False
    elapsed_sec = (history[-1].stamp_ns - history[0].stamp_ns) / 1e9
    if elapsed_sec + 1e-12 < float(minimum_window_sec):
        return False
    active_ratio = sum(
        abs(item.command_linear) >= float(linear_command_threshold)
        or abs(item.command_angular) >= float(angular_command_threshold)
        for item in history
    ) / len(history)
    first = history[0]
    displacement = max(
        math.hypot(item.x - first.x, item.y - first.y) for item in history
    )
    yaw_progress = max(
        abs(
            math.atan2(
                math.sin(item.yaw - first.yaw),
                math.cos(item.yaw - first.yaw),
            )
        )
        for item in history
    )
    return (
        active_ratio >= float(minimum_command_ratio)
        and displacement < float(maximum_displacement_m)
        and yaw_progress < float(maximum_yaw_progress_rad)
    )
