"""Pure helpers for the Isaac command/actual-velocity measurement contract.

The bridge and Kit runner deliberately share this tiny module so telemetry can
be audited without importing ROS 2 or Isaac Sim.  In particular, no helper in
this file accepts a command as a substitute for an observed body velocity.
"""

from __future__ import annotations

import math
from typing import Iterable

COMMAND_PROTOCOL_VERSION = 2
ACTUAL_VELOCITY_SOURCES = frozenset(
    {"physx_rigid_body_api", "fixed_tick_pose_difference"}
)


def finite_or_none(value: object) -> float | None:
    """Represent an optional scalar in strict JSON without NaN/Infinity."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def finite_twist(values: Iterable[object]) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("twist must contain three finite values")
    return result


def actual_velocity_from_actuation(actuation: dict[str, object]) -> tuple[float, float, float]:
    """Extract only independently measured velocity from simulator telemetry."""
    source = str(actuation.get("actual_velocity_source", ""))
    if source not in ACTUAL_VELOCITY_SOURCES:
        raise ValueError(f"actual velocity source is unavailable: {source or 'missing'}")
    # Deliberately ignore received_command/applied_command.  This helper is the
    # single bridge boundary used by both /odom.twist and actuation-state truth.
    return finite_twist(actuation.get("actual_velocity", ()))


def world_to_ros_body_twist(
    world_linear_xyz: Iterable[object], world_angular_xyz: Iterable[object], yaw: float,
    stage_up_axis: str, stage_meters_per_unit: float = 1.0,
) -> tuple[float, float, float]:
    """Convert an observed stage/world twist to ROS base_link vx, vy, wz."""
    linear = finite_twist(world_linear_xyz)
    angular = finite_twist(world_angular_xyz)
    if (
        not math.isfinite(float(yaw))
        or stage_up_axis not in {"Y", "Z"}
        or not math.isfinite(float(stage_meters_per_unit))
        or stage_meters_per_unit <= 0.0
    ):
        raise ValueError("yaw, stage_up_axis and stage scale must be valid")
    # stage_to_ros_vector maps Y-up (x,y,z) -> ROS (x,-z,y), and is identity
    # in planar axes for Z-up.  Rotate the resulting world vector into body.
    world_x, world_y = (
        (linear[0], linear[1])
        if stage_up_axis == "Z"
        else (linear[0], -linear[2])
    )
    world_x *= stage_meters_per_unit
    world_y *= stage_meters_per_unit
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * world_x + s * world_y, -s * world_x + c * world_y,
            angular[2] if stage_up_axis == "Z" else angular[1])


def fixed_tick_pose_twist(
    previous: tuple[float, float, float, float] | None,
    current: tuple[float, float, float, float],
    *, max_dt_sec: float, max_speed_mps: float = 5.0,
    max_angular_speed_radps: float = 10.0,
) -> tuple[tuple[float, float, float] | None, str | None]:
    """Return body twist from adjacent fixed simulation ticks, never commands.

    Reset/rollback/duplicate and abnormal intervals are explicitly invalid so a
    relocation cannot become a physically impossible velocity sample.
    """
    if previous is None:
        return None, "first_sample"
    t0, x0, y0, yaw0 = previous
    t1, x1, y1, yaw1 = current
    values = (t0, x0, y0, yaw0, t1, x1, y1, yaw1)
    if not all(math.isfinite(value) for value in values):
        return None, "nonfinite"
    dt = t1 - t0
    if dt <= 0.0:
        return None, "nonpositive_dt"
    if dt > max_dt_sec:
        return None, "abnormal_dt"
    world_x, world_y = (x1 - x0) / dt, (y1 - y0) / dt
    c, s = math.cos(yaw1), math.sin(yaw1)
    yaw_rate = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)) / dt
    if math.hypot(world_x, world_y) > max_speed_mps or abs(yaw_rate) > max_angular_speed_radps:
        return None, "pose_jump"
    return (c * world_x + s * world_y, -s * world_x + c * world_y, yaw_rate), None


def sequence_gap(previous: int | None, current: int) -> int:
    """Count missing sequence values; rollback/restart is not a huge gap."""
    if previous is None or current <= previous:
        return 0
    return max(0, int(current) - int(previous) - 1)
