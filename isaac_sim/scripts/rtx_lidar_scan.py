#!/usr/bin/env python3
"""Project Isaac RTX GenericModelOutput returns into a ROS 2D LaserScan grid."""

from __future__ import annotations

import math
from collections.abc import Sequence


def project_rtx_returns(
    x_values: Sequence[float],
    y_values: Sequence[float],
    z_values: Sequence[float],
    scalar_values: Sequence[float],
    *,
    cartesian: bool,
    sample_count: int = 360,
    angle_min: float = -math.pi,
    angle_increment: float = 2.0 * math.pi / 360.0,
    range_min: float = 0.5,
    range_max: float = 50.0,
) -> tuple[list[float | None], list[float], dict[str, float | int]]:
    """Return range/intensity slots from native RTX lidar returns.

    ``scalar_values`` is the native normalized RTX lidar intensity.  ROS's
    Isaac RTX LaserScan publisher represents it on the conventional 0..255
    scale, so this projector does the same.  LaserScan stores float32 values;
    keep that precision instead of first converting to uint8, because valid
    warehouse returns can be below 1/255 and would otherwise all become zero.
    When several native returns land in one output angle slot, the nearest
    return and its matching intensity win.  Missing returns remain ``None``
    with zero intensity; the external ROS bridge converts ``None`` to
    ``+inf``.
    """

    if sample_count <= 0:
        raise ValueError("sample_count must be positive")
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        raise ValueError("scan angles must be finite")
    if angle_increment <= 0.0:
        raise ValueError("angle_increment must be positive")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("scan range limits are invalid")
    lengths = {
        len(x_values),
        len(y_values),
        len(z_values),
        len(scalar_values),
    }
    if len(lengths) != 1:
        raise ValueError("RTX coordinate and intensity arrays must have equal lengths")

    ranges: list[float | None] = [None] * sample_count
    intensities = [0.0] * sample_count
    accepted = 0
    clipped_intensities = 0
    native_min = math.inf
    native_max = -math.inf

    for raw_x, raw_y, raw_z, raw_scalar in zip(
        x_values, y_values, z_values, scalar_values
    ):
        x = float(raw_x)
        y = float(raw_y)
        z = float(raw_z)
        scalar = float(raw_scalar)
        if cartesian:
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            azimuth = math.atan2(y, x)
            distance = math.sqrt(x * x + y * y + z * z)
        else:
            # Isaac GMO spherical output follows its ROS publisher contract:
            # x is azimuth in degrees and z is range in metres.
            if not math.isfinite(x) or not math.isfinite(z):
                continue
            azimuth = math.radians(x)
            distance = z
        if not range_min <= distance <= range_max:
            continue

        slot = int((azimuth - angle_min) / angle_increment)
        slot = min(sample_count - 1, max(0, slot))
        current = ranges[slot]
        if current is not None and distance >= current:
            continue

        if math.isfinite(scalar):
            native_min = min(native_min, scalar)
            native_max = max(native_max, scalar)
            bounded = min(1.0, max(0.0, scalar))
            if bounded != scalar:
                clipped_intensities += 1
            intensity = bounded * 255.0
        else:
            intensity = 0.0
        ranges[slot] = distance
        intensities[slot] = intensity
        accepted += 1

    finite_intensities = [value for value in intensities if value > 0.0]
    stats: dict[str, float | int] = {
        "native_returns": len(x_values),
        "accepted_returns": accepted,
        "finite_slots": sum(value is not None for value in ranges),
        "nonzero_intensity_slots": len(finite_intensities),
        "clipped_intensities": clipped_intensities,
        "native_intensity_min": native_min if native_min != math.inf else 0.0,
        "native_intensity_max": native_max if native_max != -math.inf else 0.0,
        "ros_intensity_min": min(finite_intensities) if finite_intensities else 0.0,
        "ros_intensity_max": max(finite_intensities) if finite_intensities else 0.0,
    }
    return ranges, intensities, stats
