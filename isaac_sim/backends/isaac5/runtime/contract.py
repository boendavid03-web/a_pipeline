"""Pure Python helpers for the Isaac 5.1 backend contract.

This module intentionally has no Isaac, ROS, NumPy, or GPU dependency so the
backend contract can be checked before starting Kit.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable


def generated_output_path(backend_root: Path, requested: Path | None) -> Path | None:
    """Return an output path and reject writes outside the backend generated tree."""

    generated = (backend_root / "generated").resolve()
    if requested is None:
        return None
    candidate = requested if requested.is_absolute() else generated / requested
    candidate = candidate.resolve()
    if candidate != generated and generated not in candidate.parents:
        raise ValueError(f"output scene must stay below {generated}: {candidate}")
    return candidate


def measured_rate(samples: Iterable[tuple[float, float]]) -> float | None:
    """Measure events per simulated/wall second from (sim, wall) samples."""

    rows = list(samples)
    if len(rows) < 2:
        return None
    delta = rows[-1][1] - rows[0][1]
    if not math.isfinite(delta) or delta <= 0.0:
        return None
    return (len(rows) - 1) / delta


def finite_pose(position: Iterable[float], orientation: Iterable[float]) -> bool:
    values = [*position, *orientation]
    return all(math.isfinite(float(value)) for value in values)


def pass_or_fail(checks: dict[str, bool], failures: Iterable[str] = ()) -> tuple[str, list[str]]:
    reasons = [name for name, passed in checks.items() if not passed]
    reasons.extend(str(reason) for reason in failures if str(reason))
    return ("PASS" if not reasons else "FAIL"), reasons
