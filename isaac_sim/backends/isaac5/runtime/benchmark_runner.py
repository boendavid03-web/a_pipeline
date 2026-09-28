#!/usr/bin/env python3
"""Pure-Python evaluation records for the Isaac5 related benchmark gate."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    goal_xy: tuple[float, float]


def summarize_episode_metrics(rows: Iterable[dict[str, object]]) -> dict[str, object]:
    rows = list(rows)
    success_count = sum(row.get("status") == "SUCCESS" for row in rows)
    collision_count = sum(row.get("collision_obstacle") is not None for row in rows)
    timeout_count = sum(row.get("failure_reason") == "timeout" for row in rows)
    distances = [float(row["goal_distance_m"]) for row in rows]
    return {
        "episode_count": len(rows),
        "success_count": int(success_count),
        "success_rate": float(success_count / len(rows)) if rows else 0.0,
        "collision_count": int(collision_count),
        "timeout_count": int(timeout_count),
        "mean_goal_distance_m": float(sum(distances) / len(distances)) if distances else None,
    }
