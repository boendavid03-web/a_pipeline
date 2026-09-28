#!/usr/bin/env python3
"""Deterministic Mecanum/scared-person interaction profile and acceptance checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence


PROFILE_NAME = "cross_stop_leave"
NARROW_PROFILE_NAME = "narrow_block_wait"
MINIMUM_PROFILE_DURATION_SEC = 25.0


@dataclass(frozen=True)
class RobotCommandPhase:
    name: str
    start_sec: float
    end_sec: float
    linear_mps: float
    angular_radps: float


PHASES = (
    RobotCommandPhase("unapproached_route_observation", 0.0, 2.5, 0.0, 0.0),
    RobotCommandPhase("crossing_and_slow_approach", 2.5, 7.5, 0.30, 0.0),
    RobotCommandPhase("sudden_stop_and_static_block", 7.5, 9.5, 0.0, 0.0),
    RobotCommandPhase("turn_to_leave", 9.5, 12.75, 0.0, 1.0),
    RobotCommandPhase("leave_interaction_area", 12.75, 20.75, 0.30, 0.0),
    RobotCommandPhase("post_departure_recovery_observation", 20.75, math.inf, 0.0, 0.0),
)

NARROW_PHASES = (
    RobotCommandPhase("unapproached_narrow_route_observation", 0.0, 2.50, 0.0, 0.0),
    RobotCommandPhase("narrow_corridor_static_block", 2.50, 8.00, 0.0, 0.0),
    RobotCommandPhase("turn_to_clear_narrow_corridor", 8.00, 9.60, 0.0, 1.0),
    RobotCommandPhase("leave_narrow_corridor", 9.60, 15.60, 0.25, 0.0),
    RobotCommandPhase("narrow_corridor_recovery_observation", 15.60, math.inf, 0.0, 0.0),
)

PROFILES = {
    PROFILE_NAME: PHASES,
    NARROW_PROFILE_NAME: NARROW_PHASES,
}


def command_for_time(
    sim_time: float, profile_name: str = PROFILE_NAME
) -> tuple[str, tuple[float, float]]:
    """Return the named bounded command phase for simulation time."""
    value = float(sim_time)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    try:
        phases = PROFILES[profile_name]
    except KeyError as error:
        raise ValueError(f"unsupported Gate 6 profile: {profile_name}") from error
    for phase in phases:
        if phase.start_sec <= value < phase.end_sec:
            return phase.name, (phase.linear_mps, phase.angular_radps)
    raise AssertionError("the final command phase must be unbounded")


def evaluate_narrow_wait(
    *,
    observed_phases: Sequence[str],
    maximum_wait_sec: float,
    minimum_rectangle_clearance_m: float,
    route_progress_at_block_end_m: float,
    final_route_progress_m: float,
    robot_travel_m: float,
) -> dict[str, bool]:
    phase_set = set(observed_phases)
    return {
        "gate6_narrow_all_command_phases_observed": {
            phase.name for phase in NARROW_PHASES
        }.issubset(phase_set),
        "gate6_narrow_robot_entered_and_left": float(robot_travel_m) >= 0.60,
        "gate6_narrow_safe_wait_observed": 0.50 <= float(maximum_wait_sec) <= 5.0,
        "gate6_narrow_rectangle_collision_free": float(minimum_rectangle_clearance_m) >= 0.0,
        "gate6_narrow_route_recovered_after_clearance": (
            float(final_route_progress_m) >= float(route_progress_at_block_end_m) + 0.10
        ),
    }


def evaluate_interaction(
    *,
    behaviors: Iterable[str],
    scared_rows: Sequence[Mapping[str, object]],
    observed_phases: Sequence[str],
    robot_travel_m: float,
) -> dict[str, bool]:
    """Evaluate only direct Gate 6 interaction claims; ordinary gates stay separate."""
    behavior_set = {str(value) for value in behaviors}
    phase_set = set(observed_phases)
    required_phases = {phase.name for phase in PHASES}
    rows = list(scared_rows)

    triggered = bool(rows) and all(row.get("trigger_time") is not None for row in rows)
    outward = bool(rows) and all(
        float(row.get("maximum_away_radial_velocity_during_interaction_mps", -math.inf)) >= 0.05
        for row in rows
    )
    collision_free = bool(rows) and all(
        float(row.get("minimum_robot_rectangle_net_clearance_m", -math.inf)) >= 0.0
        for row in rows
    )
    recovered = bool(rows) and all(
        row.get("recovery_time") is not None and bool(row.get("original_route_recovered"))
        for row in rows
    )
    progress_before = bool(rows) and all(
        float(row.get("route_progress_at_trigger_m") or 0.0) >= 0.10 for row in rows
    )
    progress_after = bool(rows) and all(
        float(row.get("final_route_progress_m") or 0.0)
        >= float(row.get("route_progress_at_trigger_m") or 0.0) + 0.10
        for row in rows
    )
    departed = bool(rows) and all(
        float(row.get("maximum_robot_distance_after_trigger_m", 0.0)) >= 2.20
        for row in rows
    )
    return {
        "gate6_regular_and_scared_present": "regular" in behavior_set and "scared" in behavior_set,
        "gate6_all_command_phases_observed": required_phases.issubset(phase_set),
        "gate6_robot_crossed_stopped_and_left": float(robot_travel_m) >= 1.0,
        "gate6_scared_triggered": triggered,
        "gate6_scared_walked_original_route_before_trigger": progress_before,
        "gate6_scared_moved_away_from_robot": outward,
        "gate6_robot_rectangle_collision_free": collision_free,
        "gate6_robot_departed_interaction_range": departed,
        "gate6_scared_recovered_original_route": recovered,
        "gate6_scared_continued_original_waypoint": progress_after,
    }
