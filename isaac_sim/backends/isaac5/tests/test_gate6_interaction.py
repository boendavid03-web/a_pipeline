import math
import sys
from pathlib import Path


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))

from gate6_interaction import (
    NARROW_PHASES,
    NARROW_PROFILE_NAME,
    PHASES,
    command_for_time,
    evaluate_interaction,
    evaluate_narrow_wait,
)


def test_gate6_profile_has_static_cross_stop_leave_and_recovery_phases() -> None:
    assert [command_for_time(phase.start_sec)[0] for phase in PHASES] == [
        phase.name for phase in PHASES
    ]
    assert command_for_time(0.0)[1] == (0.0, 0.0)
    assert command_for_time(3.0)[1] == (0.30, 0.0)
    assert command_for_time(8.0)[1] == (0.0, 0.0)
    assert command_for_time(10.0)[1] == (0.0, 1.0)
    assert command_for_time(14.0)[1] == (0.30, 0.0)
    assert command_for_time(24.0)[1] == (0.0, 0.0)


def test_gate6_profile_rejects_invalid_time() -> None:
    for value in (-0.1, math.nan, math.inf):
        try:
            command_for_time(value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid time {value}")


def test_narrow_profile_turns_enters_blocks_and_leaves() -> None:
    assert [command_for_time(phase.start_sec, NARROW_PROFILE_NAME)[0] for phase in NARROW_PHASES] == [
        phase.name for phase in NARROW_PHASES
    ]
    assert command_for_time(5.0, NARROW_PROFILE_NAME)[1] == (0.0, 0.0)
    assert command_for_time(13.0, NARROW_PROFILE_NAME)[1] == (0.25, 0.0)


def test_narrow_wait_acceptance_requires_wait_clearance_and_recovery() -> None:
    checks = evaluate_narrow_wait(
        observed_phases=[phase.name for phase in NARROW_PHASES],
        maximum_wait_sec=1.0,
        minimum_rectangle_clearance_m=0.1,
        route_progress_at_block_end_m=1.0,
        final_route_progress_m=1.5,
        robot_travel_m=0.8,
    )
    assert all(checks.values())


def _passing_row() -> dict[str, object]:
    return {
        "trigger_time": 4.0,
        "maximum_away_radial_velocity_during_interaction_mps": 0.2,
        "minimum_robot_rectangle_net_clearance_m": 0.1,
        "recovery_time": 22.0,
        "route_progress_at_trigger_m": 0.4,
        "final_route_progress_m": 1.0,
        "maximum_robot_distance_after_trigger_m": 2.5,
        "original_route_recovered": True,
    }


def test_gate6_interaction_acceptance_is_strict_and_complete() -> None:
    checks = evaluate_interaction(
        behaviors=("regular", "scared"),
        scared_rows=[_passing_row()],
        observed_phases=[phase.name for phase in PHASES],
        robot_travel_m=1.4,
    )
    assert checks and all(checks.values())

    failed = _passing_row()
    failed["original_route_recovered"] = False
    checks = evaluate_interaction(
        behaviors=("regular", "scared"),
        scared_rows=[failed],
        observed_phases=[phase.name for phase in PHASES],
        robot_travel_m=1.4,
    )
    assert checks["gate6_scared_recovered_original_route"] is False
