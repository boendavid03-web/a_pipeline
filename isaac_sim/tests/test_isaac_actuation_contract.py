import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from isaac_actuation_contract import (
    actual_velocity_from_actuation,
    finite_or_none,
    fixed_tick_pose_twist,
    sequence_gap,
    world_to_ros_body_twist,
)


def test_optional_telemetry_scalar_never_emits_nonfinite_json_values():
    assert finite_or_none(-math.inf) is None
    assert finite_or_none(math.inf) is None
    assert finite_or_none(math.nan) is None
    assert finite_or_none(None) is None
    assert finite_or_none(1.25) == 1.25


def test_actual_velocity_extraction_never_uses_command_backfill():
    payload = {
        "received_command": [9.0, 8.0, 7.0],
        "applied_command": [6.0, 5.0, 4.0],
        "actual_velocity": [0.3, -0.1, 0.2],
        "actual_velocity_source": "physx_rigid_body_api",
    }
    assert actual_velocity_from_actuation(payload) == (0.3, -0.1, 0.2)
    missing_truth = dict(payload)
    missing_truth.pop("actual_velocity")
    try:
        actual_velocity_from_actuation(missing_truth)
    except ValueError:
        pass
    else:
        raise AssertionError("command values must not substitute for missing actual velocity")


def test_observed_world_velocity_is_rotated_not_copied_from_command():
    # World +x at yaw pi/2 becomes negative body y.  A command value cannot
    # produce this result, which catches command-backfill regressions.
    actual = world_to_ros_body_twist((1.0, 0.0, 0.0), (0.0, 0.0, 0.3), math.pi / 2, "Z")
    assert math.isclose(actual[0], 0.0, abs_tol=1e-9)
    assert math.isclose(actual[1], -1.0, abs_tol=1e-9)
    assert actual[2] == 0.3
    centimetre_stage = world_to_ros_body_twist(
        (100.0, 0.0, 0.0), (0.0, 0.0, 0.3), 0.0, "Z", 0.01
    )
    assert centimetre_stage == (1.0, 0.0, 0.3)


def test_pose_difference_rejects_reset_and_duplicate_time():
    assert fixed_tick_pose_twist(None, (0.0, 0.0, 0.0, 0.0), max_dt_sec=0.1)[0] is None
    assert fixed_tick_pose_twist((1.0, 0.0, 0.0, 0.0), (1.0, 50.0, 0.0, 0.0), max_dt_sec=0.1)[1] == "nonpositive_dt"
    assert fixed_tick_pose_twist((0.0, 0.0, 0.0, 0.0), (0.05, 50.0, 0.0, 0.0), max_dt_sec=0.1)[1] == "pose_jump"
    twist, reason = fixed_tick_pose_twist((0.0, 0.0, 0.0, 0.0), (0.05, 0.1, 0.0, 0.0), max_dt_sec=0.1)
    assert reason is None and math.isclose(twist[0], 2.0)
    assert sequence_gap(7, 10) == 2
    assert sequence_gap(10, 1) == 0
