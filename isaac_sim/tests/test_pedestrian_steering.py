from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from pedestrian_steering import (  # noqa: E402
    PatrolPolylineCursor,
    steering_target_from_velocity,
)


class ContinuousSteeringAdapterGeometryTest(unittest.TestCase):
    def test_complete_velocity_enters_target_without_lateral_projection(self) -> None:
        command = steering_target_from_velocity(
            position_m=(2.0, 3.0),
            desired_velocity_mps=(0.6, 0.4),
            lookahead_m=1.0,
        )

        speed = math.hypot(0.6, 0.4)
        self.assertAlmostEqual(command.speed_mps, speed)
        self.assertAlmostEqual(command.direction[0], 0.6 / speed)
        self.assertAlmostEqual(command.direction[1], 0.4 / speed)
        self.assertGreater(command.target_offset_m[1], 0.0)
        self.assertAlmostEqual(
            command.target_offset_m[1] / command.target_offset_m[0],
            0.4 / 0.6,
        )
        self.assertAlmostEqual(
            command.target_position_m[1], 3.0 + 0.4 / speed
        )

    def test_opposite_lateral_commands_produce_opposite_navigation_targets(self) -> None:
        left = steering_target_from_velocity((0.0, 0.0), (0.8, 0.3), 1.2)
        right = steering_target_from_velocity((0.0, 0.0), (0.8, -0.3), 1.2)

        self.assertAlmostEqual(left.target_position_m[0], right.target_position_m[0])
        self.assertAlmostEqual(left.target_position_m[1], -right.target_position_m[1])
        self.assertNotEqual(left.target_position_m[1], 0.0)

    def test_cyclic_patrol_advances_without_per_waypoint_stop_command(self) -> None:
        cursor = PatrolPolylineCursor(
            [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
            (0.0, 0.0),
            waypoint_reach_m=0.2,
            route_lookahead_m=1.1,
        )

        first = cursor.desired_direction((0.0, 0.0))
        second = cursor.desired_direction((1.05, 0.0))

        self.assertGreater(first[0], 0.0)
        self.assertGreater(second[0], 0.0)
        self.assertGreater(cursor.summary()["advance_count"], 0)

    def test_route_lookahead_stops_before_an_invisible_corner_cut(self) -> None:
        cursor = PatrolPolylineCursor(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)],
            (0.9, -0.1),
            waypoint_reach_m=0.05,
            route_lookahead_m=2.0,
        )

        direction = cursor.desired_direction(
            (0.9, -0.1),
            segment_is_safe=lambda _start, end: end != (2.0, 1.0),
        )

        expected_length = math.hypot(0.1, 1.1)
        self.assertAlmostEqual(direction[0], 0.1 / expected_length)
        self.assertAlmostEqual(direction[1], 1.1 / expected_length)
        self.assertEqual(cursor.summary()["lookahead_index"], 2)
        self.assertTrue(cursor.summary()["last_visibility_limited"])
        self.assertEqual(cursor.summary()["visibility_limited_count"], 1)

    def test_route_lookahead_is_unchanged_without_a_visibility_guard(self) -> None:
        cursor = PatrolPolylineCursor(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)],
            (0.9, -0.1),
            waypoint_reach_m=0.05,
            route_lookahead_m=2.0,
        )

        direction = cursor.desired_direction((0.9, -0.1))

        expected_length = math.hypot(1.1, 1.1)
        self.assertAlmostEqual(direction[0], 1.1 / expected_length)
        self.assertAlmostEqual(direction[1], 1.1 / expected_length)
        self.assertEqual(cursor.summary()["lookahead_index"], 3)
        self.assertFalse(cursor.summary()["last_visibility_limited"])

    def test_invisible_current_target_uses_visible_local_predecessor(self) -> None:
        cursor = PatrolPolylineCursor(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)],
            (1.0, 0.0),
            waypoint_reach_m=0.2,
            route_lookahead_m=1.0,
        )

        direction = cursor.desired_direction(
            (0.0, 0.0),
            segment_is_safe=lambda _start, end: end != (1.0, 1.0),
        )

        self.assertEqual(cursor.target_index, 2)
        self.assertEqual(cursor.lookahead_index, 1)
        self.assertEqual(direction, (1.0, 0.0))
        self.assertTrue(cursor.last_visibility_limited)

    def test_rejects_zero_lookahead_and_nonfinite_velocity(self) -> None:
        with self.assertRaisesRegex(ValueError, "lookahead_m"):
            steering_target_from_velocity((0.0, 0.0), (0.6, 0.4), 0.0)
        with self.assertRaisesRegex(ValueError, "desired_velocity_mps"):
            steering_target_from_velocity(
                (0.0, 0.0), (math.inf, 0.4), 1.0
            )


if __name__ == "__main__":
    unittest.main()
