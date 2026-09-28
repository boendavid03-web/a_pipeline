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
    PedestrianRobotEmergencyStopLatch,
    PredictiveBoundaryCommand,
    PredictiveBoundaryLatch,
    circular_body_net_clearance_m,
    pedestrian_robot_emergency_stop_ownership,
    predictive_boundary_velocity,
    steering_target_from_velocity,
)


class ContinuousSteeringAdapterGeometryTest(unittest.TestCase):
    def make_emergency_stop_latch(self) -> PedestrianRobotEmergencyStopLatch:
        return PedestrianRobotEmergencyStopLatch(
            final_trigger_clearance_m=0.05,
            early_trigger_clearance_m=0.20,
            resume_clearance_m=0.30,
            release_safe_samples=3,
            minimum_closing_speed_mps=0.01,
        )

    def test_body_net_clearance_subtracts_circle_radius(self) -> None:
        self.assertAlmostEqual(circular_body_net_clearance_m(0.40, 0.35), 0.05)
        self.assertAlmostEqual(circular_body_net_clearance_m(0.30, 0.35), -0.05)

    def test_emergency_stop_triggers_at_final_net_clearance(self) -> None:
        latch = self.make_emergency_stop_latch()

        decision = latch.update("person", 0.05, 1.0 / 15.0)

        self.assertTrue(decision.active)
        self.assertTrue(decision.entering)
        self.assertEqual(decision.reason, "final_clearance_threshold")

    def test_emergency_stop_triggers_for_existing_overlap(self) -> None:
        decision = self.make_emergency_stop_latch().update(
            "person", -0.08, 1.0 / 15.0
        )

        self.assertTrue(decision.active)
        self.assertEqual(decision.reason, "final_clearance_threshold")

    def test_emergency_stop_uses_measured_early_braking_margin_only_on_approach(self) -> None:
        latch = self.make_emergency_stop_latch()
        first = latch.update("person", 0.21, 0.1)
        approaching = latch.update("person", 0.19, 0.1)
        other = self.make_emergency_stop_latch()
        other.update("person", 0.18, 0.1)
        not_approaching = other.update("person", 0.19, 0.1)

        self.assertFalse(first.active)
        self.assertTrue(approaching.active)
        self.assertEqual(
            approaching.reason, "evidence_braking_margin_approach"
        )
        self.assertFalse(not_approaching.active)

    def test_emergency_stop_release_requires_hysteresis_and_safe_samples(self) -> None:
        latch = self.make_emergency_stop_latch()
        latch.update("person", 0.05, 0.1)

        below_resume = latch.update("person", 0.29, 0.1)
        first = latch.update("person", 0.31, 0.1)
        dipped = latch.update("person", 0.29, 0.1)
        latch.update("person", 0.31, 0.1)
        latch.update("person", 0.32, 0.1)
        released = latch.update("person", 0.33, 0.1)

        self.assertTrue(below_resume.active)
        self.assertEqual(first.consecutive_safe_samples, 1)
        self.assertEqual(dipped.consecutive_safe_samples, 0)
        self.assertFalse(released.active)
        self.assertTrue(released.leaving)
        self.assertEqual(released.reason, "released_after_consecutive_safe")

    def test_emergency_stop_release_defers_follow_while_yield_owns_motion(self) -> None:
        latch = self.make_emergency_stop_latch()
        entered = latch.update("person", 0.05, 0.1)
        latch.update("person", 0.31, 0.1)
        latch.update("person", 0.32, 0.1)
        released = latch.update("person", 0.33, 0.1)

        stopped = pedestrian_robot_emergency_stop_ownership(
            entered, externally_inhibited=True
        )
        deferred = pedestrian_robot_emergency_stop_ownership(
            released, externally_inhibited=True
        )
        resumed = pedestrian_robot_emergency_stop_ownership(
            released, externally_inhibited=False
        )

        self.assertEqual(stopped.action, "idle_and_set_speed_zero")
        self.assertTrue(stopped.motion_inhibited)
        self.assertEqual(deferred.action, "release_deferred_to_external_owner")
        self.assertTrue(deferred.motion_inhibited)
        self.assertEqual(resumed.action, "resume_follow")
        self.assertFalse(resumed.motion_inhibited)

    def test_emergency_stop_feature_switch_defaults_off(self) -> None:
        source = (
            PROJECT_ROOT / "isaac_sim/scripts/show_warehouse_people_robot_6_0.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            '"ISAAC_PEDESTRIAN_ROBOT_EMERGENCY_STOP", False', source
        )
        self.assertIn(
            "if PEDESTRIAN_ROBOT_EMERGENCY_STOP_ENABLED:", source
        )

    def test_predictive_boundary_latch_requires_consecutive_safe_samples(self) -> None:
        latch = PredictiveBoundaryLatch(release_safe_samples=3)
        constrained = PredictiveBoundaryCommand(
            velocity_mps=(-0.6, 0.8),
            predicted_stop_position_m=(1.0, 0.0),
            stopping_distance_m=0.8,
            constrained=True,
            inward_direction=(-1.0, 0.0),
        )
        safe = PredictiveBoundaryCommand(
            velocity_mps=(0.0, 1.0),
            predicted_stop_position_m=(0.0, 1.0),
            stopping_distance_m=0.2,
            constrained=False,
            inward_direction=(0.0, 0.0),
        )

        self.assertEqual(
            latch.update("a", constrained, current_position_safe=True)[2],
            "prediction",
        )
        first = latch.update("a", safe, current_position_safe=True)
        second = latch.update("a", safe, current_position_safe=True)
        third = latch.update("a", safe, current_position_safe=True)

        self.assertEqual(first, ((-1.0, 0.0), True, "latched_until_consecutive_safe"))
        self.assertEqual(second, first)
        self.assertEqual(third, ((0.0, 1.0), False, "released"))
        self.assertEqual(latch.active_people, ())

    def test_predictive_boundary_latch_yields_to_outside_recovery(self) -> None:
        latch = PredictiveBoundaryLatch(release_safe_samples=3)
        constrained = PredictiveBoundaryCommand(
            velocity_mps=(-1.0, 0.0),
            predicted_stop_position_m=(1.0, 0.0),
            stopping_distance_m=0.8,
            constrained=True,
            inward_direction=(-1.0, 0.0),
        )
        latch.update("a", constrained, current_position_safe=True)

        result = latch.update("a", constrained, current_position_safe=False)

        self.assertEqual(result, ((-1.0, 0.0), False, "outside_recovery_owned"))
        self.assertEqual(latch.active_people, ())

    def test_predictive_boundary_keeps_safe_stopping_trajectory_unchanged(self) -> None:
        command = predictive_boundary_velocity(
            (0.0, 0.0),
            (0.8, 0.2),
            (1.0, 0.0),
            response_time_sec=0.7,
            maximum_deceleration_mps2=4.0,
            boundary_probe_m=0.05,
            segment_is_safe=lambda _start, end: end[0] <= 2.0,
            nearest_safe_point=lambda point: point,
        )

        self.assertFalse(command.constrained)
        self.assertEqual(command.velocity_mps, (0.8, 0.2))
        self.assertAlmostEqual(command.stopping_distance_m, 0.825)

    def test_predictive_boundary_countersteers_measured_outward_motion(self) -> None:
        command = predictive_boundary_velocity(
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 0.0),
            response_time_sec=0.7,
            maximum_deceleration_mps2=4.0,
            boundary_probe_m=0.05,
            segment_is_safe=lambda _start, end: end[0] <= 0.3,
            nearest_safe_point=lambda point: (0.25, point[1]),
        )

        self.assertTrue(command.constrained)
        self.assertLess(command.velocity_mps[0], 0.0)
        self.assertGreater(command.velocity_mps[1], 0.0)
        self.assertAlmostEqual(math.hypot(*command.velocity_mps), 1.0)
        self.assertEqual(command.inward_direction, (-1.0, 0.0))

    def test_predictive_boundary_projects_outward_desired_velocity_inward(self) -> None:
        command = predictive_boundary_velocity(
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
            response_time_sec=0.7,
            maximum_deceleration_mps2=4.0,
            boundary_probe_m=0.05,
            segment_is_safe=lambda _start, end: end[0] <= 0.3,
            nearest_safe_point=lambda point: (0.25, point[1]),
        )

        self.assertTrue(command.constrained)
        self.assertAlmostEqual(command.velocity_mps[0], -1.0)
        self.assertAlmostEqual(command.velocity_mps[1], 0.0)

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

    def test_startup_window_disambiguates_route_seam_near_spawn(self) -> None:
        points = (
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (10.0, 0.0),
            (1.09, 0.0),
            (0.0, 0.0),
        )
        unrestricted = PatrolPolylineCursor(
            points,
            (1.10, 0.0),
            waypoint_reach_m=0.05,
            route_lookahead_m=0.5,
        )
        startup_scoped = PatrolPolylineCursor(
            points,
            (1.10, 0.0),
            waypoint_reach_m=0.05,
            route_lookahead_m=0.5,
            initial_route_window_m=2.0,
        )

        self.assertEqual(unrestricted.target_index, 5)
        self.assertEqual(startup_scoped.target_index, 2)
        self.assertEqual(startup_scoped.summary()["initial_route_window_m"], 2.0)

    def test_reaching_commanded_lookahead_advances_skipped_waypoints(self) -> None:
        cursor = PatrolPolylineCursor(
            [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)],
            (0.0, 0.0),
            waypoint_reach_m=0.2,
            route_lookahead_m=2.1,
        )

        cursor.desired_direction((0.0, 0.0))
        self.assertEqual(cursor.target_index, 1)
        self.assertEqual(cursor.lookahead_index, 3)
        cursor.desired_direction((3.05, 0.0))

        self.assertEqual(cursor.target_index, 4)
        self.assertEqual(cursor.summary()["lookahead_checkpoint_advance_count"], 1)

    def test_rejects_zero_lookahead_and_nonfinite_velocity(self) -> None:
        with self.assertRaisesRegex(ValueError, "lookahead_m"):
            steering_target_from_velocity((0.0, 0.0), (0.6, 0.4), 0.0)
        with self.assertRaisesRegex(ValueError, "desired_velocity_mps"):
            steering_target_from_velocity(
                (0.0, 0.0), (math.inf, 0.4), 1.0
            )


if __name__ == "__main__":
    unittest.main()
