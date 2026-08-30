from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from pedestrian_social import (  # noqa: E402
    PedestrianMotionState,
    PedestrianSocialForceController,
    RobotMotionState,
    SocialForceParameters,
    SocialQualityTracker,
    SocialYieldPlanner,
    oriented_box_clearance,
    resolve_social_mode,
)


def pedestrian(position, velocity, direction, speed=1.0):
    return PedestrianMotionState(position, velocity, direction, speed)


class PedestrianSocialForceControllerTest(unittest.TestCase):
    def make_controller(self, **overrides):
        return PedestrianSocialForceController(
            SocialForceParameters(smoothing_time_sec=0.01, **overrides)
        )

    def test_distant_people_have_negligible_social_force(self) -> None:
        controller = self.make_controller()
        result = controller.update(
            {
                "a": pedestrian((0.0, 0.0), (1.0, 0.0), (1.0, 0.0)),
                "b": pedestrian((20.0, 0.0), (-1.0, 0.0), (-1.0, 0.0)),
            },
            None,
            0.1,
        )

        self.assertEqual(result["a"].human_social_component_mps2, (0.0, 0.0))
        self.assertEqual(result["b"].human_social_component_mps2, (0.0, 0.0))

    def test_head_on_people_slow_and_choose_stable_opposite_lateral_sides(self) -> None:
        controller = self.make_controller()
        result = controller.update(
            {
                "a": pedestrian((0.0, 0.0), (1.0, 0.0), (1.0, 0.0)),
                "b": pedestrian((2.0, 0.0), (-1.0, 0.0), (-1.0, 0.0)),
            },
            None,
            0.1,
        )

        self.assertLess(result["a"].human_social_component_mps2[0], 0.0)
        self.assertGreater(result["b"].human_social_component_mps2[0], 0.0)
        self.assertLess(result["a"].final_desired_velocity_mps[1], 0.0)
        self.assertGreater(result["b"].final_desired_velocity_mps[1], 0.0)
        self.assertLess(result["a"].speed_command_mps, 1.0)
        self.assertLess(result["b"].speed_command_mps, 1.0)

    def test_side_crossing_produces_lateral_steering(self) -> None:
        controller = self.make_controller()
        result = controller.update(
            {
                "eastbound": pedestrian(
                    (0.0, 0.0), (1.0, 0.0), (1.0, 0.0)
                ),
                "southbound": pedestrian(
                    (1.0, 1.0), (0.0, -1.0), (0.0, -1.0)
                ),
            },
            None,
            0.1,
        )["eastbound"]

        self.assertLess(result.human_social_component_mps2[1], 0.0)
        self.assertLess(result.final_desired_velocity_mps[1], 0.0)

    def test_exact_overlap_remains_finite(self) -> None:
        controller = self.make_controller()
        result = controller.update(
            {
                "a": pedestrian((0.0, 0.0), (0.0, 0.0), (1.0, 0.0)),
                "b": pedestrian((0.0, 0.0), (0.0, 0.0), (-1.0, 0.0)),
            },
            None,
            0.1,
        )

        for output in result.values():
            values = (
                *output.human_social_component_mps2,
                *output.final_desired_velocity_mps,
                output.speed_command_mps,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertGreaterEqual(output.speed_command_mps, 0.15)

    def test_distant_robot_has_zero_force(self) -> None:
        controller = self.make_controller()
        result = controller.update(
            {"a": pedestrian((20.0, 0.0), (-1.0, 0.0), (-1.0, 0.0))},
            RobotMotionState((0.0, 0.0), (0.0, 0.0), 0.0, (0.3, 0.25)),
            0.1,
        )["a"]

        self.assertEqual(result.robot_social_component_mps2, (0.0, 0.0))
        self.assertEqual(
            result.robot_personal_space_component_mps2, (0.0, 0.0)
        )

    def test_robot_personal_space_force_grows_continuously(self) -> None:
        robot = RobotMotionState((0.0, 0.0), (0.0, 0.0), 0.0, (0.3, 0.25))
        magnitudes = []
        violations = []
        for x in (1.5, 1.0, 0.8):
            output = self.make_controller().update(
                {"a": pedestrian((x, 0.0), (-1.0, 0.0), (-1.0, 0.0))},
                robot,
                0.1,
            )["a"]
            magnitudes.append(math.hypot(*output.robot_personal_space_component_mps2))
            violations.append(output.robot_personal_space_violation)

        self.assertLess(magnitudes[0], magnitudes[1])
        self.assertLess(magnitudes[1], magnitudes[2])
        self.assertEqual(violations, [False, False, True])

    def test_rotated_robot_uses_oriented_footprint(self) -> None:
        unrotated = oriented_box_clearance(
            (1.0, 0.0), (0.0, 0.0), 0.0, (0.5, 0.2)
        )
        rotated = oriented_box_clearance(
            (1.0, 0.0), (0.0, 0.0), math.pi / 2.0, (0.5, 0.2)
        )

        self.assertAlmostEqual(unrotated[0], 0.5)
        self.assertAlmostEqual(rotated[0], 0.8)
        self.assertAlmostEqual(math.hypot(*rotated[1]), 1.0)

    def test_multiple_neighbors_accumulate_before_bounded_output(self) -> None:
        controller = self.make_controller(max_total_social_accel_mps2=0.5)
        states = {
            "center": pedestrian((0.0, 0.0), (1.0, 0.0), (1.0, 0.0)),
            "front": pedestrian((0.6, 0.0), (-1.0, 0.0), (-1.0, 0.0)),
            "left": pedestrian((0.2, 0.4), (0.0, -1.0), (0.0, -1.0)),
            "right": pedestrian((0.2, -0.4), (0.0, 1.0), (0.0, 1.0)),
        }
        output = controller.update(states, None, 0.2)["center"]

        self.assertGreater(math.hypot(*output.human_social_component_mps2), 0.5)
        self.assertLessEqual(
            math.hypot(*output.applied_social_accel_mps2), 0.5 + 1.0e-9
        )
        self.assertLessEqual(output.speed_command_mps, 1.0)
        self.assertLessEqual(
            math.hypot(*output.final_desired_velocity_mps), 1.0 + 1.0e-9
        )

    def test_social_mode_contract_preserves_legacy_and_opt_in(self) -> None:
        self.assertEqual(resolve_social_mode("legacy"), "legacy")
        self.assertEqual(resolve_social_mode("GAZEBO_SOCIAL"), "gazebo_social")
        with self.assertRaisesRegex(ValueError, "legacy or gazebo_social"):
            resolve_social_mode("continuous")

        with self.assertRaisesRegex(ValueError, r"robot_radius_m \+ agent_radius_m"):
            SocialForceParameters(
                robot_radius_m=0.47,
                agent_radius_m=0.35,
                robot_clearance_m=0.80,
            )

    def test_multistep_head_on_encounter_is_finite_and_side_stable(self) -> None:
        controller = PedestrianSocialForceController(
            SocialForceParameters(smoothing_time_sec=0.1)
        )
        positions = {"a": [-2.0, 0.0], "b": [2.0, 0.0]}
        velocities = {"a": [1.0, 0.0], "b": [-1.0, 0.0]}
        directions = {"a": (1.0, 0.0), "b": (-1.0, 0.0)}
        lateral_signs = {"a": 0, "b": 0}
        sign_flips = {"a": 0, "b": 0}
        minimum_distance = math.inf

        for _ in range(160):
            states = {
                name: pedestrian(
                    tuple(positions[name]),
                    tuple(velocities[name]),
                    directions[name],
                )
                for name in positions
            }
            outputs = controller.update(states, None, 0.05)
            for name, output in outputs.items():
                velocity = output.final_desired_velocity_mps
                sign = 0 if abs(velocity[1]) < 1.0e-8 else math.copysign(1, velocity[1])
                if lateral_signs[name] and sign and sign != lateral_signs[name]:
                    sign_flips[name] += 1
                if sign:
                    lateral_signs[name] = sign
                velocities[name] = list(velocity)
                positions[name][0] += 0.05 * velocity[0]
                positions[name][1] += 0.05 * velocity[1]
                self.assertTrue(all(math.isfinite(value) for value in velocity))
            minimum_distance = min(
                minimum_distance,
                math.dist(positions["a"], positions["b"]),
            )

        self.assertGreater(minimum_distance, 0.45)
        self.assertEqual(sign_flips, {"a": 0, "b": 0})

    def test_multistep_robot_encounter_avoids_oriented_footprint(self) -> None:
        controller = PedestrianSocialForceController(
            SocialForceParameters(smoothing_time_sec=0.1)
        )
        robot = RobotMotionState((0.0, 0.0), (0.0, 0.0), 0.0, (0.35, 0.30))
        position = [-3.0, 0.0]
        velocity = [1.0, 0.0]
        minimum_clearance = math.inf

        for _ in range(160):
            output = controller.update(
                {"a": pedestrian(tuple(position), tuple(velocity), (1.0, 0.0))},
                robot,
                0.05,
            )["a"]
            minimum_clearance = min(
                minimum_clearance, output.robot_footprint_clearance_m
            )
            self.assertLessEqual(
                math.hypot(*output.applied_social_accel_mps2), 4.0 + 1.0e-9
            )
            velocity = list(output.final_desired_velocity_mps)
            position[0] += 0.05 * velocity[0]
            position[1] += 0.05 * velocity[1]

        self.assertGreater(minimum_clearance, 0.53)


class SocialQualityTrackerTest(unittest.TestCase):
    def test_accumulates_frame_and_pair_metrics(self) -> None:
        tracker = SocialQualityTracker(
            personal_space_m=1.0,
            visual_overlap_m=0.5,
        )

        first = tracker.update(
            {
                "char_c": (2.0, 0.0, 9.0),
                "char_a": (0.0, 0.0, 9.0),
                "char_b": (0.4, 0.0, 9.0),
            }
        )
        second = tracker.update(
            {
                "char_a": (0.0, 0.0),
                "char_b": (0.75, 0.0),
                "char_c": (2.0, 0.0),
            }
        )

        self.assertEqual(first.sample_index, 1)
        self.assertEqual(first.pedestrian_count, 3)
        self.assertEqual(first.pair_count, 3)
        self.assertEqual(first.closest_pair, ("char_a", "char_b"))
        self.assertAlmostEqual(first.min_center_distance_m, 0.4)
        self.assertEqual(first.visual_overlap_pairs, 1)
        self.assertEqual(first.personal_space_violation_pairs, 1)
        self.assertTrue(first.has_visual_overlap)
        self.assertEqual(second.visual_overlap_pairs, 0)
        self.assertEqual(second.personal_space_violation_pairs, 1)

        summary = tracker.summary()
        self.assertEqual(summary["sample_frames"], 2)
        self.assertEqual(summary["pair_samples"], 6)
        self.assertEqual(summary["closest_pair"], ["char_a", "char_b"])
        self.assertAlmostEqual(summary["min_center_distance_m"], 0.4)
        self.assertEqual(summary["visual_overlap_frames"], 1)
        self.assertEqual(summary["visual_overlap_pair_samples"], 1)
        self.assertAlmostEqual(summary["visual_overlap_pair_ratio"], 1.0 / 6.0)
        self.assertEqual(summary["personal_space_violation_frames"], 2)
        self.assertEqual(summary["personal_space_violation_pair_samples"], 2)
        self.assertAlmostEqual(
            summary["personal_space_violation_pair_ratio"], 2.0 / 6.0
        )

    def test_threshold_comparisons_are_strict(self) -> None:
        tracker = SocialQualityTracker(
            personal_space_m=1.0,
            visual_overlap_m=0.5,
        )
        visual_boundary = tracker.update({"a": (0, 0), "b": (0.5, 0)})
        personal_boundary = tracker.update({"a": (0, 0), "b": (1.0, 0)})

        self.assertEqual(visual_boundary.visual_overlap_pairs, 0)
        self.assertEqual(visual_boundary.personal_space_violation_pairs, 1)
        self.assertFalse(visual_boundary.has_visual_overlap)
        self.assertEqual(personal_boundary.personal_space_violation_pairs, 0)
        self.assertFalse(personal_boundary.has_personal_space_violation)

    def test_zero_or_one_pedestrian_has_no_pairs(self) -> None:
        tracker = SocialQualityTracker()
        tracker.update({})
        tracker.update({"only": (1.0, 2.0)})

        summary = tracker.summary()
        self.assertEqual(summary["sample_frames"], 2)
        self.assertEqual(summary["pair_samples"], 0)
        self.assertIsNone(summary["min_center_distance_m"])
        self.assertIsNone(summary["closest_pair"])
        self.assertEqual(summary["visual_overlap_frames"], 0)
        self.assertEqual(summary["visual_overlap_pair_ratio"], 0.0)
        self.assertEqual(summary["personal_space_violation_frames"], 0)
        self.assertEqual(summary["personal_space_violation_pair_ratio"], 0.0)

    def test_reset_preserves_configuration_and_clears_samples(self) -> None:
        tracker = SocialQualityTracker(
            personal_space_m=0.9,
            visual_overlap_m=0.4,
        )
        tracker.update({"a": (0.0, 0.0), "b": (0.1, 0.0)})
        tracker.reset()

        self.assertEqual(tracker.visual_overlap_m, 0.4)
        self.assertEqual(tracker.personal_space_m, 0.9)
        self.assertEqual(tracker.summary()["sample_frames"], 0)
        self.assertEqual(tracker.summary()["pair_samples"], 0)


class SocialYieldPlannerTest(unittest.TestCase):
    def test_custom_hysteresis_contract_values(self) -> None:
        planner = SocialYieldPlanner(
            trigger_distance_m=1.25,
            resume_distance_m=1.50,
        )
        started = planner.update({"a": (0.0, 0.0), "b": (1.20, 0.0)})
        held = planner.update({"a": (0.0, 0.0), "b": (1.40, 0.0)})
        resumed = planner.update({"a": (0.0, 0.0), "b": (1.50, 0.0)})

        self.assertEqual(started.begin_yielding, ("b",))
        self.assertEqual(held.end_yielding, ())
        self.assertEqual(resumed.end_yielding, ("b",))

    def test_yields_one_person_and_uses_hysteresis(self) -> None:
        planner = SocialYieldPlanner(
            trigger_distance_m=0.9,
            resume_distance_m=1.1,
        )
        first = planner.update({"a": (0.0, 0.0), "b": (0.8, 0.0)})
        held = planner.update({"a": (0.0, 0.0), "b": (1.0, 0.0)})
        resumed = planner.update({"a": (0.0, 0.0), "b": (1.1, 0.0)})

        self.assertEqual(first.begin_yielding, ("b",))
        self.assertEqual(first.active_yielders, ("b",))
        self.assertEqual(held.begin_yielding, ())
        self.assertEqual(held.end_yielding, ())
        self.assertEqual(resumed.end_yielding, ("b",))
        self.assertEqual(resumed.active_yielders, ())

    def test_active_yielder_prevents_both_people_stopping(self) -> None:
        planner = SocialYieldPlanner()
        planner.update({"a": (0.0, 0.0), "b": (0.8, 0.0)})
        decision = planner.update({"a": (0.0, 0.0), "b": (0.7, 0.0)})

        self.assertEqual(decision.begin_yielding, ())
        self.assertEqual(decision.active_yielders, ("b",))

    def test_rejects_inverted_hysteresis(self) -> None:
        with self.assertRaisesRegex(ValueError, "greater than"):
            SocialYieldPlanner(
                trigger_distance_m=1.0,
                resume_distance_m=1.0,
            )

    def test_rejects_invalid_thresholds(self) -> None:
        invalid = (0.0, -1.0, math.inf, -math.inf, math.nan, "bad", None)
        for value in invalid:
            with self.subTest(visual=value):
                with self.assertRaises(ValueError):
                    SocialQualityTracker(visual_overlap_m=value)
            with self.subTest(personal=value):
                with self.assertRaises(ValueError):
                    SocialQualityTracker(personal_space_m=value)

        with self.assertRaisesRegex(ValueError, "less than or equal"):
            SocialQualityTracker(
                visual_overlap_m=1.1,
                personal_space_m=1.0,
            )

    def test_rejects_invalid_positions_without_mutating_state(self) -> None:
        tracker = SocialQualityTracker()
        invalid_frames = (
            None,
            [(0.0, 0.0)],
            {"": (0.0, 0.0)},
            {1: (0.0, 0.0)},
            {"a": (0.0,)},
            {"a": "0,0"},
            {"a": (math.nan, 0.0)},
            {"a": (0.0, math.inf)},
            {"a": ("bad", 0.0)},
        )
        for frame in invalid_frames:
            with self.subTest(frame=frame):
                with self.assertRaises((TypeError, ValueError)):
                    tracker.update(frame)

        self.assertEqual(tracker.summary()["sample_frames"], 0)
        self.assertEqual(tracker.summary()["pair_samples"], 0)


if __name__ == "__main__":
    unittest.main()
