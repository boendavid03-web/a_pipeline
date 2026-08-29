from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from pedestrian_social import SocialQualityTracker, SocialYieldPlanner  # noqa: E402


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
