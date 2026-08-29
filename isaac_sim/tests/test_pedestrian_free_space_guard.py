from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from pedestrian_free_space_guard import (  # noqa: E402
    LastSafePositionGuard,
    SustainedIntrusionTracker,
)


class FakeFreeSpace:
    def __init__(self, safe_points, safe_edges=None):
        self.safe_points = set(safe_points)
        self.safe_edges = (
            {frozenset(edge) for edge in safe_edges}
            if safe_edges is not None
            else None
        )

    def contains_world(self, x, y):
        return (x, y) in self.safe_points

    def segment_world_free(self, start_x, start_y, end_x, end_y):
        if self.safe_edges is None:
            return True
        return frozenset(((start_x, start_y), (end_x, end_y))) in self.safe_edges


class FakeAgent:
    def __init__(self, result=True):
        self.result = result
        self.targets = []

    def teleport(self, *, target):
        self.targets.append(target)
        return self.result


def make_guard(*, free_space, agent=None):
    agent = agent or FakeAgent()
    tracker = SustainedIntrusionTracker(sustained_samples=3)
    guard = LastSafePositionGuard(
        free_space=free_space,
        agents={"a": agent},
        tracker=tracker,
        target_factory=lambda position: tuple(position),
    )
    return guard, tracker, agent


class SustainedIntrusionTrackerTest(unittest.TestCase):
    def test_counts_one_event_per_continuous_intrusion(self) -> None:
        tracker = SustainedIntrusionTracker(sustained_samples=3)

        self.assertEqual(tracker.update({"a": False}).sustained_intrusions, ())
        self.assertEqual(tracker.update({"a": False}).sustained_intrusions, ())
        third = tracker.update({"a": False})
        fourth = tracker.update({"a": False})

        self.assertEqual(third.sustained_intrusions, ("a",))
        self.assertEqual(fourth.sustained_intrusions, ())
        self.assertEqual(tracker.summary()["sustained_intrusion_count"], 1)
        self.assertEqual(tracker.summary()["involved_people"], ["a"])

    def test_safe_sample_rearms_person(self) -> None:
        tracker = SustainedIntrusionTracker(sustained_samples=2)
        tracker.update({"a": False})
        tracker.update({"a": False})
        tracker.update({"a": True})
        tracker.update({"a": False})
        second = tracker.update({"a": False})

        self.assertEqual(second.sustained_intrusions, ("a",))
        self.assertEqual(tracker.summary()["sustained_intrusion_count"], 2)

    def test_tracks_people_independently_and_removes_stale_state(self) -> None:
        tracker = SustainedIntrusionTracker(sustained_samples=2)
        tracker.update({"a": False, "b": True})
        event = tracker.update({"a": False, "b": False})
        tracker.update({"b": False})

        self.assertEqual(event.sustained_intrusions, ("a",))
        self.assertEqual(tracker.summary()["sustained_intrusion_count"], 2)
        self.assertEqual(tracker.summary()["involved_people"], ["a", "b"])
        self.assertEqual(tracker.summary()["active_intrusions"], ["b"])

    def test_rejected_frame_does_not_mutate_state(self) -> None:
        tracker = SustainedIntrusionTracker()
        invalid = (None, [], {"": True}, {1: True}, {"a": 1})
        for frame in invalid:
            with self.subTest(frame=frame):
                with self.assertRaises((TypeError, ValueError)):
                    tracker.update(frame)
        self.assertEqual(tracker.summary()["sample_frames"], 0)

    def test_rejects_invalid_threshold(self) -> None:
        for value in (0, -1, True, 1.5, "3"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    SustainedIntrusionTracker(sustained_samples=value)


class LastSafePositionGuardTest(unittest.TestCase):
    def test_recovers_point_intrusion_and_clears_on_safe_readback(self) -> None:
        guard, tracker, agent = make_guard(
            free_space=FakeFreeSpace({(0.0, 0.0)})
        )
        guard.update({"a": (0.0, 0.0, 0.0)})
        snapshot = guard.update({"a": (3.0, 0.0, 0.0)})

        self.assertEqual(snapshot.unsafe_people, ("a",))
        self.assertEqual(tracker.summary()["unsafe_person_samples"], 1)
        self.assertEqual(guard.recovery_count, 1)
        self.assertEqual(guard.pending_recovery, {"a"})
        self.assertEqual(agent.targets, [(0.0, 0.0, 0.0)])

        guard.update({"a": (0.0, 0.0, 0.0)})
        self.assertEqual(guard.pending_recovery, set())
        self.assertEqual(guard.last_safe_positions["a"], (0.0, 0.0, 0.0))

    def test_unsafe_last_safe_edge_recovers_without_tracker_intrusion(self) -> None:
        guard, tracker, agent = make_guard(
            free_space=FakeFreeSpace(
                {(0.0, 0.0), (1.0, 0.0)},
                safe_edges={((0.0, 0.0), (0.0, 0.0))},
            )
        )
        guard.update({"a": (0.0, 0.0, 0.0)})
        snapshot = guard.update({"a": (1.0, 0.0, 0.0)})

        self.assertEqual(snapshot.unsafe_people, ())
        self.assertEqual(tracker.summary()["unsafe_person_samples"], 0)
        self.assertEqual(agent.targets, [(0.0, 0.0, 0.0)])

    def test_first_unsafe_missing_agent_and_failed_teleport_fail_fast(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "no last-safe"):
            make_guard(free_space=FakeFreeSpace(set()))[0].update(
                {"a": (1.0, 0.0, 0.0)}
            )

        missing_tracker = SustainedIntrusionTracker()
        guard = LastSafePositionGuard(
            free_space=FakeFreeSpace({(0.0, 0.0)}),
            agents={},
            tracker=missing_tracker,
            target_factory=tuple,
        )
        with self.assertRaisesRegex(RuntimeError, "BehaviorAgent missing"):
            guard.update({"a": (1.0, 0.0, 0.0)})
        self.assertEqual(missing_tracker.summary()["unsafe_person_samples"], 1)

        guard, _, _ = make_guard(
            free_space=FakeFreeSpace({(0.0, 0.0)}), agent=FakeAgent(False)
        )
        guard.update({"a": (0.0, 0.0, 0.0)})
        with self.assertRaisesRegex(RuntimeError, "refused"):
            guard.update({"a": (1.0, 0.0, 0.0)})

    def test_unsafe_readback_and_nonfinite_position_fail_fast(self) -> None:
        guard, _, _ = make_guard(free_space=FakeFreeSpace({(0.0, 0.0)}))
        guard.update({"a": (0.0, 0.0, 0.0)})
        guard.update({"a": (1.0, 0.0, 0.0)})
        with self.assertRaisesRegex(RuntimeError, "readback"):
            guard.update({"a": (2.0, 0.0, 0.0)})

        guard, _, _ = make_guard(free_space=FakeFreeSpace({(0.0, 0.0)}))
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            guard.update({"a": (float("nan"), 0.0, 0.0)})

if __name__ == "__main__":
    unittest.main()
