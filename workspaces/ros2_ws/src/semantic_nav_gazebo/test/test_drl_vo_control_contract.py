#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from drl_vo_control_contract import (  # noqa: E402
    ActuationSample,
    actuation_deadlock_detected,
    final_goal_rearms_after_reset,
)


def sample(stamp_ns, angular=0.105, x=2.0, yaw=0.0):
    return ActuationSample(stamp_ns, 0.0, angular, x, 2.0, yaw)


class DrlVoControlContractTests(unittest.TestCase):
    def detect(self, samples):
        return actuation_deadlock_detected(
            samples, 2.5, 0.8, 0.02, 0.05, 0.02, 0.03
        )

    def test_exact_window_boundary(self):
        self.assertFalse(self.detect([sample(0), sample(2_499_999_999)]))
        self.assertTrue(self.detect([sample(0), sample(2_500_000_000)]))

    def test_command_ratio_is_not_dead_configuration(self):
        four_active = [
            sample(index * 625_000_000, 0.0 if index == 2 else 0.105)
            for index in range(5)
        ]
        three_active = [
            sample(
                index * 625_000_000,
                0.0 if index in (1, 2) else 0.105,
            )
            for index in range(5)
        ]
        self.assertTrue(self.detect(four_active))
        self.assertFalse(self.detect(three_active))

    def test_real_translation_or_yaw_response_prevents_deadlock(self):
        translated = [sample(0), sample(2_500_000_000, x=2.02)]
        turning = [sample(0), sample(2_500_000_000, yaw=0.03)]
        self.assertFalse(self.detect(translated))
        self.assertFalse(self.detect(turning))

    def test_zero_command_ratio_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "minimum_command_ratio"):
            actuation_deadlock_detected(
                [sample(0), sample(2_500_000_000)],
                2.5,
                0.0,
                0.02,
                0.05,
                0.02,
                0.03,
            )

    def test_reset_goal_rearm_requires_a_genuinely_new_goal(self):
        self.assertFalse(
            final_goal_rearms_after_reset([1.0, 2.0], [1.0, 2.0])
        )
        self.assertFalse(
            final_goal_rearms_after_reset(
                [1.0, 2.0], [1.0 + 5e-5, 2.0]
            )
        )
        self.assertTrue(
            final_goal_rearms_after_reset(
                [1.0, 2.0], [1.0 + 2e-4, 2.0]
            )
        )
        self.assertTrue(final_goal_rearms_after_reset(None, [1.0, 2.0]))
        self.assertFalse(
            final_goal_rearms_after_reset([1.0, 2.0], [float("nan"), 2.0])
        )


if __name__ == "__main__":
    unittest.main()
