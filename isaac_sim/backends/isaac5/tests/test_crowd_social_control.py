"""Pure-Python Stage A contract tests for the Isaac 5 crowd controller."""

from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

import numpy as np

RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))
from crowd_social_control import CrowdAgent, CrowdSocialConfig, CrowdSocialController, PatrolCursor


def safe(_start, _end):
    return True


class CrowdSocialControlTest(unittest.TestCase):
    def agent(self, name, track, route, speed=0.8):
        start = np.asarray(route[0], dtype=float)
        return CrowdAgent(name, track, PatrolCursor(tuple(route)), speed, start, 0.0)

    def test_seed_independent_deterministic_two_person_head_on_avoidance(self):
        agents = [
            self.agent("person_01", 1, ((-2.0, 0.0), (2.0, 0.0))),
            CrowdAgent("person_02", 2, PatrolCursor(((2.0, 0.0), (-2.0, 0.0))), 0.8, np.asarray((2.0, 0.0)), math.pi),
        ]
        controller = CrowdSocialController(agents)
        minimum = math.inf
        for _ in range(600):
            controller.step(1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), safe)
            minimum = min(minimum, float(np.linalg.norm(agents[0].position_xy - agents[1].position_xy)))
        self.assertGreater(minimum, 0.48)
        self.assertLessEqual(max(controller.summary()["maximum_yaw_rate_radps"]), controller.config.max_yaw_rate_radps + 1e-8)

    def test_crossing_and_single_route_progress(self):
        crossing = [
            self.agent("person_01", 1, ((-2.0, 0.0), (2.0, 0.0))),
            CrowdAgent("person_02", 2, PatrolCursor(((0.0, -2.0), (0.0, 2.0))), 0.8, np.asarray((0.0, -2.0)), math.pi / 2.0),
        ]
        controller = CrowdSocialController(crossing)
        for _ in range(600):
            controller.step(1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), safe)
        self.assertGreater(min(float(np.linalg.norm(a.position_xy - b.position_xy)) for a in crossing for b in crossing if a is not b), 0.48)
        solo = CrowdSocialController([self.agent("person_03", 3, ((0.0, 0.0), (2.0, 0.0)))])
        for _ in range(480):
            solo.step(1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), safe)
        self.assertGreater(solo.summary()["route_progress"][0], 0)

    def test_robot_repulsion_and_acceleration_bounds(self):
        person = self.agent("person_01", 1, ((-2.0, 0.0), (2.0, 0.0)))
        controller = CrowdSocialController([person])
        for _ in range(240):
            controller.step(1.0 / 60.0, (0.0, 0.0), (0.0, 0.0), safe)
        self.assertGreater(abs(float(person.position_xy[1])), 0.02)
        self.assertLessEqual(person.max_acceleration_mps2, controller.config.max_acceleration_mps2 + 1e-8)

    def test_unsafe_segment_holds_without_teleport_then_bounded_cursor_recovery(self):
        person = self.agent("person_01", 1, ((0.0, 0.0), (2.0, 0.0)))
        controller = CrowdSocialController([person], CrowdSocialConfig(stall_recovery_sec=0.1))
        for _ in range(12):
            controller.step(1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), lambda _a, _b: False)
        self.assertTrue(np.allclose(person.position_xy, (0.0, 0.0)))
        self.assertGreaterEqual(person.recovery_count, 1)

    def test_open_route_ping_pongs_without_unvalidated_closing_segment(self):
        person = self.agent("person_01", 1, ((0.0, 0.0), (2.0, 0.0)))
        controller = CrowdSocialController([person])

        def cleared_open_segment(_start, end):
            x, y = end
            return -0.34 <= x <= 2.34 and abs(y) <= 0.34

        for _ in range(900):
            controller.step(
                1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), cleared_open_segment
            )
        summary = controller.summary()
        self.assertGreater(summary["route_progress"][0], 1)
        self.assertGreater(summary["cumulative_path_length_m"][0], 3.0)
        self.assertEqual(summary["free_space_emergency_hold_count"][0], 0)
        self.assertLessEqual(
            summary["maximum_acceleration_mps2"][0],
            controller.config.max_deceleration_mps2 + 1.0e-8,
        )

    def test_diagonal_head_on_uses_route_relative_predictive_pass(self):
        agents = [
            self.agent("person_01", 1, ((0.0, 0.0), (3.0, 3.0)), speed=0.9),
            CrowdAgent(
                "person_02",
                2,
                PatrolCursor(((3.0, 3.0), (0.0, 0.0))),
                0.9,
                np.asarray((3.0, 3.0)),
                -3.0 * math.pi / 4.0,
            ),
        ]
        controller = CrowdSocialController(agents)
        minimum = math.inf
        for _ in range(360):
            controller.step(1.0 / 60.0, (20.0, 20.0), (0.0, 0.0), safe)
            minimum = min(
                minimum,
                float(np.linalg.norm(agents[0].position_xy - agents[1].position_xy)),
            )
        self.assertGreater(minimum, 2.0 * controller.config.agent_radius_m)

    def test_lookahead_acknowledges_a_sharp_corner_it_commanded_past(self):
        # Regression for the live crowd's apparent in-place stepping: the
        # old cursor looked beyond this corner but only accepted a 0.38 m
        # radius hit, so the walker curved away and never advanced target 1.
        route = (
            (0.675, 14.975),
            (3.925, 12.825),
            (3.925, 10.225),
            (3.525, 7.825),
        )
        person = CrowdAgent(
            "person_01",
            1,
            PatrolCursor(route),
            0.8,
            np.asarray(route[0]),
            math.atan2(route[1][1] - route[0][1], route[1][0] - route[0][0]),
        )
        controller = CrowdSocialController(
            [person],
            CrowdSocialConfig(human_repulsion_mps2=0.0, robot_repulsion_mps2=0.0),
        )
        for _ in range(600):
            controller.step(1.0 / 30.0, (99.0, 99.0), (0.0, 0.0), safe)
        self.assertGreaterEqual(person.cursor.route_progress, 2)
        self.assertGreater(person.cumulative_path_m, 10.0)


if __name__ == "__main__":
    unittest.main()
