#!/usr/bin/env python3

import importlib.util
import json
import math
import sys
import unittest
from collections import Counter
from collections import deque
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location(
    "auto_goal_rosbag_scheduler",
    SCRIPTS / "auto_goal_rosbag_scheduler.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class NullLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeClock:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(
            nanoseconds=self.nanoseconds,
            to_msg=lambda: Time(
                sec=int(self.nanoseconds // 1_000_000_000),
                nanosec=int(self.nanoseconds % 1_000_000_000),
            ),
        )


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def complete(self, success):
        self.response = SimpleNamespace(success=success)
        self.callback(self)

    def result(self):
        return self.response


class FakeResetClient:
    def __init__(self):
        self.ready = True
        self.future = None
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.future = FakeFuture()
        return self.future


def odom(x, y, yaw=0.0, linear=0.0, angular=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = linear
    message.twist.twist.angular.z = angular
    return message


class AutoGoalSchedulerHelperTests(unittest.TestCase):
    def make_relocation_scheduler(self):
        scheduler = object.__new__(MODULE.AutoGoalRosbagScheduler)
        scheduler.state = "recording"
        scheduler.done = False
        scheduler.pose = (0.0, 0.0, 0.0)
        scheduler.session_start_ns = 0
        scheduler.odom_velocity = (0.0, 0.0)
        scheduler.cmd_velocity = (0.0, 0.0)
        scheduler.odom_sequence = 0
        scheduler.steady_clock = FakeClock(0)
        scheduler.get_clock = lambda: FakeClock(0)
        scheduler.get_logger = lambda: NullLogger()
        scheduler.reset_client = FakeResetClient()
        scheduler.relocation_backend = "gazebo_set_entity_pose"
        scheduler.isaac_reset_pose_topic = "/isaac/reset_pose"
        scheduler.isaac_reset_pub = None
        scheduler.frame_id = "map"
        scheduler.robot_entity_name = "robot"
        scheduler.robot_reset_service = "/reset"
        scheduler.robot_radius = 0.34
        scheduler.pedestrian_radius = 0.125
        scheduler.stop_linear_threshold = 0.02
        scheduler.stop_angular_threshold = 0.05
        scheduler.pedestrian_truth_timeout_sec = 1.0
        scheduler.relocation_odom_tolerance_m = 0.2
        scheduler.recovery_stop_dwell_sec = 0.5
        scheduler.relocation_service_timeout_sec = 5.0
        scheduler.relocation_odom_timeout_sec = 2.0
        scheduler.relocation_state = None
        scheduler.relocation_target = None
        scheduler.relocation_origin = None
        scheduler.relocation_future = None
        scheduler.relocation_response_odom_sequence = None
        scheduler.relocation_deadline_ns = None
        scheduler.relocation_stopped_since_ns = None
        scheduler.relocation_attempt_count = 0
        scheduler.relocation_count = 0
        scheduler.relocation_failure_count = 0
        scheduler.consecutive_episode_failures = 3
        scheduler.pedestrians = [(10.0, 10.0)]
        scheduler.pedestrian_stamp_ns = 0
        scheduler.progress = deque()
        scheduler.pending_goal = None
        scheduler.goal = None
        scheduler.goal_path_length = None
        scheduler.goal_bucket = None
        scheduler.arrival_since_ns = None
        scheduler.state_start_ns = 0
        scheduler.failure_next_goal_delay_sec = 0.1
        scheduler._publish_episode_reset = lambda: None
        scheduler._enter_cooldown = lambda _delay: setattr(
            scheduler, "state", "cooldown"
        )
        scheduler._complete = lambda outcome, reason: setattr(
            scheduler, "completed", (outcome, reason)
        )
        return scheduler

    def test_quality_quota_is_a_strict_and_gate(self):
        self.assertFalse(MODULE.quality_quota_met(2, 60.0, 3, 60.0))
        self.assertFalse(MODULE.quality_quota_met(3, 59.999, 3, 60.0))
        self.assertTrue(MODULE.quality_quota_met(3, 60.0, 3, 60.0))
        self.assertTrue(MODULE.quality_quota_met(4, 61.0, 3, 60.0))

    def test_real_yaw_progress_is_not_classified_as_stuck(self):
        frozen = [
            (index, 2.0, 2.0, 0.0, 8.0, True)
            for index in range(5)
        ]
        self.assertTrue(
            MODULE.progress_window_is_stuck(
                frozen, 0.2, 0.15, 0.15, 0.5
            )
        )
        turning = [
            (index, 2.0, 2.0, 0.05 * index, 8.0, True)
            for index in range(5)
        ]
        self.assertFalse(
            MODULE.progress_window_is_stuck(
                turning, 0.2, 0.15, 0.15, 0.5
            )
        )

    def test_control_event_requires_matching_active_goal(self):
        finish_calls = []
        fake = SimpleNamespace(
            state="recording",
            goal=(4.0, 5.0),
            resolution=0.05,
            get_logger=lambda: NullLogger(),
            _finish_episode=lambda success, reason: finish_calls.append(
                (success, reason)
            ),
        )
        handler = MODULE.AutoGoalRosbagScheduler._on_control_event
        handler(fake, SimpleNamespace(data="not-json"))
        handler(
            fake,
            SimpleNamespace(
                data=json.dumps(
                    {
                        "schema": "drl_vo_control_event/v1",
                        "event": "actuation_deadlock",
                        "final_goal": [9.0, 9.0],
                    }
                )
            ),
        )
        self.assertEqual(finish_calls, [])
        handler(
            fake,
            SimpleNamespace(
                data=json.dumps(
                    {
                        "schema": "drl_vo_control_event/v1",
                        "event": "actuation_deadlock",
                        "final_goal": [4.0, 5.0],
                    }
                )
            ),
        )
        self.assertEqual(finish_calls, [(False, "actuation_deadlock")])

    def test_failed_episode_continues_in_same_bag(self):
        cooldown_calls = []
        fake = SimpleNamespace(
            state="recording",
            state_start_ns=1_000_000_000,
            goal_bucket="medium",
            goal_path_length=8.0,
            episode_id=3,
            success_count=1,
            failure_count=0,
            successful_duration_sec=12.0,
            discarded_duration_sec=0.0,
            failure_reasons=Counter(),
            static_collision_since_ns=None,
            human_collision_since_ns=None,
            continue_after_episode_failure=True,
            capture_deadline_reached=False,
            failure_next_goal_delay_sec=0.25,
            get_logger=lambda: NullLogger(),
            _now_ns=lambda: 3_000_000_000,
            _publish_event=lambda *_args: None,
            _publish_episode_reset=lambda: None,
            _quality_quota_met=lambda: False,
            _complete=lambda *_args: self.fail(
                "recoverable episode failure must not complete the bag"
            ),
            _enter_cooldown=lambda delay: cooldown_calls.append(delay),
        )
        MODULE.AutoGoalRosbagScheduler._finish_episode(
            fake, False, "actuation_deadlock"
        )
        self.assertEqual(fake.success_count, 1)
        self.assertEqual(fake.failure_count, 1)
        self.assertEqual(fake.successful_duration_sec, 12.0)
        self.assertEqual(fake.discarded_duration_sec, 2.0)
        self.assertEqual(fake.failure_reasons, {"actuation_deadlock": 1})
        self.assertEqual(cooldown_calls, [0.25])

    def test_three_completed_failures_trigger_relocation(self):
        fake = self.make_relocation_scheduler()
        fake.state_start_ns = 1_000_000_000
        fake.goal_bucket = "short"
        fake.goal_path_length = 4.0
        fake.episode_id = 1
        fake.success_count = 0
        fake.failure_count = 0
        fake.successful_duration_sec = 0.0
        fake.discarded_duration_sec = 0.0
        fake.failure_reasons = Counter()
        fake.continue_after_episode_failure = True
        fake.capture_deadline_reached = False
        fake.failure_next_goal_delay_sec = 0.25
        fake.relocation_after_failures = 3
        fake.consecutive_episode_failures = 0
        fake._publish_event = lambda *_args: None
        fake._enter_cooldown = lambda _delay: None
        fake._complete = lambda *_args: self.fail(
            "completed failures should remain recoverable"
        )
        for _ in range(3):
            MODULE.AutoGoalRosbagScheduler._finish_episode(
                fake, False, "stuck_no_progress"
            )
        self.assertEqual(fake.failure_count, 3)
        self.assertEqual(fake.consecutive_episode_failures, 3)
        self.assertEqual(fake.relocation_state, "waiting_service")
        self.assertEqual(fake.relocation_attempt_count, 1)

    def test_success_clears_completed_failure_streak(self):
        fake = SimpleNamespace(
            state="recording",
            state_start_ns=1_000_000_000,
            goal_bucket="short",
            goal_path_length=4.0,
            episode_id=1,
            success_count=0,
            failure_count=2,
            successful_duration_sec=0.0,
            discarded_duration_sec=2.0,
            failure_reasons=Counter(),
            static_collision_since_ns=None,
            human_collision_since_ns=None,
            continue_after_episode_failure=True,
            capture_deadline_reached=False,
            next_goal_delay_sec=0.25,
            consecutive_episode_failures=2,
            get_logger=lambda: NullLogger(),
            _now_ns=lambda: 3_000_000_000,
            _publish_event=lambda *_args: None,
            _publish_episode_reset=lambda: None,
            _quality_quota_met=lambda: False,
            _enter_cooldown=lambda _delay: None,
        )
        MODULE.AutoGoalRosbagScheduler._finish_episode(fake, True, "goal_reached")
        self.assertEqual(fake.consecutive_episode_failures, 0)

    def test_real_fixed_relocation_request_and_settle_state_machine(self):
        scheduler = self.make_relocation_scheduler()
        history = [(8.0, 8.0)]
        scheduler.goal_history = history
        rng = object()
        scheduler.rng = rng
        MODULE.AutoGoalRosbagScheduler._begin_relocation(
            scheduler, "stuck_no_progress"
        )
        self.assertEqual(scheduler.relocation_state, "waiting_service")
        self.assertEqual(scheduler.relocation_target, (2.0, 2.0, 0.0))
        self.assertIs(scheduler.rng, rng)
        self.assertEqual(scheduler.goal_history, history)

        scheduler.cmd_velocity = (0.1, 0.0)
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        self.assertEqual(scheduler.reset_client.requests, [])
        scheduler.cmd_velocity = (0.0, 0.0)
        scheduler.steady_clock.nanoseconds = 2_000_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        scheduler.steady_clock.nanoseconds = 2_500_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        self.assertEqual(scheduler.relocation_state, "service_pending")
        request = scheduler.reset_client.requests[0]
        self.assertEqual(request.entity.name, "robot")
        self.assertEqual(request.entity.type, MODULE.Entity.MODEL)
        self.assertEqual(request.pose.position.x, 2.0)
        self.assertEqual(request.pose.position.y, 2.0)
        self.assertEqual(request.pose.position.z, 0.0)
        self.assertEqual(request.pose.orientation.z, 0.0)
        self.assertEqual(request.pose.orientation.w, 1.0)

        future = scheduler.reset_client.future
        future.complete(True)
        self.assertEqual(scheduler.relocation_state, "waiting_odom")
        MODULE.AutoGoalRosbagScheduler._on_odom(
            scheduler, odom(2.0, 2.0, 2.0 * math.pi - 0.1)
        )
        self.assertEqual(scheduler.relocation_state, "settling")
        scheduler.steady_clock.nanoseconds = 3_000_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        self.assertIsNone(scheduler.relocation_state)
        self.assertEqual(scheduler.relocation_count, 1)

        future.response = SimpleNamespace(success=False)
        MODULE.AutoGoalRosbagScheduler._on_relocation_response(
            scheduler, future, future
        )
        self.assertEqual(scheduler.relocation_count, 1)

    def test_frozen_sim_clock_uses_steady_timeout(self):
        scheduler = self.make_relocation_scheduler()
        scheduler.steady_clock.nanoseconds = 0
        scheduler.relocation_state = "service_pending"
        scheduler.relocation_deadline_ns = 1
        scheduler._relocation_failed = lambda reason: setattr(
            scheduler, "timeout_reason", reason
        )
        scheduler.get_clock = lambda: FakeClock(0)
        scheduler.steady_clock.nanoseconds = 2
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        self.assertEqual(scheduler.timeout_reason, "relocation_service_timeout")

    def test_isaac_relocation_publishes_pose_without_gazebo_service(self):
        scheduler = self.make_relocation_scheduler()
        scheduler.relocation_backend = "isaac_pose_topic"
        scheduler.reset_client = None
        scheduler.isaac_reset_pub = FakePublisher()
        MODULE.AutoGoalRosbagScheduler._begin_relocation(scheduler, "failure")
        scheduler.steady_clock.nanoseconds = 2_000_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        scheduler.steady_clock.nanoseconds = 2_500_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        self.assertEqual(scheduler.relocation_state, "waiting_odom")
        self.assertEqual(len(scheduler.isaac_reset_pub.messages), 1)
        message = scheduler.isaac_reset_pub.messages[0]
        self.assertEqual(message.header.frame_id, "map")
        self.assertEqual(message.pose.position.x, 2.0)
        self.assertEqual(message.pose.position.y, 2.0)
        self.assertEqual(message.pose.orientation.w, 1.0)

    def test_rejected_service_and_late_future_have_no_side_effect(self):
        scheduler = self.make_relocation_scheduler()
        MODULE.AutoGoalRosbagScheduler._begin_relocation(scheduler, "failure")
        scheduler.steady_clock.nanoseconds = 2_000_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        scheduler.steady_clock.nanoseconds = 2_500_000_000
        MODULE.AutoGoalRosbagScheduler._on_relocation_watchdog(scheduler)
        future = scheduler.reset_client.future
        future.complete(False)
        self.assertEqual(scheduler.relocation_failure_count, 1)
        state = scheduler.relocation_state
        future.response = SimpleNamespace(success=True)
        MODULE.AutoGoalRosbagScheduler._on_relocation_response(
            scheduler, future, future
        )
        self.assertEqual(scheduler.relocation_state, state)


    def test_deadline_and_quota_complete_without_relocation(self):
        calls = []
        fake = SimpleNamespace(
            state="recording", state_start_ns=1_000_000_000,
            goal_bucket="short", goal_path_length=4.0, episode_id=1,
            success_count=3, failure_count=0, successful_duration_sec=20.0,
            discarded_duration_sec=0.0, failure_reasons=Counter(),
            static_collision_since_ns=None, human_collision_since_ns=None,
            continue_after_episode_failure=True, capture_deadline_reached=True,
            failure_next_goal_delay_sec=0.25, relocation_after_failures=3,
            consecutive_episode_failures=2, get_logger=lambda: NullLogger(),
            _now_ns=lambda: 3_000_000_000,
            _publish_event=lambda *_args: None,
            _publish_episode_reset=lambda: None,
            _quality_quota_met=lambda: True,
            _complete=lambda *args: calls.append(args),
            _begin_relocation=lambda *_args: self.fail("must not relocate"),
        )
        MODULE.AutoGoalRosbagScheduler._finish_episode(
            fake, False, "stuck_no_progress"
        )
        self.assertEqual(calls[0][0], "complete")


if __name__ == "__main__":
    unittest.main()
