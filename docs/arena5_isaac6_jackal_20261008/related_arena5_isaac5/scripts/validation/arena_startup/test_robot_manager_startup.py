"""Deterministic Isaac Task Generator startup/goal ownership tests.

Run with the frozen host overlay: source [LOCAL_PATH]
then /usr/bin/python3 -m unittest discover -s scripts/validation/arena_startup -v.
"""

import time
import unittest
from types import SimpleNamespace as NS

from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid

from task_generator.constants import Constants
from task_generator.manager.robot_manager.robot_manager import RobotManager
from task_generator.shared import Pose


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.value is not None:
            callback(self)

    def set_result(self, value):
        self.value = value
        for callback in list(self.callbacks):
            callback(self)

    def result(self):
        return self.value


class Handle:
    def __init__(self, uuid):
        self.accepted = True
        self.goal_id = NS(uuid=bytes.fromhex(uuid))
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class Action:
    def __init__(self):
        self.requests = []
        self.futures = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, request):
        self.requests.append(request)
        future = Future()
        self.futures.append(future)
        return future


class Logger:
    def info(self, *_):
        pass

    def warn(self, *_):
        pass

    def error(self, *_):
        pass


def transform(x, y, stamp):
    return NS(
        header=NS(stamp=NS(sec=stamp, nanosec=0)),
        transform=NS(translation=NS(x=x, y=y)),
    )


class RobotManagerStartupTest(unittest.TestCase):
    def setUp(self):
        self.owner = RobotManager.__new__(RobotManager)
        self.owner.node = NS(
            conf=NS(Arena=NS(SIM=NS(value=Constants.SimSimulator.ISAAC))),
            get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=101))),
            get_logger=lambda: NS(get_child=lambda _: Logger()),
        )
        self.owner._robot = NS(record_data_dir=None)
        self.owner._pose = Pose.parse((25.3, 1.25, 0.7))
        self.owner._start_pos = self.owner._pose
        self.owner._goal_pos = Pose.parse((6, 21.8, 0))
        self.owner._goal_timer = None
        self.owner._is_goal_reached = False
        self.owner._goal_action_running = False
        self.owner._robot_radius = 0.267
        self.owner._isaac_generation = 1
        self.owner._isaac_goal_sent = False
        self.owner._isaac_pending_send = None
        self.owner._isaac_active_handle = None
        self.owner._isaac_active_uuid = None
        self.owner._isaac_retiring = {}
        self.owner._isaac_goal_pos = self.owner._goal_pos
        self.owner._isaac_expected_start = self.owner._pose
        self.owner._isaac_reset_wall = time.monotonic() - 1
        self.owner._isaac_last_wait_log = 0.0
        self.owner._isaac_reset_odom_stamp = 10_000_000_000
        self.owner._isaac_reset_tf_stamp = 10_000_000_000
        self.owner._isaac_last_odom_stamp = 12_000_000_000
        self.owner._isaac_last_odom_arrival = time.monotonic()
        self.owner._isaac_lifecycle_names = ('bt_navigator', 'planner_server', 'controller_server')
        self.owner._isaac_lifecycle_clients = {}
        self.owner._isaac_lifecycle_futures = {}
        self.owner._isaac_lifecycle_active = {}
        self.owner._isaac_action_client = Action()
        self.current_tf = transform(-10, -10, 12)
        self.owner._isaac_robot_transform = lambda: self.current_tf
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'map'
        costmap.info.width = 626
        costmap.info.height = 481
        costmap.info.resolution = 0.05
        self.owner._isaac_costmap = costmap
        self.owner._isaac_costmap_arrival = time.monotonic()
        self.costmap = costmap

    def activate_nav2(self):
        self.owner._isaac_lifecycle_active = dict.fromkeys(
            self.owner._isaac_lifecycle_names, True)

    def test_readiness_rejects_placeholder_stale_and_outside_pose(self):
        self.activate_nav2()
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.current_tf = transform(25.3, 1.25, 10)  # pre-reset TF
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_last_odom_arrival = self.owner._isaac_reset_wall - 0.1
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.owner._isaac_last_odom_arrival = time.monotonic()
        self.owner._isaac_costmap.info.width = 100  # real robot now outside
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.owner._isaac_costmap.info.width = 626
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)

    def test_costmap_must_arrive_after_reset(self):
        self.activate_nav2()
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_costmap_arrival = self.owner._isaac_reset_wall - 0.1
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.owner._isaac_costmap_callback(self.costmap)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)

    def test_one_owned_goal_terminal_abort_never_retries(self):
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)
        self.activate_nav2()
        self.owner._isaac_goal_tick()
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)
        handle = Handle('01' * 16)
        self.owner._isaac_action_client.futures[0].set_result(handle)
        self.assertEqual(self.owner._isaac_active_uuid, '01' * 16)
        self.assertTrue(self.owner._goal_action_running)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)
        handle.result_future.set_result(NS(status=GoalStatus.STATUS_ABORTED))
        self.assertIsNone(self.owner._isaac_active_uuid)
        self.assertFalse(self.owner.is_done)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)

    def test_reset_cancels_old_goal_and_ignores_stale_result(self):
        self.activate_nav2()
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_goal_tick()
        old = Handle('02' * 16)
        self.owner._isaac_action_client.futures[0].set_result(old)
        self.owner._isaac_begin_episode()
        self.assertEqual(old.cancel_calls, 1)
        self.assertIsNone(self.owner._isaac_active_uuid)
        self.assertEqual(self.owner._isaac_generation, 2)
        self.owner._isaac_goal_pos = self.owner._goal_pos
        self.owner._isaac_expected_start = self.owner._pose
        self.owner._isaac_reset_odom_stamp = 12_000_000_000
        self.owner._isaac_reset_tf_stamp = 12_000_000_000
        self.owner._isaac_last_odom_stamp = 13_000_000_000
        self.owner._isaac_last_odom_arrival = time.monotonic()
        self.current_tf = transform(25.3, 1.25, 13)
        self.activate_nav2()
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)
        old.result_future.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
        self.assertFalse(self.owner.is_done)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)
        self.owner._isaac_costmap_callback(self.costmap)
        self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 2)
        new = Handle('03' * 16)
        self.owner._isaac_action_client.futures[1].set_result(new)
        new.result_future.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
        self.assertTrue(self.owner.is_done)
        self.assertIsNone(self.owner._isaac_active_uuid)

    def test_reset_while_send_pending_cancels_late_acceptance(self):
        self.activate_nav2()
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_goal_tick()
        self.owner._isaac_begin_episode()
        late = Handle('04' * 16)
        self.owner._isaac_action_client.futures[0].set_result(late)
        self.assertEqual(late.cancel_calls, 1)
        self.assertFalse(self.owner.is_done)

    def test_isaac_goal_pose_path_does_not_publish(self):
        publisher = NS(publish=lambda _: self.fail('goal_pose publisher used'))
        self.owner._goal_pub = publisher
        goal = Pose.parse((2, 3, 0))
        self.owner._publish_goal(goal)
        self.assertIs(self.owner._isaac_goal_pos, goal)
        self.assertEqual(len(self.owner._isaac_action_client.requests), 0)

    def test_rejected_goal_is_terminal_for_episode(self):
        self.activate_nav2()
        self.current_tf = transform(25.3, 1.25, 12)
        self.owner._isaac_goal_tick()
        self.owner._isaac_action_client.futures[0].set_result(NS(accepted=False))
        for _ in range(3):
            self.owner._isaac_goal_tick()
        self.assertEqual(len(self.owner._isaac_action_client.requests), 1)
        self.assertFalse(self.owner.is_done)

    def test_stale_lifecycle_callback_cannot_activate_new_episode(self):
        future = Future()
        self.owner._isaac_lifecycle_futures['bt_navigator'] = future
        self.owner._isaac_begin_episode()
        future.set_result(NS(current_state=NS(id=3)))
        self.owner._isaac_lifecycle_response('bt_navigator', 1, future)
        self.assertNotIn('bt_navigator', self.owner._isaac_lifecycle_active)

    def test_non_isaac_topic_and_status_path_remains(self):
        published = []
        self.owner.node.conf.Arena.SIM.value = Constants.SimSimulator.GAZEBO
        self.owner.node.get_clock = lambda: NS(
            now=lambda: NS(nanoseconds=101_000_000_000,
                           to_msg=lambda: Time(sec=101)))
        self.owner.node.create_timer = lambda *_: NS(cancel=lambda: None, destroy=lambda: None)
        self.owner._goal_pub = NS(publish=published.append)
        self.owner._publish_goal(Pose.parse((2, 3, 0)))
        self.assertEqual(len(published), 1)
        self.assertIsNotNone(self.owner._goal_timer)
        self.owner._goal_status_callback(GoalStatusArray(status_list=[
            GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)]))
        self.assertTrue(self.owner.is_done)


if __name__ == '__main__':
    unittest.main()
