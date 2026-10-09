"""Native HumanSim adapter must observe motion without re-teleporting the robot."""
import asyncio
import copy
import math
from types import SimpleNamespace

import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from arena_robots.Robot import RobotIdentifier
from arena_runtime._node import NodeInterface
from task_generator.manager.realizer import Realizer
from task_generator.shared import Pose, Position, Robot
from task_generator.simulators.human.arena_humansim.arena_humansim import ArenaHumanSimulator


@pytest.fixture
def adapter():
    messages = []
    buffer = tf2_ros.Buffer()
    node = SimpleNamespace(tf_buffer=buffer, sim_time=Time(seconds=10), get_logger=lambda: rclpy.logging.get_logger('feedback_test'))
    s = ArenaHumanSimulator.__new__(ArenaHumanSimulator)
    NodeInterface.__init__(s, node=node)
    s._realizer = Realizer(Realizer._Configuration(x=5, y=5))
    s._dirty_robots = {}
    s._tracked_robots = {}
    s._robot_feedback_previous = {}
    s._world_state_pub = SimpleNamespace(publish=lambda m: messages.append(copy.deepcopy(m)))
    s._agent_names = {}
    s.possessed_peds = lambda: {}
    robot = Robot(name='jackal', pose=Pose(Position(10, 11)), model=RobotIdentifier.parse('jackal'))
    robot.sim_path = 'env_0/jackal'
    def transform(t, x, y, yaw=0):
        node.sim_time = Time(seconds=t)
        msg = TransformStamped()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'env_0/jackal/base_link'
        msg.header.stamp = node.sim_time.to_msg()
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.rotation.z = math.sin(yaw/2)
        msg.transform.rotation.w = math.cos(yaw/2)
        buffer.set_transform(msg, 'test')
    return s, robot, messages, transform


def test_continuous_tf_updates_pose_velocity_heading_and_time(adapter):
    s, robot, messages, transform = adapter
    transform(10, 10, 11)
    asyncio.run(s._spawn_robot_impl([robot]))
    transform(10.1, 10.02, 11.01, .1)
    s._publish_world_state()
    transform(10.2, 10.04, 11.02, .2)
    s._publish_world_state()
    assert len(messages) == 3
    last = messages[-1].agents[0]
    assert last.pose.x == pytest.approx(5.04)
    assert last.pose.y == pytest.approx(6.02)
    assert last.pose.theta == pytest.approx(.2)
    assert last.velocity.x == pytest.approx(.2)
    assert last.velocity.y == pytest.approx(.1)
    assert messages[-1].header.stamp != messages[0].header.stamp
    assert last.agent_id == messages[0].agents[0].agent_id
    assert last.kind == 1


def test_remove_robot_stops_observation(adapter):
    s, robot, messages, transform = adapter
    transform(10, 10, 11)
    asyncio.run(s._spawn_robot_impl([robot]))
    asyncio.run(s._remove_robot_impl([robot]))
    before = len(messages)
    transform(10.1, 10.1, 11)
    s._publish_world_state()
    assert len(messages) == before


def test_reset_pose_does_not_report_teleport_as_velocity(adapter):
    s, robot, messages, transform = adapter
    transform(10, 10, 11)
    asyncio.run(s._spawn_robot_impl([robot]))
    transform(10.1, 10.1, 11)
    s._publish_world_state()
    robot.pose = Pose(Position(20, 21))
    asyncio.run(s._move_robot_impl([robot]))
    transform(10.2, 20, 21)
    s._publish_world_state()
    assert messages[-1].agents[0].pose.x == 15
    assert messages[-1].agents[0].velocity.x == 0
