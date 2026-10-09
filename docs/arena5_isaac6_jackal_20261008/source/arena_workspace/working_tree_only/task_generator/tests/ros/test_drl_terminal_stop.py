"""A completed/canceled DRL phase must stop an action already in flight."""
import asyncio
from types import SimpleNamespace

from arena_planners.bridge.edge_node import PlannerEdgeNode
from arena_planners.bridge.protocol import Action, CancelAck
from geometry_msgs.msg import PoseStamped
from task_generator.shared import Pose, Position
from task_generator.tasks.robots.adapters.mobile.drl import DrlAdapter
from task_generator.tasks.robots.request import GoToPhase
from task_generator.manager.robot_manager.robot_manager import RobotManager
from task_generator.node import TaskGenerator


def edge():
    node = PlannerEdgeNode.__new__(PlannerEdgeNode)
    messages = []
    node._driving = True
    node._cmd_vel_pub = SimpleNamespace(publish=messages.append)
    return node, messages


def test_completed_phase_stops_before_late_policy_action():
    node, messages = edge()
    adapter = DrlAdapter.__new__(DrlAdapter)
    adapter._edge_node = node
    phase = GoToPhase(Pose(Position(3, 4)), tolerance_radius=.25, tolerance_angle=0)
    robot = SimpleNamespace(pose=Pose(Position(3.1, 4)), controls_orientation=False,
                            node=SimpleNamespace(conf=SimpleNamespace(Robot=SimpleNamespace())))
    assert adapter.is_phase_done(phase, robot) is True
    node._publish_action(Action(seq=1, action_type='differential_drive', action=[.8, .2]), {})
    assert len(messages) >= 2
    assert all(m.linear.x == 0 and m.angular.z == 0 for m in messages)


def test_unreached_phase_does_not_stop():
    node, messages = edge()
    adapter = DrlAdapter.__new__(DrlAdapter)
    adapter._edge_node = node
    phase = GoToPhase(Pose(Position(3, 4)), tolerance_radius=.25, tolerance_angle=0)
    robot = SimpleNamespace(pose=Pose(Position(2, 4)), controls_orientation=False,
                            node=SimpleNamespace(conf=SimpleNamespace(Robot=SimpleNamespace())))
    assert adapter.is_phase_done(phase, robot) is None
    assert node._driving and not messages


def test_cancel_publishes_stop_before_waiting_for_slow_sdk_ack():
    async def run():
        node, messages = edge()
        ack = asyncio.Event()
        node._send_control = lambda frame: None
        async def drain(kind):
            await ack.wait()
            return CancelAck()
        node._drain_until = drain
        task = asyncio.create_task(node.request_cancel())
        await asyncio.sleep(0)
        try:
            assert not node._driving and len(messages) == 1
            assert messages[0].linear.x == 0
        finally:
            ack.set()
            await task
    asyncio.run(run())


def test_terminal_action_result_stops_phase():
    async def run():
        node, messages = edge()
        adapter = DrlAdapter.__new__(DrlAdapter)
        adapter._edge_node = node
        adapter._run_loop_task = None
        adapter._phase_to_pose_stamped = lambda phase, robot: PoseStamped()
        adapter._resolve_tolerances = lambda phase, robot: (.25, 0.)
        async def reset(**kwargs): pass
        node.request_reset = reset
        future = asyncio.get_running_loop().create_future()
        handle = SimpleNamespace(get_result_async=lambda: future)
        async def send(goal): return handle
        adapter._clients = {'goto_pose': SimpleNamespace(is_done=lambda: True, send_goal=send)}
        await adapter.dispatch_phase(GoToPhase(Pose(Position(3, 4))), SimpleNamespace())
        future.set_result(SimpleNamespace(status=5))
        await asyncio.sleep(0)
        assert not node._driving and messages
    asyncio.run(run())


def test_late_previous_goal_result_does_not_stop_new_goal():
    node, messages = edge()
    adapter = DrlAdapter.__new__(DrlAdapter)
    adapter._edge_node = node
    old, new = object(), object()
    adapter._phase_goal_handle = new
    adapter._stop_completed_goal(old)
    assert node._driving and not messages
    adapter._stop_completed_goal(new)
    assert not node._driving and messages


def test_cancel_robot_task_stops_edge_and_cancels_child_once():
    node, messages = edge()
    canceled = []
    adapter = DrlAdapter.__new__(DrlAdapter)
    adapter._edge_node = node
    adapter._clients = {'goto_pose': SimpleNamespace(is_done=lambda: False, cancel=lambda: canceled.append(True))}
    robot = RobotManager.__new__(RobotManager)
    robot._adapters = {'mobile': adapter, 'also_mobile': adapter}
    robot._current_request = object()
    robot._phase_index = 0
    robot._publish_goal_task = None
    robot._stop_pub = SimpleNamespace(publish=messages.append)
    robot.cancel_task()
    assert canceled == [True] and not node._driving
    assert robot._current_request is None
    assert messages and all(m.linear.x == 0 for m in messages)


def test_episode_cancel_stops_children_before_publishing_terminal_outcome():
    async def run():
        future = asyncio.get_running_loop().create_future()
        order = []
        def stop():
            assert not future.done()
            order.append('stop')
        stub = SimpleNamespace(
            _episodes=SimpleNamespace(current=SimpleNamespace(episode_id=1), pending_outcomes={1: future}),
            event_loop=SimpleNamespace(call_soon_threadsafe=lambda fn,*args: fn(*args)),
            _robots_manager=SimpleNamespace(managers={'jackal': SimpleNamespace(cancel_task=stop)}))
        stub._cancel_robot_tasks = lambda: TaskGenerator._cancel_robot_tasks(stub)
        TaskGenerator._cancel_callback(stub, SimpleNamespace())
        assert order == ['stop'] and future.done()
    asyncio.run(run())
