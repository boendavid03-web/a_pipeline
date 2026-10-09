"""One bounded, headless Jackal/Nav2 task on the shipped map_empty/quicktest."""

import json
import math
import os
import pathlib
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from arena_runtime_msgs.msg import EnvRegistry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from task_generator_msgs.action import RunEpisode
from tf2_ros import Buffer, TransformListener


rclpy.init()
node = rclpy.create_node(
    "autonomous_nav_readonly_probe",
    parameter_overrides=[Parameter("use_sim_time", value=True)],
)
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
namespace = "/arena/env_0/task_generator_node/jackal"
evidence = {"poses": [], "cmd_count": 0, "moving_cmd_count": 0, "scan_count": 0, "plan_count": 0}
registry = {"reference": None}
best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
latched = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def on_registry(msg):
    for env in msg.envs:
        if env.env_id == 0 and env.ready:
            registry["reference"] = [float(env.reference[0]), float(env.reference[1])]


def on_cmd(msg):
    evidence["cmd_count"] += 1
    if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
        evidence["moving_cmd_count"] += 1


def on_scan(msg):
    if msg.ranges and any(math.isfinite(v) for v in msg.ranges):
        evidence["scan_count"] += 1


def on_plan(msg):
    if msg.poses:
        evidence["plan_count"] += 1


node.create_subscription(EnvRegistry, "/arena/state/envs", on_registry, latched)
node.create_subscription(Twist, namespace + "/cmd_vel", on_cmd, best_effort)
node.create_subscription(LaserScan, namespace + "/lidar", on_scan, best_effort)
node.create_subscription(Path, namespace + "/plan", on_plan, best_effort)

last_pose_wall = 0.0


def step():
    global last_pose_wall
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.monotonic()
    if now - last_pose_wall < 0.15:
        return
    try:
        transform = tf_buffer.lookup_transform("map", "env_0/jackal/base_link", Time())
    except Exception:
        return
    p = transform.transform.translation
    evidence["poses"].append([now, float(p.x), float(p.y)])
    last_pose_wall = now


def wait_for(future, seconds):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        step()
    return future.done()


summary = {"world": "map_empty", "scenario": "quicktest", "robot": "jackal", "human_count": 0}
try:
    client = ActionClient(node, RunEpisode, "/arena/env_0/task_generator_node/lifecycle/run_episode")
    deadline = time.monotonic() + 30
    while not client.wait_for_server(timeout_sec=1) and time.monotonic() < deadline:
        step()
    assert client.server_is_ready(), "RunEpisode action server unavailable"

    sent = client.send_goal_async(RunEpisode.Goal(world="map_empty", seed=42))
    assert wait_for(sent, 45), "RunEpisode goal acceptance timed out"
    handle = sent.result()
    assert handle.accepted, "RunEpisode goal rejected"
    result_future = handle.get_result_async()
    started_wall = time.monotonic()
    started_sim = node.get_clock().now().nanoseconds * 1e-9
    while not result_future.done():
        step()
        if time.monotonic() - started_wall > 210 or node.get_clock().now().nanoseconds * 1e-9 - started_sim > 120:
            cancel = handle.cancel_goal_async()
            wait_for(cancel, 20)
            wait_for(result_future, 40)
            raise TimeoutError("Autonomous navigation did not reach a terminal result within the bounded task window")

    wrapped = result_future.result()
    summary["terminal_status"] = int(wrapped.status)
    summary["episode_state"] = int(wrapped.result.state)
    summary["episode_info"] = wrapped.result.info
    summary["sim_elapsed_s"] = round(node.get_clock().now().nanoseconds * 1e-9 - started_sim, 3)
    summary["wall_elapsed_s"] = round(time.monotonic() - started_wall, 3)

    hold_start = time.monotonic()
    while time.monotonic() - hold_start < 5:
        step()
    poses = [p for p in evidence["poses"] if p[0] >= started_wall]
    leg_lengths = [math.dist(a[1:], b[1:]) for a, b in zip(poses, poses[1:])]
    summary["tf_samples"] = len(poses)
    summary["travel_m_excluding_reset_jumps"] = round(sum(d for d in leg_lengths if d < 0.3), 3)
    summary["start_map_xy"] = [round(v, 3) for v in poses[0][1:]] if poses else None
    summary["end_map_xy"] = [round(v, 3) for v in poses[-1][1:]] if poses else None
    summary["reference_map_xy"] = registry["reference"]
    if poses and registry["reference"] is not None:
        goal = [registry["reference"][0] + 10.0, registry["reference"][1] + 5.0]
        summary["goal_map_xy"] = goal
        summary["final_goal_error_m"] = round(math.dist(poses[-1][1:], goal), 3)
    summary.update({k: evidence[k] for k in ("cmd_count", "moving_cmd_count", "scan_count", "plan_count")})

    assert wrapped.status == 4 and wrapped.result.state == RunEpisode.Result.SUCCESS, "Robot task did not succeed"
    assert summary["travel_m_excluding_reset_jumps"] > 1.0, "No meaningful physical robot motion"
    assert summary["moving_cmd_count"] > 0 and summary["scan_count"] > 0 and summary["plan_count"] > 0, "Navigation control, lidar, or path missing"
    assert summary.get("final_goal_error_m", float("inf")) < 1.0, "Final robot pose is too far from the scenario goal"
    summary["status"] = "AUTONOMOUS_NAV_TASK_PASS"
except Exception as exc:
    summary["status"] = "FAIL"
    summary["error"] = repr(exc)
    raise
finally:
    trace = os.environ.get("A_TRACE_DIR")
    if trace:
        pathlib.Path(trace, "autonomous_nav_summary.json").write_text(json.dumps(summary, indent=2))
    print("AUTONOMOUS_NAV_JSON " + json.dumps(summary), flush=True)
    node.destroy_node()
    rclpy.shutdown()
