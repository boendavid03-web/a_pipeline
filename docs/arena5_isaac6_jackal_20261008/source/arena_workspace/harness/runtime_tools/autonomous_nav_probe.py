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
from arena_robots_msgs.msg import CollisionEvents
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
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
evidence = {"poses": [], "commands": [], "wheel_commands": [], "odometry": [], "collisions": [], "cmd_count": 0, "moving_cmd_count": 0, "scan_count": 0, "plan_count": 0}
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
    evidence["commands"].append([time.monotonic(), node.get_clock().now().nanoseconds * 1e-9,
                                  float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)])
    evidence["cmd_count"] += 1
    if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
        evidence["moving_cmd_count"] += 1


def on_scan(msg):
    if msg.ranges and any(math.isfinite(v) for v in msg.ranges):
        evidence["scan_count"] += 1


def on_plan(msg):
    if msg.poses:
        evidence["plan_count"] += 1


def on_wheels(msg):
    evidence["wheel_commands"].append([time.monotonic(), node.get_clock().now().nanoseconds * 1e-9,
                                       list(msg.name), list(msg.velocity)])


def on_odom(msg):
    p = msg.pose.pose.position
    t = msg.twist.twist
    evidence["odometry"].append([time.monotonic(), node.get_clock().now().nanoseconds * 1e-9,
                                 float(p.x), float(p.y), float(t.linear.x), float(t.angular.z)])


def on_collision(msg):
    for event in msg.events:
        evidence["collisions"].append([time.monotonic(), str(event.kind), str(event.obstacle_id)])


node.create_subscription(EnvRegistry, "/arena/state/envs", on_registry, latched)
node.create_subscription(Twist, namespace + "/cmd_vel", on_cmd, best_effort)
node.create_subscription(LaserScan, namespace + "/lidar", on_scan, best_effort)
node.create_subscription(Path, namespace + "/plan", on_plan, best_effort)
node.create_subscription(JointState, namespace + "/isaac/joint_commands_velocity", on_wheels, best_effort)
node.create_subscription(Odometry, namespace + "/odom", on_odom, best_effort)
node.create_subscription(CollisionEvents, namespace + "/collision_events", on_collision, best_effort)

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
    evidence["poses"].append([now, node.get_clock().now().nanoseconds * 1e-9, float(p.x), float(p.y)])
    last_pose_wall = now


def wait_for(future, seconds):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        step()
    return future.done()


world = os.environ.get("A_WORLD", "map_empty")
scenario = os.environ.get("A_SCENARIO_NAME", "quicktest")
goal_local = [float(os.environ.get("A_GOAL_X", "10.0")), float(os.environ.get("A_GOAL_Y", "5.0"))]
sim_limit_s = float(os.environ.get("A_PROBE_SIM_LIMIT_S", "120"))
wall_limit_s = float(os.environ.get("A_PROBE_WALL_LIMIT_S", "210"))
summary = {"world": world, "scenario": scenario, "robot": "jackal", "human_count": 0,
           "goal_local_xy": goal_local, "seed": 42, "sim_limit_s": sim_limit_s, "wall_limit_s": wall_limit_s}
try:
    client = ActionClient(node, RunEpisode, "/arena/env_0/task_generator_node/lifecycle/run_episode")
    deadline = time.monotonic() + 30
    while not client.wait_for_server(timeout_sec=1) and time.monotonic() < deadline:
        step()
    assert client.server_is_ready(), "RunEpisode action server unavailable"

    sent = client.send_goal_async(RunEpisode.Goal(world=world, seed=42))
    assert wait_for(sent, 45), "RunEpisode goal acceptance timed out"
    handle = sent.result()
    assert handle.accepted, "RunEpisode goal rejected"
    result_future = handle.get_result_async()
    started_wall = time.monotonic()
    started_sim = node.get_clock().now().nanoseconds * 1e-9
    while not result_future.done():
        step()
        if time.monotonic() - started_wall > wall_limit_s or node.get_clock().now().nanoseconds * 1e-9 - started_sim > sim_limit_s:
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
    hold_sim = node.get_clock().now().nanoseconds * 1e-9
    while time.monotonic() - hold_start < 8 or node.get_clock().now().nanoseconds * 1e-9 - hold_sim < 5:
        assert time.monotonic() - hold_start < 25, "Simulation clock did not advance through stop hold"
        step()
    poses = [p for p in evidence["poses"] if p[0] >= started_wall]
    leg_lengths = [math.dist(a[2:], b[2:]) for a, b in zip(poses, poses[1:])]
    summary["tf_samples"] = len(poses)
    summary["travel_m_excluding_reset_jumps"] = round(sum(d for d in leg_lengths if d < 0.3), 3)
    summary["start_map_xy"] = [round(v, 3) for v in poses[0][2:]] if poses else None
    summary["end_map_xy"] = [round(v, 3) for v in poses[-1][2:]] if poses else None
    summary["reference_map_xy"] = registry["reference"]
    if poses and registry["reference"] is not None:
        goal = [registry["reference"][0] + goal_local[0], registry["reference"][1] + goal_local[1]]
        summary["goal_map_xy"] = goal
        summary["final_goal_error_m"] = round(math.dist(poses[-1][2:], goal), 3)
        if world == "map_empty":
            table_center = [registry["reference"][0] + 7.0, registry["reference"][1] + 5.0]
            summary["table_center_map_xy"] = table_center
            summary["min_tf_distance_to_table_center_m"] = round(min(math.dist(p[2:], table_center) for p in poses), 3)
    summary["trajectory_y_range_m"] = [round(min(p[3] for p in poses), 3), round(max(p[3] for p in poses), 3)] if poses else None
    summary["collision_event_count_2d"] = len(evidence["collisions"])
    summary["collision_event_ids_2d"] = sorted({row[2] for row in evidence["collisions"]})
    post_cmds = [row for row in evidence["commands"] if row[0] >= hold_start + 0.5]
    post_poses = [row for row in evidence["poses"] if row[0] >= hold_start + 0.5]
    post_wheels = [row for row in evidence["wheel_commands"] if row[0] >= hold_start + 0.5]
    post_odom = [row for row in evidence["odometry"] if row[0] >= hold_start + 0.5]
    summary["post_goal_sim_s"] = round(node.get_clock().now().nanoseconds * 1e-9 - hold_sim, 3)
    summary["post_goal_cmd_samples"] = len(post_cmds)
    summary["post_goal_tf_samples"] = len(post_poses)
    summary["post_goal_wheel_target_samples"] = len(post_wheels)
    summary["post_goal_odom_samples"] = len(post_odom)
    summary["post_goal_max_cmd_component"] = max((abs(v) for row in post_cmds for v in row[2:]), default=None)
    summary["post_goal_max_wheel_target"] = max((abs(v) for row in post_wheels for v in row[3]), default=None)
    summary["post_goal_max_odom_twist"] = max((abs(v) for row in post_odom for v in row[4:]), default=None)
    summary["post_goal_tf_drift_m"] = round(math.dist(post_poses[0][2:], post_poses[-1][2:]), 4) if len(post_poses) >= 2 else None
    summary["last_cmd_before_terminal"] = evidence["commands"][-1][2:] if evidence["commands"] else None
    summary.update({k: evidence[k] for k in ("cmd_count", "moving_cmd_count", "scan_count", "plan_count")})

    assert wrapped.status == 4 and wrapped.result.state == RunEpisode.Result.SUCCESS, "Robot task did not succeed"
    assert summary["travel_m_excluding_reset_jumps"] > 1.0, "No meaningful physical robot motion"
    assert summary["moving_cmd_count"] > 0 and summary["scan_count"] > 0 and summary["plan_count"] > 0, "Navigation control, lidar, or path missing"
    assert summary.get("final_goal_error_m", float("inf")) <= 0.25, "Final robot pose exceeds the configured 0.25 m goal tolerance"
    assert len(post_poses) >= 10 and len(post_wheels) >= 10, "Insufficient post-goal TF or wheel target samples"
    assert not post_cmds or summary["post_goal_max_cmd_component"] <= 1e-6, "Nonzero post-goal command"
    assert summary["post_goal_max_wheel_target"] <= 1e-6, "Nonzero post-goal wheel target"
    assert summary["post_goal_tf_drift_m"] <= 0.02, "Robot drifted more than 0.02 m after goal"
    assert summary["collision_event_count_2d"] == 0, "Arena 2D footprint collision events occurred"
    summary["status"] = "AUTONOMOUS_NAV_TASK_PASS" if post_cmds else "TASK_PASS_STOP_CMD_INCONCLUSIVE"
except Exception as exc:
    summary["status"] = "FAIL"
    summary["error"] = repr(exc)
    raise
finally:
    if "collision_event_count_2d" not in summary:
        summary["collision_event_count_2d"] = len(evidence["collisions"])
        summary["collision_event_ids_2d"] = sorted({row[2] for row in evidence["collisions"]})
    if "final_goal_error_m" not in summary and registry["reference"] is not None and evidence["poses"]:
        goal = [registry["reference"][0] + goal_local[0], registry["reference"][1] + goal_local[1]]
        summary["goal_map_xy"] = goal
        summary["final_goal_error_m"] = round(math.dist(evidence["poses"][-1][2:], goal), 3)
        summary["closest_goal_error_m"] = round(min(math.dist(row[2:], goal) for row in evidence["poses"]), 3)
        summary["end_map_xy"] = [round(v, 3) for v in evidence["poses"][-1][2:]]
    trace = os.environ.get("A_TRACE_DIR")
    if trace:
        pathlib.Path(trace, "autonomous_nav_summary.json").write_text(json.dumps(summary, indent=2))
        pathlib.Path(trace, "autonomous_nav_stop_trace.json").write_text(json.dumps({
            "commands": evidence["commands"], "wheel_commands": evidence["wheel_commands"],
            "odometry": evidence["odometry"], "poses": evidence["poses"], "collisions": evidence["collisions"]}))
    print("AUTONOMOUS_NAV_JSON " + json.dumps(summary), flush=True)
    node.destroy_node()
    rclpy.shutdown()
