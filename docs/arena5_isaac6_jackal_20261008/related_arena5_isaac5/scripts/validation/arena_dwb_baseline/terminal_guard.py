#!/usr/bin/env python3
"""Observe one Arena high-level action and stop the capture at its terminal state."""
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

run = Path(sys.argv[1])
launch_pid = int(sys.argv[2])
startup_wall_limit = 300.0
episode_sim_limit = 600.0
wall_limit = 1200.0
clock_stall_limit = 45.0
post_terminal_wall = 10.0
post_terminal_sim = 5.0
started = time.monotonic()
first_status_wall = None
first_status_sim = None
last_clock_wall = None
current_sim = None
terminal = None
terminal_wall = None
terminal_sim = None
statuses = {}
cancel_requested = False
reason = None
timeout_cleanup_until = None
last_log_check = started
log_offset = 0

rclpy.init()
node = rclpy.create_node('arena_scenario1_terminal_guard')


def on_clock(msg):
    global current_sim, last_clock_wall
    current_sim = msg.clock.sec + msg.clock.nanosec * 1e-9
    last_clock_wall = time.monotonic()


def on_status(msg):
    global first_status_wall, first_status_sim, terminal, terminal_wall, terminal_sim
    now = time.monotonic()
    for item in msg.status_list:
        uid = bytes(item.goal_info.goal_id.uuid).hex()
        events = statuses.setdefault(uid, [])
        if not events or events[-1]['code'] != item.status:
            events.append({'code': int(item.status), 'wall_monotonic': now, 'sim_s': current_sim})
        if first_status_wall is None:
            first_status_wall, first_status_sim = now, current_sim
        if item.status in (4, 5, 6) and terminal is None:
            terminal = {'uuid': uid, 'code': int(item.status)}
            terminal_wall, terminal_sim = now, current_sim


node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
node.create_subscription(GoalStatusArray,
    '/task_generator_node/jackal/navigate_to_pose/_action/status', on_status, 10)
client = node.create_client(CancelGoal,
    '/task_generator_node/jackal/navigate_to_pose/_action/cancel_goal')


def request_cancel():
    global cancel_requested
    if cancel_requested or len(statuses) != 1:
        return
    from unique_identifier_msgs.msg import UUID
    uid = next(iter(statuses))
    request = CancelGoal.Request()
    request.goal_info.goal_id = UUID(uuid=list(bytes.fromhex(uid)))
    if client.service_is_ready():
        client.call_async(request)
        cancel_requested = True


try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if timeout_cleanup_until is not None:
            request_cancel()
            if now >= timeout_cleanup_until:
                break
        elif terminal is not None:
            if (current_sim is not None and terminal_sim is not None
                    and current_sim - terminal_sim >= post_terminal_sim):
                reason = 'TERMINAL_POST_WINDOW_SIM'
                break
            if now - terminal_wall >= post_terminal_wall:
                reason = 'TERMINAL_POST_WINDOW_WALL'
                break
        elif now - started >= wall_limit:
            reason = 'WALL_PROTECTION_TIMEOUT'
            timeout_cleanup_until = now + 5.0
            request_cancel()
        elif first_status_wall is None and now - started >= startup_wall_limit:
            reason = 'STARTUP_READINESS_TIMEOUT'
            break
        elif first_status_sim is not None and current_sim is not None \
                and current_sim - first_status_sim >= episode_sim_limit:
            reason = 'EPISODE_SIM_TIMEOUT'
            timeout_cleanup_until = now + 5.0
            request_cancel()
        elif last_clock_wall is not None and now - last_clock_wall >= clock_stall_limit:
            reason = 'CLOCK_STALL'
            break
        if first_status_wall is None and now - last_log_check >= 1.0:
            last_log_check = now
            launch_log = run / 'launch.log'
            if launch_log.exists():
                with launch_log.open('rb') as stream:
                    stream.seek(log_offset)
                    new_log = stream.read().decode(errors='replace')
                    log_offset = stream.tell()
                if 'Isaac goal rejected generation=' in new_log:
                    reason = 'GOAL_REJECTED'
                    break
                if 'Isaac goal send failed generation=' in new_log:
                    reason = 'GOAL_SEND_FAILED'
                    break
        try:
            os.kill(launch_pid, 0)
        except ProcessLookupError:
            reason = 'LAUNCH_PROCESS_EXITED'
            break
finally:
    result = {
        'reason': reason, 'statuses': statuses, 'terminal': terminal,
        'first_status_sim_s': first_status_sim,
        'terminal_sim_s': terminal_sim,
        'duration_sim_s': (terminal_sim - first_status_sim if terminal_sim is not None
                           and first_status_sim is not None else None),
        'duration_wall_s': (terminal_wall - first_status_wall if terminal_wall is not None
                            and first_status_wall is not None else None),
        'cancel_requested': cancel_requested,
        'wall_elapsed_s': time.monotonic() - started,
        'limits': {'startup_wall_s': startup_wall_limit,
                   'episode_sim_s': episode_sim_limit,
                   'wall_protection_s': wall_limit,
                   'clock_stall_wall_s': clock_stall_limit,
                   'post_terminal_sim_s': post_terminal_sim,
                   'post_terminal_wall_s': post_terminal_wall},
    }
    (run / 'guard_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    node.destroy_node()
    rclpy.shutdown()
