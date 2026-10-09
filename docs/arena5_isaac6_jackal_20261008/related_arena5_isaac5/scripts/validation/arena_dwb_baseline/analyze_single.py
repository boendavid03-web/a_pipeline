#!/usr/bin/env python3
"""Extract transparent metrics for one completed Arena scenario-1 bag."""
import bisect
import collections
import hashlib
import json
import math
import re
import sqlite3
import sys
from pathlib import Path

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from lifecycle_msgs.msg import TransitionEvent
from nav2_msgs.msg import VoxelGrid
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

run = Path(sys.argv[1])
preflight = json.loads((run / 'preflight.json').read_text())
scenario = Path(preflight['scenario_absolute_path'])
scene = json.loads(scenario.read_text())
guard = json.loads((run / 'guard_result.json').read_text())
db = run / 'raw_rosbag/raw_rosbag_0.db3'
conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
integrity = conn.execute('pragma integrity_check').fetchone()[0]
types = {
    '/clock': Clock, '/tf': TFMessage,
    '/task_generator_node/jackal/odom': Odometry,
    '/task_generator_node/jackal/lidar': LaserScan,
    '/task_generator_node/jackal/local_costmap/costmap': OccupancyGrid,
    '/task_generator_node/jackal/global_costmap/costmap': OccupancyGrid,
    '/task_generator_node/jackal/local_costmap/voxel_grid': VoxelGrid,
    '/task_generator_node/jackal/cmd_vel': Twist,
    '/task_generator_node/jackal/cmd_vel_nav': Twist,
    '/task_generator_node/jackal/navigate_to_pose/_action/status': GoalStatusArray,
    '/task_generator_node/human_states': Agents,
}
for node in ('planner_server', 'controller_server', 'bt_navigator'):
    types[f'/task_generator_node/jackal/{node}/transition_event'] = TransitionEvent
topics = {i: (name, types[name]) for i, name in conn.execute('select id,name from topics')
          if name in types}
counts = collections.Counter()
clocks, odoms, transforms, humans, scans = [], [], [], [], []
grids, voxels, commands = [], [], collections.defaultdict(list)
statuses, active_states = collections.defaultdict(list), set()
active_times = {}
global_costmap_times = []
for tid, ts, raw in conn.execute('select topic_id,timestamp,data from messages order by timestamp'):
    if tid not in topics:
        continue
    name, cls = topics[tid]
    msg = deserialize_message(raw, cls)
    wall = ts * 1e-9
    counts[name] += 1
    if name == '/clock':
        clocks.append((wall, msg.clock.sec + msg.clock.nanosec * 1e-9))
    elif name == '/tf':
        for tr in msg.transforms:
            if (tr.header.frame_id, tr.child_frame_id) == ('map', 'jackal/odom'):
                p, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                transforms.append((wall, p.x, p.y, yaw))
    elif name.endswith('/odom'):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        odoms.append((wall, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                      p.x, p.y, yaw, msg.header.frame_id))
    elif name.endswith('/lidar'):
        near = sum(math.isfinite(x) and msg.range_min <= x <= min(2.5, msg.range_max)
                   for x in msg.ranges)
        scans.append((wall, len(msg.ranges), near))
    elif name.endswith('/costmap'):
        grids.append((name, wall, sum(x > 0 for x in msg.data), msg.header.frame_id))
        if name.endswith('/global_costmap/costmap'):
            global_costmap_times.append(wall)
    elif name.endswith('/voxel_grid'):
        mask = (1 << msg.size_z) - 1
        voxels.append((wall, sum(((int(x) >> 16) & mask).bit_count() for x in msg.data)))
    elif name.endswith('/cmd_vel') or name.endswith('/cmd_vel_nav'):
        commands[name].append((wall, msg.linear.x, msg.angular.z))
    elif name.endswith('/status'):
        for item in msg.status_list:
            uid = bytes(item.goal_info.goal_id.uuid).hex()
            if not statuses[uid] or statuses[uid][-1][1] != item.status:
                statuses[uid].append((wall, int(item.status)))
    elif name.endswith('/human_states'):
        humans.append((wall, msg.header.frame_id,
                       [(x.name, x.position.position.x, x.position.position.y, x.radius)
                        for x in msg.agents]))
    elif name.endswith('/transition_event') and msg.goal_state.label == 'active':
        component = name.split('/')[-2]
        active_states.add(component)
        active_times[component] = wall
conn.close()

def latest(arr, wall):
    idx = bisect.bisect_right([x[0] for x in arr], wall) - 1
    return arr[idx] if idx >= 0 else None

def map_pose(odom):
    tf = latest(transforms, odom[0])
    if tf is None:
        return None
    _, tx, ty, angle = tf
    return (tx + math.cos(angle)*odom[2] - math.sin(angle)*odom[3],
            ty + math.sin(angle)*odom[2] + math.cos(angle)*odom[3],
            (odom[4] + angle + math.pi) % (2*math.pi) - math.pi)

events = next(iter(statuses.values()), [])
uuid = next(iter(statuses), None)
start_wall = events[0][0] if events else None
terminal_event = next(((w, c) for w, c in events if c in (4, 5, 6)), None)
end_wall = terminal_event[0] if terminal_event else None
nav_odoms = [o for o in odoms if start_wall is not None and end_wall is not None
             and start_wall <= o[0] <= end_wall and o[5] == 'jackal/odom']
sampled = []
for o in nav_odoms:
    if not sampled or o[1] - sampled[-1][1] >= 0.1:
        sampled.append(o)
if nav_odoms and sampled[-1] != nav_odoms[-1]:
    sampled.append(nav_odoms[-1])
path_length = 0.0
excluded_jumps = 0
for a, b in zip(sampled, sampled[1:]):
    step = math.hypot(b[2]-a[2], b[3]-a[3])
    if step > 0.5 or b[1] <= a[1]:
        excluded_jumps += 1
    else:
        path_length += step
final_odom = min(odoms, key=lambda o: abs(o[0]-end_wall)) if end_wall and odoms else None
final_pose = map_pose(final_odom) if final_odom else None
goal = scene['robots'][0]['goal']
final_xy_error = math.hypot(final_pose[0]-goal[0], final_pose[1]-goal[1]) if final_pose else None
final_yaw_error = abs((final_pose[2]-goal[2]+math.pi) % (2*math.pi)-math.pi) if final_pose else None

human_counts = collections.Counter()
human_trajectory = collections.defaultdict(list)
unmatched_humans = 0
min_center = None
odom_walls = [o[0] for o in odoms]
for wall, frame, people in humans:
    if start_wall is None or end_wall is None or not start_wall <= wall <= end_wall:
        continue
    for name, x, y, radius in people:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        human_counts[name] += 1
        human_trajectory[name].append((x, y))
        idx = bisect.bisect_left(odom_walls, wall)
        choices = [odoms[j] for j in (idx-1, idx) if 0 <= j < len(odoms)]
        if not choices:
            unmatched_humans += 1
            continue
        near = min(choices, key=lambda o: abs(o[0]-wall))
        if abs(near[0]-wall) > 0.2:
            unmatched_humans += 1
            continue
        robot = map_pose(near)
        if robot is None:
            unmatched_humans += 1
            continue
        distance = math.hypot(robot[0]-x, robot[1]-y)
        if min_center is None or distance < min_center['meters']:
            min_center = {'meters': distance, 'human': name,
                          'wall_receive_time_s': wall, 'robot_odom_time_gap_s': abs(near[0]-wall)}

post_commands = [x for x in commands['/task_generator_node/jackal/cmd_vel']
                 if end_wall is not None and x[0] > end_wall]
log = (run / 'launch.log').read_text(errors='replace')
accepted = re.findall(r'ISAAC_GOAL_ACCEPTED generation=\d+ uuid=([0-9a-f]+)', log)
accepted_timestamps = [float(x) for x in re.findall(
    r'\[(\d+\.\d+)\] \[task_generator_node\.RobotManager\]: ISAAC_GOAL_ACCEPTED', log)]
sends = re.findall(r'ISAAC_GOAL_SEND generation=\d+.*', log)
send_timestamps = [float(x) for x in re.findall(
    r'\[(\d+\.\d+)\] \[task_generator_node\.RobotManager\]: ISAAC_GOAL_SEND', log)]
terminals = re.findall(r'ISAAC_GOAL_TERMINAL generation=\d+ uuid=([0-9a-f]+) status=(\d+)', log)
loaded = re.findall(r'ARENA_SCENARIO_PARAMETER_READY=(\S+)', log)
shelves = {name for name in ('shelf1', 'shelf2', 'shelf3')
           if re.search(r'ARENA_OBSTACLE_STAGE name=' + name + r' .*geometry=\d+ colliders=\d+', log)}
people_ready = {name for name in ('1', '2', '3')
                if re.search(r'ARENA_PEDESTRIAN_READY stage_prefix=/World/pedestrians/_' + name, log)}
status_name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
result = status_name.get(terminal_event[1], 'NOT_STARTED') if terminal_event else 'NOT_STARTED'
if guard['reason'] in ('EPISODE_SIM_TIMEOUT', 'WALL_PROTECTION_TIMEOUT'):
    result = 'TIMEOUT'
elif not terminal_event and events:
    result = 'NOT_STARTED'
valid = (integrity == 'ok' and not preflight['drift'] and loaded == [str(scenario)]
         and len(statuses) == 1 and accepted == [uuid] and len(sends) == 1
         and len(terminals) == 1 and terminals[0][0] == uuid
         and shelves == {'shelf1', 'shelf2', 'shelf3'} and people_ready == {'1','2','3'}
         and active_states == {'planner_server','controller_server','bt_navigator'}
         and terminal_event is not None and len(nav_odoms) > 2 and len(scans) > 0
         and len(humans) > 0)
send_time = send_timestamps[0] if len(send_timestamps) == 1 else None
ready_before_send = (send_time is not None
                     and active_states == {'planner_server','controller_server','bt_navigator'}
                     and all(t < send_time for t in active_times.values())
                     and any(t < send_time for t in global_costmap_times)
                     and any(t[0] < send_time for t in transforms))
effective_task = (run / 'effective_task_generator_params.yaml').read_text()
effective_ctrl = (run / 'effective_controller_params.yaml').read_text()
tf_runtime_match = ('ARENA_TF_RUNTIME file=' + str(preflight['paths']['tf_isolated']) in log
                    and 'framePeriod=10 requested=10 created=True' in log)
plugin_match = ('global_planner: navfn' in effective_task
                and 'local_planner: dwb' in effective_task
                and 'plugin: dwb_core::DWBLocalPlanner' in effective_ctrl
                and '/opt/ros/humble/lib/libdwb_core.so' in (run / 'controller_libraries.txt').read_text())
run_validity = 'VALID' if valid and ready_before_send and tf_runtime_match and plugin_match else 'INVALID'
summary = {
    'RUN_ID': run.name, 'ROS_DOMAIN_ID': int(preflight['domain']),
    'ROOT_HEAD': preflight['root_head'],
    'RUN_VALIDITY': run_validity, 'NAVIGATION_RESULT': result,
    'FAILURE_OR_TIMEOUT_REASON': 'controller_server: Failed to make progress' if result == 'ABORTED' and 'Failed to make progress' in log else guard['reason'],
    'SCENARIO_ACTUALLY_LOADED': loaded, 'SCENARIO_SOURCE_LIVE_HASH_MATCH': preflight['sha256']['scenario_source'] == preflight['sha256']['scenario_live'],
    'ROBOT_AND_HUMAN_COUNTS': {'robots_expected': 1, 'humans_expected': 3, 'human_names_observed': sorted(human_counts),
                               'shelves_initialized_with_collision': sorted(shelves), 'pedestrians_initialized': sorted(people_ready)},
    'RUNTIME_AND_CONFIG_MATCH_FROZEN': not bool(preflight['drift']) and tf_runtime_match and plugin_match,
    'TF_RUNTIME_MATCH': tf_runtime_match, 'EFFECTIVE_PLUGIN_MATCH': plugin_match,
    'HIGH_LEVEL_GOAL_COUNT': len(statuses), 'HIGH_LEVEL_UUID': uuid,
    'HIGH_LEVEL_STATUS_EVENTS': dict(statuses), 'STARTUP_READY_BEFORE_SEND': ready_before_send,
    'GOAL_SEND_WALL_LOG_S': send_time, 'NAV2_ACTIVE_BAG_WALL_S': active_times,
    'GOAL_ACCEPTED_WALL_LOG_S': accepted_timestamps[0] if len(accepted_timestamps) == 1 else None,
    'FIRST_STATUS_WALL_BAG_S': start_wall, 'TERMINAL_STATUS_WALL_BAG_S': end_wall,
    'GLOBAL_COSTMAP_BEFORE_SEND': any(t < send_time for t in global_costmap_times) if send_time else False,
    'REAL_MAP_TF_BEFORE_SEND': any(t[0] < send_time for t in transforms) if send_time else False,
    'DURATION_SIM_S': guard['duration_sim_s'], 'DURATION_WALL_S': guard['duration_wall_s'],
    'PATH_LENGTH_M': path_length if sampled else None, 'PATH_ODOM_SAMPLE_PERIOD_S': 0.1,
    'PATH_EXCLUDED_JUMPS': excluded_jumps,
    'NAV_ODOM_START_MAP_POSE': map_pose(nav_odoms[0]) if nav_odoms else None,
    'NAV_ODOM_NET_DISPLACEMENT_M': math.hypot(nav_odoms[-1][2]-nav_odoms[0][2], nav_odoms[-1][3]-nav_odoms[0][3]) if nav_odoms else None,
    'FINAL_XY_ERROR_M': final_xy_error, 'FINAL_YAW_ERROR_RAD': final_yaw_error,
    'FINAL_MAP_POSE': final_pose, 'FINAL_ODOM_FRAME': final_odom[5] if final_odom else None,
    'POST_TERMINAL_STOP': bool(post_commands) and all(abs(x[1])+abs(x[2]) < 1e-5 for x in post_commands),
    'POST_TERMINAL_CMD_COUNT': len(post_commands),
    'TF_SCAN_COSTMAP_VALIDITY': {'map_to_odom_tf_samples': len(transforms), 'scan_count': len(scans),
                                 'scan_beam_counts': sorted(set(x[1] for x in scans)),
                                 'scans_with_near_returns': sum(x[2] > 0 for x in scans),
                                 'local_grids': sum(x[0].endswith('/local_costmap/costmap') for x in grids),
                                 'local_nonzero_grids': sum(x[0].endswith('/local_costmap/costmap') and x[2] > 0 for x in grids),
                                 'voxel_grids': len(voxels), 'nonzero_voxel_grids': sum(x[1] > 0 for x in voxels),
                                 'tf_accept_count': None},
    'HUNAV_TRAJECTORIES': {name: {'samples': len(track),
                                  'start_xy': track[0], 'end_xy': track[-1],
                                  'displacement_m': math.dist(track[0], track[-1])}
                           for name, track in human_trajectory.items() if track},
    'MIN_HUMAN_DISTANCE_AND_DEFINITION': {'minimum_center_distance': min_center,
        'definition': 'HuNav positions treated as map coordinates because their initial positions match scenario and HuNav robot pose; human header.frame_id is empty. Robot odom transformed by recorded map->jackal/odom TF. Match bag receive wall time within 0.2 s. Proxy center distance, not body clearance or collision.',
        'human_header_frames': sorted(set(x[1] for x in humans)), 'unmatched_samples': unmatched_humans},
    'COLLISION_EVIDENCE': 'NOT_MEASURED: no contact topic or complete human geometry in bag',
    'MISSING_METRICS': ['contact collision ground truth', 'exact TF_ACCEPT count on uninstrumented Nav2', 'official Arena Evaluation output'],
    'RAW_EVIDENCE_PATH': str(run), 'bag_integrity_check': integrity,
    'REPRODUCTION_COMMAND': 'scripts/validation/arena_dwb_baseline/run_single.bash 221 <new_unique_run_id>',
    'FILES_CREATED_OR_MODIFIED': ['scripts/validation/arena_dwb_baseline/run_single.bash',
                                   'scripts/validation/arena_dwb_baseline/preflight.py',
                                   'scripts/validation/arena_dwb_baseline/terminal_guard.py',
                                   'scripts/validation/arena_dwb_baseline/analyze_single.py',
                                   str(run)],
    'NEXT_SINGLE_ACTION': 'after this record is accepted, independent launch of original 2.json under the same rules',
    'bag_size_bytes': db.stat().st_size, 'bag_sha256': hashlib.sha256(db.read_bytes()).hexdigest(),
    'topic_counts': dict(counts), 'guard': guard,
    'nav2_cmd_nonzero': sum(abs(x[1])+abs(x[2]) > 1e-5 for x in commands['/task_generator_node/jackal/cmd_vel_nav']),
    'final_cmd_nonzero': sum(abs(x[1])+abs(x[2]) > 1e-5 for x in commands['/task_generator_node/jackal/cmd_vel']),
}
(run / 'run_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({key: value for key, value in summary.items() if key not in ('topic_counts','guard')}, indent=2))
