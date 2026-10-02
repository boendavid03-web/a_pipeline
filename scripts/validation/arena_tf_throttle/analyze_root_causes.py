#!/usr/bin/env python3
"""Read-only wall/sim timeline extraction from a formal Arena rosbag."""
import bisect
import collections
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run = Path(sys.argv[1])
db = run / 'raw_rosbag/raw_rosbag_0.db3'
conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
topics = {i: (n, t) for i, n, t in conn.execute('select id,name,type from topics')}
relevant = {i: (n, get_message(t)) for i, (n, t) in topics.items() if n in {
    '/clock', '/task_generator_node/jackal/odom', '/task_generator_node/jackal/goal_pose',
    '/task_generator_node/jackal/navigate_to_pose/_action/status',
    '/task_generator_node/jackal/compute_path_to_pose/_action/status',
    '/task_generator_node/jackal/follow_path/_action/status',
    '/task_generator_node/jackal/cmd_vel_nav', '/task_generator_node/jackal/cmd_vel',
    '/task_generator_node/jackal/global_costmap/costmap',
    '/task_generator_node/jackal/local_costmap/costmap',
    '/task_generator_node/jackal/planner_server/transition_event',
    '/task_generator_node/jackal/controller_server/transition_event',
    '/task_generator_node/jackal/bt_navigator/transition_event',
    '/task_generator_node/jackal/global_costmap/global_costmap/transition_event',
    '/task_generator_node/jackal/local_costmap/local_costmap/transition_event',
}}

def stamp(t):
    return t.sec + t.nanosec * 1e-9

def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

events = collections.defaultdict(list)
last_status = {}
for tid, ns, raw in conn.execute('select topic_id,timestamp,data from messages order by timestamp'):
    if tid not in relevant:
        continue
    name, cls = relevant[tid]
    m = deserialize_message(raw, cls)
    w = ns * 1e-9
    if name == '/clock':
        events['clock'].append([w, stamp(m.clock)])
    elif name.endswith('/odom'):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        events['odom'].append([w, stamp(m.header.stamp), p.x, p.y, yaw(q), v.linear.x, v.angular.z])
    elif name.endswith('/goal_pose'):
        p, q = m.pose.position, m.pose.orientation
        events['goal_pose'].append([w, stamp(m.header.stamp), p.x, p.y, yaw(q)])
    elif name.endswith('/status'):
        key = name.split('/')[-3]
        for item in m.status_list:
            u = bytes(item.goal_info.goal_id.uuid).hex()
            if last_status.get((key, u)) != item.status:
                events[key].append([w, u, item.status])
                last_status[key, u] = item.status
    elif name.endswith('/cmd_vel_nav') or name.endswith('/cmd_vel'):
        events[name.rsplit('/', 1)[-1]].append([w, m.linear.x, m.angular.z])
    elif name.endswith('/costmap'):
        g = m.info
        key = 'global_costmap' if '/global_' in name else 'local_costmap'
        events[key].append([w, stamp(m.header.stamp), g.origin.position.x,
                            g.origin.position.y, g.width, g.height, g.resolution,
                            sum(v == 100 for v in m.data)])
    elif name.endswith('/transition_event'):
        events['lifecycle'].append([w, name.split('/')[-2], m.start_state.label, m.goal_state.label])
conn.close()

clock = events['clock']
cw = [x[0] for x in clock]
odom = events['odom']
ow = [x[0] for x in odom]

def sim(w):
    i = bisect.bisect_right(cw, w) - 1
    return clock[i][1] if i >= 0 else None

def nearest_odom(w):
    i = bisect.bisect_left(ow, w)
    return min(odom[max(i-1, 0):min(i+1, len(odom))], key=lambda x: abs(x[0]-w))

def near_cmd(key, w):
    a = events[key]
    ws = [x[0] for x in a]
    i = bisect.bisect_right(ws, w)-1
    return a[i][1:] if i >= 0 else None

def present(w):
    o = nearest_odom(w)
    return {'wall': round(w, 6), 'sim': round(sim(w), 6) if sim(w) is not None else None,
            'odom_stamp': round(o[1], 6), 'x': round(o[2], 5), 'y': round(o[3], 5),
            'yaw': round(o[4], 5), 'odom_vx': round(o[5], 5), 'odom_wz': round(o[6], 5),
            'cmd_nav': near_cmd('cmd_vel_nav', w), 'cmd_final': near_cmd('cmd_vel', w)}

start = {}
for key in ('goal_pose', 'navigate_to_pose', 'compute_path_to_pose', 'follow_path', 'lifecycle'):
    start[key] = [{'wall': round(x[0], 6), 'sim': sim(x[0]), 'value': x[1:]} for x in events[key]
                  if not events['goal_pose'] or x[0] <= events['goal_pose'][-1][0] + 5]
for key in ('global_costmap', 'local_costmap'):
    start[key] = [{'wall': round(x[0], 6), 'sim': sim(x[0]), 'origin': x[2:4],
                   'size': x[4:6], 'resolution': x[6], 'occupied': x[7]} for x in events[key][:4]]
start['odom_near_goals'] = [present(x[0]) for x in events['goal_pose']]

main = events['navigate_to_pose']
terminal = next((x for x in reversed(main) if x[2] in (4,5,6)), None)
near = None
if terminal:
    end_w = terminal[0]
    end_sim = sim(end_w)
    target_sim = end_sim - 20
    i = bisect.bisect_left([x[1] for x in clock], target_sim)
    begin_w = clock[i][0]
    goal = events['goal_pose'][-1][2:]
    rows = []
    for target in range(math.ceil(target_sim), math.floor(end_sim)+1):
        j = bisect.bisect_left([x[1] for x in clock], target)
        if j >= len(clock): break
        row = present(clock[j][0])
        row['xy_error'] = round(math.hypot(row['x']-goal[0], row['y']-goal[1]), 5)
        row['yaw_error'] = round((row['yaw']-goal[2]+math.pi)%(2*math.pi)-math.pi, 5)
        rows.append(row)
    window = [x for x in odom if begin_w <= x[0] <= end_w]
    nav = [x for x in events['cmd_vel_nav'] if begin_w <= x[0] <= end_w]
    final = [x for x in events['cmd_vel'] if begin_w <= x[0] <= end_w]
    def stats(vals):
        return {'count': len(vals), 'nonzero': sum(abs(v)>1e-4 for v in vals),
                'median_abs': statistics.median(map(abs, vals)) if vals else None,
                'max_abs': max(map(abs, vals)) if vals else None}
    near = {'goal': goal, 'begin_sim': target_sim, 'end_sim': end_sim,
            'begin_wall': begin_w, 'end_wall': end_w, 'rows_1s': rows,
            'cmd_nav_vx': stats([x[1] for x in nav]),
            'cmd_nav_wz': stats([x[2] for x in nav]),
            'cmd_final_vx': stats([x[1] for x in final]),
            'cmd_final_wz': stats([x[2] for x in final]),
            'odom_vx': stats([x[5] for x in window]),
            'odom_wz': stats([x[6] for x in window]),
            'xy_displacement': math.hypot(window[-1][2]-window[0][2], window[-1][3]-window[0][3]),
            'yaw_change': (window[-1][4]-window[0][4]+math.pi)%(2*math.pi)-math.pi,
            'local_costmap_nonzero': sum(x[7]>0 for x in events['local_costmap'] if begin_w<=x[0]<=end_w),
            'local_costmap_count': sum(begin_w<=x[0]<=end_w for x in events['local_costmap'])}
    first_xy_inside = next((x for x in window if math.hypot(x[2]-goal[0], x[3]-goal[1]) <= 0.25), None)
    near['first_xy_within_0_25'] = {'wall': first_xy_inside[0], 'sim': first_xy_inside[1],
                                   'xy_yaw': first_xy_inside[2:5]} if first_xy_inside else None
    near['last_12sim_nav_commands'] = [
        {'wall': x[0], 'sim': sim(x[0]), 'vx': x[1], 'wz': x[2],
         'odom_xy_yaw': nearest_odom(x[0])[2:5]}
        for x in nav if sim(x[0]) >= end_sim - 12]
    for span in (10, 8):
        w0 = clock[bisect.bisect_left([x[1] for x in clock], end_sim-span)][0]
        os = [x for x in window if w0 <= x[0] <= end_w]
        cs = [x for x in nav if w0 <= x[0] <= end_w]
        fs = [x for x in final if w0 <= x[0] <= end_w]
        near[f'last_{span}sim'] = {
            'start_xy_yaw': os[0][2:5], 'end_xy_yaw': os[-1][2:5],
            'xy_displacement': math.hypot(os[-1][2]-os[0][2], os[-1][3]-os[0][3]),
            'yaw_change': (os[-1][4]-os[0][4]+math.pi)%(2*math.pi)-math.pi,
            'nav_vx': stats([x[1] for x in cs]), 'nav_wz': stats([x[2] for x in cs]),
            'final_vx': stats([x[1] for x in fs]), 'final_wz': stats([x[2] for x in fs]),
            'odom_vx': stats([x[5] for x in os]), 'odom_wz': stats([x[6] for x in os])}
    baseline = None
    resets = []
    for x in events['cmd_vel_nav']:
        if x[0] > end_w:
            break
        o = nearest_odom(x[0])
        if baseline is None or math.hypot(o[2]-baseline[2], o[3]-baseline[3]) > 0.5:
            baseline = o
            resets.append({'wall': x[0], 'sim': sim(x[0]), 'odom_xy_yaw': o[2:5]})
    near['proxy_progress_resets_last_5'] = resets[-5:]
    if baseline:
        o = nearest_odom(end_w)
        near['proxy_last_baseline_to_terminal'] = {
            'sim_elapsed': end_sim-sim(resets[-1]['wall']),
            'xy_displacement': math.hypot(o[2]-baseline[2], o[3]-baseline[3]),
            'yaw_change': (o[4]-baseline[4]+math.pi)%(2*math.pi)-math.pi}
print(json.dumps({'run': str(run), 'startup': start, 'near_goal': near}, indent=2))
