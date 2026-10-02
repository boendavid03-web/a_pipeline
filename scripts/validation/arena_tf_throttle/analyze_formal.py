#!/usr/bin/env python3
"""Summarize direct bag evidence from an uninstrumented Arena Nav2 gate."""
import collections
import hashlib
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav2_msgs.msg import VoxelGrid

run = Path(sys.argv[1])
scenario = Path(sys.argv[2])
db = run / 'raw_rosbag/raw_rosbag_0.db3'
topic_types = {
    '/clock': Clock,
    '/tf': TFMessage,
    '/task_generator_node/jackal/lidar': LaserScan,
    '/task_generator_node/jackal/odom': Odometry,
    '/task_generator_node/jackal/local_costmap/costmap': OccupancyGrid,
    '/task_generator_node/jackal/local_costmap/voxel_grid': VoxelGrid,
    '/task_generator_node/jackal/cmd_vel_nav': Twist,
    '/task_generator_node/jackal/cmd_vel': Twist,
    '/task_generator_node/jackal/navigate_to_pose/_action/status': GoalStatusArray,
}

def stamp(x):
    return x.sec + x.nanosec * 1e-9

conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
topics = {i: n for i, n in conn.execute('select id,name from topics')}
clock = []
tf_lidar = []
scans = []
odoms = []
grids = []
voxels = []
twists = collections.defaultdict(list)
statuses = collections.defaultdict(list)
for tid, ts, raw in conn.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name = topics[tid]
    if name not in topic_types:
        continue
    msg = deserialize_message(raw, topic_types[name])
    wall = ts * 1e-9
    if name == '/clock':
        clock.append((wall, stamp(msg.clock)))
    elif name == '/tf':
        for tr in msg.transforms:
            if tr.header.frame_id == 'jackal/chassis_link' and tr.child_frame_id == 'jackal/lidar_link':
                tf_lidar.append((wall, stamp(tr.header.stamp)))
    elif name.endswith('/lidar'):
        near = sum(msg.range_min <= r <= min(2.5, msg.range_max) for r in msg.ranges)
        scans.append((wall, stamp(msg.header.stamp), near, len(msg.ranges)))
    elif name.endswith('/odom'):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        p = msg.pose.pose.position
        odoms.append((wall, stamp(msg.header.stamp), p.x, p.y, yaw,
                      msg.twist.twist.linear.x, msg.twist.twist.angular.z))
    elif name.endswith('/costmap'):
        grids.append((wall, stamp(msg.header.stamp), sum(x == 100 for x in msg.data)))
    elif name.endswith('/voxel_grid'):
        mask = (1 << msg.size_z) - 1
        marked = sum(((int(x) >> 16) & mask).bit_count() for x in msg.data)
        voxels.append((wall, stamp(msg.header.stamp), marked))
    elif name.endswith('/cmd_vel_nav') or name.endswith('/cmd_vel'):
        twists[name].append((wall, msg.linear.x, msg.angular.z))
    elif name.endswith('/status'):
        for item in msg.status_list:
            uid = bytes(item.goal_info.goal_id.uuid).hex()
            if not statuses[uid] or statuses[uid][-1][1] != item.status:
                statuses[uid].append((wall, item.status))
conn.close()

goal = json.loads(scenario.read_text())['robots'][0]['goal']
terminal = [(uid, wall, code) for uid, events in statuses.items() for wall, code in events if code in (4, 5, 6)]
terminal.sort(key=lambda x: x[1])
result = None
if terminal and odoms:
    uid, wall, code = terminal[-1]
    odom = min(odoms, key=lambda x: abs(x[0]-wall))
    active_wall = next((event_wall for event_wall, status in statuses[uid] if status == 2), None)
    start_odom = min(odoms, key=lambda x: abs(x[0]-active_wall)) if active_wall is not None else None
    yaw_error = (odom[4] - goal[2] + math.pi) % (2*math.pi) - math.pi
    result = {'uuid': uid, 'status': code, 'wall': wall, 'odom_sim_s': odom[1],
              'xy_error_m': math.hypot(odom[2]-goal[0], odom[3]-goal[1]),
              'yaw_error_rad': abs(yaw_error), 'robot_xy_yaw': odom[2:5],
              'goal_start_robot_xy_yaw': start_odom[2:5] if start_odom else None,
              'goal_to_result_displacement_m': math.hypot(odom[2]-start_odom[2], odom[3]-start_odom[3]) if start_odom else None,
              'odom_twist_linear_angular': odom[5:7]}
    post_commands = [x for x in twists['/task_generator_node/jackal/cmd_vel'] if x[0] > wall]
    result['postterminal_cmd_vel_count'] = len(post_commands)
    result['postterminal_cmd_vel_nonzero'] = sum(abs(x[1])+abs(x[2]) > 1e-5 for x in post_commands)
    result['last_postterminal_cmd_vel'] = post_commands[-1][1:3] if post_commands else None
intervals = [b[1]-a[1] for a,b in zip(tf_lidar,tf_lidar[1:])]
clock_w = [x[0] for x in clock]
import bisect
ages = []
for wall, tf_stamp in tf_lidar:
    i = bisect.bisect_right(clock_w, wall)-1
    if i >= 0:
        ages.append(clock[i][1]-tf_stamp)
summary = {
    'bag': str(db), 'scenario_sha256': hashlib.sha256(scenario.read_bytes()).hexdigest(),
    'sim_max_s': max((x[1] for x in clock), default=None),
    'scan_count': len(scans), 'scan_near_frames': sum(x[2]>0 for x in scans),
    'scan_near_returns': sum(x[2] for x in scans),
    'scan_beam_counts': sorted(set(x[3] for x in scans)),
    'tf_lidar_count': len(tf_lidar),
    'tf_lidar_interval_median_s': statistics.median(intervals) if intervals else None,
    'tf_lidar_age_median_s': statistics.median(ages) if ages else None,
    'tf_lidar_age_max_s': max(ages) if ages else None,
    'local_grid_count': len(grids), 'local_grid_nonzero_count': sum(x[2]>0 for x in grids),
    'local_grid_max_lethal_cells': max((x[2] for x in grids), default=None),
    'voxel_grid_count': len(voxels), 'voxel_grid_nonzero_count': sum(x[2]>0 for x in voxels),
    'voxel_grid_max_marked': max((x[2] for x in voxels), default=None),
    'cmd_vel_nav_nonzero': sum(abs(x[1])+abs(x[2])>1e-5 for x in twists['/task_generator_node/jackal/cmd_vel_nav']),
    'cmd_vel_nonzero': sum(abs(x[1])+abs(x[2])>1e-5 for x in twists['/task_generator_node/jackal/cmd_vel']),
    'odom_count': len(odoms),
    'final_odom_twist_linear_angular': odoms[-1][5:7] if odoms else None,
    'goal': goal, 'goal_status_events': dict(statuses), 'last_terminal': result,
    'final_odom': odoms[-1] if odoms else None,
}
out = run / 'formal_analysis.json'
out.write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k != 'goal_status_events'}, indent=2))
