#!/usr/bin/env python3
"""Read one completed startup gate bag without changing its contents."""

import argparse
import bisect
import json
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('scenario', type=Path)
    args = parser.parse_args()
    db = args.run_dir / 'raw_rosbag/raw_rosbag_0.db3'
    start = json.loads(args.scenario.read_text())['robots'][0]['start']
    conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = dict((i, (n, get_message(t))) for i, n, t in conn.execute(
        'select id,name,type from topics') if n in {
            '/clock', '/tf', '/task_generator_node/jackal/odom',
            '/task_generator_node/jackal/goal_pose',
            '/task_generator_node/jackal/navigate_to_pose/_action/status',
            '/task_generator_node/jackal/global_costmap/costmap',
            '/task_generator_node/jackal/planner_server/transition_event',
            '/task_generator_node/jackal/controller_server/transition_event',
            '/task_generator_node/jackal/bt_navigator/transition_event',
        })
    clocks, odoms, tfs, maps, lifecycle, goals = [], [], [], [], [], []
    statuses = {}
    for topic_id, timestamp, raw in conn.execute(
            'select topic_id,timestamp,data from messages order by timestamp'):
        if topic_id not in topics:
            continue
        topic, msg_type = topics[topic_id]
        msg = deserialize_message(raw, msg_type)
        wall = timestamp * 1e-9
        if topic == '/clock':
            clocks.append((wall, stamp(msg.clock)))
        elif topic.endswith('/odom'):
            pos = msg.pose.pose.position
            odoms.append((wall, stamp(msg.header.stamp), pos.x, pos.y))
        elif topic == '/tf':
            for transform in msg.transforms:
                if (transform.header.frame_id == 'jackal/odom'
                        and transform.child_frame_id == 'jackal/base_link'):
                    pos = transform.transform.translation
                    tfs.append((wall, stamp(transform.header.stamp), pos.x, pos.y))
        elif topic.endswith('/global_costmap/costmap'):
            info = msg.info
            maps.append((wall, info.origin.position.x, info.origin.position.y,
                         info.width * info.resolution, info.height * info.resolution))
        elif topic.endswith('/goal_pose'):
            goals.append(wall)
        elif topic.endswith('/status'):
            for item in msg.status_list:
                uuid = bytes(item.goal_info.goal_id.uuid).hex()
                events = statuses.setdefault(uuid, [])
                if not events or events[-1][1] != item.status:
                    events.append((wall, item.status))
        elif topic.endswith('/transition_event'):
            lifecycle.append((wall, topic.split('/')[-2],
                              msg.start_state.label, msg.goal_state.label))
    conn.close()
    all_events = sorted((event[0], uuid, event[1]) for uuid, events in statuses.items()
                        for event in events)
    first = all_events[0][0] if all_events else None
    clock_walls = [item[0] for item in clocks]

    def sim_at(wall):
        index = bisect.bisect_right(clock_walls, wall) - 1
        return clocks[index][1] if index >= 0 else None

    def latest_before(events, wall):
        return next((item for item in reversed(events) if item[0] <= wall), None)

    real_odom = [item for item in odoms if math.hypot(item[2]-start[0], item[3]-start[1]) < 0.5]
    real_tf = [item for item in tfs if math.hypot(item[2]-start[0], item[3]-start[1]) < 0.5]
    latest_odom = latest_before(odoms, first) if first is not None else None
    latest_tf = latest_before(tfs, first) if first is not None else None
    latest_map = latest_before(maps, first) if first is not None else None
    in_bounds = None
    if latest_map and latest_tf:
        _, xmin, ymin, width, height = latest_map
        _, _, x, y = latest_tf
        radius = 0.267
        in_bounds = (xmin+radius <= x < xmin+width-radius
                     and ymin+radius <= y < ymin+height-radius)
    report = {
        'bag': str(db),
        'scenario': str(args.scenario),
        'goal_pose_topic_messages': len(goals),
        'high_level_uuid_count': len(statuses),
        'statuses': statuses,
        'first_high_level_status_wall': first,
        'first_high_level_status_sim': sim_at(first) if first is not None else None,
        'first_real_odom_wall': real_odom[0][0] if real_odom else None,
        'first_real_tf_wall': real_tf[0][0] if real_tf else None,
        'last_odom_before_first_status': latest_odom,
        'last_tf_before_first_status': latest_tf,
        'last_global_costmap_before_first_status': latest_map,
        'robot_inside_global_costmap_at_first_status': in_bounds,
        'nav2_activation_before_first_status': [event for event in lifecycle
                                               if first is not None and event[0] <= first
                                               and event[3] == 'active'],
    }
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
