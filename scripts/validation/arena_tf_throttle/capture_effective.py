#!/usr/bin/env python3
"""Capture runtime ROS graph and parameters without changing them."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, ROS2CLI_NO_DAEMON='1')
target = '/task_generator_node/jackal/local_costmap/local_costmap'
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    try:
        p = subprocess.run(['ros2', 'node', 'list'], env=env, capture_output=True,
                           text=True, timeout=5)
        if target in p.stdout.splitlines():
            break
    except subprocess.TimeoutExpired:
        pass
    time.sleep(1)
else:
    (out / 'capture_status.json').write_text(json.dumps({'error':'local_costmap node not found'})+'\n')
    sys.exit(2)

commands = {
    'effective_local_costmap_params.yaml': ['ros2','param','dump',target],
    'effective_controller_params.yaml': ['ros2','param','dump','/task_generator_node/jackal/controller_server'],
    'effective_task_generator_params.yaml': ['ros2','param','dump','/task_generator_node'],
    'local_node_info.txt': ['ros2','node','info',target],
    'lidar_topic_info.txt': ['ros2','topic','info','-v','/task_generator_node/jackal/lidar'],
    'topic_list.txt': ['ros2','topic','list'],
}
status = {}
for filename, cmd in commands.items():
    try:
        p = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=18)
        (out / filename).write_text(p.stdout)
        if p.stderr:
            (out / (filename+'.stderr')).write_text(p.stderr)
        status[filename] = {'exit_code':p.returncode, 'bytes':len(p.stdout)}
    except subprocess.TimeoutExpired:
        status[filename] = {'timeout_s':18}

for name in ['voxel_grid','clearing_endpoints']:
    topic = f'/task_generator_node/jackal/local_costmap/{name}'
    filename = name+'_topic_info.txt'
    try:
        p = subprocess.run(['ros2','topic','info','-v',topic], env=env,
                           capture_output=True, text=True, timeout=10)
        (out / filename).write_text(p.stdout + p.stderr)
        status[filename] = {'exit_code':p.returncode, 'bytes':len(p.stdout)}
    except subprocess.TimeoutExpired:
        status[filename] = {'timeout_s':10}
(out / 'capture_status.json').write_text(json.dumps(status, indent=2)+'\n')
print(json.dumps(status), flush=True)
