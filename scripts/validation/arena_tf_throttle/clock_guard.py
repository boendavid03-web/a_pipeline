#!/usr/bin/env python3
"""Stop signal for the one bounded default diagnostic; no simulation control."""
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

import os
target = float(os.environ.get("ARENA_CLOCK_TARGET", "30"))
wall_limit = float(os.environ.get("ARENA_CLOCK_WALL_LIMIT", "180"))
first_wall = time.monotonic()
last_report = -1
last_sim = None

rclpy.init()
node = rclpy.create_node('arena_costmap_diagnostic_clock_guard')

def on_clock(msg):
    global last_report, last_sim
    value = msg.clock.sec + msg.clock.nanosec * 1e-9
    last_sim = value
    bucket = int(value // 5)
    if bucket > last_report:
        last_report = bucket
        print(f'CLOCK sim={value:.3f} wall_elapsed={time.monotonic()-first_wall:.3f}', flush=True)

sub = node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
try:
    while rclpy.ok() and time.monotonic()-first_wall < wall_limit:
        rclpy.spin_once(node, timeout_sec=0.2)
        if last_sim is not None and last_sim >= target:
            print(f'CLOCK_TARGET_REACHED sim={last_sim:.6f}', flush=True)
            sys.exit(0)
    print(f'CLOCK_WALL_TIMEOUT last_sim={last_sim}', flush=True)
    sys.exit(124)
finally:
    node.destroy_node()
    rclpy.shutdown()
