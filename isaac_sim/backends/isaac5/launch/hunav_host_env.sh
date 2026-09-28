#!/usr/bin/env bash
# Source-only CPython 3.10/HuNav environment. Never source this inside Isaac.
set -euo pipefail

ARENA_WS="${ARENA_WS:-/home/user/arena_full_ws}"
set +u
source /opt/ros/humble/setup.bash
set -u
export AMENT_PREFIX_PATH="$ARENA_WS/install_harmonic:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$ARENA_WS/build_harmonic/hunav_msgs/rosidl_generator_py:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ARENA_WS/install_harmonic/lib:$ARENA_WS/build_harmonic/hunav_msgs:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ISAAC_HUNAV_ROS_DOMAIN_ID:-54}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
