#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/user/isaacsim/5.1.0}"

if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim python.sh not found: $ISAAC_SIM_ROOT/python.sh" >&2
    exit 1
fi

export ROS_DISTRO="humble"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS2_BRIDGE_LIB="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/humble/lib"
if [[ ! -d "$ROS2_BRIDGE_LIB" ]]; then
    echo "ERROR: Isaac Sim ROS 2 Humble libraries not found: $ROS2_BRIDGE_LIB" >&2
    exit 1
fi
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROS2_BRIDGE_LIB"

exec "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/run_navigation.py" "$@"
