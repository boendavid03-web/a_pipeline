#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAAC_SIM_5_ROOT="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"
ROS2_BRIDGE_ROOT="$ISAAC_SIM_5_ROOT/exts/isaacsim.ros2.bridge/humble"

if [[ ! -f "$ISAAC_SIM_5_ROOT/VERSION" ]] || ! grep -q '^5\.1\.0' "$ISAAC_SIM_5_ROOT/VERSION"; then
    echo "ERROR: invalid Isaac Sim 5.1 root: $ISAAC_SIM_5_ROOT" >&2
    exit 1
fi
for required in \
    "$ISAAC_SIM_5_ROOT/python.sh" \
    "$ROS2_BRIDGE_ROOT/lib" \
    "$BACKEND_ROOT/generated/minimal_core_physx_no_rtx.kit"; do
    [[ -e "$required" ]] || { echo "ERROR: missing required path: $required" >&2; exit 1; }
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
    ISAAC_SIM_ROOT ISAAC_SIM_PATH ISAAC_PATH ISAACSIM_PATH PYTHONHOME PYTHONPATH \
    LD_PRELOAD VK_LAYER_PATH || true
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ISAAC5_GATE5_ROS_DOMAIN_ID:-52}"
export ROS_LOCALHOST_ONLY=1
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$ROS2_BRIDGE_ROOT/lib"

exec "$ISAAC_SIM_5_ROOT/python.sh" "$BACKEND_ROOT/runtime/stage2a_dual_lidar_probe.py"
