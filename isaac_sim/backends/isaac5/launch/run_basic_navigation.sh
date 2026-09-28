#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/user/isaacsim/5.1.0}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONNOUSERSITE=1
unset ISAAC_PATH ISAACSIM_PATH OMNI_KIT_ALLOW_ROOT OMNI_KIT_CACHE_PATH || true

if [[ " ${*} " == *" --ros "* || " ${*} " == *" --phase ros "* ]]; then
  ROS2_BRIDGE_LIB="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/humble/lib"
  export LD_LIBRARY_PATH="${ROS2_BRIDGE_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

exec "${ISAAC_SIM_ROOT}/python.sh" "${BACKEND_ROOT}/runtime/basic_navigation.py" "$@"
