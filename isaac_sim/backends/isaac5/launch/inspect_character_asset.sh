#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAAC_SIM_5_ROOT="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"

if [[ ! -f "$ISAAC_SIM_5_ROOT/VERSION" ]] || ! grep -q '^5\.1\.0' "$ISAAC_SIM_5_ROOT/VERSION"; then
    echo "ERROR: invalid Isaac Sim 5.1 root: $ISAAC_SIM_5_ROOT" >&2
    exit 1
fi

unset ROS_DISTRO AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
    RMW_IMPLEMENTATION ISAAC_SIM_ROOT ISAAC_SIM_PATH ISAAC_PATH ISAACSIM_PATH \
    PYTHONHOME PYTHONPATH LD_PRELOAD VK_LAYER_PATH || true
export PYTHONNOUSERSITE=1

exec "$ISAAC_SIM_5_ROOT/python.sh" "$BACKEND_ROOT/runtime/inspect_character_asset.py" "$@"
