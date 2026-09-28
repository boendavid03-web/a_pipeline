#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ISAAC_SIM_5_ROOT="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"
ROBOT_USD="/home/user/navigation_project/robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd"
ISAAC5_ROS_CP311_OVERLAY_INSTALL="${ISAAC5_ROS_CP311_OVERLAY_INSTALL:-$PROJECT_ROOT/sim_to_real/robot/comparison_models/ros2_ws_cp311_overlay/install}"

if [[ ! -f "$ISAAC_SIM_5_ROOT/VERSION" ]] || ! grep -q '^5\.1\.0' "$ISAAC_SIM_5_ROOT/VERSION"; then
    echo "ERROR: ISAAC_SIM_5_ROOT is not an Isaac Sim 5.1.0 installation: $ISAAC_SIM_5_ROOT" >&2
    exit 1
fi
if [[ ! -x "$ISAAC_SIM_5_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim 5.1 python.sh not found: $ISAAC_SIM_5_ROOT/python.sh" >&2
    exit 1
fi
if [[ ! -f "$ROBOT_USD" ]]; then
    echo "ERROR: canonical robot USD not found: $ROBOT_USD" >&2
    exit 1
fi

export ROS_DISTRO="humble"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS2_BRIDGE_LIB="$ISAAC_SIM_5_ROOT/exts/isaacsim.ros2.bridge/humble/lib"
if [[ ! -d "$ROS2_BRIDGE_LIB" ]]; then
    echo "ERROR: Isaac Sim 5.1 ROS 2 Humble libraries not found: $ROS2_BRIDGE_LIB" >&2
    exit 1
fi
if ! find "$ROS2_BRIDGE_LIB" -maxdepth 1 -type f -name 'libstd_msgs__rosidl_generator_py.so' -print -quit | grep -q .; then
    echo "ERROR: Isaac Sim 5.1 Humble std_msgs Python support is missing: $ROS2_BRIDGE_LIB" >&2
    exit 1
fi

# Do not let a parent Isaac installation or host ROS Python package enter Kit.
unset ISAAC_SIM_ROOT ISAAC_SIM_PATH ISAAC_PATH ISAACSIM_PATH ISAAC_ASSET_ROOT ISAAC_ASSETS_ROOT \
    ISAACSIM_ASSET_ROOT OMNI_KIT_ROOT OMNI_KIT_ALLOW_ROOT CARB_APP_PATH EXP_PATH \
    OMNI_KIT_APP OMNI_KIT_EXTENSIONS_PATH OMNI_KIT_EXT_PATH OMNI_KIT_CACHE_DIR \
    OMNI_USER_PATH PXR_PLUGINPATH_NAME PYTHONHOME PYTHONPATH LD_PRELOAD VK_LAYER_PATH || true
export PYTHONNOUSERSITE=1
export ISAAC5_ROS_CP311_OVERLAY_INSTALL
export LD_LIBRARY_PATH="$ROS2_BRIDGE_LIB:$ISAAC5_ROS_CP311_OVERLAY_INSTALL/navigation_evaluation_msgs/lib"

if ! "$ISAAC_SIM_5_ROOT/python.sh" -c 'import sys; assert sys.version_info[:2] == (3, 11), sys.version; print(sys.version)' >/dev/null; then
    echo "ERROR: Isaac Sim 5.1 embedded Python is not 3.11" >&2
    exit 1
fi

exec "$ISAAC_SIM_5_ROOT/python.sh" "$BACKEND_ROOT/runtime/run_navigation.py" "$@"
