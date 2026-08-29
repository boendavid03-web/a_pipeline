#!/usr/bin/env bash
# Start system Nav2 only. This script never starts Isaac, Gazebo, Arena, or RViz.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
LAUNCH_FILE="$SCRIPT_DIR/launch/standalone_level3.launch.py"
RUNNER="$SCRIPT_DIR/tools/run_launch_file.py"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]] || \
   [[ "${CMAKE_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]] || \
   [[ "${PYTHONPATH:-}" == *"/isaac_sim/arena_ws/"* ]]; then
    echo "ERROR: arena_ws is sourced in this shell. Open a fresh terminal." >&2
    exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
    echo "ERROR: ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
    exit 2
fi
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1

arena_fragment="/isaac_sim/arena_ws"
arena_env_vars=(
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
    LD_LIBRARY_PATH PATH ROS_PACKAGE_PATH
)
for variable in "${arena_env_vars[@]}"; do
    if [[ "${!variable:-}" == *"${arena_fragment}"* ]]; then
        echo "ERROR: arena_ws remains in ${variable}. Use a fresh shell." >&2
        exit 2
    fi
done

# Refuse stale alignment/config inputs before creating any ROS node.
bash "$SCRIPT_DIR/offline_validate.sh" >/dev/null
echo "OFFLINE_LEVEL3_PREP=PASS"

required_system_packages=(
    nav2_map_server nav2_planner nav2_controller nav2_mppi_controller
    nav2_behaviors nav2_bt_navigator nav2_velocity_smoother
    nav2_lifecycle_manager tf2_ros
)
for package in "${required_system_packages[@]}"; do
    prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
    if [[ "$prefix" != "/opt/ros/humble" ]]; then
        echo "ERROR: $package must resolve to /opt/ros/humble, got ${prefix:-NOT_FOUND}" >&2
        exit 2
    fi
done

semantic_prefix="$(ros2 pkg prefix semantic_nav_gazebo 2>/dev/null || true)"
if [[ "$semantic_prefix" != "$ROS_WS/install/semantic_nav_gazebo" ]]; then
    echo "ERROR: semantic_nav_gazebo is not from the standalone ROS workspace." >&2
    exit 2
fi

existing_nodes="$(ros2 node list 2>/dev/null || true)"
for forbidden in \
    /amcl /slam_toolbox /map_server /planner_server /controller_server \
    /behavior_server /bt_navigator /velocity_smoother \
    /level3_ground_truth_map_to_odom /map_to_odom_capture_tf \
    /map_to_odom_static_tf_publisher /map_broadcaster \
    /level3_goal_pose_adapter /lifecycle_manager_level3; do
    if grep -Fxq "$forbidden" <<<"$existing_nodes"; then
        echo "ERROR: conflicting ROS node already exists: $forbidden" >&2
        exit 2
    fi
done

if ! grep -Fxq "/isaac_6_udp_ros_bridge" <<<"$existing_nodes"; then
    echo "ERROR: Isaac ROS/UDP bridge is not ready (/isaac_6_udp_ros_bridge missing)." >&2
    exit 2
fi
if ! grep -Fxq "/v7_dual_laser_scan_merger" <<<"$existing_nodes"; then
    echo "ERROR: dual-laser merger is not ready (/v7_dual_laser_scan_merger missing)." >&2
    exit 2
fi

echo "LEVEL3_BRINGUP_STARTING map=existing localization=ground_truth controller=MPPI_Omni"
exec python3 "$RUNNER" "$LAUNCH_FILE" "$@"
