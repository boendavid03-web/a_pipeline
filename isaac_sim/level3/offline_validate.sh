#!/usr/bin/env bash
# CPU-light static validation only. Never starts ROS nodes or a simulator.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]] || \
   [[ "${CMAKE_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]] || \
   [[ "${PYTHONPATH:-}" == *"/isaac_sim/arena_ws/"* ]]; then
    echo "ERROR: arena_ws is sourced. Static validation must use a fresh shell." >&2
    exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${TMPDIR:-/tmp}/a_pipeline_level3_static_ros_log"
export MPLCONFIGDIR="${TMPDIR:-/tmp}/a_pipeline_level3_matplotlib"
mkdir -p "$ROS_LOG_DIR" "$MPLCONFIGDIR"

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

mapfile -t shell_files < <(find "$SCRIPT_DIR" -type f -name '*.sh' -print | sort)
for path in "${shell_files[@]}"; do
    bash -n "$path"
done
echo "STATIC_BASH_SYNTAX=PASS files=${#shell_files[@]}"

python3 "$SCRIPT_DIR/tools/calibrate_map_alignment.py" \
    --output "$SCRIPT_DIR/reports/map_alignment.json" \
    --overlay "$SCRIPT_DIR/reports/map_alignment_overlay.png"
python3 "$PROJECT_ROOT/isaac_sim/scripts/convert_gazebo_boxes_to_usda.py" \
    --expected-boxes 79 --check
export LEVEL3_CUSTOM_SCENE_CHECK=PASS
python3 "$SCRIPT_DIR/tools/validate_test_routes.py" \
    --output "$SCRIPT_DIR/reports/test_routes.json"

required_packages=(
    nav2_map_server nav2_planner nav2_controller nav2_mppi_controller
    nav2_behaviors nav2_bt_navigator nav2_velocity_smoother
    nav2_lifecycle_manager nav2_navfn_planner nav2_costmap_2d tf2_ros
)
for package in "${required_packages[@]}"; do
    prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
    if [[ "$prefix" != "/opt/ros/humble" ]]; then
        echo "STATIC_PACKAGE_DISCOVERY=FAIL package=$package prefix=${prefix:-NOT_FOUND}" >&2
        exit 1
    fi
done
echo "STATIC_PACKAGE_DISCOVERY=PASS source=/opt/ros/humble"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]]; then
    echo "STATIC_ARENA_ISOLATION=FAIL" >&2
    exit 1
fi
echo "STATIC_ARENA_ISOLATION=PASS"

exec python3 "$SCRIPT_DIR/tools/offline_validate_level3.py"
