#!/usr/bin/env bash
# Run only after resetting the robot to the default spawn in the static scene.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/isaac_sim/arena_ws/"* ]]; then
    echo "ERROR: arena_ws is sourced in this shell. Open a fresh terminal." >&2
    exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1

arena_fragment="/isaac_sim/arena_ws"
for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH \
    LD_LIBRARY_PATH PATH ROS_PACKAGE_PATH; do
    if [[ "${!variable:-}" == *"${arena_fragment}"* ]]; then
        echo "ERROR: arena_ws remains in ${variable}. Use a fresh shell." >&2
        exit 2
    fi
done

exec python3 "$SCRIPT_DIR/tools/send_omni_follow_path.py" "$@"
