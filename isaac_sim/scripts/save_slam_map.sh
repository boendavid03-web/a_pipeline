#!/usr/bin/env bash
# Save the current slam_toolbox occupancy map without overwriting old maps.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
ISAAC_ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
MAP_DIR="${ISAAC_SLAM_MAP_DIR:-${PROJECT_ROOT}/isaac_sim/maps/slam}"
MAP_NAME="${1:-$(date +%Y%m%d_%H%M%S)_isaac_warehouse_slam}"
LOG_DIR="${PROJECT_ROOT}/isaac_sim/maps/logs"
LOG_FILE="${LOG_DIR}/$(date +%Y%m%d_%H%M%S)_isaac_slam_save.log"

if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "ERROR: map name may contain only letters, numbers, dot, underscore and dash." >&2
    exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "${ROS_WS}/install/setup.bash" ]]; then
    echo "ERROR: ROS 2 Humble or the project workspace is not available." >&2
    exit 1
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1

mkdir -p "${MAP_DIR}" "${LOG_DIR}"
MAP_PREFIX="${MAP_DIR}/${MAP_NAME}"
MAP_YAML="${MAP_PREFIX}.yaml"
MAP_PGM="${MAP_PREFIX}.pgm"
if [[ -e "${MAP_YAML}" || -e "${MAP_PGM}" ]]; then
    echo "ERROR: refusing to overwrite existing map: ${MAP_PREFIX}.{yaml,pgm}" >&2
    exit 3
fi
if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    echo "ERROR: ROS package nav2_map_server is unavailable." >&2
    exit 1
fi
map_info="$(ros2 topic info /map --no-daemon --spin-time 2.0 2>/dev/null || true)"
if ! grep -q '^Type: nav_msgs/msg/OccupancyGrid$' <<<"${map_info}" ||
   ! awk '/^Publisher count:/ {if ($3 + 0 > 0) found=1} END {exit !found}' <<<"${map_info}"; then
    echo "ERROR: no live /map publisher; start Isaac SLAM before saving." >&2
    exit 4
fi
if ! timeout 10s ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
        --once --no-daemon --spin-time 2.0 \
        --qos-durability transient_local --field info >/dev/null 2>&1; then
    echo "ERROR: /map was discovered but no occupancy grid was received." >&2
    exit 4
fi
if ! timeout 8s ros2 topic echo /clock rosgraph_msgs/msg/Clock \
        --once --no-daemon --spin-time 2.0 --field clock >/dev/null 2>&1; then
    echo "ERROR: /clock is not advancing; map_saver_cli uses simulation time." >&2
    exit 4
fi

{
    echo "Saving Isaac SLAM map to ${MAP_PREFIX}"
    ros2 run nav2_map_server map_saver_cli -f "${MAP_PREFIX}" \
        --ros-args -p use_sim_time:=true
    test -s "${MAP_YAML}"
    test -s "${MAP_PGM}"
    echo
    ls -lh "${MAP_YAML}" "${MAP_PGM}"
    echo
    sha256sum "${MAP_YAML}" "${MAP_PGM}"
    echo "ISAAC_SLAM_MAP_SAVED=${MAP_PREFIX}"
} 2>&1 | tee "${LOG_FILE}"

echo "Wrote ${LOG_FILE}"
