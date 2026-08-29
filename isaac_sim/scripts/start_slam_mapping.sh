#!/usr/bin/env bash
# Start slam_toolbox and RViz against the running Isaac 6.0.1 ROS graph.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
SLAM_CONFIG="${ISAAC_SLAM_CONFIG:-${PROJECT_ROOT}/isaac_sim/config/isaac_slam_online_async.yaml}"
RVIZ_CONFIG="${ISAAC_SLAM_RVIZ_CONFIG:-${ROS_WS}/src/semantic_nav_gazebo/rviz/v7_dual_slam.rviz}"
START_RVIZ="${ISAAC_SLAM_START_RVIZ:-1}"
READY_TIMEOUT="${ISAAC_SLAM_READY_TIMEOUT:-180}"
ISAAC_ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
LOG_DIR="${PROJECT_ROOT}/isaac_sim/maps/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
SLAM_LOG="${LOG_DIR}/${STAMP}_isaac_slam_toolbox.log"
RVIZ_LOG="${LOG_DIR}/${STAMP}_isaac_slam_rviz.log"

case "${START_RVIZ}" in
    0|1) ;;
    *) echo "ERROR: ISAAC_SLAM_START_RVIZ must be 0 or 1." >&2; exit 2 ;;
esac
if [[ ! "${READY_TIMEOUT}" =~ ^[0-9]+$ ]] || (( READY_TIMEOUT < 1 )); then
    echo "ERROR: ISAAC_SLAM_READY_TIMEOUT must be a positive integer." >&2
    exit 2
fi
for required in \
    /opt/ros/humble/setup.bash \
    "${ROS_WS}/install/setup.bash" \
    "${SLAM_CONFIG}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file is missing: ${required}" >&2
        exit 1
    fi
done
if (( START_RVIZ )) && [[ ! -f "${RVIZ_CONFIG}" ]]; then
    echo "ERROR: RViz config is missing: ${RVIZ_CONFIG}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
set -u

# Keep exactly the same DDS context as the isolated Isaac launcher and teleop.
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1
mkdir -p "${LOG_DIR}"

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    echo "ERROR: ROS package slam_toolbox is unavailable." >&2
    exit 1
fi
if (( START_RVIZ )) && ! ros2 pkg prefix rviz2 >/dev/null 2>&1; then
    echo "ERROR: ROS package rviz2 is unavailable." >&2
    exit 1
fi
if ros2 node list --no-daemon --spin-time 1.0 2>/dev/null | grep -qx '/slam_toolbox'; then
    echo "ERROR: /slam_toolbox is already running. Do not start a second mapper." >&2
    exit 3
fi

topic_has_publisher() {
    local topic="$1"
    local expected_type="$2"
    local info
    info="$(ros2 topic info "${topic}" --no-daemon --spin-time 0.5 2>/dev/null || true)"
    grep -q "^Type: ${expected_type}$" <<<"${info}" &&
        awk '/^Publisher count:/ {if ($3 + 0 > 0) found=1} END {exit !found}' <<<"${info}"
}

echo "Waiting up to ${READY_TIMEOUT}s for live, time-aligned Isaac SLAM input..."
if ! python3 "${SCRIPT_DIR}/check_slam_time_sync.py" \
        --timeout "${READY_TIMEOUT}" --max-delta 1.0; then
    echo "ERROR: the Isaac SLAM input graph is incomplete, stale, or time-misaligned." >&2
    echo "Start or fully restart the Isaac launcher and wait for WAREHOUSE_PEOPLE_ROBOT_READY=." >&2
    exit 4
fi

slam_pid=""
rviz_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ "${rviz_pid}" =~ ^[0-9]+$ ]] && kill -0 "${rviz_pid}" 2>/dev/null; then
        kill -TERM -- "-${rviz_pid}" 2>/dev/null || true
        wait "${rviz_pid}" 2>/dev/null || true
    fi
    if [[ "${slam_pid}" =~ ^[0-9]+$ ]] && kill -0 "${slam_pid}" 2>/dev/null; then
        kill -INT -- "-${slam_pid}" 2>/dev/null || true
        wait "${slam_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting Isaac SLAM mapping."
echo "  config: ${SLAM_CONFIG}"
echo "  input:  /scan_merged; TF odom -> base_link"
echo "  log:    ${SLAM_LOG}"
setsid ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file "${SLAM_CONFIG}" \
    > >(tee "${SLAM_LOG}") 2>&1 &
slam_pid=$!

map_deadline=$((SECONDS + 30))
map_ready=0
while (( SECONDS < map_deadline )); do
    if ! kill -0 "${slam_pid}" 2>/dev/null; then
        wait "${slam_pid}" || true
        echo "ERROR: slam_toolbox exited before publishing /map. See ${SLAM_LOG}" >&2
        exit 5
    fi
    if topic_has_publisher /map nav_msgs/msg/OccupancyGrid &&
       timeout 3s ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
           --once --no-daemon --spin-time 1.0 \
           --qos-durability transient_local --field info \
           >/dev/null 2>&1; then
        map_ready=1
        break
    fi
    sleep 1
done
if (( ! map_ready )); then
    echo "ERROR: slam_toolbox did not publish /map within 30 seconds. See ${SLAM_LOG}" >&2
    exit 5
fi

if (( START_RVIZ )); then
    echo "Starting RViz: ${RVIZ_CONFIG}"
    echo "  RViz log: ${RVIZ_LOG}"
    # A terminal embedded in the Snap build of VS Code exports GTK/GIO paths
    # from /snap/code and /snap/core20.  Native Ubuntu RViz must not inherit
    # those libraries; mixing their glibc with the host glibc causes the
    # __libc_pthread_init@GLIBC_PRIVATE startup failure seen on this host.
    (
        for variable in \
            SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA \
            SNAP_EUID SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET \
            SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REVISION SNAP_UID \
            SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION \
            GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE \
            GIO_LAUNCHED_DESKTOP_FILE GIO_LAUNCHED_DESKTOP_FILE_PID \
            GIO_MODULE_DIR GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH \
            GSETTINGS_SCHEMA_DIR LOCPATH XDG_DATA_HOME; do
            unset "${variable}"
        done
        export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/share/ubuntu:/usr/share/gnome:/usr/local/share:/usr/share:/var/lib/snapd/desktop}"
        export XDG_CONFIG_DIRS="${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-/etc/xdg/xdg-ubuntu:/etc/xdg}"
        exec setsid ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" \
            --ros-args -p use_sim_time:=true
    ) > >(tee "${RVIZ_LOG}") 2>&1 &
    rviz_pid=$!
fi

echo "ISAAC_SLAM_READY: drive with isaac_sim/scripts/teleop_robot.sh"
echo "When mapping is complete, run isaac_sim/scripts/save_slam_map.sh in another terminal."
echo "Keep this terminal open; Ctrl-C stops SLAM and RViz."

set +e
wait "${slam_pid}"
status=$?
set -e
slam_pid=""
if (( status != 0 )); then
    echo "ERROR: slam_toolbox exited with status ${status}. See ${SLAM_LOG}" >&2
fi
exit "${status}"
