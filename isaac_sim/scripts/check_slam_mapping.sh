#!/usr/bin/env bash
# Verify that the live Isaac graph is sufficient for SLAM and map saving.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
ISAAC_ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
LOG_DIR="${PROJECT_ROOT}/isaac_sim/maps/logs"
LOG_FILE="${LOG_DIR}/$(date +%Y%m%d_%H%M%S)_isaac_slam_check.log"

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
mkdir -p "${LOG_DIR}"

failures=0
check_topic() {
    local topic="$1"
    local expected_type="$2"
    local attempt info="" type="" publishers=""
    # A new no-daemon Fast DDS participant can occasionally need more than
    # one discovery round. Retry so a healthy live publisher is not reported
    # missing due to a one-shot discovery race.
    for attempt in 1 2 3; do
        info="$(ros2 topic info "${topic}" --no-daemon --spin-time 1.0 2>/dev/null || true)"
        type="$(awk '/^Type:/ {print $2}' <<<"${info}")"
        publishers="$(awk '/^Publisher count:/ {print $3}' <<<"${info}")"
        if [[ "${type}" == "${expected_type}" && "${publishers:-0}" =~ ^[0-9]+$ ]] &&
           (( publishers > 0 )); then
            echo "PASS ${topic} type=${type} publishers=${publishers}"
            return
        fi
    done
    echo "FAIL ${topic} expected=${expected_type} actual=${type:-missing} publishers=${publishers:-0}"
    failures=$((failures + 1))
}

check_node() {
    local node_name="$1"
    local attempt
    for attempt in 1 2 3 4 5; do
        if ros2 node list --no-daemon --spin-time 1.5 2>/dev/null |
                grep -Fxq -- "${node_name}"; then
            echo "PASS ${node_name}"
            return
        fi
    done
    echo "FAIL ${node_name} is not running"
    failures=$((failures + 1))
}

{
    echo "===== Isaac SLAM topic contract ====="
    check_topic /scan_01 sensor_msgs/msg/LaserScan
    check_topic /scan_02 sensor_msgs/msg/LaserScan
    check_topic /scan_merged sensor_msgs/msg/LaserScan
    check_topic /odom nav_msgs/msg/Odometry
    check_topic /tf tf2_msgs/msg/TFMessage
    check_topic /tf_static tf2_msgs/msg/TFMessage
    check_topic /clock rosgraph_msgs/msg/Clock
    check_topic /map nav_msgs/msg/OccupancyGrid

    echo
    echo "===== Time alignment ====="
    if python3 "${SCRIPT_DIR}/check_slam_time_sync.py" \
            --timeout 8.0 --max-delta 1.0; then
        :
    else
        failures=$((failures + 1))
    fi

    echo
    echo "===== SLAM node ====="
    check_node /slam_toolbox

    echo
    echo "===== Live messages ====="
    if timeout 8s ros2 topic echo /scan_merged sensor_msgs/msg/LaserScan \
            --once --no-daemon --spin-time 2.0 --field header >/dev/null 2>&1; then
        echo "PASS live /scan_merged"
    else
        echo "FAIL no live /scan_merged message"
        failures=$((failures + 1))
    fi
    if timeout 8s ros2 topic echo /clock rosgraph_msgs/msg/Clock \
            --once --no-daemon --spin-time 2.0 --field clock >/dev/null 2>&1; then
        echo "PASS live /clock"
    else
        echo "FAIL no live /clock message"
        failures=$((failures + 1))
    fi
    if timeout 8s ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
            --once --no-daemon --spin-time 2.0 \
            --qos-durability transient_local --field info >/dev/null 2>&1; then
        echo "PASS live /map"
    else
        echo "FAIL no live /map message"
        failures=$((failures + 1))
    fi

    echo
    if (( failures == 0 )); then
        echo "ISAAC_SLAM_CHECK=PASS"
    else
        echo "ISAAC_SLAM_CHECK=FAIL failures=${failures}"
    fi
} > >(tee "${LOG_FILE}") 2>&1

echo "Wrote ${LOG_FILE}"
(( failures == 0 ))
