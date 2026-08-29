#!/usr/bin/env bash
# Record one manual Isaac 6.0.1 teleoperation session using the Gazebo/V7 topic contract.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
DURATION_SEC="${1:-0}"
BAG_ROOT="${ISAAC_BAG_ROOT:-${PROJECT_ROOT}/isaac_sim/bags}"
ISAAC_ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
REQUIRE_LIDAR_INTENSITY="${ISAAC_REQUIRE_LIDAR_INTENSITY:-1}"
REQUIRE_REALTIME_LIDAR="${ISAAC_REQUIRE_REALTIME_LIDAR:-0}"

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "ERROR: duration must be zero or a positive number" >&2
    exit 2
fi
if [[ "${REQUIRE_LIDAR_INTENSITY}" != "0" && "${REQUIRE_LIDAR_INTENSITY}" != "1" ]]; then
    echo "ERROR: ISAAC_REQUIRE_LIDAR_INTENSITY must be 0 or 1" >&2
    exit 2
fi
if [[ "${REQUIRE_REALTIME_LIDAR}" != "0" && "${REQUIRE_REALTIME_LIDAR}" != "1" ]]; then
    echo "ERROR: ISAAC_REQUIRE_REALTIME_LIDAR must be 0 or 1" >&2
    exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
    exit 1
fi
if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
    echo "ERROR: ROS 2 workspace is not built: ${ROS_WS}/install/setup.bash" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
set -u

# These must exactly match the Isaac launcher, relay, and keyboard terminal.
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="1"

TOPICS=(
    /scan
    /scan_01
    /scan_02
    /scan_merged
    /odom
    /tf
    /tf_static
    /clock
    /cmd_vel
    /cmd_vel_stamped
    /pedestrian_ground_truth
    /data_collection/episode_event
    /data_collection/sensor_config
)

echo "Checking Isaac teleoperation capture topics..."
echo "ROS: RMW=${RMW_IMPLEMENTATION}, domain=${ROS_DOMAIN_ID}, localhost=${ROS_LOCALHOST_ONLY}"
CAPTURE_CHECK_ARGS=(--timeout 20.0)
CAPTURE_CHECK_ARGS+=(--verify-lidar-rate)
if [[ "${REQUIRE_REALTIME_LIDAR}" == "1" ]]; then
    CAPTURE_CHECK_ARGS+=(--require-realtime-lidar)
fi
if [[ "${REQUIRE_LIDAR_INTENSITY}" == "1" ]]; then
    CAPTURE_CHECK_ARGS+=(--require-lidar-intensity)
fi
if ! python3 "${SCRIPT_DIR}/check_capture_ready.py" "${CAPTURE_CHECK_ARGS[@]}"; then
    echo "ERROR: Isaac capture graph is incomplete, frozen, or has multiple teleop publishers" >&2
    exit 3
fi

mkdir -p "${BAG_ROOT}" "${BAG_ROOT}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${BAG_ROOT}/${STAMP}_isaac_6_teleop"
while [[ -e "${BAG_DIR}" ]]; do
    sleep 1
    STAMP="$(date +%Y%m%d_%H%M%S)"
    BAG_DIR="${BAG_ROOT}/${STAMP}_isaac_6_teleop"
done
RECORDER_LOG="${BAG_ROOT}/logs/${STAMP}_isaac_6_teleop_record.log"
SESSION_MANIFEST="${BAG_ROOT}/logs/${STAMP}_isaac_6_teleop.env"

{
    printf 'STARTED_AT=%q\n' "$(date --iso-8601=seconds)"
    printf 'BAG_DIR=%q\n' "${BAG_DIR}"
    printf 'DURATION_SIM_SEC=%q\n' "${DURATION_SEC}"
    printf 'ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID}"
    printf 'RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION}"
    printf 'ROS_LOCALHOST_ONLY=%q\n' "${ROS_LOCALHOST_ONLY}"
    printf 'REQUIRE_LIDAR_INTENSITY=%q\n' "${REQUIRE_LIDAR_INTENSITY}"
    printf 'REQUIRE_REALTIME_LIDAR=%q\n' "${REQUIRE_REALTIME_LIDAR}"
    printf 'CALLER_LIDAR_RATE_HZ=%q\n' "${ISAAC_LIDAR_RATE_HZ:-auto_from_simulator}"
    printf 'CALLER_LIDAR_SAMPLE_COUNT=%q\n' "${ISAAC_LIDAR_SAMPLE_COUNT:-auto_from_simulator}"
    printf 'RECORDED_TOPICS=%q\n' "${TOPICS[*]}"
} >"${SESSION_MANIFEST}"

RECORDER_PID=""
stop_recorder() {
    if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
        kill -INT "${RECORDER_PID}" >/dev/null 2>&1 || true
    fi
    if [[ -n "${RECORDER_PID}" ]]; then
        wait "${RECORDER_PID}" >/dev/null 2>&1 || true
    fi
    RECORDER_PID=""
}
handle_interrupt() {
    echo
    echo "Stopping recorder and finalizing metadata..."
    stop_recorder
}
trap handle_interrupt INT TERM
trap stop_recorder EXIT

echo "Recording to ${BAG_DIR}"
echo "Recorder log: ${RECORDER_LOG}"
ros2 bag record --storage sqlite3 --output "${BAG_DIR}" "${TOPICS[@]}" \
    >"${RECORDER_LOG}" 2>&1 &
RECORDER_PID="$!"

ready=0
for _ in $(seq 1 15); do
    if ! kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
        echo "ERROR: rosbag recorder exited during startup" >&2
        tail -n 40 "${RECORDER_LOG}" >&2 || true
        exit 4
    fi
    if grep -q "All requested topics are subscribed" "${RECORDER_LOG}"; then
        ready=1
        break
    fi
    sleep 1
done
if [[ "${ready}" != "1" ]]; then
    echo "ERROR: recorder did not subscribe to every requested topic within 15 seconds" >&2
    tail -n 40 "${RECORDER_LOG}" >&2 || true
    exit 4
fi

# Establish the common /clock/scan/odom interval before the operator is told
# to move.  This prevents the first nonzero stamped command from preceding the
# bag's first /clock sample during recorder discovery.
if ! python3 "${ROS_WS}/tools/wait_for_sim_duration.py" \
    --duration 0.2 --progress-interval 1.0 --startup-timeout 20.0 \
    >/dev/null; then
    echo "ERROR: /clock stopped while the recorder was starting" >&2
    exit 4
fi
echo "CAPTURE_READY: start driving now. Keep the teleop terminal focused."
if awk -v value="${DURATION_SEC}" 'BEGIN {exit !(value > 0.0)}'; then
    echo "The recorder will stop after ${DURATION_SEC} seconds of /clock simulation time."
    wait_status=0
    python3 "${ROS_WS}/tools/wait_for_sim_duration.py" \
        --duration "${DURATION_SEC}" --progress-interval 10.0 --startup-timeout 20.0 \
        || wait_status="$?"
    stop_recorder
    if [[ "${wait_status}" != "0" && "${wait_status}" != "130" ]]; then
        echo "ERROR: simulation-duration waiter failed with status ${wait_status}" >&2
        exit "${wait_status}"
    fi
else
    echo "Manual mode: press k in teleop, wait at least 1 simulated second, then Ctrl-C here."
    set +e
    wait "${RECORDER_PID}"
    recorder_status="$?"
    set -e
    RECORDER_PID=""
    if [[ "${recorder_status}" != "0" && "${recorder_status}" != "130" ]]; then
        echo "ERROR: rosbag recorder failed with status ${recorder_status}" >&2
        exit "${recorder_status}"
    fi
fi

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "ERROR: bag metadata was not created: ${BAG_DIR}/metadata.yaml" >&2
    exit 4
fi

{
    recorded_lidar_rate_hz="$(
        python3 "${SCRIPT_DIR}/read_sensor_config_bag.py" \
            --bag "${BAG_DIR}" --field lidar_rate_hz
    )"
    recorded_lidar_sample_count="$(
        python3 "${SCRIPT_DIR}/read_sensor_config_bag.py" \
            --bag "${BAG_DIR}" --field lidar_samples
    )"
    printf 'COMPLETED_AT=%q\n' "$(date --iso-8601=seconds)"
    printf 'BAG_METADATA=%q\n' "${BAG_DIR}/metadata.yaml"
    printf 'RECORDER_LOG=%q\n' "${RECORDER_LOG}"
    printf 'RECORDED_LIDAR_RATE_HZ=%q\n' "${recorded_lidar_rate_hz}"
    printf 'RECORDED_LIDAR_SAMPLE_COUNT=%q\n' "${recorded_lidar_sample_count}"
} >>"${SESSION_MANIFEST}"

trap - INT TERM EXIT
echo "CAPTURE_COMPLETE"
echo "BAG_DIR=${BAG_DIR}"
echo "SESSION_MANIFEST=${SESSION_MANIFEST}"
ros2 bag info "${BAG_DIR}"
echo "Next strict check:"
echo "bash ${SCRIPT_DIR}/check_rosbag.sh \"${BAG_DIR}\""
