#!/usr/bin/env bash
# Strict read-only checks for one Isaac 6.0.1 manual-teleop bag.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
BAG_ROOT="${ISAAC_BAG_ROOT:-${PROJECT_ROOT}/isaac_sim/bags}"
BAG_DIR="${1:-}"
REQUIRE_LIDAR_INTENSITY="${ISAAC_REQUIRE_LIDAR_INTENSITY:-1}"
REQUIRE_REALTIME_LIDAR="${ISAAC_REQUIRE_REALTIME_LIDAR:-0}"
LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-}"
LIDAR_SAMPLE_COUNT="${ISAAC_LIDAR_SAMPLE_COUNT:-}"

if [[ "${REQUIRE_LIDAR_INTENSITY}" != "0" && "${REQUIRE_LIDAR_INTENSITY}" != "1" ]]; then
    echo "ERROR: ISAAC_REQUIRE_LIDAR_INTENSITY must be 0 or 1" >&2
    exit 2
fi
if [[ "${REQUIRE_REALTIME_LIDAR}" != "0" && "${REQUIRE_REALTIME_LIDAR}" != "1" ]]; then
    echo "ERROR: ISAAC_REQUIRE_REALTIME_LIDAR must be 0 or 1" >&2
    exit 2
fi

if [[ -z "${BAG_DIR}" ]]; then
    BAG_DIR="$(find "${BAG_ROOT}" -mindepth 1 -maxdepth 1 -type d \
        -name '*_isaac_6_teleop' -printf '%T@ %p\n' 2>/dev/null \
        | sort -nr | awk 'NR==1 {$1=""; sub(/^ /, ""); print}')"
fi
if [[ -z "${BAG_DIR}" || ! -d "${BAG_DIR}" ]]; then
    echo "ERROR: bag directory not found: ${BAG_DIR:-<latest>}" >&2
    exit 2
fi
if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "ERROR: metadata.yaml is missing: ${BAG_DIR}" >&2
    exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="1"

if [[ -z "${LIDAR_RATE_HZ}" ]]; then
    if LIDAR_RATE_HZ="$(
        python3 "${SCRIPT_DIR}/read_sensor_config_bag.py" \
            --bag "${BAG_DIR}" --field lidar_rate_hz 2>/dev/null
    )"; then
        echo "Using LiDAR rate from bag sensor config: ${LIDAR_RATE_HZ} Hz"
    else
        LIDAR_RATE_HZ=10
        echo "INFO: legacy bag has no sensor config; using 10 Hz default"
    fi
fi
if [[ ! "${LIDAR_RATE_HZ}" =~ ^[0-9]+$ ]] \
    || (( LIDAR_RATE_HZ < 1 || LIDAR_RATE_HZ > 30 )); then
    echo "ERROR: ISAAC_LIDAR_RATE_HZ must be an integer from 1 through 30" >&2
    exit 2
fi
if [[ -z "${LIDAR_SAMPLE_COUNT}" ]]; then
    if LIDAR_SAMPLE_COUNT="$(
        python3 "${SCRIPT_DIR}/read_sensor_config_bag.py" \
            --bag "${BAG_DIR}" --field lidar_samples 2>/dev/null
    )"; then
        echo "Using LiDAR sample count from bag sensor config: ${LIDAR_SAMPLE_COUNT}"
    else
        LIDAR_SAMPLE_COUNT=360
        echo "INFO: legacy bag has no sensor config; using 360-sample default"
    fi
fi
if [[ ! "${LIDAR_SAMPLE_COUNT}" =~ ^[0-9]+$ ]] \
    || (( LIDAR_SAMPLE_COUNT < 90 || LIDAR_SAMPLE_COUNT > 4096 )); then
    echo "ERROR: ISAAC_LIDAR_SAMPLE_COUNT must be an integer from 90 through 4096" >&2
    exit 2
fi

echo "===== ros2 bag info ====="
ros2 bag info "${BAG_DIR}"

echo
echo "===== metadata topic/type/count contract ====="
python3 - "${BAG_DIR}/metadata.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

expected = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/scan_01": "sensor_msgs/msg/LaserScan",
    "/scan_02": "sensor_msgs/msg/LaserScan",
    "/scan_merged": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_stamped": "geometry_msgs/msg/TwistStamped",
    "/pedestrian_ground_truth": "semantic_nav_gazebo/msg/PedestrianStateArray",
    "/data_collection/episode_event": "std_msgs/msg/String",
}
metadata = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
entries = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
actual = {
    item["topic_metadata"]["name"]: (
        item["topic_metadata"]["type"], int(item["message_count"])
    )
    for item in entries
}
if "/data_collection/sensor_config" in actual:
    expected["/data_collection/sensor_config"] = "std_msgs/msg/String"
errors = []
for topic, expected_type in expected.items():
    actual_type, count = actual.get(topic, (None, 0))
    if actual_type != expected_type:
        errors.append(f"{topic}: type={actual_type!r}, expected={expected_type!r}")
    if count <= 0:
        errors.append(f"{topic}: message_count={count}")
    if actual_type == expected_type and count > 0:
        print(f"PASS {topic} {actual_type} messages={count}")
if errors:
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(1)
PY

echo
echo "===== dual-LiDAR geometry/rate checks ====="
LIDAR_CHECK_ARGS=(
    bag --bag "${BAG_DIR}" --samples "${LIDAR_SAMPLE_COUNT}" --rate "${LIDAR_RATE_HZ}"
    --range-min 0.5 --range-max 50.0
)
if [[ "${REQUIRE_REALTIME_LIDAR}" == "1" ]]; then
    LIDAR_CHECK_ARGS+=(--require-wall-rate)
fi
python3 "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py" \
    "${LIDAR_CHECK_ARGS[@]}"

if [[ "${REQUIRE_LIDAR_INTENSITY}" == "1" ]]; then
    echo
    echo "===== native RTX LiDAR intensity checks ====="
    python3 "${SCRIPT_DIR}/check_lidar_intensity_bag.py" --bag "${BAG_DIR}"
else
    echo
    echo "INFO: LiDAR intensity validation disabled by ISAAC_REQUIRE_LIDAR_INTENSITY=0"
fi

echo
echo "===== control timestamp/alignment checks ====="
python3 "${ROS_WS}/tools/check_cmd_vel_stamped_bag.py" --bag "${BAG_DIR}"

echo
echo "===== teleoperation motion/stop checks ====="
python3 "${SCRIPT_DIR}/check_teleop_commands.py" --bag "${BAG_DIR}"

echo
echo "===== manual episode start/end checks ====="
python3 "${SCRIPT_DIR}/check_manual_episode_events.py" --bag "${BAG_DIR}"

echo
echo "===== pedestrian ground-truth checks ====="
python3 "${ROS_WS}/tools/check_pedestrian_ground_truth_bag.py" \
    --bag "${BAG_DIR}" --min-duration-sec 0

echo
echo "ISAAC_TELEOP_ROSBAG_CHECK=PASS"
echo "BAG_DIR=${BAG_DIR}"
