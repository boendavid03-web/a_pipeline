#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
CMD_TOPIC="${ISAAC_TELEOP_CMD_TOPIC:-/cmd_vel}"
PUBLISH_RATE="${ISAAC_TELEOP_PUBLISH_RATE:-20.0}"
LINEAR_SPEED="${ISAAC_TELEOP_LINEAR_SPEED:-0.5}"
ANGULAR_SPEED="${ISAAC_TELEOP_ANGULAR_SPEED:-1.0}"
ISAAC_ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"

if [[ ! -t 0 ]]; then
    echo "ERROR: Isaac teleop must run in an interactive terminal." >&2
    exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: ROS 2 Humble setup was not found: /opt/ros/humble/setup.bash" >&2
    exit 1
fi
if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
    echo "ERROR: ROS 2 workspace is not built: ${ROS_WS}/install/setup.bash" >&2
    echo "Run: bash environment/03_build_ros2_workspace.sh" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"
set -u

export ROS_DISTRO="humble"
# /opt/ros/humble/setup.bash sets ROS_LOCALHOST_ONLY=0.  Override it after
# sourcing so this terminal always matches the Isaac localhost relay.  Do not
# inherit generic ROS/RMW variables from an unrelated navigation terminal.
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="1"

# Bypass the long-lived ros2 daemon: its cached DDS context may belong to a
# different RMW/domain/localhost setting and falsely report zero endpoints.
topic_info="$(ros2 topic info "${CMD_TOPIC}" --no-daemon --spin-time 3.0 2>/dev/null || true)"
publisher_count="$(awk '/^Publisher count:/ {print $3}' <<<"${topic_info}")"
subscriber_count="$(awk '/^Subscription count:/ {print $3}' <<<"${topic_info}")"
if [[ -n "${publisher_count}" && "${publisher_count}" != "0" ]]; then
    echo "ERROR: ${CMD_TOPIC} already has ${publisher_count} publisher(s)." >&2
    echo "Stop other teleop or navigation controllers before starting this one." >&2
    exit 3
fi
if [[ -z "${subscriber_count}" || "${subscriber_count}" == "0" ]]; then
    echo "ERROR: no subscriber is listening on ${CMD_TOPIC}." >&2
    echo "Start Isaac Sim first and wait for WAREHOUSE_PEOPLE_ROBOT_READY= (6.0) or NAVIGATION_RUNTIME_READY= (legacy)." >&2
    exit 4
fi

echo "Isaac Sim Mecanum keyboard teleop"
echo "  u/i/o     forward-left turn / forward / forward-right turn"
echo "  j/l       turn left / turn right"
echo "  m/,/.     backward-left turn / backward / backward-right turn"
echo "  U/I/O     forward-left strafe / forward / forward-right strafe"
echo "  J/L       strafe left / strafe right"
echo "  M/</>     backward-left strafe / backward / backward-right strafe"
echo "  t/b       positive / negative linear z (ignored by the planar base)"
echo "  k/space   stop"
echo "  q/z       increase/decrease linear and angular speed"
echo "  w/x       increase/decrease linear speed"
echo "  e/c       increase/decrease angular speed"
echo "  Ctrl-C    stop and exit"
echo "Publishing ${CMD_TOPIC} at ${PUBLISH_RATE} Hz; linear=${LINEAR_SPEED} m/s, angular=${ANGULAR_SPEED} rad/s"
echo "ROS: RMW=${RMW_IMPLEMENTATION}, domain=${ROS_DOMAIN_ID}, localhost=${ROS_LOCALHOST_ONLY}"

exec ros2 run semantic_nav_gazebo continuous_teleop.py \
    --ros-args \
    -p cmd_topic:="${CMD_TOPIC}" \
    -p publish_rate:="${PUBLISH_RATE}" \
    -p speed:="${LINEAR_SPEED}" \
    -p turn:="${ANGULAR_SPEED}"
