#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ROS_DOMAIN_ID="${ISAAC5_GATE4_ROS_DOMAIN_ID:-51}"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1

set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/workspaces/ros2_ws/install/setup.bash"
set -u

RUNTIME_LOG="$BACKEND_ROOT/generated/gate4_ros_shadow_runtime.log"
TOPICS_LOG="$BACKEND_ROOT/generated/gate4_ros_shadow_topics.log"
CLOCK_LOG="$BACKEND_ROOT/generated/gate4_clock.yaml"
ODOM_LOG="$BACKEND_ROOT/generated/gate4_odom.yaml"
TF_LOG="$BACKEND_ROOT/generated/gate4_tf.yaml"
TF_STATIC_LOG="$BACKEND_ROOT/generated/gate4_tf_static.yaml"
DECISION_LOG="$BACKEND_ROOT/generated/gate4_shadow_decision.yaml"
: >"$RUNTIME_LOG"
: >"$TOPICS_LOG"
: >"$CLOCK_LOG"
: >"$ODOM_LOG"
: >"$TF_LOG"
: >"$TF_STATIC_LOG"
: >"$DECISION_LOG"

isaac_pid=""
probe_pids=()
cleanup() {
    for pid in "${probe_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    if [[ -n "$isaac_pid" ]] && kill -0 "$isaac_pid" 2>/dev/null; then
        kill "$isaac_pid" 2>/dev/null || true
        wait "$isaac_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"$SCRIPT_DIR/run_navigation.sh" \
    --headless --duration 8 --no-lidar --shadow-command \
    >"$RUNTIME_LOG" 2>&1 &
isaac_pid="$!"

ready=0
for _ in $(seq 1 60); do
    if grep -F 'NAVIGATION_RUNTIME_READY=' "$RUNTIME_LOG" >/dev/null 2>&1; then
        ready=1
        break
    fi
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        break
    fi
    sleep 0.5
done
if [[ "$ready" != 1 ]]; then
    echo "GATE4_ROS_SHADOW_RESULT=FAIL reason=runtime_not_ready"
    tail -n 80 "$RUNTIME_LOG"
    exit 2
fi

timeout 12 ros2 topic echo --once /clock >"$CLOCK_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /odom >"$ODOM_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /tf >"$TF_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --qos-durability transient_local --once /tf_static \
    >"$TF_STATIC_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /isaac5/gate4/shadow_actuation_decision \
    >"$DECISION_LOG" 2>&1 & probe_pids+=("$!")

sleep 1
ros2 topic list | sort >"$TOPICS_LOG"
ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

for pid in "${probe_pids[@]}"; do
    wait "$pid"
done
probe_pids=()
wait "$isaac_pid"
isaac_pid=""

required_topics=(
    /clock /cmd_vel /cmd_vel_stamped /odom /tf /tf_static
    /isaac5/gate4/shadow_actuation_decision
)
for topic in "${required_topics[@]}"; do
    grep -Fx "$topic" "$TOPICS_LOG" >/dev/null || {
        echo "GATE4_ROS_SHADOW_RESULT=FAIL reason=missing_topic topic=$topic"
        exit 2
    }
done
grep -F 'NAVIGATION_RUNTIME_RESULT={"status": "PASS"' "$RUNTIME_LOG" >/dev/null
grep -F 'linear:' "$DECISION_LOG" >/dev/null
grep -F 'x: 0.15' "$DECISION_LOG" >/dev/null
grep -F 'gated: true' "$DECISION_LOG" >/dev/null
grep -F -- '- gate4_shadow_only' "$DECISION_LOG" >/dev/null
grep -F 'child_frame_id: base_link' "$ODOM_LOG" >/dev/null
grep -F 'child_frame_id: base_link' "$TF_LOG" >/dev/null
grep -F 'child_frame_id: base_scan' "$TF_STATIC_LOG" >/dev/null

echo "GATE4_ROS_SHADOW_RESULT=PASS domain=$ROS_DOMAIN_ID runtime_log=$RUNTIME_LOG decision_log=$DECISION_LOG"
