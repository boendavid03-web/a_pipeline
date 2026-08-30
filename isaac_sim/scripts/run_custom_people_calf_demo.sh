#!/usr/bin/env bash
# One short Isaac PhysX closed-loop smoke episode with CALF as the sole policy.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
ISAAC_LAUNCHER="$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh"
CALF_PYTHON="$PROJECT_ROOT/.venvs/calf_ros2/bin/python"
CALF_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/LegNav-Sim-master"
CALF_CHECKPOINT="${CALF_CHECKPOINT:-$CALF_ROOT/checkpoints/ppo/ppo_legs_best.msgpack}"
MAP_YAML="${ISAAC_DEMO_MAP_YAML:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_DOMAIN_ID
export ISAAC_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export NAVIGATION_PROJECT_ROOT="$PROJECT_ROOT"
export ISAAC_SCENE=custom
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_ROBOT_PHYSICS="${ISAAC_ROBOT_PHYSICS:-1}"
export ISAAC_LIDAR_MODE=physx
export ISAAC_LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-10}"
export ISAAC_LIDAR_SAMPLE_COUNT="${ISAAC_LIDAR_SAMPLE_COUNT:-2000}"
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off
export ISAAC_PEDESTRIAN_COUNT="${ISAAC_PEDESTRIAN_COUNT:-19}"
export ISAAC_PEDESTRIAN_SEED="${ISAAC_PEDESTRIAN_SEED:-7}"
export ISAAC_PEDESTRIAN_SPEED="${ISAAC_PEDESTRIAN_SPEED:-1.0}"
export ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML="${ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML:-$ROS_WS/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml}"
export ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M:-0.55}"
export ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M:-0.20}"
export ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M="${ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M:-1.0}"
export ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M="${ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M:-0.5}"

for required in \
    /opt/ros/humble/setup.bash \
    "$ROS_WS/install/setup.bash" \
    "$ISAAC_LAUNCHER" \
    "$CALF_PYTHON" \
    "$CALF_CHECKPOINT" \
    "$MAP_YAML"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required CALF demo input is missing: $required" >&2
        exit 1
    fi
done

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID ISAAC_ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

# Never stop unknown processes. Refuse to mix this smoke test with another sim.
conflicts=()
for topic in /scan_01 /scan_02 /odom /cmd_vel /isaac/actuation_state; do
    count="$(
        timeout 6s ros2 topic info "$topic" --no-daemon --spin-time 2.0 2>/dev/null \
            | awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2}' \
            || true
    )"
    if [[ "$count" =~ ^[0-9]+$ ]] && (( count > 0 )); then
        conflicts+=("$topic=$count")
    fi
done
if (( ${#conflicts[@]} > 0 )); then
    echo "ERROR: ROS_DOMAIN_ID=$ROS_DOMAIN_ID is already in use: ${conflicts[*]}" >&2
    exit 4
fi

run_tag="$(date +%Y%m%d_%H%M%S)"
output_dir="${CALF_SMOKE_OUTPUT_DIR:-$PROJECT_ROOT/runs/isaac_custom_calf_demo/$run_tag}"
if ! mkdir -p "$(dirname "$output_dir")" || ! mkdir "$output_dir"; then
    echo "ERROR: refusing to overwrite CALF smoke directory: $output_dir" >&2
    exit 1
fi
calf_trace="$output_dir/calf_inference.jsonl"
trajectory="$output_dir/trajectory.csv"

isaac_pid=""
policy_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ "$policy_pid" =~ ^[0-9]+$ ]]; then
        kill -INT -- "-$policy_pid" 2>/dev/null || true
    fi
    if [[ "$isaac_pid" =~ ^[0-9]+$ ]]; then
        kill -TERM -- "-$isaac_pid" 2>/dev/null || true
    fi
    sleep 2
    if [[ "$policy_pid" =~ ^[0-9]+$ ]]; then
        kill -TERM -- "-$policy_pid" 2>/dev/null || true
        wait "$policy_pid" 2>/dev/null || true
    fi
    if [[ "$isaac_pid" =~ ^[0-9]+$ ]]; then
        kill -KILL -- "-$isaac_pid" 2>/dev/null || true
        wait "$isaac_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting short Isaac CALF smoke: lidar=physx pedestrian_avoidance=off domain=$ROS_DOMAIN_ID"
setsid "$ISAAC_LAUNCHER" --duration "${CALF_SMOKE_ISAAC_DURATION_SEC:-120}" \
    >"$output_dir/isaac.log" 2>&1 &
isaac_pid=$!

ready=0
for _ in $(seq 1 180); do
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        echo "ERROR: Isaac exited before topics became ready; see $output_dir/isaac.log" >&2
        exit 1
    fi
    topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"
    if grep -qx /scan_01 <<<"$topics" \
        && grep -qx /scan_02 <<<"$topics" \
        && grep -qx /odom <<<"$topics" \
        && grep -qx /pedestrian_ground_truth <<<"$topics"; then
        ready=1
        break
    fi
    sleep 1
done
if (( ! ready )); then
    echo "ERROR: Isaac sensor topics were not ready in 180 seconds" >&2
    exit 1
fi

if ! python3 "$SCRIPT_DIR/check_capture_ready.py" \
    --sensor-preflight --verify-lidar-rate --timeout 120.0 \
    >"$output_dir/sensor_preflight.log" 2>&1; then
    echo "ERROR: Isaac sensor preflight failed" >&2
    sed -n '1,220p' "$output_dir/sensor_preflight.log" >&2
    exit 1
fi

echo "Isaac topics ready; starting CALF PPO as the only /cmd_vel publisher"
setsid ros2 launch semantic_nav_gazebo calf_fixed_dual_start_goal_demo.launch.py \
    start_simulator:=false \
    start_rviz:="${CALF_SMOKE_RVIZ:-false}" \
    use_sim_time:=true \
    map_yaml:="$MAP_YAML" \
    robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
    goal_x:="${CALF_SMOKE_GOAL_X:-6.0}" \
    goal_y:="${CALF_SMOKE_GOAL_Y:-4.0}" \
    auto_set_initial_goal:=true \
    enable_goal_picker:=false \
    fixed_test:=false \
    calf_python:="$CALF_PYTHON" \
    calf_checkpoint:="$CALF_CHECKPOINT" \
    calf_trace_path:="$calf_trace" \
    max_linear:="${CALF_MAX_LINEAR:-0.8}" \
    scan_timeout:=0.75 odom_timeout:=0.50 subgoal_timeout:=0.50 \
    lookahead:=1.0 inflate_radius:="${CALF_SMOKE_INFLATE_RADIUS:-0.45}" \
    show_actual_trajectory:=true \
    record_trace:=true trace_path:="$trajectory" trace_timeout_sec:=180.0 \
    evaluate_episode:=false \
    >"$output_dir/calf.log" 2>&1 &
policy_pid=$!

goal_ready=0
for _ in $(seq 1 90); do
    if ! kill -0 "$policy_pid" 2>/dev/null; then
        echo "ERROR: CALF launch exited; see $output_dir/calf.log" >&2
        exit 1
    fi
    publisher_count="$(
        ros2 topic info /cmd_vel 2>/dev/null \
            | awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2}'
    )"
    if [[ "$publisher_count" == "1" ]] \
        && timeout 2s ros2 topic echo /semantic_cnn/local_subgoal --once \
            >"$output_dir/local_subgoal_sample.txt" 2>/dev/null; then
        goal_ready=1
        break
    fi
    sleep 1
done
if (( ! goal_ready )); then
    echo "ERROR: CALF/local-subgoal controller contract not ready" >&2
    exit 1
fi

if ! python3 "$SCRIPT_DIR/check_capture_ready.py" \
    --timeout "${CALF_SMOKE_VERIFY_TIMEOUT_SEC:-60.0}" \
    --verify-motion --minimum-motion-distance "${CALF_SMOKE_MIN_MOTION_M:-0.05}" \
    >"$output_dir/closed_loop_preflight.log" 2>&1; then
    echo "ERROR: CALF closed-loop motion verification failed" >&2
    sed -n '1,260p' "$output_dir/closed_loop_preflight.log" >&2
    exit 1
fi

if [[ ! -s "$calf_trace" ]]; then
    echo "ERROR: CALF inference trace is missing or empty" >&2
    exit 1
fi

"$CALF_PYTHON" - "$calf_trace" <<'PY' | tee "$output_dir/calf_summary.txt"
import json
import math
import statistics
import sys

rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8") if line.strip()]
actions = [row["action"] for row in rows]
freq = [row["inference_frequency_hz"] for row in rows if row["inference_frequency_hz"]]
nonzero = sum(abs(a[0]) > 1e-4 or abs(a[1]) > 1e-4 for a in actions)
print(f"CALF_TRACE_ROWS={len(rows)}")
print(f"CALF_NONZERO_ACTIONS={nonzero}")
print(f"CALF_LINEAR_RANGE=[{min(a[0] for a in actions):.6f},{max(a[0] for a in actions):.6f}]")
print(f"CALF_ANGULAR_RANGE=[{min(a[1] for a in actions):.6f},{max(a[1] for a in actions):.6f}]")
print(f"CALF_INFERENCE_FREQUENCY_MEDIAN_HZ={statistics.median(freq):.3f}" if freq else "CALF_INFERENCE_FREQUENCY_MEDIAN_HZ=unavailable")
print(f"CALF_OBSERVATION_SHAPE={rows[-1]['observation_shape']}")
print(f"CALF_CHECKPOINT={rows[-1]['checkpoint']}")
if not rows or nonzero == 0 or any(not all(math.isfinite(v) for v in a) for a in actions):
    raise SystemExit(1)
PY

echo "ISAAC_CALF_SMOKE_TEST=PASS logs=$output_dir"
