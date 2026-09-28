#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
TRAIN_PYTHON="$PROJECT_ROOT/.venvs/train/bin/python"
RUNTIME_ROOT="$PROJECT_ROOT/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts"
CHECKPOINT_ROOT="$PROJECT_ROOT/sim_to_real/robot/comparison_models/checkpoints"
ADAPTER="$RUNTIME_ROOT/fixed_dual_scan_adapter.py"
GOAL_BRIDGE="$RUNTIME_ROOT/fixed_global_goal_bridge.py"
S3NET_NODE="$RUNTIME_ROOT/s3net_fixed_dual_inference_node.py"
SEMANTIC_NODE="$RUNTIME_ROOT/semantic_cnn_fixed_dual_inference_node.py"
FAULT_INJECTOR="$BACKEND_ROOT/runtime/gate74_scan_fault_injector.py"
MONITOR="$BACKEND_ROOT/runtime/monitor_freshness.py"
S3NET_MODEL="$CHECKPOINT_ROOT/s3net/s3net_native_stats_best_dev.pth"
S3NET_CODE="$CHECKPOINT_ROOT/s3net"
S3NET_STATS="$CHECKPOINT_ROOT/s3net/s3net_native_lidar_train_stats.json"
SEMANTIC_MODEL="$CHECKPOINT_ROOT/semantic_cnn/semantic_cnn_native_cmd_best_dev.pth"
SEMANTIC_CODE="$CHECKPOINT_ROOT/semantic_cnn"
export ROS_DOMAIN_ID="${ISAAC5_GATE74_ROS_DOMAIN_ID:-58}"
export ISAAC5_GATE6B_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" "$TRAIN_PYTHON" \
    "$ADAPTER" "$GOAL_BRIDGE" "$S3NET_NODE" "$SEMANTIC_NODE" \
    "$FAULT_INJECTOR" "$MONITOR" "$S3NET_MODEL" "$S3NET_CODE/model.py" \
    "$S3NET_STATS" "$SEMANTIC_MODEL" "$SEMANTIC_CODE/model.py"; do
    [[ -e "$required" ]] || { echo "GATE74_FRESHNESS_RESULT=FAIL reason=missing_input path=$required"; exit 2; }
done

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

ADAPTER_LOG="$BACKEND_ROOT/generated/gate74_scan_adapter.log"
GOAL_LOG="$BACKEND_ROOT/generated/gate74_goal_bridge.log"
FAULT_LOG="$BACKEND_ROOT/generated/gate74_fault_injector.log"
S3NET_LOG="$BACKEND_ROOT/generated/gate74_s3net.log"
SEMANTIC_LOG="$BACKEND_ROOT/generated/gate74_semantic_cnn.log"
CROWD_LOG="$BACKEND_ROOT/generated/gate74_crowd_runtime_580.log"
MONITOR_LOG="$BACKEND_ROOT/generated/gate74_freshness_monitor.log"
GRAPH_LOG="$BACKEND_ROOT/generated/gate74_ros_graph.log"
for log in "$ADAPTER_LOG" "$GOAL_LOG" "$FAULT_LOG" "$S3NET_LOG" "$SEMANTIC_LOG" "$CROWD_LOG" "$MONITOR_LOG" "$GRAPH_LOG"; do
    : >"$log"
done

adapter_pid="" goal_pid="" fault_pid="" s3net_pid="" semantic_pid="" crowd_pid="" monitor_pid=""
stop_group() {
    local signal="$1" pid="$2"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" -- "-$pid" 2>/dev/null || kill "-$signal" "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    stop_group TERM "$monitor_pid"
    stop_group TERM "$crowd_pid"
    stop_group INT "$semantic_pid"
    stop_group INT "$s3net_pid"
    stop_group INT "$fault_pid"
    stop_group INT "$goal_pid"
    stop_group INT "$adapter_pid"
}
trap cleanup EXIT INT TERM

setsid /usr/bin/python3 "$ADAPTER" --ros-args \
    -p use_sim_time:=true -p input_scan_01_topic:=/scan_01 -p input_scan_02_topic:=/scan_02 \
    -p output_scan_01_topic:=/sim_to_real/aligned/scan_01 \
    -p output_scan_02_topic:=/sim_to_real/aligned/scan_02 \
    -p output_beams:=2000 -p expected_input_beams:=2000 \
    -p range_min:=0.1 -p range_max:=8.0 -p max_scan_age:=0.5 >"$ADAPTER_LOG" 2>&1 &
adapter_pid="$!"
setsid /usr/bin/python3 "$GOAL_BRIDGE" --ros-args \
    -p use_sim_time:=true -p global_frame:=odom -p world_frame:=odom \
    -p base_frame:=base_link -p goal_x:=6.0 -p goal_y:=2.0 \
    -p publish_rate:=10.0 -p pose_topic:=/odom >"$GOAL_LOG" 2>&1 &
goal_pid="$!"
setsid /usr/bin/python3 "$FAULT_INJECTOR" \
    --normal-until 4.0 --stale-until 7.5 --stale-offset 1.0 --ros-args \
    -p use_sim_time:=true >"$FAULT_LOG" 2>&1 &
fault_pid="$!"
setsid "$TRAIN_PYTHON" "$S3NET_NODE" --ros-args \
    -p use_sim_time:=true -p model:="$S3NET_MODEL" -p model_code:="$S3NET_CODE" \
    -p stats_json:="$S3NET_STATS" -p scan_01_topic:=/isaac5/gate74/scan_01 \
    -p scan_02_topic:=/isaac5/gate74/scan_02 -p labels_topic:=/s3net/labels \
    -p base_frame:=base_link -p enforce_message_layout:=true -p device:=cuda \
    -p visualization_rate_hz:=5.0 >"$S3NET_LOG" 2>&1 &
s3net_pid="$!"
setsid "$TRAIN_PYTHON" "$SEMANTIC_NODE" --ros-args \
    -p use_sim_time:=true -p model:="$SEMANTIC_MODEL" -p model_code:="$SEMANTIC_CODE" \
    -p scan_01_topic:=/isaac5/gate74/scan_01 -p scan_02_topic:=/isaac5/gate74/scan_02 \
    -p use_online_s3net:=true -p s3net_labels_topic:=/s3net/labels \
    -p odom_topic:=/odom -p local_subgoal_topic:=/semantic_cnn/local_subgoal \
    -p final_goal_topic:=/semantic_cnn/final_goal \
    -p cmd_vel_topic:=/isaac5/gate74/semantic_cmd_shadow \
    -p actuation_decision_topic:=/semantic_cnn/actuation_decision \
    -p base_frame:=base_link -p device:=cuda -p visualize:=false \
    -p publish_debug_images:=false -p sync_slop:=0.05 \
    -p scan_timeout:=0.8 -p odom_timeout:=0.8 -p subgoal_timeout:=0.8 \
    -p command_timeout:=0.8 >"$SEMANTIC_LOG" 2>&1 &
semantic_pid="$!"

nodes_ready=0
ready_nodes=""
for _ in $(seq 1 120); do
    if ! kill -0 "$adapter_pid" 2>/dev/null || ! kill -0 "$goal_pid" 2>/dev/null \
        || ! kill -0 "$fault_pid" 2>/dev/null || ! kill -0 "$s3net_pid" 2>/dev/null \
        || ! kill -0 "$semantic_pid" 2>/dev/null; then break; fi
    nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
    if grep -qx '/fixed_dual_scan_adapter' <<<"$nodes" \
        && grep -qx '/fixed_global_goal_bridge' <<<"$nodes" \
        && grep -qx '/isaac5_gate74_scan_fault_injector' <<<"$nodes" \
        && grep -qx '/s3net_fixed_dual_inference' <<<"$nodes" \
        && grep -qx '/semantic_cnn_fixed_dual_inference' <<<"$nodes"; then
        nodes_ready=1; ready_nodes="$nodes"; break
    fi
    sleep 0.5
done
if [[ "$nodes_ready" != 1 ]]; then
    echo "GATE74_FRESHNESS_RESULT=FAIL reason=nodes_not_ready"
    tail -n 100 "$FAULT_LOG"; tail -n 100 "$S3NET_LOG"; tail -n 100 "$SEMANTIC_LOG"
    exit 2
fi

"$SCRIPT_DIR/validate_crowd.sh" --headless --duration 15 --suppress-tracks >"$CROWD_LOG" 2>&1 &
crowd_pid="$!"
crowd_ready=0
for _ in $(seq 1 120); do
    if grep -F 'GATE6B_CROWD_READY=' "$CROWD_LOG" >/dev/null 2>&1; then crowd_ready=1; break; fi
    if ! kill -0 "$crowd_pid" 2>/dev/null; then break; fi
    sleep 0.25
done
if [[ "$crowd_ready" != 1 ]]; then
    echo "GATE74_FRESHNESS_RESULT=FAIL reason=crowd_not_ready"
    tail -n 120 "$CROWD_LOG"; exit 2
fi

{
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "FAULT_SEQUENCE=normal_until_4.0s,stale_minus_1.0s_until_7.5s,recovery"
    echo "S3NET_SHA256=$(sha256sum "$S3NET_MODEL" | awk '{print $1}')"
    echo "SEMANTIC_SHA256=$(sha256sum "$SEMANTIC_MODEL" | awk '{print $1}')"
    echo "NODES"; printf '%s\n' "$ready_nodes" | sort; echo "/isaac5_gate6_pedestrian"
} >"$GRAPH_LOG"

setsid /usr/bin/python3 "$MONITOR" --duration 11.5 >"$MONITOR_LOG" 2>&1 &
monitor_pid="$!"
monitor_status=0; wait "$monitor_pid" || monitor_status="$?"; monitor_pid=""
crowd_status=0; wait "$crowd_pid" || crowd_status="$?"; crowd_pid=""
if [[ "$monitor_status" != 0 ]]; then
    echo "GATE74_FRESHNESS_RESULT=FAIL reason=monitor_failed status=$monitor_status"
    cat "$MONITOR_LOG"; tail -n 120 "$SEMANTIC_LOG"; exit 2
fi
if [[ "$crowd_status" != 0 ]]; then
    echo "GATE74_FRESHNESS_RESULT=FAIL reason=crowd_failed status=$crowd_status"
    tail -n 120 "$CROWD_LOG"; exit 2
fi
for pid in "$adapter_pid" "$goal_pid" "$fault_pid" "$s3net_pid" "$semantic_pid"; do
    kill -0 "$pid" 2>/dev/null || { echo "GATE74_FRESHNESS_RESULT=FAIL reason=process_exited pid=$pid"; exit 2; }
done
grep -F 'GATE6B_CROWD_RESULT={"status": "PASS"' "$CROWD_LOG" >/dev/null
grep -F 'GATE74_FRESHNESS_MONITOR_RESULT={"status": "PASS"' "$MONITOR_LOG" >/dev/null
if grep -E '/dr_spaam|/pedestrian_point_tracker|/drl_vo' "$GRAPH_LOG" >/dev/null; then
    echo "GATE74_FRESHNESS_RESULT=FAIL reason=other_policy_node_present"; exit 2
fi
echo "GATE74_FRESHNESS_RESULT=PASS domain=$ROS_DOMAIN_ID monitor_log=$MONITOR_LOG"
