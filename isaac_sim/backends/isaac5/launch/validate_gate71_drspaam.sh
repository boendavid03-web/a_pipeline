#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
TRAIN_PYTHON="$PROJECT_ROOT/.venvs/train/bin/python"
DRSPAAM_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/2D_lidar_person_detection/dr_spaam"
DRSPAAM_ROS2_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2"
DRSPAAM_NODE="$DRSPAAM_ROS2_ROOT/dr_spaam_ros2/dr_spaam_w_score_ros.py"
CHECKPOINT="$DRSPAAM_ROS2_ROOT/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
MERGER="$PROJECT_ROOT/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/v7_dual_laser_scan_merger.py"
MONITOR="$BACKEND_ROOT/runtime/monitor_drspaam.py"
export ROS_DOMAIN_ID="${ISAAC5_GATE71_ROS_DOMAIN_ID:-55}"
export ISAAC5_GATE6B_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
DRSPAAM_CONF_THRESH="${ISAAC5_GATE71_CONF_THRESH:-0.95}"
if ! awk -v value="$DRSPAAM_CONF_THRESH" 'BEGIN { exit !(value >= 0.0 && value <= 1.0) }'; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=invalid_confidence_threshold value=$DRSPAAM_CONF_THRESH"
    exit 2
fi

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" "$TRAIN_PYTHON" \
    "$DRSPAAM_NODE" "$CHECKPOINT" "$MERGER" "$MONITOR"; do
    [[ -e "$required" ]] || { echo "GATE71_DRSPAAM_RESULT=FAIL reason=missing_input path=$required"; exit 2; }
done

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONPATH="$DRSPAAM_ROOT:$DRSPAAM_ROS2_ROOT:${PYTHONPATH:-}"

MERGER_LOG="$BACKEND_ROOT/generated/gate71_scan_merger.log"
DETECTOR_LOG="$BACKEND_ROOT/generated/gate71_drspaam_detector.log"
CROWD_LOG="$BACKEND_ROOT/generated/gate71_crowd_runtime_580.log"
MONITOR_LOG="$BACKEND_ROOT/generated/gate71_drspaam_monitor.log"
GRAPH_LOG="$BACKEND_ROOT/generated/gate71_ros_graph.log"
: >"$MERGER_LOG"
: >"$DETECTOR_LOG"
: >"$CROWD_LOG"
: >"$MONITOR_LOG"
: >"$GRAPH_LOG"

merger_pid=""
detector_pid=""
crowd_pid=""
monitor_pid=""
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
    stop_group INT "$detector_pid"
    stop_group INT "$merger_pid"
}
trap cleanup EXIT INT TERM

setsid /usr/bin/python3 "$MERGER" --ros-args \
    -p input_scan_01_topic:=/scan_01 \
    -p input_scan_02_topic:=/scan_02 \
    -p output_topic:=/scan_merged \
    -p output_frame:=base_link \
    -p output_samples:=360 \
    -p enable_self_filter:=true \
    >"$MERGER_LOG" 2>&1 &
merger_pid="$!"

setsid "$TRAIN_PYTHON" "$DRSPAAM_NODE" --ros-args \
    -p weight_file:="$CHECKPOINT" \
    -p detector_model:=DR-SPAAM \
    -p conf_thresh:="$DRSPAAM_CONF_THRESH" \
    -p stride:=5 \
    -p panoramic_scan:=true \
    -p reverse_scan:=true \
    -p drow_to_ros:=true \
    -p target_frame:=base_link \
    -p subscriber.scan.topic:=/scan_merged \
    >"$DETECTOR_LOG" 2>&1 &
detector_pid="$!"

nodes_ready=0
ready_nodes=""
for _ in $(seq 1 60); do
    if ! kill -0 "$merger_pid" 2>/dev/null || ! kill -0 "$detector_pid" 2>/dev/null; then
        break
    fi
    nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
    if grep -qx '/v7_dual_laser_scan_merger' <<<"$nodes" \
        && grep -qx '/dr_spaam_ros' <<<"$nodes"; then
        nodes_ready=1
        ready_nodes="$nodes"
        break
    fi
    sleep 0.5
done
if [[ "$nodes_ready" != 1 ]]; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=perception_nodes_not_ready"
    tail -n 80 "$DETECTOR_LOG"
    exit 2
fi

"$SCRIPT_DIR/validate_crowd.sh" --headless --duration 10 --suppress-tracks >"$CROWD_LOG" 2>&1 &
crowd_pid="$!"
crowd_ready=0
for _ in $(seq 1 100); do
    if grep -F 'GATE6B_CROWD_READY=' "$CROWD_LOG" >/dev/null 2>&1; then
        crowd_ready=1
        break
    fi
    if ! kill -0 "$crowd_pid" 2>/dev/null; then
        break
    fi
    sleep 0.25
done
if [[ "$crowd_ready" != 1 ]]; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=crowd_not_ready"
    tail -n 100 "$CROWD_LOG"
    exit 2
fi

{
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "CHECKPOINT_SHA256=$(sha256sum "$CHECKPOINT" | awk '{print $1}')"
    echo "CONFIDENCE_THRESHOLD=$DRSPAAM_CONF_THRESH"
    echo "NODES"
    printf '%s\n' "$ready_nodes" | sort
    echo "/isaac5_gate6_pedestrian"
    echo "TOPICS"
    ros2 topic list --no-daemon | sort
} >"$GRAPH_LOG"

setsid /usr/bin/python3 "$MONITOR" --duration 7 >"$MONITOR_LOG" 2>&1 &
monitor_pid="$!"
monitor_status=0
wait "$monitor_pid" || monitor_status="$?"
monitor_pid=""
crowd_status=0
wait "$crowd_pid" || crowd_status="$?"
crowd_pid=""

if [[ "$monitor_status" != 0 ]]; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=monitor_failed status=$monitor_status"
    cat "$MONITOR_LOG"
    exit 2
fi
if [[ "$crowd_status" != 0 ]]; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=crowd_runtime_failed status=$crowd_status"
    tail -n 100 "$CROWD_LOG"
    exit 2
fi
kill -0 "$detector_pid" 2>/dev/null || { echo "GATE71_DRSPAAM_RESULT=FAIL reason=detector_exited"; exit 2; }
kill -0 "$merger_pid" 2>/dev/null || { echo "GATE71_DRSPAAM_RESULT=FAIL reason=merger_exited"; exit 2; }
grep -F 'GATE6B_CROWD_RESULT={"status": "PASS"' "$CROWD_LOG" >/dev/null
grep -F 'GATE71_DRSPAAM_MONITOR_RESULT={"status": "PASS"' "$MONITOR_LOG" >/dev/null
grep -Fx '/dr_spaam_ros' "$GRAPH_LOG" >/dev/null
if grep -E '/pedestrian_point_tracker|/drl_vo|/semantic_cnn|/s3net' "$GRAPH_LOG" >/dev/null; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=later_gate_node_present"
    exit 2
fi
if grep -Fx '/pedestrian_tracks' "$GRAPH_LOG" >/dev/null; then
    echo "GATE71_DRSPAAM_RESULT=FAIL reason=ground_truth_tracks_not_suppressed"
    exit 2
fi

echo "GATE71_DRSPAAM_RESULT=PASS domain=$ROS_DOMAIN_ID confidence_threshold=$DRSPAAM_CONF_THRESH checkpoint=$CHECKPOINT monitor_log=$MONITOR_LOG"
