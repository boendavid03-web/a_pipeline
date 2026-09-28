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
TRACKER="$ROS_WS/src/semantic_nav_gazebo/scripts/pedestrian_point_tracker.py"
MONITOR="$BACKEND_ROOT/runtime/monitor_tracking.py"
export ROS_DOMAIN_ID="${ISAAC5_GATE72_ROS_DOMAIN_ID:-56}"
export ISAAC5_GATE6B_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LOG_TAG="${ISAAC5_GATE72_LOG_TAG:-}"
if [[ -n "$LOG_TAG" && ! "$LOG_TAG" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=invalid_log_tag value=$LOG_TAG"
    exit 2
fi
LOG_SUFFIX="${LOG_TAG:+_$LOG_TAG}"
CROWD_DURATION="${ISAAC5_GATE72_CROWD_DURATION:-10}"
MONITOR_DURATION="${ISAAC5_GATE72_MONITOR_DURATION:-7}"
for value in "$CROWD_DURATION" "$MONITOR_DURATION"; do
    if ! awk -v value="$value" 'BEGIN { exit !(value > 0.0) }'; then
        echo "GATE72_TRACKING_RESULT=FAIL reason=invalid_duration value=$value"
        exit 2
    fi
done

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" "$TRAIN_PYTHON" \
    "$DRSPAAM_NODE" "$CHECKPOINT" "$MERGER" "$TRACKER" "$MONITOR"; do
    [[ -e "$required" ]] || { echo "GATE72_TRACKING_RESULT=FAIL reason=missing_input path=$required"; exit 2; }
done

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONPATH="$DRSPAAM_ROOT:$DRSPAAM_ROS2_ROOT:${PYTHONPATH:-}"

MERGER_LOG="$BACKEND_ROOT/generated/gate72_scan_merger${LOG_SUFFIX}.log"
DETECTOR_LOG="$BACKEND_ROOT/generated/gate72_drspaam_detector${LOG_SUFFIX}.log"
TRACKER_LOG="$BACKEND_ROOT/generated/gate72_tracker${LOG_SUFFIX}.log"
CROWD_LOG="$BACKEND_ROOT/generated/gate72_crowd_runtime_580${LOG_SUFFIX}.log"
MONITOR_LOG="$BACKEND_ROOT/generated/gate72_tracking_monitor${LOG_SUFFIX}.log"
GRAPH_LOG="$BACKEND_ROOT/generated/gate72_ros_graph${LOG_SUFFIX}.log"
for log in "$MERGER_LOG" "$DETECTOR_LOG" "$TRACKER_LOG" "$CROWD_LOG" "$MONITOR_LOG" "$GRAPH_LOG"; do
    : >"$log"
done

merger_pid="" detector_pid="" tracker_pid="" crowd_pid="" monitor_pid=""
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
    stop_group INT "$tracker_pid"
    stop_group INT "$detector_pid"
    stop_group INT "$merger_pid"
}
trap cleanup EXIT INT TERM

setsid /usr/bin/python3 "$MERGER" --ros-args \
    -p input_scan_01_topic:=/scan_01 -p input_scan_02_topic:=/scan_02 \
    -p output_topic:=/scan_merged -p output_frame:=base_link \
    -p output_samples:=360 -p enable_self_filter:=true >"$MERGER_LOG" 2>&1 &
merger_pid="$!"
setsid "$TRAIN_PYTHON" "$DRSPAAM_NODE" --ros-args \
    -p weight_file:="$CHECKPOINT" -p detector_model:=DR-SPAAM \
    -p conf_thresh:=0.20 -p stride:=5 -p panoramic_scan:=true \
    -p reverse_scan:=true -p drow_to_ros:=true -p target_frame:=base_link \
    -p subscriber.scan.topic:=/scan_merged >"$DETECTOR_LOG" 2>&1 &
detector_pid="$!"
setsid /usr/bin/python3 "$TRACKER" --ros-args \
    -p use_sim_time:=true -p tracking_frame:=odom \
    -p input_topic:=/dr_spaam_detections_scored -p output_topic:=/pedestrian_tracks \
    -p association_threshold:=0.8 -p min_hits:=3 -p max_age:=8 \
    -p max_coast_time:=0.75 -p acceleration_sigma:=2.0 \
    -p measurement_sigma:=0.10 -p max_prediction_dt:=0.50 \
    -p measurement_history_size:=8 -p velocity_fit_min_samples:=3 \
    -p velocity_fit_min_span:=0.15 >"$TRACKER_LOG" 2>&1 &
tracker_pid="$!"

nodes_ready=0
ready_nodes=""
for _ in $(seq 1 60); do
    if ! kill -0 "$merger_pid" 2>/dev/null \
        || ! kill -0 "$detector_pid" 2>/dev/null \
        || ! kill -0 "$tracker_pid" 2>/dev/null; then
        break
    fi
    nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
    if grep -qx '/v7_dual_laser_scan_merger' <<<"$nodes" \
        && grep -qx '/dr_spaam_ros' <<<"$nodes" \
        && grep -qx '/pedestrian_point_tracker' <<<"$nodes"; then
        nodes_ready=1
        ready_nodes="$nodes"
        break
    fi
    sleep 0.5
done
if [[ "$nodes_ready" != 1 ]]; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=perception_nodes_not_ready"
    tail -n 80 "$TRACKER_LOG"
    exit 2
fi

"$SCRIPT_DIR/validate_crowd.sh" --headless --duration "$CROWD_DURATION" --suppress-tracks >"$CROWD_LOG" 2>&1 &
crowd_pid="$!"
crowd_ready=0
for _ in $(seq 1 100); do
    if grep -F 'GATE6B_CROWD_READY=' "$CROWD_LOG" >/dev/null 2>&1; then
        crowd_ready=1
        break
    fi
    if ! kill -0 "$crowd_pid" 2>/dev/null; then break; fi
    sleep 0.25
done
if [[ "$crowd_ready" != 1 ]]; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=crowd_not_ready"
    tail -n 100 "$CROWD_LOG"
    exit 2
fi

{
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "CHECKPOINT_SHA256=$(sha256sum "$CHECKPOINT" | awk '{print $1}')"
    echo "DETECTOR_CONFIDENCE_THRESHOLD=0.20"
    echo "NODES"
    printf '%s\n' "$ready_nodes" | sort
    echo "/isaac5_gate6_pedestrian"
} >"$GRAPH_LOG"

setsid /usr/bin/python3 "$MONITOR" --duration "$MONITOR_DURATION" >"$MONITOR_LOG" 2>&1 &
monitor_pid="$!"
monitor_status=0
wait "$monitor_pid" || monitor_status="$?"
monitor_pid=""
crowd_status=0
wait "$crowd_pid" || crowd_status="$?"
crowd_pid=""
if [[ "$monitor_status" != 0 ]]; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=monitor_failed status=$monitor_status"
    cat "$MONITOR_LOG"
    exit 2
fi
if [[ "$crowd_status" != 0 ]]; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=crowd_runtime_failed status=$crowd_status"
    tail -n 100 "$CROWD_LOG"
    exit 2
fi
for pid in "$merger_pid" "$detector_pid" "$tracker_pid"; do
    kill -0 "$pid" 2>/dev/null || { echo "GATE72_TRACKING_RESULT=FAIL reason=perception_process_exited pid=$pid"; exit 2; }
done
grep -F 'GATE6B_CROWD_RESULT={"status": "PASS"' "$CROWD_LOG" >/dev/null
grep -F 'GATE72_TRACKING_MONITOR_RESULT={"status": "PASS"' "$MONITOR_LOG" >/dev/null
if grep -E '/drl_vo|/semantic_cnn|/s3net' "$GRAPH_LOG" >/dev/null; then
    echo "GATE72_TRACKING_RESULT=FAIL reason=later_gate_node_present"
    exit 2
fi

echo "GATE72_TRACKING_RESULT=PASS domain=$ROS_DOMAIN_ID monitor_log=$MONITOR_LOG tracker_log=$TRACKER_LOG"
