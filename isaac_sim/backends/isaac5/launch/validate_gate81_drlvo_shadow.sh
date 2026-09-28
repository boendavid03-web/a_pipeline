#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
COMPARISON_ROOT="$PROJECT_ROOT/sim_to_real/robot/comparison_models"
COMPARISON_WS="$COMPARISON_ROOT/ros2_ws"
TRAIN_PYTHON="$PROJECT_ROOT/.venvs/train/bin/python"
RUNTIME_ROOT="$COMPARISON_WS/src/semantic_nav_runtime/scripts"
DRSPAAM_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/2D_lidar_person_detection/dr_spaam"
DRSPAAM_ROS2_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2"
DRSPAAM_NODE="$DRSPAAM_ROS2_ROOT/dr_spaam_ros2/dr_spaam_w_score_ros.py"
DRSPAAM_MODEL="$DRSPAAM_ROS2_ROOT/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
MERGER="$RUNTIME_ROOT/v7_dual_laser_scan_merger.py"
TRACKER="$RUNTIME_ROOT/pedestrian_point_tracker.py"
GOAL_BRIDGE="$RUNTIME_ROOT/fixed_global_goal_bridge.py"
DRLVO_NODE="$RUNTIME_ROOT/drl_vo_fixed_dual_inference_node.py"
DRLVO_MODEL="$COMPARISON_ROOT/checkpoints/drl_vo/base_bc_best.pt"
MODE="baseline"
if [[ "${1:-}" == "--track-freshness" ]]; then
    MODE="track_freshness"
    shift
elif [[ "${1:-}" == "--front-veto" ]]; then
    MODE="front_veto"
    shift
elif [[ "${1:-}" == "--closed-loop-smoke" ]]; then
    MODE="closed_loop_smoke"
    shift
elif [[ "${1:-}" == "--fixed-goal" ]]; then
    MODE="fixed_goal"
    shift
elif [[ "${1:-}" == "--fixed-goal-gui" ]]; then
    MODE="fixed_goal_gui"
    shift
fi
[[ "$#" == 0 ]] || { echo "usage: $0 [--track-freshness|--front-veto|--closed-loop-smoke|--fixed-goal|--fixed-goal-gui]"; exit 2; }
if [[ "$MODE" == "track_freshness" ]]; then
    GATE_ID="82"
    DEFAULT_DOMAIN="60"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks_raw"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate82/drl_vo_cmd_shadow"
    MONITOR="$BACKEND_ROOT/runtime/monitor_drlvo_freshness.py"
    RELAY="$BACKEND_ROOT/runtime/gate82_track_dropout_relay.py"
    MONITOR_MARKER="GATE82_DRLVO_FRESHNESS_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="14"
    CROWD_DURATION="19"
    DRLVO_SCAN_01_TOPIC="/scan_01"
    DRLVO_SCAN_02_TOPIC="/scan_02"
elif [[ "$MODE" == "front_veto" ]]; then
    GATE_ID="83"
    DEFAULT_DOMAIN="61"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate83/drl_vo_cmd_shadow"
    MONITOR="$BACKEND_ROOT/runtime/monitor_drlvo_front_veto.py"
    RELAY="$BACKEND_ROOT/runtime/gate83_front_obstacle_relay.py"
    MONITOR_MARKER="GATE83_DRLVO_FRONT_VETO_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="14"
    CROWD_DURATION="19"
    DRLVO_SCAN_01_TOPIC="/isaac5/gate83/scan_01"
    DRLVO_SCAN_02_TOPIC="/isaac5/gate83/scan_02"
elif [[ "$MODE" == "closed_loop_smoke" ]]; then
    GATE_ID="9"
    DEFAULT_DOMAIN="63"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate9/cmd_vel"
    MONITOR="$BACKEND_ROOT/runtime/monitor_gate9_closed_loop.py"
    RELAY=""
    MONITOR_MARKER="GATE9_CLOSED_LOOP_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="5.5"
    CROWD_DURATION="7"
    DRLVO_SCAN_01_TOPIC="/scan_01"
    DRLVO_SCAN_02_TOPIC="/scan_02"
    GOAL_X="6.0"
    GOAL_Y="2.0"
elif [[ "$MODE" == "fixed_goal" ]]; then
    GATE_ID="9f"
    DEFAULT_DOMAIN="64"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate9/fixed_goal_cmd_vel"
    MONITOR="$BACKEND_ROOT/runtime/monitor_gate9_fixed_goal.py"
    RELAY=""
    MONITOR_MARKER="GATE9_FIXED_GOAL_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="15"
    CROWD_DURATION="16.5"
    DRLVO_SCAN_01_TOPIC="/scan_01"
    DRLVO_SCAN_02_TOPIC="/scan_02"
    GOAL_X="3.5"
    GOAL_Y="2.0"
elif [[ "$MODE" == "fixed_goal_gui" ]]; then
    GATE_ID="9g"
    DEFAULT_DOMAIN="65"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate9/gui_fixed_goal_cmd_vel"
    MONITOR="$BACKEND_ROOT/runtime/monitor_gate9_fixed_goal.py"
    RELAY=""
    MONITOR_MARKER="GATE9_FIXED_GOAL_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="15"
    CROWD_DURATION="16.5"
    DRLVO_SCAN_01_TOPIC="/scan_01"
    DRLVO_SCAN_02_TOPIC="/scan_02"
    GOAL_X="3.5"
    GOAL_Y="2.0"
else
    GATE_ID="81"
    DEFAULT_DOMAIN="59"
    TRACKER_OUTPUT_TOPIC="/pedestrian_tracks"
    DRLVO_TRACK_TOPIC="/pedestrian_tracks"
    SHADOW_TOPIC="/isaac5/gate81/drl_vo_cmd_shadow"
    MONITOR="$BACKEND_ROOT/runtime/monitor_drlvo_shadow.py"
    RELAY=""
    MONITOR_MARKER="GATE81_DRLVO_MONITOR_RESULT={\"status\": \"PASS\""
    MONITOR_DURATION="10"
    CROWD_DURATION="15"
    DRLVO_SCAN_01_TOPIC="/scan_01"
    DRLVO_SCAN_02_TOPIC="/scan_02"
    GOAL_X="6.0"
    GOAL_Y="2.0"
fi
: "${GOAL_X:=6.0}"
: "${GOAL_Y:=2.0}"
export SIM_TO_REAL_BUNDLE_ROOT="$COMPARISON_ROOT"
export ROS_DOMAIN_ID="${ISAAC5_DRLVO_ROS_DOMAIN_ID:-$DEFAULT_DOMAIN}"
export ISAAC5_GATE6B_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LOG_TAG="${ISAAC5_GATE_LOG_TAG:-}"
if [[ -n "$LOG_TAG" && ! "$LOG_TAG" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=invalid_log_tag value=$LOG_TAG"
    exit 2
fi
LOG_SUFFIX="${LOG_TAG:+_$LOG_TAG}"

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" \
    "$COMPARISON_WS/install/setup.bash" "$TRAIN_PYTHON" "$DRSPAAM_NODE" \
    "$DRSPAAM_MODEL" "$MERGER" "$TRACKER" "$GOAL_BRIDGE" "$DRLVO_NODE" \
    "$DRLVO_MODEL" "$MONITOR" "$COMPARISON_ROOT/runtime_code"; do
    [[ -e "$required" ]] || { echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=missing_input path=$required"; exit 2; }
done
if [[ -n "$RELAY" && ! -e "$RELAY" ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=missing_input path=$RELAY"; exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
source "$COMPARISON_WS/install/setup.bash"
set -u
export PYTHONPATH="$DRSPAAM_ROOT:$DRSPAAM_ROS2_ROOT:${PYTHONPATH:-}"

MERGER_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_scan_merger${LOG_SUFFIX}.log"
DETECTOR_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_drspaam${LOG_SUFFIX}.log"
TRACKER_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_tracker${LOG_SUFFIX}.log"
GOAL_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_goal_bridge${LOG_SUFFIX}.log"
RELAY_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_track_relay${LOG_SUFFIX}.log"
DRLVO_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_drlvo${LOG_SUFFIX}.log"
CROWD_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_crowd_runtime_580${LOG_SUFFIX}.log"
MONITOR_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_drlvo_monitor${LOG_SUFFIX}.log"
GRAPH_LOG="$BACKEND_ROOT/generated/gate${GATE_ID}_ros_graph${LOG_SUFFIX}.log"
for log in "$MERGER_LOG" "$DETECTOR_LOG" "$TRACKER_LOG" "$GOAL_LOG" "$RELAY_LOG" "$DRLVO_LOG" "$CROWD_LOG" "$MONITOR_LOG" "$GRAPH_LOG"; do
    : >"$log"
done

merger_pid="" detector_pid="" tracker_pid="" goal_pid="" relay_pid="" drlvo_pid="" crowd_pid="" monitor_pid=""
stop_group() {
    local signal="$1" pid="$2"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" -- "-$pid" 2>/dev/null || kill "-$signal" "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    stop_group TERM "$monitor_pid"; stop_group TERM "$crowd_pid"
    stop_group INT "$drlvo_pid"; stop_group INT "$relay_pid"; stop_group INT "$goal_pid"
    stop_group INT "$tracker_pid"; stop_group INT "$detector_pid"; stop_group INT "$merger_pid"
}
trap cleanup EXIT INT TERM

setsid /usr/bin/python3 "$MERGER" --ros-args \
    -p input_scan_01_topic:=/scan_01 -p input_scan_02_topic:=/scan_02 \
    -p output_topic:=/scan_merged -p output_frame:=base_link \
    -p output_samples:=360 -p enable_self_filter:=true >"$MERGER_LOG" 2>&1 &
merger_pid="$!"
setsid "$TRAIN_PYTHON" "$DRSPAAM_NODE" --ros-args \
    -p weight_file:="$DRSPAAM_MODEL" -p detector_model:=DR-SPAAM \
    -p conf_thresh:=0.20 -p stride:=5 -p panoramic_scan:=true \
    -p reverse_scan:=true -p drow_to_ros:=true -p target_frame:=base_link \
    -p subscriber.scan.topic:=/scan_merged >"$DETECTOR_LOG" 2>&1 &
detector_pid="$!"
setsid /usr/bin/python3 "$TRACKER" --ros-args \
    -p use_sim_time:=true -p tracking_frame:=odom \
    -p input_topic:=/dr_spaam_detections_scored -p output_topic:="$TRACKER_OUTPUT_TOPIC" \
    -p association_threshold:=0.8 -p min_hits:=3 -p max_age:=8 \
    -p max_coast_time:=0.75 -p acceleration_sigma:=2.0 -p measurement_sigma:=0.10 \
    -p max_prediction_dt:=0.50 -p measurement_history_size:=8 \
    -p velocity_fit_min_samples:=3 -p velocity_fit_min_span:=0.15 >"$TRACKER_LOG" 2>&1 &
tracker_pid="$!"
if [[ "$MODE" == "track_freshness" ]]; then
    setsid /usr/bin/python3 "$RELAY" --normal-until 4.0 --stale-after 4.9 \
        --recover-after 7.5 --ros-args -p use_sim_time:=true >"$RELAY_LOG" 2>&1 &
    relay_pid="$!"
elif [[ "$MODE" == "front_veto" ]]; then
    setsid /usr/bin/python3 "$RELAY" --normal-until 6.0 --recover-after 9.0 \
        --obstacle-range 0.5 --ros-args -p use_sim_time:=true >"$RELAY_LOG" 2>&1 &
    relay_pid="$!"
fi
setsid /usr/bin/python3 "$GOAL_BRIDGE" --ros-args \
    -p use_sim_time:=true -p global_frame:=odom -p world_frame:=odom \
    -p base_frame:=base_link -p goal_x:="$GOAL_X" -p goal_y:="$GOAL_Y" \
    -p publish_rate:=10.0 -p pose_topic:=/odom >"$GOAL_LOG" 2>&1 &
goal_pid="$!"
setsid "$TRAIN_PYTHON" "$DRLVO_NODE" --ros-args \
    -p use_sim_time:=true -p mode:=base -p model:="$DRLVO_MODEL" -p device:=cuda \
    -p scan_01_topic:="$DRLVO_SCAN_01_TOPIC" -p scan_02_topic:="$DRLVO_SCAN_02_TOPIC" -p odom_topic:=/odom \
    -p local_subgoal_topic:=/semantic_cnn/local_subgoal \
    -p final_goal_topic:=/semantic_cnn/final_goal \
    -p pedestrian_source:=dr_spaam -p require_pedestrian_truth:=false \
    -p pedestrian_tracks_topic:="$DRLVO_TRACK_TOPIC" \
    -p pedestrian_track_timeout:=0.8 -p base_frame:=base_link \
    -p odom_frame:=odom -p map_frame:=odom -p pedestrian_track_frame:=odom \
    -p cmd_vel_topic:="$SHADOW_TOPIC" \
    -p actuation_decision_topic:=/drl_vo/actuation_decision \
    -p inference_metrics_topic:=/navigation_evaluation/inference_metrics \
    -p publish_policy_actions:=true -p scan_timeout:=0.8 -p odom_timeout:=0.8 \
    -p subgoal_timeout:=0.8 -p final_goal_timeout:=0.8 \
    -p front_stop_distance:=0.5 -p max_linear:=0.3 -p max_angular:=1.5 >"$DRLVO_LOG" 2>&1 &
drlvo_pid="$!"

nodes_ready=0; ready_nodes=""
for _ in $(seq 1 140); do
    if ! kill -0 "$merger_pid" 2>/dev/null || ! kill -0 "$detector_pid" 2>/dev/null \
        || ! kill -0 "$tracker_pid" 2>/dev/null || ! kill -0 "$goal_pid" 2>/dev/null \
        || ! kill -0 "$drlvo_pid" 2>/dev/null; then break; fi
    if [[ -n "$relay_pid" ]] && ! kill -0 "$relay_pid" 2>/dev/null; then break; fi
    nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
    if grep -qx '/v7_dual_laser_scan_merger' <<<"$nodes" \
        && grep -qx '/dr_spaam_ros' <<<"$nodes" \
        && grep -qx '/pedestrian_point_tracker' <<<"$nodes" \
        && grep -qx '/fixed_global_goal_bridge' <<<"$nodes" \
        && grep -qx '/drl_vo_fixed_dual_inference' <<<"$nodes" \
        && { [[ "$MODE" != "track_freshness" ]] || grep -qx '/isaac5_gate82_track_dropout_relay' <<<"$nodes"; } \
        && { [[ "$MODE" != "front_veto" ]] || grep -qx '/isaac5_gate83_front_obstacle_relay' <<<"$nodes"; }; then
        nodes_ready=1; ready_nodes="$nodes"; break
    fi
    sleep 0.5
done
if [[ "$nodes_ready" != 1 ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=nodes_not_ready"
    tail -n 120 "$TRACKER_LOG"; tail -n 120 "$RELAY_LOG"; tail -n 160 "$DRLVO_LOG"; exit 2
fi

crowd_args=(
    --duration "$CROWD_DURATION" --suppress-tracks
    --pedestrian-count "${ISAAC5_GATE9_PEDESTRIAN_COUNT:-20}"
    --pedestrian-seed "${ISAAC5_GATE9_PEDESTRIAN_SEED:-7}"
    --social-mode "${ISAAC5_GATE9_SOCIAL_MODE:-social}"
    --pedestrian-update-rate "${ISAAC5_GATE9_PEDESTRIAN_UPDATE_RATE:-30}"
    --gui-render-rate "${ISAAC5_GATE9_GUI_RENDER_RATE:-10}"
)
if [[ "$MODE" != "fixed_goal_gui" ]]; then
    crowd_args=(--headless "${crowd_args[@]}")
else
    crowd_args+=(--screenshot "${ISAAC5_GATE9_SCREENSHOT:-gate9g_fixed_goal_gui.png}")
fi
if [[ "$MODE" == "closed_loop_smoke" || "$MODE" == "fixed_goal" || "$MODE" == "fixed_goal_gui" ]]; then
    crowd_args+=(--mobile-robot --command-topic "$SHADOW_TOPIC" --command-timeout 0.5)
fi
"$SCRIPT_DIR/validate_crowd.sh" "${crowd_args[@]}" >"$CROWD_LOG" 2>&1 &
crowd_pid="$!"
crowd_ready=0
for _ in $(seq 1 120); do
    if grep -F 'GATE6B_CROWD_READY=' "$CROWD_LOG" >/dev/null 2>&1; then crowd_ready=1; break; fi
    if ! kill -0 "$crowd_pid" 2>/dev/null; then break; fi
    sleep 0.25
done
if [[ "$crowd_ready" != 1 ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=crowd_not_ready"; tail -n 120 "$CROWD_LOG"; exit 2
fi

{
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "DRSPAAM_MODEL=$DRSPAAM_MODEL"
    echo "DRSPAAM_SHA256=$(sha256sum "$DRSPAAM_MODEL" | awk '{print $1}')"
    echo "DRSPAAM_CONFIDENCE_THRESHOLD=0.20"
    echo "DRLVO_MODEL=$DRLVO_MODEL"
    echo "DRLVO_SHA256=$(sha256sum "$DRLVO_MODEL" | awk '{print $1}')"
    echo "DRLVO_MODE=base"
    echo "DRLVO_DEVICE=cuda"
    echo "DRLVO_OBSERVATION_SHAPE=19202"
    echo "RAW_LIDAR_LAYOUT=2x2000_range_0.5_to_50m_at_15Hz"
    echo "DRLVO_EFFECTIVE_RANGE=0.1_to_8m_parameter_contract"
    echo "DRLVO_CMD_TOPIC=$SHADOW_TOPIC"
    echo "PEDESTRIAN_COUNT=${ISAAC5_GATE9_PEDESTRIAN_COUNT:-20}"
    echo "PEDESTRIAN_SEED=${ISAAC5_GATE9_PEDESTRIAN_SEED:-7}"
    echo "PEDESTRIAN_SOCIAL_MODE=${ISAAC5_GATE9_SOCIAL_MODE:-social}"
    echo "PEDESTRIAN_CONTROL_RATE_HZ=${ISAAC5_GATE9_PEDESTRIAN_UPDATE_RATE:-30}"
    echo "PEDESTRIAN_POSE_COMMIT_RATE_HZ=${ISAAC5_GATE9_PEDESTRIAN_UPDATE_RATE:-30}"
    echo "GUI_RENDER_RATE_HZ=${ISAAC5_GATE9_GUI_RENDER_RATE:-10}"
    echo "TRACK_FRESHNESS_MODE=$MODE"
    if [[ "$MODE" == "front_veto" ]]; then
        echo "SAFETY_FAULT=synthetic_dual_lidar_near_ring_0.5m"
        echo "SAFETY_TRUTH_SCOPE=geometric_proximity_proxy_not_PhysX_contact"
    fi
    if [[ "$MODE" == "closed_loop_smoke" || "$MODE" == "fixed_goal" || "$MODE" == "fixed_goal_gui" ]]; then
        echo "CONTROL_SCOPE=bounded_closed_loop_to_isaac5_mobile_robot_adapter"
        echo "ARM_LOCK=authored_upright_pose_on_dynamic_base_no_arm_dynamics"
        echo "FIXED_GOAL_ODOM=$GOAL_X,$GOAL_Y"
    fi
    echo "NODES"; printf '%s\n' "$ready_nodes" | sort; echo "/isaac5_gate6_pedestrian"
} >"$GRAPH_LOG"

monitor_args=(--duration "$MONITOR_DURATION")
if [[ "$MODE" == "fixed_goal" || "$MODE" == "fixed_goal_gui" ]]; then
    monitor_args+=(--goal-x "$GOAL_X" --goal-y "$GOAL_Y" --command-topic "$SHADOW_TOPIC")
fi
setsid /usr/bin/python3 "$MONITOR" "${monitor_args[@]}" >"$MONITOR_LOG" 2>&1 &
monitor_pid="$!"
monitor_status=0; wait "$monitor_pid" || monitor_status="$?"; monitor_pid=""
crowd_status=0; wait "$crowd_pid" || crowd_status="$?"; crowd_pid=""
if [[ "$monitor_status" != 0 ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=monitor_failed status=$monitor_status"
    cat "$MONITOR_LOG"; tail -n 160 "$DRLVO_LOG"; exit 2
fi
if [[ "$crowd_status" != 0 ]]; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=crowd_failed status=$crowd_status"
    tail -n 120 "$CROWD_LOG"; exit 2
fi
for pid in "$merger_pid" "$detector_pid" "$tracker_pid" "$goal_pid" "$relay_pid" "$drlvo_pid"; do
    [[ -z "$pid" ]] && continue
    kill -0 "$pid" 2>/dev/null || { echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=process_exited pid=$pid"; exit 2; }
done
grep -F 'GATE6B_CROWD_RESULT={"status": "PASS"' "$CROWD_LOG" >/dev/null
grep -F "$MONITOR_MARKER" "$MONITOR_LOG" >/dev/null
if grep -E '/semantic_cnn_fixed_dual|/s3net' "$GRAPH_LOG" >/dev/null; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=other_policy_node_present"; exit 2
fi
if ros2 topic info /cmd_vel --no-daemon 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
    echo "GATE${GATE_ID}_DRLVO_RESULT=FAIL reason=real_cmd_vel_publisher_present"; exit 2
fi
echo "GATE${GATE_ID}_DRLVO_RESULT=PASS domain=$ROS_DOMAIN_ID monitor_log=$MONITOR_LOG drlvo_log=$DRLVO_LOG"
