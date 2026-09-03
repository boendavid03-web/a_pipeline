#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
TRAIN_PYTHON="$PROJECT_ROOT/.venvs/train/bin/python"
DR_SPAAM_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/2D_lidar_person_detection/dr_spaam"
DR_SPAAM_ROS2_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2"
DR_SPAAM_NODE="$DR_SPAAM_ROS2_ROOT/dr_spaam_ros2/dr_spaam_w_score_ros.py"
TRACKER="$ROS_WS/src/semantic_nav_gazebo/scripts/pedestrian_point_tracker.py"
EVALUATOR="$ROS_WS/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_evaluator.py"
GENERATOR="$SCRIPT_DIR/generate_crowded_tracking_config.py"
CHECKPOINT="$DR_SPAAM_ROS2_ROOT/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"

SCENARIO="${CROWDED_STRESS_SCENARIO:-C}"
SCENARIO="${SCENARIO^^}"
DURATION_SEC="${CROWDED_STRESS_DURATION_SEC:-25}"
DOMAIN_ID="${CROWDED_STRESS_DOMAIN_ID:-80}"
SPACING_M="${CROWDED_STRESS_SPACING_M:-}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${CROWDED_STRESS_RUN_DIR:-$PROJECT_ROOT/runs/dr_spaam_isaac_crowded_tracking/${SCENARIO,,}_$RUN_ID}"

case "$SCENARIO" in
    A) PEOPLE_COUNT=2; STRESS_IDS='[stress_a_0, stress_b_0]'; DEFAULT_SPACING=1.0 ;;
    B) PEOPLE_COUNT=2; STRESS_IDS='[stress_a_0, stress_b_0]'; DEFAULT_SPACING=-1.0 ;;
    C) PEOPLE_COUNT=2; STRESS_IDS='[stress_a_0, stress_b_0]'; DEFAULT_SPACING=-1.0 ;;
    D) PEOPLE_COUNT=2; STRESS_IDS='[stress_a_0, stress_b_0]'; DEFAULT_SPACING=0.75 ;;
    E) PEOPLE_COUNT=3; STRESS_IDS='[stress_a_0, stress_b_0, stress_c_0]'; DEFAULT_SPACING=-1.0 ;;
    *) echo "ERROR: CROWDED_STRESS_SCENARIO must be A, B, C, D, or E." >&2; exit 2 ;;
esac
REQUESTED_SPACING="$DEFAULT_SPACING"
generator_spacing=()
if [[ -n "$SPACING_M" ]]; then
    REQUESTED_SPACING="$SPACING_M"
    generator_spacing+=(--spacing "$SPACING_M")
fi

for required in \
    /opt/ros/humble/setup.bash \
    "$ROS_WS/install/setup.bash" \
    "$TRAIN_PYTHON" \
    "$DR_SPAAM_NODE" \
    "$TRACKER" \
    "$EVALUATOR" \
    "$GENERATOR" \
    "$CHECKPOINT"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required crowded tracking input is missing: $required" >&2
        exit 1
    fi
done
if ! awk -v value="$DURATION_SEC" 'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value >= 10) }'; then
    echo "ERROR: CROWDED_STRESS_DURATION_SEC must be at least 10 seconds." >&2
    exit 2
fi
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]] || (( DOMAIN_ID > 232 )); then
    echo "ERROR: CROWDED_STRESS_DOMAIN_ID must be an integer from 0 through 232." >&2
    exit 2
fi

mkdir -p "$RUN_DIR"
SCENARIO_CONFIG="$RUN_DIR/scenario.yaml"
SCENARIO_METADATA="$RUN_DIR/scenario_metadata.json"
/usr/bin/python3 "$GENERATOR" \
    --scenario "$SCENARIO" \
    --output "$SCENARIO_CONFIG" \
    --metadata-output "$SCENARIO_METADATA" \
    --speed "${CROWDED_STRESS_PEDESTRIAN_SPEED:-0.8}" \
    --seed "${CROWDED_STRESS_SEED:-7}" \
    "${generator_spacing[@]}" \
    >"$RUN_DIR/scenario_generator.log" 2>&1

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export PYTHONPATH="$DR_SPAAM_ROOT:$DR_SPAAM_ROS2_ROOT:${PYTHONPATH:-}"

existing_cmd_vel="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
if [[ "$existing_cmd_vel" =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
    echo "ERROR: stationary stress test found an existing /cmd_vel publisher in ROS_DOMAIN_ID=$DOMAIN_ID:" >&2
    echo "$existing_cmd_vel" >&2
    exit 5
fi

detector_pid=""
tracker_pid=""
evaluator_pid=""
bag_pid=""
preflight_pid=""
guard_pid=""
isaac_pid=""
stop_process() {
    local signal="$1"
    local pid="$2"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    stop_process TERM "$guard_pid"
    stop_process TERM "$preflight_pid"
    stop_process INT "$bag_pid"
    stop_process INT "$evaluator_pid"
    stop_process TERM "$tracker_pid"
    stop_process TERM "$detector_pid"
}
trap cleanup EXIT INT TERM

"$TRAIN_PYTHON" "$DR_SPAAM_NODE" --ros-args \
    -p weight_file:="$CHECKPOINT" \
    -p detector_model:=DR-SPAAM \
    -p conf_thresh:=0.95 \
    -p stride:=5 \
    -p panoramic_scan:=true \
    -p reverse_scan:=true \
    -p drow_to_ros:=true \
    -p target_frame:=base_link \
    -p subscriber.scan.topic:=/scan_merged \
    >"$RUN_DIR/dr_spaam.log" 2>&1 &
detector_pid=$!

/usr/bin/python3 "$TRACKER" --ros-args \
    -p use_sim_time:=true \
    -p tracking_frame:=odom \
    -p association_threshold:=0.8 \
    -p min_hits:=3 \
    -p max_age:=8 \
    -p max_coast_time:=0.75 \
    -p acceleration_sigma:=2.0 \
    -p measurement_sigma:=0.10 \
    -p max_prediction_dt:=0.50 \
    -p measurement_history_size:=8 \
    -p velocity_fit_min_samples:=3 \
    -p velocity_fit_min_span:=0.15 \
    >"$RUN_DIR/tracker.log" 2>&1 &
tracker_pid=$!

/usr/bin/python3 "$EVALUATOR" --ros-args \
    -p use_sim_time:=true \
    -p output_dir:="$RUN_DIR/evaluation" \
    -p scenario:="$SCENARIO" \
    -p stress_ids:="$STRESS_IDS" \
    -p requested_spacing:="$REQUESTED_SPACING" \
    -p target_frame:=odom \
    -p max_sync_offset:=0.08 \
    -p match_threshold:=0.5 \
    -p visible_distance:=8.0 \
    -p close_encounter_distance:=1.5 \
    -p crossing_window:=1.0 \
    -p failure_lookback:=0.5 \
    >"$RUN_DIR/evaluator.log" 2>&1 &
evaluator_pid=$!

ros2 bag record -o "$RUN_DIR/rosbag" \
    /scan_01 /scan_02 /scan_merged \
    /dr_spaam_detections_scored /pedestrian_tracks \
    /pedestrian_track_velocity_diagnostics /pedestrian_ground_truth \
    /odom /tf /tf_static /clock /isaac/reset_event \
    >"$RUN_DIR/rosbag.log" 2>&1 &
bag_pid=$!

(
    for _ in $(seq 1 300); do
        if timeout 2s ros2 topic echo --once /pedestrian_tracks >/dev/null 2>&1; then
            status=0
            {
                echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
                echo "SCENARIO=$SCENARIO"
                echo "ROBOT_MODE=stationary"
                echo "DETECTOR_PARAMETERS checkpoint=$CHECKPOINT confidence_threshold=0.95"
                echo "TRACKER_PARAMETERS association_threshold=0.8 min_hits=3 max_age=8 max_coast_time=0.75 acceleration_sigma=2.0 measurement_sigma=0.10 max_prediction_dt=0.50 measurement_history_size=8 velocity_fit_min_samples=3 velocity_fit_min_span=0.15"
                for topic in /scan_01 /scan_02 /scan_merged /pedestrian_ground_truth /odom /dr_spaam_detections_scored /pedestrian_tracks; do
                    info="$(ros2 topic info "$topic" -v 2>&1 || true)"
                    echo "TOPIC $topic"
                    echo "$info"
                    if [[ ! "$info" =~ Publisher\ count:\ 1 ]]; then
                        status=1
                    fi
                done
                echo "TOPIC /cmd_vel"
                cmd_info="$(ros2 topic info /cmd_vel -v 2>&1 || true)"
                echo "$cmd_info"
                if [[ "$cmd_info" =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
                    status=1
                fi
            } >"$RUN_DIR/runtime_topic_info.txt"
            if (( status == 0 )); then
                touch "$RUN_DIR/runtime_contract_pass"
            else
                touch "$RUN_DIR/runtime_contract_fail"
            fi
            exit "$status"
        fi
        sleep 1
    done
    echo "Timed out waiting for /pedestrian_tracks" >"$RUN_DIR/runtime_topic_info.txt"
    touch "$RUN_DIR/runtime_contract_fail"
    exit 1
) &
preflight_pid=$!

echo "CROWDED_STRESS_RUN_DIR=$RUN_DIR"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "SCENARIO=$SCENARIO PEOPLE=$PEOPLE_COUNT REQUESTED_SPACING_M=$REQUESTED_SPACING"
echo "Detector PID=$detector_pid; tracker PID=$tracker_pid; evaluator PID=$evaluator_pid; rosbag PID=$bag_pid"

export ISAAC_ROS_DOMAIN_ID="$DOMAIN_ID"
export ISAAC_SCENE=custom
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_PEDESTRIAN_COUNT="$PEOPLE_COUNT"
export ISAAC_PEDESTRIAN_SEED="${CROWDED_STRESS_SEED:-7}"
export ISAAC_PEDESTRIAN_SPEED="${CROWDED_STRESS_PEDESTRIAN_SPEED:-0.8}"
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off
export ISAAC_ROBOT_PHYSICS=0
export ISAAC_LIDAR_MODE=physx
export ISAAC_LIDAR_RATE_HZ=15
export ISAAC_LIDAR_SAMPLE_COUNT=2000
export ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG="$SCENARIO_CONFIG"
export ISAAC_EXPLICIT_CUSTOM_IRA_MIN_START_SEPARATION_M=0.0
export ISAAC_CUSTOM_SPAWN_X_M=13.5
export ISAAC_CUSTOM_SPAWN_Y_M=6.5
export ISAAC_CUSTOM_SPAWN_Z_M=0.01

set +e
"$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh" \
    --headless --duration "$DURATION_SEC" \
    >"$RUN_DIR/isaac.log" 2>&1 &
isaac_pid=$!
(
    while kill -0 "$isaac_pid" 2>/dev/null; do
        cmd_info="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
        if [[ "$cmd_info" =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
            {
                echo "STATIONARY_GUARD=FAIL"
                echo "$cmd_info"
            } >"$RUN_DIR/stationary_guard.txt"
            kill -TERM "$isaac_pid" 2>/dev/null || true
            exit 1
        fi
        sleep 1
    done
    echo "STATIONARY_GUARD=PASS no_cmd_vel_publisher_observed" >"$RUN_DIR/stationary_guard.txt"
) &
guard_pid=$!
wait "$isaac_pid"
isaac_status=$?
set -e

sleep 2
cleanup
trap - EXIT INT TERM
echo "$isaac_status" >"$RUN_DIR/isaac_exit_code.txt"

if [[ ! -f "$RUN_DIR/evaluation/summary.json" ]]; then
    echo "ERROR: crowded evaluator summary was not produced." >&2
    exit 8
fi
if [[ ! -f "$RUN_DIR/runtime_contract_pass" ]]; then
    echo "ERROR: runtime topic/publisher contract did not pass." >&2
    sed -n '1,240p' "$RUN_DIR/runtime_topic_info.txt" >&2 || true
    exit 9
fi
if ! grep -q '^STATIONARY_GUARD=PASS' "$RUN_DIR/stationary_guard.txt"; then
    echo "ERROR: stationary /cmd_vel guard failed." >&2
    cat "$RUN_DIR/stationary_guard.txt" >&2 || true
    exit 10
fi
echo "CROWDED_STRESS_SUMMARY=$RUN_DIR/evaluation/summary.json"
exit "$isaac_status"
