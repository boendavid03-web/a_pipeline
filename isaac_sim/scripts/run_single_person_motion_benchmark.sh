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
EVALUATOR="$ROS_WS/src/semantic_nav_gazebo/scripts/pedestrian_tracking_evaluator.py"
CHECKPOINT="$DR_SPAAM_ROS2_ROOT/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
GENERATOR="$SCRIPT_DIR/generate_single_motion_config.py"
ANALYZER="$SCRIPT_DIR/analyze_single_motion_benchmark.py"

SCENARIO="${SCENARIO:-front_approach}"
DOMAIN_ID="${SINGLE_MOTION_DOMAIN_ID:-81}"
CAPTURE_SEC="${SINGLE_MOTION_CAPTURE_SEC:-30}"
SUITE_ID="${SINGLE_MOTION_SUITE_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_ROOT="${SINGLE_MOTION_RUN_ROOT:-$PROJECT_ROOT/runs/dr_spaam_single_motion}"

case "$SCENARIO" in
    front_approach|front_leave|lateral|diagonal|all) ;;
    *) echo "ERROR: SCENARIO must be front_approach, front_leave, lateral, diagonal, or all." >&2; exit 2 ;;
esac
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]] || (( DOMAIN_ID > 232 )); then
    echo "ERROR: SINGLE_MOTION_DOMAIN_ID must be an integer from 0 through 232." >&2
    exit 2
fi
if ! awk -v value="$CAPTURE_SEC" 'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value >= 30) }'; then
    echo "ERROR: SINGLE_MOTION_CAPTURE_SEC must be at least 30 seconds." >&2
    exit 2
fi

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" "$TRAIN_PYTHON" \
    "$DR_SPAAM_NODE" "$TRACKER" "$EVALUATOR" "$CHECKPOINT" "$GENERATOR" "$ANALYZER"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required single-motion input is missing: $required" >&2
        exit 1
    fi
done

if [[ "$SCENARIO" == "all" ]]; then
    summaries=()
    for item in front_approach front_leave lateral diagonal; do
        SCENARIO="$item" SINGLE_MOTION_SUITE_ID="$SUITE_ID" \
            SINGLE_MOTION_DOMAIN_ID="$DOMAIN_ID" SINGLE_MOTION_CAPTURE_SEC="$CAPTURE_SEC" \
            SINGLE_MOTION_RUN_ROOT="$RUN_ROOT" "$0"
        summaries+=(--summary "$RUN_ROOT/$item/$SUITE_ID/summary.json")
    done
    suite_dir="$RUN_ROOT/suites/$SUITE_ID"
    mkdir -p "$suite_dir"
    "$ANALYZER" report --output "$suite_dir/single_person_motion_report.md" "${summaries[@]}"
    /usr/bin/python3 - "$suite_dir/suite_manifest.json" "$SUITE_ID" "${summaries[@]}" <<'PY'
import json
import sys
from pathlib import Path
output = Path(sys.argv[1])
values = sys.argv[3:]
summaries = [values[index + 1] for index in range(0, len(values), 2)]
output.write_text(json.dumps({"schema": "dr_spaam_single_motion_suite/v1", "suite_id": sys.argv[2], "summaries": summaries}, indent=2) + "\n", encoding="utf-8")
PY
    echo "SINGLE_MOTION_SUITE=PASS report=$suite_dir/single_person_motion_report.md"
    exit 0
fi

RUN_DIR="$RUN_ROOT/$SCENARIO/$SUITE_ID"
if [[ -e "$RUN_DIR" ]]; then
    echo "ERROR: refusing to overwrite existing run directory: $RUN_DIR" >&2
    exit 3
fi
mkdir -p "$RUN_DIR/evaluation"
SCENARIO_CONFIG="$RUN_DIR/scenario.yaml"
SCENARIO_METADATA="$RUN_DIR/scenario_metadata.yaml"
"$GENERATOR" --scenario "$SCENARIO" --output "$SCENARIO_CONFIG" \
    --metadata-output "$SCENARIO_METADATA" --speed 0.8 --seed 7 --clearance 0.55 \
    >"$RUN_DIR/scenario_generator.log" 2>&1

read -r ROBOT_X ROBOT_Y ROBOT_YAW < <(
    /usr/bin/python3 - "$SCENARIO_METADATA" <<'PY'
import sys, yaml
pose = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["robot_pose_odom"]
print(pose["x_m"], pose["y_m"], pose["yaw_rad"])
PY
)
read -r RESET_QZ RESET_QW < <(
    /usr/bin/python3 - "$ROBOT_YAW" <<'PY'
import math, sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

existing_processes="$(
    ps -eo pid=,args= \
        | grep -E 'isaacsim-6\.0\.1/(kit/)?python/bin/python3|[g]z sim|[g]zserver|[g]azebo|ros2 bag record' \
        | grep -v '[g]rep -E' \
        | grep -v "$$" \
        || true
)"
if [[ -n "$existing_processes" ]]; then
    echo "ERROR: existing simulator or rosbag process found; refusing parallel benchmark:" >&2
    echo "$existing_processes" >&2
    exit 5
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export PYTHONPATH="$DR_SPAAM_ROOT:$DR_SPAAM_ROS2_ROOT:${PYTHONPATH:-}"

if [[ -n "$(ros2 node list 2>/dev/null || true)" ]]; then
    echo "ERROR: ROS_DOMAIN_ID=$DOMAIN_ID is not empty; use an isolated domain." >&2
    ros2 node list >&2 || true
    exit 5
fi
existing_cmd_vel="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
if [[ "$existing_cmd_vel" =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
    echo "ERROR: /cmd_vel already has a publisher in ROS_DOMAIN_ID=$DOMAIN_ID." >&2
    exit 5
fi

detector_pid=""
tracker_pid=""
evaluator_pid=""
bag_pid=""
isaac_pid=""
guard_pid=""
intentional_cleanup=0
stop_process() {
    local signal="$1" pid="$2"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    intentional_cleanup=1
    stop_process TERM "$guard_pid"
    stop_process INT "$bag_pid"
    stop_process INT "$evaluator_pid"
    stop_process TERM "$tracker_pid"
    stop_process TERM "$detector_pid"
    stop_process TERM "$isaac_pid"
}
trap cleanup EXIT INT TERM

"$TRAIN_PYTHON" "$DR_SPAAM_NODE" --ros-args \
    -p weight_file:="$CHECKPOINT" -p detector_model:=DR-SPAAM -p conf_thresh:=0.95 \
    -p stride:=5 -p panoramic_scan:=true -p reverse_scan:=true -p drow_to_ros:=true \
    -p target_frame:=base_link -p subscriber.scan.topic:=/scan_merged \
    >"$RUN_DIR/dr_spaam.log" 2>&1 &
detector_pid=$!

export ISAAC_ROS_DOMAIN_ID="$DOMAIN_ID"
export ISAAC_SCENE=custom
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_PEDESTRIAN_COUNT=1
export ISAAC_PEDESTRIAN_SEED=7
export ISAAC_PEDESTRIAN_SPEED=0.8
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off
export ISAAC_ROBOT_PHYSICS=0
export ISAAC_LIDAR_MODE=physx
export ISAAC_LIDAR_RATE_HZ=15
export ISAAC_LIDAR_SAMPLE_COUNT=2000
export ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG="$SCENARIO_CONFIG"
export ISAAC_EXPLICIT_CUSTOM_IRA_MIN_START_SEPARATION_M=0.0
export ISAAC_CUSTOM_SPAWN_X_M="$ROBOT_X"
export ISAAC_CUSTOM_SPAWN_Y_M="$ROBOT_Y"
export ISAAC_CUSTOM_SPAWN_Z_M=0.01

"$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh" --headless --duration 0 \
    >"$RUN_DIR/isaac.log" 2>&1 &
isaac_pid=$!

topic_once() {
    timeout 5s ros2 topic echo --once "$1" >/dev/null 2>&1
}
topic_has_one_publisher() {
    local info
    info="$(ros2 topic info "$1" -v 2>&1 || true)"
    [[ "$info" =~ Publisher\ count:\ 1 ]]
}
ready=0
for _ in $(seq 1 300); do
    if ! kill -0 "$isaac_pid" 2>/dev/null || ! kill -0 "$detector_pid" 2>/dev/null; then
        echo "ERROR: an owned process exited before readiness." >&2
        exit 8
    fi
    if topic_once /scan_01 && topic_once /scan_02 && topic_once /scan_merged \
        && topic_once /pedestrian_ground_truth && topic_once /odom \
        && topic_once /dr_spaam_detections_scored && topic_once /clock; then
        ready=1
        break
    fi
    sleep 1
done
if (( ready == 0 )); then
    echo "ERROR: timed out waiting for sensor/detector readiness." >&2
    exit 8
fi

reset_capture="$RUN_DIR/reset_event.txt"
timeout 15s ros2 topic echo --once /isaac/reset_event >"$reset_capture" 2>&1 &
reset_wait_pid=$!
reset_listener_ready=0
for _ in $(seq 1 30); do
    reset_info="$(ros2 topic info /isaac/reset_event -v 2>&1 || true)"
    if [[ "$reset_info" =~ Subscription\ count:\ ([1-9]|[1-9][0-9]+) ]]; then
        reset_listener_ready=1
        break
    fi
    sleep 0.2
done
if (( reset_listener_ready == 0 )); then
    echo "ERROR: reset-event listener did not join the ROS graph." >&2
    stop_process TERM "$reset_wait_pid"
    exit 9
fi
ros2 topic pub --once /isaac/reset_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: $ROBOT_X, y: $ROBOT_Y, z: 0.01}, orientation: {z: $RESET_QZ, w: $RESET_QW}}}" \
    >"$RUN_DIR/reset_request.txt" 2>&1
if ! wait "$reset_wait_pid"; then
    echo "ERROR: reset event was not observed." >&2
    exit 9
fi
sleep 1

/usr/bin/python3 "$TRACKER" --ros-args \
    -p use_sim_time:=true -p tracking_frame:=odom -p association_threshold:=0.8 \
    -p min_hits:=3 -p max_age:=8 -p max_coast_time:=0.75 \
    -p acceleration_sigma:=2.0 -p measurement_sigma:=0.10 -p max_prediction_dt:=0.50 \
    -p measurement_history_size:=8 -p velocity_fit_min_samples:=3 -p velocity_fit_min_span:=0.15 \
    >"$RUN_DIR/tracker.log" 2>&1 &
tracker_pid=$!

/usr/bin/python3 "$EVALUATOR" --ros-args \
    -p use_sim_time:=true -p output_dir:="$RUN_DIR/evaluation" -p target_frame:=odom \
    -p max_sync_offset:=0.08 -p match_threshold:=0.5 \
    -p gt_velocity_fit_half_window:=0.30 \
    -p gt_velocity_fit_half_windows:="[0.20, 0.30, 0.40]" \
    -p gt_velocity_fit_min_samples:=5 -p direction_min_speed:=0.20 \
    >"$RUN_DIR/evaluator.log" 2>&1 &
evaluator_pid=$!

for _ in $(seq 1 120); do
    if topic_once /pedestrian_tracks \
        && grep -q 'tracking evaluator ready' "$RUN_DIR/evaluator.log" 2>/dev/null; then
        ready=1
        break
    fi
    ready=0
    sleep 1
done
if (( ready == 0 )); then
    echo "ERROR: timed out waiting for tracker/evaluator readiness." >&2
    exit 10
fi

{
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "SCENARIO=$SCENARIO"
    echo "ROBOT_MODE=stationary"
    echo "LIDAR_PARAMETERS backend=physx rate_hz=15 sample_count=2000"
    echo "DETECTOR_PARAMETERS checkpoint=$CHECKPOINT model=DR-SPAAM confidence_threshold=0.95 stride=5 panoramic=true reverse=true drow_to_ros=true"
    echo "TRACKER_PARAMETERS association_threshold=0.8 min_hits=3 max_age=8 max_coast_time=0.75 acceleration_sigma=2.0 measurement_sigma=0.10 max_prediction_dt=0.50 measurement_history_size=8 velocity_fit_min_samples=3 velocity_fit_min_span=0.15"
    for topic in /scan_01 /scan_02 /scan_merged /pedestrian_ground_truth /odom /clock /dr_spaam_detections_scored /pedestrian_tracks /pedestrian_track_velocity_diagnostics; do
        echo "TOPIC $topic"
        ros2 topic info "$topic" -v 2>&1 || true
    done
    echo "TOPIC /cmd_vel"
    ros2 topic info /cmd_vel -v 2>&1 || true
} >"$RUN_DIR/runtime_topic_info.txt"
for topic in /scan_01 /scan_02 /scan_merged /pedestrian_ground_truth /odom /clock /dr_spaam_detections_scored /pedestrian_tracks /pedestrian_track_velocity_diagnostics; do
    if ! topic_has_one_publisher "$topic"; then
        echo "ERROR: required topic does not have exactly one publisher: $topic" >&2
        exit 11
    fi
done
if ros2 topic info /cmd_vel 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
    echo "ERROR: /cmd_vel publisher appeared before capture." >&2
    exit 11
fi

(
    while kill -0 "$isaac_pid" 2>/dev/null; do
        if ros2 topic info /cmd_vel 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
            echo "STATIONARY_GUARD=FAIL cmd_vel_publisher_detected" >"$RUN_DIR/stationary_guard.txt"
            kill -TERM "$isaac_pid" 2>/dev/null || true
            exit 1
        fi
        sleep 1
    done
    [[ -f "$RUN_DIR/stationary_guard.txt" ]] || echo "STATIONARY_GUARD=PASS no_cmd_vel_publisher_observed" >"$RUN_DIR/stationary_guard.txt"
) &
guard_pid=$!

ros2 bag record --use-sim-time -o "$RUN_DIR/rosbag" \
    /scan_01 /scan_02 /scan_merged /dr_spaam_detections /dr_spaam_detections_scored \
    /pedestrian_tracks /pedestrian_track_velocity_diagnostics /pedestrian_ground_truth \
    /odom /tf /tf_static /clock /isaac/reset_event /cmd_vel \
    >"$RUN_DIR/rosbag.log" 2>&1 &
bag_pid=$!
bag_ready=0
for _ in $(seq 1 50); do
    if ! kill -0 "$bag_pid" 2>/dev/null; then
        echo "ERROR: rosbag recorder exited before capture." >&2
        exit 12
    fi
    bag_info="$(ros2 node info /rosbag2_recorder 2>/dev/null || true)"
    if [[ "$bag_info" == *"/pedestrian_ground_truth"* && "$bag_info" == *"/pedestrian_tracks"* ]]; then
        bag_ready=1
        break
    fi
    sleep 0.2
done
if (( bag_ready == 0 )); then
    echo "ERROR: rosbag recorder subscriptions were not ready." >&2
    exit 12
fi
if ! timeout 900s /usr/bin/python3 - "$CAPTURE_SEC" "$RUN_DIR/capture_window.yaml" <<'PY'
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

duration_ns = int(round(float(sys.argv[1]) * 1_000_000_000))
output = Path(sys.argv[2])
rclpy.init()
node = Node("single_motion_capture_clock")
start_ns = None
end_ns = None

def callback(message):
    global start_ns, end_ns
    current = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
    if start_ns is None:
        start_ns = current
    if current - start_ns >= duration_ns:
        end_ns = current

node.create_subscription(Clock, "/clock", callback, 20)
while rclpy.ok() and end_ns is None:
    rclpy.spin_once(node, timeout_sec=1.0)
if start_ns is None or end_ns is None:
    raise SystemExit("no complete /clock capture interval")
output.write_text(
    f"capture_start_ns: {start_ns}\ncapture_end_ns: {end_ns}\n"
    f"duration_sec: {(end_ns - start_ns) / 1.0e9:.9f}\n",
    encoding="utf-8",
)
node.destroy_node()
rclpy.shutdown()
PY
then
    echo "ERROR: 30-second simulation-time capture failed or exceeded 900 wall seconds." >&2
    exit 13
fi
if ! kill -0 "$isaac_pid" 2>/dev/null || ! kill -0 "$detector_pid" 2>/dev/null \
    || ! kill -0 "$tracker_pid" 2>/dev/null || ! kill -0 "$evaluator_pid" 2>/dev/null \
    || ! kill -0 "$bag_pid" 2>/dev/null; then
    echo "ERROR: an owned process exited during capture." >&2
    exit 13
fi
read -r capture_start_ns capture_end_ns < <(
    /usr/bin/python3 - "$RUN_DIR/capture_window.yaml" <<'PY'
import sys, yaml
window = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
print(window["capture_start_ns"], window["capture_end_ns"])
PY
)

stop_process INT "$evaluator_pid"; evaluator_pid=""
stop_process INT "$bag_pid"; bag_pid=""
stop_process TERM "$tracker_pid"; tracker_pid=""
stop_process TERM "$detector_pid"; detector_pid=""
stop_process TERM "$isaac_pid"; isaac_pid=""
stop_process TERM "$guard_pid"; guard_pid=""
trap - EXIT INT TERM

/usr/bin/python3 - "$RUN_DIR/process_exit_codes.json" <<'PY'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({"capture_completed": True, "shutdown": "intentional_owned_pids_only"}, indent=2) + "\n", encoding="utf-8")
PY

set +e
"$ANALYZER" run --run-dir "$RUN_DIR" --capture-start-ns "$capture_start_ns" --capture-end-ns "$capture_end_ns"
analysis_status=$?
set -e
if [[ ! -f "$RUN_DIR/summary.json" ]]; then
    echo "ERROR: benchmark summary was not produced." >&2
    exit 14
fi
"$ANALYZER" verify --run-dir "$RUN_DIR"
echo "SINGLE_MOTION_RUN_DIR=$RUN_DIR"
echo "SINGLE_MOTION_SUMMARY=$RUN_DIR/summary.json"
exit "$analysis_status"
