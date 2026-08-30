#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
TRAIN_PYTHON="$PROJECT_ROOT/.venvs/train/bin/python"
DR_SPAAM_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/2D_lidar_person_detection/dr_spaam"
DR_SPAAM_ROS2_ROOT="$PROJECT_ROOT/github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2"
DR_SPAAM_NODE="$DR_SPAAM_ROS2_ROOT/dr_spaam_ros2/dr_spaam_w_score_ros.py"
EVALUATOR="$ROS_WS/src/semantic_nav_gazebo/scripts/dr_spaam_detection_evaluator.py"
CHECKPOINT="$DR_SPAAM_ROS2_ROOT/model_weight/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
DURATION_SEC="${DR_SPAAM_SMOKE_DURATION_SEC:-75}"
DOMAIN_ID="${DR_SPAAM_SMOKE_DOMAIN_ID:-79}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${DR_SPAAM_SMOKE_RUN_DIR:-$PROJECT_ROOT/runs/dr_spaam_isaac_smoke/$RUN_ID}"

for required in \
    /opt/ros/humble/setup.bash \
    "$ROS_WS/install/setup.bash" \
    "$TRAIN_PYTHON" \
    "$DR_SPAAM_NODE" \
    "$EVALUATOR" \
    "$CHECKPOINT"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required DR-SPAAM smoke input is missing: $required" >&2
        exit 1
    fi
done
if ! awk -v value="$DURATION_SEC" 'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value >= 10) }'; then
    echo "ERROR: DR_SPAAM_SMOKE_DURATION_SEC must be at least 10 seconds." >&2
    exit 2
fi
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]] || (( DOMAIN_ID > 232 )); then
    echo "ERROR: DR_SPAAM_SMOKE_DOMAIN_ID must be an integer from 0 through 232." >&2
    exit 2
fi

mkdir -p "$RUN_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$DR_SPAAM_ROOT:$DR_SPAAM_ROS2_ROOT:${PYTHONPATH:-}"

existing_cmd_vel="$(ros2 topic info /cmd_vel 2>/dev/null || true)"
if [[ "$existing_cmd_vel" =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
    echo "ERROR: /cmd_vel already has a publisher in ROS_DOMAIN_ID=$DOMAIN_ID:" >&2
    echo "$existing_cmd_vel" >&2
    exit 5
fi

detector_pid=""
evaluator_pid=""
bag_pid=""
preflight_pid=""
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
    stop_process TERM "$preflight_pid"
    stop_process INT "$bag_pid"
    stop_process TERM "$evaluator_pid"
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

/usr/bin/python3 "$EVALUATOR" --ros-args \
    -p output_dir:="$RUN_DIR/evaluation" \
    -p match_threshold:=0.5 \
    -p target_frame:=base_link \
    >"$RUN_DIR/evaluator.log" 2>&1 &
evaluator_pid=$!

ros2 bag record -o "$RUN_DIR/rosbag" \
    /scan_01 /scan_02 /scan_merged \
    /pedestrian_ground_truth \
    /dr_spaam_detections /dr_spaam_detections_scored /dr_spaam_rviz \
    /odom /tf /tf_static /clock /isaac/reset_event \
    >"$RUN_DIR/rosbag.log" 2>&1 &
bag_pid=$!

(
    for _ in $(seq 1 300); do
        if timeout 2s ros2 topic echo --once /scan_merged \
            --qos-reliability best_effort >/dev/null 2>&1; then
            # This project-owned pose is the existing custom-scene pedestrian
            # avoidance test spawn.  The robot remains stationary there while
            # one generated patrol crosses laterally along y~=9.475 m.
            ros2 topic pub --once /isaac/reset_pose \
                geometry_msgs/msg/PoseStamped \
                "{header: {frame_id: map}, pose: {position: {x: 6.0, y: 9.75, z: 0.01}, orientation: {w: 1.0}}}" \
                >/dev/null 2>&1
            {
                echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
                for topic in /scan_01 /scan_02 /scan_merged /pedestrian_ground_truth /odom /cmd_vel /dr_spaam_detections_scored; do
                    echo "TOPIC $topic"
                    ros2 topic info "$topic" -v 2>&1 || true
                done
            } >"$RUN_DIR/runtime_topic_info.txt"
            exit 0
        fi
        sleep 1
    done
    echo "Timed out waiting for /scan_merged" >"$RUN_DIR/runtime_topic_info.txt"
) &
preflight_pid=$!

echo "DR_SPAAM_SMOKE_RUN_DIR=$RUN_DIR"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Detector PID=$detector_pid; evaluator PID=$evaluator_pid; rosbag PID=$bag_pid"

export ISAAC_ROS_DOMAIN_ID="$DOMAIN_ID"
export ISAAC_SCENE=custom
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_PEDESTRIAN_COUNT=3
export ISAAC_PEDESTRIAN_SEED=7
export ISAAC_PEDESTRIAN_SPEED=1.0
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off
export ISAAC_ROBOT_PHYSICS=0
export ISAAC_LIDAR_MODE=physx
export ISAAC_LIDAR_RATE_HZ=15
export ISAAC_LIDAR_SAMPLE_COUNT=2000

set +e
"$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh" \
    --headless --duration "$DURATION_SEC" \
    >"$RUN_DIR/isaac.log" 2>&1
isaac_status=$?
set -e
sleep 2
cleanup
trap - EXIT INT TERM

echo "$isaac_status" >"$RUN_DIR/isaac_exit_code.txt"
if [[ -f "$RUN_DIR/evaluation/summary.json" ]]; then
    echo "DR_SPAAM_SMOKE_SUMMARY=$RUN_DIR/evaluation/summary.json"
else
    echo "ERROR: evaluator summary was not produced." >&2
    exit 8
fi
exit "$isaac_status"
