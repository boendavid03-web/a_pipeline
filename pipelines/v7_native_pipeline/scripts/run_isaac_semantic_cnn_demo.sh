#!/usr/bin/env bash
# Run the existing Isaac scene with the fixed-dual SemanticCNN controller.
# The simulator, sensors, goal picker, and evaluator remain owned by their
# existing launchers; this wrapper only selects the CNN controller interface.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
ISAAC_LAUNCHER="$PROJECT_ROOT/isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_DOMAIN_ID
export ISAAC_ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ISAAC_SCENE=custom
export ISAAC_CUSTOM_SCENE_USD="$PROJECT_ROOT/isaac_sim/scenes/a_pipeline_eng_lobby.usda"
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_ROBOT_PHYSICS="${ISAAC_ROBOT_PHYSICS:-1}"
export ISAAC_ROBOT_COLLISION_PROTECTION="${ISAAC_ROBOT_COLLISION_PROTECTION:-1}"
export ISAAC_PEDESTRIAN_COUNT="${ISAAC_PEDESTRIAN_COUNT:-19}"
export ISAAC_PEDESTRIAN_SEED="${ISAAC_PEDESTRIAN_SEED:-7}"
export ISAAC_PEDESTRIAN_SPEED="${ISAAC_PEDESTRIAN_SPEED:-1.0}"
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE="${ISAAC_PEDESTRIAN_AVOIDANCE_MODE:-gentle}"
export ISAAC_LIDAR_MODE="${ISAAC_LIDAR_MODE:-physx}"
export ISAAC_LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-15}"
export ISAAC_LIDAR_SAMPLE_COUNT="${ISAAC_LIDAR_SAMPLE_COUNT:-2000}"

MODEL="${SEMANTIC_CNN_MODEL:-$PROJECT_ROOT/runs/20260808_gazebo_play/training/semantic_cnn_formal_auto_teacher/20260828_000241_semantic_cnn_native_cmd_51epoch/semantic_cnn_native_cmd_best_dev.pth}"
MODEL_CODE="${SEMANTIC_CNN_MODEL_CODE:-$(dirname "$MODEL")/model_code_scripts}"
MAP_YAML="${ISAAC_DEMO_MAP_YAML:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml}"
SEMANTIC_LABEL="${ISAAC_DEMO_SEMANTIC_LABEL:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/label.png}"

for required in "$ISAAC_LAUNCHER" "$MODEL" "$MODEL_CODE/model.py" "$MAP_YAML" "$SEMANTIC_LABEL"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required SemanticCNN/Isaac input is missing: $required" >&2
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

# Avoid using a stale ROS 2 graph cache from an earlier Isaac run during the
# readiness check below.
ros2 daemon stop >/dev/null 2>&1 || true

run_tag="$(date +%Y%m%d_%H%M%S)"
output_root="$PROJECT_ROOT/runs/isaac_custom_semantic_cnn_demo/$run_tag"
mkdir -p "$output_root"
evaluation_dir="$output_root/evaluation"
trace_path="$output_root/trajectory.csv"
isaac_log="$output_root/isaac.log"

echo "Run output: $output_root"
echo "Evaluation: $evaluation_dir"
echo "Trajectory: $trace_path"
echo "Isaac log: $isaac_log"

isaac_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ "$isaac_pid" =~ ^[0-9]+$ ]]; then
        kill -TERM -- "-$isaac_pid" 2>/dev/null || true
        wait "$isaac_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting the existing Isaac scene for SemanticCNN (ROS domain $ROS_DOMAIN_ID)."
setsid "$ISAAC_LAUNCHER" >"$isaac_log" 2>&1 &
isaac_pid=$!

ready=0
for _ in $(seq 1 180); do
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        echo "ERROR: Isaac exited before its ROS topics became ready. See $isaac_log" >&2
        exit 1
    fi
    topics="$(ros2 topic list 2>/dev/null || true)"
    if grep -qx '/scan_01' <<<"$topics" \
        && grep -qx '/scan_02' <<<"$topics" \
        && grep -qx '/odom' <<<"$topics" \
        && grep -qx '/isaac/actuation_state' <<<"$topics" \
        && grep -qx '/pedestrian_ground_truth' <<<"$topics"; then
        ready=1
        break
    fi
    sleep 1
done
if (( ! ready )); then
    echo "ERROR: Isaac ROS topics were not ready within 180 seconds. See $isaac_log" >&2
    exit 1
fi

echo "Isaac topics ready; starting SemanticCNN."
set +e
ros2 launch semantic_nav_gazebo semantic_cnn_fixed_dual_start_goal_demo.launch.py \
    start_bringup:=false \
    start_aux_map:=false \
    gui:=false \
    start_rviz:=false \
    enable_goal_picker:=true \
    auto_set_initial_goal:=false \
    semantic_cnn_model:="$MODEL" \
    semantic_cnn_model_code:="$MODEL_CODE" \
    map_yaml:="$MAP_YAML" \
    semantic_label:="$SEMANTIC_LABEL" \
    device:="${SEMANTIC_CNN_DEVICE:-cuda}" \
    semantic_cnn_pool_mode:="${SEMANTIC_CNN_POOL_MODE:-global_virtual_angle_80}" \
    cmd_vel_topic:=/cmd_vel \
    actuation_decision_topic:=/semantic_cnn/actuation_decision \
    simulator_actuation_topic:=/isaac/actuation_state \
    inference_metrics_topic:=/navigation_evaluation/inference_metrics \
    max_linear:="${SEMANTIC_CNN_MAX_LINEAR:-0.99}" \
    max_angular:="${SEMANTIC_CNN_MAX_ANGULAR:-1.99}" \
    lidar_range_max:="${SEMANTIC_CNN_LIDAR_RANGE_MAX:-50.0}" \
    pool_range_max:="${SEMANTIC_CNN_POOL_RANGE_MAX:-8.0}" \
    scan_timeout:="${SEMANTIC_CNN_SCAN_TIMEOUT:-0.75}" \
    subgoal_timeout:="${SEMANTIC_CNN_SUBGOAL_TIMEOUT:-0.50}" \
    odom_timeout:="${SEMANTIC_CNN_ODOM_TIMEOUT:-0.30}" \
    front_stop_distance:="${SEMANTIC_CNN_FRONT_STOP_DISTANCE:-0.50}" \
    goal_tolerance:="${SEMANTIC_CNN_GOAL_TOLERANCE:-0.35}" \
    lookahead:=1.0 \
    inflate_radius:=0.4 \
    visualize:=false \
    publish_debug_images:=false \
    record_trace:=true \
    trace_path:="$trace_path" \
    evaluate_episode:=true \
    evaluation_output_dir:="$evaluation_dir" \
    evaluation_timeout_sec:="${SEMANTIC_CNN_EVALUATION_TIMEOUT_SEC:-86400.0}" \
    evaluation_multi_episode:=true \
    experiment_scene_id:="isaac_custom_semantic_cnn_${run_tag}" \
    scene_file:="$PROJECT_ROOT/isaac_sim/scenes/a_pipeline_eng_lobby.usda" \
    pedestrian_count:="$ISAAC_PEDESTRIAN_COUNT" \
    pedestrian_seed:="$ISAAC_PEDESTRIAN_SEED"
status=$?
set -e
exit "$status"
