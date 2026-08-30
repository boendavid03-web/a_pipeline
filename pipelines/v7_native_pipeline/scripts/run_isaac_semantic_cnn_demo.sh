#!/usr/bin/env bash
# Run the existing Isaac scene with the fixed-dual SemanticCNN controller.
# The simulator, sensors, goal picker, and evaluator remain owned by their
# existing launchers; this wrapper only selects the CNN controller interface.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
ISAAC_LAUNCHER="$PROJECT_ROOT/isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh"
VIDEO_RENDERER="$PROJECT_ROOT/pipelines/v7_native_pipeline/scripts/render_fixed_four_evaluation_video.py"

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
ONLINE_S3NET="${SEMANTIC_CNN_USE_ONLINE_S3NET:-false}"
S3NET_DIR="$PROJECT_ROOT/runs/20260808_gazebo_play/training/s3net_formal_auto_teacher/20260827_234825_s3net_native_stats_81epoch"
S3NET_MODEL="${SEMANTIC_CNN_S3NET_MODEL:-$S3NET_DIR/s3net_native_stats_best_dev.pth}"
S3NET_MODEL_CODE="${SEMANTIC_CNN_S3NET_MODEL_CODE:-$(dirname "$S3NET_MODEL")/model_code_scripts}"
S3NET_STATS_JSON="${SEMANTIC_CNN_S3NET_STATS_JSON:-$(dirname "$S3NET_MODEL")/s3net_native_lidar_train_stats.json}"
S3NET_ENFORCE_LAYOUT="${SEMANTIC_CNN_S3NET_ENFORCE_MESSAGE_LAYOUT:-false}"
RVIZ_ENABLED="${ISAAC_DEMO_RVIZ:-false}"
FIXED_TEST="${SEMANTIC_CNN_FIXED_TEST:-false}"
FIXED_GOALS_FILE="${SEMANTIC_CNN_FIXED_GOALS_FILE:-$PROJECT_ROOT/configs/evaluation/fixed_four_goals.yaml}"
RECORD_VIDEO="${SEMANTIC_CNN_RECORD_VIDEO:-false}"
case "${FIXED_TEST,,}" in
    1|true|yes|on) FIXED_TEST=true; GOAL_PICKER_ENABLED=false ;;
    0|false|no|off) FIXED_TEST=false; GOAL_PICKER_ENABLED=true ;;
    *) echo "ERROR: SEMANTIC_CNN_FIXED_TEST must be a boolean value." >&2; exit 2 ;;
esac
case "${RECORD_VIDEO,,}" in
    1|true|yes|on) RECORD_VIDEO=true ;;
    0|false|no|off) RECORD_VIDEO=false ;;
    *) echo "ERROR: SEMANTIC_CNN_RECORD_VIDEO must be a boolean value." >&2; exit 2 ;;
esac
case "${ONLINE_S3NET,,}" in
    1|true|yes|on) ONLINE_S3NET=true ;;
    0|false|no|off) ONLINE_S3NET=false ;;
    *) echo "ERROR: SEMANTIC_CNN_USE_ONLINE_S3NET must be a boolean value." >&2; exit 2 ;;
esac
case "${S3NET_ENFORCE_LAYOUT,,}" in
    1|true|yes|on) S3NET_ENFORCE_LAYOUT=true ;;
    0|false|no|off) S3NET_ENFORCE_LAYOUT=false ;;
    *) echo "ERROR: SEMANTIC_CNN_S3NET_ENFORCE_MESSAGE_LAYOUT must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$RECORD_VIDEO" == "true" && "$FIXED_TEST" != "true" ]]; then
    echo "ERROR: SEMANTIC_CNN_RECORD_VIDEO=true requires SEMANTIC_CNN_FIXED_TEST=true." >&2
    exit 2
fi

for required in "$ISAAC_LAUNCHER" "$MODEL" "$MODEL_CODE/model.py" "$MAP_YAML" "$SEMANTIC_LABEL"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required SemanticCNN/Isaac input is missing: $required" >&2
        exit 1
    fi
done
if [[ "$ONLINE_S3NET" == "true" ]]; then
    for required in "$S3NET_MODEL" "$S3NET_MODEL_CODE/model.py" "$S3NET_STATS_JSON"; do
        if [[ ! -f "$required" ]]; then
            echo "ERROR: required online S3-Net input is missing: $required" >&2
            exit 1
        fi
    done
fi
if [[ "$FIXED_TEST" == "true" && ! -f "$FIXED_GOALS_FILE" ]]; then
    echo "ERROR: fixed goals file is missing: $FIXED_GOALS_FILE" >&2
    exit 1
fi

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
output_root="${SEMANTIC_CNN_DEMO_OUTPUT_DIR:-$PROJECT_ROOT/runs/isaac_custom_semantic_cnn_demo/$run_tag}"
if ! mkdir "$output_root"; then
    echo "ERROR: refusing to overwrite existing run directory: $output_root" >&2
    exit 1
fi
evaluation_dir="$output_root/evaluation"
video_dir="$evaluation_dir/video"
trace_path="$output_root/trajectory.csv"
isaac_log="$output_root/isaac.log"

echo "Run output: $output_root"
echo "Evaluation: $evaluation_dir"
echo "Trajectory: $trace_path"
echo "Isaac log: $isaac_log"
echo "Semantic input: $([[ "$ONLINE_S3NET" == "true" ]] && echo online_s3net || echo static_map)"
if [[ "$RECORD_VIDEO" == "true" ]]; then
    echo "Video: $video_dir/evaluation_video.mp4"
fi

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
    start_rviz:="$RVIZ_ENABLED" \
    enable_goal_picker:="$GOAL_PICKER_ENABLED" \
    fixed_test:="$FIXED_TEST" \
    fixed_goals_file:="$FIXED_GOALS_FILE" \
    fixed_test_readiness_timeout_sec:="${SEMANTIC_CNN_FIXED_TEST_READINESS_TIMEOUT_SEC:-60.0}" \
    fixed_test_auto_shutdown_delay_sec:="${SEMANTIC_CNN_FIXED_TEST_AUTO_SHUTDOWN_DELAY_SEC:-2.0}" \
    fixed_test_max_linear:="${SEMANTIC_CNN_FIXED_TEST_MAX_LINEAR:-0.8}" \
    fixed_test_max_angular:="${SEMANTIC_CNN_FIXED_TEST_MAX_ANGULAR:-1.8}" \
    record_video:="$RECORD_VIDEO" \
    video_output_dir:="$video_dir" \
    video_simulator_name:=isaac \
    auto_set_initial_goal:=false \
    semantic_cnn_model:="$MODEL" \
    semantic_cnn_model_code:="$MODEL_CODE" \
    use_online_s3net:="$ONLINE_S3NET" \
    s3net_model:="$S3NET_MODEL" \
    s3net_model_code:="$S3NET_MODEL_CODE" \
    s3net_stats_json:="$S3NET_STATS_JSON" \
    s3net_sampling_strategy:="${SEMANTIC_CNN_S3NET_SAMPLING_STRATEGY:-contract}" \
    s3net_sampling_seed:="${SEMANTIC_CNN_S3NET_SAMPLING_SEED:-1337}" \
    s3net_enforce_message_layout:="$S3NET_ENFORCE_LAYOUT" \
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
    inflate_radius:="${ISAAC_DEMO_INFLATE_RADIUS:-0.45}" \
    visualize:=false \
    publish_debug_images:=false \
    record_trace:="${SEMANTIC_CNN_RECORD_TRACE:-true}" \
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
if [[ "$RECORD_VIDEO" == "true" && -f "$evaluation_dir/session_summary.json" ]]; then
    mkdir -p "$video_dir"
    if ! python3 "$VIDEO_RENDERER" \
        --evaluation-dir "$evaluation_dir" \
        --map-yaml "$MAP_YAML" \
        --capture-dir "$video_dir/sync" \
        --output-mp4 "$video_dir/evaluation_video.mp4" \
        --save-episode-screenshots \
        >"$video_dir/render.log" 2>&1; then
        echo "ERROR: video rendering failed. See $video_dir/render.log" >&2
        exit 1
    fi
    echo "FIXED_FOUR_VIDEO_READY path=$video_dir/evaluation_video.mp4"
fi
exit "$status"
