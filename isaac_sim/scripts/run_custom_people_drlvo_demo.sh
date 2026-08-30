#!/usr/bin/env bash
# One-command Isaac engineering-lobby + walking people + DRL-VO demo.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
ISAAC_LAUNCHER="$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh"
TASK_ROOT="${TASK_ROOT:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1}"
MODEL="${DRL_VO_MODEL:-$TASK_ROOT/training/drl_vo/base_bc/20260727_114455/checkpoints/best.pt}"
MAP_YAML="${ISAAC_DEMO_MAP_YAML:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml}"
SEMANTIC_LABEL="${ISAAC_DEMO_SEMANTIC_LABEL:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/label.png}"
DEFAULT_PEDESTRIAN_SLAM_MAP="$ROS_WS/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml"
PEOPLE_ROUTE_VALIDATOR="$SCRIPT_DIR/validate_custom_people_routes.py"
TRAJECTORY_VISUALIZER="$PROJECT_ROOT/pipelines/v7_native_pipeline/scripts/visualize_auto_capture_trajectories.py"
VIDEO_RENDERER="$PROJECT_ROOT/pipelines/v7_native_pipeline/scripts/render_fixed_four_evaluation_video.py"
GAZEBO_LOBBY_WORLD="$ROS_WS/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_DOMAIN_ID
export ISAAC_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_SCENE=custom
export ISAAC_ENABLE_PEOPLE=1
export ISAAC_ROBOT_PHYSICS="${ISAAC_ROBOT_PHYSICS:-1}"
demo_control_mode="${ISAAC_DEMO_CONTROL_MODE:-policy}"
case "${demo_control_mode,,}" in
    policy|teleop) demo_control_mode="${demo_control_mode,,}" ;;
    *) echo "ERROR: ISAAC_DEMO_CONTROL_MODE must be policy or teleop." >&2; exit 2 ;;
esac
if [[ "$demo_control_mode" == "teleop" ]]; then
    export ISAAC_MANUAL_EPISODE_EVENTS=1
else
    export ISAAC_MANUAL_EPISODE_EVENTS=0
fi
export ISAAC_PEDESTRIAN_COUNT="${ISAAC_PEDESTRIAN_COUNT:-19}"
export ISAAC_PEDESTRIAN_SEED="${ISAAC_PEDESTRIAN_SEED:-7}"
export ISAAC_PEDESTRIAN_SPEED="${ISAAC_PEDESTRIAN_SPEED:-1.0}"
if [[ ! "$ISAAC_PEDESTRIAN_COUNT" =~ ^-?[0-9]+$ ]] \
    || (( ISAAC_PEDESTRIAN_COUNT < -1 || ISAAC_PEDESTRIAN_COUNT > 50 )); then
    echo "ERROR: ISAAC_PEDESTRIAN_COUNT must be -1 or an integer from 0 through 50." >&2
    exit 2
fi
if [[ ! "$ISAAC_PEDESTRIAN_SEED" =~ ^[0-9]+$ ]] \
    || (( ISAAC_PEDESTRIAN_SEED > 4294967295 )); then
    echo "ERROR: ISAAC_PEDESTRIAN_SEED must be an integer from 0 through 4294967295." >&2
    exit 2
fi
if ! awk -v value="$ISAAC_PEDESTRIAN_SPEED" \
    'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
    echo "ERROR: ISAAC_PEDESTRIAN_SPEED must be positive." >&2
    exit 2
fi
export ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML="${ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML:-$DEFAULT_PEDESTRIAN_SLAM_MAP}"
# Keep route planning conservative while using a smaller, observation-only
# runtime boundary for genuine obstacle incursions.
export ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M:-0.55}"
export ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M:-0.20}"
export ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M="${ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M:-1.0}"
export ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M="${ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M:-0.5}"
# The policy consumes the two raw Gazebo scans, each 2000 beams at 15 Hz.  The
# 360-beam topic is only a merged visualization product and must not be used as
# the policy's raw-input contract.  PhysX remains range-only; RTX is opt-in.
export ISAAC_LIDAR_MODE="${ISAAC_LIDAR_MODE:-physx}"
case "${ISAAC_LIDAR_MODE,,}" in
    physx|rtx) export ISAAC_LIDAR_MODE="${ISAAC_LIDAR_MODE,,}" ;;
    *) echo "ERROR: ISAAC_LIDAR_MODE must be physx or rtx." >&2; exit 2 ;;
esac
export ISAAC_LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-15}"
if [[ -z "${ISAAC_LIDAR_SAMPLE_COUNT+x}" ]]; then
    if [[ "$ISAAC_LIDAR_MODE" == "physx" ]]; then
        export ISAAC_LIDAR_SAMPLE_COUNT=2000
    else
        export ISAAC_LIDAR_SAMPLE_COUNT=2000
    fi
else
    export ISAAC_LIDAR_SAMPLE_COUNT
fi
if [[ ! "$ISAAC_LIDAR_RATE_HZ" =~ ^[0-9]+$ ]] \
    || (( ISAAC_LIDAR_RATE_HZ < 1 || ISAAC_LIDAR_RATE_HZ > 30 )); then
    echo "ERROR: ISAAC_LIDAR_RATE_HZ must be an integer from 1 through 30." >&2
    exit 2
fi
if (( ISAAC_LIDAR_RATE_HZ != 15 )); then
    echo "ERROR: DRL-VO dual-LiDAR input is fixed at 15 Hz; requested ${ISAAC_LIDAR_RATE_HZ} Hz." >&2
    echo "Remove ISAAC_LIDAR_RATE_HZ or set ISAAC_LIDAR_RATE_HZ=15." >&2
    exit 2
fi
if [[ ! "$ISAAC_LIDAR_SAMPLE_COUNT" =~ ^[0-9]+$ ]] \
    || (( ISAAC_LIDAR_SAMPLE_COUNT < 90 || ISAAC_LIDAR_SAMPLE_COUNT > 4096 )); then
    echo "ERROR: ISAAC_LIDAR_SAMPLE_COUNT must be an integer from 90 through 4096." >&2
    exit 2
fi
if (( ISAAC_LIDAR_SAMPLE_COUNT != 2000 )); then
    echo "ERROR: DRL-VO dual-LiDAR input is fixed at 2000 beams per sensor; requested ${ISAAC_LIDAR_SAMPLE_COUNT}." >&2
    echo "Remove ISAAC_LIDAR_SAMPLE_COUNT or set ISAAC_LIDAR_SAMPLE_COUNT=2000." >&2
    exit 2
fi
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE="${ISAAC_PEDESTRIAN_AVOIDANCE_MODE:-gentle}"
export ISAAC_PEDESTRIAN_SOCIAL_MODE="${ISAAC_PEDESTRIAN_SOCIAL_MODE:-legacy}"
export ISAAC_PEDESTRIAN_SOCIAL_MODE="${ISAAC_PEDESTRIAN_SOCIAL_MODE,,}"
case "$ISAAC_PEDESTRIAN_SOCIAL_MODE" in
    legacy|gazebo_social) ;;
    *) echo "ERROR: ISAAC_PEDESTRIAN_SOCIAL_MODE must be legacy or gazebo_social." >&2; exit 2 ;;
esac
if (( ISAAC_PEDESTRIAN_COUNT == 0 )); then
    export ISAAC_ENABLE_PEOPLE=0
    export ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off
fi
if (( ISAAC_PEDESTRIAN_COUNT == -1 )); then
    isaac_expected_pedestrians=15
else
    isaac_expected_pedestrians="$ISAAC_PEDESTRIAN_COUNT"
fi
demo_auto_goal="${ISAAC_DEMO_AUTO_GOAL:-false}"
demo_goal_picker="${ISAAC_DEMO_GOAL_PICKER:-true}"
demo_fixed_test="${ISAAC_DEMO_FIXED_TEST:-false}"
demo_fixed_goals_file="${ISAAC_DEMO_FIXED_GOALS_FILE:-$PROJECT_ROOT/configs/evaluation/fixed_four_goals.yaml}"
case "${demo_fixed_test,,}" in
    1|true|yes|on) demo_fixed_test=true ;;
    0|false|no|off) demo_fixed_test=false ;;
    *) echo "ERROR: ISAAC_DEMO_FIXED_TEST must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$demo_fixed_test" == "true" ]]; then
    demo_auto_goal=false
    demo_goal_picker=false
    if [[ ! -f "$demo_fixed_goals_file" ]]; then
        echo "ERROR: fixed goals file is missing: $demo_fixed_goals_file" >&2
        exit 1
    fi
fi
if [[ "$demo_control_mode" == "teleop" && "${ISAAC_DEMO_AUTO_CAPTURE:-0}" == "1" ]]; then
    echo "ERROR: ISAAC_DEMO_AUTO_CAPTURE requires ISAAC_DEMO_CONTROL_MODE=policy." >&2
    exit 2
fi
demo_capture_duration_sec="${ISAAC_DEMO_CAPTURE_DURATION_SEC:-1800.0}"
if ! awk -v value="$demo_capture_duration_sec" \
    'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
    echo "ERROR: ISAAC_DEMO_CAPTURE_DURATION_SEC must be a positive number." >&2
    exit 2
fi
# rclpy infers an override without a decimal point as INTEGER, while the
# scheduler declares this parameter as DOUBLE. Normalize common inputs such
# as `180` to unambiguously floating-point YAML text.
demo_capture_duration_sec="$(
    awk -v value="$demo_capture_duration_sec" 'BEGIN { printf "%.9f", value }'
)"
demo_verify_navigation="${ISAAC_DEMO_VERIFY_NAVIGATION:-false}"
case "${demo_verify_navigation,,}" in
    1|true|yes|on) demo_verify_navigation=true ;;
    0|false|no|off) demo_verify_navigation=false ;;
    *) echo "ERROR: ISAAC_DEMO_VERIFY_NAVIGATION must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$demo_control_mode" == "teleop" && "$demo_verify_navigation" == "true" ]]; then
    echo "ERROR: ISAAC_DEMO_VERIFY_NAVIGATION requires ISAAC_DEMO_CONTROL_MODE=policy." >&2
    exit 2
fi
demo_exit_after_verify="${ISAAC_DEMO_EXIT_AFTER_VERIFY:-true}"
case "${demo_exit_after_verify,,}" in
    1|true|yes|on) demo_exit_after_verify=true ;;
    0|false|no|off) demo_exit_after_verify=false ;;
    *) echo "ERROR: ISAAC_DEMO_EXIT_AFTER_VERIFY must be a boolean value." >&2; exit 2 ;;
esac
demo_record_trace="${ISAAC_DEMO_RECORD_TRACE:-true}"
case "${demo_record_trace,,}" in
    1|true|yes|on) demo_record_trace=true ;;
    0|false|no|off) demo_record_trace=false ;;
    *) echo "ERROR: ISAAC_DEMO_RECORD_TRACE must be a boolean value." >&2; exit 2 ;;
esac
demo_record_video="${ISAAC_DEMO_RECORD_VIDEO:-false}"
case "${demo_record_video,,}" in
    1|true|yes|on) demo_record_video=true ;;
    0|false|no|off) demo_record_video=false ;;
    *) echo "ERROR: ISAAC_DEMO_RECORD_VIDEO must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$demo_record_video" == "true" && "$demo_fixed_test" != "true" ]]; then
    echo "ERROR: ISAAC_DEMO_RECORD_VIDEO=true requires ISAAC_DEMO_FIXED_TEST=true." >&2
    exit 2
fi
# Evaluation is on by default for policy runs; callers that only need an
# interactive demo can still select ISAAC_DEMO_EVALUATE=false explicitly.
demo_evaluate="${ISAAC_DEMO_EVALUATE:-true}"
case "${demo_evaluate,,}" in
    1|true|yes|on) demo_evaluate=true ;;
    0|false|no|off) demo_evaluate=false ;;
    *) echo "ERROR: ISAAC_DEMO_EVALUATE must be a boolean value." >&2; exit 2 ;;
esac
if [[ "${ISAAC_DEMO_AUTO_CAPTURE:-0}" == "1" ]]; then
    if [[ -z "${ISAAC_DEMO_AUTO_GOAL+x}" ]]; then
        demo_auto_goal=false
    fi
    if [[ -z "${ISAAC_DEMO_GOAL_PICKER+x}" ]]; then
        demo_goal_picker=false
    fi
fi
case "${demo_auto_goal,,}" in
    1|true|yes|on) demo_auto_goal=true ;;
    0|false|no|off) demo_auto_goal=false ;;
    *) echo "ERROR: ISAAC_DEMO_AUTO_GOAL must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$demo_auto_goal" == "true" && -z "${ISAAC_DEMO_GOAL_PICKER+x}" ]]; then
    # A fixed initial goal and a startup picker are competing goal sources.
    demo_goal_picker=false
fi
case "${demo_goal_picker,,}" in
    1|true|yes|on) demo_goal_picker=true ;;
    0|false|no|off) demo_goal_picker=false ;;
    *) echo "ERROR: ISAAC_DEMO_GOAL_PICKER must be a boolean value." >&2; exit 2 ;;
esac
if [[ "$demo_control_mode" == "policy" \
    && "$demo_auto_goal" == "false" && "$demo_goal_picker" == "false" \
    && "$demo_fixed_test" != "true" \
    && "${ISAAC_DEMO_AUTO_CAPTURE:-0}" != "1" ]]; then
    echo "ERROR: no goal source is enabled; enable ISAAC_DEMO_GOAL_PICKER or ISAAC_DEMO_AUTO_GOAL." >&2
    exit 2
fi
if [[ "$demo_control_mode" == "policy" \
    && "$demo_goal_picker" == "true" && -z "${DISPLAY:-}" ]]; then
    echo "ERROR: the automatic goal-picker window requires DISPLAY; use a desktop terminal." >&2
    echo "For unattended capture, set ISAAC_DEMO_AUTO_CAPTURE=1 instead." >&2
    exit 2
fi

# Isaac and Gazebo use the same simulation-time timestamps, but the 25-person
# Isaac scene can have more scheduling jitter than Gazebo.  These bounds keep
# the ten-frame DRL-VO history intact without accepting multi-second stale
# control inputs.  Every value remains explicitly overridable for experiments.
if [[ "$ISAAC_LIDAR_MODE" == "physx" ]]; then
    default_scan_timeout=0.75
    default_input_timeout=0.50
else
    default_scan_timeout=2.00
    default_input_timeout=1.00
fi
demo_scan_timeout="${ISAAC_DEMO_SCAN_TIMEOUT:-$default_scan_timeout}"
demo_odom_timeout="${ISAAC_DEMO_ODOM_TIMEOUT:-$default_input_timeout}"
demo_subgoal_timeout="${ISAAC_DEMO_SUBGOAL_TIMEOUT:-$default_input_timeout}"
demo_final_goal_timeout="${ISAAC_DEMO_FINAL_GOAL_TIMEOUT:-$default_input_timeout}"
demo_pedestrian_truth_timeout="${ISAAC_DEMO_PEDESTRIAN_TRUTH_TIMEOUT:-$default_input_timeout}"
demo_actuation_source_timeout="${ISAAC_DEMO_ACTUATION_SOURCE_TIMEOUT:-180.0}"
for timeout_spec in \
    "ISAAC_DEMO_SCAN_TIMEOUT=$demo_scan_timeout" \
    "ISAAC_DEMO_ODOM_TIMEOUT=$demo_odom_timeout" \
    "ISAAC_DEMO_SUBGOAL_TIMEOUT=$demo_subgoal_timeout" \
    "ISAAC_DEMO_FINAL_GOAL_TIMEOUT=$demo_final_goal_timeout" \
    "ISAAC_DEMO_PEDESTRIAN_TRUTH_TIMEOUT=$demo_pedestrian_truth_timeout" \
    "ISAAC_DEMO_ACTUATION_SOURCE_TIMEOUT=$demo_actuation_source_timeout"; do
    timeout_name="${timeout_spec%%=*}"
    timeout_value="${timeout_spec#*=}"
    if ! awk -v value="$timeout_value" \
        'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
        echo "ERROR: $timeout_name must be a positive number." >&2
        exit 2
    fi
done

# This launcher never kills stale simulator/controller processes.  An active
# Gazebo or Isaac run may belong to the user; the ownership checks below fail
# closed and ask for a different domain instead.
if [[ "${ISAAC_DEMO_CLEAN_STALE:-0}" == "1" ]]; then
    echo "ERROR: ISAAC_DEMO_CLEAN_STALE=1 is intentionally unsupported; stop only your own processes explicitly." >&2
    exit 4
fi

for required in /opt/ros/humble/setup.bash "$ROS_WS/install/setup.bash" "$MODEL" "$MAP_YAML" "$SEMANTIC_LABEL" "$ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML" "$PEOPLE_ROUTE_VALIDATOR" "$TRAJECTORY_VISUALIZER" "$GAZEBO_LOBBY_WORLD"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required demo input is missing: $required" >&2
        exit 1
    fi
done

# The lower-level Isaac launcher converts the Gazebo XML routes through the
# confirmed-free grid and validates that generated per-person configuration.
# The YAML here is a schema/asset template, not the runtime route source.

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID ISAAC_ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

# Restart discovery after stale-demo cleanup so readiness checks cannot use a
# cached graph from the previous Gazebo process.
ros2 daemon stop >/dev/null 2>&1 || true

# Do not launch into an already-owned ROS domain.  A topic-list-only readiness
# check can otherwise mistake a running Gazebo scene for the new Isaac scene
# if Isaac exits during its GPU or memory preflight.
domain_conflicts=()
for exclusive_topic in /scan_01 /scan_02 /odom /pedestrian_ground_truth /isaac/actuation_state; do
    topic_info="$(
        timeout 6s ros2 topic info "$exclusive_topic" \
            --no-daemon --spin-time 2.0 2>/dev/null || true
    )"
    publisher_count="$(
        awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2}' \
            <<<"$topic_info"
    )"
    if [[ "$publisher_count" =~ ^[0-9]+$ ]] && (( publisher_count > 0 )); then
        domain_conflicts+=("$exclusive_topic=$publisher_count")
    fi
done
if (( ${#domain_conflicts[@]} > 0 )); then
    echo "ERROR: ROS_DOMAIN_ID=$ROS_DOMAIN_ID is already owned by another simulator:" >&2
    printf '  %s\n' "${domain_conflicts[@]}" >&2
    echo "Stop that run, or launch Isaac with an unused ROS_DOMAIN_ID (for example 79)." >&2
    exit 4
fi

run_tag="$(date +%Y%m%d_%H%M%S)"
log_dir="${ISAAC_DEMO_OUTPUT_DIR:-$PROJECT_ROOT/runs/isaac_custom_drlvo_demo/$run_tag}"
if ! mkdir "$log_dir"; then
    echo "ERROR: refusing to overwrite existing run directory: $log_dir" >&2
    exit 1
fi
demo_trace_path="${ISAAC_DEMO_TRACE_PATH:-$log_dir/trajectory.csv}"
demo_evaluation_output_dir="${ISAAC_DEMO_EVALUATION_OUTPUT_DIR:-$log_dir/evaluation}"
demo_video_dir="$demo_evaluation_output_dir/video"

isaac_pid=""
policy_pid=""
bag_pid=""
scheduler_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ "$policy_pid" =~ ^[0-9]+$ ]]; then
        kill -INT -- "-$policy_pid" 2>/dev/null || true
    fi
    if [[ "$isaac_pid" =~ ^[0-9]+$ ]]; then
        kill -TERM -- "-$isaac_pid" 2>/dev/null || true
    fi
    if [[ "$scheduler_pid" =~ ^[0-9]+$ ]]; then
        kill -INT "$scheduler_pid" 2>/dev/null || true
    fi
    if [[ "$bag_pid" =~ ^[0-9]+$ ]]; then
        kill -INT -- "-$bag_pid" 2>/dev/null || true
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
    wait "${scheduler_pid:-}" 2>/dev/null || true
    wait "${bag_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting Isaac walking-people scene (ROS domain $ROS_DOMAIN_ID)..."
echo "Control mode: $demo_control_mode; LiDAR: $ISAAC_LIDAR_MODE ${ISAAC_LIDAR_SAMPLE_COUNT}x2 @ ${ISAAC_LIDAR_RATE_HZ} Hz; pedestrian social: $ISAAC_PEDESTRIAN_SOCIAL_MODE"
setsid "$ISAAC_LAUNCHER" "$@" >"$log_dir/isaac.log" 2>&1 &
isaac_pid=$!

ready=0
for _ in $(seq 1 180); do
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        echo "ERROR: Isaac exited before ROS topics became ready. See $log_dir/isaac.log" >&2
        exit 1
    fi
    topics="$(ros2 topic list 2>/dev/null || true)"
    if grep -qx '/scan_01' <<<"$topics" \
        && grep -qx '/scan_02' <<<"$topics" \
        && grep -qx '/scan_merged' <<<"$topics" \
        && grep -qx '/odom' <<<"$topics" \
        && grep -qx '/isaac/actuation_state' <<<"$topics" \
        && grep -qx '/pedestrian_ground_truth' <<<"$topics"; then
        ready=1
        break
    fi
    sleep 1
done
if (( ! ready )); then
    echo "ERROR: Isaac ROS contract was not ready within 180 seconds. See $log_dir/isaac.log" >&2
    exit 1
fi

# Reject a second simulator that joined this ROS domain after the initial
# stale-process cleanup.  Otherwise the policy could silently mix Gazebo and
# Isaac scans/odometry and produce a meaningless apparent demo result.
for exclusive_topic in /scan_01 /scan_02 /odom /pedestrian_ground_truth /isaac/actuation_state; do
    publisher_count="$(
        ros2 topic info "$exclusive_topic" 2>/dev/null \
            | awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2}'
    )"
    if [[ "$publisher_count" != "1" ]]; then
        echo "ERROR: expected one Isaac publisher on $exclusive_topic, found ${publisher_count:-unknown}." >&2
        echo "Another simulator is sharing ROS_DOMAIN_ID=$ROS_DOMAIN_ID; stop it or choose another domain." >&2
        exit 1
    fi
done

actuation_source="$(
    timeout "$demo_actuation_source_timeout" ros2 topic echo /isaac/actuation_state 2>/dev/null \
        | awk -F': ' '/^actual_velocity_source: (physx_rigid_body_api|fixed_tick_pose_difference)$/ {print $2; exit}' \
        | tr -d "'\"" \
        || true
)"
if grep -q '\[WAREHOUSE-ROBOT\] ERROR:' "$log_dir/isaac.log"; then
    echo "ERROR: Isaac failed while waiting for actual velocity telemetry. See $log_dir/isaac.log" >&2
    grep '\[WAREHOUSE-ROBOT\] ERROR:' "$log_dir/isaac.log" | tail -n 1 >&2
    exit 1
fi
if ! kill -0 "$isaac_pid" 2>/dev/null; then
    echo "ERROR: Isaac exited while waiting for actual velocity telemetry. See $log_dir/isaac.log" >&2
    tail -n 40 "$log_dir/isaac.log" >&2
    exit 1
fi
case "$actuation_source" in
    physx_rigid_body_api|fixed_tick_pose_difference) ;;
    *)
        echo "ERROR: Isaac actual velocity source is missing or invalid: ${actuation_source:-unavailable}." >&2
        exit 1
        ;;
esac

sensor_preflight_log="$log_dir/sensor_preflight.log"
if ! python3 "$SCRIPT_DIR/check_capture_ready.py" \
    --sensor-preflight --verify-lidar-rate --require-realtime-lidar \
    --timeout "${ISAAC_DEMO_SENSOR_PREFLIGHT_TIMEOUT:-180.0}" \
    >"$sensor_preflight_log" 2>&1; then
    echo "ERROR: Isaac sensor preflight failed. See $sensor_preflight_log" >&2
    sed -n '1,200p' "$sensor_preflight_log" >&2
    exit 1
fi
if ! kill -0 "$isaac_pid" 2>/dev/null; then
    echo "ERROR: Isaac exited during sensor preflight. See $log_dir/isaac.log" >&2
    exit 1
fi

if [[ "${ISAAC_DEMO_RECORD_BAG:-0}" == "1" || "${ISAAC_DEMO_AUTO_CAPTURE:-0}" == "1" ]]; then
    setsid ros2 bag record \
        -o "$log_dir/rosbag" \
        /scan /scan_01 /scan_02 /scan_merged \
        /odom /tf /tf_static /clock \
        /cmd_vel /cmd_vel_stamped \
        /drl_vo/raw_model_cmd /drl_vo/control_event /drl_vo/actuation_decision \
        /isaac/actuation_state /isaac/reset_pose /isaac/reset_event \
        /navigation_evaluation/inference_metrics \
        /pedestrian_ground_truth \
        /semantic_cnn/final_goal /semantic_cnn/local_subgoal /semantic_cnn/global_path \
        /data_collection/goal_accepted \
        /data_collection/episode_event \
        /data_collection/sensor_config \
        /data_collection/auto_capture_status \
        >"$log_dir/rosbag.log" 2>&1 &
    bag_pid=$!
fi

if [[ "$demo_control_mode" == "teleop" ]]; then
    echo "ISAAC_CUSTOM_TELEOP_READY scene=custom pedestrians=$isaac_expected_pedestrians seed=$ISAAC_PEDESTRIAN_SEED lidar=$ISAAC_LIDAR_MODE samples=$ISAAC_LIDAR_SAMPLE_COUNT"
    echo "Start keyboard control in another terminal with: ISAAC_ROS_DOMAIN_ID=$ROS_DOMAIN_ID bash isaac_sim/scripts/teleop_robot.sh"
    wait "$isaac_pid"
    exit $?
fi

echo "Isaac topics ready; starting DRL-VO base policy. Logs: $log_dir"
setsid ros2 launch semantic_nav_gazebo drl_vo_fixed_dual_start_goal_demo.launch.py \
    start_simulator:=false \
    policy_mode:=base \
    drl_vo_model:="$MODEL" \
    device:="${DRL_VO_DEVICE:-cuda}" \
    publish_policy_actions:=true \
    pedestrian_source:=oracle \
    oracle_pedestrian_velocity:=true \
    require_pedestrian_truth:=true \
    start_rviz:="${ISAAC_DEMO_RVIZ:-true}" \
    use_sim_time:=true \
    map_yaml:="$MAP_YAML" \
    semantic_label:="$SEMANTIC_LABEL" \
    robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
    goal_x:="${ISAAC_DEMO_GOAL_X:-6.0}" \
    goal_y:="${ISAAC_DEMO_GOAL_Y:-4.0}" \
    enable_goal_picker:="$demo_goal_picker" \
    fixed_test:="$demo_fixed_test" \
    fixed_goals_file:="$demo_fixed_goals_file" \
    fixed_test_readiness_timeout_sec:="${ISAAC_DEMO_FIXED_TEST_READINESS_TIMEOUT_SEC:-60.0}" \
    fixed_test_auto_shutdown_delay_sec:="${ISAAC_DEMO_FIXED_TEST_AUTO_SHUTDOWN_DELAY_SEC:-2.0}" \
    fixed_test_max_linear:="${ISAAC_DEMO_FIXED_TEST_MAX_LINEAR:-0.8}" \
    fixed_test_max_angular:="${ISAAC_DEMO_FIXED_TEST_MAX_ANGULAR:-1.8}" \
    record_video:="$demo_record_video" \
    video_output_dir:="$demo_video_dir" \
    video_simulator_name:=isaac \
    auto_set_initial_goal:="$demo_auto_goal" \
    cmd_vel_topic:=/cmd_vel \
    max_linear:="${ISAAC_DEMO_MAX_LINEAR:-0.99}" max_angular:="${ISAAC_DEMO_MAX_ANGULAR:-1.99}" \
    goal_tolerance:=0.35 \
    front_stop_distance:=0.50 \
    stop_on_empty_front:=true \
    scan_timeout:="$demo_scan_timeout" \
    odom_timeout:="$demo_odom_timeout" \
    subgoal_timeout:="$demo_subgoal_timeout" \
    final_goal_timeout:="$demo_final_goal_timeout" \
    pedestrian_truth_timeout:="$demo_pedestrian_truth_timeout" \
    lookahead:=1.0 inflate_radius:="${ISAAC_DEMO_INFLATE_RADIUS:-0.45}" \
    show_actual_trajectory:=true \
    evaluate_episode:="$demo_evaluate" \
    evaluation_output_dir:="$demo_evaluation_output_dir" \
    evaluation_timeout_sec:="${ISAAC_DEMO_EVALUATION_TIMEOUT_SEC:-86400.0}" \
    evaluation_multi_episode:=true \
    record_trace:="$demo_record_trace" \
    trace_path:="$demo_trace_path" \
    trace_timeout_sec:="${ISAAC_DEMO_TRACE_TIMEOUT_SEC:-86400.0}" \
    start_online_ppo_training:=false \
    >"$log_dir/drlvo.log" 2>&1 &
policy_pid=$!

controller_preflight_log="$log_dir/controller_preflight.log"
if ! python3 "$SCRIPT_DIR/check_capture_ready.py" --timeout 90.0 \
    >"$controller_preflight_log" 2>&1; then
    echo "ERROR: DRL-VO controller preflight failed. See $controller_preflight_log" >&2
    sed -n '1,200p' "$controller_preflight_log" >&2
    exit 1
fi
if ! kill -0 "$policy_pid" 2>/dev/null; then
    echo "ERROR: DRL-VO exited during controller preflight. See $log_dir/drlvo.log" >&2
    exit 1
fi

if [[ "$demo_verify_navigation" == "true" ]]; then
    navigation_preflight_log="$log_dir/navigation_preflight.log"
    if ! python3 "$SCRIPT_DIR/check_capture_ready.py" \
        --timeout 120.0 --verify-navigation --verify-motion \
        >"$navigation_preflight_log" 2>&1; then
        echo "ERROR: DRL-VO navigation/motion verification failed. See $navigation_preflight_log" >&2
        sed -n '1,240p' "$navigation_preflight_log" >&2
        exit 1
    fi
    echo "ISAAC_DRLVO_NAVIGATION_VERIFIED=PASS"
    if [[ "$demo_exit_after_verify" == "true" ]]; then
        echo "ISAAC_DRLVO_SMOKE_TEST=PASS logs=$log_dir"
        exit 0
    fi
fi

if [[ "${ISAAC_DEMO_AUTO_CAPTURE:-0}" == "1" ]]; then
    echo "ISAAC_DRLVO_DEMO_READY scene=custom pedestrians=$isaac_expected_pedestrians seed=$ISAAC_PEDESTRIAN_SEED policy=base lidar=$ISAAC_LIDAR_MODE samples=$ISAAC_LIDAR_SAMPLE_COUNT goal=automatic_scheduler"
elif [[ "$demo_goal_picker" == "true" ]]; then
    echo "ISAAC_DRLVO_DEMO_READY scene=custom pedestrians=$isaac_expected_pedestrians seed=$ISAAC_PEDESTRIAN_SEED policy=base lidar=$ISAAC_LIDAR_MODE samples=$ISAAC_LIDAR_SAMPLE_COUNT goal=waiting_for_popup_selection"
else
    echo "ISAAC_DRLVO_DEMO_READY scene=custom pedestrians=$isaac_expected_pedestrians seed=$ISAAC_PEDESTRIAN_SEED policy=base lidar=$ISAAC_LIDAR_MODE samples=$ISAAC_LIDAR_SAMPLE_COUNT goal=(${ISAAC_DEMO_GOAL_X:-6.0},${ISAAC_DEMO_GOAL_Y:-4.0})"
fi
if [[ "${ISAAC_DEMO_AUTO_CAPTURE:-0}" == "1" ]]; then
    /usr/bin/python3 "$ROS_WS/install/semantic_nav_gazebo/lib/semantic_nav_gazebo/auto_goal_rosbag_scheduler.py" \
        --ros-args \
        -p use_sim_time:=true \
        -p map_yaml:="$MAP_YAML" \
        -p status_path:="$log_dir/auto_capture_status.json" \
        -p seed:="${ISAAC_DEMO_COLLECTION_SEED:-7001}" \
        -p capture_duration_sec:="$demo_capture_duration_sec" \
        -p goal_inflation_radius:=0.5 \
        -p route_inflation_radius:=0.4 \
        -p relocation_backend:=isaac_pose_topic \
        -p isaac_reset_pose_topic:=/isaac/reset_pose \
        >"$log_dir/auto_capture.log" 2>&1 &
    scheduler_pid=$!
    echo "ISAAC_DATA_COLLECTION_READY bag=$log_dir/rosbag status=$log_dir/auto_capture_status.json"
    set +e
    wait "$scheduler_pid"
    scheduler_exit_code=$?
    set -e
    scheduler_pid=""

    # The report reader needs a closed, fully indexed bag.  Stop only the
    # recorder process group created above, wait for rosbag2 to flush, then
    # render this run into its own never-overwritten output directory.
    if [[ "$bag_pid" =~ ^[0-9]+$ ]]; then
        kill -INT -- "-$bag_pid" 2>/dev/null || true
        set +e
        wait "$bag_pid"
        bag_exit_code=$?
        set -e
        bag_pid=""
        if (( bag_exit_code != 0 )); then
            echo "ERROR: rosbag recorder exited with status $bag_exit_code. See $log_dir/rosbag.log" >&2
            exit "$bag_exit_code"
        fi
    fi

    if (( scheduler_exit_code != 0 )); then
        echo "ERROR: automatic capture scheduler exited with status $scheduler_exit_code. See $log_dir/auto_capture.log" >&2
        exit "$scheduler_exit_code"
    fi

    visualization_args=(
        --bag "$log_dir/rosbag"
        --map-yaml "$MAP_YAML"
        --semantic-label "$SEMANTIC_LABEL"
        --output-dir "$log_dir/visualization"
    )
    if [[ -f "$log_dir/auto_capture_status.json" ]]; then
        visualization_args+=(--status-json "$log_dir/auto_capture_status.json")
    fi
    if ! /usr/bin/python3 "$TRAJECTORY_VISUALIZER" "${visualization_args[@]}" \
        >"$log_dir/visualization.log" 2>&1; then
        echo "ERROR: trajectory visualization failed. See $log_dir/visualization.log" >&2
        exit 1
    fi
    echo "ISAAC_EVALUATION_REPORT_READY directory=$log_dir/visualization index=$log_dir/visualization/episode_index.html"
else
    # Closing Isaac (or reaching a lower-level --duration) must also stop the
    # ROS policy launch.  Waiting only for the policy used to leave a headless
    # smoke test hanging after the simulator had already exited.
    while kill -0 "$policy_pid" 2>/dev/null \
        && kill -0 "$isaac_pid" 2>/dev/null; do
        sleep 1
    done
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        set +e
        wait "$isaac_pid"
        isaac_exit_code=$?
        set -e
        if (( isaac_exit_code != 0 )); then
            echo "ERROR: Isaac exited with status $isaac_exit_code. See $log_dir/isaac.log" >&2
        fi
        exit "$isaac_exit_code"
    fi
    wait "$policy_pid"
fi

if [[ "$demo_record_video" == "true" && -f "$demo_evaluation_output_dir/session_summary.json" ]]; then
    mkdir -p "$demo_video_dir"
    if ! python3 "$VIDEO_RENDERER" \
        --evaluation-dir "$demo_evaluation_output_dir" \
        --map-yaml "$MAP_YAML" \
        --capture-dir "$demo_video_dir/sync" \
        --output-mp4 "$demo_video_dir/evaluation_video.mp4" \
        --save-episode-screenshots \
        >"$demo_video_dir/render.log" 2>&1; then
        echo "ERROR: video rendering failed. See $demo_video_dir/render.log" >&2
        exit 1
    fi
    echo "FIXED_FOUR_VIDEO_READY path=$demo_video_dir/evaluation_video.mp4"
fi
