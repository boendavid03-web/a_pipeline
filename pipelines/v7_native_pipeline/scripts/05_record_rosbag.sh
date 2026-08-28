#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--duration, --iso-8601, --no-arr, --once, --progress-interval, --ros-args, --startup-timeout
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /data_collection/episode_event, /data_collection/goal_accepted, /odom, /pedestrian_ground_truth, /scan_01, /scan_02, /scan_merged, /scenario_pedestrian_controller, /semantic_cnn/final_goal, /semantic_cnn/global_path, /semantic_cnn/local_subgoal, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：SDF, YAML
# 可能使用的关键环境变量：AUTO_EPISODE_READY, AUTO_EPISODE_RECORDING, AUTO_EPISODE_RECORDING_VALUE, BAG_DIR, BAG_METADATA, BASH_SOURCE, BEGIN, CAPTURE_ACTUAL_MODEL_FILE, CAPTURE_AUTO_EPISODE_RECORDING, CAPTURE_CMD_VEL_PUBLISHER_COUNT, CAPTURE_COMPLETE, CAPTURE_EPISODE_ARRIVAL_DWELL_SEC, CAPTURE_EPISODE_END_EVENT_GRACE_SEC, CAPTURE_EXPECTED_TOPICS, CAPTURE_LIDAR_RANGE_MAX, CAPTURE_LIDAR_RANGE_MAX_01, CAPTURE_LIDAR_RANGE_MAX_02, CAPTURE_LIDAR_RANGE_MIN, CAPTURE_LIDAR_RANGE_MIN_01, CAPTURE_LIDAR_RANGE_MIN_02
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 10:12:21.153568331 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:27.122706557 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh（ros2 run 启动该节点）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（ros2 run 启动该节点）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/cmd_vel_stamper.py（ros2 run 启动该节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（ros2 run 启动该节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py（ros2 run 启动该节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/wait_for_sim_duration.py（ros2 run 启动该节点）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh（ros2 run 启动该节点）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/cmd_vel_stamper.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/wait_for_sim_duration.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜05_record_rosbag.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_manifest "${RUN_MANIFEST:-}"

safe_source_ros
export ROS_DOMAIN_ID
export IGN_PARTITION

LIDAR_SAMPLES_VALUE="${LIDAR_SAMPLES:-360}"
LIDAR_UPDATE_RATE_VALUE="${LIDAR_UPDATE_RATE:-10.0}"
LIDAR_SAMPLES_01_VALUE="${LIDAR_SAMPLES_01:-${LIDAR_SAMPLES_VALUE}}"
LIDAR_SAMPLES_02_VALUE="${LIDAR_SAMPLES_02:-${LIDAR_SAMPLES_VALUE}}"
LIDAR_UPDATE_RATE_01_VALUE="${LIDAR_UPDATE_RATE_01:-${LIDAR_UPDATE_RATE_VALUE}}"
LIDAR_UPDATE_RATE_02_VALUE="${LIDAR_UPDATE_RATE_02:-${LIDAR_UPDATE_RATE_VALUE}}"
LIDAR_RANGE_MIN_VALUE="${LIDAR_RANGE_MIN:-0.1}"
LIDAR_RANGE_MAX_VALUE="${LIDAR_RANGE_MAX:-50.0}"
LIDAR_RANGE_MIN_01_VALUE="${LIDAR_RANGE_MIN_01:-${LIDAR_RANGE_MIN_VALUE}}"
LIDAR_RANGE_MIN_02_VALUE="${LIDAR_RANGE_MIN_02:-${LIDAR_RANGE_MIN_VALUE}}"
LIDAR_RANGE_MAX_01_VALUE="${LIDAR_RANGE_MAX_01:-${LIDAR_RANGE_MAX_VALUE}}"
LIDAR_RANGE_MAX_02_VALUE="${LIDAR_RANGE_MAX_02:-${LIDAR_RANGE_MAX_VALUE}}"
LIDAR_RUNTIME_MODEL_FILE_VALUE="${LIDAR_RUNTIME_MODEL_FILE:-${RUN_ROOT}/runtime_models/lidar/model.sdf}"
EXPECTED_LIDAR_TOPICS_VALUE="${EXPECTED_LIDAR_TOPICS:-/scan_01 /scan_02}"
CAPTURE_SIM_DURATION_SEC_VALUE="${CAPTURE_SIM_DURATION_SEC:-0}"
REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE="${REQUIRE_PEDESTRIAN_GROUND_TRUTH:-0}"
REQUIRE_CMD_VEL_PUBLISHER_VALUE="${REQUIRE_CMD_VEL_PUBLISHER:-0}"
AUTO_EPISODE_RECORDING_VALUE="${AUTO_EPISODE_RECORDING:-0}"
EPISODE_ARRIVAL_DWELL_SEC_VALUE="${EPISODE_ARRIVAL_DWELL_SEC:-0.5}"
EPISODE_END_EVENT_GRACE_SEC_VALUE="${EPISODE_END_EVENT_GRACE_SEC:-0.2}"

require_file "${LIDAR_RUNTIME_MODEL_FILE_VALUE}" "runtime LiDAR model SDF"
if ! [[ "${CAPTURE_SIM_DURATION_SEC_VALUE}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: CAPTURE_SIM_DURATION_SEC must be zero or a positive number" >&2
  exit 2
fi
case "${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE,,}" in
  1|true|yes|on)
    REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE=1
    ;;
  0|false|no|off)
    REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE=0
    ;;
  *)
    echo "ERROR: REQUIRE_PEDESTRIAN_GROUND_TRUTH must be 0/1 or false/true" >&2
    exit 2
    ;;
esac
case "${REQUIRE_CMD_VEL_PUBLISHER_VALUE,,}" in
  1|true|yes|on)
    REQUIRE_CMD_VEL_PUBLISHER_VALUE=1
    ;;
  0|false|no|off)
    REQUIRE_CMD_VEL_PUBLISHER_VALUE=0
    ;;
  *)
    echo "ERROR: REQUIRE_CMD_VEL_PUBLISHER must be 0/1 or false/true" >&2
    exit 2
    ;;
esac
case "${AUTO_EPISODE_RECORDING_VALUE,,}" in
  1|true|yes|on)
    AUTO_EPISODE_RECORDING_VALUE=1
    ;;
  0|false|no|off)
    AUTO_EPISODE_RECORDING_VALUE=0
    ;;
  *)
    echo "ERROR: AUTO_EPISODE_RECORDING must be 0/1 or false/true" >&2
    exit 2
    ;;
esac
if [[ "${AUTO_EPISODE_RECORDING_VALUE}" == "1" ]] &&
   awk -v value="${CAPTURE_SIM_DURATION_SEC_VALUE}" 'BEGIN { exit !(value > 0.0) }'; then
  echo "ERROR: AUTO_EPISODE_RECORDING=1 requires CAPTURE_SIM_DURATION_SEC=0" >&2
  exit 2
fi
for value_name in \
  EPISODE_ARRIVAL_DWELL_SEC_VALUE \
  EPISODE_END_EVENT_GRACE_SEC_VALUE; do
  value="${!value_name}"
  if ! awk -v value="${value}" \
    'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0.0) }'; then
    echo "ERROR: ${value_name%_VALUE} must be a positive number" >&2
    exit 2
  fi
done

declare -A EXPECTED_TYPES=(
  ["/scan_merged"]="sensor_msgs/msg/LaserScan"
  ["/scan_01"]="sensor_msgs/msg/LaserScan"
  ["/scan_02"]="sensor_msgs/msg/LaserScan"
  ["/odom"]="nav_msgs/msg/Odometry"
  ["/tf"]="tf2_msgs/msg/TFMessage"
  ["/tf_static"]="tf2_msgs/msg/TFMessage"
  ["/cmd_vel"]="geometry_msgs/msg/Twist"
  ["/clock"]="rosgraph_msgs/msg/Clock"
  ["/pedestrian_ground_truth"]="semantic_nav_gazebo/msg/PedestrianStateArray"
  ["/semantic_cnn/global_path"]="nav_msgs/msg/Path"
  ["/semantic_cnn/local_subgoal"]="geometry_msgs/msg/PointStamped"
  ["/semantic_cnn/final_goal"]="geometry_msgs/msg/PointStamped"
  ["/data_collection/goal_accepted"]="geometry_msgs/msg/PointStamped"
  ["/data_collection/episode_event"]="std_msgs/msg/String"
)
CORE_TOPICS=(
  /scan_merged
  /scan_01
  /scan_02
  /odom
  /tf
  /tf_static
  /cmd_vel
  /clock
)

topic_type() {
  ros2 topic type "$1" 2>/dev/null | head -n 1
}

topic_has_publisher() {
  ros2 topic info "$1" 2>/dev/null | awk '
    /^Publisher count:/ { found = 1; exit !($3 > 0) }
    END { if (!found) exit 1 }
  '
}

topic_publisher_count() {
  ros2 topic info "$1" 2>/dev/null | awk '
    /^Publisher count:/ { print $3; found = 1; exit }
    END { if (!found) print 0 }
  '
}

echo "Checking live capture topics before creating a bag..."
for topic in "${CORE_TOPICS[@]}"; do
  actual_type="$(topic_type "${topic}")"
  if [[ "${actual_type}" != "${EXPECTED_TYPES[${topic}]}" ]]; then
    echo "ERROR: ${topic} has type '${actual_type:-missing}', expected '${EXPECTED_TYPES[${topic}]}'" >&2
    exit 3
  fi
  echo "PASS ${topic} ${actual_type}"
done
CMD_VEL_PUBLISHER_COUNT="$(topic_publisher_count /cmd_vel)"
if (( CMD_VEL_PUBLISHER_COUNT > 1 )); then
  echo "ERROR: /cmd_vel already has ${CMD_VEL_PUBLISHER_COUNT} publishers; keep exactly one teleop controller" >&2
  ros2 topic info -v /cmd_vel >&2 || true
  exit 3
fi
if [[ "${REQUIRE_CMD_VEL_PUBLISHER_VALUE}" == "1" && "${CMD_VEL_PUBLISHER_COUNT}" != "1" ]]; then
  echo "ERROR: start exactly one 05b_teleop.sh before recording; /cmd_vel publisher count is ${CMD_VEL_PUBLISHER_COUNT}" >&2
  exit 3
fi
for topic in /clock /scan_01 /scan_02 /odom; do
  if ! timeout 15 ros2 topic echo "${topic}" --once --no-arr >/dev/null 2>&1; then
    echo "ERROR: ${topic} exists but did not publish within 15 wall seconds" >&2
    exit 3
  fi
done

PEDESTRIAN_GROUND_TRUTH_TYPE="$(topic_type /pedestrian_ground_truth)"
if [[ "${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE}" == "1" ]]; then
  if [[ "${PEDESTRIAN_GROUND_TRUTH_TYPE}" != "${EXPECTED_TYPES[/pedestrian_ground_truth]}" ]]; then
    echo "ERROR: /pedestrian_ground_truth is required but has type '${PEDESTRIAN_GROUND_TRUTH_TYPE:-missing}'" >&2
    exit 3
  fi
  if ! timeout 15 ros2 topic echo /pedestrian_ground_truth --once --no-arr >/dev/null 2>&1; then
    echo "ERROR: /pedestrian_ground_truth did not publish within 15 wall seconds" >&2
    exit 3
  fi
fi

CONTROLLER_NODE="/scenario_pedestrian_controller"
PEDESTRIAN_COUNT_VALUE=""
PEDESTRIAN_SEED_VALUE=""
PEDESTRIAN_SPEED_VALUE=""
PEDESTRIAN_UPDATE_RATE_VALUE=""
PEDESTRIAN_SIMULATION_FACTOR_VALUE=""
if ros2 node list 2>/dev/null | grep -qx "${CONTROLLER_NODE}"; then
  PEDESTRIAN_COUNT_VALUE="$(ros2 param get "${CONTROLLER_NODE}" pedestrian_count 2>/dev/null | awk '{print $NF}')"
  PEDESTRIAN_SEED_VALUE="$(ros2 param get "${CONTROLLER_NODE}" seed 2>/dev/null | awk '{print $NF}')"
  PEDESTRIAN_SPEED_VALUE="$(ros2 param get "${CONTROLLER_NODE}" speed 2>/dev/null | awk '{print $NF}')"
  PEDESTRIAN_UPDATE_RATE_VALUE="$(ros2 param get "${CONTROLLER_NODE}" update_rate 2>/dev/null | awk '{print $NF}')"
  PEDESTRIAN_SIMULATION_FACTOR_VALUE="$(ros2 param get "${CONTROLLER_NODE}" simulation_factor 2>/dev/null | awk '{print $NF}')"
fi
if [[ "${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE}" == "1" ]]; then
  if [[ -z "${PEDESTRIAN_SIMULATION_FACTOR_VALUE}" ]]; then
    echo "ERROR: pedestrian controller parameters are unavailable" >&2
    exit 3
  fi
  if ! awk -v value="${PEDESTRIAN_SIMULATION_FACTOR_VALUE}" \
    'BEGIN { exit !((value + 0.0) >= 0.999999999 && (value + 0.0) <= 1.000000001) }'; then
    echo "ERROR: pedestrian simulation_factor must be 1.0, got ${PEDESTRIAN_SIMULATION_FACTOR_VALUE}" >&2
    exit 3
  fi
fi

if ! python3 "${ROS_WS}/tools/wait_for_sim_duration.py" \
  --duration 0.2 --progress-interval 1.0 --startup-timeout 15.0; then
  echo "ERROR: /clock is not advancing" >&2
  exit 3
fi

RECORD_TOPICS=("${CORE_TOPICS[@]}" /cmd_vel_stamped)
if [[ "${PEDESTRIAN_GROUND_TRUTH_TYPE}" == "${EXPECTED_TYPES[/pedestrian_ground_truth]}" ]]; then
  RECORD_TOPICS+=(/pedestrian_ground_truth)
else
  echo "INFO: pedestrian controller is not active; ground truth will not be recorded"
fi

ONLINE_GOAL_TOPICS=(
  /semantic_cnn/global_path
  /semantic_cnn/local_subgoal
  /semantic_cnn/final_goal
)
for topic in "${ONLINE_GOAL_TOPICS[@]}"; do
  actual_type="$(topic_type "${topic}")"
  if [[ "${actual_type}" != "${EXPECTED_TYPES[${topic}]}" ]]; then
    echo "ERROR: ${topic} has type '${actual_type:-missing}', expected '${EXPECTED_TYPES[${topic}]}'" >&2
    echo "Start semantic_start_goal_path_node.py before recording." >&2
    exit 3
  fi
  if ! topic_has_publisher "${topic}"; then
    echo "ERROR: ${topic} has no publisher; start the path-only node before recording" >&2
    exit 3
  fi
  if ! timeout 15 ros2 topic echo "${topic}" --once --no-arr >/dev/null 2>&1; then
    echo "ERROR: ${topic} did not publish within 15 wall seconds" >&2
    exit 3
  fi
  RECORD_TOPICS+=("${topic}")
  echo "PASS ${topic} ${actual_type}; REQUIRED topic will be recorded"
done
if [[ "${AUTO_EPISODE_RECORDING_VALUE}" == "1" ]]; then
  GOAL_ACCEPTED_TOPIC=/data_collection/goal_accepted
  actual_type="$(topic_type "${GOAL_ACCEPTED_TOPIC}")"
  if [[ "${actual_type}" != "${EXPECTED_TYPES[${GOAL_ACCEPTED_TOPIC}]}" ]]; then
    echo "ERROR: ${GOAL_ACCEPTED_TOPIC} has type '${actual_type:-missing}', expected '${EXPECTED_TYPES[${GOAL_ACCEPTED_TOPIC}]}'" >&2
    echo "Rebuild and restart semantic_start_goal_path_node.py with dynamic-goal support." >&2
    exit 3
  fi
  if ! topic_has_publisher "${GOAL_ACCEPTED_TOPIC}"; then
    echo "ERROR: ${GOAL_ACCEPTED_TOPIC} has no publisher" >&2
    exit 3
  fi
  RECORD_TOPICS+=(/data_collection/episode_event)
  echo "PASS ${GOAL_ACCEPTED_TOPIC} ${actual_type}; automatic episode control enabled"
fi

STEP_ID="$(timestamp)_v7_dual_teleop_bag"
while [[ -e "${RUN_ROOT}/bags/raw/${STEP_ID}" ]]; do
  sleep 1
  STEP_ID="$(timestamp)_v7_dual_teleop_bag"
done

BAG_DIR="${RUN_ROOT}/bags/raw/${STEP_ID}"
LOG="${RUN_ROOT}/logs/05_record_rosbag_$(timestamp).log"
STAMPER_LOG="${RUN_ROOT}/logs/05_record_rosbag_cmd_vel_stamper_$(timestamp).log"
CAPTURE_MANIFEST="${RUN_ROOT}/bags/capture_manifests/${STEP_ID}.env"
STAMPER="${ROS_WS}/tools/cmd_vel_stamper.py"
mkdir -p "$(dirname "${BAG_DIR}")" "$(dirname "${CAPTURE_MANIFEST}")" "${RUN_ROOT}/logs"

RECORDED_TOPICS_VALUE="${RECORD_TOPICS[*]}"
set_manifest_var "LAST_BAG_DIR" "${BAG_DIR}"
set_manifest_var "BAG_DIR" "${BAG_DIR}"
set_manifest_var "CAPTURE_LIDAR_SAMPLES" "${LIDAR_SAMPLES_VALUE}"
set_manifest_var "CAPTURE_LIDAR_UPDATE_RATE" "${LIDAR_UPDATE_RATE_VALUE}"
set_manifest_var "CAPTURE_LIDAR_SAMPLES_01" "${LIDAR_SAMPLES_01_VALUE}"
set_manifest_var "CAPTURE_LIDAR_SAMPLES_02" "${LIDAR_SAMPLES_02_VALUE}"
set_manifest_var "CAPTURE_LIDAR_UPDATE_RATE_01" "${LIDAR_UPDATE_RATE_01_VALUE}"
set_manifest_var "CAPTURE_LIDAR_UPDATE_RATE_02" "${LIDAR_UPDATE_RATE_02_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MIN" "${LIDAR_RANGE_MIN_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MAX" "${LIDAR_RANGE_MAX_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MIN_01" "${LIDAR_RANGE_MIN_01_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MIN_02" "${LIDAR_RANGE_MIN_02_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MAX_01" "${LIDAR_RANGE_MAX_01_VALUE}"
set_manifest_var "CAPTURE_LIDAR_RANGE_MAX_02" "${LIDAR_RANGE_MAX_02_VALUE}"
set_manifest_var "CAPTURE_ACTUAL_MODEL_FILE" "${LIDAR_RUNTIME_MODEL_FILE_VALUE}"
set_manifest_var "CAPTURE_EXPECTED_TOPICS" "${EXPECTED_LIDAR_TOPICS_VALUE}"
set_manifest_var "CAPTURE_RECORDED_TOPICS" "${RECORDED_TOPICS_VALUE}"
set_manifest_var "CAPTURE_SIM_DURATION_REQUESTED" "${CAPTURE_SIM_DURATION_SEC_VALUE}"
set_manifest_var "CAPTURE_AUTO_EPISODE_RECORDING" "${AUTO_EPISODE_RECORDING_VALUE}"
set_manifest_var "CAPTURE_EPISODE_ARRIVAL_DWELL_SEC" "${EPISODE_ARRIVAL_DWELL_SEC_VALUE}"
set_manifest_var "CAPTURE_EPISODE_END_EVENT_GRACE_SEC" "${EPISODE_END_EVENT_GRACE_SEC_VALUE}"
set_manifest_var "CAPTURE_CMD_VEL_PUBLISHER_COUNT" "${CMD_VEL_PUBLISHER_COUNT}"
set_manifest_var "CAPTURE_PEDESTRIAN_COUNT" "${PEDESTRIAN_COUNT_VALUE}"
set_manifest_var "CAPTURE_PEDESTRIAN_SEED" "${PEDESTRIAN_SEED_VALUE}"
set_manifest_var "CAPTURE_PEDESTRIAN_SPEED" "${PEDESTRIAN_SPEED_VALUE}"
set_manifest_var "CAPTURE_PEDESTRIAN_UPDATE_RATE" "${PEDESTRIAN_UPDATE_RATE_VALUE}"
set_manifest_var "CAPTURE_PEDESTRIAN_SIMULATION_FACTOR" "${PEDESTRIAN_SIMULATION_FACTOR_VALUE}"
set_manifest_var "CAPTURE_OUTPUT_DIR" "${BAG_DIR}"
set_manifest_var "CAPTURE_MANIFEST" "${CAPTURE_MANIFEST}"

{
  printf 'RUN_ID=%q\n' "${RUN_ID}"
  printf 'STARTED_AT=%q\n' "$(date --iso-8601=seconds)"
  printf 'ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID}"
  printf 'IGN_PARTITION=%q\n' "${IGN_PARTITION}"
  printf 'BAG_DIR=%q\n' "${BAG_DIR}"
  printf 'RECORDED_TOPICS=%q\n' "${RECORDED_TOPICS_VALUE}"
  printf 'SIM_DURATION_REQUESTED=%q\n' "${CAPTURE_SIM_DURATION_SEC_VALUE}"
  printf 'AUTO_EPISODE_RECORDING=%q\n' "${AUTO_EPISODE_RECORDING_VALUE}"
  printf 'EPISODE_ARRIVAL_DWELL_SEC=%q\n' "${EPISODE_ARRIVAL_DWELL_SEC_VALUE}"
  printf 'EPISODE_END_EVENT_GRACE_SEC=%q\n' "${EPISODE_END_EVENT_GRACE_SEC_VALUE}"
  printf 'CMD_VEL_PUBLISHER_COUNT=%q\n' "${CMD_VEL_PUBLISHER_COUNT}"
  printf 'PEDESTRIAN_COUNT=%q\n' "${PEDESTRIAN_COUNT_VALUE}"
  printf 'PEDESTRIAN_SEED=%q\n' "${PEDESTRIAN_SEED_VALUE}"
  printf 'PEDESTRIAN_SPEED=%q\n' "${PEDESTRIAN_SPEED_VALUE}"
  printf 'PEDESTRIAN_UPDATE_RATE=%q\n' "${PEDESTRIAN_UPDATE_RATE_VALUE}"
  printf 'PEDESTRIAN_SIMULATION_FACTOR=%q\n' "${PEDESTRIAN_SIMULATION_FACTOR_VALUE}"
  printf 'LIDAR_SAMPLES_01=%q\n' "${LIDAR_SAMPLES_01_VALUE}"
  printf 'LIDAR_SAMPLES_02=%q\n' "${LIDAR_SAMPLES_02_VALUE}"
  printf 'LIDAR_UPDATE_RATE_01=%q\n' "${LIDAR_UPDATE_RATE_01_VALUE}"
  printf 'LIDAR_UPDATE_RATE_02=%q\n' "${LIDAR_UPDATE_RATE_02_VALUE}"
} >"${CAPTURE_MANIFEST}"

STAMPER_PID=""
RECORDER_PID=""
EPISODE_CONTROLLER_PID=""

stop_recorder() {
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
    kill -INT "${RECORDER_PID}" >/dev/null 2>&1 || true
    wait "${RECORDER_PID}" >/dev/null 2>&1 || true
  fi
  RECORDER_PID=""
}

stop_stamper() {
  if [[ -n "${STAMPER_PID}" ]] && kill -0 "${STAMPER_PID}" >/dev/null 2>&1; then
    kill -INT "${STAMPER_PID}" >/dev/null 2>&1 || true
    wait "${STAMPER_PID}" >/dev/null 2>&1 || true
  fi
  STAMPER_PID=""
}

stop_episode_controller() {
  if [[ -n "${EPISODE_CONTROLLER_PID}" ]] &&
     kill -0 "${EPISODE_CONTROLLER_PID}" >/dev/null 2>&1; then
    kill -INT "${EPISODE_CONTROLLER_PID}" >/dev/null 2>&1 || true
    wait "${EPISODE_CONTROLLER_PID}" >/dev/null 2>&1 || true
  fi
  EPISODE_CONTROLLER_PID=""
}

cleanup() {
  stop_recorder
  stop_episode_controller
  stop_stamper
}

handle_interrupt() {
  echo
  echo "Stopping recorder and finalizing metadata..."
  stop_recorder
}

trap handle_interrupt INT TERM
trap cleanup EXIT

python3 "${STAMPER}" >"${STAMPER_LOG}" 2>&1 &
STAMPER_PID="$!"
sleep 1
if ! kill -0 "${STAMPER_PID}" >/dev/null 2>&1; then
  echo "ERROR: cmd_vel stamper exited before recording. See ${STAMPER_LOG}" >&2
  exit 1
fi
if [[ "$(topic_type /cmd_vel_stamped)" != "geometry_msgs/msg/TwistStamped" ]]; then
  echo "ERROR: /cmd_vel_stamped was not created by the stamper" >&2
  exit 1
fi

echo "Recording to ${BAG_DIR}"
echo "Recorder log: ${LOG}"
RECORDER_ARGS=(-o "${BAG_DIR}")
ros2 bag record "${RECORDER_ARGS[@]}" "${RECORD_TOPICS[@]}" >"${LOG}" 2>&1 &
RECORDER_PID="$!"

if [[ "${AUTO_EPISODE_RECORDING_VALUE}" == "1" ]]; then
  EPISODE_CONTROLLER_LOG="${RUN_ROOT}/logs/05_record_rosbag_episode_controller_$(timestamp).log"
  ros2 run semantic_nav_gazebo teleop_episode_recorder_controller.py --ros-args \
    -p arrival_dwell_sec:="${EPISODE_ARRIVAL_DWELL_SEC_VALUE}" \
    -p end_event_grace_sec:="${EPISODE_END_EVENT_GRACE_SEC_VALUE}" \
    -p manage_recorder_pause:=false \
    >"${EPISODE_CONTROLLER_LOG}" 2>&1 &
  EPISODE_CONTROLLER_PID="$!"
  EPISODE_READY=0
  for _ in $(seq 1 15); do
    if ! kill -0 "${EPISODE_CONTROLLER_PID}" >/dev/null 2>&1; then
      echo "ERROR: episode controller exited during startup. See ${EPISODE_CONTROLLER_LOG}" >&2
      tail -n 30 "${EPISODE_CONTROLLER_LOG}" >&2 || true
      exit 1
    fi
    if [[ "$(topic_type /data_collection/episode_event)" == "std_msgs/msg/String" ]]; then
      EPISODE_READY=1
      break
    fi
    sleep 1
  done
  if [[ "${EPISODE_READY}" != "1" ]]; then
    echo "ERROR: episode controller did not create /data_collection/episode_event" >&2
    exit 1
  fi
fi

READY=0
for _ in $(seq 1 15); do
  if ! kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
    echo "ERROR: ros2 bag recorder exited during startup. See ${LOG}" >&2
    tail -n 30 "${LOG}" >&2 || true
    exit 1
  fi
  if grep -q "All requested topics are subscribed" "${LOG}"; then
    READY=1
    break
  fi
  sleep 1
done
if [[ "${READY}" != "1" ]]; then
  echo "ERROR: recorder did not subscribe to all requested topics within 15 seconds" >&2
  tail -n 30 "${LOG}" >&2 || true
  exit 1
fi

echo "CAPTURE_READY: all requested topics are subscribed."
if [[ "${AUTO_EPISODE_RECORDING_VALUE}" == "1" ]]; then
  echo "AUTO_EPISODE_READY: continuous rosbag recording with event-marked episodes."
  echo "Move with teleop to emit the start event for this episode."
  echo "After reaching the goal, press k and remain stopped for at least ${EPISODE_ARRIVAL_DWELL_SEC_VALUE} seconds."
  echo "Wait for EPISODE_SAVED in ${EPISODE_CONTROLLER_LOG}, then publish the next /goal_pose."
  echo "Monitor episode state with: tail -f \"${EPISODE_CONTROLLER_LOG}\""
  echo "Use Ctrl-C here only after all episodes are complete."
  set +e
  wait "${RECORDER_PID}"
  set -e
  RECORDER_PID=""
elif awk -v value="${CAPTURE_SIM_DURATION_SEC_VALUE}" 'BEGIN { exit !(value > 0.0) }'; then
  echo "Start teleoperation now. Press k before the recording ends."
  echo "The bag will stop after ${CAPTURE_SIM_DURATION_SEC_VALUE} seconds of /clock simulation time."
  python3 "${ROS_WS}/tools/wait_for_sim_duration.py" \
    --duration "${CAPTURE_SIM_DURATION_SEC_VALUE}" \
    --progress-interval 10.0 \
    --startup-timeout 15.0
  stop_recorder
else
  echo "Start teleoperation now. Press k before the recording ends."
  echo "Stop manually with Ctrl-C after driving."
  set +e
  wait "${RECORDER_PID}"
  set -e
  RECORDER_PID=""
fi
stop_episode_controller
stop_stamper

require_file "${BAG_DIR}/metadata.yaml" "recorded bag metadata"
{
  printf 'COMPLETED_AT=%q\n' "$(date --iso-8601=seconds)"
  printf 'BAG_METADATA=%q\n' "${BAG_DIR}/metadata.yaml"
  printf 'RECORDER_LOG=%q\n' "${LOG}"
  printf 'STAMPER_LOG=%q\n' "${STAMPER_LOG}"
  if [[ "${AUTO_EPISODE_RECORDING_VALUE}" == "1" ]]; then
    printf 'EPISODE_CONTROLLER_LOG=%q\n' "${EPISODE_CONTROLLER_LOG}"
  fi
} >>"${CAPTURE_MANIFEST}"

trap - INT TERM EXIT
echo "CAPTURE_COMPLETE"
echo "BAG_DIR=${BAG_DIR}"
echo "CAPTURE_MANIFEST=${CAPTURE_MANIFEST}"
echo "Next: check only this bag:"
echo "REQUIRE_PEDESTRIAN_GROUND_TRUTH=${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE} bash ${SCRIPT_DIR}/06_check_bag.sh \"${BAG_DIR}\""
