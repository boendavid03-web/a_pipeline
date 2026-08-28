#!/usr/bin/env bash
# One-terminal unattended DRL-VO teacher capture with machine-selected goals.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
safe_source_ros

CAPTURE_ROOT="${AUTO_CAPTURE_ROOT:-${PROJECT_ROOT}/runs/20260808_gazebo_play/bags/raw}"
LOG_ROOT="${AUTO_CAPTURE_LOG_ROOT:-${PROJECT_ROOT}/runs/20260808_gazebo_play/logs}"
ROS_LOG_DIR_VALUE="${AUTO_ROS_LOG_DIR:-${LOG_ROOT}/ros2}"
MPLCONFIGDIR_VALUE="${AUTO_MPLCONFIGDIR:-${LOG_ROOT}/matplotlib}"
DEMO_ROOT="${AUTO_DEMO_OUTPUT_ROOT:-${PROJECT_ROOT}/runs/20260808_gazebo_play/evaluations}"
MAP_YAML_VALUE="${AUTO_MAP_YAML:-${PROJECT_ROOT}/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml}"
SEMANTIC_LABEL_VALUE="${AUTO_SEMANTIC_LABEL:-${PROJECT_ROOT}/runs/20260717_042135_v7_dual/maps/semantic_label/label.png}"
LABEL_NAMES_VALUE="${AUTO_LABEL_NAMES:-$(dirname "${SEMANTIC_LABEL_VALUE}")/label_names.txt}"
TASK_ROOT_VALUE="${AUTO_TASK_ROOT:-${PROJECT_ROOT}/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1}"
CHECKPOINT_VALUE="${AUTO_DRLVO_CHECKPOINT:-${TASK_ROOT_VALUE}/training/drl_vo/base_bc/20260727_114455/checkpoints/best.pt}"
CAPTURE_DURATION_MIN_VALUE="${CAPTURE_DURATION_MIN:-}"
if [[ -n "${CAPTURE_DURATION_MIN_VALUE}" && -n "${CAPTURE_SIM_DURATION_SEC:-}" ]]; then
  echo "ERROR: set only one of CAPTURE_DURATION_MIN or CAPTURE_SIM_DURATION_SEC" >&2
  exit 2
fi
if [[ -n "${CAPTURE_DURATION_MIN_VALUE}" ]]; then
  if ! awk -v number="${CAPTURE_DURATION_MIN_VALUE}" \
    'BEGIN { exit !(number ~ /^[0-9]+([.][0-9]+)?$/ && number > 0) }'; then
    echo "ERROR: CAPTURE_DURATION_MIN must be a positive number" >&2
    exit 2
  fi
  CAPTURE_DURATION_VALUE="$(awk -v minutes="${CAPTURE_DURATION_MIN_VALUE}" \
    'BEGIN { printf "%.9g", minutes * 60.0 }')"
else
  CAPTURE_DURATION_VALUE="${CAPTURE_SIM_DURATION_SEC:-1800}"
  CAPTURE_DURATION_MIN_VALUE="$(awk -v seconds="${CAPTURE_DURATION_VALUE}" \
    'BEGIN { printf "%.9g", seconds / 60.0 }')"
fi
COMPLETE_BAGS_TARGET="${AUTO_COMPLETE_BAG_COUNT:-1}"
MAX_FAILED_ATTEMPTS="${AUTO_MAX_FAILED_ATTEMPTS:-10}"
PEDESTRIAN_COUNT_VALUE="${PEDESTRIAN_COUNT:-19}"
PEDESTRIAN_SEED_START_VALUE="${PEDESTRIAN_SEED_START:-7}"
GOAL_SEED_START_VALUE="${AUTO_GOAL_SEED_START:-7001}"
GUI_VALUE="${GUI:-true}"
START_RVIZ_VALUE="${START_RVIZ:-true}"
AUTO_DRLVO_DEVICE_VALUE="${AUTO_DRLVO_DEVICE:-auto}"
GOAL_INFLATION_VALUE="${GOAL_SAFE_INFLATION_RADIUS:-0.5}"
ROUTE_INFLATION_VALUE="${ROUTE_INFLATION_RADIUS:-0.4}"
EPISODE_TIMEOUT_VALUE="${AUTO_EPISODE_TIMEOUT_SEC:-240}"
STUCK_WINDOW_VALUE="${AUTO_STUCK_WINDOW_SEC:-20}"
INITIAL_GOAL_DELAY_VALUE="${AUTO_INITIAL_GOAL_DELAY_SEC:-8}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
case "${AUTO_VALIDATE:-1}" in
  1|true|TRUE|yes|YES|on|ON) AUTO_VALIDATE_VALUE=1 ;;
  0|false|FALSE|no|NO|off|OFF) AUTO_VALIDATE_VALUE=0 ;;
  *)
    echo "ERROR: AUTO_VALIDATE must be a boolean" >&2
    exit 2
    ;;
esac
AUTO_CLEAN_STALE_VALUE="${AUTO_CLEAN_STALE:-1}"
AUTO_CONTINUE_AFTER_FAILURE_VALUE="${AUTO_CONTINUE_AFTER_EPISODE_FAILURE:-1}"
AUTO_MIN_SUCCESSFUL_EPISODES_VALUE="${AUTO_MIN_SUCCESSFUL_EPISODES:-3}"
AUTO_MIN_SUCCESSFUL_DURATION_VALUE="${AUTO_MIN_SUCCESSFUL_DURATION_SEC:-$(awk -v seconds="${CAPTURE_DURATION_VALUE}" 'BEGIN { printf "%.9g", seconds * 0.8 }')}"
AUTO_FAILURE_NEXT_GOAL_DELAY_VALUE="${AUTO_FAILURE_NEXT_GOAL_DELAY_SEC:-0.25}"
AUTO_RECOVERY_STOP_DWELL_VALUE="${AUTO_RECOVERY_STOP_DWELL_SEC:-0.5}"
AUTO_RELOCATION_AFTER_FAILURES_VALUE="${AUTO_RELOCATION_AFTER_FAILURES:-3}"
AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE="${AUTO_RELOCATION_SERVICE_TIMEOUT_SEC:-10}"
AUTO_RELOCATION_ODOM_TIMEOUT_VALUE="${AUTO_RELOCATION_ODOM_TIMEOUT_SEC:-5}"
AUTO_RELOCATION_ODOM_TOLERANCE_VALUE="${AUTO_RELOCATION_ODOM_TOLERANCE_M:-0.5}"
ROBOT_RESET_SERVICE_VALUE="${AUTO_ROBOT_RESET_SERVICE:-/world/default/set_pose}"
ROBOT_ENTITY_NAME_VALUE="${AUTO_ROBOT_ENTITY_NAME:-mecanum730_xms5_v7_teacher_dual_scan}"
AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE="${AUTO_HUMAN_COLLISION_CONFIRMATION_SEC:-0.2}"
AUTO_HUMAN_COLLISION_PENETRATION_VALUE="${AUTO_HUMAN_COLLISION_PENETRATION_M:-0.0}"
AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE="${AUTO_ACTUATION_DEADLOCK_WINDOW_SEC:-2.5}"
AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE="${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO:-0.8}"
AUTO_ACTUATION_DEADLOCK_GOAL_X_VALUE="${AUTO_ACTUATION_DEADLOCK_GOAL_X_THRESHOLD:--0.05}"
AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_VALUE="${AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_COMMAND:-0.02}"
AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE="${AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_COMMAND:-0.05}"
AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE="${AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_M:-0.02}"
AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE="${AUTO_ACTUATION_DEADLOCK_MAX_YAW_PROGRESS_RAD:-0.03}"
AUTO_SUPERVISION_EXPORT_VALUE="${AUTO_EXPORT_CNN_SUPERVISION:-1}"
SUPERVISION_ROOT="${AUTO_SUPERVISION_OUTPUT_ROOT:-${PROJECT_ROOT}/runs/20260808_gazebo_play/datasets/auto_teacher_cnn_supervision}"
CNN_TRAINING_ROOT="${AUTO_CNN_TRAINING_OUTPUT_ROOT:-${PROJECT_ROOT}/runs/20260808_gazebo_play/datasets/auto_teacher_semantic2d}"
AUTO_CNN_SPLIT_ROLE_VALUE="${AUTO_CNN_SPLIT_ROLE:-train}"
AUTO_SUPERVISION_MIN_DURATION_VALUE="${AUTO_SUPERVISION_MIN_DURATION_SEC:-$(awk -v seconds="${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}" 'BEGIN { printf "%.9g", seconds * 0.9 }')}"
AUTO_SUPERVISION_MIN_RATE_VALUE="${AUTO_SUPERVISION_MIN_EFFECTIVE_RATE_HZ:-12.0}"
AUTO_SUPERVISION_MIN_SAMPLES_VALUE="${AUTO_SUPERVISION_MIN_SAMPLES:-$(awk -v seconds="${AUTO_SUPERVISION_MIN_DURATION_VALUE}" -v rate="${AUTO_SUPERVISION_MIN_RATE_VALUE}" 'BEGIN { value=seconds*rate; whole=int(value); print whole < value ? whole + 1 : whole }')}"
AUTO_SUPERVISION_MIN_UNIQUE_COMMANDS_VALUE="${AUTO_SUPERVISION_MIN_UNIQUE_COMMANDS:-100}"
AUTO_SUPERVISION_MIN_NONZERO_FRACTION_VALUE="${AUTO_SUPERVISION_MIN_NONZERO_FRACTION:-0.5}"
AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE="${AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION:-0.1}"
AUTO_MAX_CMD_AGE_MS_VALUE="${AUTO_MAX_CMD_VEL_AGE_MS:-100}"
AUTO_SUPERVISION_SAMPLES_VALUE="${AUTO_SUPERVISION_SAMPLES:-2000}"
AUTO_LIDAR_EXPECTED_RATE_VALUE="${AUTO_LIDAR_EXPECTED_RATE:-15.0}"
CMD_VEL_ANGULAR_SCALE_VALUE="${AUTO_CMD_VEL_ANGULAR_Z_SCALE:-1.5}"
AUTO_RESTART_FAILED_ATTEMPTS_VALUE="${AUTO_RESTART_FAILED_ATTEMPTS:-0}"
AUTO_CLOCK_STALL_WALL_SEC_VALUE="${AUTO_CLOCK_STALL_WALL_SEC:-60}"
AUTO_FIRST_EPISODE_WALL_SEC_VALUE="${AUTO_FIRST_EPISODE_WALL_SEC:-300}"
AUTO_MAX_CAPTURE_WALL_SEC_VALUE="${AUTO_MAX_CAPTURE_WALL_SEC:-$(awk \
  -v capture="${CAPTURE_DURATION_VALUE}" -v episode="${EPISODE_TIMEOUT_VALUE}" \
  'BEGIN { value=capture*10.0+episode; if (value < 900) value=900; print int(value+0.999999) }')}"
AUTO_MIN_FREE_DISK_GIB_VALUE="${AUTO_MIN_FREE_DISK_GIB:-20}"
case "${AUTO_RECORD_DEMO:-0}" in
  1|true|TRUE|yes|YES|on|ON) AUTO_RECORD_DEMO_VALUE=true ;;
  0|false|FALSE|no|NO|off|OFF) AUTO_RECORD_DEMO_VALUE=false ;;
  *)
    echo "ERROR: AUTO_RECORD_DEMO must be a boolean" >&2
    exit 2
    ;;
esac
export ROS_DOMAIN_ID IGN_IP=127.0.0.1 GZ_IP=127.0.0.1

case "${AUTO_DRLVO_DEVICE_VALUE}" in
  auto|cpu|cuda) ;;
  *)
    echo "ERROR: AUTO_DRLVO_DEVICE must be auto, cpu, or cuda" >&2
    exit 2
    ;;
esac
case "${AUTO_CNN_SPLIT_ROLE_VALUE}" in
  auto|train|dev|test|preserve) ;;
  *)
    echo "ERROR: AUTO_CNN_SPLIT_ROLE must be auto, train, dev, test, or preserve" >&2
    exit 2
    ;;
esac

require_file "${MAP_YAML_VALUE}" "semantic occupancy map"
require_file "${SEMANTIC_LABEL_VALUE}" "semantic label image"
require_file "${LABEL_NAMES_VALUE}" "semantic label names"
require_file "${CHECKPOINT_VALUE}" "DRL-VO teacher checkpoint"
require_file "${TORCH_PY}" "training-environment Python"
for required_tool in \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/snapshot_supervision_assets.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/write_auto_capture_contract.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py" \
  "${PROJECT_ROOT}/pipelines/v7_native_pipeline/scripts/visualize_auto_capture_trajectories.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py" \
  "${ROS_WS}/tools/check_cmd_vel_stamped_bag.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/seal_cnn_supervision_dataset.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/seal_semantic2d_training_session.py" \
  "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/register_semantic2d_session.py" \
  "${ROS_WS}/src/semantic_nav_gazebo/config/v7_dual_laser_scan_merger.yaml"; do
  require_file "${required_tool}" "automatic-capture dependency"
done
SUPERVISION_ROOT_RESOLVED="$(realpath -m "${SUPERVISION_ROOT}")"
CNN_TRAINING_ROOT_RESOLVED="$(realpath -m "${CNN_TRAINING_ROOT}")"
if [[ "${SUPERVISION_ROOT_RESOLVED}" == "${CNN_TRAINING_ROOT_RESOLVED}" || \
      "${SUPERVISION_ROOT_RESOLVED}" == "${CNN_TRAINING_ROOT_RESOLVED}/"* || \
      "${CNN_TRAINING_ROOT_RESOLVED}" == "${SUPERVISION_ROOT_RESOLVED}/"* ]]; then
  echo "ERROR: fixed-slot and SemanticCNN training roots must be separate, non-nested directories" >&2
  exit 2
fi
if [[ -n "${PEDESTRIAN_FREE_SPACE_MAP_YAML:-}" ]]; then
  require_file "${PEDESTRIAN_FREE_SPACE_MAP_YAML}" \
    "pedestrian free-space map"
  if [[ "$(realpath "${PEDESTRIAN_FREE_SPACE_MAP_YAML}")" != \
        "$(realpath "${MAP_YAML_VALUE}")" ]]; then
    echo "ERROR: pedestrian free-space map must match AUTO_MAP_YAML so one immutable map contract governs the run" >&2
    exit 2
  fi
fi
if ! AUTO_DRLVO_CUDA_AVAILABLE="$("${TORCH_PY}" -c \
  'import rclpy, torch; from semantic_nav_gazebo.msg import PedestrianStateArray; print(1 if torch.cuda.is_available() else 0)')"; then
  echo "ERROR: the training Python cannot import torch, rclpy and semantic_nav_gazebo messages" >&2
  exit 2
fi
case "${AUTO_DRLVO_DEVICE_VALUE}" in
  auto)
    if [[ "${AUTO_DRLVO_CUDA_AVAILABLE}" == "1" ]]; then
      AUTO_DRLVO_RESOLVED_DEVICE_VALUE=cuda
    else
      AUTO_DRLVO_RESOLVED_DEVICE_VALUE=cpu
    fi
    ;;
  cuda)
    if [[ "${AUTO_DRLVO_CUDA_AVAILABLE}" != "1" ]]; then
      echo "ERROR: AUTO_DRLVO_DEVICE=cuda but CUDA is unavailable" >&2
      exit 2
    fi
    AUTO_DRLVO_RESOLVED_DEVICE_VALUE=cuda
    ;;
  cpu) AUTO_DRLVO_RESOLVED_DEVICE_VALUE=cpu ;;
esac
for value in "${CAPTURE_DURATION_VALUE}" "${GOAL_INFLATION_VALUE}" \
  "${ROUTE_INFLATION_VALUE}" "${EPISODE_TIMEOUT_VALUE}" "${STUCK_WINDOW_VALUE}"; do
  if ! awk -v number="${value}" 'BEGIN { exit !(number ~ /^[0-9]+([.][0-9]+)?$/ && number > 0) }'; then
    echo "ERROR: duration, inflation, episode timeout and stuck window must be positive numbers" >&2
    exit 2
  fi
done
for value in "${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}" \
  "${AUTO_FAILURE_NEXT_GOAL_DELAY_VALUE}" "${AUTO_RECOVERY_STOP_DWELL_VALUE}" \
  "${AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE}" "${AUTO_RELOCATION_ODOM_TIMEOUT_VALUE}" \
  "${AUTO_RELOCATION_ODOM_TOLERANCE_VALUE}" \
  "${AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE}" \
  "${AUTO_HUMAN_COLLISION_PENETRATION_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE}" \
  "${AUTO_LIDAR_EXPECTED_RATE_VALUE}" \
  "${CMD_VEL_ANGULAR_SCALE_VALUE}" \
  "${AUTO_SUPERVISION_MIN_DURATION_VALUE}" \
  "${AUTO_SUPERVISION_MIN_RATE_VALUE}" \
  "${AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE}" \
  "${AUTO_MAX_CMD_AGE_MS_VALUE}" \
  "${AUTO_SUPERVISION_MIN_NONZERO_FRACTION_VALUE}"; do
  if ! awk -v number="${value}" \
    'BEGIN { exit !(number ~ /^[0-9]+([.][0-9]+)?$/ && number >= 0) }'; then
    echo "ERROR: quality and recovery thresholds must be non-negative numbers" >&2
    exit 2
  fi
done
if ! [[ "${AUTO_RELOCATION_AFTER_FAILURES_VALUE}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: AUTO_RELOCATION_AFTER_FAILURES must be a positive integer" >&2
  exit 2
fi
if [[ -z "${ROBOT_RESET_SERVICE_VALUE}" || -z "${ROBOT_ENTITY_NAME_VALUE}" ]]; then
  echo "ERROR: robot reset service and entity name must be non-empty" >&2
  exit 2
fi
if ! awk -v number="${AUTO_ACTUATION_DEADLOCK_GOAL_X_VALUE}" \
  'BEGIN { exit !(number ~ /^-?[0-9]+([.][0-9]+)?$/) }'; then
  echo "ERROR: actuation deadlock goal-x threshold must be numeric" >&2
  exit 2
fi
if ! awk -v number="${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE}" \
  'BEGIN { exit !(number > 0.0 && number <= 1.0) }'; then
  echo "ERROR: actuation deadlock command ratio must be in (0,1]" >&2
  exit 2
fi
if ! awk -v number="${AUTO_SUPERVISION_MIN_NONZERO_FRACTION_VALUE}" \
  'BEGIN { exit !(number > 0.0 && number <= 1.0) }'; then
  echo "ERROR: supervision nonzero-command fraction must be in (0,1]" >&2
  exit 2
fi
if ! awk -v number="${AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE}" \
  'BEGIN { exit !(number > 0.0 && number <= 1.0) }'; then
  echo "ERROR: supervision Person-positive sample fraction must be in (0,1]" >&2
  exit 2
fi
if ! awk -v delay="${INITIAL_GOAL_DELAY_VALUE}" \
  'BEGIN { exit !(delay ~ /^[0-9]+([.][0-9]+)?$/ && delay >= 0.0) }'; then
  echo "ERROR: AUTO_INITIAL_GOAL_DELAY_SEC must be a non-negative number" >&2
  exit 2
fi

# ROS 2 CLI otherwise infers values such as "180" as INTEGER and rejects
# overrides for parameters declared as DOUBLE.
as_ros_double() {
  awk -v number="$1" 'BEGIN { printf "%.9g", number + 0.0; if (number == int(number)) printf ".0" }'
}
CAPTURE_DURATION_VALUE="$(as_ros_double "${CAPTURE_DURATION_VALUE}")"
GOAL_INFLATION_VALUE="$(as_ros_double "${GOAL_INFLATION_VALUE}")"
ROUTE_INFLATION_VALUE="$(as_ros_double "${ROUTE_INFLATION_VALUE}")"
EPISODE_TIMEOUT_VALUE="$(as_ros_double "${EPISODE_TIMEOUT_VALUE}")"
STUCK_WINDOW_VALUE="$(as_ros_double "${STUCK_WINDOW_VALUE}")"
INITIAL_GOAL_DELAY_VALUE="$(as_ros_double "${INITIAL_GOAL_DELAY_VALUE}")"
AUTO_MIN_SUCCESSFUL_DURATION_VALUE="$(as_ros_double "${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}")"
AUTO_FAILURE_NEXT_GOAL_DELAY_VALUE="$(as_ros_double "${AUTO_FAILURE_NEXT_GOAL_DELAY_VALUE}")"
AUTO_RECOVERY_STOP_DWELL_VALUE="$(as_ros_double "${AUTO_RECOVERY_STOP_DWELL_VALUE}")"
AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE="$(as_ros_double "${AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE}")"
AUTO_RELOCATION_ODOM_TIMEOUT_VALUE="$(as_ros_double "${AUTO_RELOCATION_ODOM_TIMEOUT_VALUE}")"
AUTO_RELOCATION_ODOM_TOLERANCE_VALUE="$(as_ros_double "${AUTO_RELOCATION_ODOM_TOLERANCE_VALUE}")"
AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE="$(as_ros_double "${AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE}")"
AUTO_HUMAN_COLLISION_PENETRATION_VALUE="$(as_ros_double "${AUTO_HUMAN_COLLISION_PENETRATION_VALUE}")"
AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE}")"
AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE}")"
AUTO_ACTUATION_DEADLOCK_GOAL_X_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_GOAL_X_VALUE}")"
AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_VALUE}")"
AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE}")"
AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE}")"
AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE="$(as_ros_double "${AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE}")"
AUTO_MAX_CMD_AGE_MS_VALUE="$(as_ros_double "${AUTO_MAX_CMD_AGE_MS_VALUE}")"
AUTO_SUPERVISION_MIN_DURATION_VALUE="$(as_ros_double "${AUTO_SUPERVISION_MIN_DURATION_VALUE}")"
AUTO_SUPERVISION_MIN_RATE_VALUE="$(as_ros_double "${AUTO_SUPERVISION_MIN_RATE_VALUE}")"
AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE="$(as_ros_double "${AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE}")"
CMD_VEL_ANGULAR_SCALE_VALUE="$(as_ros_double "${CMD_VEL_ANGULAR_SCALE_VALUE}")"
for value in "${COMPLETE_BAGS_TARGET}" "${MAX_FAILED_ATTEMPTS}" \
  "${PEDESTRIAN_COUNT_VALUE}" "${AUTO_MIN_SUCCESSFUL_EPISODES_VALUE}" \
  "${AUTO_SUPERVISION_MIN_SAMPLES_VALUE}" \
  "${AUTO_SUPERVISION_SAMPLES_VALUE}" \
  "${AUTO_SUPERVISION_MIN_UNIQUE_COMMANDS_VALUE}"; do
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: bag counts, failure limit and pedestrian count must be integers" >&2
    exit 2
  fi
done
if (( COMPLETE_BAGS_TARGET < 1 || AUTO_MIN_SUCCESSFUL_EPISODES_VALUE < 1 || \
      PEDESTRIAN_COUNT_VALUE < 1 || AUTO_SUPERVISION_MIN_SAMPLES_VALUE < 1 || \
      AUTO_SUPERVISION_MIN_UNIQUE_COMMANDS_VALUE < 1 )); then
  echo "ERROR: bag/episode/pedestrian/sample/command-count quality targets must be positive" >&2
  exit 2
fi
if (( AUTO_SUPERVISION_SAMPLES_VALUE != 2000 )); then
  echo "ERROR: AUTO_SUPERVISION_SAMPLES must be 2000; the launched dual-LiDAR/model contract is fixed at 2000 beams per sensor" >&2
  exit 2
fi
if ! awk -v rate="${AUTO_LIDAR_EXPECTED_RATE_VALUE}" \
  'BEGIN { exit !(rate == 15.0) }'; then
  echo "ERROR: AUTO_LIDAR_EXPECTED_RATE must be 15.0; the launched sensor contract is fixed at 15 Hz" >&2
  exit 2
fi
for value in "${AUTO_CLOCK_STALL_WALL_SEC_VALUE}" \
  "${AUTO_FIRST_EPISODE_WALL_SEC_VALUE}" \
  "${AUTO_MAX_CAPTURE_WALL_SEC_VALUE}" \
  "${AUTO_MIN_FREE_DISK_GIB_VALUE}"; do
  if ! [[ "${value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: wall-time watchdogs and disk low-water mark must be positive integers" >&2
    exit 2
  fi
done
if (( AUTO_MAX_CAPTURE_WALL_SEC_VALUE <= AUTO_FIRST_EPISODE_WALL_SEC_VALUE )); then
  echo "ERROR: AUTO_MAX_CAPTURE_WALL_SEC must exceed AUTO_FIRST_EPISODE_WALL_SEC" >&2
  exit 2
fi
for value in "${AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE}" \
  "${AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE}" \
  "${AUTO_LIDAR_EXPECTED_RATE_VALUE}" \
  "${AUTO_MAX_CMD_AGE_MS_VALUE}" \
  "${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}" \
  "${AUTO_SUPERVISION_MIN_DURATION_VALUE}" \
  "${AUTO_SUPERVISION_MIN_RATE_VALUE}" \
  "${CMD_VEL_ANGULAR_SCALE_VALUE}"; do
  if ! awk -v number="${value}" 'BEGIN { exit !(number > 0.0) }'; then
    echo "ERROR: actuation-deadlock window/response thresholds and LiDAR rate must be positive" >&2
    exit 2
  fi
done
if [[ "${AUTO_CONTINUE_AFTER_FAILURE_VALUE}" == "1" ]]; then
  AUTO_CONTINUE_AFTER_FAILURE_ROS=true
else
  AUTO_CONTINUE_AFTER_FAILURE_ROS=false
fi
for value in "${AUTO_CONTINUE_AFTER_FAILURE_VALUE}" \
  "${AUTO_SUPERVISION_EXPORT_VALUE}" "${AUTO_RESTART_FAILED_ATTEMPTS_VALUE}"; do
  if [[ "${value}" != "0" && "${value}" != "1" ]]; then
    echo "ERROR: automatic recovery/export/restart flags must be 0 or 1" >&2
    exit 2
  fi
done

mkdir -p "${CAPTURE_ROOT}" "${LOG_ROOT}" "${ROS_LOG_DIR_VALUE}" \
  "${MPLCONFIGDIR_VALUE}"
export ROS_LOG_DIR="${ROS_LOG_DIR_VALUE}"
export MPLCONFIGDIR="${MPLCONFIGDIR_VALUE}"
if [[ "${AUTO_SUPERVISION_EXPORT_VALUE}" == "1" ]]; then
  mkdir -p "${SUPERVISION_ROOT}" "${CNN_TRAINING_ROOT}"
fi
if [[ "${AUTO_RECORD_DEMO_VALUE}" == "true" ]]; then
  mkdir -p "${DEMO_ROOT}"
fi

LAUNCH_PID=""
MERGER_PID=""
STAMPER_PID=""
SCHEDULER_PID=""
RECORDER_PID=""
CURRENT_BAG_DIR=""
CURRENT_STATUS_PATH=""
CURRENT_ASSET_STAGING_DIR=""
CURRENT_TRAJECTORY_OUTPUT_DIR=""
CURRENT_BAG_TRAJECTORY_OUTPUT_DIR=""
CURRENT_TRAJECTORIES_ATTEMPTED=0

attach_attempt_diagnostics() {
  if [[ -z "${CURRENT_BAG_DIR}" ]]; then
    return
  fi
  if [[ -d "${CURRENT_ASSET_STAGING_DIR}" ]]; then
    mkdir -p "${CURRENT_BAG_DIR}"
    if [[ -e "${CURRENT_BAG_DIR}/supervision_assets" ]]; then
      echo "WARNING: could not attach staged assets because the bag target exists" >&2
    elif ! mv "${CURRENT_ASSET_STAGING_DIR}" \
      "${CURRENT_BAG_DIR}/supervision_assets"; then
      echo "WARNING: could not attach staged assets to rejected bag" >&2
    fi
  fi
  if [[ -f "${CURRENT_STATUS_PATH}" && -d "${CURRENT_BAG_DIR}" ]]; then
    cp "${CURRENT_STATUS_PATH}" \
      "${CURRENT_BAG_DIR}/auto_capture_status.json" 2>/dev/null || \
      echo "WARNING: could not attach scheduler status to rejected bag" >&2
  fi
}

stop_pid() {
  local signal="$1"
  local pid="$2"
  local grace="${3:-10}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "-${signal}" "${pid}" >/dev/null 2>&1 || true
    for _ in $(seq 1 "${grace}"); do
      kill -0 "${pid}" >/dev/null 2>&1 || break
      sleep 1
    done
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
      sleep 1
    fi
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
    wait "${pid}" >/dev/null 2>&1 || true
  fi
}

stop_group() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 -- "-${pid}" >/dev/null 2>&1; then
    kill -INT -- "-${pid}" >/dev/null 2>&1 || true
    for _ in $(seq 1 10); do
      kill -0 -- "-${pid}" >/dev/null 2>&1 || break
      sleep 1
    done
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL -- "-${pid}" >/dev/null 2>&1 || true
  fi
  [[ -n "${pid}" ]] && wait "${pid}" >/dev/null 2>&1 || true
}

cleanup_attempt() {
  if [[ -n "${LAUNCH_PID}" || -n "${SCHEDULER_PID}" ]]; then
    timeout 3 ros2 topic pub --once /drl_vo/episode_reset \
      std_msgs/msg/Empty '{}' >/dev/null 2>&1 || true
    ros2 param set /drl_vo_fixed_dual_inference \
      publish_policy_actions false >/dev/null 2>&1 || true
  fi
  stop_group "${SCHEDULER_PID}"
  sleep 1
  stop_pid INT "${RECORDER_PID}" 20
  stop_pid INT "${STAMPER_PID}"
  stop_group "${MERGER_PID}"
  stop_group "${LAUNCH_PID}"
  attach_attempt_diagnostics
  LAUNCH_PID=""
  MERGER_PID=""
  STAMPER_PID=""
  SCHEDULER_PID=""
  RECORDER_PID=""
}

cleanup_all() {
  local cleanup_status="$?"
  cleanup_attempt
  # INT/TERM leave start_attempt immediately, bypassing the normal loop path
  # that renders rejected bags.  By this point cleanup_attempt has finalized
  # the recorder and attached immutable assets, so the read-only report is
  # safe to attempt.  The helper's guard prevents duplicate work on ordinary
  # failures that already rendered before reaching the EXIT trap.
  render_current_attempt_trajectories
  return "${cleanup_status}"
}
trap cleanup_all EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

clean_stale_processes() {
  if [[ "${AUTO_CLEAN_STALE_VALUE}" != "1" ]]; then
    return
  fi
  pkill -INT -f '[r]viz2|[i]gnition-gazebo|/usr/bin/[i]gn.*gazebo|[i]gn gazebo|[g]z sim' 2>/dev/null || true
  pkill -INT -f '[a]uto_goal_rosbag_scheduler.py|[c]md_vel_stamper.py|[v]7_dual_laser_scan_merger.py|[d]rl_vo_fixed_dual_start_goal_demo.launch.py|[d]rl_vo_fixed_dual_inference_node.py|[s]emantic_start_goal_path_node.py|[s]cenario_pedestrian_controller.py' 2>/dev/null || true
  sleep 2
}

topic_type() {
  ros2 topic type "$1" 2>/dev/null | head -n 1
}

require_unique_publisher() {
  local topic="$1"
  local expected_node="${2:-}"
  local info count
  for _ in $(seq 1 10); do
    info="$(ros2 topic info -v "${topic}" 2>/dev/null)" || info=""
    count="$(awk '/^Publisher count:/ {print $3; exit}' <<<"${info}")"
    if [[ "${count}" == "1" ]] && \
       { [[ -z "${expected_node}" ]] || \
         grep -Eq "Node name: /?${expected_node}$" <<<"${info}"; }; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${topic} must have exactly one expected publisher" >&2
  ros2 topic info -v "${topic}" >&2 || true
  return 1
}

wait_for_topic() {
  local topic="$1"
  local expected="$2"
  local attempts="${3:-45}"
  for _ in $(seq 1 "${attempts}"); do
    if [[ "$(topic_type "${topic}")" == "${expected}" ]] && \
       timeout 2 ros2 topic echo "${topic}" --once --no-arr >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${topic} did not publish as ${expected}" >&2
  return 1
}

capture_topics_are_fresh() {
  local topic
  for topic in /scan_01 /scan_02 /odom /pedestrian_ground_truth \
    /cmd_vel_stamped /semantic_cnn/local_subgoal; do
    if ! timeout 3 ros2 topic echo "${topic}" --once --no-arr \
      >/dev/null 2>&1; then
      echo "WARNING: no fresh message observed on ${topic}" >&2
      return 1
    fi
  done
}

clock_stamp_ns() {
  local message seconds nanoseconds
  message="$(timeout 3 ros2 topic echo /clock --once --no-arr 2>/dev/null)" || return 1
  seconds="$(awk '/^[[:space:]]*sec:/ {print $2; exit}' <<<"${message}")"
  nanoseconds="$(awk '/^[[:space:]]*nanosec:/ {print $2; exit}' <<<"${message}")"
  [[ "${seconds}" =~ ^[0-9]+$ && "${nanoseconds}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$((seconds * 1000000000 + nanoseconds))"
}

available_disk_kib() {
  local target="$1"
  df -Pk "${target}" 2>/dev/null | awk 'NR == 2 {print $4}'
}

render_current_attempt_trajectories() {
  local map_yaml semantic_label status_json
  local -a status_args=()
  if [[ "${CURRENT_TRAJECTORIES_ATTEMPTED}" == "1" ]]; then
    return 0
  fi
  CURRENT_TRAJECTORIES_ATTEMPTED=1
  if [[ -z "${CURRENT_BAG_DIR}" || ! -f "${CURRENT_BAG_DIR}/metadata.yaml" ]]; then
    echo "WARNING: trajectory visualization skipped because no finalized bag is available" >&2
    return 0
  fi
  map_yaml="${CURRENT_BAG_DIR}/supervision_assets/map.yaml"
  semantic_label="${CURRENT_BAG_DIR}/supervision_assets/label.png"
  if [[ ! -f "${map_yaml}" || ! -f "${semantic_label}" ]]; then
    echo "WARNING: trajectory visualization skipped because attached map assets are unavailable" >&2
    return 0
  fi
  status_json="${CURRENT_BAG_DIR}/auto_capture_status.json"
  if [[ -f "${status_json}" ]]; then
    status_args=(--status-json "${status_json}")
  fi
  if ! python3 "${PROJECT_ROOT}/pipelines/v7_native_pipeline/scripts/visualize_auto_capture_trajectories.py" \
    --bag "${CURRENT_BAG_DIR}" \
    --map-yaml "${map_yaml}" \
    --semantic-label "${semantic_label}" \
    "${status_args[@]}" \
    --output-dir "${CURRENT_TRAJECTORY_OUTPUT_DIR}" \
    --dpi "${AUTO_TRAJECTORY_DPI:-300}"; then
    echo "WARNING: evaluation trajectory visualization failed; capture result is unchanged" >&2
  fi
  if ! python3 "${PROJECT_ROOT}/pipelines/v7_native_pipeline/scripts/visualize_auto_capture_trajectories.py" \
    --bag "${CURRENT_BAG_DIR}" \
    --map-yaml "${map_yaml}" \
    --semantic-label "${semantic_label}" \
    "${status_args[@]}" \
    --output-dir "${CURRENT_BAG_TRAJECTORY_OUTPUT_DIR}" \
    --dpi "${AUTO_TRAJECTORY_DPI:-300}"; then
    echo "WARNING: bag-local trajectory visualization failed; capture result is unchanged" >&2
  fi
  return 0
}

start_attempt() {
  local attempt="$1"
  local pedestrian_seed="$2"
  local goal_seed="$3"
  local stamp step_id bag_dir status_path evaluation_dir trajectory_output_dir bag_trajectory_output_dir
  local asset_staging_dir attempt_map_yaml attempt_semantic_label attempt_label_names
  local attempt_checkpoint training_split_role
  local launch_log merger_log stamper_log scheduler_log recorder_log
  local initial_free_disk_kib
  # Do not allow an early failure in this attempt to reuse artifact paths from
  # the preceding attempt.
  CURRENT_BAG_DIR=""
  CURRENT_STATUS_PATH=""
  CURRENT_ASSET_STAGING_DIR=""
  CURRENT_TRAJECTORY_OUTPUT_DIR=""
  CURRENT_BAG_TRAJECTORY_OUTPUT_DIR=""
  CURRENT_TRAJECTORIES_ATTEMPTED=0
  initial_free_disk_kib="$(available_disk_kib "${CAPTURE_ROOT}")" || \
    initial_free_disk_kib=""
  if ! [[ "${initial_free_disk_kib}" =~ ^[0-9]+$ ]] || \
     (( initial_free_disk_kib < AUTO_MIN_FREE_DISK_GIB_VALUE * 1024 * 1024 )); then
    echo "ERROR: capture filesystem has less than ${AUTO_MIN_FREE_DISK_GIB_VALUE} GiB free (or could not be inspected); refusing to start a new bag" >&2
    return 3
  fi
  stamp="$(date +%Y%m%d_%H%M%S)"
  step_id="${stamp}_drlvo_teacher_auto_p${PEDESTRIAN_COUNT_VALUE}_seed${pedestrian_seed}_a${attempt}"
  bag_dir="${CAPTURE_ROOT}/${step_id}"
  status_path="${CAPTURE_ROOT}/${step_id}_status.json"
  evaluation_dir="${DEMO_ROOT}/${step_id}_demo"
  trajectory_output_dir="${DEMO_ROOT}/${step_id}_trajectory"
  # Keep a self-contained copy with the raw bag, so a completed capture can be
  # inspected without locating the separate evaluation tree.
  bag_trajectory_output_dir="${bag_dir}/trajectory_visualization"
  launch_log="${LOG_ROOT}/${step_id}_launch.log"
  merger_log="${LOG_ROOT}/${step_id}_merger.log"
  stamper_log="${LOG_ROOT}/${step_id}_stamper.log"
  scheduler_log="${LOG_ROOT}/${step_id}_scheduler.log"
  recorder_log="${LOG_ROOT}/${step_id}_recorder.log"
  asset_staging_dir="${CAPTURE_ROOT}/${step_id}_supervision_assets"
  CURRENT_BAG_DIR="${bag_dir}"
  CURRENT_STATUS_PATH="${status_path}"
  CURRENT_ASSET_STAGING_DIR="${asset_staging_dir}"
  CURRENT_TRAJECTORY_OUTPUT_DIR="${trajectory_output_dir}"
  CURRENT_BAG_TRAJECTORY_OUTPUT_DIR="${bag_trajectory_output_dir}"
  training_split_role="${AUTO_CNN_SPLIT_ROLE_VALUE}"
  if [[ "${training_split_role}" == "auto" ]]; then
    local split_bucket
    split_bucket="$(python3 -c \
      'import hashlib,sys; print(hashlib.sha256((sys.argv[1]+":"+sys.argv[2]).encode()).digest()[0] % 10)' \
      "${pedestrian_seed}" "${goal_seed}")"
    case "${split_bucket}" in
      0) training_split_role=dev ;;
      1) training_split_role=test ;;
      *) training_split_role=train ;;
    esac
  fi
  if ! python3 \
    "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/snapshot_supervision_assets.py" \
    --map-yaml "${MAP_YAML_VALUE}" \
    --semantic-label "${SEMANTIC_LABEL_VALUE}" \
    --label-names "${LABEL_NAMES_VALUE}" \
    --output-dir "${asset_staging_dir}" \
    >"${LOG_ROOT}/${step_id}_asset_snapshot.log" 2>&1; then
    echo "ERROR: failed to snapshot supervision assets" >&2
    return 3
  fi
  attempt_map_yaml="${asset_staging_dir}/map.yaml"
  attempt_semantic_label="${asset_staging_dir}/label.png"
  attempt_label_names="${asset_staging_dir}/label_names.txt"
  attempt_checkpoint="${asset_staging_dir}/teacher_checkpoint.pt"
  if ! cp "${CHECKPOINT_VALUE}" "${attempt_checkpoint}"; then
    echo "ERROR: failed to snapshot the teacher checkpoint" >&2
    return 3
  fi
  export IGN_PARTITION="semantic_nav_auto_${stamp}_${attempt}_$$"
  export GZ_PARTITION="${IGN_PARTITION}"

  echo "===== AUTO CAPTURE ATTEMPT ${attempt} ====="
  echo "Bag: ${bag_dir}"
  echo "Pedestrian seed: ${pedestrian_seed}; goal seed: ${goal_seed}"
  echo "Requested capture duration: ${CAPTURE_DURATION_MIN_VALUE} min (${CAPTURE_DURATION_VALUE} sim sec)"
  echo "Duration is a soft deadline: an active episode will finish before the bag stops."
  if [[ "${AUTO_RECORD_DEMO_VALUE}" == "true" ]]; then
    echo "Demo metrics: ${evaluation_dir}"
  else
    echo "Demo metrics: disabled (set AUTO_RECORD_DEMO=1 to enable)"
  fi
  echo "Episode trajectory visualization (evaluation): ${trajectory_output_dir}"
  echo "Episode trajectory visualization (bag): ${bag_trajectory_output_dir}"
  echo "Goal clearance: ${GOAL_INFLATION_VALUE} m; route inflation: ${ROUTE_INFLATION_VALUE} m"
  echo "Immutable supervision assets: ${asset_staging_dir}"
  echo "SemanticCNN seed-level split role: ${training_split_role}"

  setsid ros2 launch semantic_nav_gazebo drl_vo_fixed_dual_start_goal_demo.launch.py \
    policy_mode:=base \
    drl_vo_model:="${attempt_checkpoint}" \
    drl_vo_python:="${TORCH_PY}" \
    device:="${AUTO_DRLVO_RESOLVED_DEVICE_VALUE}" \
    publish_policy_actions:=false \
    pedestrian_source:=oracle \
    oracle_pedestrian_velocity:=true \
    require_pedestrian_truth:=true \
    world:=gazebo_eng_lobby.world \
    scene_file:=scenarios/lobby/eng_hall_15.xml \
    gui:="${GUI_VALUE}" \
    start_rviz:="${START_RVIZ_VALUE}" \
    use_sim_time:=true \
    map_yaml:="${attempt_map_yaml}" \
    semantic_label:="${attempt_semantic_label}" \
    robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
    enable_goal_picker:=false auto_set_initial_goal:=false \
    spawn_scene_pedestrians:=true \
    pedestrian_count:="${PEDESTRIAN_COUNT_VALUE}" \
    pedestrian_speed:=1.0 pedestrian_seed:="${pedestrian_seed}" \
    pedestrian_use_actors:=false \
    cmd_vel_topic:=/cmd_vel \
    cmd_vel_angular_z_scale:="${CMD_VEL_ANGULAR_SCALE_VALUE}" \
    max_linear:=0.99 max_angular:=1.99 \
    goal_tolerance:=0.35 front_stop_distance:=0.01 \
    stop_on_empty_front:=false lookahead:=1.0 \
    enable_actuation_deadlock_detection:=true \
    actuation_deadlock_window_sec:="${AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE}" \
    actuation_deadlock_min_command_ratio:="${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE}" \
    actuation_deadlock_goal_x_threshold:="${AUTO_ACTUATION_DEADLOCK_GOAL_X_VALUE}" \
    actuation_deadlock_max_linear_command:="${AUTO_ACTUATION_DEADLOCK_MAX_LINEAR_VALUE}" \
    actuation_deadlock_min_angular_command:="${AUTO_ACTUATION_DEADLOCK_MIN_ANGULAR_VALUE}" \
    actuation_deadlock_max_displacement_m:="${AUTO_ACTUATION_DEADLOCK_MAX_DISPLACEMENT_VALUE}" \
    actuation_deadlock_max_yaw_progress_rad:="${AUTO_ACTUATION_DEADLOCK_MAX_YAW_VALUE}" \
    inflate_radius:="${ROUTE_INFLATION_VALUE}" \
    show_actual_trajectory:=false \
    evaluate_episode:="${AUTO_RECORD_DEMO_VALUE}" \
    evaluation_output_dir:="${evaluation_dir}" \
    evaluation_timeout_sec:="${EPISODE_TIMEOUT_VALUE}" \
    evaluation_multi_episode:=true \
    experiment_scene_id:="${step_id}" \
    record_trace:=false start_online_ppo_training:=false start_auto_capture:=true \
    robot_reset_service:="${ROBOT_RESET_SERVICE_VALUE}" \
    robot_entity_name:="${ROBOT_ENTITY_NAME_VALUE}" \
    >"${launch_log}" 2>&1 &
  LAUNCH_PID="$!"

  wait_for_topic /scan_01 sensor_msgs/msg/LaserScan 60 || return 3
  wait_for_topic /scan_02 sensor_msgs/msg/LaserScan 20 || return 3
  wait_for_topic /odom nav_msgs/msg/Odometry 20 || return 3
  wait_for_topic /clock rosgraph_msgs/msg/Clock 20 || return 3
  wait_for_topic /pedestrian_ground_truth semantic_nav_gazebo/msg/PedestrianStateArray 20 || return 3
  require_unique_publisher /cmd_vel drl_vo_fixed_dual_inference || return 3
  require_unique_publisher /scan_01 v7_dual_lidar_bridge || return 3
  require_unique_publisher /scan_02 v7_dual_lidar_bridge || return 3
  require_unique_publisher /odom || return 3
  require_unique_publisher /pedestrian_ground_truth scenario_pedestrian_controller || return 3
  require_unique_publisher /semantic_cnn/local_subgoal drl_vo_start_goal_path || return 3

  setsid ros2 run semantic_nav_gazebo v7_dual_laser_scan_merger.py --ros-args \
    --params-file "${ROS_WS}/src/semantic_nav_gazebo/config/v7_dual_laser_scan_merger.yaml" \
    >"${merger_log}" 2>&1 &
  MERGER_PID="$!"
  wait_for_topic /scan_merged sensor_msgs/msg/LaserScan 20 || return 3

  python3 "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/cmd_vel_stamper.py" \
    >"${stamper_log}" 2>&1 &
  STAMPER_PID="$!"
  for _ in $(seq 1 15); do
    [[ "$(topic_type /cmd_vel_stamped)" == "geometry_msgs/msg/TwistStamped" ]] && break
    sleep 1
  done
  if [[ "$(topic_type /cmd_vel_stamped)" != "geometry_msgs/msg/TwistStamped" ]]; then
    echo "ERROR: cmd_vel stamper failed to advertise" >&2
    return 3
  fi
  local stamped_publishers
  stamped_publishers="$(ros2 topic info /cmd_vel_stamped 2>/dev/null | awk '/^Publisher count:/ {print $3}')"
  if [[ "${stamped_publishers}" != "1" ]]; then
    echo "ERROR: /cmd_vel_stamped has ${stamped_publishers:-unknown} publishers; refusing ambiguous labels" >&2
    ros2 topic info -v /cmd_vel_stamped >&2 || true
    return 3
  fi

  setsid ros2 run semantic_nav_gazebo auto_goal_rosbag_scheduler.py --ros-args \
    -p use_sim_time:=true \
    -p capture_enabled:=false \
    -p map_yaml:="${attempt_map_yaml}" \
    -p status_path:="${status_path}" \
    -p seed:="${goal_seed}" \
    -p capture_duration_sec:="${CAPTURE_DURATION_VALUE}" \
    -p initial_goal_delay_sec:="${INITIAL_GOAL_DELAY_VALUE}" \
    -p goal_inflation_radius:="${GOAL_INFLATION_VALUE}" \
    -p route_inflation_radius:="${ROUTE_INFLATION_VALUE}" \
    -p episode_timeout_sec:="${EPISODE_TIMEOUT_VALUE}" \
    -p stuck_window_sec:="${STUCK_WINDOW_VALUE}" \
    -p continue_after_episode_failure:="${AUTO_CONTINUE_AFTER_FAILURE_ROS}" \
    -p minimum_successful_episodes:="${AUTO_MIN_SUCCESSFUL_EPISODES_VALUE}" \
    -p minimum_successful_duration_sec:="${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}" \
    -p failure_next_goal_delay_sec:="${AUTO_FAILURE_NEXT_GOAL_DELAY_VALUE}" \
    -p recovery_stop_dwell_sec:="${AUTO_RECOVERY_STOP_DWELL_VALUE}" \
    -p robot_reset_service:="${ROBOT_RESET_SERVICE_VALUE}" \
    -p robot_entity_name:="${ROBOT_ENTITY_NAME_VALUE}" \
    -p relocation_after_failures:="${AUTO_RELOCATION_AFTER_FAILURES_VALUE}" \
    -p relocation_service_timeout_sec:="${AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE}" \
    -p relocation_odom_timeout_sec:="${AUTO_RELOCATION_ODOM_TIMEOUT_VALUE}" \
    -p relocation_odom_tolerance_m:="${AUTO_RELOCATION_ODOM_TOLERANCE_VALUE}" \
    -p human_collision_confirmation_sec:="${AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE}" \
    -p human_collision_penetration_m:="${AUTO_HUMAN_COLLISION_PENETRATION_VALUE}" \
    -p short_weight:="${AUTO_SHORT_WEIGHT:-0.15}" \
    -p medium_weight:="${AUTO_MEDIUM_WEIGHT:-0.45}" \
    -p long_weight:="${AUTO_LONG_WEIGHT:-0.40}" \
    >"${scheduler_log}" 2>&1 &
  SCHEDULER_PID="$!"

  ros2 bag record --storage sqlite3 -o "${bag_dir}" \
    /scan_merged /scan_01 /scan_02 /odom /tf /tf_static \
    /cmd_vel /cmd_vel_stamped /clock /pedestrian_ground_truth \
    /semantic_cnn/global_path /semantic_cnn/local_subgoal /semantic_cnn/final_goal \
    /data_collection/goal_accepted /data_collection/episode_event \
    /data_collection/auto_capture_status /drl_vo/raw_model_cmd \
    /drl_vo/control_event /drl_vo/episode_reset \
    /navigation_evaluation/inference_metrics \
    >"${recorder_log}" 2>&1 &
  RECORDER_PID="$!"

  local recorder_ready=0
  for _ in $(seq 1 20); do
    if ! kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
      echo "ERROR: rosbag recorder exited; see ${recorder_log}" >&2
      return 3
    fi
    if grep -q "All requested topics are subscribed" "${recorder_log}"; then
      recorder_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${recorder_ready}" != "1" ]]; then
    echo "ERROR: rosbag recorder startup timed out; see ${recorder_log}" >&2
    return 3
  fi
  if ! ros2 param set /auto_goal_rosbag_scheduler \
    capture_enabled true >/dev/null; then
    echo "ERROR: could not arm the goal scheduler after recorder startup" >&2
    return 3
  fi
  local capture_armed_wall
  capture_armed_wall="${SECONDS}"

  local goal_ready=0
  for _ in $(seq 1 30); do
    if ! kill -0 "${SCHEDULER_PID}" >/dev/null 2>&1; then
      echo "ERROR: goal scheduler exited before accepting a goal" >&2
      tail -n 40 "${scheduler_log}" >&2 || true
      return 3
    fi
    if grep -q "AUTO_GOAL_ACCEPTED" "${scheduler_log}"; then
      goal_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${goal_ready}" != "1" ]]; then
    echo "ERROR: machine-selected goal was not accepted" >&2
    return 3
  fi
  if ! ros2 param set /drl_vo_fixed_dual_inference publish_policy_actions true; then
    echo "ERROR: could not enable DRL-VO teacher actions" >&2
    return 3
  fi
  echo "AUTO_CAPTURE_RUNNING: unattended teacher control is enabled."

  local scheduler_code process_pid process_name current_clock_ns
  local last_clock_ns last_clock_progress_wall last_clock_check_wall
  local last_topic_health_wall topic_health_failures episode_started
  local last_disk_check_wall free_disk_kib minimum_free_disk_kib
  last_clock_ns="$(clock_stamp_ns)" || last_clock_ns=0
  last_clock_progress_wall="${SECONDS}"
  last_clock_check_wall="${SECONDS}"
  last_topic_health_wall="${SECONDS}"
  last_disk_check_wall="${SECONDS}"
  topic_health_failures=0
  episode_started=0
  minimum_free_disk_kib=$((AUTO_MIN_FREE_DISK_GIB_VALUE * 1024 * 1024))
  while kill -0 "${SCHEDULER_PID}" >/dev/null 2>&1; do
    for process_name in recorder launch merger stamper; do
      case "${process_name}" in
        recorder) process_pid="${RECORDER_PID}" ;;
        launch) process_pid="${LAUNCH_PID}" ;;
        merger) process_pid="${MERGER_PID}" ;;
        stamper) process_pid="${STAMPER_PID}" ;;
      esac
      if [[ -z "${process_pid}" ]] || ! kill -0 "${process_pid}" >/dev/null 2>&1; then
        echo "ERROR: ${process_name} process exited during capture; preserving the current bag" >&2
        return 3
      fi
    done
    if (( SECONDS - capture_armed_wall >= AUTO_MAX_CAPTURE_WALL_SEC_VALUE )); then
      echo "ERROR: capture exceeded the ${AUTO_MAX_CAPTURE_WALL_SEC_VALUE}s wall-time safety limit; finalizing and preserving the current bag" >&2
      return 3
    fi
    if (( episode_started == 0 )); then
      if grep -q "AUTO_EPISODE_STARTED" "${scheduler_log}"; then
        episode_started=1
      elif (( SECONDS - capture_armed_wall >= AUTO_FIRST_EPISODE_WALL_SEC_VALUE )); then
        echo "ERROR: no episode emitted a nonzero teacher command within ${AUTO_FIRST_EPISODE_WALL_SEC_VALUE}s after recorder arming; finalizing and preserving the current bag" >&2
        return 3
      fi
    fi
    if (( SECONDS - last_clock_check_wall >= 5 )); then
      last_clock_check_wall="${SECONDS}"
      if current_clock_ns="$(clock_stamp_ns)" && (( current_clock_ns > last_clock_ns )); then
        last_clock_ns="${current_clock_ns}"
        last_clock_progress_wall="${SECONDS}"
      elif (( SECONDS - last_clock_progress_wall >= AUTO_CLOCK_STALL_WALL_SEC_VALUE )); then
        echo "ERROR: /clock made no progress for ${AUTO_CLOCK_STALL_WALL_SEC_VALUE}s; preserving the current bag" >&2
        return 3
      fi
    fi
    if (( SECONDS - last_topic_health_wall >= 10 )); then
      last_topic_health_wall="${SECONDS}"
      if capture_topics_are_fresh; then
        topic_health_failures=0
      else
        topic_health_failures=$((topic_health_failures + 1))
        if (( topic_health_failures >= 2 )); then
          echo "ERROR: required capture topics failed two consecutive freshness checks; preserving the current bag" >&2
          return 3
        fi
      fi
    fi
    if (( SECONDS - last_disk_check_wall >= 10 )); then
      last_disk_check_wall="${SECONDS}"
      free_disk_kib="$(available_disk_kib "${bag_dir}")" || free_disk_kib=""
      if ! [[ "${free_disk_kib}" =~ ^[0-9]+$ ]]; then
        echo "ERROR: could not determine free space for the active bag; finalizing and preserving it" >&2
        return 3
      fi
      if (( free_disk_kib < minimum_free_disk_kib )); then
        echo "ERROR: free disk space fell below ${AUTO_MIN_FREE_DISK_GIB_VALUE} GiB; finalizing and preserving the current bag" >&2
        return 3
      fi
    fi
    sleep 1
  done
  if wait "${SCHEDULER_PID}"; then
    scheduler_code=0
  else
    scheduler_code="$?"
  fi
  SCHEDULER_PID=""
  timeout 3 ros2 topic pub --once /drl_vo/episode_reset \
    std_msgs/msg/Empty '{}' >/dev/null 2>&1 || true
  ros2 param set /drl_vo_fixed_dual_inference \
    publish_policy_actions false >/dev/null 2>&1 || true
  sleep 1
  stop_pid INT "${RECORDER_PID}" 20
  RECORDER_PID=""
  if [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
    echo "ERROR: recorder did not finalize metadata; raw DB files were preserved" >&2
    cleanup_attempt
    return 5
  fi
  cleanup_attempt
  if [[ ! -d "${bag_dir}/supervision_assets" ]]; then
    echo "ERROR: immutable assets were not attached to the finalized bag" >&2
    return 5
  fi
  attempt_map_yaml="${bag_dir}/supervision_assets/map.yaml"
  attempt_semantic_label="${bag_dir}/supervision_assets/label.png"
  attempt_label_names="${bag_dir}/supervision_assets/label_names.txt"
  attempt_checkpoint="${bag_dir}/supervision_assets/teacher_checkpoint.pt"
  if [[ ! -f "${status_path}" ]] || (( scheduler_code != 0 )) || \
     ! python3 -c 'import json,sys; s=json.load(open(sys.argv[1])); raise SystemExit(not (s.get("outcome") == "complete" and s.get("duration_deadline_reached") is True and s.get("quality_quota_met") is True))' "${status_path}"; then
    echo "AUTO_CAPTURE_REJECTED: scheduler/status quality gate failed (code ${scheduler_code}); bag preserved at ${bag_dir}" >&2
    return 4
  fi
  if ! cp "${status_path}" "${bag_dir}/auto_capture_status.json"; then
    echo "ERROR: could not attach final scheduler status to bag" >&2
    return 5
  fi
  if ! python3 \
    "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/write_auto_capture_contract.py" \
    --bag "${bag_dir}" \
    --status-json "${bag_dir}/auto_capture_status.json" \
    --asset-manifest "${bag_dir}/supervision_assets/manifest.json" \
    --checkpoint "${attempt_checkpoint}" \
    --output "${bag_dir}/capture_contract.json" \
    --step-id "${step_id}" \
    --pedestrian-seed "${pedestrian_seed}" \
    --goal-seed "${goal_seed}" \
    --pedestrian-count "${PEDESTRIAN_COUNT_VALUE}" \
    --setting "capture_duration_sec=${CAPTURE_DURATION_VALUE}" \
    --setting "minimum_successful_episodes=${AUTO_MIN_SUCCESSFUL_EPISODES_VALUE}" \
    --setting "minimum_successful_duration_sec=${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}" \
    --setting "continue_after_episode_failure=${AUTO_CONTINUE_AFTER_FAILURE_ROS}" \
    --setting "robot_reset_service=${ROBOT_RESET_SERVICE_VALUE}" \
    --setting "robot_entity_name=${ROBOT_ENTITY_NAME_VALUE}" \
    --setting "relocation_after_failures=${AUTO_RELOCATION_AFTER_FAILURES_VALUE}" \
    --setting "relocation_target=2.0,2.0,0.0" \
    --setting "relocation_service_timeout_sec=${AUTO_RELOCATION_SERVICE_TIMEOUT_VALUE}" \
    --setting "relocation_odom_timeout_sec=${AUTO_RELOCATION_ODOM_TIMEOUT_VALUE}" \
    --setting "relocation_odom_tolerance_m=${AUTO_RELOCATION_ODOM_TOLERANCE_VALUE}" \
    --setting "requested_device=${AUTO_DRLVO_DEVICE_VALUE}" \
    --setting "resolved_device=${AUTO_DRLVO_RESOLVED_DEVICE_VALUE}" \
    --setting "cmd_label_interface=pre_relay_ros_cmd_vel" \
    --setting "cmd_vel_angular_z_relay_scale=${CMD_VEL_ANGULAR_SCALE_VALUE}" \
    --setting "maximum_cmd_vel_label_age_ms=${AUTO_MAX_CMD_AGE_MS_VALUE}" \
    --setting "minimum_supervision_effective_duration_sec=${AUTO_SUPERVISION_MIN_DURATION_VALUE}" \
    --setting "minimum_supervision_effective_rate_hz=${AUTO_SUPERVISION_MIN_RATE_VALUE}" \
    --setting "semantic_cnn_seed_level_split_role=${training_split_role}" \
    --setting "actuation_deadlock_window_sec=${AUTO_ACTUATION_DEADLOCK_WINDOW_VALUE}" \
    --setting "actuation_deadlock_min_command_ratio=${AUTO_ACTUATION_DEADLOCK_MIN_COMMAND_RATIO_VALUE}" \
    --setting "human_collision_confirmation_sec=${AUTO_HUMAN_COLLISION_CONFIRMATION_VALUE}" \
    >"${LOG_ROOT}/${step_id}_capture_contract.log" 2>&1; then
    echo "ERROR: could not seal the capture reproducibility contract" >&2
    return 5
  fi
  if [[ "${AUTO_VALIDATE_VALUE}" == "1" || "${AUTO_SUPERVISION_EXPORT_VALUE}" == "1" ]]; then
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py" \
      --bag "${bag_dir}" \
      --status-json "${status_path}" \
      --require-complete-status \
      --minimum-successful-episodes "${AUTO_MIN_SUCCESSFUL_EPISODES_VALUE}" \
      --minimum-successful-duration-sec "${AUTO_MIN_SUCCESSFUL_DURATION_VALUE}"; then
      echo "ERROR: strict episode-event quality gate failed; bag preserved" >&2
      return 5
    fi
  fi
  # Rendering is nonfatal and runs before any supervision export.  The same
  # helper is also invoked after cleanup for rejected/interrupted attempts.
  render_current_attempt_trajectories
  if [[ "${AUTO_RECORD_DEMO_VALUE}" == "true" ]]; then
    if [[ ! -f "${evaluation_dir}/session_summary.json" ]]; then
      echo "WARNING: demo evaluation session summary is missing; supervision export will continue" >&2
    fi
  fi

  echo "Bag finalized: ${bag_dir}"
  echo "Status: ${status_path}"
  if [[ "${AUTO_RECORD_DEMO_VALUE}" == "true" ]]; then
    echo "Demo metrics finalized: ${evaluation_dir}"
  fi
  echo "Trajectory visualization finalized: ${trajectory_output_dir}"
  echo "Bag-local trajectory visualization finalized: ${bag_trajectory_output_dir}"
  if [[ "${AUTO_SUPERVISION_EXPORT_VALUE}" == "1" ]]; then
    local supervision_session_name supervision_session_dir quality_report
    supervision_session_name="${step_id}_cnn_supervision"
    supervision_session_dir="${SUPERVISION_ROOT}/${supervision_session_name}"
    quality_report="${supervision_session_dir}/quality_report.json"
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py" \
      bag --bag "${bag_dir}" --samples "${AUTO_SUPERVISION_SAMPLES_VALUE}" \
      --rate "${AUTO_LIDAR_EXPECTED_RATE_VALUE}" --range-min 0.1 --range-max 8.0; then
      echo "ERROR: raw dual-LiDAR contract failed; bag preserved" >&2
      return 5
    fi
    if ! python3 "${ROS_WS}/tools/check_cmd_vel_stamped_bag.py" \
      --bag "${bag_dir}" --episode-aware; then
      echo "ERROR: causal executed-command contract failed; bag preserved" >&2
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py" \
      --bag "${bag_dir}" \
      --output-root "${SUPERVISION_ROOT}" \
      --session-name "${supervision_session_name}" \
      --map-yaml "${attempt_map_yaml}" \
      --semantic-label "${attempt_semantic_label}" \
      --samples-01 "${AUTO_SUPERVISION_SAMPLES_VALUE}" \
      --samples-02 "${AUTO_SUPERVISION_SAMPLES_VALUE}" \
      --pose-source auto \
      --cmd-vel-max-age-ms "${AUTO_MAX_CMD_AGE_MS_VALUE}" \
      --cmd-label-interface pre-relay_ros_cmd_vel \
      --cmd-vel-angular-z-relay-scale "${CMD_VEL_ANGULAR_SCALE_VALUE}" \
      --subgoal-source online --subgoal-max-age-ms 300 \
      --successful-episodes-only \
      --person-label-mode ground-truth-legs \
      --self-mask-mode first-synchronized-pair-fixed-beam-identity \
      --exclude-reverse-linear-x --reverse-recovery-frames 15; then
      echo "ERROR: CNN supervision conversion failed; bag preserved" >&2
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py" \
      --session "${supervision_session_dir}" \
      --report-json "${quality_report}" \
      --minimum-samples "${AUTO_SUPERVISION_MIN_SAMPLES_VALUE}" \
      --minimum-duration-sec "${AUTO_SUPERVISION_MIN_DURATION_VALUE}" \
      --minimum-unique-command-vectors "${AUTO_SUPERVISION_MIN_UNIQUE_COMMANDS_VALUE}" \
      --minimum-nonzero-command-fraction "${AUTO_SUPERVISION_MIN_NONZERO_FRACTION_VALUE}" \
      --minimum-effective-sample-rate-hz "${AUTO_SUPERVISION_MIN_RATE_VALUE}" \
      --minimum-person-positive-sample-fraction "${AUTO_SUPERVISION_MIN_PERSON_POSITIVE_FRACTION_VALUE}" \
      --maximum-subgoal-age-ms 300 \
      --maximum-cmd-vel-age-ms "${AUTO_MAX_CMD_AGE_MS_VALUE}" \
      --maximum-person-truth-unmatched-samples 0 \
      --require-online-subgoal --require-successful-episodes-only \
      --require-ground-truth-person-labels --require-person-observations \
      --require-forward-only --require-pre-relay-command-labels \
      --fail-on-warnings; then
      echo "ERROR: CNN supervision quality gate failed; bag and diagnostic dataset preserved" >&2
      return 5
    fi
    if ! cp "${bag_dir}/capture_contract.json" \
      "${supervision_session_dir}/capture_contract.json"; then
      echo "ERROR: could not attach capture provenance to supervision dataset" >&2
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/seal_cnn_supervision_dataset.py" \
      --session "${supervision_session_dir}" \
      --quality-report "${quality_report}" \
      --capture-contract "${supervision_session_dir}/capture_contract.json" \
      --output "${supervision_session_dir}/QUALITY_PASS.json" \
      >"${LOG_ROOT}/${step_id}_supervision_seal.log" 2>&1; then
      echo "ERROR: could not hash and atomically seal the approved supervision dataset" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_supervision_seal.log" >&2 || true
      return 5
    fi
    local training_name training_dir training_staging training_report
    training_name="${step_id}-v7-fixed-dual-v3-2000x2000-training-pedgt-v1-sgonline"
    training_dir="${CNN_TRAINING_ROOT}/${training_name}"
    training_staging="${CNN_TRAINING_ROOT}/.${training_name}.tmp.$$"
    training_report="${training_dir}/quality_report.json"
    if [[ -e "${training_dir}" || -e "${training_staging}" ]]; then
      echo "ERROR: SemanticCNN training target/staging already exists" >&2
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py" \
      --source-session "${supervision_session_dir}" \
      --output-session "${training_staging}" \
      --session-name "${training_name}" \
      --pool-range-max 8.0 \
      --frame-period-tolerance-ms 20.0 \
      --split-role "${training_split_role}" \
      >"${LOG_ROOT}/${step_id}_semantic2d_export.log" 2>&1; then
      echo "ERROR: SemanticCNN-native export failed; fixed-slot supervision was preserved" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_semantic2d_export.log" >&2 || true
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py" \
      --session "${training_staging}" \
      --source-session "${supervision_session_dir}" \
      --dataset-root "${CNN_TRAINING_ROOT}" \
      --expected-rate-01 15.0 --expected-rate-02 15.0 \
      --rate-tolerance-percent 10.0 \
      >"${LOG_ROOT}/${step_id}_semantic2d_staging_check.log" 2>&1; then
      echo "ERROR: staged SemanticCNN-native dataset failed validation" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_semantic2d_staging_check.log" >&2 || true
      return 5
    fi
    if ! mv "${training_staging}" "${training_dir}"; then
      echo "ERROR: could not commit the checked SemanticCNN-native session" >&2
      return 5
    fi
    if ! cp "${supervision_session_dir}/QUALITY_PASS.json" \
      "${training_dir}/source_QUALITY_PASS.json" || \
       ! cp "${supervision_session_dir}/capture_contract.json" \
      "${training_dir}/capture_contract.json"; then
      echo "ERROR: could not attach source provenance to training session" >&2
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py" \
      --session "${training_dir}" \
      --source-session "${supervision_session_dir}" \
      --dataset-root "${CNN_TRAINING_ROOT}" \
      --expected-rate-01 15.0 --expected-rate-02 15.0 \
      --rate-tolerance-percent 10.0 \
      --report-json "${training_report}" \
      >"${LOG_ROOT}/${step_id}_semantic2d_final_check.log" 2>&1; then
      echo "ERROR: committed SemanticCNN-native dataset failed validation" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_semantic2d_final_check.log" >&2 || true
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/seal_semantic2d_training_session.py" \
      --session "${training_dir}" \
      --source-session "${supervision_session_dir}" \
      --quality-report "${training_report}" \
      --source-approval "${training_dir}/source_QUALITY_PASS.json" \
      --capture-contract "${training_dir}/capture_contract.json" \
      --output "${training_dir}/CNN_READY.json" \
      >"${LOG_ROOT}/${step_id}_semantic2d_seal.log" 2>&1; then
      echo "ERROR: could not hash and seal SemanticCNN-native training data" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_semantic2d_seal.log" >&2 || true
      return 5
    fi
    if ! python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/register_semantic2d_session.py" \
      --dataset-root "${CNN_TRAINING_ROOT}" \
      --session "${training_dir}" \
      >"${LOG_ROOT}/${step_id}_semantic2d_register.log" 2>&1; then
      echo "ERROR: sealed training session could not be registered atomically" >&2
      tail -n 40 "${LOG_ROOT}/${step_id}_semantic2d_register.log" >&2 || true
      return 5
    fi
    echo "Fixed-slot supervision approved: ${supervision_session_dir}"
    echo "SemanticCNN training session approved: ${training_dir}"
  fi
  echo "AUTO_CAPTURE_COMPLETE: ${bag_dir}"
  LAST_AUTO_BAG_DIR="${bag_dir}"
  export LAST_AUTO_BAG_DIR
  return 0
}

clean_stale_processes
completed=0
failed=0
attempt=0
while (( completed < COMPLETE_BAGS_TARGET )); do
  attempt=$((attempt + 1))
  pedestrian_seed=$((PEDESTRIAN_SEED_START_VALUE + attempt - 1))
  goal_seed=$((GOAL_SEED_START_VALUE + attempt - 1))
  if start_attempt "${attempt}" "${pedestrian_seed}" "${goal_seed}"; then
    result=0
  else
    result="$?"
  fi
  cleanup_attempt
  if [[ "${result}" != "0" ]]; then
    render_current_attempt_trajectories
  fi
  if [[ "${result}" == "0" ]]; then
    completed=$((completed + 1))
  else
    failed=$((failed + 1))
    if [[ "${result}" == "5" ]]; then
      echo "ERROR: post-capture quality/export failed; refusing to recollect a completed bag" >&2
      exit 5
    fi
    if [[ "${AUTO_RESTART_FAILED_ATTEMPTS_VALUE}" != "1" ]]; then
      echo "ERROR: attempt failed with code ${result}; automatic new-bag restart is disabled" >&2
      exit "${result}"
    fi
    if (( failed >= MAX_FAILED_ATTEMPTS )); then
      echo "ERROR: reached ${MAX_FAILED_ATTEMPTS} failed attempts; stopping safely" >&2
      exit 4
    fi
    sleep 2
  fi
done

trap - EXIT INT TERM
echo "ALL_AUTO_CAPTURES_COMPLETE completed=${completed} failed_attempts=${failed}"
echo "Latest bag: ${LAST_AUTO_BAG_DIR}"
