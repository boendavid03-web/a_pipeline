#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--allow-odom-map-alignment, --bag, --dev-ratio, --episode-aware, --exclude-reverse-linear-x, --map-yaml, --no-clobber, --output-root, --overwrite, --person-ground-truth-leg-match-radius-m, --person-ground-truth-max-delta-ms, --person-ground-truth-radius-m, --person-label-mode, --pose-source, --range-max, --range-min, --rate, --report-json, --reverse-linear-x-epsilon, --reverse-recovery-frames, --samples, --samples-01, --samples-02, --self-mask-mode, --semantic-label, --session, --session-name, --split-seed, --subgoal-lookahead, --subgoal-max-age-ms, --subgoal-source, --sync-tolerance-ms, --test-ratio, --train-ratio
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PNG, TXT, YAML
# 可能使用的关键环境变量：ALLOW_ODOM_ALIGNMENT, BAG_DIR, BAG_TS, BASH_SOURCE, BEGIN, CHECKER, CHECK_JSON, CHECK_LOG, CONVERTER, DEV_RATIO, DUAL_SLOT_ACTUAL_LABEL_NAMES, DUAL_SLOT_ACTUAL_MAP_YAML, DUAL_SLOT_ACTUAL_SEMANTIC_LABEL, DUAL_SLOT_ALLOW_ODOM_MAP_ALIGNMENT, DUAL_SLOT_ALLOW_OVERWRITE, DUAL_SLOT_BAG_DIR, DUAL_SLOT_CHECK_REPORT, DUAL_SLOT_CONTRACT, DUAL_SLOT_CONVERSION_LOG, DUAL_SLOT_DATASET_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:35.854867558 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py（执行该脚本，使用其输出继续当前流程）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py（执行该脚本，使用其输出继续当前流程）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07b_convert_bag_to_fixed_dual_lidar.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
PERSON_MODE_OVERRIDE="${DUAL_SLOT_PERSON_LABEL_MODE:-}"
load_manifest "${RUN_MANIFEST:-}"

safe_source_ros

if [[ -n "${DUAL_SLOT_BAG_DIR:-}" ]]; then
  BAG_DIR="${DUAL_SLOT_BAG_DIR}"
fi
require_dir "${BAG_DIR}" "ROS 2 bag directory"

SAMPLES_01="${DUAL_SLOT_SAMPLES_01:-${LIDAR_SAMPLES_01:-${LIDAR_SAMPLES:-360}}}"
SAMPLES_02="${DUAL_SLOT_SAMPLES_02:-${LIDAR_SAMPLES_02:-${LIDAR_SAMPLES:-360}}}"
if ! [[ "${SAMPLES_01}" =~ ^[1-9][0-9]*$ && "${SAMPLES_02}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: dual-slot sample counts must be positive integers" >&2
  exit 2
fi

MAP_DIR="${RUN_ROOT}/maps/semantic_label"
MAP_YAML="${MAP_DIR}/map.yaml"
SEMANTIC_LABEL="${MAP_DIR}/label.png"
LABEL_NAMES="${MAP_DIR}/label_names.txt"
if [[ ! -f "${MAP_YAML}" || ! -f "${SEMANTIC_LABEL}" ]]; then
  SOURCE_DIR="${DUAL_SLOT_MAP_SOURCE_DIR:-}"
  if [[ -z "${SOURCE_DIR}" ]]; then
    echo "ERROR: map assets are missing under ${MAP_DIR}." >&2
    echo "Set DUAL_SLOT_MAP_SOURCE_DIR to a verified semantic_label directory." >&2
    exit 2
  fi
  require_dir "${SOURCE_DIR}" "dual-slot source semantic map directory"
  require_file "${SOURCE_DIR}/map.yaml" "source semantic map yaml"
  require_file "${SOURCE_DIR}/label.png" "source semantic label png"
  mkdir -p "${MAP_DIR}"
  cp -a --no-clobber "${SOURCE_DIR}/." "${MAP_DIR}/"
fi
require_file "${MAP_YAML}" "dual-slot semantic map yaml"
require_file "${SEMANTIC_LABEL}" "dual-slot semantic label png"
require_file "${LABEL_NAMES}" "dual-slot semantic label names"

POSE_SOURCE="${DUAL_SLOT_POSE_SOURCE:-auto}"
ALLOW_ODOM_ALIGNMENT="${DUAL_SLOT_ALLOW_ODOM_MAP_ALIGNMENT:-0}"
if [[ "${POSE_SOURCE}" == "odom" && "${ALLOW_ODOM_ALIGNMENT}" != "1" ]]; then
  echo "ERROR: pose-source odom requires DUAL_SLOT_ALLOW_ODOM_MAP_ALIGNMENT=1" >&2
  exit 2
fi
PERSON_MODE="${PERSON_MODE_OVERRIDE:-${DUAL_SLOT_PERSON_LABEL_MODE:-dynamic}}"
case "${PERSON_MODE}" in
  ground-truth-legs|ground-truth-radius|dynamic|disabled) ;;
  *)
    echo "ERROR: DUAL_SLOT_PERSON_LABEL_MODE must be ground-truth-legs, ground-truth-radius, dynamic, or disabled" >&2
    exit 2
    ;;
esac
if [[ "${PERSON_MODE}" != "disabled" ]] && ! awk '
  { sub(/\r$/, "") }
  tolower($0) == "person" { found = 1 }
  END { exit(found ? 0 : 1) }
' "${LABEL_NAMES}"; then
  echo "ERROR: Person labeling needs a Person entry in ${LABEL_NAMES}." >&2
  echo "Dynamic mode writes free-space endpoints to that file-defined Person ID; add Person or set DUAL_SLOT_PERSON_LABEL_MODE=disabled." >&2
  exit 2
fi
SELF_MASK_MODE="${DUAL_SLOT_SELF_MASK_MODE:-first-synchronized-pair-fixed-beam-identity}"
case "${SELF_MASK_MODE}" in
  first-synchronized-pair-fixed-beam-identity|per-frame-footprint) ;;
  *)
    echo "ERROR: DUAL_SLOT_SELF_MASK_MODE must be first-synchronized-pair-fixed-beam-identity or per-frame-footprint" >&2
    exit 2
    ;;
esac
SYNC_TOLERANCE_MS="${DUAL_SLOT_SYNC_TOLERANCE_MS:-50.0}"
SUBGOAL_SOURCE="${DUAL_SLOT_SUBGOAL_SOURCE:-hindsight}"
SUBGOAL_MAX_AGE_MS="${DUAL_SLOT_SUBGOAL_MAX_AGE_MS:-300.0}"
FORWARD_ONLY="${DUAL_SLOT_FORWARD_ONLY:-1}"
REVERSE_LINEAR_X_EPSILON="${DUAL_SLOT_REVERSE_LINEAR_X_EPSILON:-0.001}"
REVERSE_RECOVERY_FRAMES="${DUAL_SLOT_REVERSE_RECOVERY_FRAMES:-15}"
case "${SUBGOAL_SOURCE}" in
  hindsight|online) ;;
  *)
    echo "ERROR: DUAL_SLOT_SUBGOAL_SOURCE must be hindsight or online" >&2
    exit 2
    ;;
esac
if ! [[ "${SUBGOAL_MAX_AGE_MS}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: DUAL_SLOT_SUBGOAL_MAX_AGE_MS must be a non-negative number" >&2
  exit 2
fi
case "${FORWARD_ONLY,,}" in
  1|true|yes|on) FORWARD_ONLY=1 ;;
  0|false|no|off) FORWARD_ONLY=0 ;;
  *)
    echo "ERROR: DUAL_SLOT_FORWARD_ONLY must be 0/1 or false/true" >&2
    exit 2
    ;;
esac
if ! awk -v value="${REVERSE_LINEAR_X_EPSILON}" \
  'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0.0) }'; then
  echo "ERROR: DUAL_SLOT_REVERSE_LINEAR_X_EPSILON must be positive" >&2
  exit 2
fi
if ! [[ "${REVERSE_RECOVERY_FRAMES}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: DUAL_SLOT_REVERSE_RECOVERY_FRAMES must be a non-negative integer" >&2
  exit 2
fi
TRAIN_RATIO="${DUAL_SLOT_TRAIN_RATIO:-0.7}"
DEV_RATIO="${DUAL_SLOT_DEV_RATIO:-0.1}"
TEST_RATIO="${DUAL_SLOT_TEST_RATIO:-0.2}"
SPLIT_SEED="${DUAL_SLOT_SPLIT_SEED:-0}"

BAG_TS="$(v7_dual_bag_timestamp "${BAG_DIR}")"
OUTPUT_ROOT="${DUAL_SLOT_OUTPUT_ROOT:-${RUN_ROOT}/datasets/fixed_dual_lidar_slots}"
SESSION_SUFFIX=""
if [[ "${PERSON_MODE}" == "ground-truth-legs" ]]; then
  SESSION_SUFFIX="-pedgt-v1"
fi
if [[ "${SUBGOAL_SOURCE}" == "online" ]]; then
  SESSION_SUFFIX="${SESSION_SUFFIX}-sgonline"
fi
SESSION_NAME="${BAG_TS}-v7-fixed-dual-v3-${SAMPLES_01}x${SAMPLES_02}-converted${SESSION_SUFFIX}"
SESSION_DIR="${OUTPUT_ROOT}/${SESSION_NAME}"
if [[ -e "${SESSION_DIR}" && "${DUAL_SLOT_ALLOW_OVERWRITE:-0}" != "1" ]]; then
  echo "ERROR: target session exists: ${SESSION_DIR}; refusing to overwrite" >&2
  exit 2
fi

TS="$(timestamp)"
LOG="${RUN_ROOT}/logs/07b_fixed_dual_lidar_${TS}.log"
CHECK_LOG="${RUN_ROOT}/logs/07b_fixed_dual_lidar_check_${TS}.log"
CHECK_JSON="${RUN_ROOT}/logs/07b_fixed_dual_lidar_check_${TS}.json"
CONVERTER="${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/v7_rosbag_to_fixed_dual_lidar_dataset.py"
CHECKER="${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_fixed_dual_lidar_dataset.py"
require_file "${CONVERTER}" "fixed dual-LiDAR converter"
require_file "${CHECKER}" "fixed dual-LiDAR dataset checker"

set_manifest_var "DUAL_SLOT_REQUESTED_SAMPLES_01" "${SAMPLES_01}"
set_manifest_var "DUAL_SLOT_REQUESTED_SAMPLES_02" "${SAMPLES_02}"
set_manifest_var "DUAL_SLOT_REQUESTED_TOTAL_SLOTS" "$((SAMPLES_01 + SAMPLES_02))"
set_manifest_var "DUAL_SLOT_INPUT_BAG" "${BAG_DIR}"
set_manifest_var "DUAL_SLOT_MAP_SOURCE_DIR" "${DUAL_SLOT_MAP_SOURCE_DIR:-${MAP_DIR}}"
set_manifest_var "DUAL_SLOT_ACTUAL_MAP_YAML" "${MAP_YAML}"
set_manifest_var "DUAL_SLOT_ACTUAL_SEMANTIC_LABEL" "${SEMANTIC_LABEL}"
set_manifest_var "DUAL_SLOT_ACTUAL_LABEL_NAMES" "${LABEL_NAMES}"
set_manifest_var "DUAL_SLOT_PERSON_LABEL_MODE" "${PERSON_MODE}"
set_manifest_var "DUAL_SLOT_POSE_SOURCE" "${POSE_SOURCE}"
set_manifest_var "DUAL_SLOT_ALLOW_ODOM_MAP_ALIGNMENT" "${ALLOW_ODOM_ALIGNMENT}"
set_manifest_var "DUAL_SLOT_SELF_MASK_MODE" "${SELF_MASK_MODE}"
set_manifest_var "DUAL_SLOT_SUBGOAL_SOURCE" "${SUBGOAL_SOURCE}"
set_manifest_var "DUAL_SLOT_SUBGOAL_MAX_AGE_MS" "${SUBGOAL_MAX_AGE_MS}"
set_manifest_var "DUAL_SLOT_FORWARD_ONLY" "${FORWARD_ONLY}"
set_manifest_var "DUAL_SLOT_REVERSE_LINEAR_X_EPSILON" "${REVERSE_LINEAR_X_EPSILON}"
set_manifest_var "DUAL_SLOT_REVERSE_RECOVERY_FRAMES" "${REVERSE_RECOVERY_FRAMES}"
set_manifest_var "DUAL_SLOT_CONTRACT" "fixed raw sensor slots; no resampling, cross-sensor deduplication, angle binning, or scan fusion"

echo "Input bag: ${BAG_DIR}"
echo "Output session: ${SESSION_DIR}"
echo "Slot contract: scan_01=${SAMPLES_01}, scan_02=${SAMPLES_02}, total=$((SAMPLES_01 + SAMPLES_02))"
echo "Self-mask mode: ${SELF_MASK_MODE}"
echo "Subgoal source: ${SUBGOAL_SOURCE}"
echo "Online subgoal max age: ${SUBGOAL_MAX_AGE_MS} ms"
echo "Forward-only filtering: ${FORWARD_ONLY}"
echo "Reverse recovery frames: ${REVERSE_RECOVERY_FRAMES}"

OVERWRITE_ARGS=()
if [[ "${DUAL_SLOT_ALLOW_OVERWRITE:-0}" == "1" ]]; then
  OVERWRITE_ARGS=(--overwrite)
fi
ODOM_ARGS=()
if [[ "${ALLOW_ODOM_ALIGNMENT}" == "1" ]]; then
  ODOM_ARGS=(--allow-odom-map-alignment)
fi
FORWARD_ONLY_ARGS=()
if [[ "${FORWARD_ONLY}" == "1" ]]; then
  FORWARD_ONLY_ARGS=(
    --exclude-reverse-linear-x
    --reverse-linear-x-epsilon "${REVERSE_LINEAR_X_EPSILON}"
    --reverse-recovery-frames "${REVERSE_RECOVERY_FRAMES}"
  )
fi
SUCCESSFUL_EPISODE_ARGS=()
case "${DUAL_SLOT_SUCCESSFUL_EPISODES_ONLY:-0}" in
  1|true|TRUE|yes|YES|on|ON)
    SUCCESSFUL_EPISODE_ARGS=(--successful-episodes-only)
    ;;
  0|false|FALSE|no|NO|off|OFF) ;;
  *)
    echo "ERROR: DUAL_SLOT_SUCCESSFUL_EPISODES_ONLY must be a boolean" >&2
    exit 2
    ;;
esac

{
  if [[ "${SAMPLES_01}" == "${SAMPLES_02}" ]]; then
    python3 "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/validate_v7_dual_lidar_capture.py" \
      bag --bag "${BAG_DIR}" --samples "${SAMPLES_01}" \
      --rate "${LIDAR_UPDATE_RATE_01:-${LIDAR_UPDATE_RATE:-10.0}}" \
      --range-min "${LIDAR_RANGE_MIN_01:-${LIDAR_RANGE_MIN:-0.1}}" \
      --range-max "${LIDAR_RANGE_MAX_01:-${LIDAR_RANGE_MAX:-50.0}}"
  else
    echo "Skipping the common-count capture validator for asymmetric sensors; the converter checks each stream independently."
  fi
  python3 "${ROS_WS}/tools/check_cmd_vel_stamped_bag.py" \
    --bag "${BAG_DIR}" \
    --episode-aware
  python3 "${CONVERTER}" \
    --bag "${BAG_DIR}" \
    --output-root "${OUTPUT_ROOT}" \
    --session-name "${SESSION_NAME}" \
    --map-yaml "${MAP_YAML}" \
    --semantic-label "${SEMANTIC_LABEL}" \
    --samples-01 "${SAMPLES_01}" \
    --samples-02 "${SAMPLES_02}" \
    --pose-source "${POSE_SOURCE}" \
    --person-label-mode "${PERSON_MODE}" \
    --person-ground-truth-radius-m "${DUAL_SLOT_PERSON_GROUND_TRUTH_RADIUS_M:-0.25}" \
    --person-ground-truth-leg-match-radius-m "${DUAL_SLOT_PERSON_GROUND_TRUTH_LEG_MATCH_RADIUS_M:-0.105}" \
    --person-ground-truth-max-delta-ms "${DUAL_SLOT_PERSON_GROUND_TRUTH_MAX_DELTA_MS:-150.0}" \
    --self-mask-mode "${SELF_MASK_MODE}" \
    --sync-tolerance-ms "${SYNC_TOLERANCE_MS}" \
    --train-ratio "${TRAIN_RATIO}" \
    --dev-ratio "${DEV_RATIO}" \
    --test-ratio "${TEST_RATIO}" \
    --split-seed "${SPLIT_SEED}" \
    --subgoal-source "${SUBGOAL_SOURCE}" \
    --subgoal-max-age-ms "${SUBGOAL_MAX_AGE_MS}" \
    --subgoal-lookahead "${SUBGOAL_LOOKAHEAD}" \
    "${ODOM_ARGS[@]}" \
    "${FORWARD_ONLY_ARGS[@]}" \
    "${SUCCESSFUL_EPISODE_ARGS[@]}" \
    "${OVERWRITE_ARGS[@]}"
} 2>&1 | tee "${LOG}"

python3 "${CHECKER}" --session "${SESSION_DIR}" --report-json "${CHECK_JSON}" \
  2>&1 | tee "${CHECK_LOG}"

set_manifest_var "DUAL_SLOT_DATASET_ROOT" "${OUTPUT_ROOT}"
set_manifest_var "DUAL_SLOT_SESSION_NAME" "${SESSION_NAME}"
set_manifest_var "DUAL_SLOT_SESSION_DIR" "${SESSION_DIR}"
set_manifest_var "DUAL_SLOT_CONVERSION_LOG" "${LOG}"
set_manifest_var "DUAL_SLOT_CHECK_REPORT" "${CHECK_JSON}"

echo "PASS: fixed dual-LiDAR conversion and full dataset check completed."
echo "Dataset: ${SESSION_DIR}"
echo "Report: ${CHECK_JSON}"
