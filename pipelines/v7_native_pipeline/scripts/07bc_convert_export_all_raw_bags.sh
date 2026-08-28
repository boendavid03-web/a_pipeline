#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, TXT, YAML
# 可能使用的关键环境变量：ALLOWLIST_FILE, BAG_DIRS, BASH_SOURCE, DUAL_SLOT_BAG_ALLOWLIST_FILE, DUAL_SLOT_BAG_DIR, DUAL_SLOT_INPUT_BAG, DUAL_SLOT_KEEP_INTERMEDIATE, DUAL_SLOT_OUTPUT_ROOT, DUAL_SLOT_PERSON_LABEL_MODE, DUAL_SLOT_REQUESTED_SAMPLES_01, DUAL_SLOT_REQUESTED_SAMPLES_02, DUAL_SLOT_SESSION_DIR, DUAL_SLOT_SESSION_NAME, DUAL_SLOT_SUBGOAL_SOURCE, ERROR, FIXED_DUAL_TRAINING_CHECK_REPORT, FIXED_DUAL_TRAINING_DATASET_ROOT, FIXED_DUAL_TRAINING_OUTPUT_ROOT, FIXED_DUAL_TRAINING_SESSION_DIR, FIXED_DUAL_TRAINING_SESSION_NAME
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 08:01:46.170516674 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:35.854867558 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07bc_convert_export_all_raw_bags.sh】
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

ORIGINAL_MANIFEST="${RUN_MANIFEST}"
RAW_ROOT="${RUN_ROOT}/bags/raw"
LABEL_NAMES="${RUN_ROOT}/maps/semantic_label/label_names.txt"
PERSON_MODE="${PERSON_MODE_OVERRIDE:-${DUAL_SLOT_PERSON_LABEL_MODE:-dynamic}}"
SAMPLES_01="${LIDAR_SAMPLES_01:-${LIDAR_SAMPLES:-360}}"
SAMPLES_02="${LIDAR_SAMPLES_02:-${LIDAR_SAMPLES:-360}}"
SLOT_ROOT="${DUAL_SLOT_OUTPUT_ROOT:-${RUN_ROOT}/datasets/fixed_dual_lidar_slots}"
TRAIN_ROOT="${FIXED_DUAL_TRAINING_OUTPUT_ROOT:-${RUN_ROOT}/datasets/semantic2d_fixed_dual_native}"
STATUS=0
FOUND_BAG=0
ALLOWLIST_FILE="${DUAL_SLOT_BAG_ALLOWLIST_FILE:-}"
SUBGOAL_SOURCE="${DUAL_SLOT_SUBGOAL_SOURCE:-hindsight}"
KEEP_INTERMEDIATE="${DUAL_SLOT_KEEP_INTERMEDIATE:-1}"

case "${PERSON_MODE}" in
  ground-truth-legs|ground-truth-radius|dynamic|disabled) ;;
  *)
    echo "ERROR: DUAL_SLOT_PERSON_LABEL_MODE must be ground-truth-legs, ground-truth-radius, dynamic, or disabled" >&2
    exit 2
    ;;
esac
require_dir "${RAW_ROOT}" "raw bag directory"
if [[ -z "${ALLOWLIST_FILE}" ]]; then
  echo "ERROR: DUAL_SLOT_BAG_ALLOWLIST_FILE is required; refusing to scan all raw bags." >&2
  exit 2
fi
require_file "${ALLOWLIST_FILE}" "explicit raw bag allowlist"
case "${SUBGOAL_SOURCE}" in
  hindsight|online) ;;
  *)
    echo "ERROR: DUAL_SLOT_SUBGOAL_SOURCE must be hindsight or online" >&2
    exit 2
    ;;
esac
case "${KEEP_INTERMEDIATE}" in
  0|1) ;;
  *)
    echo "ERROR: DUAL_SLOT_KEEP_INTERMEDIATE must be 0 or 1" >&2
    exit 2
    ;;
esac
if [[ "${PERSON_MODE}" != "disabled" ]]; then
  require_file "${LABEL_NAMES}" "semantic label names"
  if ! awk '
    { sub(/\r$/, "") }
    tolower($0) == "person" { found = 1 }
    END { exit(found ? 0 : 1) }
  ' "${LABEL_NAMES}"; then
    echo "ERROR: Person labeling requires a Person entry in ${LABEL_NAMES}" >&2
    exit 2
  fi
fi

promote_export_manifest() {
  local overlay_manifest="$1"
  local key

  # shellcheck disable=SC1090
  source "${overlay_manifest}"
  RUN_MANIFEST="${ORIGINAL_MANIFEST}"
  for key in \
    FIXED_DUAL_TRAINING_DATASET_ROOT \
    FIXED_DUAL_TRAINING_SESSION_NAME \
    FIXED_DUAL_TRAINING_SESSION_DIR \
    FIXED_DUAL_TRAINING_CHECK_REPORT; do
    set_manifest_var "${key}" "${!key}"
  done
}

remove_generated_slot_session() {
  local session_dir="$1"
  local resolved_root
  local resolved_session

  require_dir "${SLOT_ROOT}" "fixed-slot output root"
  require_dir "${session_dir}" "newly generated fixed-slot session"
  require_file "${session_dir}/metadata.json" "newly generated fixed-slot metadata"
  resolved_root="$(realpath -e "${SLOT_ROOT}")"
  resolved_session="$(realpath -e "${session_dir}")"
  if [[ "$(dirname "${resolved_session}")" != "${resolved_root}" ]]; then
    echo "ERROR: refusing to remove fixed-slot session outside ${resolved_root}: ${resolved_session}" >&2
    return 1
  fi

  rm -rf -- "${resolved_session}"
  set_manifest_var "DUAL_SLOT_SESSION_NAME" ""
  set_manifest_var "DUAL_SLOT_SESSION_DIR" ""
  echo "07b: removed newly generated intermediate session after successful 07c validation: ${resolved_session}"
}

BAG_DIRS=()
declare -A SEEN_BAGS=()
while IFS= read -r requested || [[ -n "${requested}" ]]; do
  requested="${requested%$'\r'}"
  [[ -z "${requested}" || "${requested}" == \#* ]] && continue
  if [[ "${requested}" != /* ]]; then
    echo "ERROR: allowlist entries must be absolute paths: ${requested}" >&2
    exit 2
  fi
  bag_dir="$(realpath -e "${requested}")"
  if [[ "${bag_dir}" != "${RAW_ROOT}/"* ]]; then
    echo "ERROR: allowlisted bag is outside ${RAW_ROOT}: ${bag_dir}" >&2
    exit 2
  fi
  require_dir "${bag_dir}" "allowlisted raw bag"
  require_file "${bag_dir}/metadata.yaml" "allowlisted raw bag metadata"
  if [[ "${bag_dir##*/}" != *"${V7_DUAL_BAG_SUFFIX}" ]]; then
    echo "ERROR: allowlisted path is not a v7 dual bag: ${bag_dir}" >&2
    exit 2
  fi
  if [[ -n "${SEEN_BAGS[${bag_dir}]:-}" ]]; then
    echo "ERROR: duplicate bag in allowlist: ${bag_dir}" >&2
    exit 2
  fi
  SEEN_BAGS["${bag_dir}"]=1
  BAG_DIRS+=("${bag_dir}")
done < "${ALLOWLIST_FILE}"
if [[ "${#BAG_DIRS[@]}" -eq 0 ]]; then
  echo "ERROR: bag allowlist is empty: ${ALLOWLIST_FILE}" >&2
  exit 2
fi

for bag_dir in "${BAG_DIRS[@]}"; do
  FOUND_BAG=1
  bag_ts="$(v7_dual_bag_timestamp "${bag_dir}")"
  session_suffix=""
  if [[ "${PERSON_MODE}" == "ground-truth-legs" ]]; then
    session_suffix="-pedgt-v1"
  fi
  if [[ "${SUBGOAL_SOURCE}" == "online" ]]; then
    session_suffix="${session_suffix}-sgonline"
  fi
  slot_session="${SLOT_ROOT}/${bag_ts}-v7-fixed-dual-v3-${SAMPLES_01}x${SAMPLES_02}-converted${session_suffix}"
  train_session="${TRAIN_ROOT}/${bag_ts}-v7-fixed-dual-v3-${SAMPLES_01}x${SAMPLES_02}-training${session_suffix}"
  generated_slot=0

  echo
  echo "===== ${bag_dir##*/} ====="

  if [[ -d "${slot_session}" ]]; then
    echo "07b: existing session, skip: ${slot_session}"
  else
    echo "07b: missing; converting with person mode=${PERSON_MODE}"
    if ! DUAL_SLOT_BAG_DIR="${bag_dir}" \
      DUAL_SLOT_PERSON_LABEL_MODE="${PERSON_MODE}" \
      DUAL_SLOT_SUBGOAL_SOURCE="${SUBGOAL_SOURCE}" \
      bash "${SCRIPT_DIR}/07b_convert_bag_to_fixed_dual_lidar.sh"; then
      echo "ERROR: 07b failed; skip 07c for this bag" >&2
      STATUS=1
      continue
    fi
    generated_slot=1
  fi

  if [[ -d "${train_session}" ]]; then
    echo "07c: existing session, skip: ${train_session}"
    continue
  fi

  echo "07c: missing; exporting training session"
  overlay_manifest="$(mktemp)"
  cp "${ORIGINAL_MANIFEST}" "${overlay_manifest}"
  printf 'export DUAL_SLOT_SESSION_DIR=%q\n' "${slot_session}" >> "${overlay_manifest}"
  printf 'export DUAL_SLOT_INPUT_BAG=%q\n' "${bag_dir}" >> "${overlay_manifest}"
  printf 'export DUAL_SLOT_REQUESTED_SAMPLES_01=%q\n' "${SAMPLES_01}" >> "${overlay_manifest}"
  printf 'export DUAL_SLOT_REQUESTED_SAMPLES_02=%q\n' "${SAMPLES_02}" >> "${overlay_manifest}"

  if RUN_MANIFEST="${overlay_manifest}" \
    bash "${SCRIPT_DIR}/07c_export_fixed_dual_training_dataset.sh"; then
    promote_export_manifest "${overlay_manifest}"
    echo "07c: completed"
    if [[ "${KEEP_INTERMEDIATE}" == "0" && "${generated_slot}" == "1" ]]; then
      if ! remove_generated_slot_session "${slot_session}"; then
        echo "ERROR: 07c succeeded, but cleanup of the new 07b intermediate failed" >&2
        STATUS=1
      fi
    fi
  else
    echo "ERROR: 07c failed" >&2
    STATUS=1
  fi
  rm -f "${overlay_manifest}"
done

if [[ "${FOUND_BAG}" == "0" ]]; then
  echo "ERROR: no v7 dual raw bags found under ${RAW_ROOT}" >&2
  exit 2
fi

exit "${STATUS}"
