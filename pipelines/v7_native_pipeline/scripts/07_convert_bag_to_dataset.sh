#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --cmd-vel-topic, --dev-ratio, --map-yaml, --multi, --odom-topic, --output-root, --overwrite, --scan-topic, --semantic-label, --session-name, --skip-dataset-index, --subgoal-lookahead, --target-points
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：TXT, YAML
# 可能使用的关键环境变量：BAG_DIR, BAG_DIRS, BASH_SOURCE, CMD_VEL_TOPIC, DATASET_ROOT, DEV_RATIO, ERROR, LAST_BAG_DIR, MAP_YAML, MODE, MULTI_BAG, ODOM_TOPIC, REQUESTED_BAG_DIR, REQUESTED_SESSION_NAME, ROS_WS, RUN_MANIFEST, RUN_ROOT, SCAN_TOPIC, SCRIPT_DIR, SEMANTIC_LABEL_PNG
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:27.123706575 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_baseline.py（执行该脚本，使用其输出继续当前流程）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_baseline.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07_convert_bag_to_dataset.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

MODE="single"
REQUESTED_BAG_DIR=""
REQUESTED_SESSION_NAME="${SESSION_NAME:-}"
case "${1:-}" in
  --multi)
    MODE="multi"
    ;;
  "")
    ;;
  *)
    MODE="single_path"
    REQUESTED_BAG_DIR="$1"
    ;;
esac

load_manifest "${RUN_MANIFEST:-}"

safe_source_ros
require_file "${MAP_YAML}" "map yaml"
require_file "${SEMANTIC_LABEL_PNG}" "semantic label png"
mkdir -p "${DATASET_ROOT}"

LOG="${RUN_ROOT}/logs/07_convert_bag_to_dataset_$(timestamp).log"

convert_one_bag() {
  local bag_dir="$1"
  local session_name="$2"
  local overwrite="$3"
  local skip_dataset_index="$4"
  local session_dir="${DATASET_ROOT}/${session_name}"
  local -a args

  require_dir "${bag_dir}" "ROS 2 bag directory"
  require_file "${bag_dir}/metadata.yaml" "ROS 2 bag metadata"

  args=(
    python3 "${ROS_WS}/tools/convert_rosbag2_to_semantic2d_baseline.py"
    --bag "${bag_dir}"
    --output-root "${DATASET_ROOT}"
    --session-name "${session_name}"
    --map-yaml "${MAP_YAML}"
    --semantic-label "${SEMANTIC_LABEL_PNG}"
    --scan-topic "${SCAN_TOPIC}"
    --odom-topic "${ODOM_TOPIC}"
    --cmd-vel-topic "${CMD_VEL_TOPIC}"
    --target-points "${TARGET_POINTS}"
    --dev-ratio "${DEV_RATIO}"
    --subgoal-lookahead "${SUBGOAL_LOOKAHEAD}"
  )
  if [[ "${overwrite}" == "1" ]]; then
    args+=(--overwrite)
  elif [[ -d "${session_dir}" ]]; then
    echo "Keeping existing session: ${session_dir}"
    return 0
  fi
  if [[ "${skip_dataset_index}" == "1" ]]; then
    args+=(--skip-dataset-index)
  fi

  echo "Converting bag:"
  echo "  bag: ${bag_dir}"
  echo "  session: ${session_name}"
  "${args[@]}"
}

normalize_dataset_entry() {
  local entry="$1"
  local session
  session="${entry%/}"
  if [[ -z "${session}" || "${session}" != *-* ]]; then
    return 0
  fi
  if [[ ! -d "${DATASET_ROOT}/${session}" ]]; then
    return 0
  fi
  if [[ ! -f "${DATASET_ROOT}/${session}/train.txt" || ! -f "${DATASET_ROOT}/${session}/dev.txt" ]]; then
    return 0
  fi
  printf '%s/\n' "${session}"
}

write_dataset_index() {
  local tmp
  local session_name
  local session_dir
  tmp="$(mktemp)"
  {
    if [[ -f "${DATASET_ROOT}/dataset.txt" ]]; then
      while IFS= read -r entry; do
        normalize_dataset_entry "${entry}"
      done < "${DATASET_ROOT}/dataset.txt"
    fi
    while IFS= read -r session_dir; do
      normalize_dataset_entry "${session_dir##*/}"
    done < <(find "${DATASET_ROOT}" -mindepth 1 -maxdepth 1 -type d | sort)
    for session_name in "$@"; do
      normalize_dataset_entry "${session_name}"
    done
  } | awk '!seen[$0]++' > "${tmp}"

  if [[ ! -s "${tmp}" ]]; then
    rm -f "${tmp}"
    echo "ERROR: no valid sessions to write to ${DATASET_ROOT}/dataset.txt" >&2
    exit 2
  fi

  mv "${tmp}" "${DATASET_ROOT}/dataset.txt"
}

{
  if [[ "${MULTI_BAG:-0}" == "1" ]]; then
    MODE="multi"
  fi

  echo "Converting bag(s) to Semantic2D baseline dataset."

  if [[ "${MODE}" == "multi" ]]; then
    mapfile -t BAG_DIRS < <(list_v7_dual_bag_dirs)
    if [[ "${#BAG_DIRS[@]}" -eq 0 ]]; then
      echo "ERROR: no bags found under ${RUN_ROOT}/bags/raw/*${V7_DUAL_BAG_SUFFIX}" >&2
      exit 2
    fi

    SESSION_NAMES=()
    for bag_dir in "${BAG_DIRS[@]}"; do
      session_name="$(v7_dual_session_name_from_bag "${bag_dir}")"
      SESSION_NAMES+=("${session_name}")
      convert_one_bag "${bag_dir}" "${session_name}" 0 1
      echo
    done
    write_dataset_index "${SESSION_NAMES[@]}"
  else
    if [[ "${MODE}" == "single_path" ]]; then
      BAG_DIR="${REQUESTED_BAG_DIR}"
    else
      if ! BAG_DIR="$(latest_v7_dual_bag_dir)"; then
        echo "ERROR: no latest bag found. Record one first, or pass a bag directory." >&2
        exit 2
      fi
    fi

    SESSION_NAME="$(v7_dual_session_name_from_bag "${BAG_DIR}")"
    if [[ -n "${REQUESTED_SESSION_NAME}" && "${REQUESTED_SESSION_NAME}" == *-* ]]; then
      SESSION_NAME="${REQUESTED_SESSION_NAME}"
    fi
    set_manifest_var "SESSION_NAME" "${SESSION_NAME}"
    set_manifest_var "LAST_BAG_DIR" "${BAG_DIR}"
    set_manifest_var "BAG_DIR" "${BAG_DIR}"

    convert_one_bag "${BAG_DIR}" "${SESSION_NAME}" 1 0
  fi

  echo
  echo "===== dataset.txt ====="
  cat "${DATASET_ROOT}/dataset.txt"
  echo
  echo "===== dataset files ====="
  find "${DATASET_ROOT}" -maxdepth 2 -type f | sort
} 2>&1 | tee "${LOG}"

echo "Wrote ${LOG}"
