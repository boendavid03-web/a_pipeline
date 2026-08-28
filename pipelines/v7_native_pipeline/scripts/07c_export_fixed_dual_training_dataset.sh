#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --expected-rate-01, --expected-rate-02, --frame-period-tolerance-ms, --output-session, --pool-range-max, --rate-tolerance-percent, --report-json, --require-session-listed, --session, --session-name, --source-session, --split-role
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, TXT
# 可能使用的关键环境变量：BAG_DIR, BAG_TS, BASH_SOURCE, CHECKER, CHECK_JSON, DATASET_ROOT, DUAL_SLOT_INPUT_BAG, DUAL_SLOT_REQUESTED_SAMPLES_01, DUAL_SLOT_REQUESTED_SAMPLES_02, DUAL_SLOT_SESSION_DIR, ERROR, EXIT, EXPECTED_RATE_01, EXPECTED_RATE_02, EXPORTER, FIXED_DUAL_FRAME_PERIOD_TOLERANCE_MS, FIXED_DUAL_RATE_TOLERANCE_PERCENT, FIXED_DUAL_SPLIT_ROLE, FIXED_DUAL_TRAINING_CHECK_REPORT, FIXED_DUAL_TRAINING_DATASET_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07c_export_fixed_dual_training_dataset.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_manifest "${RUN_MANIFEST:-}"

require_dir "${DUAL_SLOT_SESSION_DIR}" "validated fixed dual-LiDAR NPZ session"
require_file "${DUAL_SLOT_SESSION_DIR}/metadata.json" "fixed dual-LiDAR metadata"

readarray -t SOURCE_INFO < <(python3 - "${DUAL_SLOT_SESSION_DIR}/metadata.json" <<'PY'
import json
import sys

metadata = json.loads(open(sys.argv[1], encoding="utf-8").read())
for key in ("samples_01", "samples_02"):
    value = metadata.get(key)
    if not isinstance(value, int) or value <= 0:
        raise SystemExit(f"ERROR: source metadata {key} must be a positive integer")
    print(value)
print(metadata.get("person_label_mode", ""))
print(metadata.get("subgoal_source", ""))
PY
)
SAMPLES_01="${SOURCE_INFO[0]:-}"
SAMPLES_02="${SOURCE_INFO[1]:-}"
SOURCE_PERSON_MODE="${SOURCE_INFO[2]:-}"
SOURCE_SUBGOAL_SOURCE="${SOURCE_INFO[3]:-}"
if [[ -z "${SAMPLES_01}" || -z "${SAMPLES_02}" ]]; then
  echo "ERROR: could not read dual-LiDAR sample counts from source metadata" >&2
  exit 2
fi
for key in DUAL_SLOT_REQUESTED_SAMPLES_01 DUAL_SLOT_REQUESTED_SAMPLES_02; do
  requested="${!key:-}"
  actual="${SAMPLES_01}"
  if [[ "${key}" == "DUAL_SLOT_REQUESTED_SAMPLES_02" ]]; then
    actual="${SAMPLES_02}"
  fi
  if [[ -n "${requested}" && "${requested}" != "${actual}" ]]; then
    echo "ERROR: ${key}=${requested} does not match source metadata ${actual}" >&2
    exit 2
  fi
done
POOL_RANGE_MAX="${SEMANTIC_CNN_POOL_RANGE_MAX:-}"
FRAME_PERIOD_TOLERANCE_MS="${FIXED_DUAL_FRAME_PERIOD_TOLERANCE_MS:-20.0}"
EXPECTED_RATE_01="${LIDAR_UPDATE_RATE_01:-${LIDAR_UPDATE_RATE:-10.0}}"
EXPECTED_RATE_02="${LIDAR_UPDATE_RATE_02:-${LIDAR_UPDATE_RATE:-10.0}}"
RATE_TOLERANCE_PERCENT="${FIXED_DUAL_RATE_TOLERANCE_PERCENT:-10.0}"
POOL_ARGS=()
if [[ -n "${POOL_RANGE_MAX}" ]]; then
  POOL_ARGS=(--pool-range-max "${POOL_RANGE_MAX}")
fi
RATE_ARGS=(
  --expected-rate-01 "${EXPECTED_RATE_01}"
  --expected-rate-02 "${EXPECTED_RATE_02}"
  --rate-tolerance-percent "${RATE_TOLERANCE_PERCENT}"
)

BAG_TS="$(v7_dual_bag_timestamp "${DUAL_SLOT_INPUT_BAG:-${BAG_DIR}}")"
DATASET_ROOT="${FIXED_DUAL_TRAINING_OUTPUT_ROOT:-${RUN_ROOT}/datasets/semantic2d_fixed_dual_native}"
SESSION_SUFFIX=""
if [[ "${SOURCE_PERSON_MODE}" == "ground-truth-legs" ]]; then
  SESSION_SUFFIX="-pedgt-v1"
fi
if [[ "${SOURCE_SUBGOAL_SOURCE}" == "online" ]]; then
  SESSION_SUFFIX="${SESSION_SUFFIX}-sgonline"
fi
SESSION_NAME="${BAG_TS}-v7-fixed-dual-v3-${SAMPLES_01}x${SAMPLES_02}-training${SESSION_SUFFIX}"
SESSION_DIR="${DATASET_ROOT}/${SESSION_NAME}"
STAGING_DIR="${DATASET_ROOT}/.${SESSION_NAME}.tmp.$$"
if [[ -e "${SESSION_DIR}" ]]; then
  echo "ERROR: target session exists: ${SESSION_DIR}; refusing to overwrite" >&2
  exit 2
fi

EXPORTER="${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py"
CHECKER="${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py"
require_file "${EXPORTER}" "fixed-dual Semantic2D exporter"
require_file "${CHECKER}" "fixed-dual Semantic2D checker"

TS="$(timestamp)"
LOG="${RUN_ROOT}/logs/07c_export_fixed_dual_training_${TS}.log"
CHECK_JSON="${RUN_ROOT}/logs/07c_export_fixed_dual_training_check_${TS}.json"
mkdir -p "${DATASET_ROOT}" "${RUN_ROOT}/logs"

cleanup_staging() {
  if [[ -d "${STAGING_DIR}" ]]; then
    rm -rf "${STAGING_DIR}"
  fi
}
trap cleanup_staging EXIT

{
  echo "Source NPZ session: ${DUAL_SLOT_SESSION_DIR}"
  echo "Output session: ${SESSION_DIR}"
  echo "Pool range max: ${POOL_RANGE_MAX:-auto from source sensor ranges}"
  echo "Expected capture rates: scan_01=${EXPECTED_RATE_01} Hz, scan_02=${EXPECTED_RATE_02} Hz"

  python3 "${EXPORTER}" \
    --source-session "${DUAL_SLOT_SESSION_DIR}" \
    --output-session "${STAGING_DIR}" \
    --session-name "${SESSION_NAME}" \
    --frame-period-tolerance-ms "${FRAME_PERIOD_TOLERANCE_MS}" \
    --split-role "${FIXED_DUAL_SPLIT_ROLE:-preserve}" \
    "${POOL_ARGS[@]}"

  python3 "${CHECKER}" \
    --session "${STAGING_DIR}" \
    --source-session "${DUAL_SLOT_SESSION_DIR}" \
    --dataset-root "${DATASET_ROOT}" \
    "${RATE_ARGS[@]}"

  mv "${STAGING_DIR}" "${SESSION_DIR}"

  python3 - "${DATASET_ROOT}" "${SESSION_NAME}" <<'PY'
import sys
from pathlib import Path

root = Path(sys.argv[1])
new_session = sys.argv[2]
label_names = (root / new_session / "label_names.txt").read_text(encoding="utf-8")
if len([line for line in label_names.splitlines() if line.strip()]) < 2:
    raise SystemExit("ERROR: exported session has no usable label_names.txt")
root_label_names = root / "label_names.txt"
if not root_label_names.exists():
    root_label_names.write_text(label_names, encoding="utf-8")
index_path = root / "dataset.txt"
names = []
if index_path.exists():
    names.extend(line.strip() for line in index_path.read_text(encoding="utf-8").splitlines())
names.append(new_session)
names = list(dict.fromkeys(name for name in names if name))
temp_path = root / ".dataset.txt.tmp"
temp_path.write_text("".join(f"{name}\n" for name in names), encoding="utf-8")
temp_path.replace(index_path)
PY

  python3 "${CHECKER}" \
    --session "${SESSION_DIR}" \
    --source-session "${DUAL_SLOT_SESSION_DIR}" \
    --dataset-root "${DATASET_ROOT}" \
    --require-session-listed \
    "${RATE_ARGS[@]}" \
    --report-json "${CHECK_JSON}"
} 2>&1 | tee "${LOG}"

trap - EXIT
set_manifest_var "FIXED_DUAL_TRAINING_DATASET_ROOT" "${DATASET_ROOT}"
set_manifest_var "FIXED_DUAL_TRAINING_SESSION_NAME" "${SESSION_NAME}"
set_manifest_var "FIXED_DUAL_TRAINING_SESSION_DIR" "${SESSION_DIR}"
set_manifest_var "FIXED_DUAL_TRAINING_CHECK_REPORT" "${CHECK_JSON}"

echo "PASS: fixed-dual Semantic2D training export completed."
echo "Dataset root: ${DATASET_ROOT}"
echo "Session: ${SESSION_DIR}"
echo "Report: ${CHECK_JSON}"
