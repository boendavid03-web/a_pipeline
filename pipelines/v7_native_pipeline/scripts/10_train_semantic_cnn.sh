#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --delete, --device, --log, --model-code, --model-dir, --out-dir, --periodic-max
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT, TXT
# 可能使用的关键环境变量：ALLOW_CPU, BASH_SOURCE, BATCH_SIZE, CHECKPOINT_INTERVAL, CNN_WORK, CUDA, DATASET_INDEX_MODE, EPOCHS, ERROR, EVAL_DIR, EVAL_LOG, FIXED_DUAL_TRAINING_DATASET_ROOT, LAST_SEMANTIC_CNN_BEST_DEV, LAST_SEMANTIC_CNN_MODEL, LAST_SEMANTIC_CNN_REPORT, LAST_SEMANTIC_CNN_RESULT_DIR, MODEL, MODEL_CODE_SNAPSHOT, NATIVE_DATASET_ROOT, NUM_EPOCHS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:35.856867595 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜10_train_semantic_cnn.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_manifest "${RUN_MANIFEST:-}"

DATASET_INDEX_MODE="native_cmd"
if [[ -n "${FIXED_DUAL_TRAINING_DATASET_ROOT:-}" ]]; then
  NATIVE_DATASET_ROOT="${FIXED_DUAL_TRAINING_DATASET_ROOT}"
  DATASET_INDEX_MODE="fixed_dual_cmd"
  SEMANTIC_CNN_POOL_MODE="${SEMANTIC_CNN_POOL_MODE:-global_virtual_angle_80}"
else
  NATIVE_DATASET_ROOT="${NATIVE_DATASET_ROOT:-}"
  SEMANTIC_CNN_POOL_MODE="${SEMANTIC_CNN_POOL_MODE:-global_virtual_angle_80}"
fi
export SEMANTIC_CNN_POOL_MODE
require_dir "${NATIVE_DATASET_ROOT}" "native converted dataset root"
require_file "${TORCH_PY}" "torch python"
require_file "${NATIVE_DATASET_ROOT}/dataset.txt" "native dataset.txt"
refresh_dataset_index "${NATIVE_DATASET_ROOT}" "${DATASET_INDEX_MODE}" verify
SEMANTIC_CNN_STATS_SOURCE="${SEMANTIC_CNN_STATS_JSON:-}"
if [[ -z "${SEMANTIC_CNN_STATS_SOURCE}" && -f "${NATIVE_DATASET_ROOT}/train_normalization_stats.json" ]]; then
  SEMANTIC_CNN_STATS_SOURCE="${NATIVE_DATASET_ROOT}/train_normalization_stats.json"
fi
if [[ -n "${SEMANTIC_CNN_STATS_SOURCE}" ]]; then
  require_file "${SEMANTIC_CNN_STATS_SOURCE}" "SemanticCNN train normalization stats"
  SEMANTIC_CNN_STATS_SOURCE="$(readlink -f "${SEMANTIC_CNN_STATS_SOURCE}")"
fi

EPOCHS="${SEMANTIC_CNN_EPOCHS:-51}"
BATCH_SIZE="${SEMANTIC_CNN_BATCH_SIZE:-64}"
CHECKPOINT_INTERVAL="${SEMANTIC_CNN_CHECKPOINT_INTERVAL:-10}"
STOP_LOSS_WEIGHT="${SEMANTIC_CNN_STOP_LOSS_WEIGHT:-1.0}"
ALLOW_CPU="${SEMANTIC_CNN_ALLOW_CPU:-0}"
WRITE_COMPAT_ALIASES="${SEMANTIC_CNN_WRITE_COMPAT_ALIASES:-0}"
if [[ "${ALLOW_CPU}" != "0" && "${ALLOW_CPU}" != "1" ]]; then
  echo "ERROR: SEMANTIC_CNN_ALLOW_CPU must be 0 or 1." >&2
  exit 2
fi
if [[ "${WRITE_COMPAT_ALIASES}" != "0" && "${WRITE_COMPAT_ALIASES}" != "1" ]]; then
  echo "ERROR: SEMANTIC_CNN_WRITE_COMPAT_ALIASES must be 0 or 1." >&2
  exit 2
fi
"${TORCH_PY}" -c 'import math, sys; value = float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value >= 0.0 else 2)' "${STOP_LOSS_WEIGHT}" \
  || { echo "ERROR: SEMANTIC_CNN_STOP_LOSS_WEIGHT must be a finite number >= 0." >&2; exit 2; }
TRAIN_DEVICE="$("${TORCH_PY}" -c 'import torch; print("cuda" if torch.cuda.is_available() else "cpu")')"
if [[ "${TRAIN_DEVICE}" != "cuda" && "${ALLOW_CPU}" != "1" ]]; then
  echo "ERROR: CUDA is unavailable; refusing full SemanticCNN training on CPU." >&2
  echo "Set SEMANTIC_CNN_ALLOW_CPU=1 only for an intentional bounded CPU run." >&2
  exit 2
fi
STEP_TS="$(timestamp)"
STEP_ID="${STEP_TS}_semantic_cnn_native_cmd_${EPOCHS}epoch"
SEMANTIC_CNN_TRAINING_OUTPUT_ROOT="${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT:-${RUN_ROOT}/training/semantic_cnn}"
RESULT_DIR="${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}/${STEP_ID}"
CNN_WORK="${RESULT_DIR}/work/semantic_cnn_v7"
MODEL="${RESULT_DIR}/${STEP_ID}_model.pth"
LOG="${RESULT_DIR}/10_train_semantic_cnn_${STEP_TS}.log"
EVAL_DIR="${RESULT_DIR}/eval_reports"
EVAL_LOG="${RESULT_DIR}/check_semantic_cnn_training_${STEP_TS}.log"
SEMANTIC_CNN_STATS_SNAPSHOT=""
if [[ -n "${SEMANTIC_CNN_STATS_SOURCE}" ]]; then
  SEMANTIC_CNN_STATS_SNAPSHOT="${RESULT_DIR}/semantic_cnn_train_normalization_stats.json"
fi
if [[ -e "${RESULT_DIR}" ]]; then
  echo "ERROR: refusing to overwrite existing SemanticCNN result directory: ${RESULT_DIR}" >&2
  exit 2
fi
if [[ "${WRITE_COMPAT_ALIASES}" == "1" ]]; then
  for alias_name in latest best_dev final; do
    alias_path="${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}/semantic_cnn_native_cmd_${alias_name}.pth"
    if [[ -e "${alias_path}" ]]; then
      echo "ERROR: refusing to overwrite compatibility alias: ${alias_path}" >&2
      exit 2
    fi
  done
fi
mkdir -p "${CNN_WORK}"

{
  rsync -a --delete "${SEMANTIC_CNN_ROOT}/training/" "${CNN_WORK}/"
  rsync -a "${CNN_WORK}/scripts/" "${RESULT_DIR}/model_code_scripts/"
  if [[ -n "${SEMANTIC_CNN_STATS_SOURCE}" ]]; then
    cp "${SEMANTIC_CNN_STATS_SOURCE}" "${SEMANTIC_CNN_STATS_SNAPSHOT}"
    export SEMANTIC_CNN_STATS_JSON="${SEMANTIC_CNN_STATS_SNAPSHOT}"
  else
    unset SEMANTIC_CNN_STATS_JSON
  fi
  cat > "${RESULT_DIR}/training_config.env" <<EOF
STEP_TS="${STEP_TS}"
STEP_ID="${STEP_ID}"
EPOCHS="${EPOCHS}"
BATCH_SIZE="${BATCH_SIZE}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL}"
STOP_LOSS_WEIGHT="${STOP_LOSS_WEIGHT}"
TRAIN_DEVICE="${TRAIN_DEVICE}"
SEMANTIC_CNN_TRAINING_OUTPUT_ROOT="${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}"
NATIVE_DATASET_ROOT="${NATIVE_DATASET_ROOT}"
MODEL="${MODEL}"
LOG="${LOG}"
TARGET="cmd_velocities/[linear_x, angular_z]"
SEMANTIC_CNN_POOL_MODE="${SEMANTIC_CNN_POOL_MODE}"
SEMANTIC_CNN_STATS_SOURCE="${SEMANTIC_CNN_STATS_SOURCE}"
SEMANTIC_CNN_STATS_JSON="${SEMANTIC_CNN_STATS_SNAPSHOT}"
WRITE_COMPAT_ALIASES="${WRITE_COMPAT_ALIASES}"
MODEL_CODE_SNAPSHOT="${RESULT_DIR}/model_code_scripts"
EOF
  cd "${CNN_WORK}"

  echo "===== Train SemanticCNN: ${EPOCHS} epochs ====="
  echo "result dir: ${RESULT_DIR}"
  echo "model path: ${MODEL}"
  echo "dataset root: ${NATIVE_DATASET_ROOT}"
  echo "target: cmd_velocities/[linear_x, angular_z]"
  echo "pool mode: ${SEMANTIC_CNN_POOL_MODE}"
  echo "normalization stats: ${SEMANTIC_CNN_STATS_JSON:-legacy constants}"
  echo "stop loss weight: ${STOP_LOSS_WEIGHT}"
  echo "device: ${TRAIN_DEVICE}"
  "${TORCH_PY}" - <<PY
import sys
sys.path.insert(0, "scripts")
import train
train.NUM_EPOCHS = ${EPOCHS}
train.BATCH_SIZE = ${BATCH_SIZE}
train.CHECKPOINT_INTERVAL = ${CHECKPOINT_INTERVAL}
train.STOP_LOSS_WEIGHT = ${STOP_LOSS_WEIGHT}
train.main([
    "${MODEL}",
    "${NATIVE_DATASET_ROOT}/",
    "${NATIVE_DATASET_ROOT}/",
])
PY

  ls -lh "${MODEL}" \
    "${RESULT_DIR}/semantic_cnn_native_cmd_latest.pth" \
    "${RESULT_DIR}/semantic_cnn_native_cmd_best_dev.pth" \
    "${RESULT_DIR}/semantic_cnn_native_cmd_final.pth"
} 2>&1 | tee "${LOG}"

echo "Wrote ${LOG}"

if [[ "${WRITE_COMPAT_ALIASES}" == "1" ]]; then
  cp "${RESULT_DIR}/semantic_cnn_native_cmd_latest.pth" \
    "${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}/semantic_cnn_native_cmd_latest.pth"
  cp "${RESULT_DIR}/semantic_cnn_native_cmd_best_dev.pth" \
    "${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}/semantic_cnn_native_cmd_best_dev.pth"
  cp "${RESULT_DIR}/semantic_cnn_native_cmd_final.pth" \
    "${SEMANTIC_CNN_TRAINING_OUTPUT_ROOT}/semantic_cnn_native_cmd_final.pth"
fi

{
  echo "===== Evaluate SemanticCNN training result ====="
  echo "result dir: ${RESULT_DIR}"
  echo "eval dir: ${EVAL_DIR}"
  "${TORCH_PY}" -u "${SCRIPT_DIR}/check_semantic_cnn_training.py" \
    --out-dir "${EVAL_DIR}" \
    --dataset-root "${NATIVE_DATASET_ROOT}" \
    --model-dir "${RESULT_DIR}" \
    --model-code "${RESULT_DIR}/model_code_scripts" \
    --log "${LOG}" \
    --device "${SEMANTIC_CNN_EVAL_DEVICE:-auto}" \
    --periodic-max "${EPOCHS}"
} 2>&1 | tee "${EVAL_LOG}"

set_manifest_var "LAST_SEMANTIC_CNN_RESULT_DIR" "${RESULT_DIR}"
set_manifest_var "LAST_SEMANTIC_CNN_MODEL" "${MODEL}"
set_manifest_var "LAST_SEMANTIC_CNN_BEST_DEV" "${RESULT_DIR}/semantic_cnn_native_cmd_best_dev.pth"
set_manifest_var "LAST_SEMANTIC_CNN_REPORT" "${EVAL_DIR}/semantic_cnn_full_check_report.md"

echo "Wrote ${EVAL_LOG}"
echo "SemanticCNN result dir: ${RESULT_DIR}"
echo "SemanticCNN report: ${EVAL_DIR}/semantic_cnn_full_check_report.md"
