#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --dataset-root, --delete, --device, --ignore-class-ids, --log, --model-code, --model-dir, --num-classes, --out-dir, --output-json, --periodic-max, --split, --stats-json
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, TXT
# 可能使用的关键环境变量：ALLOW_CPU, BASH_SOURCE, BATCH_SIZE, CHECKPOINT_INTERVAL, CUDA, DATASET_INDEX_MODE, DECODE_BEST_DIR, DECODE_FINAL_DIR, EPOCHS, ERROR, EVAL_BATCH_SIZE, EVAL_DIR, EVAL_LOG, FIXED_DUAL_TRAINING_DATASET_ROOT, IGNORE_CLASS_IDS, IGNORE_TAG, LAST_S3NET_BEST_DEV, LAST_S3NET_MODEL, LAST_S3NET_REPORT, LAST_S3NET_RESULT_DIR
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:35.855867577 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_training.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_training.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜09_train_s3net_native_stats.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

# 09_train_s3net_native_stats.sh — S3-Net native stats 正式训练
#
# 每次训练在 training/s3net/ 下创建一个带时间戳的独立结果目录，
# 目录内保存配置、代码快照、stats、日志、周期 checkpoint、
# latest/best/final 模型、dev 评估 JSON、loss 曲线和可视化 decode 图。
#
# 环境变量:
#   S3NET_EPOCHS              训练 epoch 数 (默认 301)
#   S3NET_BATCH_SIZE          训练 batch size (默认 512)
#   S3NET_CHECKPOINT_INTERVAL 周期 checkpoint 间隔 (默认 10)
#   S3NET_EVAL_BATCH_SIZE     eval batch size (默认 128)
#   S3NET_SKIP_DECODE         设为 1 跳过 decode (默认不跳过)
#   S3NET_IGNORE_CLASS_IDS    逗号分隔的训练忽略类 id，默认空；设为 0 可忽略 Other
#   RUN_MANIFEST              manifest 路径 (默认 self/run/run_manifest.env)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_manifest "${RUN_MANIFEST:-}"

DATASET_INDEX_MODE="native"
if [[ -n "${FIXED_DUAL_TRAINING_DATASET_ROOT:-}" ]]; then
  NATIVE_DATASET_ROOT="${FIXED_DUAL_TRAINING_DATASET_ROOT}"
  DATASET_INDEX_MODE="fixed_dual_cmd"
  S3NET_FEATURE_MODE="${S3NET_FEATURE_MODE:-range_incidence}"
else
  NATIVE_DATASET_ROOT="${NATIVE_DATASET_ROOT:-}"
  S3NET_FEATURE_MODE="${S3NET_FEATURE_MODE:-range_intensity_incidence}"
fi
export S3NET_FEATURE_MODE
require_dir "${NATIVE_DATASET_ROOT}" "native converted dataset root"
require_file "${TORCH_PY}" "torch python"
require_file "${NATIVE_DATASET_ROOT}/dataset.txt" "native dataset.txt"
refresh_dataset_index "${NATIVE_DATASET_ROOT}" "${DATASET_INDEX_MODE}" verify
S3NET_NUM_CLASSES="$(dataset_num_classes "${NATIVE_DATASET_ROOT}")"
export S3NET_NUM_CLASSES

# ---- config -----------------------------------------------------------
EPOCHS="${S3NET_EPOCHS:-301}"
BATCH_SIZE="${S3NET_BATCH_SIZE:-512}"
CHECKPOINT_INTERVAL="${S3NET_CHECKPOINT_INTERVAL:-10}"
EVAL_BATCH_SIZE="${S3NET_EVAL_BATCH_SIZE:-128}"
SKIP_DECODE="${S3NET_SKIP_DECODE:-0}"
IGNORE_CLASS_IDS="${S3NET_IGNORE_CLASS_IDS:-}"
ALLOW_CPU="${S3NET_ALLOW_CPU:-0}"
WRITE_COMPAT_ALIASES="${S3NET_WRITE_COMPAT_ALIASES:-0}"
export S3NET_IGNORE_CLASS_IDS="${IGNORE_CLASS_IDS}"
if [[ "${ALLOW_CPU}" != "0" && "${ALLOW_CPU}" != "1" ]]; then
  echo "ERROR: S3NET_ALLOW_CPU must be 0 or 1." >&2
  exit 2
fi
if [[ "${WRITE_COMPAT_ALIASES}" != "0" && "${WRITE_COMPAT_ALIASES}" != "1" ]]; then
  echo "ERROR: S3NET_WRITE_COMPAT_ALIASES must be 0 or 1." >&2
  exit 2
fi
TRAIN_DEVICE="$("${TORCH_PY}" -c 'import torch; print("cuda" if torch.cuda.is_available() else "cpu")')"
if [[ "${TRAIN_DEVICE}" != "cuda" && "${ALLOW_CPU}" != "1" ]]; then
  echo "ERROR: CUDA is unavailable; refusing full S3-Net training on CPU." >&2
  echo "Set S3NET_ALLOW_CPU=1 only for an intentional bounded CPU run." >&2
  exit 2
fi

STEP_TS="$(timestamp)"
IGNORE_TAG=""
if [[ -n "${IGNORE_CLASS_IDS}" ]]; then
  IGNORE_TAG="_ignore_classes_${IGNORE_CLASS_IDS//,/_}"
fi
STEP_ID="${STEP_TS}_s3net_native_stats${IGNORE_TAG}_${EPOCHS}epoch"
S3NET_TRAINING_OUTPUT_ROOT="${S3NET_TRAINING_OUTPUT_ROOT:-${RUN_ROOT}/training/s3net}"
RESULT_DIR="${S3NET_TRAINING_OUTPUT_ROOT}/${STEP_ID}"
S3_WORK="${RESULT_DIR}/work/s3_net_v7"
MODEL="${RESULT_DIR}/${STEP_ID}_model.pth"
LOG="${RESULT_DIR}/09_train_s3net_native_stats_${STEP_TS}.log"
NATIVE_STATS_JSON="${RESULT_DIR}/s3net_native_lidar_train_stats.json"
EVAL_DIR="${RESULT_DIR}/eval_reports"
EVAL_LOG="${RESULT_DIR}/check_s3net_training_${STEP_TS}.log"
DECODE_BEST_DIR="${RESULT_DIR}/decode_best_dev"
DECODE_FINAL_DIR="${RESULT_DIR}/decode_final"
if [[ -e "${RESULT_DIR}" ]]; then
  echo "ERROR: refusing to overwrite existing S3-Net result directory: ${RESULT_DIR}" >&2
  exit 2
fi
if [[ "${WRITE_COMPAT_ALIASES}" == "1" ]]; then
  for alias_name in latest best_dev final; do
    alias_path="${S3NET_TRAINING_OUTPUT_ROOT}/s3net_native_stats_${alias_name}.pth"
    if [[ -e "${alias_path}" ]]; then
      echo "ERROR: refusing to overwrite compatibility alias: ${alias_path}" >&2
      exit 2
    fi
  done
fi

mkdir -p "${S3_WORK}"

{
  # ---- prepare work dir ------------------------------------------------
  echo "===== Prepare work dir ====="
  rsync -a --delete "${S3NET_ROOT}/" "${S3_WORK}/"

  # ---- save config & code snapshot ------------------------------------
  echo "===== Save config & code snapshot ====="
  rsync -a "${S3_WORK}/scripts/" "${RESULT_DIR}/model_code_scripts/"
  cat > "${RESULT_DIR}/training_config.env" <<EOF
STEP_TS="${STEP_TS}"
STEP_ID="${STEP_ID}"
EPOCHS="${EPOCHS}"
BATCH_SIZE="${BATCH_SIZE}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL}"
EVAL_BATCH_SIZE="${EVAL_BATCH_SIZE}"
S3NET_IGNORE_CLASS_IDS="${IGNORE_CLASS_IDS}"
S3NET_FEATURE_MODE="${S3NET_FEATURE_MODE}"
S3NET_NUM_CLASSES="${S3NET_NUM_CLASSES}"
TRAIN_DEVICE="${TRAIN_DEVICE}"
WRITE_COMPAT_ALIASES="${WRITE_COMPAT_ALIASES}"
S3NET_TRAINING_OUTPUT_ROOT="${S3NET_TRAINING_OUTPUT_ROOT}"
NATIVE_DATASET_ROOT="${NATIVE_DATASET_ROOT}"
MODEL="${MODEL}"
LOG="${LOG}"
NATIVE_STATS_JSON="${NATIVE_STATS_JSON}"
MODEL_CODE_SNAPSHOT="${RESULT_DIR}/model_code_scripts"
EOF

  cd "${S3_WORK}"

  echo "===== Compute stats ====="
  stats_args=(
    scripts/compute_dataset_stats.py
    "${NATIVE_DATASET_ROOT}"
    "${NATIVE_STATS_JSON}"
    --split train
    --num-classes "${S3NET_NUM_CLASSES}"
  )
  if [[ -n "${IGNORE_CLASS_IDS}" ]]; then
    stats_args+=(--ignore-class-ids "${IGNORE_CLASS_IDS}")
  fi
  "${TORCH_PY}" "${stats_args[@]}"

  # ---- train ----------------------------------------------------------
  echo "===== Train S3-Net: ${EPOCHS} epochs, batch ${BATCH_SIZE} ====="
  echo "result dir: ${RESULT_DIR}"
  echo "model path: ${MODEL}"
  echo "dataset root: ${NATIVE_DATASET_ROOT}"
  echo "checkpoint interval: ${CHECKPOINT_INTERVAL}"
  echo "ignore class ids: ${IGNORE_CLASS_IDS:-none}"
  echo "feature mode: ${S3NET_FEATURE_MODE}"
  echo "num classes: ${S3NET_NUM_CLASSES}"
  echo "device: ${TRAIN_DEVICE}"
  S3NET_STATS_JSON="${NATIVE_STATS_JSON}" "${TORCH_PY}" - <<PY
import sys
sys.path.insert(0, "scripts")
import train
train.NUM_EPOCHS = ${EPOCHS}
train.BATCH_SIZE = ${BATCH_SIZE}
train.CHECKPOINT_INTERVAL = ${CHECKPOINT_INTERVAL}
train.main([
    "${MODEL}",
    "${NATIVE_DATASET_ROOT}/",
    "${NATIVE_DATASET_ROOT}/",
])
PY

  # ---- verify checkpoint outputs --------------------------------------
  echo ""
  echo "===== Checkpoint outputs ====="
  ls -lh "${MODEL}" \
    "${RESULT_DIR}/s3net_native_stats_latest.pth" \
    "${RESULT_DIR}/s3net_native_stats_best_dev.pth" \
    "${RESULT_DIR}/s3net_native_stats_final.pth" || true

  # ---- evaluate dev ---------------------------------------------------
  echo ""
  echo "===== Evaluate S3-Net (dev split) ====="
  S3NET_STATS_JSON="${NATIVE_STATS_JSON}" "${TORCH_PY}" scripts/evaluate_segmentation.py \
    "${RESULT_DIR}/s3net_native_stats_best_dev.pth" \
    "${NATIVE_DATASET_ROOT}" \
    --split dev \
    --stats-json "${NATIVE_STATS_JSON}" \
    --batch-size "${EVAL_BATCH_SIZE}" \
    --output-json "${RESULT_DIR}/s3net_native_stats_best_dev_eval_dev.json"

  # ---- decode visuals -------------------------------------------------
  if [[ "${SKIP_DECODE}" == "1" ]]; then
    echo ""
    echo "===== Decode SKIPPED (S3NET_SKIP_DECODE=1) ====="
  else
    echo ""
    echo "===== Decode best_dev ====="
    MPLBACKEND=Agg S3NET_STATS_JSON="${NATIVE_STATS_JSON}" "${TORCH_PY}" scripts/decode_demo.py \
      "${DECODE_BEST_DIR}" \
      "${RESULT_DIR}/s3net_native_stats_best_dev.pth" \
      "${NATIVE_DATASET_ROOT}/"

    echo ""
    echo "===== Decode final ====="
    MPLBACKEND=Agg S3NET_STATS_JSON="${NATIVE_STATS_JSON}" "${TORCH_PY}" scripts/decode_demo.py \
      "${DECODE_FINAL_DIR}" \
      "${RESULT_DIR}/s3net_native_stats_final.pth" \
      "${NATIVE_DATASET_ROOT}/"
  fi

  echo ""
  ls -lh "${MODEL}" "${NATIVE_STATS_JSON}" "${RESULT_DIR}/s3net_native_stats_best_dev_eval_dev.json" || true
} 2>&1 | tee "${LOG}"

echo ""
echo "Wrote ${LOG}"

# ---- copy shortcut checkpoints to training/s3net/ ----------------------
if [[ "${WRITE_COMPAT_ALIASES}" == "1" ]]; then
  cp "${RESULT_DIR}/s3net_native_stats_latest.pth" \
    "${S3NET_TRAINING_OUTPUT_ROOT}/s3net_native_stats_latest.pth"
  cp "${RESULT_DIR}/s3net_native_stats_best_dev.pth" \
    "${S3NET_TRAINING_OUTPUT_ROOT}/s3net_native_stats_best_dev.pth"
  cp "${RESULT_DIR}/s3net_native_stats_final.pth" \
    "${S3NET_TRAINING_OUTPUT_ROOT}/s3net_native_stats_final.pth"
fi

# ---- run full training check ------------------------------------------
{
  echo ""
  echo "===== Evaluate S3-Net training result ====="
  echo "result dir: ${RESULT_DIR}"
  echo "eval dir: ${EVAL_DIR}"
  "${TORCH_PY}" -u "${SCRIPT_DIR}/check_s3net_training.py" \
    --out-dir "${EVAL_DIR}" \
    --dataset-root "${NATIVE_DATASET_ROOT}" \
    --model-dir "${RESULT_DIR}" \
    --model-code "${RESULT_DIR}/model_code_scripts" \
    --stats-json "${NATIVE_STATS_JSON}" \
    --log "${LOG}" \
    --device "${S3NET_EVAL_DEVICE:-auto}" \
    --periodic-max "${EPOCHS}"
} 2>&1 | tee "${EVAL_LOG}"

# ---- update manifest ---------------------------------------------------
set_manifest_var "LAST_S3NET_RESULT_DIR" "${RESULT_DIR}"
set_manifest_var "LAST_S3NET_MODEL" "${MODEL}"
set_manifest_var "LAST_S3NET_BEST_DEV" "${RESULT_DIR}/s3net_native_stats_best_dev.pth"
set_manifest_var "LAST_S3NET_REPORT" "${EVAL_DIR}/s3net_full_check_report.md"

echo ""
echo "Wrote ${EVAL_LOG}"
echo "S3-Net result dir: ${RESULT_DIR}"
echo "S3-Net report: ${EVAL_DIR}/s3net_full_check_report.md"
