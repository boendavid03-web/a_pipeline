#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --delete, --num-classes, --output-json, --split, --stats-json
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, TXT
# 可能使用的关键环境变量：BASH_SOURCE, BATCH_SIZE, DATASET_INDEX_MODE, EPOCHS, EVAL_JSON, FIXED_DUAL_TRAINING_DATASET_ROOT, JSON, MODEL, NATIVE_DATASET_ROOT, NUM_EPOCHS, PASSED, RUN_MANIFEST, RUN_ROOT, S3NET_BATCH_SIZE, S3NET_EPOCHS, S3NET_FEATURE_MODE, S3NET_NUM_CLASSES, S3NET_ROOT, S3NET_STATS_JSON, S3_WORK
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜08b_smoke_train_s3net.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

# 08b_smoke_train_s3net.sh — S3-Net only smoke (不跑 SemanticCNN)
# 和 08_smoke_train.sh 的区别：只 smoke S3-Net，避免同时跑 SemanticCNN 造成语义/导航混淆。
#
# 环境变量:
#   S3NET_EPOCHS      smoke epoch 数 (默认 2)
#   S3NET_BATCH_SIZE  smoke batch size (默认 64)
#   RUN_MANIFEST      manifest 路径 (默认 self/run/run_manifest.env)

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
refresh_dataset_index "${NATIVE_DATASET_ROOT}" "${DATASET_INDEX_MODE}"
require_file "${NATIVE_DATASET_ROOT}/dataset.txt" "native dataset.txt"
S3NET_NUM_CLASSES="$(dataset_num_classes "${NATIVE_DATASET_ROOT}")"
export S3NET_NUM_CLASSES

EPOCHS="${S3NET_EPOCHS:-2}"
BATCH_SIZE="${S3NET_BATCH_SIZE:-64}"
S3_WORK="${RUN_ROOT}/training/work/s3_net_v7"
STEP_ID="$(timestamp)_s3net_smoke_${EPOCHS}epoch"
MODEL="${RUN_ROOT}/training/s3net/${STEP_ID}_model.pth"
EVAL_JSON="${RUN_ROOT}/training/s3net/${STEP_ID}_eval_dev.json"
SMOKE_STATS_JSON="${RUN_ROOT}/training/s3net/s3net_native_lidar_smoke_stats.json"
LOG="${RUN_ROOT}/logs/08b_smoke_train_s3net_$(timestamp).log"
mkdir -p "${S3_WORK}" "${RUN_ROOT}/training/s3net" "${RUN_ROOT}/logs"

{
  echo "===== S3-Net Only Smoke ====="
  echo "step_id   : ${STEP_ID}"
  echo "epochs    : ${EPOCHS}"
  echo "batch_size: ${BATCH_SIZE}"
  echo "feature_mode: ${S3NET_FEATURE_MODE}"
  echo "num_classes : ${S3NET_NUM_CLASSES}"
  echo "model     : ${MODEL}"
  echo "stats     : ${SMOKE_STATS_JSON}"
  echo "eval      : ${EVAL_JSON}"
  echo ""

  echo "===== Prepare work dir ====="
  rsync -a --delete "${S3NET_ROOT}/" "${S3_WORK}/"
  cd "${S3_WORK}"

  echo "===== Compute native stats ====="
  "${TORCH_PY}" scripts/compute_dataset_stats.py \
    "${NATIVE_DATASET_ROOT}" \
    "${SMOKE_STATS_JSON}" \
    --split train \
    --num-classes "${S3NET_NUM_CLASSES}"

  echo "===== Smoke train: ${EPOCHS} epochs, batch ${BATCH_SIZE} ====="
  S3NET_STATS_JSON="${SMOKE_STATS_JSON}" "${TORCH_PY}" - <<PY
import sys
sys.path.insert(0, "scripts")
import train
train.NUM_EPOCHS = ${EPOCHS}
train.BATCH_SIZE = ${BATCH_SIZE}
train.main([
    "${MODEL}",
    "${NATIVE_DATASET_ROOT}/",
    "${NATIVE_DATASET_ROOT}/",
])
PY

  echo "===== Smoke eval (dev split) ====="
  S3NET_STATS_JSON="${SMOKE_STATS_JSON}" "${TORCH_PY}" scripts/evaluate_segmentation.py \
    "${MODEL}" \
    "${NATIVE_DATASET_ROOT}" \
    --split dev \
    --stats-json "${SMOKE_STATS_JSON}" \
    --batch-size 64 \
    --output-json "${EVAL_JSON}"

  echo ""
  echo "===== Smoke outputs ====="
  ls -lh "${MODEL}" "${SMOKE_STATS_JSON}" "${EVAL_JSON}"

  echo ""
  echo "===== Smoke PASSED ====="
} 2>&1 | tee "${LOG}"

echo ""
echo "Wrote log: ${LOG}"
echo "Model     : ${MODEL}"
echo "Eval JSON : ${EVAL_JSON}"
