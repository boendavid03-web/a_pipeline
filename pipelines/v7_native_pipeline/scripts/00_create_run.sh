#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PGM, PNG, SDF, YAML
# 可能使用的关键环境变量：BAG_DIR, BASELINE_ROOT, BASH_SOURCE, BEGIN, CMD_VEL_TOPIC, CMD_VEL_TOPIC_VALUE, DATASET_ROOT, DEFAULT_CMD_VEL_TOPIC, DEFAULT_DEV_RATIO, DEFAULT_IGN_PARTITION_PREFIX, DEFAULT_ODOM_TOPIC, DEFAULT_ROS_DOMAIN_ID, DEFAULT_SCAN_TOPIC, DEFAULT_SUBGOAL_LOOKAHEAD, DEFAULT_TARGET_POINTS, DEV_RATIO, DEV_RATIO_VALUE, ERROR, EXPECTED_LIDAR_TOPICS, FORCE_REINIT_RUN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.379546481 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜00_create_run.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

RUN_ID="${RUN_ID:-v7_dual_lidar_pipeline}"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
PIPELINE_ROOT="${PROJECT_ROOT}/pipelines/v7_native_pipeline"
METHODS_ROOT="${PROJECT_ROOT}/methods"
BASELINE_ROOT="${METHODS_ROOT}/baselines"
S3NET_ROOT="${BASELINE_ROOT}/s3net"
SEMANTIC_CNN_ROOT="${BASELINE_ROOT}/semantic_cnn"
ROS_WS="${PROJECT_ROOT}/workspaces/ros2_ws"
RUN_ROOT="${PROJECT_ROOT}/runs/${RUN_ID}"
export NAVIGATION_PROJECT_ROOT="${PROJECT_ROOT}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-${DEFAULT_ROS_DOMAIN_ID}}"
IGN_PARTITION_VALUE="${IGN_PARTITION:-${DEFAULT_IGN_PARTITION_PREFIX}_${RUN_ID}}"
SCAN_TOPIC_VALUE="${SCAN_TOPIC:-${DEFAULT_SCAN_TOPIC}}"
ODOM_TOPIC_VALUE="${ODOM_TOPIC:-${DEFAULT_ODOM_TOPIC}}"
CMD_VEL_TOPIC_VALUE="${CMD_VEL_TOPIC:-${DEFAULT_CMD_VEL_TOPIC}}"
TARGET_POINTS_VALUE="${TARGET_POINTS:-${DEFAULT_TARGET_POINTS}}"
DEV_RATIO_VALUE="${DEV_RATIO:-${DEFAULT_DEV_RATIO}}"
SUBGOAL_LOOKAHEAD_VALUE="${SUBGOAL_LOOKAHEAD:-${DEFAULT_SUBGOAL_LOOKAHEAD}}"
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
LIDAR_RUNTIME_MODEL_FILE_VALUE="${RUN_ROOT}/runtime_models/lidar/model.sdf"

for value in "${LIDAR_SAMPLES_VALUE}" "${LIDAR_SAMPLES_01_VALUE}" "${LIDAR_SAMPLES_02_VALUE}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: LiDAR samples must be positive integers, got: ${value}" >&2
    exit 2
  fi
done
for pair in \
  "${LIDAR_RANGE_MIN_VALUE}:${LIDAR_RANGE_MAX_VALUE}" \
  "${LIDAR_RANGE_MIN_01_VALUE}:${LIDAR_RANGE_MAX_01_VALUE}" \
  "${LIDAR_RANGE_MIN_02_VALUE}:${LIDAR_RANGE_MAX_02_VALUE}"
do
  range_min="${pair%%:*}"
  range_max="${pair#*:}"
  if ! awk -v range_min="${range_min}" -v range_max="${range_max}" \
    'BEGIN { exit !(range_min + 0 > 0 && range_max + 0 > range_min) }'; then
    echo "ERROR: LiDAR range must satisfy 0 < min < max, got: ${pair}" >&2
    exit 2
  fi
done
for value in "${LIDAR_UPDATE_RATE_VALUE}" "${LIDAR_UPDATE_RATE_01_VALUE}" "${LIDAR_UPDATE_RATE_02_VALUE}"; do
  if ! awk -v value="${value}" 'BEGIN { exit !(value + 0 > 0) }'; then
    echo "ERROR: LiDAR update rates must be positive numbers, got: ${value}" >&2
    exit 2
  fi
done

mkdir -p \
  "${RUN_ROOT}/maps/slam" \
  "${RUN_ROOT}/maps/semantic_label" \
  "${RUN_ROOT}/annotations" \
  "${RUN_ROOT}/bags/raw" \
  "${RUN_ROOT}/datasets/semantic2d_baseline" \
  "${RUN_ROOT}/datasets/semantic2d_native_lidar" \
  "${RUN_ROOT}/training/s3net" \
  "${RUN_ROOT}/training/semantic_cnn" \
  "${RUN_ROOT}/training/work" \
  "${RUN_ROOT}/runtime_models/lidar" \
  "${RUN_ROOT}/logs"

if [[ -f "${RUN_ROOT}/run_manifest.env" && "${FORCE_REINIT_RUN:-0}" != "1" ]]; then
  echo "Run folder already exists:"
  echo "  ${RUN_ROOT}"
  echo
  echo "Existing manifest preserved:"
  echo "  ${RUN_ROOT}/run_manifest.env"
  echo
  echo "To rebuild run_manifest.env, rerun with FORCE_REINIT_RUN=1."
  exit 0
fi

cat > "${RUN_ROOT}/run_manifest.env" <<EOF
export RUN_ID="${RUN_ID}"
export PROJECT_ROOT="${PROJECT_ROOT}"
export PIPELINE_ROOT="${PIPELINE_ROOT}"
export METHODS_ROOT="${METHODS_ROOT}"
export RUN_ROOT="${RUN_ROOT}"
export BASELINE_ROOT="${BASELINE_ROOT}"
export S3NET_ROOT="${S3NET_ROOT}"
export SEMANTIC_CNN_ROOT="${SEMANTIC_CNN_ROOT}"
export ROS_WS="${ROS_WS}"
export NAVIGATION_PROJECT_ROOT="${PROJECT_ROOT}"
export TORCH_PY="${TORCH_PY}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export IGN_PARTITION="${IGN_PARTITION_VALUE}"
export SCAN_TOPIC="${SCAN_TOPIC_VALUE}"
export ODOM_TOPIC="${ODOM_TOPIC_VALUE}"
export CMD_VEL_TOPIC="${CMD_VEL_TOPIC_VALUE}"
export TARGET_POINTS="${TARGET_POINTS_VALUE}"
export DEV_RATIO="${DEV_RATIO_VALUE}"
export SUBGOAL_LOOKAHEAD="${SUBGOAL_LOOKAHEAD_VALUE}"
export LIDAR_SAMPLES="${LIDAR_SAMPLES_VALUE}"
export LIDAR_UPDATE_RATE="${LIDAR_UPDATE_RATE_VALUE}"
export LIDAR_SAMPLES_01="${LIDAR_SAMPLES_01_VALUE}"
export LIDAR_SAMPLES_02="${LIDAR_SAMPLES_02_VALUE}"
export LIDAR_UPDATE_RATE_01="${LIDAR_UPDATE_RATE_01_VALUE}"
export LIDAR_UPDATE_RATE_02="${LIDAR_UPDATE_RATE_02_VALUE}"
export LIDAR_RANGE_MIN="${LIDAR_RANGE_MIN_VALUE}"
export LIDAR_RANGE_MAX="${LIDAR_RANGE_MAX_VALUE}"
export LIDAR_RANGE_MIN_01="${LIDAR_RANGE_MIN_01_VALUE}"
export LIDAR_RANGE_MIN_02="${LIDAR_RANGE_MIN_02_VALUE}"
export LIDAR_RANGE_MAX_01="${LIDAR_RANGE_MAX_01_VALUE}"
export LIDAR_RANGE_MAX_02="${LIDAR_RANGE_MAX_02_VALUE}"
export LIDAR_RUNTIME_MODEL_FILE="${RUN_ROOT}/runtime_models/lidar/model.sdf"
export EXPECTED_LIDAR_TOPICS="/scan_01 /scan_02"
export RAW_LIDAR_CONVERSION_CONTRACT="raw capture only; no resampling, cross-LiDAR deduplication, angle binning, or fusion; future 800+800 conversion output is fixed at 1600 slots"
export MAP_BASENAME="${RUN_ID}_slam_map"
export MAP_YAML="${RUN_ROOT}/maps/slam/${RUN_ID}_slam_map.yaml"
export MAP_PGM="${RUN_ROOT}/maps/slam/${RUN_ID}_slam_map.pgm"
export LABELME_IMAGE="${RUN_ROOT}/maps/semantic_label/map.png"
export LABELME_JSON="${RUN_ROOT}/maps/semantic_label/map_labelme.json"
export SEMANTIC_LABEL_DIR="${RUN_ROOT}/maps/semantic_label"
export SEMANTIC_LABEL_PNG="${RUN_ROOT}/maps/semantic_label/label.png"
export BAG_DIR="${RUN_ROOT}/bags/raw/${RUN_ID}_teleop_bag"
export LAST_BAG_DIR=""
export LAST_S3NET_RESULT_DIR=""
export LAST_S3NET_MODEL=""
export LAST_S3NET_BEST_DEV=""
export LAST_S3NET_REPORT=""
export DATASET_ROOT="${RUN_ROOT}/datasets/semantic2d_baseline"
export SESSION_NAME="${RUN_ID}_converted"
export STATS_JSON="${RUN_ROOT}/training/s3net/s3net_train_stats.json"
export LAST_SEMANTIC_CNN_RESULT_DIR=""
export LAST_SEMANTIC_CNN_MODEL=""
export LAST_SEMANTIC_CNN_BEST_DEV=""
export LAST_SEMANTIC_CNN_REPORT=""
EOF

echo "Initialized run:"
echo "  ${RUN_ROOT}"
echo
echo "Use this in later terminals:"
echo "  export RUN_MANIFEST=${RUN_ROOT}/run_manifest.env"
