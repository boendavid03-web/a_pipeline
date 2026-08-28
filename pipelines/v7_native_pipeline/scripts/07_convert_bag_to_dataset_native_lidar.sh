#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --base-frame, --cmd-vel-topic, --dev-ratio, --map-frame, --map-yaml, --odom-topic, --output-root, --overwrite, --person-label-mode, --pose-source, --report-md, --scan-topic, --semantic-label, --session-name, --split-seed, --subgoal-lookahead, --test-ratio, --train-ratio, --write-projection-debug
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, PNG, YAML
# 可能使用的关键环境变量：BAG_DIR, BAG_TS, BASE_FRAME, BASH_SOURCE, CHECK_LOG, CHECK_STATUS, CHECK_TS, CMD_VEL_TOPIC, ERROR, FINAL_STATUS, MAP_FRAME, NATIVE_ALLOW_OVERWRITE, NATIVE_BAG_DIR, NATIVE_DATASET_ROOT, NATIVE_DATASET_SESSION_DIR, NATIVE_DEV_RATIO, NATIVE_MAP_YAML, NATIVE_PERSON_LABEL_MODE, NATIVE_SEMANTIC_LABEL_PNG, NATIVE_SESSION_NAME
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_native_lidar_tf_alignment.py（执行该脚本，使用其输出继续当前流程）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py（执行该脚本，使用其输出继续当前流程）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_native_lidar_tf_alignment.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07_convert_bag_to_dataset_native_lidar.sh】
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

if [[ -n "${NATIVE_BAG_DIR:-}" ]]; then
  BAG_DIR="${NATIVE_BAG_DIR}"
fi
require_dir "${BAG_DIR}" "ROS 2 bag directory"

NATIVE_MAP_YAML="${RUN_ROOT}/maps/semantic_label/map.yaml"
NATIVE_SEMANTIC_LABEL_PNG="${RUN_ROOT}/maps/semantic_label/label.png"
require_file "${NATIVE_MAP_YAML}" "semantic label map yaml"
require_file "${NATIVE_SEMANTIC_LABEL_PNG}" "semantic label png"

NATIVE_TRAIN_RATIO="${NATIVE_TRAIN_RATIO:-0.7}"
NATIVE_DEV_RATIO="${NATIVE_DEV_RATIO:-0.1}"
NATIVE_TEST_RATIO="${NATIVE_TEST_RATIO:-0.2}"
NATIVE_SPLIT_SEED="${NATIVE_SPLIT_SEED:-0}"
NATIVE_PERSON_LABEL_MODE="${NATIVE_PERSON_LABEL_MODE:-dynamic}"
case "${NATIVE_PERSON_LABEL_MODE}" in
  dynamic|disabled) ;;
  *)
    echo "ERROR: NATIVE_PERSON_LABEL_MODE must be dynamic or disabled, got ${NATIVE_PERSON_LABEL_MODE}" >&2
    exit 2
    ;;
esac

CHECK_TS="$(timestamp)"
CHECK_LOG="${RUN_ROOT}/logs/check_native_lidar_tf_alignment_${CHECK_TS}.log"
SCAN_FUSION_REPORT="${RUN_ROOT}/logs/scan_fusion_check_report_${CHECK_TS}.md"
LOG="${RUN_ROOT}/logs/07_convert_bag_to_dataset_native_lidar_$(timestamp).log"
BAG_TS="$(v7_dual_bag_timestamp "${BAG_DIR}")"
STEP_ID="${BAG_TS}_v7_dual_native_lidar_dataset_mode_${NATIVE_PERSON_LABEL_MODE}"
set_manifest_var "NATIVE_DATASET_ROOT" "${RUN_ROOT}/datasets/semantic2d_native_lidar_mode_${NATIVE_PERSON_LABEL_MODE}"
set_manifest_var "NATIVE_SESSION_NAME" "${STEP_ID}-converted"
set_manifest_var "NATIVE_DATASET_SESSION_DIR" "${NATIVE_DATASET_ROOT}/${NATIVE_SESSION_NAME}"

if [[ -e "${NATIVE_DATASET_SESSION_DIR}" && "${NATIVE_ALLOW_OVERWRITE:-0}" != "1" ]]; then
  echo "ERROR: target session exists: ${NATIVE_DATASET_SESSION_DIR}; set NATIVE_ALLOW_OVERWRITE=1 to replace it" >&2
  exit 2
fi

echo "BAG_DIR=${BAG_DIR}"
echo "NATIVE_PERSON_LABEL_MODE=${NATIVE_PERSON_LABEL_MODE}"
echo "NATIVE_SESSION_NAME=${NATIVE_SESSION_NAME}"

echo "Checking native LiDAR TF alignment."
python3 "${ROS_WS}/tools/check_native_lidar_tf_alignment.py" \
  --bag "${BAG_DIR}" \
  --map-yaml "${NATIVE_MAP_YAML}" \
  --scan-topic "${SCAN_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --cmd-vel-topic "${CMD_VEL_TOPIC}" \
  --base-frame "${BASE_FRAME:-base_link}" \
  --map-frame "${MAP_FRAME:-map}" \
  --report-md "${SCAN_FUSION_REPORT}" \
  2>&1 | tee "${CHECK_LOG}"

CHECK_STATUS="$(grep '^FINAL_STATUS=' "${CHECK_LOG}" | tail -n 1 | cut -d= -f2)"
if [[ "${CHECK_STATUS}" == "unsafe" ]]; then
  echo "ERROR: native LiDAR TF alignment check is unsafe; see ${CHECK_LOG}" >&2
  exit 2
fi
if [[ "${CHECK_STATUS}" == "warning" ]]; then
  echo "WARNING: native LiDAR TF alignment check reported warning; continuing. See ${CHECK_LOG}" >&2
fi

{
  echo "Converting bag to native-LiDAR Semantic2D dataset."
  OVERWRITE_ARGS=()
  if [[ "${NATIVE_ALLOW_OVERWRITE:-0}" == "1" ]]; then
    OVERWRITE_ARGS=(--overwrite)
  fi
  python3 "${ROS_WS}/tools/convert_rosbag2_to_semantic2d_native_lidar.py" \
    --bag "${BAG_DIR}" \
    --output-root "${NATIVE_DATASET_ROOT}" \
    --session-name "${NATIVE_SESSION_NAME}" \
    --map-yaml "${NATIVE_MAP_YAML}" \
    --semantic-label "${NATIVE_SEMANTIC_LABEL_PNG}" \
    --scan-topic "${SCAN_TOPIC}" \
    --odom-topic "${ODOM_TOPIC}" \
    --cmd-vel-topic "${CMD_VEL_TOPIC}" \
    --train-ratio "${NATIVE_TRAIN_RATIO}" \
    --dev-ratio "${NATIVE_DEV_RATIO}" \
    --test-ratio "${NATIVE_TEST_RATIO}" \
    --split-seed "${NATIVE_SPLIT_SEED}" \
    --subgoal-lookahead "${SUBGOAL_LOOKAHEAD}" \
    --pose-source auto \
    --base-frame "${BASE_FRAME:-base_link}" \
    --map-frame "${MAP_FRAME:-map}" \
    --person-label-mode "${NATIVE_PERSON_LABEL_MODE}" \
    --write-projection-debug \
    "${OVERWRITE_ARGS[@]}"
  echo
  refresh_dataset_index "${NATIVE_DATASET_ROOT}" native_cmd
  echo
  find "${NATIVE_DATASET_ROOT}" -maxdepth 2 -type f | sort
  echo
  python3 - "${NATIVE_DATASET_SESSION_DIR}" <<'PY'
import json
import sys
from pathlib import Path

import numpy as np

session_dir = Path(sys.argv[1])
sample = sorted((session_dir / "scans_lidar").glob("*.npy"))[0].name
checks = [
    ("scan", "scans_lidar"),
    ("angles", "angles_lidar"),
    ("valid_mask", "valid_mask_lidar"),
    ("semantic_label", "semantic_label"),
    ("position", "positions"),
    ("velocity", "velocities"),
    ("cmd_velocity", "cmd_velocities"),
    ("subgoal", "sub_goals_local"),
]
print(f"First sample: {sample}")
for label, subdir in checks:
    path = session_dir / subdir / sample
    if not path.exists():
        print(f"{label}: missing")
        continue
    arr = np.load(path)
    print(f"{label}: shape={arr.shape} dtype={arr.dtype}")

valid_mask = np.load(session_dir / "valid_mask_lidar" / sample)
semantic = np.load(session_dir / "semantic_label" / sample)
invalid_count = int((~valid_mask).sum())
if invalid_count:
    invalid_labels_are_ignore = bool(np.all(semantic[~valid_mask] == -1))
else:
    invalid_labels_are_ignore = None
print(f"invalid_beams={invalid_count}")
print(f"invalid_labels_are_-1={invalid_labels_are_ignore}")
print(f"semantic_contains_-1={bool(np.any(semantic == -1))}")

metadata = json.loads((session_dir / "metadata.json").read_text())
for key in (
    "native_lidar",
    "interpolated_to_baseline_1081",
    "ignore_label",
    "beam_count_unique",
    "beam_count_min",
    "beam_count_max",
    "train_ratio",
    "dev_ratio",
    "test_ratio",
    "split_seed",
    "train_samples",
    "dev_samples",
    "test_samples",
    "velocity_source_used",
    "cmd_velocities_generated",
    "cmd_velocity_dim",
    "cmd_velocities_source",
    "cmd_vel_match_policy",
    "cmd_vel_alignment_status",
    "cmd_vel_clock_mapping_status",
    "cmd_vel_clock_mapping_monotonic",
    "cmd_velocities_nonzero_count",
    "cmd_velocities_no_prior_count",
    "cmd_velocities_hold_last_after_final_count",
    "tf_alignment_status",
    "pose_source_used",
    "fallback_to_odom",
):
    print(f"metadata.{key}={metadata.get(key)}")
PY
} 2>&1 | tee "${LOG}"

echo "Wrote ${LOG}"
