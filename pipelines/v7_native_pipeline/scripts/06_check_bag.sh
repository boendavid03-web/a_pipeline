#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--all, --bag, --min-duration-sec
# 代码中检测到的 ROS 2 话题/路径字符串：/semantic_cnn/final_goal, /semantic_cnn/global_path, /semantic_cnn/local_subgoal
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：YAML
# 可能使用的关键环境变量：ALL_BAGS, BAG_DIR, BAG_DIRS, BASH_SOURCE, ERROR, INFO, MIN_BAG_SIM_DURATION_SEC, MIN_BAG_SIM_DURATION_SEC_VALUE, MISSING, MODE, PASS, PROJECT_ROOT, REQUESTED_BAG_DIR, REQUIRE_PEDESTRIAN_GROUND_TRUTH, REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE, ROS_WS, RUN_MANIFEST, RUN_ROOT, SCRIPT_DIR, SKIP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:27.122706557 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py（执行该脚本，使用其输出继续当前流程）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_pedestrian_ground_truth_bag.py（执行该脚本，使用其输出继续当前流程）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（ros2 run 启动该节点）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_cmd_vel_stamped_bag.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_pedestrian_ground_truth_bag.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜06_check_bag.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

MODE="latest"
REQUESTED_BAG_DIR=""
case "${1:-}" in
  --all|-a)
    MODE="all"
    ;;
  "")
    ;;
  *)
    MODE="path"
    REQUESTED_BAG_DIR="$1"
    ;;
esac

load_manifest "${RUN_MANIFEST:-}"

safe_source_ros

LOG="${RUN_ROOT}/logs/06_check_bag_$(timestamp).log"
REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE="${REQUIRE_PEDESTRIAN_GROUND_TRUTH:-0}"
MIN_BAG_SIM_DURATION_SEC_VALUE="${MIN_BAG_SIM_DURATION_SEC:-0}"
case "${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE,,}" in
  1|true|yes|on)
    REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE=1
    ;;
  0|false|no|off)
    REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE=0
    ;;
  *)
    echo "ERROR: REQUIRE_PEDESTRIAN_GROUND_TRUTH must be 0/1 or false/true" >&2
    exit 2
    ;;
esac
if ! [[ "${MIN_BAG_SIM_DURATION_SEC_VALUE}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: MIN_BAG_SIM_DURATION_SEC must be zero or a positive number" >&2
  exit 2
fi

check_bag() {
  local bag_dir="$1"
  local missing_required=0

  require_dir "${bag_dir}" "ROS 2 bag directory"
  require_file "${bag_dir}/metadata.yaml" "ROS 2 bag metadata"

  echo "===== ros2 bag info: ${bag_dir} ====="
  ros2 bag info "${bag_dir}"
  echo
  echo "===== required topics in metadata: ${bag_dir} ====="
  for topic in /scan_merged /scan_01 /scan_02 /odom /tf /tf_static /cmd_vel /cmd_vel_stamped /clock; do
    if grep -R "name: ${topic}" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
      echo "PASS ${topic}"
    else
      echo "MISSING ${topic}"
      missing_required=1
    fi
  done
  if grep -R "name: /pedestrian_ground_truth" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
    echo "PASS /pedestrian_ground_truth"
  else
    if [[ "${REQUIRE_PEDESTRIAN_GROUND_TRUTH_VALUE}" == "1" ]]; then
      echo "MISSING /pedestrian_ground_truth"
      missing_required=1
    else
      echo "INFO /pedestrian_ground_truth not recorded (allowed by current check mode)"
    fi
  fi
  for topic in /semantic_cnn/global_path /semantic_cnn/local_subgoal /semantic_cnn/final_goal; do
    if grep -R "name: ${topic}" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
      echo "PASS ${topic}"
    else
      echo "MISSING ${topic}"
      missing_required=1
    fi
  done
  if ! python3 - "${bag_dir}/metadata.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

expected = {
    "/semantic_cnn/global_path": "nav_msgs/msg/Path",
    "/semantic_cnn/local_subgoal": "geometry_msgs/msg/PointStamped",
    "/semantic_cnn/final_goal": "geometry_msgs/msg/PointStamped",
}
metadata = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
entries = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
topics = {
    item["topic_metadata"]["name"]: (
        item["topic_metadata"]["type"],
        int(item["message_count"]),
    )
    for item in entries
}
errors = []
for topic, expected_type in expected.items():
    actual_type, count = topics.get(topic, (None, 0))
    if actual_type != expected_type:
        errors.append(
            f"{topic} type is {actual_type!r}, expected {expected_type!r}"
        )
    if count <= 0:
        errors.append(f"{topic} has no recorded messages")
    if actual_type == expected_type and count > 0:
        print(f"PASS {topic} {actual_type} messages={count}")
if errors:
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(1)
PY
  then
    missing_required=1
  fi
  if [[ "${missing_required}" != "0" ]]; then
    echo "ERROR: required bag topics are missing" >&2
    return 1
  fi
  if grep -R "name: /cmd_vel_stamped" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
    echo
    echo "===== cmd_vel_stamped semantic checks: ${bag_dir} ====="
    python3 "${ROS_WS}/tools/check_cmd_vel_stamped_bag.py" --bag "${bag_dir}"
  else
    echo
    echo "SKIP cmd_vel_stamped semantic checks: /cmd_vel_stamped is missing"
  fi
  if grep -R "name: /pedestrian_ground_truth" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
    echo
    echo "===== pedestrian ground-truth kinematic checks: ${bag_dir} ====="
    python3 "${ROS_WS}/tools/check_pedestrian_ground_truth_bag.py" \
      --bag "${bag_dir}" \
      --min-duration-sec "${MIN_BAG_SIM_DURATION_SEC_VALUE}"
  fi
  if grep -R "name: /data_collection/episode_event" "${bag_dir}/metadata.yaml" >/dev/null 2>&1; then
    echo
    echo "===== automatic episode checks: ${bag_dir} ====="
    python3 \
      "${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py" \
      --bag "${bag_dir}"
  fi
}

{
  if [[ "${ALL_BAGS:-0}" == "1" ]]; then
    MODE="all"
  fi

  if [[ "${MODE}" == "all" ]]; then
    mapfile -t BAG_DIRS < <(list_v7_dual_bag_dirs)
    if [[ "${#BAG_DIRS[@]}" -eq 0 ]]; then
      echo "ERROR: no bags found under ${RUN_ROOT}/bags/raw/*${V7_DUAL_BAG_SUFFIX}" >&2
      exit 2
    fi
    for bag_dir in "${BAG_DIRS[@]}"; do
      check_bag "${bag_dir}"
      echo
    done
  elif [[ "${MODE}" == "path" ]]; then
    check_bag "${REQUESTED_BAG_DIR}"
  else
    if ! BAG_DIR="$(latest_v7_dual_bag_dir)"; then
      echo "ERROR: no latest bag found. Record one first, or pass a bag directory." >&2
      exit 2
    fi
    check_bag "${BAG_DIR}"
  fi
} 2>&1 | tee "${LOG}"

echo "Wrote ${LOG}"
