#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--duration, --iso-8601, --profile, --ros-args, --stop-distance
# 代码中检测到的 ROS 2 话题/路径字符串：/pedestrian_ground_truth
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：YAML
# 可能使用的关键环境变量：AUTONOMOUS_CAPTURE_COMPLETE, BAG_DIR, BAG_METADATA, BASH_SOURCE, CAPTURE_MANIFEST, COMPLETED_AT, CONTROLLER_PIDS, CONTROLLER_SEED, DRIVER, DRIVER_LOG, DURATION, DURATION_SECONDS, ERROR, EXIT, GROUND_TRUTH_COUNT, GROUND_TRUTH_TYPE, IGN_PARTITION, LAST_BAG_DIR, PEDESTRIAN_GROUND_TRUTH_COUNT, PEDESTRIAN_GROUND_TRUTH_TOPIC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/drive_v7_safe_profile.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/cmd_vel_stamper.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（执行该脚本，使用其输出继续当前流程）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/drive_v7_safe_profile.py; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/cmd_vel_stamper.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜05c_record_autonomous_profile.sh】
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
export ROS_DOMAIN_ID IGN_PARTITION

PROFILE="${1:-}"
DURATION="${2:-90}"
if [[ ! "${PROFILE}" =~ ^[123]$ ]]; then
  echo "usage: $0 PROFILE(1|2|3) [DURATION_SECONDS]" >&2
  exit 2
fi
if [[ ! "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${DURATION}" == "0" ]]; then
  echo "ERROR: duration must be a positive number" >&2
  exit 2
fi

STEP_ID="$(timestamp)_ped_seed${PEDESTRIAN_SEED:-unknown}_profile${PROFILE}_v7_dual_teleop_bag"
while [[ -e "${RUN_ROOT}/bags/raw/${STEP_ID}" ]]; do
  sleep 1
  STEP_ID="$(timestamp)_ped_seed${PEDESTRIAN_SEED:-unknown}_profile${PROFILE}_v7_dual_teleop_bag"
done

BAG_DIR="${RUN_ROOT}/bags/raw/${STEP_ID}"
LOG="${RUN_ROOT}/logs/05c_record_autonomous_profile_${STEP_ID}.log"
STAMPER_LOG="${RUN_ROOT}/logs/05c_cmd_vel_stamper_${STEP_ID}.log"
DRIVER_LOG="${RUN_ROOT}/logs/05c_safe_driver_${STEP_ID}.log"
CAPTURE_MANIFEST="${RUN_ROOT}/bags/capture_manifests/${STEP_ID}.env"
STAMPER="${ROS_WS}/tools/cmd_vel_stamper.py"
DRIVER="${SCRIPT_DIR}/drive_v7_safe_profile.py"

mkdir -p "${RUN_ROOT}/bags/raw" "${RUN_ROOT}/bags/capture_manifests" "${RUN_ROOT}/logs"
set_manifest_var "LAST_BAG_DIR" "${BAG_DIR}"
set_manifest_var "BAG_DIR" "${BAG_DIR}"

STAMPER_PID=""
RECORDER_PID=""
cleanup() {
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" >/dev/null 2>&1; then
    kill -INT "${RECORDER_PID}" >/dev/null 2>&1 || true
    wait "${RECORDER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${STAMPER_PID}" ]] && kill -0 "${STAMPER_PID}" >/dev/null 2>&1; then
    kill "${STAMPER_PID}" >/dev/null 2>&1 || true
    wait "${STAMPER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

mapfile -t CONTROLLER_PIDS < <(
  pgrep -f '^python3 .*/scenario_pedestrian_controller.py --ros-args' || true
)
if [[ "${#CONTROLLER_PIDS[@]}" -ne 1 ]]; then
  echo "ERROR: expected exactly one pedestrian controller; found ${#CONTROLLER_PIDS[@]}" >&2
  printf 'controller_pid=%s\n' "${CONTROLLER_PIDS[@]}" >&2
  exit 3
fi

CONTROLLER_SEED="$(ros2 param get /scenario_pedestrian_controller seed 2>/dev/null | awk '{print $NF}')"
if [[ -z "${CONTROLLER_SEED}" ]]; then
  echo "ERROR: pedestrian controller or seed parameter is unavailable" >&2
  exit 3
fi
if [[ -n "${PEDESTRIAN_SEED:-}" && "${CONTROLLER_SEED}" != "${PEDESTRIAN_SEED}" ]]; then
  echo "ERROR: running pedestrian seed ${CONTROLLER_SEED} != requested ${PEDESTRIAN_SEED}" >&2
  exit 3
fi
GROUND_TRUTH_TYPE="$(ros2 topic type /pedestrian_ground_truth 2>/dev/null || true)"
if [[ "${GROUND_TRUTH_TYPE}" != "semantic_nav_gazebo/msg/PedestrianStateArray" ]]; then
  echo "ERROR: /pedestrian_ground_truth has type '${GROUND_TRUTH_TYPE:-missing}', expected semantic_nav_gazebo/msg/PedestrianStateArray" >&2
  echo "Rebuild and source semantic_nav_gazebo before recording a pedestrian profile." >&2
  exit 3
fi

cat >"${CAPTURE_MANIFEST}" <<EOF
RUN_ID="${RUN_ID}"
STARTED_AT="$(date --iso-8601=seconds)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
IGN_PARTITION="${IGN_PARTITION}"
PEDESTRIAN_SEED="${CONTROLLER_SEED}"
PROFILE="${PROFILE}"
DURATION_SECONDS="${DURATION}"
STOP_DISTANCE_METERS="1.0"
PEDESTRIAN_GROUND_TRUTH_TOPIC="/pedestrian_ground_truth"
PEDESTRIAN_GROUND_TRUTH_TYPE="${GROUND_TRUTH_TYPE}"
BAG_DIR="${BAG_DIR}"
EOF

python3 "${STAMPER}" >"${STAMPER_LOG}" 2>&1 &
STAMPER_PID="$!"
sleep 1
kill -0 "${STAMPER_PID}"

ros2 bag record \
  -o "${BAG_DIR}" \
  /scan_merged /scan_01 /scan_02 /odom /tf /tf_static \
  /cmd_vel /cmd_vel_stamped /pedestrian_ground_truth /clock \
  >"${LOG}" 2>&1 &
RECORDER_PID="$!"
sleep 3
kill -0 "${RECORDER_PID}"

echo "Recording ${BAG_DIR}"
echo "pedestrian_seed=${CONTROLLER_SEED} profile=${PROFILE} duration=${DURATION}s"
python3 "${DRIVER}" \
  --profile "${PROFILE}" \
  --duration "${DURATION}" \
  --stop-distance 1.0 \
  >"${DRIVER_LOG}" 2>&1

kill -INT "${RECORDER_PID}"
wait "${RECORDER_PID}"
RECORDER_PID=""
kill "${STAMPER_PID}" >/dev/null 2>&1 || true
wait "${STAMPER_PID}" >/dev/null 2>&1 || true
STAMPER_PID=""

require_file "${BAG_DIR}/metadata.yaml" "recorded bag metadata"
if ! grep -q "name: /pedestrian_ground_truth" "${BAG_DIR}/metadata.yaml"; then
  echo "ERROR: completed pedestrian capture is missing /pedestrian_ground_truth" >&2
  exit 4
fi
GROUND_TRUTH_COUNT="$(
  awk '
    /name: \/pedestrian_ground_truth$/ { found=1; next }
    found && /message_count:/ { print $2; exit }
  ' "${BAG_DIR}/metadata.yaml"
)"
if [[ ! "${GROUND_TRUTH_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: /pedestrian_ground_truth has no recorded messages" >&2
  exit 4
fi
{
  echo "COMPLETED_AT=\"$(date --iso-8601=seconds)\""
  echo "BAG_METADATA=\"${BAG_DIR}/metadata.yaml\""
  echo "PEDESTRIAN_GROUND_TRUTH_COUNT=\"${GROUND_TRUTH_COUNT}\""
  echo "RECORDER_LOG=\"${LOG}\""
  echo "DRIVER_LOG=\"${DRIVER_LOG}\""
} >>"${CAPTURE_MANIFEST}"

echo "AUTONOMOUS_CAPTURE_COMPLETE"
echo "BAG_DIR=${BAG_DIR}"
echo "CAPTURE_MANIFEST=${CAPTURE_MANIFEST}"
echo "DRIVER_LOG=${DRIVER_LOG}"
