#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--fps, --max-frames, --output-dir, --session, --stride
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON
# 可能使用的关键环境变量：BASH_SOURCE, ERROR, FIXED_DUAL_VIZ_OUTPUT_DIR, MPLCONFIGDIR, OUTPUT_DIR, PROJECT_ROOT, RUN_ROOT, SCRIPT_DIR, SESSION_ARG, SESSION_DIR, SESSION_NAME, SLOT_ROOT, TMPDIR, VISUALIZER
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07d_visualize_fixed_dual_lidar.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 08:25:34.576386173 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:35.855867577 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_fixed_dual_lidar_session.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07d_visualize_fixed_dual_lidar.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/visualize_fixed_dual_lidar_session.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜07d_visualize_fixed_dual_lidar.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
VISUALIZER="${PROJECT_ROOT}/scripts/validation/ros2_workspace_tools/visualize_fixed_dual_lidar_session.py"

if (( $# < 1 )); then
  echo "Usage: $0 SESSION_DIR [--stride N] [--max-frames N] [--fps FPS] [visualizer options]" >&2
  echo "Optional: FIXED_DUAL_VIZ_OUTPUT_DIR=/new/output/directory" >&2
  exit 2
fi

SESSION_ARG="$1"
shift
if [[ ! -d "${SESSION_ARG}" ]]; then
  echo "ERROR: fixed dual-LiDAR session directory not found: ${SESSION_ARG}" >&2
  exit 2
fi
if [[ ! -f "${SESSION_ARG}/metadata.json" || ! -d "${SESSION_ARG}/samples" ]]; then
  echo "ERROR: expected metadata.json and samples/ under: ${SESSION_ARG}" >&2
  exit 2
fi
if [[ ! -f "${VISUALIZER}" ]]; then
  echo "ERROR: visualizer not found: ${VISUALIZER}" >&2
  exit 2
fi

SESSION_DIR="$(cd "${SESSION_ARG}" && pwd)"
SLOT_ROOT="$(dirname "${SESSION_DIR}")"
if [[ "$(basename "${SLOT_ROOT}")" != "fixed_dual_lidar_slots" ]]; then
  echo "ERROR: session must be directly under a fixed_dual_lidar_slots directory: ${SESSION_DIR}" >&2
  exit 2
fi
RUN_ROOT="$(cd "${SESSION_DIR}/../../.." && pwd)"
SESSION_NAME="$(basename "${SESSION_DIR}")"
OUTPUT_DIR="${FIXED_DUAL_VIZ_OUTPUT_DIR:-${RUN_ROOT}/visualizations/fixed_dual/${SESSION_NAME}}"

if [[ -e "${OUTPUT_DIR}" ]]; then
  echo "ERROR: output already exists; refusing to overwrite: ${OUTPUT_DIR}" >&2
  exit 2
fi

export MPLCONFIGDIR="${MPLCONFIGDIR:-${TMPDIR:-/tmp}/matplotlib-fixed-dual-${UID}}"

echo "Input session: ${SESSION_DIR}"
echo "Output directory: ${OUTPUT_DIR}"
echo "Source artifacts are read-only; existing output directories are rejected."

python3 "${VISUALIZER}" \
  --session "${SESSION_DIR}" \
  --output-dir "${OUTPUT_DIR}" \
  "$@"
