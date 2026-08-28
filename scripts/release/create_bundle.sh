#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--exclude, --zstd
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：BASE, BASH_SOURCE, MANIFEST, PARENT, RELEASE_MANIFEST, ROOT, STAMP, VERSION
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/scripts/release/create_bundle.sh
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:46:55.725904249 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:28.982870277 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（执行该脚本，使用其处理结果继续当前流程）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/release/create_bundle.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜create_bundle.sh】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE="$(basename "${ROOT}")"
PARENT="$(dirname "${ROOT}")"
VERSION="$(tr -d '[:space:]' < "${ROOT}/VERSION")"
STAMP="$(date +%Y%m%d)"
OUT="${ROOT}/dist/${BASE}-${VERSION}-${STAMP}.tar.zst"
MANIFEST="${ROOT}/RELEASE_MANIFEST.sha256"

python3 "${ROOT}/scripts/validation/verify_portable_bundle.py" "${ROOT}"

(
  cd "${ROOT}"
  find . -type f \
    ! -path "./.venvs/*" \
    ! -path "./.runtime/*" \
    ! -path "./workspaces/ros2_ws/build/*" \
    ! -path "./workspaces/ros2_ws/install/*" \
    ! -path "./workspaces/ros2_ws/log/*" \
    ! -path "./workspaces/ros2_ws/build.stale-*/*" \
    ! -path "./workspaces/ros2_ws/install.stale-*/*" \
    ! -path "./workspaces/ros2_ws/log.stale-*/*" \
    ! -path "./runs/*" \
    ! -path "./dist/*" \
    ! -path "./RELEASE_MANIFEST.sha256" \
    ! -path "*/__pycache__/*" \
    ! -name "*.pyc" \
    -print0 | sort -z | xargs -0 sha256sum
) > "${MANIFEST}"

tar --zstd -cf "${OUT}" \
  --exclude="${BASE}/.venvs/*" \
  --exclude="${BASE}/.runtime/*" \
  --exclude="${BASE}/workspaces/ros2_ws/build" \
  --exclude="${BASE}/workspaces/ros2_ws/install" \
  --exclude="${BASE}/workspaces/ros2_ws/log" \
  --exclude="${BASE}/workspaces/ros2_ws/build.stale-*" \
  --exclude="${BASE}/workspaces/ros2_ws/install.stale-*" \
  --exclude="${BASE}/workspaces/ros2_ws/log.stale-*" \
  --exclude="${BASE}/runs/*" \
  --exclude="${BASE}/dist/*" \
  --exclude="*/__pycache__" \
  --exclude="*.pyc" \
  -C "${PARENT}" "${BASE}"
sha256sum "${OUT}" > "${OUT}.sha256"

echo "bundle=${OUT}"
echo "checksum=${OUT}.sha256"
