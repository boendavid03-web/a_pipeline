#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--packages-select, --symlink-install
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：BASH_SOURCE, ERROR, PASS, ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/environment/03_build_ros2_workspace.sh
# 输入来源：操作系统、Python、ROS 2 和项目目录状态。
# 输出结果：虚拟环境、ROS 2 构建结果或环境配置。
# 前置条件：需要系统权限或已安装基础工具。
# 后续步骤：完成后 source 环境并运行项目 pipeline。
# 副作用与安全：可能安装依赖、创建环境或构建工作空间。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:45:42.976870135 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:34.392758946 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/environment/03_build_ros2_workspace.sh
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/workspaces/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble not found; run 01_install_system_deps.sh." >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WS}"

# A copied CMake cache can retain the source path from the machine that built
# the bundle. Move generated ROS artifacts aside so the first build on this
# host starts from the current checkout without deleting the old artifacts.
CMAKE_CACHE="${WS}/build/semantic_nav_gazebo/CMakeCache.txt"
EXPECTED_SOURCE="${WS}/src/semantic_nav_gazebo"
if [[ -f "${CMAKE_CACHE}" ]] && ! grep -Fqx "CMAKE_HOME_DIRECTORY:INTERNAL=${EXPECTED_SOURCE}" "${CMAKE_CACHE}"; then
  STALE_STAMP="$(date +%Y%m%d_%H%M%S)"
  echo "WARN: stale ROS build cache detected; moving build/install/log aside."
  for generated in build install log; do
    if [[ -e "${WS}/${generated}" ]]; then
      mv -- "${WS}/${generated}" "${WS}/${generated}.stale-${STALE_STAMP}"
    fi
  done
fi

colcon build --symlink-install --packages-select semantic_nav_gazebo
echo "PASS: ROS 2 workspace built under ${WS}"
