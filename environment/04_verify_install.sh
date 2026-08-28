#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：BASH_SOURCE, CUDA, LABELME_BIN, LABELME_PY, PASS, ROOT, ROS_WS, TORCH_PY
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/environment/04_verify_install.sh
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/activate.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py（执行该脚本，使用其输出继续当前流程）; /home/user/navigation_project/a_pipeline/scripts/validation/verify_smoke_example.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/run_v7_dual_slam.sh（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/environment/activate.sh; /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py; /home/user/navigation_project/a_pipeline/scripts/validation/verify_smoke_example.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/run_v7_dual_slam.sh
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/environment/activate.sh"

test -x "${TORCH_PY}"
test -x "${LABELME_BIN}"
if [[ "$(head -n 1 "${LABELME_BIN}")" != "#!${LABELME_PY}" ]]; then
  echo "ERROR: LabelMe entrypoint belongs to a different project path:" >&2
  echo "  ${LABELME_BIN}" >&2
  echo "Run environment/02_create_python_envs.sh to rebuild copied virtual environments." >&2
  exit 2
fi
test -f "${ROS_WS}/install/setup.bash"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available; install/source ROS 2 Humble before verification." >&2
  exit 2
fi
ros2 pkg executables semantic_nav_gazebo | grep -Fqx 'semantic_nav_gazebo run_v7_dual_slam.sh'
python3 -c 'import rclpy, rosbag2_py, yaml, numpy; print("ROS Python imports PASS")'
"${LABELME_PY}" -c 'import labelme, imgviz; print("LabelMe imports PASS")'
"${TORCH_PY}" -c 'import torch, numpy, tensorboardX, matplotlib; assert torch.cuda.is_available(); print("training imports and CUDA PASS")'
"${TORCH_PY}" "${ROOT}/scripts/validation/verify_smoke_example.py" "${ROOT}/examples/smoke"
python3 "${ROOT}/scripts/validation/verify_portable_bundle.py" "${ROOT}"
echo "PASS: a_pipeline installation verified"
