#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：A_PIPELINE_ROOT, BASH_SOURCE, IGN_GAZEBO_RESOURCE_PATH, LABELME_BIN, LABELME_PY, NAVIGATION_PROJECT_ROOT, PIPELINE_ROOT, PROJECT_ROOT, ROS_HOME, ROS_LOG_DIR, ROS_WS, RUNS_ROOT, TORCH_PY, XDG_CACHE_HOME, XDG_CONFIG_HOME, XDG_DATA_HOME
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/environment/activate.sh
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
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_gazebo_ped_map_comparison.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（source 加载公共环境变量/函数）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/environment/activate.sh
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_gazebo_ped_map_comparison.sh; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

A_PIPELINE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export A_PIPELINE_ROOT
export PROJECT_ROOT="${A_PIPELINE_ROOT}"
export NAVIGATION_PROJECT_ROOT="${A_PIPELINE_ROOT}"
export PIPELINE_ROOT="${A_PIPELINE_ROOT}/pipelines/v7_native_pipeline"
export ROS_WS="${A_PIPELINE_ROOT}/workspaces/ros2_ws"
export RUNS_ROOT="${A_PIPELINE_ROOT}/runs"
export TORCH_PY="${A_PIPELINE_ROOT}/.venvs/train/bin/python"
export LABELME_PY="${A_PIPELINE_ROOT}/.venvs/labelme/bin/python"
export LABELME_BIN="${A_PIPELINE_ROOT}/.venvs/labelme/bin/labelme"
export ROS_HOME="${A_PIPELINE_ROOT}/.runtime/ros_home"
export ROS_LOG_DIR="${A_PIPELINE_ROOT}/.runtime/ros_logs"
export XDG_CACHE_HOME="${A_PIPELINE_ROOT}/.runtime/cache"
export XDG_CONFIG_HOME="${A_PIPELINE_ROOT}/.runtime/config"
export XDG_DATA_HOME="${A_PIPELINE_ROOT}/.runtime/data"
# Keep the local Ignition / Gazebo transport on loopback.  On laptops with
# Wi-Fi, automatic interface selection can advertise an address that local
# ROS-Gazebo processes cannot use reliably.
export IGN_IP="${IGN_IP:-127.0.0.1}"
export IGN_GAZEBO_RESOURCE_PATH="${A_PIPELINE_ROOT}/workspaces/ros2_ws/src/semantic_nav_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}"

# Terminals opened by Snap-packaged IDEs can inject an older GTK/GIO runtime.
# Child GUI programs such as rviz2 must use the host Ubuntu libraries instead.
while IFS= read -r snap_var; do
  unset "${snap_var}"
done < <(env | awk -F= '/^SNAP[A-Z0-9_]*=/{print $1}')
unset GTK_PATH LOCPATH SNAP_LIBRARY_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
export XDG_DATA_DIRS="${XDG_DATA_DIRS:-/usr/local/share:/usr/share}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}" "${XDG_CACHE_HOME}" "${XDG_CONFIG_HOME}" "${XDG_DATA_HOME}" "${RUNS_ROOT}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  old_flags="$-"
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
    source "${ROS_WS}/install/setup.bash"
  fi
  case "${old_flags}" in *u*) set -u ;; esac
fi

echo "A_PIPELINE_ROOT=${A_PIPELINE_ROOT}"
