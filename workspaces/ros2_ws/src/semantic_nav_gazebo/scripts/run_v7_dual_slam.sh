#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--1
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：BASH_SOURCE, GTK_PATH, GUI_VALUE, IGN_PARTITION, IGN_PARTITION_VALUE, LIDAR_RANGE_MAX, LIDAR_RANGE_MAX_01, LIDAR_RANGE_MAX_01_VALUE, LIDAR_RANGE_MAX_02, LIDAR_RANGE_MAX_02_VALUE, LIDAR_RANGE_MAX_VALUE, LIDAR_RANGE_MIN, LIDAR_RANGE_MIN_01, LIDAR_RANGE_MIN_01_VALUE, LIDAR_RANGE_MIN_02, LIDAR_RANGE_MIN_02_VALUE, LIDAR_RANGE_MIN_VALUE, LIDAR_RUNTIME_MODEL_FILE, LIDAR_RUNTIME_MODEL_FILE_VALUE, LIDAR_SAMPLES
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/run_v7_dual_slam.sh
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.643741916 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（通过 ros2 launch 启动该 ROS 2 场景）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh（通过 ros2 run 启动该 ROS 2 节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/run_v7_dual_slam.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜run_v7_dual_slam.sh】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-78}"
IGN_PARTITION_VALUE="${IGN_PARTITION:-semantic_nav_v7_teacher_dual_scan}"
START_RVIZ_VALUE="${START_RVIZ:-false}"
GUI_VALUE="${GUI:-true}"
SPAWN_SCENE_PEDESTRIANS_VALUE="${SPAWN_SCENE_PEDESTRIANS:-false}"
PEDESTRIAN_USE_ACTORS_VALUE="${PEDESTRIAN_USE_ACTORS:-false}"
PEDESTRIAN_UPDATE_RATE_VALUE="${PEDESTRIAN_UPDATE_RATE:-20.0}"
PEDESTRIAN_SIMULATION_FACTOR_VALUE="${PEDESTRIAN_SIMULATION_FACTOR:-1.0}"
PEDESTRIAN_SPEED_VALUE="${PEDESTRIAN_SPEED:-1.34}"
PEDESTRIAN_COUNT_VALUE="${PEDESTRIAN_COUNT:--1}"
PEDESTRIAN_STATIC_OBSTACLE_CLEARANCE_VALUE="${PEDESTRIAN_STATIC_OBSTACLE_CLEARANCE:-0.75}"
PEDESTRIAN_SEED_VALUE="${PEDESTRIAN_SEED:-7}"
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
LIDAR_RUNTIME_MODEL_FILE_VALUE="${LIDAR_RUNTIME_MODEL_FILE:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${NAVIGATION_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../../../../" && pwd)}"
export NAVIGATION_PROJECT_ROOT="${PROJECT_ROOT}"
WORKSPACE="${PROJECT_ROOT}/workspaces/ros2_ws"
USE_OPENUSD_VENV="${USE_OPENUSD_VENV:-false}"
VENV="${PROJECT_ROOT}/.venvs/openusd-core/bin/activate"

# Snap-packaged IDE terminals can leak old core libraries into RViz.
for snap_var in $(env | awk -F= '/^SNAP/{print $1}'); do
  unset "${snap_var}"
done
unset GTK_PATH LOCPATH SNAP_LIBRARY_PATH

if [[ "${USE_OPENUSD_VENV}" == "true" && -f "${VENV}" ]]; then
  # shellcheck disable=SC1090
  source "${VENV}"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export IGN_PARTITION="${IGN_PARTITION_VALUE}"

cleanup_partition_processes() {
  local pid env domain partition cmd
  while read -r pid cmd; do
    [[ -z "${pid}" ]] && continue
    [[ "${pid}" == "$$" ]] && continue
    env="/proc/${pid}/environ"
    [[ -r "${env}" ]] || continue
    domain="$(tr '\0' '\n' < "${env}" | awk -F= '$1=="ROS_DOMAIN_ID"{print $2; exit}')"
    partition="$(tr '\0' '\n' < "${env}" | awk -F= '$1=="IGN_PARTITION"{print $2; exit}')"
    if [[ "${domain}" == "${ROS_DOMAIN_ID}" && "${partition}" == "${IGN_PARTITION}" ]]; then
      echo "Stopping stale process in this run: pid=${pid} ${cmd}"
      kill "${pid}" 2>/dev/null || true
    fi
  done < <(
    pgrep -af 'semantic_cnn_nav_v7_dual_bringup|scenario_pedestrian_controller|v7_dual_laser_scan_merger|async_slam_toolbox_node|parameter_bridge|cmd_vel_ign_relay|static_transform_publisher|continuous_teleop|cmd_vel_stamper|ros2 bag record|rviz2|ign gazebo' || true
  )
  sleep 1
}

cleanup_partition_processes

exec ros2 launch semantic_nav_gazebo semantic_cnn_nav_v7_dual_bringup.launch.py \
  gui:="${GUI_VALUE}" \
  start_merger:=true \
  start_slam:=true \
  start_rviz:="${START_RVIZ_VALUE}" \
  spawn_scene_pedestrians:="${SPAWN_SCENE_PEDESTRIANS_VALUE}" \
  pedestrian_use_actors:="${PEDESTRIAN_USE_ACTORS_VALUE}" \
  pedestrian_update_rate:="${PEDESTRIAN_UPDATE_RATE_VALUE}" \
  pedestrian_simulation_factor:="${PEDESTRIAN_SIMULATION_FACTOR_VALUE}" \
  pedestrian_speed:="${PEDESTRIAN_SPEED_VALUE}" \
  pedestrian_count:="${PEDESTRIAN_COUNT_VALUE}" \
  pedestrian_static_obstacle_clearance:="${PEDESTRIAN_STATIC_OBSTACLE_CLEARANCE_VALUE}" \
  pedestrian_seed:="${PEDESTRIAN_SEED_VALUE}" \
  lidar_samples:="${LIDAR_SAMPLES_VALUE}" \
  lidar_update_rate:="${LIDAR_UPDATE_RATE_VALUE}" \
  lidar_samples_01:="${LIDAR_SAMPLES_01_VALUE}" \
  lidar_samples_02:="${LIDAR_SAMPLES_02_VALUE}" \
  lidar_update_rate_01:="${LIDAR_UPDATE_RATE_01_VALUE}" \
  lidar_update_rate_02:="${LIDAR_UPDATE_RATE_02_VALUE}" \
  lidar_range_min:="${LIDAR_RANGE_MIN_VALUE}" \
  lidar_range_max:="${LIDAR_RANGE_MAX_VALUE}" \
  lidar_range_min_01:="${LIDAR_RANGE_MIN_01_VALUE}" \
  lidar_range_min_02:="${LIDAR_RANGE_MIN_02_VALUE}" \
  lidar_range_max_01:="${LIDAR_RANGE_MAX_01_VALUE}" \
  lidar_range_max_02:="${LIDAR_RANGE_MAX_02_VALUE}" \
  lidar_runtime_model_file:="${LIDAR_RUNTIME_MODEL_FILE_VALUE}" \
  slam_scan_topic:=/scan_merged
