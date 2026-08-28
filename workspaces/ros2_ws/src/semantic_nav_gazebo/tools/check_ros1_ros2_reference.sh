#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT, WORLD, YAML
# 可能使用的关键环境变量：BASH_SOURCE, BOLD, FAIL, FAIL_COUNT, GREEN, LD_LIBRARY_PATH, LD_PATH, MAP_COUNT, NAVIGATION_PROJECT_ROOT, PASS, PASS_COUNT, PATH, PATH_VALUE, PROJECT_ROOT, ROS1, ROS1_WS, ROS2, ROS2_PKG, ROS2_WS, SCENARIO_COUNT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.377915240 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_v7_inference_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/v7_dual_laser_scan_merger.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_v7_inference_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/v7_dual_laser_scan_merger.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_v7_inference_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_v7_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_ros1_ros2_reference.sh】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
# Read-only checklist for the ROS1 reference workspace and ROS2 target workspace.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${NAVIGATION_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../../../.." && pwd)}"
ROS1_WS="${ROS1_WS:-${PROJECT_ROOT}/workspaces/ros1_legacy}"
ROS2_WS="${ROS2_WS:-${PROJECT_ROOT}/workspaces/ros2_ws}"
ROS2_PKG="${ROS2_WS}/src/semantic_nav_gazebo"

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() {
    printf "  ${GREEN}PASS${NC} %s\n" "$1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

warn() {
    printf "  ${YELLOW}WARN${NC} %s\n" "$1"
    WARN_COUNT=$((WARN_COUNT + 1))
}

fail() {
    printf "  ${RED}FAIL${NC} %s\n" "$1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

check_path() {
    local path="$1"
    local label="$2"

    if [[ -e "${path}" ]]; then
        pass "${label}: ${path}"
    else
        fail "${label} missing: ${path}"
    fi
}

check_optional_path() {
    local path="$1"
    local label="$2"

    if [[ -e "${path}" ]]; then
        pass "${label}: ${path}"
    else
        warn "${label} not found: ${path}"
    fi
}

count_files() {
    local root="$1"
    local pattern="$2"

    find "${root}" -name "${pattern}" -type f 2>/dev/null | wc -l
}

echo ""
echo -e "${BOLD}================================================${NC}"
echo -e "${BOLD}  ROS1/ROS2 Semantic Navigation Reference Check${NC}"
echo -e "${BOLD}================================================${NC}"
echo ""
echo "ROS1_WS=${ROS1_WS}"
echo "ROS2_WS=${ROS2_WS}"
echo ""

echo -e "${BOLD}[1/6] Workspace layout${NC}"
check_path "${ROS1_WS}/.catkin_workspace" "ROS1 catkin marker"
check_path "${ROS1_WS}/src" "ROS1 source tree"
check_path "${ROS2_WS}/install/setup.bash" "ROS2 install setup"
check_path "${ROS2_PKG}/package.xml" "ROS2 semantic_nav_gazebo package"
echo ""

echo -e "${BOLD}[2/6] ROS1 reference resources${NC}"
check_path "${ROS1_WS}/src/semantic_cnn_nav/semantic_cnn/launch/semantic_cnn_nav_gazebo.launch" "ROS1 semantic CNN launch"
check_path "${ROS1_WS}/src/pedsim_ros_with_gazebo/pedsim_simulator/launch/robot.launch" "ROS1 pedsim robot launch"
check_path "${ROS1_WS}/src/robot_gazebo/param/dwa_local_planner_params.yaml" "ROS1 DWA parameters"
check_path "${ROS1_WS}/src/robot_gazebo/param/move_base_params.yaml" "ROS1 move_base parameters"
check_path "${ROS1_WS}/src/semantic_cnn_nav/cnn_msgs/msg/CNN_data.msg" "ROS1 CNN_data message"
check_optional_path "${ROS1_WS}/src/semantic_cnn_nav/semantic_cnn/src/model/s3_net_model.pth" "ROS1 S3 model"
check_optional_path "${ROS1_WS}/src/semantic_cnn_nav/semantic_cnn/src/model/semantic_cnn_model.pth" "ROS1 semantic CNN model"
echo ""

echo -e "${BOLD}[3/6] ROS2 target resources${NC}"
check_path "${ROS2_PKG}/launch/semantic_cnn_nav_gazebo.launch.py" "ROS2 base Gazebo launch"
check_path "${ROS2_PKG}/launch/semantic_cnn_nav_gazebo.launch_1.py" "ROS2 robust Gazebo launch"
check_path "${ROS2_PKG}/launch/semantic_cnn_nav_v7_dual_bringup.launch.py" "ROS2 v7 dual lidar bringup"
check_path "${ROS2_PKG}/config/v7_dual_laser_scan_merger.yaml" "ROS2 dual lidar merger params"
check_path "${ROS2_PKG}/config/slam_v7_online_async.yaml" "ROS2 slam_toolbox params"
check_path "${ROS2_PKG}/scripts/v7_dual_laser_scan_merger.py" "ROS2 dual lidar merger node"
check_path "${ROS2_PKG}/scripts/scenario_pedestrian_controller.py" "ROS2 pedestrian scenario controller"
check_path "${ROS2_PKG}/scripts/semantic_cnn_v7_inference_node.py" "ROS2 semantic inference node"
check_path "${ROS2_PKG}/scripts/semantic_start_goal_path_node.py" "ROS2 start-goal node"
check_path "${ROS2_PKG}/docs/pedestrian_migration_notes.md" "ROS2 pedestrian migration notes"
check_path "${ROS2_PKG}/docs/ros1_ros2_environment_reference.md" "ROS1/ROS2 comparison doc"
echo ""

echo -e "${BOLD}[4/6] Migrated maps, worlds, and scenarios${NC}"
WORLD_COUNT=$(count_files "${ROS2_PKG}/worlds" "*.world")
MAP_COUNT=$(count_files "${ROS2_PKG}/maps" "*.yaml")
SCENARIO_COUNT=$(count_files "${ROS2_PKG}/scenarios" "*.xml")
[[ "${WORLD_COUNT}" -gt 0 ]] && pass "ROS2 worlds found: ${WORLD_COUNT}" || fail "No ROS2 .world files found"
[[ "${MAP_COUNT}" -gt 0 ]] && pass "ROS2 map YAML files found: ${MAP_COUNT}" || fail "No ROS2 map YAML files found"
[[ "${SCENARIO_COUNT}" -gt 0 ]] && pass "ROS2 scenario XML files found: ${SCENARIO_COUNT}" || warn "No ROS2 scenario XML files found"
check_path "${ROS2_PKG}/scenarios/lobby/eng_hall_15.xml" "Default ROS2 lobby scenario"
echo ""

echo -e "${BOLD}[5/6] Build and command availability${NC}"
if command -v colcon >/dev/null 2>&1; then
    pass "colcon command available: $(command -v colcon)"
else
    warn "colcon command not found; source ROS2 or install colcon before building"
fi

if command -v ros2 >/dev/null 2>&1; then
    pass "ros2 command available: $(command -v ros2)"
else
    warn "ros2 command not found; source /opt/ros/humble/setup.bash"
fi

check_optional_path "${ROS2_WS}/build/semantic_nav_gazebo" "ROS2 package build directory"
check_optional_path "${ROS2_WS}/install/semantic_nav_gazebo" "ROS2 package install directory"
echo ""

echo -e "${BOLD}[6/6] Environment risk hints${NC}"
LD_PATH="${LD_LIBRARY_PATH:-}"
PATH_VALUE="${PATH:-}"

if [[ "${LD_PATH}" == *"miniconda"* ]]; then
    warn "LD_LIBRARY_PATH contains a Miniconda library directory; this can shadow ROS/Gazebo system libraries"
else
    pass "LD_LIBRARY_PATH does not contain a Miniconda library directory"
fi

if [[ "${PATH_VALUE}" == *"miniconda"* ]]; then
    warn "PATH contains a Miniconda directory; keep this in mind for ROS/Gazebo debugging"
else
    pass "PATH does not contain a Miniconda directory"
fi
echo ""

echo -e "${BOLD}================================================${NC}"
echo -e "${BOLD}  Summary${NC}"
echo -e "${BOLD}================================================${NC}"
printf "  PASS: %s\n" "${PASS_COUNT}"
printf "  WARN: %s\n" "${WARN_COUNT}"
printf "  FAIL: %s\n" "${FAIL_COUNT}"
echo ""

if [[ "${FAIL_COUNT}" -gt 0 ]]; then
    exit 1
fi

exit 0
