#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--once
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：AVAILABLE_TOPICS, BOLD, ERROR, EXISTS, GREEN, GZ_EXIT, GZ_TIMEOUT_OUTPUT, GZ_TOOL, LINE, MISSING, PASS, PATH, SHORT_NAME, SUMMARY_LINES, TF_CHECK_OK, TIMEOUT_OUTPUT, TOPIC, TOPICS_TO_CHECK, WARN, YELLOW
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/diagnose_ros_runtime.sh
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
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/diagnose_ros_runtime.sh
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜diagnose_ros_runtime.sh】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
#===============================================================================
# diagnose_ros_runtime.sh
#
# A read-only diagnostic script for a running ROS 2 Humble + Gazebo Sim
# environment.  It inspects nodes, topics, TF transforms, and prints a concise
# summary at the end.
#
# This script does **not** modify any system or project state.
# It does **not** source ROS 2 setup files automatically.
#===============================================================================

#-------------------------------------------------------------------------------
# Colour helpers (optional – just makes the output friendlier)
#-------------------------------------------------------------------------------
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'  # No Colour

#-------------------------------------------------------------------------------
# Summary accumulator
#-------------------------------------------------------------------------------
SUMMARY_LINES=()

# Simple helper to add a line to the summary
add_summary() {
    local label="$1"
    local topic="$2"
    local msg="$3"
    SUMMARY_LINES+=("  ${label}  ${topic}  ${msg}")
}

#-------------------------------------------------------------------------------
# Step 0 – banner
#-------------------------------------------------------------------------------
echo ""
echo "${BOLD}================================================${NC}"
echo "${BOLD}  ROS 2 Runtime Diagnostic${NC}"
echo "${BOLD}================================================${NC}"
echo ""

#-------------------------------------------------------------------------------
# Step 1 – check that 'ros2' is available
#-------------------------------------------------------------------------------
echo "${BOLD}[1/7] Checking for ros2 command ...${NC}"

if ! command -v ros2 &>/dev/null; then
    echo ""
    echo "  ${RED}ERROR:${NC} the 'ros2' command was not found in your PATH."
    echo "  Make sure ROS 2 Humble has been sourced, for example:"
    echo "      source /opt/ros/humble/setup.bash"
    echo ""
    exit 1
fi

echo "  ${GREEN}OK${NC}  ros2 is available at $(command -v ros2)"
echo ""

#-------------------------------------------------------------------------------
# Step 2 – list ROS 2 nodes
#-------------------------------------------------------------------------------
echo "${BOLD}[2/7] ROS 2 node list ...${NC}"
echo ""

ros2 node list | sort
echo ""

#-------------------------------------------------------------------------------
# Step 3 – inspect key topics
#-------------------------------------------------------------------------------
echo "${BOLD}[3/7] Inspecting key topics ...${NC}"
echo ""

TOPICS_TO_CHECK=(
    "/cmd_vel"
    "/odom"
    "/tf"
    "/tf_static"
)

# Gather the list of topics that are currently available
AVAILABLE_TOPICS=$(ros2 topic list 2>/dev/null)

for TOPIC in "${TOPICS_TO_CHECK[@]}"; do

    SHORT_NAME="${TOPIC#/}"  # e.g. odom instead of /odom

    echo "--------------------------------------------------"
    echo "  Topic: ${BOLD}${TOPIC}${NC}"

    if echo "${AVAILABLE_TOPICS}" | grep -qFx "${TOPIC}"; then
        echo "  Status: ${GREEN}EXISTS${NC}"
        echo ""

        # Verbose info about the topic
        echo "  --- Verbose info ---"
        ros2 topic info "${TOPIC}" -v 2>&1 || true
        echo ""

        # For /odom specifically, try to read one message with a timeout
        if [[ "${TOPIC}" == "/odom" ]]; then
            echo "  --- One /odom message (timeout 4s) ---"
            timeout 4 ros2 topic echo "${TOPIC}" --once 2>&1 || true
            echo ""
        fi

        add_summary "${GREEN}PASS${NC}" "${TOPIC}" ""

    else
        echo "  Status: ${RED}MISSING${NC}"
        echo ""
        add_summary "${RED}MISSING${NC}" "${TOPIC}" ""
    fi

done

#-------------------------------------------------------------------------------
# Step 4 – TF transform  odom -> base_link
#-------------------------------------------------------------------------------
echo "${BOLD}[4/7] TF transform  odom -> base_link ...${NC}"
echo ""

TF_CHECK_OK=false
echo "  Attempting 'ros2 run tf2_ros tf2_echo odom base_link' (timeout 4s) ..."
echo ""

TIMEOUT_OUTPUT=$(timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1) && TF_CHECK_OK=true || true

echo "${TIMEOUT_OUTPUT}"
echo ""

# Heuristic: if ros2 run exited okay *and* output contains a translation line,
# we consider the transform present.
if ${TF_CHECK_OK} && echo "${TIMEOUT_OUTPUT}" | grep -qi "Translation"; then
    add_summary "${GREEN}PASS${NC}" "odom -> base_link" ""
elif echo "${TIMEOUT_OUTPUT}" | grep -qi "Could not find"; then
    add_summary "${RED}MISSING${NC}" "odom -> base_link" "(could not find transform)"
elif [[ "${TIMEOUT_OUTPUT}" == "" ]]; then
    add_summary "${YELLOW}WARN${NC}" "odom -> base_link" "(timed out – no data within 4 s)"
else
    add_summary "${YELLOW}WARN${NC}" "odom -> base_link" "(unexpected output – see above)"
fi

#-------------------------------------------------------------------------------
# Step 5 – TF transform  base_link -> sensor_mount
#-------------------------------------------------------------------------------
echo "${BOLD}[5/7] TF transform  base_link -> sensor_mount ...${NC}"
echo ""

TF_CHECK_OK=false
echo "  Attempting 'ros2 run tf2_ros tf2_echo base_link sensor_mount' (timeout 4s) ..."
echo ""

TIMEOUT_OUTPUT=$(timeout 4 ros2 run tf2_ros tf2_echo base_link sensor_mount 2>&1) && TF_CHECK_OK=true || true

echo "${TIMEOUT_OUTPUT}"
echo ""

if ${TF_CHECK_OK} && echo "${TIMEOUT_OUTPUT}" | grep -qi "Translation"; then
    add_summary "${GREEN}PASS${NC}" "base_link -> sensor_mount" ""
elif echo "${TIMEOUT_OUTPUT}" | grep -qi "Could not find"; then
    add_summary "${RED}MISSING${NC}" "base_link -> sensor_mount" "(could not find transform)"
elif [[ "${TIMEOUT_OUTPUT}" == "" ]]; then
    add_summary "${YELLOW}WARN${NC}" "base_link -> sensor_mount" "(timed out – no data within 4 s)"
else
    add_summary "${YELLOW}WARN${NC}" "base_link -> sensor_mount" "(unexpected output – see above)"
fi

#-------------------------------------------------------------------------------
# Step 6 – TF transform  sensor_mount -> lidar_link
#-------------------------------------------------------------------------------
echo "${BOLD}[6/7] TF transform  sensor_mount -> lidar_link ...${NC}"
echo ""

TF_CHECK_OK=false
echo "  Attempting 'ros2 run tf2_ros tf2_echo sensor_mount lidar_link' (timeout 4s) ..."
echo ""

TIMEOUT_OUTPUT=$(timeout 4 ros2 run tf2_ros tf2_echo sensor_mount lidar_link 2>&1) && TF_CHECK_OK=true || true

echo "${TIMEOUT_OUTPUT}"
echo ""

if ${TF_CHECK_OK} && echo "${TIMEOUT_OUTPUT}" | grep -qi "Translation"; then
    add_summary "${GREEN}PASS${NC}" "sensor_mount -> lidar_link" ""
elif echo "${TIMEOUT_OUTPUT}" | grep -qi "Could not find"; then
    add_summary "${RED}MISSING${NC}" "sensor_mount -> lidar_link" "(could not find transform)"
elif [[ "${TIMEOUT_OUTPUT}" == "" ]]; then
    add_summary "${YELLOW}WARN${NC}" "sensor_mount -> lidar_link" "(timed out – no data within 4 s)"
else
    add_summary "${YELLOW}WARN${NC}" "sensor_mount -> lidar_link" "(unexpected output – see above)"
fi

#-------------------------------------------------------------------------------
# Step 7 – Gazebo Transport /tf
#-------------------------------------------------------------------------------
echo "${BOLD}[7/7] Gazebo Transport /tf ...${NC}"
echo ""

GZ_TOOL=""
if command -v ign &>/dev/null; then
    GZ_TOOL="ign"
elif command -v gz &>/dev/null; then
    GZ_TOOL="gz"
fi

if [[ -z "${GZ_TOOL}" ]]; then
    echo "  ${YELLOW}WARN:${NC} Neither 'ign' nor 'gz' command found in PATH."
    echo "  Skipping Gazebo Transport /tf check."
    echo ""
    add_summary "${YELLOW}WARN${NC}" "Gazebo Transport /tf" "(no ign/gz tool)"
else
    echo "  Using '${GZ_TOOL}' for Gazebo Transport check..."
    echo "  Running 'timeout 4 ${GZ_TOOL} topic -i -t /tf' ..."
    echo ""

    GZ_TIMEOUT_OUTPUT=$(timeout 4 "${GZ_TOOL}" topic -i -t /tf 2>&1)
    GZ_EXIT=$?
    echo "${GZ_TIMEOUT_OUTPUT}"
    echo ""

    if echo "${GZ_TIMEOUT_OUTPUT}" | grep -qi "No publishers"; then
        add_summary "${RED}MISSING${NC}" "Gazebo Transport /tf" "(no publishers on /tf)"
    elif echo "${GZ_TIMEOUT_OUTPUT}" | grep -qi "not found"; then
        add_summary "${RED}MISSING${NC}" "Gazebo Transport /tf" "(topic not found)"
    elif [[ "${GZ_TIMEOUT_OUTPUT}" == "" ]]; then
        add_summary "${YELLOW}WARN${NC}" "Gazebo Transport /tf" "(timed out or empty output)"
    elif [[ "${GZ_EXIT}" -eq 124 ]]; then
        add_summary "${YELLOW}WARN${NC}" "Gazebo Transport /tf" "(timed out after 4 s)"
    else
        add_summary "${GREEN}PASS${NC}" "Gazebo Transport /tf" ""
    fi
fi

#-------------------------------------------------------------------------------
# Final – concise summary
#-------------------------------------------------------------------------------
echo ""
echo "${BOLD}================================================${NC}"
echo "${BOLD}  Diagnostic Summary${NC}"
echo "${BOLD}================================================${NC}"
echo ""

for LINE in "${SUMMARY_LINES[@]}"; do
    echo -e "${LINE}"
done

echo ""

#-------------------------------------------------------------------------------
# Explain legends
#-------------------------------------------------------------------------------
echo -e "Legend:"
echo -e "  ${GREEN}PASS${NC}    – check succeeded"
echo -e "  ${YELLOW}WARN${NC}    – check hit a recoverable issue (e.g. timeout)"
echo -e "  ${RED}MISSING${NC}  – the expected resource was not found"
echo ""

exit 0
