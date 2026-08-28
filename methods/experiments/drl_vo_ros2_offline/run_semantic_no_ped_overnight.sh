#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --drop-pedestrian-velocity, --epochs, --feature-batch-size, --learning-rate, --model, --output-root, --patience, --replay-dir, --seed, --semantic-num-classes, --semantic-person-class, --use-semantics
# 代码中检测到的 ROS 2 话题/路径字符串：/home/user/navigation_project/a_pipeline
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：LOG_ROOT, OUTPUT_ROOT, PRETRAINED_MODEL, PROJECT_ROOT, PYTHON, REPLAY_DIR, RUN_STAMP, TASK_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_semantic_no_ped_overnight.sh
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:34:22.792610055 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_semantic_no_ped_overnight.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
TASK_ROOT="${PROJECT_ROOT}/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1"
PYTHON="${PROJECT_ROOT}/.venvs/train/bin/python"
REPLAY_DIR="${TASK_ROOT}/training/drl_vo/replay/20260727_110005"
PRETRAINED_MODEL="${PROJECT_ROOT}/github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip"
OUTPUT_ROOT="${TASK_ROOT}/training/drl_vo/semantic_no_ped_overnight"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${OUTPUT_ROOT}/launcher_logs/${RUN_STAMP}"

mkdir -p "${LOG_ROOT}"
cd "${PROJECT_ROOT}/methods/experiments/drl_vo_ros2_offline"

for seed in 1337 2026 3407 47 71; do
    log_path="${LOG_ROOT}/seed_${seed}.log"
    echo "Starting seed ${seed}; log: ${log_path}"
    "${PYTHON}" train_behavior_cloning.py \
        --replay-dir "${REPLAY_DIR}" \
        --model "${PRETRAINED_MODEL}" \
        --output-root "${OUTPUT_ROOT}" \
        --use-semantics \
        --drop-pedestrian-velocity \
        --semantic-num-classes 7 \
        --semantic-person-class 6 \
        --epochs 1200 \
        --batch-size 64 \
        --feature-batch-size 16 \
        --learning-rate 1e-4 \
        --patience 200 \
        --seed "${seed}" \
        >"${log_path}" 2>&1
done

echo "All runs completed. Results: ${OUTPUT_ROOT}"
echo "Launcher logs: ${LOG_ROOT}"
