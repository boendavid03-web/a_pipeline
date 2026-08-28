#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--kill-after, --signal
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, WORLD
# 可能使用的关键环境变量：BASH_SOURCE, DRLVO_PEDESTRIAN_SEED, DRLVO_PPO_BATCH_SIZE, DRLVO_PPO_FREEZE_FEATURE_EXTRACTOR, DRLVO_PPO_GUI, DRLVO_PPO_LEARNING_RATE, DRLVO_PPO_MAX_EPISODE_STEPS, DRLVO_PPO_OUTPUT_ROOT, DRLVO_PPO_ROLLOUT_STEPS, DRLVO_PPO_RVIZ, DRLVO_PPO_SEED, DRLVO_PPO_TOTAL_TIMESTEPS, DRLVO_PPO_UPDATE_EPOCHS, DRLVO_PPO_WALL_TIMEOUT_SEC, PIPESTATUS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 12:44:32.881971566 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/activate.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（ros2 launch 启动该场景）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/environment/activate.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
run_root="$project_root/runs/20260717_042135_v7_dual"
stamp="$(date +%Y%m%d_%H%M%S_%N)"
output_root="${DRLVO_PPO_OUTPUT_ROOT:-$run_root/training/drl_vo/online_ppo_predicted}"
run_dir="$output_root/$stamp"
training_dir="$run_dir/training"
total_timesteps="${DRLVO_PPO_TOTAL_TIMESTEPS:-100000}"
wall_timeout_sec="${DRLVO_PPO_WALL_TIMEOUT_SEC:-43200}"
pedestrian_seed="${DRLVO_PEDESTRIAN_SEED:-107}"

mkdir -p "$output_root"
mkdir "$run_dir"

source "$project_root/environment/activate.sh"
set +u
source "$project_root/workspaces/ros2_ws/install/setup.bash"
set -u

set +e
timeout \
  --signal=INT \
  --kill-after=20s \
  "${wall_timeout_sec}s" \
  ros2 launch semantic_nav_gazebo \
  drl_vo_fixed_dual_start_goal_demo.launch.py \
  policy_mode:=original \
  drl_vo_model:="$project_root/github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip" \
  pedestrian_source:=predicted \
  perception_model:="$run_root/training/dual_lidar_pedestrian_bev/20260731_opt_velw100_h12_c24_v1/checkpoints/epoch_014.pt" \
  require_pedestrian_truth:=false \
  oracle_pedestrian_velocity:=false \
  publish_policy_actions:=false \
  start_online_ppo_training:=true \
  ppo_output_dir:="$training_dir" \
  ppo_total_timesteps:="$total_timesteps" \
  ppo_rollout_steps:="${DRLVO_PPO_ROLLOUT_STEPS:-256}" \
  ppo_max_episode_steps:="${DRLVO_PPO_MAX_EPISODE_STEPS:-512}" \
  ppo_update_epochs:="${DRLVO_PPO_UPDATE_EPOCHS:-4}" \
  ppo_batch_size:="${DRLVO_PPO_BATCH_SIZE:-64}" \
  ppo_learning_rate:="${DRLVO_PPO_LEARNING_RATE:-5e-5}" \
  ppo_seed:="${DRLVO_PPO_SEED:-1337}" \
  ppo_freeze_feature_extractor:="${DRLVO_PPO_FREEZE_FEATURE_EXTRACTOR:-true}" \
  perception_metrics_path:="$run_dir/perception_metrics.jsonl" \
  world:=gazebo_eng_lobby.world \
  robot_x:=2.0 \
  robot_y:=2.0 \
  robot_yaw:=0.0 \
  goal_x:=16.0 \
  goal_y:=16.0 \
  pedestrian_seed:="$pedestrian_seed" \
  pedestrian_count:=15 \
  pedestrian_speed:=1.0 \
  pedestrian_use_actors:=false \
  enable_goal_picker:=false \
  auto_set_initial_goal:=true \
  cmd_vel_angular_z_scale:=1.0 \
  max_linear:=0.5 \
  max_angular:=2.0 \
  gui:="${DRLVO_PPO_GUI:-false}" \
  start_rviz:="${DRLVO_PPO_RVIZ:-false}" \
  2>&1 | tee "$run_dir/launch.log"
launch_status=${PIPESTATUS[0]}
set -e

if [[ "$launch_status" -ne 0 && "$launch_status" -ne 124 ]]; then
  exit "$launch_status"
fi
echo "outputs: $run_dir"
