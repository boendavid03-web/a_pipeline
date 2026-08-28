#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /map_server, /odom, /semantic_cnn/actual_trajectory, /world/default/set_pose
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PNG, PT, WORLD, YAML
# 可能使用的关键环境变量：GIO_, GTK_PATH, JSONL, NAVIGATION_PROJECT_ROOT, RUN_ROOT, SNAP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo drl_vo_fixed_dual_start_goal_demo.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:38:17.401911866 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.850391663 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_gazebo_ped_map_comparison.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_gazebo_ped_map_comparison.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_online_ppo_training_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_gazebo_ped_map_comparison.sh; /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_online_ppo_predicted_training.sh; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜drl_vo_fixed_dual_start_goal_demo.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Fixed start/goal Gazebo demo for original or behavior-cloned DRL-VO."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def clean_rviz_environment():
    """Avoid VS Code snap GTK/GIO paths loading core20 libraries into RViz."""
    environment = os.environ.copy()
    for name in tuple(environment):
        if name == "GTK_PATH" or name.startswith("GIO_") or name.startswith("SNAP"):
            environment.pop(name)
    return environment


def as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def select_policy_model(context):
    mode = LaunchConfiguration("policy_mode").perform(context).strip()
    if mode not in ("original", "base", "semantic", "semantic_no_ped"):
        raise ValueError(
            "policy_mode must be 'original', 'base', 'semantic', or "
            f"'semantic_no_ped', got {mode!r}"
        )
    override = LaunchConfiguration("drl_vo_model").perform(context).strip()
    if override:
        selected = override
    elif mode == "original":
        selected = LaunchConfiguration("original_drl_vo_model").perform(context)
    elif mode == "base":
        selected = LaunchConfiguration("base_drl_vo_model").perform(context)
    elif mode == "semantic":
        selected = LaunchConfiguration("semantic_drl_vo_model").perform(context)
    else:
        selected = LaunchConfiguration("semantic_no_ped_drl_vo_model").perform(
            context
        )
    if as_bool(
        LaunchConfiguration("start_online_ppo_training").perform(context)
    ):
        if as_bool(LaunchConfiguration("start_auto_capture").perform(context)):
            raise ValueError(
                "start_online_ppo_training and start_auto_capture are mutually exclusive"
            )
        pedestrian_source = LaunchConfiguration("pedestrian_source").perform(
            context
        )
        require_truth = as_bool(
            LaunchConfiguration("require_pedestrian_truth").perform(context)
        )
        publish_actions = as_bool(
            LaunchConfiguration("publish_policy_actions").perform(context)
        )
        if mode not in ("original", "base"):
            raise ValueError("online PPO supports original/base DRL-VO only")
        if pedestrian_source != "predicted" or require_truth:
            raise ValueError(
                "online PPO requires pedestrian_source=predicted and "
                "require_pedestrian_truth=false"
            )
        if publish_actions:
            raise ValueError(
                "online PPO requires publish_policy_actions=false so only "
                "the trainer controls /cmd_vel"
            )
    return [SetLaunchConfiguration("selected_drl_vo_model", selected)]


def generate_launch_description():
    project_root = Path(
        os.environ.get(
            "NAVIGATION_PROJECT_ROOT",
            Path(__file__).resolve().parents[5],
        )
    )
    run_root = Path(
        os.environ.get(
            "RUN_ROOT",
            project_root / "runs" / "20260717_042135_v7_dual",
        )
    )
    task_root = (
        run_root
        / "datasets"
        / "20260727_three_bag_online_seed_split_v1"
    )
    package_share = get_package_share_directory("semantic_nav_gazebo")
    bringup_launch = str(
        Path(package_share)
        / "launch"
        / "semantic_cnn_nav_v7_dual_bringup.launch.py"
    )
    default_rviz_config = str(
        Path(package_share)
        / "rviz"
        / "semantic_cnn_fixed_dual_debug.rviz"
    )
    train_python = str(project_root / ".venvs" / "train" / "bin" / "python")
    default_original_model = str(
        project_root
        / "github_src"
        / "drl_vo_nav-drl_vo"
        / "drl_vo"
        / "src"
        / "model"
        / "drl_vo.zip"
    )
    default_perception_model = str(
        run_root
        / "training"
        / "dual_lidar_pedestrian_bev"
        / "20260731_opt_velw100_h12_c24_v1"
        / "checkpoints"
        / "epoch_014.pt"
    )
    default_base_model = str(
        task_root
        / "training"
        / "drl_vo"
        / "base_bc"
        / "20260727_114455"
        / "checkpoints"
        / "best.pt"
    )
    default_semantic_model = str(
        task_root
        / "training"
        / "drl_vo"
        / "semantic_bc"
        / "20260727_115227"
        / "checkpoints"
        / "best.pt"
    )
    default_semantic_no_ped_model = str(
        task_root
        / "training"
        / "drl_vo"
        / "semantic_no_ped_bc"
        / "20260730_121840"
        / "checkpoints"
        / "best.pt"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_mode",
                default_value="base",
                description=(
                    "'original' (official drl_vo.zip), 'base' (163 weights), "
                    "'semantic' (172 weights), or "
                    "'semantic_no_ped' (172 weights, vx/vy fixed at zero)."
                ),
            ),
            DeclareLaunchArgument(
                "original_drl_vo_model",
                default_value=default_original_model,
            ),
            DeclareLaunchArgument(
                "base_drl_vo_model",
                default_value=default_base_model,
            ),
            DeclareLaunchArgument(
                "semantic_drl_vo_model",
                default_value=default_semantic_model,
            ),
            DeclareLaunchArgument(
                "semantic_no_ped_drl_vo_model",
                default_value=default_semantic_no_ped_model,
            ),
            DeclareLaunchArgument(
                "drl_vo_model",
                default_value="",
                description=(
                    "Optional explicit checkpoint override. Empty selects the "
                    "matching policy-mode checkpoint above."
                ),
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "drl_vo_python",
                default_value=train_python,
                description="Python interpreter containing torch and ROS bindings",
            ),
            DeclareLaunchArgument(
                "pedestrian_source",
                default_value="oracle",
                description="'oracle', 'predicted', or 'zero'.",
            ),
            DeclareLaunchArgument(
                "perception_model",
                default_value=default_perception_model,
            ),
            DeclareLaunchArgument(
                "perception_confidence_threshold",
                default_value="0.4",
            ),
            DeclareLaunchArgument("perception_topk", default_value="30"),
            DeclareLaunchArgument(
                "perception_nms_radius_m",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "coasting_max_time_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "max_track_age_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "include_tentative_tracks",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "perception_metrics_path",
                default_value="",
                description=(
                    "Optional new JSONL path for predicted-perception latency "
                    "and tracker statistics; existing files are never overwritten."
                ),
            ),
            DeclareLaunchArgument(
                "publish_policy_actions",
                default_value="true",
                description=(
                    "false turns the inference node into a truth-free training "
                    "state provider without publishing policy commands."
                ),
            ),
            DeclareLaunchArgument(
                "start_online_ppo_training",
                default_value="false",
            ),
            DeclareLaunchArgument("ppo_output_dir", default_value=""),
            DeclareLaunchArgument("ppo_total_timesteps", default_value="100000"),
            DeclareLaunchArgument("ppo_rollout_steps", default_value="256"),
            DeclareLaunchArgument("ppo_max_episode_steps", default_value="512"),
            DeclareLaunchArgument("ppo_update_epochs", default_value="4"),
            DeclareLaunchArgument("ppo_batch_size", default_value="64"),
            DeclareLaunchArgument("ppo_learning_rate", default_value="5e-5"),
            DeclareLaunchArgument("ppo_seed", default_value="1337"),
            DeclareLaunchArgument(
                "ppo_freeze_feature_extractor", default_value="true"
            ),
            DeclareLaunchArgument(
                "robot_reset_service", default_value="/world/default/set_pose"
            ),
            DeclareLaunchArgument(
                "robot_entity_name",
                default_value="mecanum730_xms5_v7_teacher_dual_scan",
            ),
            DeclareLaunchArgument("start_auto_capture", default_value="false"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "start_simulator",
                default_value="true",
                description=(
                    "Start Gazebo and its robot/pedestrians. Set false when an "
                    "external simulator such as Isaac Sim already publishes the "
                    "dual scans, odometry, clock, TF, and pedestrian truth."
                ),
            ),
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("robot_x", default_value="2.0"),
            DeclareLaunchArgument("robot_y", default_value="2.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="16.0"),
            DeclareLaunchArgument("goal_y", default_value="16.0"),
            DeclareLaunchArgument("enable_goal_picker", default_value="true"),
            DeclareLaunchArgument(
                "auto_set_initial_goal",
                default_value="false",
                description=(
                    "false waits for the map picker; true uses goal_x/goal_y."
                ),
            ),
            DeclareLaunchArgument(
                "goal_picker_arrival_dwell_sec",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "spawn_scene_pedestrians",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "scene_file",
                default_value="scenarios/lobby/eng_hall_15.xml",
            ),
            DeclareLaunchArgument("pedestrian_count", default_value="15"),
            DeclareLaunchArgument("pedestrian_speed", default_value="1.0"),
            DeclareLaunchArgument("pedestrian_seed", default_value="7"),
            DeclareLaunchArgument("pedestrian_use_actors", default_value="false"),
            DeclareLaunchArgument(
                "require_pedestrian_truth",
                default_value="true",
                description=(
                    "true is required by oracle/semantic modes; predicted and "
                    "zero original/base modes must set false."
                ),
            ),
            DeclareLaunchArgument(
                "oracle_pedestrian_velocity",
                default_value="true",
                description=(
                    "Explicitly acknowledges the trained policy's Gazebo "
                    "ground-truth pedestrian velocity input. semantic_no_ped "
                    "ignores this value and fixes vx/vy at zero."
                ),
            ),
            DeclareLaunchArgument(
                "oracle_person_semantics",
                default_value="true",
                description=(
                    "Semantic mode requires ground-truth lower-leg Person=6 "
                    "labels (offset 0.07 m, match radius 0.105 m)."
                ),
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=str(
                    run_root / "maps" / "semantic_label" / "map.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "semantic_label",
                default_value=str(
                    run_root / "maps" / "semantic_label" / "label.png"
                ),
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_angular_z_scale", default_value="1.5"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "show_actual_trajectory",
                default_value="false",
                description=(
                    "Publish the cumulative odometry trail for RViz. Disabled "
                    "by default so a new manual-goal demo starts without a blue "
                    "history line."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
            ),
            DeclareLaunchArgument("max_linear", default_value="0.3"),
            DeclareLaunchArgument("max_angular", default_value="1.5"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.35"),
            DeclareLaunchArgument("front_stop_distance", default_value="0.5"),
            DeclareLaunchArgument("stop_on_empty_front", default_value="true"),
            DeclareLaunchArgument(
                "front_stop_angular_deadband",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "front_stop_min_angular",
                default_value="0.35",
            ),
            DeclareLaunchArgument(
                "enable_actuation_deadlock_detection",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_window_sec",
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_min_command_ratio",
                default_value="0.8",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_goal_x_threshold",
                default_value="-0.05",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_max_linear_command",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_min_angular_command",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_max_displacement_m",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "actuation_deadlock_max_yaw_progress_rad",
                default_value="0.03",
            ),
            DeclareLaunchArgument("scan_timeout", default_value="0.5"),
            DeclareLaunchArgument("odom_timeout", default_value="0.3"),
            DeclareLaunchArgument("subgoal_timeout", default_value="0.3"),
            DeclareLaunchArgument("final_goal_timeout", default_value="0.5"),
            DeclareLaunchArgument(
                "pedestrian_truth_timeout",
                default_value="0.15",
            ),
            DeclareLaunchArgument("lookahead", default_value="1.0"),
            DeclareLaunchArgument("inflate_radius", default_value="0.5"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("record_trace", default_value="false"),
            DeclareLaunchArgument("trace_path", default_value=""),
            DeclareLaunchArgument("trace_timeout_sec", default_value="360.0"),
            DeclareLaunchArgument("evaluate_episode", default_value="false"),
            DeclareLaunchArgument("evaluation_output_dir", default_value=""),
            DeclareLaunchArgument("evaluation_timeout_sec", default_value="360.0"),
            DeclareLaunchArgument(
                "evaluation_multi_episode",
                default_value="false",
                description=(
                    "Record every accepted goal as an independent episode_XXXX "
                    "subdirectory and keep the evaluator alive for later goals."
                ),
            ),
            DeclareLaunchArgument(
                "inference_metrics_topic",
                default_value="/navigation_evaluation/inference_metrics",
            ),
            DeclareLaunchArgument("actuation_decision_topic", default_value="/drl_vo/actuation_decision"),
            DeclareLaunchArgument("simulator_actuation_topic", default_value="/isaac/actuation_state"),
            DeclareLaunchArgument("episode_reset_topic", default_value="/drl_vo/episode_reset"),
            DeclareLaunchArgument("alignment_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("alignment_freshness_sec", default_value="0.20"),
            DeclareLaunchArgument("alignment_max_delay_sec", default_value="0.50"),
            DeclareLaunchArgument("maximum_actual_linear_speed_mps", default_value="5.0"),
            DeclareLaunchArgument("maximum_actual_angular_speed_radps", default_value="10.0"),
            DeclareLaunchArgument("personal_space_radius", default_value="0.8"),
            DeclareLaunchArgument("robot_radius", default_value="0.34"),
            DeclareLaunchArgument("pedestrian_radius", default_value="0.125"),
            DeclareLaunchArgument("stopped_speed_threshold", default_value="0.02"),
            DeclareLaunchArgument("experiment_scene_id", default_value=""),
            OpaqueFunction(function=select_policy_model),
            SetLaunchConfiguration(
                "drl_vo_start_rviz",
                LaunchConfiguration("start_rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                condition=IfCondition(LaunchConfiguration("start_simulator")),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world": LaunchConfiguration("world"),
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "start_rviz": "false",
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "spawn_scene_pedestrians": LaunchConfiguration(
                        "spawn_scene_pedestrians"
                    ),
                    "scene_file": LaunchConfiguration("scene_file"),
                    "pedestrian_count": LaunchConfiguration("pedestrian_count"),
                    "pedestrian_speed": LaunchConfiguration("pedestrian_speed"),
                    "pedestrian_seed": LaunchConfiguration("pedestrian_seed"),
                    "pedestrian_use_actors": LaunchConfiguration(
                        "pedestrian_use_actors"
                    ),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "cmd_vel_angular_z_scale": LaunchConfiguration(
                        "cmd_vel_angular_z_scale"
                    ),
                    "start_merger": "false",
                    "start_slam": "false",
                    "lidar_samples_01": "2000",
                    "lidar_samples_02": "2000",
                    "lidar_update_rate_01": "15",
                    "lidar_update_rate_02": "15",
                    "lidar_range_min_01": "0.1",
                    "lidar_range_min_02": "0.1",
                    "lidar_range_max_01": "8.0",
                    "lidar_range_max_02": "8.0",
                }.items(),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("drl_vo_start_rviz")
                        ),
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_drl_vo_fixed_dual",
                        output="screen",
                        arguments=[
                            "-d",
                            LaunchConfiguration("rviz_config"),
                        ],
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                )
                            }
                        ],
                        env=clean_rviz_environment(),
                    )
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                package="semantic_nav_gazebo",
                executable="demo_velocity_display_node.py",
                name="demo_velocity_display",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "yaml_filename": LaunchConfiguration("map_yaml"),
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "lifecycle",
                            "set",
                            "/map_server",
                            "configure",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "lifecycle",
                            "set",
                            "/map_server",
                            "activate",
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                condition=IfCondition(PythonExpression([
                    "'",
                    LaunchConfiguration("start_online_ppo_training"),
                    "' == 'true' or '",
                    LaunchConfiguration("start_auto_capture"),
                    "' == 'true'",
                ])),
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="drl_vo_robot_reset_service_bridge",
                output="screen",
                arguments=[
                    [
                        LaunchConfiguration("robot_reset_service"),
                        "@ros_gz_interfaces/srv/SetEntityPose",
                    ]
                ],
            ),
            Node(
                package="semantic_nav_gazebo",
                executable="odom_path_node.py",
                name="drl_vo_actual_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "odom_topic": "/odom",
                        # The existing RViz config displays this topic.
                        "path_topic": "/semantic_cnn/actual_trajectory",
                        "enabled": ParameterValue(
                            LaunchConfiguration("show_actual_trajectory"),
                            value_type=bool,
                        ),
                        "start_on_goal": True,
                        "clear_on_goal": True,
                    }
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("record_trace")),
                package="semantic_nav_gazebo",
                executable="closed_loop_demo_recorder.py",
                name="drl_vo_closed_loop_demo_recorder",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/odom",
                        "output_csv": LaunchConfiguration("trace_path"),
                        "goal_x": ParameterValue(
                            LaunchConfiguration("goal_x"),
                            value_type=float,
                        ),
                        "goal_y": ParameterValue(
                            LaunchConfiguration("goal_y"),
                            value_type=float,
                        ),
                        "follow_accepted_goal": ParameterValue(
                            LaunchConfiguration("enable_goal_picker"),
                            value_type=bool,
                        ),
                        "goal_tolerance": ParameterValue(
                            LaunchConfiguration("goal_tolerance"),
                            value_type=float,
                        ),
                        "timeout_sec": ParameterValue(
                            LaunchConfiguration("trace_timeout_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("evaluate_episode")),
                package="semantic_nav_gazebo",
                executable="navigation_episode_evaluator.py",
                name="navigation_episode_evaluator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "evaluation_output_dir": LaunchConfiguration(
                            "evaluation_output_dir"
                        ),
                        "evaluation_timeout_sec": ParameterValue(
                            LaunchConfiguration("evaluation_timeout_sec"),
                            value_type=float,
                        ),
                        "evaluation_multi_episode": ParameterValue(
                            LaunchConfiguration("evaluation_multi_episode"),
                            value_type=bool,
                        ),
                        "inference_metrics_topic": LaunchConfiguration(
                            "inference_metrics_topic"
                        ),
                        "actuation_decision_topic": LaunchConfiguration("actuation_decision_topic"),
                        "simulator_actuation_topic": LaunchConfiguration("simulator_actuation_topic"),
                        "episode_reset_topic": LaunchConfiguration("episode_reset_topic"),
                        "alignment_rate_hz": ParameterValue(
                            LaunchConfiguration("alignment_rate_hz"), value_type=float
                        ),
                        "alignment_freshness_sec": ParameterValue(
                            LaunchConfiguration("alignment_freshness_sec"), value_type=float
                        ),
                        "alignment_max_delay_sec": ParameterValue(
                            LaunchConfiguration("alignment_max_delay_sec"), value_type=float
                        ),
                        "maximum_actual_linear_speed_mps": ParameterValue(
                            LaunchConfiguration("maximum_actual_linear_speed_mps"), value_type=float
                        ),
                        "maximum_actual_angular_speed_radps": ParameterValue(
                            LaunchConfiguration("maximum_actual_angular_speed_radps"), value_type=float
                        ),
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "semantic_label_path": LaunchConfiguration("semantic_label"),
                        "inflate_radius": ParameterValue(
                            LaunchConfiguration("inflate_radius"), value_type=float
                        ),
                        "goal_tolerance": ParameterValue(
                            LaunchConfiguration("goal_tolerance"), value_type=float
                        ),
                        "robot_radius": ParameterValue(
                            LaunchConfiguration("robot_radius"), value_type=float
                        ),
                        "pedestrian_radius": ParameterValue(
                            LaunchConfiguration("pedestrian_radius"), value_type=float
                        ),
                        "personal_space_radius": ParameterValue(
                            LaunchConfiguration("personal_space_radius"), value_type=float
                        ),
                        "stopped_speed_threshold": ParameterValue(
                            LaunchConfiguration("stopped_speed_threshold"), value_type=float
                        ),
                        "experiment_scene_id": LaunchConfiguration(
                            "experiment_scene_id"
                        ),
                        "scene_file": LaunchConfiguration("scene_file"),
                        "robot_x": ParameterValue(LaunchConfiguration("robot_x"), value_type=float),
                        "robot_y": ParameterValue(LaunchConfiguration("robot_y"), value_type=float),
                        "robot_yaw": ParameterValue(LaunchConfiguration("robot_yaw"), value_type=float),
                        "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                        "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                        "pedestrian_seed": ParameterValue(LaunchConfiguration("pedestrian_seed"), value_type=int),
                        "pedestrian_count": ParameterValue(LaunchConfiguration("pedestrian_count"), value_type=int),
                        "method_name": "DRL-VO",
                        "producer_id": "drl_vo_policy",
                        "policy_mode": LaunchConfiguration("policy_mode"),
                        "checkpoint": LaunchConfiguration("selected_drl_vo_model"),
                        "device": LaunchConfiguration("device"),
                        "pedestrian_source": LaunchConfiguration("pedestrian_source"),
                        "oracle_pedestrian_velocity": ParameterValue(
                            LaunchConfiguration("oracle_pedestrian_velocity"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="semantic_nav_gazebo",
                        executable="semantic_start_goal_path_node.py",
                        name="drl_vo_start_goal_path",
                        output="screen",
                        on_exit=[
                            Shutdown(
                                reason="DRL-VO start/goal path node exited"
                            )
                        ],
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "goal_x": ParameterValue(
                                    LaunchConfiguration("goal_x"),
                                    value_type=float,
                                ),
                                "goal_y": ParameterValue(
                                    LaunchConfiguration("goal_y"),
                                    value_type=float,
                                ),
                                "auto_set_initial_goal": ParameterValue(
                                    LaunchConfiguration(
                                        "auto_set_initial_goal"
                                    ),
                                    value_type=bool,
                                ),
                                "lookahead": ParameterValue(
                                    LaunchConfiguration("lookahead"),
                                    value_type=float,
                                ),
                                "inflate_radius": ParameterValue(
                                    LaunchConfiguration("inflate_radius"),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="semantic_nav_gazebo",
                        executable="drl_vo_fixed_dual_inference_node.py",
                        name="drl_vo_fixed_dual_inference",
                        output="screen",
                        prefix=[LaunchConfiguration("drl_vo_python")],
                        on_exit=[
                            Shutdown(reason="DRL-VO inference node exited")
                        ],
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "mode": LaunchConfiguration("policy_mode"),
                                "model": LaunchConfiguration(
                                    "selected_drl_vo_model"
                                ),
                                "device": LaunchConfiguration("device"),
                                "pedestrian_source": LaunchConfiguration(
                                    "pedestrian_source"
                                ),
                                "perception_model": LaunchConfiguration(
                                    "perception_model"
                                ),
                                "perception_confidence_threshold": ParameterValue(
                                    LaunchConfiguration(
                                        "perception_confidence_threshold"
                                    ),
                                    value_type=float,
                                ),
                                "perception_topk": ParameterValue(
                                    LaunchConfiguration("perception_topk"),
                                    value_type=int,
                                ),
                                "perception_nms_radius_m": ParameterValue(
                                    LaunchConfiguration(
                                        "perception_nms_radius_m"
                                    ),
                                    value_type=float,
                                ),
                                "coasting_max_time_s": ParameterValue(
                                    LaunchConfiguration(
                                        "coasting_max_time_s"
                                    ),
                                    value_type=float,
                                ),
                                "max_track_age_s": ParameterValue(
                                    LaunchConfiguration("max_track_age_s"),
                                    value_type=float,
                                ),
                                "include_tentative_tracks": ParameterValue(
                                    LaunchConfiguration(
                                        "include_tentative_tracks"
                                    ),
                                    value_type=bool,
                                ),
                                "perception_metrics_path": LaunchConfiguration(
                                    "perception_metrics_path"
                                ),
                                "publish_policy_actions": ParameterValue(
                                    LaunchConfiguration(
                                        "publish_policy_actions"
                                    ),
                                    value_type=bool,
                                ),
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "semantic_label": LaunchConfiguration(
                                    "semantic_label"
                                ),
                                "cmd_vel_topic": LaunchConfiguration(
                                    "cmd_vel_topic"
                                ),
                                "inference_metrics_topic": LaunchConfiguration(
                                    "inference_metrics_topic"
                                ),
                                "actuation_decision_topic": LaunchConfiguration(
                                    "actuation_decision_topic"
                                ),
                                "max_linear": ParameterValue(
                                    LaunchConfiguration("max_linear"),
                                    value_type=float,
                                ),
                                "max_angular": ParameterValue(
                                    LaunchConfiguration("max_angular"),
                                    value_type=float,
                                ),
                                "goal_tolerance": ParameterValue(
                                    LaunchConfiguration("goal_tolerance"),
                                    value_type=float,
                                ),
                                "front_stop_distance": ParameterValue(
                                    LaunchConfiguration(
                                        "front_stop_distance"
                                    ),
                                    value_type=float,
                                ),
                                "stop_on_empty_front": ParameterValue(
                                    LaunchConfiguration(
                                        "stop_on_empty_front"
                                    ),
                                    value_type=bool,
                                ),
                                "front_stop_angular_deadband": ParameterValue(
                                    LaunchConfiguration(
                                        "front_stop_angular_deadband"
                                    ),
                                    value_type=float,
                                ),
                                "front_stop_min_angular": ParameterValue(
                                    LaunchConfiguration(
                                        "front_stop_min_angular"
                                    ),
                                    value_type=float,
                                ),
                                "enable_actuation_deadlock_detection": ParameterValue(
                                    LaunchConfiguration(
                                        "enable_actuation_deadlock_detection"
                                    ),
                                    value_type=bool,
                                ),
                                "actuation_deadlock_window_sec": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_window_sec"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_min_command_ratio": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_min_command_ratio"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_goal_x_threshold": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_goal_x_threshold"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_max_linear_command": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_max_linear_command"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_min_angular_command": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_min_angular_command"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_max_displacement_m": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_max_displacement_m"
                                    ),
                                    value_type=float,
                                ),
                                "actuation_deadlock_max_yaw_progress_rad": ParameterValue(
                                    LaunchConfiguration(
                                        "actuation_deadlock_max_yaw_progress_rad"
                                    ),
                                    value_type=float,
                                ),
                                "scan_timeout": ParameterValue(
                                    LaunchConfiguration("scan_timeout"),
                                    value_type=float,
                                ),
                                "odom_timeout": ParameterValue(
                                    LaunchConfiguration("odom_timeout"),
                                    value_type=float,
                                ),
                                "subgoal_timeout": ParameterValue(
                                    LaunchConfiguration("subgoal_timeout"),
                                    value_type=float,
                                ),
                                "final_goal_timeout": ParameterValue(
                                    LaunchConfiguration(
                                        "final_goal_timeout"
                                    ),
                                    value_type=float,
                                ),
                                "pedestrian_truth_timeout": ParameterValue(
                                    LaunchConfiguration(
                                        "pedestrian_truth_timeout"
                                    ),
                                    value_type=float,
                                ),
                                "require_pedestrian_truth": ParameterValue(
                                    LaunchConfiguration(
                                        "require_pedestrian_truth"
                                    ),
                                    value_type=bool,
                                ),
                                "oracle_pedestrian_velocity": ParameterValue(
                                    LaunchConfiguration(
                                        "oracle_pedestrian_velocity"
                                    ),
                                    value_type=bool,
                                ),
                                "oracle_person_semantics": ParameterValue(
                                    LaunchConfiguration(
                                        "oracle_person_semantics"
                                    ),
                                    value_type=bool,
                                ),
                            }
                        ],
                    ),
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("start_online_ppo_training")
                        ),
                        package="semantic_nav_gazebo",
                        executable="drl_vo_online_ppo_training_node.py",
                        name="drl_vo_online_ppo_training",
                        output="screen",
                        prefix=[LaunchConfiguration("drl_vo_python")],
                        on_exit=[
                            Shutdown(
                                reason="DRL-VO online PPO trainer exited"
                            )
                        ],
                        parameters=[
                            {
                                "model": LaunchConfiguration(
                                    "selected_drl_vo_model"
                                ),
                                "output_dir": LaunchConfiguration(
                                    "ppo_output_dir"
                                ),
                                "device": LaunchConfiguration("device"),
                                "total_timesteps": ParameterValue(
                                    LaunchConfiguration(
                                        "ppo_total_timesteps"
                                    ),
                                    value_type=int,
                                ),
                                "rollout_steps": ParameterValue(
                                    LaunchConfiguration("ppo_rollout_steps"),
                                    value_type=int,
                                ),
                                "max_episode_steps": ParameterValue(
                                    LaunchConfiguration(
                                        "ppo_max_episode_steps"
                                    ),
                                    value_type=int,
                                ),
                                "update_epochs": ParameterValue(
                                    LaunchConfiguration("ppo_update_epochs"),
                                    value_type=int,
                                ),
                                "batch_size": ParameterValue(
                                    LaunchConfiguration("ppo_batch_size"),
                                    value_type=int,
                                ),
                                "learning_rate": ParameterValue(
                                    LaunchConfiguration(
                                        "ppo_learning_rate"
                                    ),
                                    value_type=float,
                                ),
                                "seed": ParameterValue(
                                    LaunchConfiguration("ppo_seed"),
                                    value_type=int,
                                ),
                                "freeze_feature_extractor": ParameterValue(
                                    LaunchConfiguration(
                                        "ppo_freeze_feature_extractor"
                                    ),
                                    value_type=bool,
                                ),
                                "reset_service": LaunchConfiguration(
                                    "robot_reset_service"
                                ),
                                "robot_entity_name": LaunchConfiguration(
                                    "robot_entity_name"
                                ),
                                "start_x": ParameterValue(
                                    LaunchConfiguration("robot_x"),
                                    value_type=float,
                                ),
                                "start_y": ParameterValue(
                                    LaunchConfiguration("robot_y"),
                                    value_type=float,
                                ),
                                "start_yaw": ParameterValue(
                                    LaunchConfiguration("robot_yaw"),
                                    value_type=float,
                                ),
                                "max_linear": ParameterValue(
                                    LaunchConfiguration("max_linear"),
                                    value_type=float,
                                ),
                                "max_angular": ParameterValue(
                                    LaunchConfiguration("max_angular"),
                                    value_type=float,
                                ),
                                "goal_tolerance": ParameterValue(
                                    LaunchConfiguration("goal_tolerance"),
                                    value_type=float,
                                ),
                                "perception_model_contract": LaunchConfiguration(
                                    "perception_model"
                                ),
                            }
                        ],
                    ),
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("enable_goal_picker")
                        ),
                        package="semantic_nav_gazebo",
                        executable="demo_goal_arrival_node.py",
                        name="drl_vo_demo_goal_arrival",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "cmd_vel_topic": LaunchConfiguration(
                                    "cmd_vel_topic"
                                ),
                                "goal_tolerance": ParameterValue(
                                    LaunchConfiguration("goal_tolerance"),
                                    value_type=float,
                                ),
                                "arrival_dwell_sec": ParameterValue(
                                    LaunchConfiguration(
                                        "goal_picker_arrival_dwell_sec"
                                    ),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("enable_goal_picker")
                        ),
                        package="semantic_nav_gazebo",
                        executable="episode_goal_picker.py",
                        name="drl_vo_episode_goal_picker",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "show_on_start": True,
                                "instructions_text": (
                                    "在地图上点击导航目标，确认后窗口会隐藏。"
                                    "机器人到达并停止后，窗口会自动再次出现。"
                                ),
                                "ready_status_text": (
                                    "请选择下一个导航目标"
                                ),
                            }
                        ],
                        env=clean_rviz_environment(),
                    ),
                ],
            ),
        ]
    )
