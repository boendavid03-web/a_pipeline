#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /map_server, /odom, /semantic_cnn/actual_trajectory
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PNG, PT, WORLD, YAML
# 可能使用的关键环境变量：GIO_, GTK_PATH, NAVIGATION_PROJECT_ROOT, RUN_ROOT, SEMANTIC_CNN_MODEL, SEMANTIC_CNN_MODEL_CODE, SNAP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo semantic_cnn_fixed_dual_start_goal_demo.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-18 00:21:44.799424853 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.851391683 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_fixed_dual_start_goal_demo.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Closed-loop start/goal demo for the fixed-dual SemanticCNN checkpoint."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def clean_rviz_environment():
    """Avoid VS Code snap GTK/GIO paths loading core20 libraries into host RViz."""
    environment = os.environ.copy()
    for name in tuple(environment):
        if name == "GTK_PATH" or name.startswith("GIO_") or name.startswith("SNAP"):
            environment.pop(name)
    return environment


def generate_launch_description():
    project_root = Path(os.environ.get("NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5]))
    run_root = Path(os.environ.get("RUN_ROOT", project_root / "runs" / "20260717_042135_v7_dual"))
    package_share = get_package_share_directory("semantic_nav_gazebo")
    bringup_launch = str(Path(package_share) / "launch" / "semantic_cnn_nav_v7_dual_bringup.launch.py")
    default_rviz_config = str(Path(package_share) / "rviz" / "semantic_cnn_fixed_dual_debug.rviz")
    train_python = str(project_root / ".venvs" / "train" / "bin" / "python")
    default_result_dir = (
        run_root
        / "datasets"
        / "20260727_three_bag_online_seed_split_v1"
        / "training"
        / "semantic_cnn"
        / "20260727_120116_semantic_cnn_native_cmd_51epoch"
    )
    default_model = os.environ.get(
        "SEMANTIC_CNN_MODEL",
        str(default_result_dir / "semantic_cnn_native_cmd_best_dev.pth"),
    )
    default_model_code = os.environ.get(
        "SEMANTIC_CNN_MODEL_CODE",
        str(Path(default_model).parent / "model_code_scripts"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_bringup", default_value="true"),
        DeclareLaunchArgument("start_aux_map", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
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
            description="false waits for the map picker; true uses goal_x/goal_y.",
        ),
        DeclareLaunchArgument("goal_picker_arrival_dwell_sec", default_value="0.5"),
        DeclareLaunchArgument("spawn_scene_pedestrians", default_value="false"),
        DeclareLaunchArgument("scene_file", default_value="scenarios/lobby/eng_hall_15.xml"),
        DeclareLaunchArgument("pedestrian_count", default_value="8"),
        DeclareLaunchArgument("pedestrian_speed", default_value="1.0"),
        DeclareLaunchArgument("pedestrian_seed", default_value="7"),
        DeclareLaunchArgument("pedestrian_use_actors", default_value="false"),
        DeclareLaunchArgument("map_yaml", default_value=str(run_root / "maps" / "semantic_label" / "map.yaml")),
        DeclareLaunchArgument("semantic_label", default_value=str(run_root / "maps" / "semantic_label" / "label.png")),
        DeclareLaunchArgument("semantic_cnn_model", default_value=default_model),
        DeclareLaunchArgument("semantic_cnn_model_code", default_value=default_model_code),
        DeclareLaunchArgument("device", default_value="cuda"),
        DeclareLaunchArgument(
            "semantic_cnn_pool_mode",
            default_value="global_virtual_angle_80",
            choices=("global_virtual_angle_80", "sensor_split_40x2"),
        ),
        DeclareLaunchArgument("lidar_range_max", default_value="8.0"),
        DeclareLaunchArgument("pool_range_max", default_value="8.0"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "actuation_decision_topic", default_value="/semantic_cnn/actuation_decision"
        ),
        DeclareLaunchArgument("cmd_vel_angular_z_scale", default_value="1.5"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("publish_debug_images", default_value="true"),
        DeclareLaunchArgument("debug_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("max_linear", default_value="0.11"),
        DeclareLaunchArgument("max_angular", default_value="1.5"),
        DeclareLaunchArgument("subgoal_timeout", default_value="0.3"),
        DeclareLaunchArgument("scan_timeout", default_value="0.5"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.35"),
        DeclareLaunchArgument("front_stop_distance", default_value="0.5"),
        DeclareLaunchArgument("stop_on_empty_front", default_value="true"),
        DeclareLaunchArgument("front_stop_angular_deadband", default_value="0.05"),
        DeclareLaunchArgument("front_stop_min_angular", default_value="0.35"),
        DeclareLaunchArgument("lookahead", default_value="1.0"),
        DeclareLaunchArgument("inflate_radius", default_value="1.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("record_trace", default_value="false"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("trace_timeout_sec", default_value="90.0"),
        DeclareLaunchArgument("evaluate_episode", default_value="false"),
        DeclareLaunchArgument("evaluation_output_dir", default_value=""),
        DeclareLaunchArgument("evaluation_timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("evaluation_multi_episode", default_value="true"),
        DeclareLaunchArgument(
            "inference_metrics_topic",
            default_value="/navigation_evaluation/inference_metrics",
        ),
        DeclareLaunchArgument(
            "simulator_actuation_topic", default_value="/isaac/actuation_state"
        ),
        DeclareLaunchArgument("alignment_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("alignment_freshness_sec", default_value="0.20"),
        DeclareLaunchArgument("alignment_max_delay_sec", default_value="0.50"),
        DeclareLaunchArgument("experiment_scene_id", default_value="semantic_cnn_manual_demo"),
        DeclareLaunchArgument("robot_radius", default_value="0.34"),
        DeclareLaunchArgument("pedestrian_radius", default_value="0.125"),
        DeclareLaunchArgument("personal_space_radius", default_value="0.8"),
        DeclareLaunchArgument("stopped_speed_threshold", default_value="0.02"),
        SetLaunchConfiguration("fixed_dual_start_rviz", LaunchConfiguration("start_rviz")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"), "world": LaunchConfiguration("world"),
                "robot_x": LaunchConfiguration("robot_x"), "robot_y": LaunchConfiguration("robot_y"),
                "robot_yaw": LaunchConfiguration("robot_yaw"), "start_rviz": "false",
                "rviz_config": LaunchConfiguration("rviz_config"),
                "spawn_scene_pedestrians": LaunchConfiguration("spawn_scene_pedestrians"),
                "scene_file": LaunchConfiguration("scene_file"),
                "pedestrian_count": LaunchConfiguration("pedestrian_count"),
                "pedestrian_speed": LaunchConfiguration("pedestrian_speed"),
                "pedestrian_seed": LaunchConfiguration("pedestrian_seed"),
                "pedestrian_use_actors": LaunchConfiguration("pedestrian_use_actors"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "cmd_vel_angular_z_scale": LaunchConfiguration(
                    "cmd_vel_angular_z_scale"
                ),
                "start_merger": "false",
                "start_slam": "false", "lidar_samples_01": "2000", "lidar_samples_02": "2000",
                "lidar_update_rate_01": "15", "lidar_update_rate_02": "15",
                "lidar_range_max_01": LaunchConfiguration("lidar_range_max"),
                "lidar_range_max_02": LaunchConfiguration("lidar_range_max"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_bringup")),
        ),
        TimerAction(period=4.0, actions=[
            Node(
                condition=IfCondition(LaunchConfiguration("fixed_dual_start_rviz")),
                package="rviz2", executable="rviz2", name="rviz2_fixed_dual_debug", output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
                env=clean_rviz_environment(),
            ),
        ]),
        Node(
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            package="semantic_nav_gazebo", executable="demo_velocity_display_node.py",
            name="demo_velocity_display", output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                         "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("start_aux_map")),
            package="nav2_map_server", executable="map_server", name="map_server", output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                         "yaml_filename": LaunchConfiguration("map_yaml")}],
        ),
        TimerAction(condition=IfCondition(LaunchConfiguration("start_aux_map")), period=2.0, actions=[ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/map_server", "configure"], output="screen")]),
        TimerAction(condition=IfCondition(LaunchConfiguration("start_aux_map")), period=3.0, actions=[ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/map_server", "activate"], output="screen")]),
        Node(
            condition=IfCondition(LaunchConfiguration("start_aux_map")),
            package="tf2_ros", executable="static_transform_publisher", name="map_to_odom_static_tf_publisher", output="screen",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "map", "--child-frame-id", "odom"],
        ),
        Node(
            package="semantic_nav_gazebo", executable="odom_path_node.py", name="semantic_cnn_actual_trajectory", output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                         "odom_topic": "/odom", "path_topic": "/semantic_cnn/actual_trajectory"}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("record_trace")),
            package="semantic_nav_gazebo", executable="closed_loop_demo_recorder.py", name="closed_loop_demo_recorder", output="screen",
            parameters=[{"odom_topic": "/odom", "output_csv": LaunchConfiguration("trace_path"),
                         "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                         "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                         "follow_accepted_goal": ParameterValue(LaunchConfiguration("enable_goal_picker"), value_type=bool),
                         "goal_tolerance": ParameterValue(LaunchConfiguration("goal_tolerance"), value_type=float),
                         "timeout_sec": ParameterValue(LaunchConfiguration("trace_timeout_sec"), value_type=float)}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("evaluate_episode")),
            package="semantic_nav_gazebo",
            executable="navigation_episode_evaluator.py",
            name="semantic_cnn_navigation_episode_evaluator",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "evaluation_output_dir": LaunchConfiguration("evaluation_output_dir"),
                "evaluation_timeout_sec": ParameterValue(
                    LaunchConfiguration("evaluation_timeout_sec"), value_type=float
                ),
                "evaluation_multi_episode": ParameterValue(
                    LaunchConfiguration("evaluation_multi_episode"), value_type=bool
                ),
                "inference_metrics_topic": LaunchConfiguration("inference_metrics_topic"),
                "actuation_decision_topic": LaunchConfiguration(
                    "actuation_decision_topic"
                ),
                "simulator_actuation_topic": LaunchConfiguration(
                    "simulator_actuation_topic"
                ),
                "alignment_rate_hz": ParameterValue(
                    LaunchConfiguration("alignment_rate_hz"), value_type=float
                ),
                "alignment_freshness_sec": ParameterValue(
                    LaunchConfiguration("alignment_freshness_sec"), value_type=float
                ),
                "alignment_max_delay_sec": ParameterValue(
                    LaunchConfiguration("alignment_max_delay_sec"), value_type=float
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
                "experiment_scene_id": LaunchConfiguration("experiment_scene_id"),
                "scene_file": LaunchConfiguration("scene_file"),
                "robot_x": ParameterValue(LaunchConfiguration("robot_x"), value_type=float),
                "robot_y": ParameterValue(LaunchConfiguration("robot_y"), value_type=float),
                "robot_yaw": ParameterValue(LaunchConfiguration("robot_yaw"), value_type=float),
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "pedestrian_seed": ParameterValue(
                    LaunchConfiguration("pedestrian_seed"), value_type=int
                ),
                "pedestrian_count": ParameterValue(
                    LaunchConfiguration("pedestrian_count"), value_type=int
                ),
                "method_name": "SemanticCNN",
                "producer_id": "semantic_cnn_policy",
                "policy_mode": "best_dev",
                "checkpoint": LaunchConfiguration("semantic_cnn_model"),
                "device": LaunchConfiguration("device"),
                "pedestrian_source": "semantic_map_lookup",
                "oracle_pedestrian_velocity": False,
            }],
        ),
        TimerAction(period=7.0, actions=[
            Node(
                package="semantic_nav_gazebo", executable="semantic_start_goal_path_node.py", name="semantic_start_goal_path", output="screen",
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                             "map_yaml": LaunchConfiguration("map_yaml"),
                             "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                             "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                             "auto_set_initial_goal": ParameterValue(LaunchConfiguration("auto_set_initial_goal"), value_type=bool),
                             "lookahead": ParameterValue(LaunchConfiguration("lookahead"), value_type=float),
                             "inflate_radius": ParameterValue(LaunchConfiguration("inflate_radius"), value_type=float)}],
            ),
            Node(
                package="semantic_nav_gazebo", executable="semantic_cnn_fixed_dual_inference_node.py",
                name="semantic_cnn_fixed_dual_inference", output="screen", prefix=[train_python],
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                             "map_yaml": LaunchConfiguration("map_yaml"), "semantic_label": LaunchConfiguration("semantic_label"),
                             "model": LaunchConfiguration("semantic_cnn_model"), "model_code": LaunchConfiguration("semantic_cnn_model_code"),
                             "device": LaunchConfiguration("device"),
                             "inference_metrics_topic": LaunchConfiguration("inference_metrics_topic"),
                             "actuation_decision_topic": LaunchConfiguration(
                                 "actuation_decision_topic"
                             ),
                             "pool_mode": LaunchConfiguration("semantic_cnn_pool_mode"),
                             "range_max": ParameterValue(LaunchConfiguration("lidar_range_max"), value_type=float),
                             "pool_range_max": ParameterValue(LaunchConfiguration("pool_range_max"), value_type=float),
                             "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                             "max_linear": ParameterValue(LaunchConfiguration("max_linear"), value_type=float),
                             "max_angular": ParameterValue(LaunchConfiguration("max_angular"), value_type=float),
                             "subgoal_timeout": ParameterValue(LaunchConfiguration("subgoal_timeout"), value_type=float),
                             "scan_timeout": ParameterValue(LaunchConfiguration("scan_timeout"), value_type=float),
                             "goal_tolerance": ParameterValue(LaunchConfiguration("goal_tolerance"), value_type=float),
                             "front_stop_distance": ParameterValue(LaunchConfiguration("front_stop_distance"), value_type=float),
                             "stop_on_empty_front": ParameterValue(LaunchConfiguration("stop_on_empty_front"), value_type=bool),
                             "front_stop_angular_deadband": ParameterValue(LaunchConfiguration("front_stop_angular_deadband"), value_type=float),
                             "front_stop_min_angular": ParameterValue(LaunchConfiguration("front_stop_min_angular"), value_type=float),
                             "visualize": ParameterValue(LaunchConfiguration("visualize"), value_type=bool),
                             "publish_debug_images": ParameterValue(LaunchConfiguration("publish_debug_images"), value_type=bool),
                             "debug_rate_hz": ParameterValue(LaunchConfiguration("debug_rate_hz"), value_type=float)}],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("enable_goal_picker")),
                package="semantic_nav_gazebo", executable="demo_goal_arrival_node.py",
                name="semantic_cnn_demo_goal_arrival", output="screen",
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                             "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                             "goal_tolerance": ParameterValue(LaunchConfiguration("goal_tolerance"), value_type=float),
                             "arrival_dwell_sec": ParameterValue(LaunchConfiguration("goal_picker_arrival_dwell_sec"), value_type=float)}],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("enable_goal_picker")),
                package="semantic_nav_gazebo", executable="episode_goal_picker.py",
                name="semantic_cnn_episode_goal_picker", output="screen",
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                             "map_yaml": LaunchConfiguration("map_yaml"),
                             "show_on_start": True,
                             "instructions_text": (
                                 "在地图上点击导航目标，确认后窗口会隐藏。"
                                 "机器人到达并停止后，窗口会自动再次出现。"
                             ),
                             "ready_status_text": "请选择下一个导航目标"}],
                env=clean_rviz_environment(),
            ),
        ]),
    ])
