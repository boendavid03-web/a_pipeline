#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /map_server, /odom, /semantic_cnn/actual_trajectory
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PNG, PT, WORLD, YAML
# 可能使用的关键环境变量：NAVIGATION_PROJECT_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo tokennav_v7_start_goal_demo.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.533297111 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.634564093 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/tokennav_v7_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/tokennav_v7_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/tokennav_v7_inference_node.py
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜tokennav_v7_start_goal_demo.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Launch the v7 Gazebo start/goal TokenNav demo."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


def generate_launch_description():
    package_share = get_package_share_directory("semantic_nav_gazebo")
    bringup_launch = os.path.join(package_share, "launch", "semantic_cnn_nav_v7_dual_bringup.launch.py")

    project_root = navigation_project_root()
    maps_root = project_root / "assets" / "maps" / "ros2_workspace" / "semantic_labeling_v6"
    default_map_yaml = str(maps_root / "v6_lidar04m_20m_static_map.yaml")
    default_label = str(maps_root / "semantic2d_manual_label" / "label.png")
    default_checkpoint = str(
        project_root
        / "experiments"
        / "tokennav"
        / "checkpoints"
        / "exp_20260628_long_continue_risk_fast_w12_e10"
        / "last.pt"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("robot_x", default_value="2.0"),
            DeclareLaunchArgument("robot_y", default_value="2.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="6.0"),
            DeclareLaunchArgument("goal_y", default_value="4.0"),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("semantic_label", default_value=default_label),
            DeclareLaunchArgument("checkpoint_path", default_value=default_checkpoint),
            DeclareLaunchArgument("motion_gate_tau", default_value="0.4"),
            DeclareLaunchArgument("instruction_id", default_value="0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="false"),
            DeclareLaunchArgument("start_map_server", default_value="true"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument("publish_actual_trajectory", default_value="true"),
            DeclareLaunchArgument("max_linear", default_value="0.3"),
            DeclareLaunchArgument("max_angular", default_value="1.0"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.35"),
            DeclareLaunchArgument("front_stop_distance", default_value="1.0"),
            DeclareLaunchArgument("front_stop_angular_deadband", default_value="0.05"),
            DeclareLaunchArgument("front_stop_min_angular", default_value="0.35"),
            DeclareLaunchArgument("lookahead", default_value="1.2"),
            DeclareLaunchArgument("inflate_radius", default_value="0.75"),
            DeclareLaunchArgument("allow_scan_resample", default_value="false"),
            DeclareLaunchArgument("target_num_beams", default_value="1081"),
            DeclareLaunchArgument("resample_method", default_value="nearest"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world": LaunchConfiguration("world"),
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "start_slam": LaunchConfiguration("start_slam"),
                }.items(),
            ),

            Node(
                condition=IfCondition(LaunchConfiguration("start_map_server")),
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "yaml_filename": LaunchConfiguration("map_yaml"),
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(LaunchConfiguration("start_map_server")),
                        cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(LaunchConfiguration("start_map_server")),
                        cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                        output="screen",
                    ),
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("publish_map_to_odom_tf")),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("publish_actual_trajectory")),
                package="semantic_nav_gazebo",
                executable="odom_path_node.py",
                name="tokennav_actual_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "odom_topic": "/odom",
                        "path_topic": "/semantic_cnn/actual_trajectory",
                    }
                ],
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="semantic_nav_gazebo",
                        executable="semantic_start_goal_path_node.py",
                        name="semantic_start_goal_path",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                                "lookahead": ParameterValue(LaunchConfiguration("lookahead"), value_type=float),
                                "inflate_radius": ParameterValue(
                                    LaunchConfiguration("inflate_radius"),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="semantic_nav_gazebo",
                        executable="tokennav_v7_inference_node.py",
                        name="tokennav_v7_inference",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "semantic_label": LaunchConfiguration("semantic_label"),
                                "checkpoint_path": LaunchConfiguration("checkpoint_path"),
                                "motion_gate_tau": ParameterValue(
                                    LaunchConfiguration("motion_gate_tau"),
                                    value_type=float,
                                ),
                                "instruction_id": ParameterValue(
                                    LaunchConfiguration("instruction_id"),
                                    value_type=int,
                                ),
                                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                                "max_linear": ParameterValue(LaunchConfiguration("max_linear"), value_type=float),
                                "max_angular": ParameterValue(LaunchConfiguration("max_angular"), value_type=float),
                                "goal_tolerance": ParameterValue(LaunchConfiguration("goal_tolerance"), value_type=float),
                                "front_stop_distance": ParameterValue(
                                    LaunchConfiguration("front_stop_distance"),
                                    value_type=float,
                                ),
                                "front_stop_angular_deadband": ParameterValue(
                                    LaunchConfiguration("front_stop_angular_deadband"),
                                    value_type=float,
                                ),
                                "front_stop_min_angular": ParameterValue(
                                    LaunchConfiguration("front_stop_min_angular"),
                                    value_type=float,
                                ),
                                "allow_scan_resample": ParameterValue(
                                    LaunchConfiguration("allow_scan_resample"),
                                    value_type=bool,
                                ),
                                "target_num_beams": ParameterValue(
                                    LaunchConfiguration("target_num_beams"),
                                    value_type=int,
                                ),
                                "resample_method": LaunchConfiguration("resample_method"),
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
