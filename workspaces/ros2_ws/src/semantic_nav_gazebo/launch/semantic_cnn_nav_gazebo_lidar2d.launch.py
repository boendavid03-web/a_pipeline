#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/scan
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：SDF, WORLD
# 可能使用的关键环境变量：NAVIGATION_PROJECT_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo semantic_cnn_nav_gazebo_lidar2d.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.533297111 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.633564074 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo_lidar2d.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_nav_gazebo_lidar2d.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
# -*- coding: utf-8 -*-

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


def generate_launch_description():
    package_share = get_package_share_directory("semantic_nav_gazebo")
    base_launch = f"{package_share}/launch/semantic_cnn_nav_gazebo.launch_1.py"

    default_robot_model = str(
        navigation_project_root()
        / "assets"
        / "robots"
        / "mecanum_v7"
        / "exported_from_usd"
        / "mecanum730_xms5_nav_proxy_fallback_v3_rotation_fix_lidar2d"
        / "model.sdf"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("run", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="3"),

            DeclareLaunchArgument("robot_model_file", default_value=default_robot_model),
            DeclareLaunchArgument("robot_name", default_value="mecanum730_xms5_lidar2d"),
            DeclareLaunchArgument("robot_x", default_value="0.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_delay", default_value="5.0"),

            DeclareLaunchArgument("lidar_topic", default_value="/scan"),
            DeclareLaunchArgument("bridge_lidar", default_value="true"),
            DeclareLaunchArgument("bridge_clock", default_value="true"),
            DeclareLaunchArgument("bridge_robot_control", default_value="true"),
            DeclareLaunchArgument("use_cmd_vel_relay", default_value="true"),

            DeclareLaunchArgument("lidar_static_parent_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "lidar_static_child_frame",
                default_value="mecanum730_xms5_lidar2d/lidar_2d_link/lidar_2d",
            ),
            DeclareLaunchArgument("lidar_static_xyz", default_value="0.303 0.120 0.995"),
            DeclareLaunchArgument("lidar_static_rpy", default_value="0.0 0.0 0.0"),

            # pedestrian parameters: pass-through to base launch
            DeclareLaunchArgument("spawn_scene_pedestrians", default_value="false"),
            DeclareLaunchArgument("spawn_demo_pedestrians", default_value="false"),
            DeclareLaunchArgument("scene_file", default_value="scenarios/lobby/eng_hall_15.xml"),
            DeclareLaunchArgument("pedestrian_model_file", default_value="models/person_standing/model.sdf"),
            DeclareLaunchArgument("pedestrian_spawn_delay", default_value="4.0"),
            DeclareLaunchArgument("pedestrian_update_rate", default_value="5.0"),
            DeclareLaunchArgument("pedestrian_simulation_factor", default_value="1.0"),
            DeclareLaunchArgument("pedestrian_speed", default_value="1.34"),
            DeclareLaunchArgument("pedestrian_agent_radius", default_value="0.35"),
            DeclareLaunchArgument("pedestrian_force_obstacle", default_value="10.0"),
            DeclareLaunchArgument("pedestrian_sigma_obstacle", default_value="0.2"),
            DeclareLaunchArgument("pedestrian_force_social", default_value="5.1"),
            DeclareLaunchArgument("pedestrian_enable_groups", default_value="true"),
            DeclareLaunchArgument("pedestrian_group_size_lambda", default_value="1.1"),
            DeclareLaunchArgument("pedestrian_force_group_gaze", default_value="3.0"),
            DeclareLaunchArgument("pedestrian_force_group_coherence", default_value="2.0"),
            DeclareLaunchArgument("pedestrian_force_group_repulsion", default_value="1.0"),
            DeclareLaunchArgument("pedestrian_force_random", default_value="0.1"),
            DeclareLaunchArgument("pedestrian_force_along_wall", default_value="2.0"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "run": LaunchConfiguration("run"),
                    "verbose": LaunchConfiguration("verbose"),

                    "spawn_robot": "true",
                    "robot_model_type": "sdf",
                    "robot_model_file": LaunchConfiguration("robot_model_file"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_z": LaunchConfiguration("robot_z"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "robot_spawn_delay": LaunchConfiguration("robot_spawn_delay"),

                    "bridge_lidar": LaunchConfiguration("bridge_lidar"),
                    "lidar_topic": LaunchConfiguration("lidar_topic"),
                    "bridge_clock": LaunchConfiguration("bridge_clock"),
                    "bridge_robot_control": LaunchConfiguration("bridge_robot_control"),
                    "use_cmd_vel_relay": LaunchConfiguration("use_cmd_vel_relay"),

                    "lidar_static_parent_frame": LaunchConfiguration("lidar_static_parent_frame"),
                    "lidar_static_child_frame": LaunchConfiguration("lidar_static_child_frame"),
                    "lidar_static_xyz": LaunchConfiguration("lidar_static_xyz"),
                    "lidar_static_rpy": LaunchConfiguration("lidar_static_rpy"),

                    "spawn_scene_pedestrians": LaunchConfiguration("spawn_scene_pedestrians"),
                    "spawn_demo_pedestrians": LaunchConfiguration("spawn_demo_pedestrians"),
                    "scene_file": LaunchConfiguration("scene_file"),
                    "pedestrian_model_file": LaunchConfiguration("pedestrian_model_file"),
                    "pedestrian_spawn_delay": LaunchConfiguration("pedestrian_spawn_delay"),
                    "pedestrian_update_rate": LaunchConfiguration("pedestrian_update_rate"),
                    "pedestrian_simulation_factor": LaunchConfiguration("pedestrian_simulation_factor"),
                    "pedestrian_speed": LaunchConfiguration("pedestrian_speed"),
                    "pedestrian_agent_radius": LaunchConfiguration("pedestrian_agent_radius"),
                    "pedestrian_force_obstacle": LaunchConfiguration("pedestrian_force_obstacle"),
                    "pedestrian_sigma_obstacle": LaunchConfiguration("pedestrian_sigma_obstacle"),
                    "pedestrian_force_social": LaunchConfiguration("pedestrian_force_social"),
                    "pedestrian_enable_groups": LaunchConfiguration("pedestrian_enable_groups"),
                    "pedestrian_group_size_lambda": LaunchConfiguration("pedestrian_group_size_lambda"),
                    "pedestrian_force_group_gaze": LaunchConfiguration("pedestrian_force_group_gaze"),
                    "pedestrian_force_group_coherence": LaunchConfiguration("pedestrian_force_group_coherence"),
                    "pedestrian_force_group_repulsion": LaunchConfiguration("pedestrian_force_group_repulsion"),
                    "pedestrian_force_random": LaunchConfiguration("pedestrian_force_random"),
                    "pedestrian_force_along_wall": LaunchConfiguration("pedestrian_force_along_wall"),
                }.items(),
            ),
        ]
    )
