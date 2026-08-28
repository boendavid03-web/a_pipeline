#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/map_server, /odom, /semantic_cnn/actual_trajectory
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：WORLD, YAML
# 可能使用的关键环境变量：GIO_, GTK_PATH, NAVIGATION_PROJECT_ROOT, RUN_ROOT, SNAP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo teleop_fixed_map_capture.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 04:10:08.804725174 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.634564093 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh（通过 ros2 launch 启动该 ROS 2 场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/odom_path_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜teleop_fixed_map_capture.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Fixed-map Gazebo visualization for multi-episode teleop data capture."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def clean_gui_environment():
    environment = os.environ.copy()
    for name in tuple(environment):
        if name == "GTK_PATH" or name.startswith("GIO_") or name.startswith("SNAP"):
            environment.pop(name)
    return environment


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
    package_share = Path(get_package_share_directory("semantic_nav_gazebo"))
    bringup = package_share / "launch" / "semantic_cnn_nav_v7_dual_bringup.launch.py"
    default_map = run_root / "maps" / "semantic_label" / "map.yaml"
    default_rviz = package_share / "rviz" / "semantic_cnn_fixed_dual_debug.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_goal_picker", default_value="true"),
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("map_yaml", default_value=str(default_map)),
            DeclareLaunchArgument("rviz_config", default_value=str(default_rviz)),
            DeclareLaunchArgument("robot_x", default_value="2.0"),
            DeclareLaunchArgument("robot_y", default_value="2.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="16.0"),
            DeclareLaunchArgument("goal_y", default_value="16.0"),
            DeclareLaunchArgument("lookahead", default_value="1.2"),
            DeclareLaunchArgument("inflate_radius", default_value="0.53"),
            DeclareLaunchArgument("spawn_scene_pedestrians", default_value="true"),
            DeclareLaunchArgument("pedestrian_use_actors", default_value="false"),
            DeclareLaunchArgument("pedestrian_count", default_value="15"),
            DeclareLaunchArgument("pedestrian_seed", default_value="7"),
            DeclareLaunchArgument("pedestrian_speed", default_value="1.0"),
            DeclareLaunchArgument("pedestrian_update_rate", default_value="15.0"),
            DeclareLaunchArgument(
                "pedestrian_simulation_factor", default_value="1.0"
            ),
            DeclareLaunchArgument("lidar_samples_01", default_value="2000"),
            DeclareLaunchArgument("lidar_samples_02", default_value="2000"),
            DeclareLaunchArgument("lidar_update_rate_01", default_value="15.0"),
            DeclareLaunchArgument("lidar_update_rate_02", default_value="15.0"),
            DeclareLaunchArgument("lidar_range_min_01", default_value="0.1"),
            DeclareLaunchArgument("lidar_range_min_02", default_value="0.1"),
            DeclareLaunchArgument("lidar_range_max_01", default_value="8.0"),
            DeclareLaunchArgument("lidar_range_max_02", default_value="8.0"),
            DeclareLaunchArgument("lidar_runtime_model_file", default_value=""),
            SetLaunchConfiguration(
                "fixed_map_capture_start_rviz",
                LaunchConfiguration("start_rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world": LaunchConfiguration("world"),
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "start_merger": "true",
                    "start_slam": "false",
                    "start_rviz": "false",
                    "spawn_scene_pedestrians": LaunchConfiguration(
                        "spawn_scene_pedestrians"
                    ),
                    "pedestrian_use_actors": LaunchConfiguration(
                        "pedestrian_use_actors"
                    ),
                    "pedestrian_count": LaunchConfiguration("pedestrian_count"),
                    "pedestrian_seed": LaunchConfiguration("pedestrian_seed"),
                    "pedestrian_speed": LaunchConfiguration("pedestrian_speed"),
                    "pedestrian_update_rate": LaunchConfiguration(
                        "pedestrian_update_rate"
                    ),
                    "pedestrian_simulation_factor": LaunchConfiguration(
                        "pedestrian_simulation_factor"
                    ),
                    "lidar_samples_01": LaunchConfiguration("lidar_samples_01"),
                    "lidar_samples_02": LaunchConfiguration("lidar_samples_02"),
                    "lidar_update_rate_01": LaunchConfiguration(
                        "lidar_update_rate_01"
                    ),
                    "lidar_update_rate_02": LaunchConfiguration(
                        "lidar_update_rate_02"
                    ),
                    "lidar_range_min_01": LaunchConfiguration("lidar_range_min_01"),
                    "lidar_range_min_02": LaunchConfiguration("lidar_range_min_02"),
                    "lidar_range_max_01": LaunchConfiguration("lidar_range_max_01"),
                    "lidar_range_max_02": LaunchConfiguration("lidar_range_max_02"),
                    "lidar_runtime_model_file": LaunchConfiguration(
                        "lidar_runtime_model_file"
                    ),
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "yaml_filename": LaunchConfiguration("map_yaml"),
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_capture_tf",
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
                package="semantic_nav_gazebo",
                executable="odom_path_node.py",
                name="capture_actual_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": "/odom",
                        "path_topic": "/semantic_cnn/actual_trajectory",
                        "min_distance": 0.03,
                        "max_poses": 20000,
                    }
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="semantic_nav_gazebo",
                        executable="semantic_start_goal_path_node.py",
                        name="semantic_start_goal_path",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "goal_x": ParameterValue(
                                    LaunchConfiguration("goal_x"), value_type=float
                                ),
                                "goal_y": ParameterValue(
                                    LaunchConfiguration("goal_y"), value_type=float
                                ),
                                "lookahead": ParameterValue(
                                    LaunchConfiguration("lookahead"), value_type=float
                                ),
                                "inflate_radius": ParameterValue(
                                    LaunchConfiguration("inflate_radius"),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("start_goal_picker")
                        ),
                        package="semantic_nav_gazebo",
                        executable="episode_goal_picker.py",
                        name="episode_goal_picker",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "map_yaml": LaunchConfiguration("map_yaml"),
                                "show_on_start": True,
                            }
                        ],
                        env=clean_gui_environment(),
                    ),
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("fixed_map_capture_start_rviz")
                        ),
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_fixed_map_capture",
                        output="screen",
                        arguments=["-d", LaunchConfiguration("rviz_config")],
                        parameters=[{"use_sim_time": True}],
                        env=clean_gui_environment(),
                    ),
                ],
            ),
        ]
    )
