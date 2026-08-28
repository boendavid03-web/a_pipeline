#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /semantic_cnn/debug/markers
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, WORLD
# 可能使用的关键环境变量：GIO_, GTK_PATH, NAVIGATION_PROJECT_ROOT, RUN_ROOT, S3NET_MODEL, S3NET_MODEL_CODE, S3NET_STATS_JSON, SAMPLING_STRATEGIES, SNAP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo s3net_fixed_dual_perception_demo.launch.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:36:51.643219959 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.851391683 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/model.py; /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/model.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_velocity_display_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜s3net_fixed_dual_perception_demo.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Perception-only Gazebo demo for a fixed-dual S3-Net checkpoint."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


SAMPLING_STRATEGIES = ("contract", "seeded_sequence", "frame_seeded")


def navigation_project_root() -> Path:
    return Path(
        os.environ.get(
            "NAVIGATION_PROJECT_ROOT",
            Path(__file__).resolve().parents[5],
        )
    )


def clean_rviz_environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in tuple(environment):
        if name == "GTK_PATH" or name.startswith("GIO_") or name.startswith("SNAP"):
            environment.pop(name)
    return environment


def validate_configuration(context):
    paths = {
        "s3net_model": Path(
            LaunchConfiguration("s3net_model").perform(context)
        ).expanduser(),
        "s3net_model_code/model.py": (
            Path(
                LaunchConfiguration("s3net_model_code").perform(context)
            ).expanduser()
            / "model.py"
        ),
        "s3net_stats_json": Path(
            LaunchConfiguration("s3net_stats_json").perform(context)
        ).expanduser(),
    }
    missing = [
        f"{name}={path}"
        for name, path in paths.items()
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "S3-Net perception demo is missing required run artifacts: "
            + ", ".join(missing)
        )
    strategy = LaunchConfiguration("sampling_strategy").perform(context)
    if strategy not in SAMPLING_STRATEGIES:
        raise ValueError(
            f"sampling_strategy must be one of {SAMPLING_STRATEGIES}, "
            f"got {strategy!r}"
        )
    return [
        LogInfo(
            msg=(
                "[S3-Net perception only] model="
                f"{paths['s3net_model']}; stats={paths['s3net_stats_json']}; "
                f"sampling={strategy}; no navigation controller is launched"
            )
        )
    ]


def generate_launch_description():
    project_root = navigation_project_root()
    package_share = Path(get_package_share_directory("semantic_nav_gazebo"))
    bringup_launch = package_share / "launch" / "semantic_cnn_nav_v7_dual_bringup.launch.py"
    rviz_config = package_share / "rviz" / "semantic_cnn_fixed_dual_debug.rviz"
    train_python = project_root / ".venvs" / "train" / "bin" / "python"
    run_root = Path(
        os.environ.get(
            "RUN_ROOT",
            project_root / "runs" / "20260717_042135_v7_dual",
        )
    )
    default_result_dir = (
        run_root
        / "datasets"
        / "20260727_three_bag_online_seed_split_v1"
        / "training"
        / "s3net"
        / "20260727_162813_s3net_native_stats_301epoch"
    )
    default_model = os.environ.get(
        "S3NET_MODEL",
        str(default_result_dir / "s3net_native_stats_epoch_0300.pth"),
    )
    selected_result_dir = Path(default_model).parent
    default_model_code = os.environ.get(
        "S3NET_MODEL_CODE",
        str(selected_result_dir / "model_code_scripts"),
    )
    default_stats = os.environ.get(
        "S3NET_STATS_JSON",
        str(selected_result_dir / "s3net_native_lidar_train_stats.json"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("robot_x", default_value="2.0"),
            DeclareLaunchArgument("robot_y", default_value="2.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("spawn_scene_pedestrians", default_value="true"),
            DeclareLaunchArgument(
                "scene_file",
                default_value="scenarios/lobby/eng_hall_15.xml",
            ),
            DeclareLaunchArgument("pedestrian_count", default_value="8"),
            DeclareLaunchArgument("pedestrian_speed", default_value="1.0"),
            DeclareLaunchArgument("pedestrian_seed", default_value="7"),
            DeclareLaunchArgument("pedestrian_use_actors", default_value="false"),
            DeclareLaunchArgument("s3net_model", default_value=default_model),
            DeclareLaunchArgument(
                "s3net_model_code",
                default_value=default_model_code,
            ),
            DeclareLaunchArgument(
                "s3net_stats_json",
                default_value=default_stats,
            ),
            DeclareLaunchArgument(
                "sampling_strategy",
                default_value="contract",
                description=(
                    "contract keeps eval-time VAE sampling unchanged; "
                    "seeded_sequence seeds it once; frame_seeded seeds each frame"
                ),
            ),
            DeclareLaunchArgument("sampling_seed", default_value="1337"),
            DeclareLaunchArgument("visualization_rate_hz", default_value="5.0"),
            DeclareLaunchArgument(
                "markers_topic",
                default_value="/semantic_cnn/debug/markers",
                description=(
                    "The existing fixed-dual RViz config listens here. "
                    "The process remains an S3-Net-only publisher."
                ),
            ),
            OpaqueFunction(function=validate_configuration),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup_launch)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world": LaunchConfiguration("world"),
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "start_rviz": "false",
                    "bridge_robot_control": "false",
                    "use_cmd_vel_relay": "false",
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
                }.items(),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        condition=IfCondition(
                            LaunchConfiguration("start_rviz")
                        ),
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_s3net_fixed_dual",
                        output="screen",
                        arguments=[
                            "-d",
                            str(rviz_config),
                            "-f",
                            "base_link",
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
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="semantic_nav_gazebo",
                        executable="s3net_fixed_dual_inference_node.py",
                        name="s3net_fixed_dual_inference",
                        output="screen",
                        prefix=[str(train_python)],
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    LaunchConfiguration("use_sim_time"),
                                    value_type=bool,
                                ),
                                "model": LaunchConfiguration("s3net_model"),
                                "model_code": LaunchConfiguration(
                                    "s3net_model_code"
                                ),
                                "stats_json": LaunchConfiguration(
                                    "s3net_stats_json"
                                ),
                                "sampling_strategy": LaunchConfiguration(
                                    "sampling_strategy"
                                ),
                                "sampling_seed": ParameterValue(
                                    LaunchConfiguration("sampling_seed"),
                                    value_type=int,
                                ),
                                "visualization_rate_hz": ParameterValue(
                                    LaunchConfiguration(
                                        "visualization_rate_hz"
                                    ),
                                    value_type=float,
                                ),
                                "markers_topic": LaunchConfiguration(
                                    "markers_topic"
                                ),
                            }
                        ],
                    )
                ],
            ),
        ]
    )
