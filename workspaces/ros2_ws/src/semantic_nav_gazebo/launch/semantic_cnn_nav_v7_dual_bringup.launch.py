#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/base_scan_01/lidar_2d_01, /base_scan_02/lidar_2d_02, /cmd_vel, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：SDF, WORLD, YAML
# 可能使用的关键环境变量：NAVIGATION_PROJECT_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo semantic_cnn_nav_v7_dual_bringup.launch.py
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/v7_dual_laser_scan_merger.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/tokennav_v7_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/run_v7_dual_slam.sh（通过 ros2 launch 启动该 ROS 2 场景）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/v7_dual_laser_scan_merger.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_v7_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/teleop_fixed_map_capture.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_nav_v7_dual_bringup.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
# -*- coding: utf-8 -*-

import io
import math
import os
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


def _positive_int(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    if not value.isdigit() or int(value) <= 0:
        raise ValueError(f"{name} must be a positive integer, got {value!r}")
    return int(value)


def _positive_float(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive number, got {value!r}") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")
    return parsed


def _prepare_runtime_lidar_model(context):
    source = Path(
        LaunchConfiguration("robot_model_file").perform(context)
    ).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Robot SDF does not exist: {source}")

    samples = {
        "lidar_2d_01": _positive_int(context, "lidar_samples_01"),
        "lidar_2d_02": _positive_int(context, "lidar_samples_02"),
    }
    update_rates = {
        "lidar_2d_01": _positive_float(context, "lidar_update_rate_01"),
        "lidar_2d_02": _positive_float(context, "lidar_update_rate_02"),
    }
    range_mins = {
        "lidar_2d_01": _positive_float(context, "lidar_range_min_01"),
        "lidar_2d_02": _positive_float(context, "lidar_range_min_02"),
    }
    range_maxs = {
        "lidar_2d_01": _positive_float(context, "lidar_range_max_01"),
        "lidar_2d_02": _positive_float(context, "lidar_range_max_02"),
    }
    for sensor_name in samples:
        if range_mins[sensor_name] >= range_maxs[sensor_name]:
            raise ValueError(
                f"{sensor_name}: lidar_range_min must be smaller than "
                f"lidar_range_max, got {range_mins[sensor_name]:g} >= "
                f"{range_maxs[sensor_name]:g}"
            )

    output_value = LaunchConfiguration("lidar_runtime_model_file").perform(context).strip()
    if output_value:
        output = Path(output_value).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output = (
            navigation_project_root()
            / ".runtime"
            / "lidar_models"
            / f"{stamp}_{os.getpid()}"
            / "model.sdf"
        )
    if output == source:
        raise ValueError("lidar_runtime_model_file must not be the source robot SDF")

    try:
        tree = ET.parse(source)
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"Unable to read source robot SDF {source}: {exc}") from exc

    sensors = {
        sensor.attrib.get("name"): sensor
        for sensor in tree.getroot().iter("sensor")
        if sensor.attrib.get("name") in samples
    }
    missing = sorted(set(samples) - set(sensors))
    if missing:
        raise ValueError(f"Source robot SDF is missing GPU LiDAR sensors: {missing}")

    for sensor_name, sensor in sensors.items():
        if sensor.attrib.get("type") != "gpu_lidar":
            raise ValueError(f"Sensor {sensor_name} is not type gpu_lidar")
        samples_element = sensor.find("./lidar/scan/horizontal/samples")
        update_rate_element = sensor.find("./update_rate")
        range_min_element = sensor.find("./lidar/range/min")
        range_max_element = sensor.find("./lidar/range/max")
        if any(
            element is None
            for element in (
                samples_element,
                update_rate_element,
                range_min_element,
                range_max_element,
            )
        ):
            raise ValueError(
                f"Sensor {sensor_name} lacks samples, update_rate, range min, or range max"
            )
        samples_element.text = str(samples[sensor_name])
        update_rate_element.text = format(update_rates[sensor_name], ".15g")
        range_min_element.text = format(range_mins[sensor_name], ".15g")
        range_max_element.text = format(range_maxs[sensor_name], ".15g")

    ET.indent(tree, space="  ")
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True)
    desired = buffer.getvalue()

    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        if not output.is_file() or output.read_bytes() != desired:
            raise FileExistsError(
                f"Refusing to overwrite mismatched runtime model: {output}"
            )
        disposition = "reusing matching runtime model"
    else:
        with output.open("xb") as stream:
            stream.write(desired)
        disposition = "created runtime model"

    details = ", ".join(
        f"{name}: samples={samples[name]}, update_rate={update_rates[name]:g}, "
        f"range=[{range_mins[name]:g}, {range_maxs[name]:g}]"
        for name in ("lidar_2d_01", "lidar_2d_02")
    )
    return [
        SetLaunchConfiguration("robot_model_file", str(output)),
        LogInfo(msg=f"[v7 dual LiDAR] {disposition}: {output}; {details}"),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("semantic_nav_gazebo")
    base_launch = os.path.join(
        package_share,
        "launch",
        "semantic_cnn_nav_gazebo.launch_1.py",
    )

    default_robot_model = str(
        Path(package_share)
        / "models"
        / "mecanum730_xms5_nav_proxy_fallback_v7_teacher_scan01"
        / "model.sdf"
    )
    default_slam_params = os.path.join(
        package_share,
        "config",
        "slam_v7_online_async.yaml",
    )
    default_merger_params = os.path.join(
        package_share,
        "config",
        "v7_dual_laser_scan_merger.yaml",
    )
    default_rviz_config = os.path.join(
        package_share,
        "rviz",
        "v7_dual_slam.rviz",
    )

    robot_name = LaunchConfiguration("robot_name")
    slam_scan_topic = LaunchConfiguration("slam_scan_topic")
    merged_scan_topic = LaunchConfiguration("merged_scan_topic")
    merged_scan_frame = LaunchConfiguration("merged_scan_frame")
    merged_scan_samples = LaunchConfiguration("merged_scan_samples")
    merger_sync_slop = LaunchConfiguration("merger_sync_slop")
    merger_queue_size = LaunchConfiguration("merger_queue_size")
    enable_self_filter = LaunchConfiguration("enable_self_filter")
    enable_high_fidelity_probe = LaunchConfiguration("enable_high_fidelity_probe")
    duplicate_mode = LaunchConfiguration("duplicate_mode")
    duplicate_epsilon = LaunchConfiguration("duplicate_epsilon")
    publish_probe_pointclouds = LaunchConfiguration("publish_probe_pointclouds")
    save_probe_files = LaunchConfiguration("save_probe_files")
    probe_output_dir = LaunchConfiguration("probe_output_dir")
    lidar_samples = LaunchConfiguration("lidar_samples")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate")
    lidar_range_min = LaunchConfiguration("lidar_range_min")
    lidar_range_max = LaunchConfiguration("lidar_range_max")

    scan01_frame = PythonExpression(
        ["'", robot_name, "/base_scan_01/lidar_2d_01'"]
    )
    scan02_frame = PythonExpression(
        ["'", robot_name, "/base_scan_02/lidar_2d_02'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", default_value="gazebo_eng_lobby.world"),
            DeclareLaunchArgument("robot_model_file", default_value=default_robot_model),
            DeclareLaunchArgument(
                "lidar_samples",
                default_value="360",
                description="Default positive beam count for each raw GPU LiDAR.",
            ),
            DeclareLaunchArgument(
                "lidar_update_rate",
                default_value="10.0",
                description="Default positive simulation update rate for each GPU LiDAR.",
            ),
            DeclareLaunchArgument(
                "lidar_range_min",
                default_value="0.1",
                description="Default positive minimum range in meters for each GPU LiDAR.",
            ),
            DeclareLaunchArgument(
                "lidar_range_max",
                default_value="50.0",
                description="Default maximum range in meters for each GPU LiDAR.",
            ),
            DeclareLaunchArgument("lidar_samples_01", default_value=lidar_samples),
            DeclareLaunchArgument("lidar_samples_02", default_value=lidar_samples),
            DeclareLaunchArgument(
                "lidar_update_rate_01", default_value=lidar_update_rate
            ),
            DeclareLaunchArgument(
                "lidar_update_rate_02", default_value=lidar_update_rate
            ),
            DeclareLaunchArgument("lidar_range_min_01", default_value=lidar_range_min),
            DeclareLaunchArgument("lidar_range_min_02", default_value=lidar_range_min),
            DeclareLaunchArgument("lidar_range_max_01", default_value=lidar_range_max),
            DeclareLaunchArgument("lidar_range_max_02", default_value=lidar_range_max),
            DeclareLaunchArgument(
                "lidar_runtime_model_file",
                default_value="",
                description=(
                    "Non-source SDF output path. Existing matching content is reused; "
                    "mismatched content is never overwritten."
                ),
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="mecanum730_xms5_v7_teacher_dual_scan",
            ),
            DeclareLaunchArgument("robot_x", default_value="2.0"),
            DeclareLaunchArgument("robot_y", default_value="2.0"),
            DeclareLaunchArgument("robot_z", default_value="0.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument("bridge_robot_control", default_value="true"),
            DeclareLaunchArgument("use_cmd_vel_relay", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "cmd_vel_angular_z_scale",
                default_value="1.5",
                description=(
                    "Scale applied by the ROS-to-Ignition relay after the "
                    "controller's angular command limit."
                ),
            ),
            DeclareLaunchArgument("start_merger", default_value="true"),
            DeclareLaunchArgument("merged_scan_topic", default_value="/scan_merged"),
            DeclareLaunchArgument("merged_scan_frame", default_value="base_link"),
            DeclareLaunchArgument("merged_scan_samples", default_value="360"),
            DeclareLaunchArgument("merger_sync_slop", default_value="0.05"),
            DeclareLaunchArgument("merger_queue_size", default_value="10"),
            DeclareLaunchArgument("enable_self_filter", default_value="true"),
            DeclareLaunchArgument("enable_high_fidelity_probe", default_value="false"),
            DeclareLaunchArgument("duplicate_mode", default_value="fixed_xy"),
            DeclareLaunchArgument("duplicate_epsilon", default_value="0.03"),
            DeclareLaunchArgument("publish_probe_pointclouds", default_value="true"),
            DeclareLaunchArgument("save_probe_files", default_value="false"),
            DeclareLaunchArgument("probe_output_dir", default_value=""),
            DeclareLaunchArgument("merger_params_file", default_value=default_merger_params),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("slam_scan_topic", default_value="/scan_merged"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("run", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="3"),
            DeclareLaunchArgument("robot_spawn_delay", default_value="5.0"),
            DeclareLaunchArgument(
                "gazebo_world_name",
                default_value="",
                description=(
                    "Gazebo service world name. Empty reads <world name> from world."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_scene_pedestrians",
                default_value="false",
                description="Whether to spawn XML-driven dynamic pedestrians.",
            ),
            DeclareLaunchArgument("spawn_demo_pedestrians", default_value="false"),
            DeclareLaunchArgument(
                "scene_file",
                default_value="scenarios/lobby/eng_hall_15.xml",
            ),
            DeclareLaunchArgument(
                "pedestrian_model_file",
                default_value="models/person_standing/model.sdf",
            ),
            DeclareLaunchArgument("pedestrian_use_actors", default_value="false"),
            DeclareLaunchArgument(
                "pedestrian_actor_model_file",
                default_value="models/walking_person_actor/model.sdf",
            ),
            DeclareLaunchArgument(
                "pedestrian_collision_proxy_model_file",
                default_value="models/pedestrian_collision_proxy/model.sdf",
            ),
            DeclareLaunchArgument("pedestrian_spawn_delay", default_value="4.0"),
            DeclareLaunchArgument("pedestrian_update_rate", default_value="20.0"),
            DeclareLaunchArgument(
                "pedestrian_simulation_factor", default_value="1.0"
            ),
            DeclareLaunchArgument("pedestrian_speed", default_value="1.34"),
            DeclareLaunchArgument(
                "pedestrian_count",
                default_value="-1",
                description="Total pedestrians; -1 preserves the scene XML counts.",
            ),
            DeclareLaunchArgument("pedestrian_agent_radius", default_value="0.35"),
            DeclareLaunchArgument(
                "pedestrian_static_obstacle_clearance",
                default_value="0.75",
                description=(
                    "Minimum pedestrian-center distance from scenario static obstacles, in meters."
                ),
            ),
            DeclareLaunchArgument(
                "pedestrian_relaxation_time", default_value="0.5"
            ),
            DeclareLaunchArgument("pedestrian_neighbor_range", default_value="10.0"),
            DeclareLaunchArgument(
                "pedestrian_seed",
                default_value="7",
                description="Random seed used to initialize the pedestrian scenario.",
            ),
            DeclareLaunchArgument("pedestrian_force_obstacle", default_value="10.0"),
            DeclareLaunchArgument("pedestrian_sigma_obstacle", default_value="0.2"),
            DeclareLaunchArgument("pedestrian_force_social", default_value="5.1"),
            DeclareLaunchArgument("pedestrian_enable_groups", default_value="true"),
            DeclareLaunchArgument(
                "pedestrian_group_size_lambda", default_value="1.1"
            ),
            DeclareLaunchArgument("pedestrian_force_group_gaze", default_value="3.0"),
            DeclareLaunchArgument(
                "pedestrian_force_group_coherence", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_repulsion", default_value="1.0"
            ),
            DeclareLaunchArgument("pedestrian_force_random", default_value="0.1"),
            DeclareLaunchArgument("pedestrian_force_along_wall", default_value="2.0"),

            OpaqueFunction(function=_prepare_runtime_lidar_model),

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
                    "robot_name": robot_name,
                    "robot_x": LaunchConfiguration("robot_x"),
                    "robot_y": LaunchConfiguration("robot_y"),
                    "robot_z": LaunchConfiguration("robot_z"),
                    "robot_yaw": LaunchConfiguration("robot_yaw"),
                    "robot_spawn_delay": LaunchConfiguration("robot_spawn_delay"),
                    "gazebo_world_name": LaunchConfiguration("gazebo_world_name"),
                    "bridge_clock": "true",
                    "bridge_robot_control": LaunchConfiguration(
                        "bridge_robot_control"
                    ),
                    "use_cmd_vel_relay": LaunchConfiguration(
                        "use_cmd_vel_relay"
                    ),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "cmd_vel_angular_z_scale": LaunchConfiguration(
                        "cmd_vel_angular_z_scale"
                    ),
                    "bridge_lidar": "false",
                    "publish_lidar_static_tf": "false",
                    "spawn_scene_pedestrians": LaunchConfiguration(
                        "spawn_scene_pedestrians"
                    ),
                    "spawn_demo_pedestrians": LaunchConfiguration(
                        "spawn_demo_pedestrians"
                    ),
                    "scene_file": LaunchConfiguration("scene_file"),
                    "pedestrian_model_file": LaunchConfiguration(
                        "pedestrian_model_file"
                    ),
                    "pedestrian_use_actors": LaunchConfiguration(
                        "pedestrian_use_actors"
                    ),
                    "pedestrian_actor_model_file": LaunchConfiguration(
                        "pedestrian_actor_model_file"
                    ),
                    "pedestrian_collision_proxy_model_file": LaunchConfiguration(
                        "pedestrian_collision_proxy_model_file"
                    ),
                    "pedestrian_spawn_delay": LaunchConfiguration(
                        "pedestrian_spawn_delay"
                    ),
                    "pedestrian_update_rate": LaunchConfiguration(
                        "pedestrian_update_rate"
                    ),
                    "pedestrian_simulation_factor": LaunchConfiguration(
                        "pedestrian_simulation_factor"
                    ),
                    "pedestrian_speed": LaunchConfiguration("pedestrian_speed"),
                    "pedestrian_count": LaunchConfiguration("pedestrian_count"),
                    "pedestrian_agent_radius": LaunchConfiguration(
                        "pedestrian_agent_radius"
                    ),
                    "pedestrian_static_obstacle_clearance": LaunchConfiguration(
                        "pedestrian_static_obstacle_clearance"
                    ),
                    "pedestrian_relaxation_time": LaunchConfiguration(
                        "pedestrian_relaxation_time"
                    ),
                    "pedestrian_neighbor_range": LaunchConfiguration(
                        "pedestrian_neighbor_range"
                    ),
                    "pedestrian_seed": LaunchConfiguration("pedestrian_seed"),
                    "pedestrian_force_obstacle": LaunchConfiguration(
                        "pedestrian_force_obstacle"
                    ),
                    "pedestrian_sigma_obstacle": LaunchConfiguration(
                        "pedestrian_sigma_obstacle"
                    ),
                    "pedestrian_force_social": LaunchConfiguration(
                        "pedestrian_force_social"
                    ),
                    "pedestrian_enable_groups": LaunchConfiguration(
                        "pedestrian_enable_groups"
                    ),
                    "pedestrian_group_size_lambda": LaunchConfiguration(
                        "pedestrian_group_size_lambda"
                    ),
                    "pedestrian_force_group_gaze": LaunchConfiguration(
                        "pedestrian_force_group_gaze"
                    ),
                    "pedestrian_force_group_coherence": LaunchConfiguration(
                        "pedestrian_force_group_coherence"
                    ),
                    "pedestrian_force_group_repulsion": LaunchConfiguration(
                        "pedestrian_force_group_repulsion"
                    ),
                    "pedestrian_force_random": LaunchConfiguration(
                        "pedestrian_force_random"
                    ),
                    "pedestrian_force_along_wall": LaunchConfiguration(
                        "pedestrian_force_along_wall"
                    ),
                }.items(),
            ),

            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="v7_dual_lidar_bridge",
                output="screen",
                arguments=[
                    "/scan_01@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/scan_02@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                ],
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="scan01_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x", "0.2",
                    "--y", "0.13",
                    "--z", "0.208",
                    "--roll", "3.14",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_link",
                    "--child-frame-id", scan01_frame,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="scan02_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x", "-0.2",
                    "--y", "-0.13",
                    "--z", "0.208",
                    "--roll", "3.14",
                    "--pitch", "0",
                    "--yaw", "3.14",
                    "--frame-id", "base_link",
                    "--child-frame-id", scan02_frame,
                ],
            ),

            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        condition=IfCondition(LaunchConfiguration("start_merger")),
                        package="semantic_nav_gazebo",
                        executable="v7_dual_laser_scan_merger.py",
                        name="v7_dual_laser_scan_merger",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("merger_params_file"),
                            {
                                "output_topic": merged_scan_topic,
                                "output_frame": merged_scan_frame,
                                "output_samples": ParameterValue(
                                    merged_scan_samples,
                                    value_type=int,
                                ),
                                "sync_slop": ParameterValue(
                                    merger_sync_slop,
                                    value_type=float,
                                ),
                                "queue_size": ParameterValue(
                                    merger_queue_size,
                                    value_type=int,
                                ),
                                "enable_self_filter": ParameterValue(
                                    enable_self_filter,
                                    value_type=bool,
                                ),
                                "enable_high_fidelity_probe": ParameterValue(
                                    enable_high_fidelity_probe,
                                    value_type=bool,
                                ),
                                "duplicate_mode": duplicate_mode,
                                "duplicate_epsilon": ParameterValue(
                                    duplicate_epsilon,
                                    value_type=float,
                                ),
                                "publish_probe_pointclouds": ParameterValue(
                                    publish_probe_pointclouds,
                                    value_type=bool,
                                ),
                                "save_probe_files": ParameterValue(
                                    save_probe_files,
                                    value_type=bool,
                                ),
                                "probe_output_dir": probe_output_dir,
                            },
                        ],
                    ),
                ],
            ),

            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        condition=IfCondition(LaunchConfiguration("start_slam")),
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("slam_params_file"),
                            {
                                "use_sim_time": True,
                                "odom_frame": "odom",
                                "map_frame": "map",
                                "base_frame": "base_link",
                                "scan_topic": slam_scan_topic,
                            },
                        ],
                    ),
                ],
            ),

            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        condition=IfCondition(LaunchConfiguration("start_rviz")),
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_v7_dual_slam",
                        output="screen",
                        arguments=[
                            "-d",
                            LaunchConfiguration("rviz_config"),
                        ],
                        parameters=[
                            {
                                "use_sim_time": True,
                            },
                        ],
                    ),
                ],
            ),
        ]
    )
