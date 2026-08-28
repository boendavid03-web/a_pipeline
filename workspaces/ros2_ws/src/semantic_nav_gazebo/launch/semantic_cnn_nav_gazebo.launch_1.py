#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--child-frame-id, --frame-id, --pitch, --roll, --x, --y, --yaw, --z
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /cmd_vel, /odom, /scan
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：SDF, WORLD
# 可能使用的关键环境变量：IGN_GAZEBO_RESOURCE_PATH, IGN_GAZEBO_SYSTEM_PLUGIN_PATH, NAVIGATION_PROJECT_ROOT, URDF
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo semantic_cnn_nav_gazebo.launch_1.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.533297111 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.851391683 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo_lidar2d.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo_lidar2d.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo_lidar2d.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_v7_dual_bringup.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜semantic_cnn_nav_gazebo.launch_1.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
# -*- coding: utf-8 -*-

import os
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def navigation_project_root():
    env_root = os.environ.get("NAVIGATION_PROJECT_ROOT")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[5]


def _as_bool(value):
    """
    将常见的 true / false 风格字符串转换为 bool。
    """
    if isinstance(value, bool):
        return value

    value = str(value).strip().lower()

    if value in ("true", "1", "yes", "y", "on"):
        return True

    if value in ("false", "0", "no", "n", "off"):
        return False

    raise ValueError(f"无法识别的布尔值：{value}")


def _get(context, name):
    """
    读取 launch 参数的实际值。
    """
    return LaunchConfiguration(name).perform(context)


def _resolve_path(path_value, package_share, default_subdir=None):
    """
    解析资源路径。

    - 绝对路径：直接使用；
    - 相对 world 路径：默认相对于 package_share/worlds；
    - 其他相对路径：默认相对于 package_share。
    """
    path_value = os.path.expanduser(path_value)

    if os.path.isabs(path_value):
        return os.path.normpath(path_value)

    if default_subdir:
        return os.path.normpath(
            os.path.join(package_share, default_subdir, path_value)
        )

    return os.path.normpath(os.path.join(package_share, path_value))


def _world_name_from_sdf(world_path):
    """Return the single Gazebo world name declared by an SDF world file."""
    try:
        root = ET.parse(world_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"无法读取 Gazebo world 名称：{world_path}: {exc}") from exc

    world = root.find("world")
    world_name = "" if world is None else world.attrib.get("name", "").strip()
    if not world_name:
        raise ValueError(
            f"Gazebo world 文件缺少 <world name=...>：{world_path}"
        )
    return world_name


def _find_missing_local_urdf_meshes(urdf_file):
    """
    检查 URDF 中直接引用的本地 mesh 文件是否存在。

    仅检查：
    - 相对路径；
    - 绝对路径；
    - file:// 路径。

    package:// 和 model:// URI 依赖外部 ROS / Gazebo 资源路径，
    此处不强行判断是否缺失。
    """
    missing_meshes = []

    try:
        root = ET.parse(urdf_file).getroot()
    except (ET.ParseError, OSError):
        return missing_meshes

    urdf_dir = os.path.dirname(os.path.abspath(urdf_file))

    for element in root.iter():
        if not element.tag.endswith("mesh"):
            continue

        mesh_ref = (
            element.attrib.get("filename")
            or element.attrib.get("url")
            or element.attrib.get("uri")
        )

        if not mesh_ref:
            continue

        mesh_ref = os.path.expanduser(mesh_ref.strip())

        if mesh_ref.startswith(("package://", "model://")):
            continue

        if mesh_ref.startswith("file://"):
            mesh_path = mesh_ref[len("file://"):]
        elif os.path.isabs(mesh_ref):
            mesh_path = mesh_ref
        else:
            mesh_path = os.path.join(urdf_dir, mesh_ref)

        mesh_path = os.path.normpath(mesh_path)

        if not os.path.isfile(mesh_path):
            missing_meshes.append(mesh_path)

    return sorted(set(missing_meshes))


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("semantic_nav_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    system_plugin_dir = str(
        Path(package_share).parents[1] / "lib" / "semantic_nav_gazebo"
    )
    model_resource_dir = os.path.join(package_share, "models")

    # ============================================================
    # Gazebo 世界参数
    # ============================================================
    world = _get(context, "world")
    gui = _as_bool(_get(context, "gui"))
    run = _as_bool(_get(context, "run"))
    verbose = _get(context, "verbose")
    gz_version = _get(context, "gz_version")
    gazebo_world_name = _get(context, "gazebo_world_name")

    world_path = _resolve_path(
        world,
        package_share,
        default_subdir="worlds",
    )

    if not os.path.isfile(world_path):
        raise FileNotFoundError(
            f"Gazebo world 文件不存在：{world_path}"
        )

    # Keep the service world aligned with the selected world file. Users can
    # still pass gazebo_world_name explicitly for unusual multi-world files.
    if not gazebo_world_name.strip():
        gazebo_world_name = _world_name_from_sdf(world_path)

    gz_args = []

    if run:
        gz_args.append("-r")

    if not gui:
        gz_args.append("-s")

    if verbose:
        gz_args.extend(["-v", str(verbose)])

    gz_args.append(world_path)

    actions = [
        SetEnvironmentVariable(
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            [
                system_plugin_dir,
                ":",
                EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                model_resource_dir,
                ":",
                EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    ros_gz_sim_share,
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments={
                "gz_args": " ".join(gz_args),
                "gz_version": gz_version,
            }.items(),
        )
    ]

    # ============================================================
    # 可选机器人生成
    # ============================================================
    spawn_robot = _as_bool(_get(context, "spawn_robot"))

    if spawn_robot:
        robot_model_type = _get(context, "robot_model_type").strip().lower()
        robot_model_file = _get(context, "robot_model_file")
        robot_urdf_file = _get(context, "robot_urdf_file")

        robot_name = _get(context, "robot_name")
        robot_x = _get(context, "robot_x")
        robot_y = _get(context, "robot_y")
        robot_z = _get(context, "robot_z")
        robot_yaw = _get(context, "robot_yaw")
        robot_spawn_delay = float(_get(context, "robot_spawn_delay"))

        if robot_model_type == "sdf":
            selected_robot_file = _resolve_path(
                robot_model_file,
                package_share,
            )

        elif robot_model_type == "urdf":
            selected_robot_file = _resolve_path(
                robot_urdf_file,
                package_share,
            )

        else:
            raise ValueError(
                "robot_model_type 参数错误："
                f"{robot_model_type!r}。"
                "仅支持 sdf 或 urdf。"
            )

        if not os.path.isfile(selected_robot_file):
            raise FileNotFoundError(
                f"机器人模型文件不存在：{selected_robot_file}"
            )

        actions.append(
            LogInfo(
                msg=(
                    "[semantic_nav_gazebo] 即将生成机器人："
                    f"type={robot_model_type}, "
                    f"name={robot_name}, "
                    f"file={selected_robot_file}"
                )
            )
        )

        if robot_model_type == "urdf":
            missing_meshes = _find_missing_local_urdf_meshes(
                selected_robot_file
            )

            if missing_meshes:
                preview = "\n    - ".join(missing_meshes[:20])

                if len(missing_meshes) > 20:
                    preview += (
                        f"\n    ... 另外还有 "
                        f"{len(missing_meshes) - 20} 个缺失文件"
                    )

                actions.append(
                    LogInfo(
                        msg=(
                            "\n"
                            "[semantic_nav_gazebo] 警告："
                            f"URDF 中有 {len(missing_meshes)} 个本地 mesh 文件不存在。\n"
                            "Gazebo Sim 仍会尝试生成机器人，"
                            "但外观、碰撞体或部分机械臂结构可能显示不完整。\n"
                            f"    - {preview}"
                        )
                    )
                )

        # 延迟生成，避免 Gazebo 服务尚未初始化导致 create 请求超时。
        actions.append(
            TimerAction(
                period=robot_spawn_delay,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_robot",
                        output="screen",
                        arguments=[
                            "-world",
                            gazebo_world_name,
                            "-file",
                            selected_robot_file,
                            "-name",
                            robot_name,
                            "-x",
                            robot_x,
                            "-y",
                            robot_y,
                            "-z",
                            robot_z,
                            "-Y",
                            robot_yaw,
                            "-allow_renaming=false",
                        ],
                    )
                ],
            )
        )

    # ============================================================
    # 可选 ROS 2 <-> Gazebo 话题桥接
    # ============================================================
    bridge_arguments = []

    if _as_bool(_get(context, "bridge_robot_control")):
        cmd_vel_topic = _get(context, "cmd_vel_topic")
        odom_topic = _get(context, "odom_topic")
        use_cmd_vel_relay = _as_bool(_get(context, "use_cmd_vel_relay"))

        if use_cmd_vel_relay:
            cmd_vel_angular_z_scale = float(
                _get(context, "cmd_vel_angular_z_scale")
            )

            actions.append(
                Node(
                    package="semantic_nav_gazebo",
                    executable="cmd_vel_ign_relay",
                    name="cmd_vel_ign_relay",
                    output="screen",
                    parameters=[
                        {
                            "ros_topic": cmd_vel_topic,
                            "ign_topic": cmd_vel_topic,
                            "angular_z_scale": cmd_vel_angular_z_scale,
                        }
                    ],
                )
            )
        else:
            bridge_arguments.append(
                f"{cmd_vel_topic}@geometry_msgs/msg/Twist]ignition.msgs.Twist"
            )

        bridge_arguments.append(
            f"{odom_topic}@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        )

        bridge_arguments.append(
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        )

    if _as_bool(_get(context, "bridge_clock")):
        clock_topic = _get(context, "clock_topic")

        bridge_arguments.append(
            f"{clock_topic}@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        )

    if _as_bool(_get(context, "bridge_lidar")):
        lidar_topic = _get(context, "lidar_topic")

        bridge_arguments.append(
            f"{lidar_topic}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"
        )

    if bridge_arguments:
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_parameter_bridge",
                output="screen",
                arguments=bridge_arguments,
            )
        )

    # ============================================================
    # 可选 LiDAR 静态坐标变换发布器
    # ============================================================
    if _as_bool(_get(context, "publish_lidar_static_tf")):
        lidar_static_parent_frame = _get(context, "lidar_static_parent_frame")
        lidar_static_child_frame = _get(context, "lidar_static_child_frame")

        lidar_static_xyz_str = _get(context, "lidar_static_xyz")
        lidar_static_xyz_parts = lidar_static_xyz_str.strip().split()
        if len(lidar_static_xyz_parts) != 3:
            raise ValueError(
                "lidar_static_xyz 必须包含恰好 3 个数值（空格分隔），"
                f"实际得到 {len(lidar_static_xyz_parts)} 个："
                f"{lidar_static_xyz_str!r}"
            )
        lidar_static_x, lidar_static_y, lidar_static_z = lidar_static_xyz_parts

        lidar_static_rpy_str = _get(context, "lidar_static_rpy")
        lidar_static_rpy_parts = lidar_static_rpy_str.strip().split()
        if len(lidar_static_rpy_parts) != 3:
            raise ValueError(
                "lidar_static_rpy 必须包含恰好 3 个数值（空格分隔），"
                f"实际得到 {len(lidar_static_rpy_parts)} 个："
                f"{lidar_static_rpy_str!r}"
            )
        lidar_static_roll, lidar_static_pitch, lidar_static_yaw = lidar_static_rpy_parts

        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x", lidar_static_x,
                    "--y", lidar_static_y,
                    "--z", lidar_static_z,
                    "--roll", lidar_static_roll,
                    "--pitch", lidar_static_pitch,
                    "--yaw", lidar_static_yaw,
                    "--frame-id", lidar_static_parent_frame,
                    "--child-frame-id", lidar_static_child_frame,
                ],
            )
        )

    # ============================================================
    # 可选动态行人
    # ============================================================
    spawn_scene_pedestrians = _as_bool(
        _get(context, "spawn_scene_pedestrians")
    )

    spawn_demo_pedestrians = _as_bool(
        _get(context, "spawn_demo_pedestrians")
    )

    if spawn_scene_pedestrians or spawn_demo_pedestrians:
        scene_path = _resolve_path(
            _get(context, "scene_file"),
            package_share,
        )

        pedestrian_model_path = _resolve_path(
            _get(context, "pedestrian_model_file"),
            package_share,
        )
        pedestrian_use_actors = _as_bool(
            _get(context, "pedestrian_use_actors")
        )
        actor_model_path = _resolve_path(
            _get(context, "pedestrian_actor_model_file"),
            package_share,
        )
        collision_proxy_model_path = _resolve_path(
            _get(context, "pedestrian_collision_proxy_model_file"),
            package_share,
        )

        if not os.path.isfile(scene_path):
            raise FileNotFoundError(
                f"行人场景文件不存在：{scene_path}"
            )

        if not os.path.isfile(pedestrian_model_path):
            raise FileNotFoundError(
                f"行人模型文件不存在：{pedestrian_model_path}"
            )

        if pedestrian_use_actors and not os.path.isfile(actor_model_path):
            raise FileNotFoundError(
                f"Actor 行人模型文件不存在：{actor_model_path}"
            )

        if pedestrian_use_actors and not os.path.isfile(collision_proxy_model_path):
            raise FileNotFoundError(
                f"行人碰撞代理文件不存在：{collision_proxy_model_path}"
            )

        actions.append(
            Node(
                package="semantic_nav_gazebo",
                executable="pedestrian_actor_pose_bridge",
                name="pedestrian_actor_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "world_name": gazebo_world_name,
                        "use_actors": pedestrian_use_actors,
                    }
                ],
            )
        )

        actions.append(
            Node(
                package="semantic_nav_gazebo",
                executable="scenario_pedestrian_controller.py",
                name="scenario_pedestrian_controller",
                output="screen",
                parameters=[
                    {
                        "world_name": gazebo_world_name,
                        "clock_topic": _get(context, "clock_topic"),
                        "scene_file": scene_path,
                        "model_file": pedestrian_model_path,
                        "use_actors": pedestrian_use_actors,
                        "use_pose_bridge": True,
                        "use_sim_time": True,
                        "actor_model_file": actor_model_path,
                        "collision_proxy_model_file": collision_proxy_model_path,
                        "spawn_delay": float(
                            _get(context, "pedestrian_spawn_delay")
                        ),
                        "update_rate": float(
                            _get(context, "pedestrian_update_rate")
                        ),
                        "simulation_factor": float(
                            _get(context, "pedestrian_simulation_factor")
                        ),
                        "speed": float(
                            _get(context, "pedestrian_speed")
                        ),
                        "pedestrian_count": int(
                            _get(context, "pedestrian_count")
                        ),
                        "agent_radius": float(
                            _get(context, "pedestrian_agent_radius")
                        ),
                        "static_obstacle_clearance": float(
                            _get(context, "pedestrian_static_obstacle_clearance")
                        ),
                        "relaxation_time": float(
                            _get(context, "pedestrian_relaxation_time")
                        ),
                        "neighbor_range": float(
                            _get(context, "pedestrian_neighbor_range")
                        ),
                        "seed": int(_get(context, "pedestrian_seed")),
                        "force_obstacle": float(
                            _get(context, "pedestrian_force_obstacle")
                        ),
                        "sigma_obstacle": float(
                            _get(context, "pedestrian_sigma_obstacle")
                        ),
                        "force_social": float(
                            _get(context, "pedestrian_force_social")
                        ),
                        "enable_groups": _as_bool(
                            _get(context, "pedestrian_enable_groups")
                        ),
                        "group_size_lambda": float(
                            _get(context, "pedestrian_group_size_lambda")
                        ),
                        "force_group_gaze": float(
                            _get(context, "pedestrian_force_group_gaze")
                        ),
                        "force_group_coherence": float(
                            _get(context, "pedestrian_force_group_coherence")
                        ),
                        "force_group_repulsion": float(
                            _get(context, "pedestrian_force_group_repulsion")
                        ),
                        "force_random": float(
                            _get(context, "pedestrian_force_random")
                        ),
                        "force_along_wall": float(
                            _get(context, "pedestrian_force_along_wall")
                        ),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # Gazebo 世界
            DeclareLaunchArgument(
                "world",
                default_value="gazebo_eng_lobby.world",
                description=(
                    "worlds 目录下的 world 文件名，或绝对路径。"
                ),
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="是否显示 Gazebo GUI。",
            ),
            DeclareLaunchArgument(
                "run",
                default_value="true",
                description="是否直接运行仿真。",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="3",
                description="Gazebo 日志详细级别。",
            ),
            DeclareLaunchArgument(
                "gz_version",
                default_value="6",
                description="ROS 2 Humble 对应的 Gazebo Fortress 版本。",
            ),
            DeclareLaunchArgument(
                "gazebo_world_name",
                default_value="",
                description=(
                    "Gazebo world 服务名称。留空时从所选 world 文件的 "
                    "<world name=...> 自动读取。"
                ),
            ),

            # 机器人
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="false",
                description="是否生成机器人。",
            ),
            DeclareLaunchArgument(
                "robot_model_type",
                default_value="sdf",
                description="机器人模型类型：sdf 或 urdf。",
            ),
            DeclareLaunchArgument(
                "robot_model_file",
                default_value="models/kobuki_hexagons_hokuyo/model.sdf",
                description=(
                    "robot_model_type:=sdf 时使用的机器人 SDF 文件。"
                    "相对路径以 semantic_nav_gazebo 包目录为基准。"
                ),
            ),
            DeclareLaunchArgument(
                "robot_urdf_file",
                default_value=str(
                    navigation_project_root()
                    / "assets"
                    / "robots"
                    / "mecanum_v7"
                    / "motion_wheel_arm_simple_sphere_urdf"
                    / "mecanum730_xms5_default.urdf"
                ),
                description=(
                    "robot_model_type:=urdf 时使用的机器人 URDF 文件。"
                ),
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="kobuki_hexagons_hokuyo",
                description="生成后的机器人实体名称。",
            ),
            DeclareLaunchArgument(
                "robot_x",
                default_value="0.0",
                description="机器人初始 X 坐标。",
            ),
            DeclareLaunchArgument(
                "robot_y",
                default_value="0.0",
                description="机器人初始 Y 坐标。",
            ),
            DeclareLaunchArgument(
                "robot_z",
                default_value="0.0",
                description="机器人初始 Z 坐标。",
            ),
            DeclareLaunchArgument(
                "robot_yaw",
                default_value="0.0",
                description="机器人初始 yaw 角，单位为弧度。",
            ),
            DeclareLaunchArgument(
                "robot_spawn_delay",
                default_value="5.0",
                description="启动 Gazebo 后延迟生成机器人的秒数。",
            ),

            # 桥接
            DeclareLaunchArgument(
                "bridge_clock",
                default_value="false",
                description="是否将 Gazebo /clock 单向桥接到 ROS 2。",
            ),
            DeclareLaunchArgument(
                "clock_topic",
                default_value="/clock",
                description="仿真时钟话题。",
            ),
            DeclareLaunchArgument(
                "bridge_lidar",
                default_value="false",
                description="是否将 Gazebo LiDAR LaserScan 单向桥接到 ROS 2。",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/scan",
                description="LiDAR 话题。",
            ),
            DeclareLaunchArgument(
                "bridge_robot_control",
                default_value="false",
                description="是否桥接 cmd_vel(ROS->Gazebo) 和 odom(Gazebo->ROS)。",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_relay",
                default_value="true",
                description=(
                    "bridge_robot_control:=true 时，是否使用本包的 "
                    "cmd_vel_ign_relay 将 ROS 2 Twist 转发为 Ignition Twist。"
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_angular_z_scale",
                default_value="1.5",
                description=(
                    "cmd_vel_ign_relay 的角速度标定系数；仅用于当前导航代理。"
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="机器人速度控制话题。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="机器人里程计话题。",
            ),

            # 动态行人
            DeclareLaunchArgument(
                "spawn_scene_pedestrians",
                default_value="false",
                description="是否根据场景文件生成动态行人。",
            ),
            DeclareLaunchArgument(
                "spawn_demo_pedestrians",
                default_value="false",
                description="spawn_scene_pedestrians 的兼容别名。",
            ),
            DeclareLaunchArgument(
                "scene_file",
                default_value="scenarios/lobby/eng_hall_15.xml",
                description="动态行人场景文件。",
            ),
            DeclareLaunchArgument(
                "pedestrian_model_file",
                default_value="models/person_standing/model.sdf",
                description="动态行人模型文件。",
            ),
            DeclareLaunchArgument(
                "pedestrian_use_actors",
                default_value="false",
                description="是否使用实验性的 Gazebo Actor；默认保留已验证的普通人形模型。",
            ),
            DeclareLaunchArgument(
                "pedestrian_actor_model_file",
                default_value="models/walking_person_actor/model.sdf",
            ),
            DeclareLaunchArgument(
                "pedestrian_collision_proxy_model_file",
                default_value="models/pedestrian_collision_proxy/model.sdf",
            ),
            DeclareLaunchArgument(
                "pedestrian_spawn_delay",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_update_rate",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_simulation_factor",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_speed",
                default_value="1.34",
            ),
            DeclareLaunchArgument(
                "pedestrian_count",
                default_value="-1",
                description=(
                    "Total pedestrians; -1 preserves scene XML counts, 0 disables them."
                ),
            ),
            DeclareLaunchArgument(
                "pedestrian_agent_radius",
                default_value="0.35",
            ),
            DeclareLaunchArgument(
                "pedestrian_static_obstacle_clearance",
                default_value="0.75",
                description=(
                    "Minimum pedestrian-center distance from scenario static obstacles, in meters."
                ),
            ),
            DeclareLaunchArgument(
                "pedestrian_relaxation_time",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "pedestrian_neighbor_range",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_seed",
                default_value="7",
                description="Random seed used to initialize the pedestrian scenario.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_obstacle",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_sigma_obstacle",
                default_value="0.2",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_social",
                default_value="5.1",
            ),
            DeclareLaunchArgument(
                "pedestrian_enable_groups",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "pedestrian_group_size_lambda",
                default_value="1.1",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_gaze",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_coherence",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_repulsion",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_random",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_along_wall",
                default_value="2.0",
            ),

            # LiDAR 静态坐标变换
            DeclareLaunchArgument(
                "publish_lidar_static_tf",
                default_value="true",
                description="是否发布 LiDAR 相对 base_link 的静态 TF。",
            ),
            DeclareLaunchArgument(
                "lidar_static_parent_frame",
                default_value="base_link",
                description="LiDAR 静态 TF 的父坐标系。",
            ),
            DeclareLaunchArgument(
                "lidar_static_child_frame",
                default_value="mecanum730_xms5_lidar2d/lidar_2d_link/lidar_2d",
                description="LiDAR 静态 TF 的子坐标系。",
            ),
            DeclareLaunchArgument(
                "lidar_static_xyz",
                default_value="0.303 0.12 0.995",
                description="LiDAR 静态 TF 的平移分量（空格分隔的 x y z）。",
            ),
            DeclareLaunchArgument(
                "lidar_static_rpy",
                default_value="0 0 0",
                description="LiDAR 静态 TF 的旋转分量（空格分隔的 roll pitch yaw）。",
            ),

            OpaqueFunction(function=_launch_setup),
        ]
    )
