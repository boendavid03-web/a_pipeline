# 【脚本说明｜semantic_cnn_nav_gazebo.launch.py】
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：SDF, WORLD
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 launch 启动入口
# 推荐运行方式：ros2 launch semantic_nav_gazebo semantic_cnn_nav_gazebo.launch.py
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
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 用途：ROS 2 launch 启动入口，配置并启动基础 Gazebo/导航仿真场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游节点；同目录其他 launch 是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("semantic_nav_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world):
        world = os.path.join(package_share, "worlds", world)

    gz_args = []
    if _as_bool(LaunchConfiguration("run").perform(context)):
        gz_args.append("-r")
    if not _as_bool(LaunchConfiguration("gui").perform(context)):
        gz_args.append("-s")

    verbose = LaunchConfiguration("verbose").perform(context)
    if verbose:
        gz_args.extend(["-v", verbose])

    gz_args.append(world)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": " ".join(gz_args),
                "gz_version": LaunchConfiguration("gz_version"),
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]

    gazebo_world_name = LaunchConfiguration("gazebo_world_name").perform(context)

    if _as_bool(LaunchConfiguration("spawn_robot").perform(context)):
        robot_model_file = LaunchConfiguration("robot_model_file").perform(context)
        if not os.path.isabs(robot_model_file):
            robot_model_file = os.path.join(package_share, robot_model_file)
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot_entity",
                output="screen",
                arguments=[
                    "-world",
                    gazebo_world_name,
                    "-file",
                    robot_model_file,
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-x",
                    LaunchConfiguration("robot_x"),
                    "-y",
                    LaunchConfiguration("robot_y"),
                    "-z",
                    LaunchConfiguration("robot_z"),
                    "-Y",
                    LaunchConfiguration("robot_yaw"),
                    "-allow_renaming=false",
                ],
            )
        )

    if _as_bool(LaunchConfiguration("bridge_robot_control").perform(context)):
        cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
        odom_topic = LaunchConfiguration("odom_topic").perform(context)
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robot_control_bridge",
                output="screen",
                arguments=[
                    f"{cmd_vel_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    f"{odom_topic}@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                ],
            )
        )

    if _as_bool(LaunchConfiguration("bridge_lidar").perform(context)):
        lidar_topic = LaunchConfiguration("lidar_topic").perform(context)
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="lidar_scan_bridge",
                output="screen",
                arguments=[f"{lidar_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
            )
        )

    spawn_pedestrians = _as_bool(
        LaunchConfiguration("spawn_scene_pedestrians").perform(context)
    ) or _as_bool(LaunchConfiguration("spawn_demo_pedestrians").perform(context))

    if spawn_pedestrians:
        model_file = LaunchConfiguration("pedestrian_model_file").perform(context)
        if not os.path.isabs(model_file):
            model_file = os.path.join(package_share, model_file)
        scene_file = LaunchConfiguration("scene_file").perform(context)
        if not os.path.isabs(scene_file):
            scene_file = os.path.join(package_share, scene_file)
        actions.append(
            Node(
                package="semantic_nav_gazebo",
                executable="scenario_pedestrian_controller.py",
                name="scenario_pedestrian_controller",
                output="screen",
                parameters=[
                    {
                        "world_name": gazebo_world_name,
                        "clock_topic": LaunchConfiguration("clock_topic"),
                        "model_file": model_file,
                        "scene_file": scene_file,
                        "spawn_delay": float(
                            LaunchConfiguration("pedestrian_spawn_delay").perform(context)
                        ),
                        "update_rate": float(
                            LaunchConfiguration("pedestrian_update_rate").perform(context)
                        ),
                        "simulation_factor": float(
                            LaunchConfiguration("pedestrian_simulation_factor").perform(context)
                        ),
                        "speed": float(LaunchConfiguration("pedestrian_speed").perform(context)),
                        "agent_radius": float(
                            LaunchConfiguration("pedestrian_agent_radius").perform(context)
                        ),
                        "force_obstacle": float(
                            LaunchConfiguration("pedestrian_force_obstacle").perform(context)
                        ),
                        "sigma_obstacle": float(
                            LaunchConfiguration("pedestrian_sigma_obstacle").perform(context)
                        ),
                        "force_social": float(
                            LaunchConfiguration("pedestrian_force_social").perform(context)
                        ),
                        "enable_groups": _as_bool(
                            LaunchConfiguration("pedestrian_enable_groups").perform(context)
                        ),
                        "group_size_lambda": float(
                            LaunchConfiguration("pedestrian_group_size_lambda").perform(context)
                        ),
                        "force_group_gaze": float(
                            LaunchConfiguration("pedestrian_force_group_gaze").perform(context)
                        ),
                        "force_group_coherence": float(
                            LaunchConfiguration("pedestrian_force_group_coherence").perform(
                                context
                            )
                        ),
                        "force_group_repulsion": float(
                            LaunchConfiguration("pedestrian_force_group_repulsion").perform(
                                context
                            )
                        ),
                        "force_random": float(
                            LaunchConfiguration("pedestrian_force_random").perform(context)
                        ),
                        "force_along_wall": float(
                            LaunchConfiguration("pedestrian_force_along_wall").perform(context)
                        ),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="gazebo_eng_lobby.world",
                description="World filename under semantic_nav_gazebo/worlds or an absolute path.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo Sim with the GUI.",
            ),
            DeclareLaunchArgument(
                "run",
                default_value="true",
                description="Start simulation unpaused.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="3",
                description="Gazebo Sim verbosity level.",
            ),
            DeclareLaunchArgument(
                "gz_version",
                default_value="6",
                description="Gazebo Sim major version. ROS 2 Humble commonly uses Fortress / version 6.",
            ),
            DeclareLaunchArgument(
                "gazebo_world_name",
                default_value="default",
                description="SDF world name used by Gazebo Sim services.",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="false",
                description="Spawn the primitive placeholder robot model.",
            ),
            DeclareLaunchArgument(
                "robot_model_file",
                default_value="models/kobuki_hexagons_hokuyo/model.sdf",
                description="Robot SDF model path, relative to semantic_nav_gazebo share or absolute.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="kobuki_hexagons_hokuyo",
                description="Name used for the spawned robot entity.",
            ),
            DeclareLaunchArgument(
                "robot_x",
                default_value="0.0",
                description="Robot spawn x position in meters.",
            ),
            DeclareLaunchArgument(
                "robot_y",
                default_value="0.0",
                description="Robot spawn y position in meters.",
            ),
            DeclareLaunchArgument(
                "robot_z",
                default_value="0.0",
                description="Robot spawn z position in meters.",
            ),
            DeclareLaunchArgument(
                "robot_yaw",
                default_value="0.0",
                description="Robot spawn yaw in radians.",
            ),
            DeclareLaunchArgument(
                "bridge_lidar",
                default_value="false",
                description="Bridge the Gazebo LiDAR scan topic into ROS 2.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/scan",
                description="Gazebo and ROS 2 topic name used for the robot LiDAR scan.",
            ),
            DeclareLaunchArgument(
                "bridge_robot_control",
                default_value="false",
                description="Bridge ROS 2 /cmd_vel to Gazebo and Gazebo /odom back to ROS 2.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Gazebo and ROS 2 velocity command topic for the robot.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Gazebo and ROS 2 odometry topic for the robot.",
            ),
            DeclareLaunchArgument(
                "scene_file",
                default_value="scenarios/lobby/eng_hall_15.xml",
                description="Pedestrian scenario XML path, relative to semantic_nav_gazebo share or absolute.",
            ),
            DeclareLaunchArgument(
                "pedestrian_model_file",
                default_value="models/person_standing/model.sdf",
                description="Pedestrian SDF model path, relative to semantic_nav_gazebo share or absolute.",
            ),
            DeclareLaunchArgument(
                "spawn_scene_pedestrians",
                default_value="false",
                description="Spawn pedestrians from the scenario XML and move them through addwaypoint targets.",
            ),
            DeclareLaunchArgument(
                "spawn_demo_pedestrians",
                default_value="false",
                description="Compatibility alias for spawn_scene_pedestrians.",
            ),
            DeclareLaunchArgument(
                "pedestrian_spawn_delay",
                default_value="4.0",
                description="Seconds to wait before spawning demo pedestrians.",
            ),
            DeclareLaunchArgument(
                "pedestrian_update_rate",
                default_value="20.0",
                description="Pedestrian pose and ground-truth update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "pedestrian_simulation_factor",
                default_value="1.0",
                description="Pedsim simulation factor; 1.0 matches ROS 1 default timing.",
            ),
            DeclareLaunchArgument(
                "pedestrian_speed",
                default_value="1.34",
                description="Mean pedestrian walking speed in m/s, matching libpedsim adult default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_agent_radius",
                default_value="0.35",
                description="Pedestrian radius used by the social-force and wall projection model.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_obstacle",
                default_value="10.0",
                description="Obstacle-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_sigma_obstacle",
                default_value="0.2",
                description="Obstacle-force sigma matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_social",
                default_value="5.1",
                description="Social-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_enable_groups",
                default_value="true",
                description="Enable ROS 1 pedsim-style group gaze, coherence, and repulsion forces.",
            ),
            DeclareLaunchArgument(
                "pedestrian_group_size_lambda",
                default_value="1.1",
                description="Poisson lambda used by ROS 1 pedsim to split agent clusters into groups.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_gaze",
                default_value="3.0",
                description="Group gaze-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_coherence",
                default_value="2.0",
                description="Group coherence-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_group_repulsion",
                default_value="1.0",
                description="Group repulsion-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_random",
                default_value="0.1",
                description="Random-force multiplier matching the ROS 1 pedsim default.",
            ),
            DeclareLaunchArgument(
                "pedestrian_force_along_wall",
                default_value="2.0",
                description="Along-wall force multiplier matching the ROS 1 pedsim default.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
# 【脚本说明｜semantic_cnn_nav_gazebo.launch.py】
# 用途：ROS 2 launch 启动入口，集中配置并启动一个导航、感知或数据采集场景。
# 输入输出：输入为 launch 参数、地图、模型和配置；输出为启动的 ROS 2 节点、话题、服务及相关日志。
# 关系：被 ros2 launch 或 pipeline 调用，依赖本 ROS 2 包和下游 scripts 节点；同目录文件是不同场景入口，不是备份。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
