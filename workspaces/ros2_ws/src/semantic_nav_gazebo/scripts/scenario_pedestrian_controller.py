#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--reptype, --req, --reqtype, --timeout
# 代码中检测到的 ROS 2 话题/路径字符串：/clock, /odom, /pedestrian_actor_pose_commands, /pedestrian_ground_truth
# 检测到的消息类型：Clock; Odometry; PedestrianState, PedestrianStateArray; Pose, PoseArray
# 检测到的文件格式：WORLD
# 可能使用的关键环境变量：ELDER_AGENT_TYPE, MAX_INTEGRATION_STEP_SECONDS, MAX_SIMULATION_CLOCK_GAP_SECONDS, NANOSECONDS_PER_SECOND, ROBOT_AGENT_TYPE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo scenario_pedestrian_controller.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.644741936 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_scenario_pedestrian_time.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_scenario_pedestrian_time.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_nav_gazebo.launch_1.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_scenario_pedestrian_time.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/check_ros1_ros2_reference.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜scenario_pedestrian_controller.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import math
import random
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from semantic_nav_gazebo.msg import PedestrianState, PedestrianStateArray


ROBOT_AGENT_TYPE = 2
ELDER_AGENT_TYPE = 3
NANOSECONDS_PER_SECOND = 1_000_000_000
MAX_INTEGRATION_STEP_SECONDS = 0.05
MAX_SIMULATION_CLOCK_GAP_SECONDS = 5.0


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def simulation_clock_delta(previous_ns, current_ns):
    if previous_ns is None:
        return "initialize", 0.0
    delta_ns = int(current_ns) - int(previous_ns)
    if delta_ns < 0:
        return "reset", 0.0
    if delta_ns == 0:
        return "paused", 0.0
    delta_seconds = delta_ns / float(NANOSECONDS_PER_SECOND)
    if delta_seconds > MAX_SIMULATION_CLOCK_GAP_SECONDS:
        return "jump", delta_seconds
    return "advance", delta_seconds


def integration_steps(duration, max_step=MAX_INTEGRATION_STEP_SECONDS):
    if duration <= 0.0:
        return []
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")
    steps = []
    remaining = float(duration)
    while remaining > 1e-12:
        step = min(remaining, max_step)
        steps.append(step)
        remaining -= step
    return steps


def simulation_stamp_is_fresh(now_ns, stamp_ns, timeout_seconds):
    if now_ns is None or stamp_ns is None:
        return False
    age_ns = int(now_ns) - int(stamp_ns)
    return 0 <= age_ns <= int(float(timeout_seconds) * NANOSECONDS_PER_SECOND)


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    radius: float


@dataclass
class Agent:
    name: str
    agent_type: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    ax: float
    ay: float
    vmax: float
    desired_force_factor: float
    desired_dir_x: float
    desired_dir_y: float
    waypoint_index: int
    waypoints: list[Waypoint]
    group_id: int | None
    random_last_x: float
    random_last_y: float
    random_next_x: float
    random_next_y: float


@dataclass(frozen=True)
class Obstacle:
    x1: float
    y1: float
    x2: float
    y2: float


class ScenarioPedestrianController(Node):
    def __init__(self):
        super().__init__("scenario_pedestrian_controller")

        self.declare_parameter("world_name", "default")
        self.declare_parameter("model_file", "")
        self.declare_parameter("use_actors", False)
        self.declare_parameter("use_pose_bridge", False)
        self.declare_parameter("actor_model_file", "")
        self.declare_parameter("collision_proxy_model_file", "")
        self.declare_parameter("scene_file", "")
        self.declare_parameter("spawn_delay", 4.0)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("simulation_factor", 1.0)
        self.declare_parameter("speed", 0.8)
        self.declare_parameter("pedestrian_count", -1)
        self.declare_parameter("agent_radius", 0.35)
        self.declare_parameter("static_obstacle_clearance", 0.75)
        self.declare_parameter("relaxation_time", 0.5)
        self.declare_parameter("neighbor_range", 10.0)
        self.declare_parameter("force_obstacle", 10.0)
        self.declare_parameter("sigma_obstacle", 0.2)
        self.declare_parameter("force_social", 5.1)
        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("clock_topic", "/clock")
        self.declare_parameter("pedestrian_ground_truth_topic", "/pedestrian_ground_truth")
        self.declare_parameter("pedestrian_ground_truth_frame", "odom")
        self.declare_parameter("robot_radius", 0.47)
        self.declare_parameter("robot_clearance", 1.0)
        self.declare_parameter("force_robot_personal_space", 6.0)
        self.declare_parameter("sigma_robot_personal_space", 0.2)
        self.declare_parameter("robot_state_timeout", 1.0)
        self.declare_parameter("enable_groups", True)
        self.declare_parameter("group_size_lambda", 1.1)
        self.declare_parameter("force_group_gaze", 3.0)
        self.declare_parameter("force_group_coherence", 2.0)
        self.declare_parameter("force_group_repulsion", 1.0)
        self.declare_parameter("force_random", 0.1)
        self.declare_parameter("force_along_wall", 2.0)
        self.declare_parameter("seed", 7)

        self.world_name = self.get_parameter("world_name").value
        self.model_file = self.get_parameter("model_file").value
        self.use_actors = bool(self.get_parameter("use_actors").value)
        self.use_pose_bridge = bool(self.get_parameter("use_pose_bridge").value)
        self.actor_model_file = self.get_parameter("actor_model_file").value
        self.collision_proxy_model_file = self.get_parameter(
            "collision_proxy_model_file"
        ).value
        self.scene_file = self.get_parameter("scene_file").value
        self.spawn_delay = float(self.get_parameter("spawn_delay").value)
        self.update_rate = max(0.5, float(self.get_parameter("update_rate").value))
        self.simulation_factor = float(self.get_parameter("simulation_factor").value)
        if not math.isclose(
            self.simulation_factor, 1.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                "simulation_factor must be 1.0 so pedestrian Pose, Twist, and "
                "/clock stay kinematically consistent; use speed to change pace"
            )
        self.base_speed = max(0.05, float(self.get_parameter("speed").value))
        self.pedestrian_count = int(self.get_parameter("pedestrian_count").value)
        if self.pedestrian_count < -1:
            raise ValueError("pedestrian_count must be -1 or a non-negative integer")
        self.agent_radius = max(0.05, float(self.get_parameter("agent_radius").value))
        self.static_obstacle_clearance = max(
            self.agent_radius + 0.03,
            float(self.get_parameter("static_obstacle_clearance").value),
        )
        self.relaxation_time = max(0.05, float(self.get_parameter("relaxation_time").value))
        self.neighbor_range = max(0.1, float(self.get_parameter("neighbor_range").value))
        self.force_obstacle = float(self.get_parameter("force_obstacle").value)
        self.sigma_obstacle = max(0.01, float(self.get_parameter("sigma_obstacle").value))
        self.force_social = float(self.get_parameter("force_social").value)
        self.robot_odom_topic = self.get_parameter("robot_odom_topic").value
        self.robot_radius = max(0.05, float(self.get_parameter("robot_radius").value))
        self.robot_clearance = max(
            self.agent_radius + self.robot_radius,
            float(self.get_parameter("robot_clearance").value),
        )
        self.force_robot_personal_space = float(
            self.get_parameter("force_robot_personal_space").value
        )
        self.sigma_robot_personal_space = max(
            0.01, float(self.get_parameter("sigma_robot_personal_space").value)
        )
        self.robot_state_timeout = max(
            0.05, float(self.get_parameter("robot_state_timeout").value)
        )
        self.enable_groups = bool(self.get_parameter("enable_groups").value)
        self.group_size_lambda = max(
            0.01, float(self.get_parameter("group_size_lambda").value)
        )
        self.force_group_gaze = float(self.get_parameter("force_group_gaze").value)
        self.force_group_coherence = float(
            self.get_parameter("force_group_coherence").value
        )
        self.force_group_repulsion = float(
            self.get_parameter("force_group_repulsion").value
        )
        self.force_random = float(self.get_parameter("force_random").value)
        self.force_along_wall = float(self.get_parameter("force_along_wall").value)
        self.rng = random.Random(int(self.get_parameter("seed").value))

        self.agents: list[Agent] = []
        self.groups: dict[int, list[Agent]] = {}
        self.obstacles: list[Obstacle] = []
        self.spawned = False
        self.last_update_sim_ns = None
        self.sim_time = 0.0
        self.robot_x = None
        self.robot_y = None
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_last_update_sim_ns = None
        self.sim_clock = None
        self.create_subscription(
            Odometry, self.robot_odom_topic, self._robot_odom_callback, 10
        )
        self.create_subscription(
            Clock,
            str(self.get_parameter("clock_topic").value),
            self._clock_callback,
            10,
        )
        self.pedestrian_ground_truth_frame = str(
            self.get_parameter("pedestrian_ground_truth_frame").value
        )
        self.pedestrian_ground_truth_publisher = self.create_publisher(
            PedestrianStateArray,
            str(self.get_parameter("pedestrian_ground_truth_topic").value),
            10,
        )
        self.pose_command_publisher = None
        if self.use_pose_bridge or self.use_actors:
            self.pose_command_publisher = self.create_publisher(
                PoseArray, "/pedestrian_actor_pose_commands", 10
            )
        self.spawn_timer = self.create_timer(self.spawn_delay, self._spawn_once)
        self.move_timer = None

    def _robot_odom_callback(self, msg):
        """Track the Gazebo robot in the same world-aligned odom frame as the scene."""
        pose = msg.pose.pose
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y

        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        linear = msg.twist.twist.linear
        self.robot_vx = math.cos(yaw) * linear.x - math.sin(yaw) * linear.y
        self.robot_vy = math.sin(yaw) * linear.x + math.cos(yaw) * linear.y
        self.robot_last_update_sim_ns = stamp_to_nanoseconds(msg.header.stamp)

    def _clock_callback(self, msg):
        previous_ns = (
            None if self.sim_clock is None else stamp_to_nanoseconds(self.sim_clock)
        )
        current_ns = stamp_to_nanoseconds(msg.clock)
        if previous_ns is not None and current_ns < previous_ns:
            self.last_update_sim_ns = None
            self.robot_last_update_sim_ns = None
            self.get_logger().warn(
                "Simulation clock moved backwards; rebasing pedestrian dynamics"
            )
        self.sim_clock = msg.clock

    def _spawn_once(self):
        self.spawn_timer.cancel()
        if not self.model_file:
            self.get_logger().error("model_file parameter is empty; cannot spawn pedestrians")
            return
        if self.use_actors and not self.actor_model_file:
            self.get_logger().error(
                "actor_model_file parameter is empty; cannot spawn actor pedestrians"
            )
            return
        if self.use_actors and not self.collision_proxy_model_file:
            self.get_logger().error(
                "collision_proxy_model_file parameter is empty; cannot spawn proxies"
            )
            return
        if not self.scene_file:
            self.get_logger().error("scene_file parameter is empty; cannot spawn pedestrians")
            return

        try:
            self.agents, self.obstacles, self.groups = self._load_scene(self.scene_file)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse scenario XML: {exc}")
            return

        if not self.agents:
            self.get_logger().warn("No non-robot agents with waypoints found in scenario XML")
            return

        for agent in self.agents:
            self._project_out_of_obstacles(agent)

        self.get_logger().info(
            f"Spawning {len(self.agents)} pedestrians from scenario: {self.scene_file}; "
            f"loaded {len(self.obstacles)} wall segments and {len(self.groups)} groups"
        )
        spawned_count = 0
        for agent in self.agents:
            if self.use_actors:
                spawned = self._spawn_model(
                    f"{agent.name}_actor",
                    # Actor trajectory poses are relative to the spawn pose.
                    # Start at the world origin so world-frame waypoint targets
                    # are not added to the agent's initial position twice.
                    0.0,
                    0.0,
                    self.actor_model_file,
                )
            else:
                spawned = self._spawn_model(
                    agent.name, agent.x, agent.y, self.model_file
                )

            if spawned:
                spawned_count += 1
                self.get_logger().info(
                    f"Spawned {agent.name} at x={agent.x:.2f}, y={agent.y:.2f}"
                )
            else:
                self.get_logger().error(f"Failed to spawn {agent.name}")

        if spawned_count == 0:
            self.get_logger().error("No pedestrians were spawned")
            return

        self.spawned = True
        # Spawning is synchronous and may take several wall-clock seconds.
        # Establish the physics baseline on the first post-spawn /clock sample
        # instead of catching up time during which the agents did not all exist.
        self.last_update_sim_ns = None
        self.move_timer = self.create_timer(1.0 / self.update_rate, self._update_poses)

    def _load_scene(self, scene_file):
        tree = ET.parse(scene_file)
        root = tree.getroot()

        waypoints = {}
        obstacles = []
        for element in root:
            if element.tag == "obstacle":
                obstacles.append(
                    Obstacle(
                        x1=float(element.attrib["x1"]),
                        y1=float(element.attrib["y1"]),
                        x2=float(element.attrib["x2"]),
                        y2=float(element.attrib["y2"]),
                    )
                )
            elif element.tag == "waypoint":
                name = element.attrib["id"]
                waypoints[name] = Waypoint(
                    name=name,
                    x=float(element.attrib["x"]),
                    y=float(element.attrib["y"]),
                    radius=max(0.1, float(element.attrib.get("r", "0.5"))),
                )
            elif element.tag == "queue":
                name = element.attrib["id"]
                waypoints[name] = Waypoint(
                    name=name,
                    x=float(element.attrib["x"]),
                    y=float(element.attrib["y"]),
                    radius=0.8,
                )

        pedestrian_clusters = []
        for cluster in root.findall("agent"):
            agent_type = int(cluster.attrib.get("type", "0"))
            if agent_type == ROBOT_AGENT_TYPE:
                continue

            waypoint_ids = [
                child.attrib["id"]
                for child in cluster
                if child.tag in ("addwaypoint", "addqueue")
            ]
            cluster_waypoints = [waypoints[name] for name in waypoint_ids if name in waypoints]
            if not cluster_waypoints:
                continue
            pedestrian_clusters.append((cluster, agent_type, cluster_waypoints))

        configured_counts = [
            max(0, int(cluster.attrib.get("n", "1")))
            for cluster, _, _ in pedestrian_clusters
        ]
        cluster_counts = self._allocate_pedestrian_counts(configured_counts)

        agents = []
        groups: dict[int, list[Agent]] = {}
        next_id = 1
        next_group_id = 1
        for (cluster, agent_type, cluster_waypoints), count in zip(
            pedestrian_clusters, cluster_counts
        ):

            x = float(cluster.attrib["x"])
            y = float(cluster.attrib["y"])
            dx = float(cluster.attrib.get("dx", "0"))
            dy = float(cluster.attrib.get("dy", "0"))

            cluster_agents = []
            for _ in range(count):
                start_x = self._distributed_value(x, dx)
                start_y = self._distributed_value(y, dy)
                first_target = cluster_waypoints[0]
                yaw = math.atan2(first_target.y - start_y, first_target.x - start_x)
                speed = self._agent_speed(agent_type)
                next_deviation_x, next_deviation_y = self._new_random_deviation()
                agents.append(
                    Agent(
                        name=f"pedestrian_{next_id}",
                        agent_type=agent_type,
                        x=start_x,
                        y=start_y,
                        yaw=yaw,
                        vx=0.0,
                        vy=0.0,
                        ax=0.0,
                        ay=0.0,
                        vmax=speed,
                        desired_force_factor=self._desired_force_factor(agent_type),
                        desired_dir_x=0.0,
                        desired_dir_y=0.0,
                        waypoint_index=0,
                        waypoints=cluster_waypoints,
                        group_id=None,
                        random_last_x=0.0,
                        random_last_y=0.0,
                        random_next_x=next_deviation_x,
                        random_next_y=next_deviation_y,
                    )
                )
                cluster_agents.append(agents[-1])
                next_id += 1

            next_group_id = self._assign_groups(cluster_agents, next_group_id)
            for agent in cluster_agents:
                if agent.group_id is not None:
                    groups.setdefault(agent.group_id, []).append(agent)

        return agents, obstacles, groups

    def _allocate_pedestrian_counts(self, configured_counts):
        """Preserve XML counts unless a total count was requested at launch."""
        if self.pedestrian_count == -1:
            return configured_counts
        if self.pedestrian_count == 0:
            return [0] * len(configured_counts)

        configured_total = sum(configured_counts)
        if configured_total <= 0:
            raise ValueError(
                "pedestrian_count was requested, but the scenario has no pedestrian agents"
            )

        scaled_counts = [
            self.pedestrian_count * count / configured_total
            for count in configured_counts
        ]
        allocated_counts = [int(count) for count in scaled_counts]
        remainder = self.pedestrian_count - sum(allocated_counts)
        for index in sorted(
            range(len(configured_counts)),
            key=lambda value: (
                scaled_counts[value] - allocated_counts[value],
                configured_counts[value],
                -value,
            ),
            reverse=True,
        )[:remainder]:
            allocated_counts[index] += 1

        return allocated_counts

    def _distributed_value(self, center, width):
        if width == 0.0:
            return center
        return center + self.rng.uniform(-width / 2.0, width / 2.0)

    def _agent_speed(self, agent_type):
        if agent_type == ELDER_AGENT_TYPE:
            return min(0.9, max(0.1, self.base_speed))
        return max(0.1, self.rng.gauss(self.base_speed, 0.26))

    def _desired_force_factor(self, agent_type):
        return 0.5 if agent_type == ELDER_AGENT_TYPE else 1.0

    def _assign_groups(self, cluster_agents, next_group_id):
        if not self.enable_groups or len(cluster_agents) <= 1:
            return next_group_id

        size_distribution = []
        assigned_count = 0
        total_count = len(cluster_agents)
        while assigned_count < total_count:
            group_size = 0
            while group_size == 0:
                group_size = self._poisson(self.group_size_lambda)
            group_size = min(group_size, total_count - assigned_count)
            while len(size_distribution) < group_size:
                size_distribution.append(0)
            size_distribution[group_size - 1] += 1
            assigned_count += group_size

        unassigned = list(cluster_agents)
        for group_size in range(len(size_distribution), 0, -1):
            for _ in range(size_distribution[group_size - 1]):
                if not unassigned:
                    return next_group_id
                leader = unassigned.pop(0)
                members = [leader]
                if group_size > 1:
                    nearest = sorted(
                        unassigned,
                        key=lambda agent: (agent.x - leader.x) ** 2
                        + (agent.y - leader.y) ** 2,
                    )[: group_size - 1]
                    for member in nearest:
                        members.append(member)
                        unassigned.remove(member)

                if len(members) > 1:
                    for member in members:
                        member.group_id = next_group_id
                    next_group_id += 1

        return next_group_id

    def _poisson(self, lambda_value):
        threshold = math.exp(-lambda_value)
        product = 1.0
        k = 0
        while product > threshold:
            k += 1
            product *= self.rng.random()
        return k - 1

    def _new_random_deviation(self):
        angle = math.radians(self.rng.uniform(0.0, 360.0))
        distance = self.rng.gauss(0.0, 1.0)
        return distance * math.cos(angle), distance * math.sin(angle)

    def _spawn_model(self, name, x, y, model_file):
        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            self.world_name,
            "-file",
            model_file,
            "-name",
            name,
            "-x",
            f"{x:.3f}",
            "-y",
            f"{y:.3f}",
            "-z",
            "0.0",
            "-allow_renaming",
            "false",
        ]

        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=8.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"{name} create command timed out")
            return False

        if result.returncode == 0:
            return True

        output = (result.stdout + result.stderr).strip()
        self.get_logger().error(output or f"{name} create command returned {result.returncode}")
        return False

    def _update_poses(self):
        if not self.spawned or self.sim_clock is None:
            return

        stamp = self.sim_clock
        current_sim_ns = stamp_to_nanoseconds(stamp)
        status, elapsed = simulation_clock_delta(
            self.last_update_sim_ns, current_sim_ns
        )
        self.last_update_sim_ns = current_sim_ns
        if status in ("initialize", "paused"):
            return
        if status == "reset":
            self.robot_last_update_sim_ns = None
            self.get_logger().warn(
                "Simulation clock moved backwards; skipped pedestrian update"
            )
            return
        if status == "jump":
            self.robot_last_update_sim_ns = None
            self.get_logger().error(
                "Simulation clock jumped forward by "
                f"{elapsed:.3f}s; rebased without moving pedestrians"
            )
            return

        max_step = min(MAX_INTEGRATION_STEP_SECONDS, 1.0 / self.update_rate)
        for step in integration_steps(elapsed, max_step):
            self.sim_time += step
            self._compute_forces(step)
            for agent in self.agents:
                self._move_agent(agent, step)

        request_parts = []
        actor_pose_commands = PoseArray()
        for agent in self.agents:
            if self.use_pose_bridge or self.use_actors:
                for _ in range(2):
                    pose = Pose()
                    pose.position.x = agent.x
                    pose.position.y = agent.y
                    pose.position.z = 0.0
                    pose.orientation.z = math.sin(agent.yaw / 2.0)
                    pose.orientation.w = math.cos(agent.yaw / 2.0)
                    actor_pose_commands.poses.append(pose)
            else:
                request_parts.append(
                    "pose { "
                    f'name: "{agent.name}" '
                    f"position {{ x: {agent.x:.3f} y: {agent.y:.3f} z: 0.0 }} "
                    f"orientation {{ z: {math.sin(agent.yaw / 2.0):.6f} "
                    f"w: {math.cos(agent.yaw / 2.0):.6f} }} "
                    "}"
                )
                # The visible person is the known-good ordinary Gazebo model.
                # Its GUI-transparent, world-preloaded proxy follows the same
                # pose so Fortress GPU LiDAR still observes the pedestrian.
                request_parts.append(
                    "pose { "
                    f'name: "{agent.name}_collision_proxy" '
                    f"position {{ x: {agent.x:.3f} y: {agent.y:.3f} z: 0.0 }} "
                    f"orientation {{ z: {math.sin(agent.yaw / 2.0):.6f} "
                    f"w: {math.cos(agent.yaw / 2.0):.6f} }} "
                    "}"
                )

        if self.use_pose_bridge or self.use_actors:
            self.pose_command_publisher.publish(actor_pose_commands)
            self._publish_pedestrian_ground_truth(stamp)
            return

        command = [
            "ign",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose_vector",
            "--reqtype",
            "ignition.msgs.Pose_V",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "500",
            "--req",
            " ".join(request_parts),
        ]

        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=2.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("set_pose_vector timed out; keeping controller alive")
            return

        if result.returncode != 0:
            output = (result.stdout + result.stderr).strip()
            self.get_logger().warn(output or "set_pose_vector failed")
            return
        self._publish_pedestrian_ground_truth(stamp)

    def _publish_pedestrian_ground_truth(self, stamp):
        message = PedestrianStateArray()
        message.header.stamp = stamp
        message.header.frame_id = self.pedestrian_ground_truth_frame
        for agent in self.agents:
            state = PedestrianState()
            state.id = agent.name
            state.pose.position.x = agent.x
            state.pose.position.y = agent.y
            state.pose.orientation.z = math.sin(agent.yaw / 2.0)
            state.pose.orientation.w = math.cos(agent.yaw / 2.0)
            state.velocity.linear.x = agent.vx
            state.velocity.linear.y = agent.vy
            state.velocity.angular.z = 0.0
            message.pedestrians.append(state)
        self.pedestrian_ground_truth_publisher.publish(message)

    def _compute_forces(self, dt):
        for agent in self.agents:
            desired_x, desired_y = self._desired_force(agent)
            social_x, social_y = self._social_force(agent)
            obstacle_x, obstacle_y = self._obstacle_force(agent)
            robot_x, robot_y = self._robot_personal_space_force(agent)
            additional_x, additional_y = self._additional_forces(agent, dt)

            agent.ax = (
                agent.desired_force_factor * desired_x
                + self.force_social * social_x
                + self.force_obstacle * obstacle_x
                + self.force_robot_personal_space * robot_x
                + additional_x
            )
            agent.ay = (
                agent.desired_force_factor * desired_y
                + self.force_social * social_y
                + self.force_obstacle * obstacle_y
                + self.force_robot_personal_space * robot_y
                + additional_y
            )

    def _desired_force(self, agent):
        if not agent.waypoints:
            agent.desired_dir_x = 0.0
            agent.desired_dir_y = 0.0
            return -agent.vx / self.relaxation_time, -agent.vy / self.relaxation_time

        target = agent.waypoints[agent.waypoint_index]
        dx = target.x - agent.x
        dy = target.y - agent.y
        distance = math.hypot(dx, dy)

        if distance <= target.radius:
            agent.waypoint_index = (agent.waypoint_index + 1) % len(agent.waypoints)
            target = agent.waypoints[agent.waypoint_index]
            dx = target.x - agent.x
            dy = target.y - agent.y
            distance = math.hypot(dx, dy)

        if distance < 1e-6:
            agent.desired_dir_x = 0.0
            agent.desired_dir_y = 0.0
            return -agent.vx / self.relaxation_time, -agent.vy / self.relaxation_time

        # Matches ROS 1 AreaWaypoint::closestPoint: agents aim through the
        # waypoint area and switch destination once they are inside its radius.
        target_x = target.x + target.radius * dx / distance
        target_y = target.y + target.radius * dy / distance
        desired_dx = target_x - agent.x
        desired_dy = target_y - agent.y
        desired_len = math.hypot(desired_dx, desired_dy)
        if desired_len < 1e-6:
            desired_x = 0.0
            desired_y = 0.0
        else:
            desired_x = desired_dx / desired_len
            desired_y = desired_dy / desired_len

        agent.desired_dir_x = desired_x
        agent.desired_dir_y = desired_y
        return (
            (desired_x * agent.vmax - agent.vx) / self.relaxation_time,
            (desired_y * agent.vmax - agent.vy) / self.relaxation_time,
        )

    def _additional_forces(self, agent, dt):
        force_x, force_y = self._random_force(agent, dt)

        along_x, along_y = self._along_wall_force(agent)
        force_x += along_x
        force_y += along_y

        if self.enable_groups and agent.group_id is not None:
            gaze_x, gaze_y = self._group_gaze_force(agent)
            coherence_x, coherence_y = self._group_coherence_force(agent)
            repulsion_x, repulsion_y = self._group_repulsion_force(agent)
            force_x += gaze_x + coherence_x + repulsion_x
            force_y += gaze_y + coherence_y + repulsion_y

        return force_x, force_y

    def _random_force(self, agent, dt):
        if self.force_random == 0.0:
            return 0.0, 0.0

        fading_duration = 1.0
        progress = math.fmod(self.sim_time, fading_duration)
        if progress < dt:
            agent.random_last_x = agent.random_next_x
            agent.random_last_y = agent.random_next_y
            agent.random_next_x, agent.random_next_y = self._new_random_deviation()

        mix = progress / fading_duration
        force_x = (1.0 - mix) * agent.random_last_x + mix * agent.random_next_x
        force_y = (1.0 - mix) * agent.random_last_y + mix * agent.random_next_y
        return self.force_random * force_x, self.force_random * force_y

    def _along_wall_force(self, agent):
        if self.force_along_wall == 0.0:
            return 0.0, 0.0
        if math.hypot(agent.vx, agent.vy) > 0.2:
            return 0.0, 0.0

        nearest = self._nearest_obstacle(agent.x, agent.y)
        if nearest is None:
            return 0.0, 0.0
        obstacle, point_x, point_y, distance = nearest
        if distance > 0.6:
            return 0.0, 0.0

        walking_len = math.hypot(agent.desired_dir_x, agent.desired_dir_y)
        if walking_len < 1e-6 or distance < 1e-6:
            return 0.0, 0.0

        to_obstacle_x = point_x - agent.x
        to_obstacle_y = point_y - agent.y
        angle = abs(
            self._angle_between(
                agent.desired_dir_x, agent.desired_dir_y, to_obstacle_x, to_obstacle_y
            )
        )
        if angle > math.radians(20.0):
            return 0.0, 0.0

        wall_x = obstacle.x2 - obstacle.x1
        wall_y = obstacle.y2 - obstacle.y1
        wall_len = math.hypot(wall_x, wall_y)
        if wall_len < 1e-6:
            return 0.0, 0.0

        if agent.desired_dir_x * wall_x + agent.desired_dir_y * wall_y < 0.0:
            wall_x = -wall_x
            wall_y = -wall_y
        return (
            self.force_along_wall * wall_x / wall_len,
            self.force_along_wall * wall_y / wall_len,
        )

    def _group_gaze_force(self, agent):
        members = self.groups.get(agent.group_id, [])
        member_count = len(members)
        if member_count <= 1:
            return 0.0, 0.0

        com_x, com_y = self._group_center(members)
        com_without_self_x = (member_count * com_x - agent.x) / (member_count - 1)
        com_without_self_y = (member_count * com_y - agent.y) / (member_count - 1)
        rel_x = com_without_self_x - agent.x
        rel_y = com_without_self_y - agent.y

        walking_len = math.hypot(agent.desired_dir_x, agent.desired_dir_y)
        rel_len = math.hypot(rel_x, rel_y)
        if walking_len < 1e-6 or rel_len < 1e-6:
            return 0.0, 0.0

        dot = agent.desired_dir_x * rel_x + agent.desired_dir_y * rel_y
        cos_angle = max(-1.0, min(1.0, dot / (walking_len * rel_len)))
        com_angle = math.acos(cos_angle)
        vision_angle = math.radians(90.0)
        if com_angle <= vision_angle:
            return 0.0, 0.0

        necessary_rotation = com_angle - vision_angle
        return (
            -self.force_group_gaze * necessary_rotation * agent.desired_dir_x,
            -self.force_group_gaze * necessary_rotation * agent.desired_dir_y,
        )

    def _group_coherence_force(self, agent):
        members = self.groups.get(agent.group_id, [])
        member_count = len(members)
        if member_count <= 1:
            return 0.0, 0.0

        com_x, com_y = self._group_center(members)
        rel_x = com_x - agent.x
        rel_y = com_y - agent.y
        distance = math.hypot(rel_x, rel_y)
        max_distance = (member_count - 1.0) / 2.0
        if distance < max_distance or distance < 1e-6:
            return 0.0, 0.0

        return (
            self.force_group_coherence * rel_x / distance,
            self.force_group_coherence * rel_y / distance,
        )

    def _group_repulsion_force(self, agent):
        members = self.groups.get(agent.group_id, [])
        if len(members) <= 1:
            return 0.0, 0.0

        force_x = 0.0
        force_y = 0.0
        for other in members:
            if other is agent:
                continue
            diff_x = agent.x - other.x
            diff_y = agent.y - other.y
            if math.hypot(diff_x, diff_y) < 0.5:
                force_x += diff_x
                force_y += diff_y

        return (
            self.force_group_repulsion * force_x,
            self.force_group_repulsion * force_y,
        )

    def _group_center(self, members):
        return (
            sum(agent.x for agent in members) / len(members),
            sum(agent.y for agent in members) / len(members),
        )

    def _social_force(self, agent):
        force_x = 0.0
        force_y = 0.0
        for other in self.agents:
            if other is agent:
                continue
            other_force_x, other_force_y = self._social_force_from_neighbor(
                agent, other.x, other.y, other.vx, other.vy
            )
            force_x += other_force_x
            force_y += other_force_y

        if self._robot_state_is_fresh():
            robot_force_x, robot_force_y = self._social_force_from_neighbor(
                agent, self.robot_x, self.robot_y, self.robot_vx, self.robot_vy
            )
            force_x += robot_force_x
            force_y += robot_force_y

        return force_x, force_y

    def _social_force_from_neighbor(self, agent, other_x, other_y, other_vx, other_vy):
        lambda_importance = 2.0
        gamma = 0.35
        n = 2.0
        n_prime = 3.0

        diff_x = other_x - agent.x
        diff_y = other_y - agent.y
        distance = math.hypot(diff_x, diff_y)
        if distance < 1e-6 or distance > self.neighbor_range:
            return 0.0, 0.0

        diff_dir_x = diff_x / distance
        diff_dir_y = diff_y / distance
        vel_diff_x = agent.vx - other_vx
        vel_diff_y = agent.vy - other_vy
        interaction_x = lambda_importance * vel_diff_x + diff_dir_x
        interaction_y = lambda_importance * vel_diff_y + diff_dir_y
        interaction_len = math.hypot(interaction_x, interaction_y)
        if interaction_len < 1e-6:
            return 0.0, 0.0

        interaction_dir_x = interaction_x / interaction_len
        interaction_dir_y = interaction_y / interaction_len
        theta = self._angle_between(
            interaction_dir_x, interaction_dir_y, diff_dir_x, diff_dir_y
        )
        b = max(1e-6, gamma * interaction_len)

        force_velocity_amount = -math.exp(
            -distance / b - (n_prime * b * theta) * (n_prime * b * theta)
        )
        force_angle_amount = -self._sign(theta) * math.exp(
            -distance / b - (n * b * theta) * (n * b * theta)
        )

        return (
            force_velocity_amount * interaction_dir_x
            + force_angle_amount * (-interaction_dir_y),
            force_velocity_amount * interaction_dir_y
            + force_angle_amount * interaction_dir_x,
        )

    def _robot_state_is_fresh(self):
        return simulation_stamp_is_fresh(
            None if self.sim_clock is None else stamp_to_nanoseconds(self.sim_clock),
            self.robot_last_update_sim_ns,
            self.robot_state_timeout,
        )

    def _robot_personal_space_force(self, agent):
        if not self._robot_state_is_fresh():
            return 0.0, 0.0

        diff_x = agent.x - self.robot_x
        diff_y = agent.y - self.robot_y
        distance = math.hypot(diff_x, diff_y)
        if distance < 1e-6:
            # An exact overlap has no geometric normal. Push opposite the
            # pedestrian heading so the next capped motion step separates it.
            direction_x = -math.cos(agent.yaw)
            direction_y = -math.sin(agent.yaw)
        else:
            direction_x = diff_x / distance
            direction_y = diff_y / distance

        clearance = distance - self.robot_clearance
        force_amount = math.exp(-clearance / self.sigma_robot_personal_space)
        return force_amount * direction_x, force_amount * direction_y

    def _obstacle_force(self, agent):
        nearest = self._nearest_obstacle(agent.x, agent.y)
        if nearest is None:
            return 0.0, 0.0

        _, point_x, point_y, distance = nearest
        diff_x = agent.x - point_x
        diff_y = agent.y - point_y
        if distance < 1e-6:
            return 0.0, 0.0

        clearance = distance - self.static_obstacle_clearance
        force_amount = math.exp(-clearance / self.sigma_obstacle)
        return force_amount * diff_x / distance, force_amount * diff_y / distance

    def _move_agent(self, agent, dt):
        old_x = agent.x
        old_y = agent.y

        agent.vx += dt * agent.ax
        agent.vy += dt * agent.ay

        speed = math.hypot(agent.vx, agent.vy)
        if speed > agent.vmax:
            agent.vx = agent.vx / speed * agent.vmax
            agent.vy = agent.vy / speed * agent.vmax

        agent.x += dt * agent.vx
        agent.y += dt * agent.vy

        crossed_obstacle = self._first_crossed_obstacle(old_x, old_y, agent.x, agent.y)
        if crossed_obstacle is not None:
            self._slide_along_obstacle(agent, old_x, old_y, crossed_obstacle)

        self._project_out_of_obstacles(agent)

        if math.hypot(agent.vx, agent.vy) > 1e-4:
            agent.yaw = math.atan2(agent.vy, agent.vx)

    def _first_crossed_obstacle(self, old_x, old_y, new_x, new_y):
        if math.hypot(new_x - old_x, new_y - old_y) < 1e-6:
            return None

        for obstacle in self.obstacles:
            if self._segments_intersect(
                old_x,
                old_y,
                new_x,
                new_y,
                obstacle.x1,
                obstacle.y1,
                obstacle.x2,
                obstacle.y2,
            ):
                return obstacle
        return None

    def _slide_along_obstacle(self, agent, old_x, old_y, obstacle):
        move_x = agent.x - old_x
        move_y = agent.y - old_y
        wall_x = obstacle.x2 - obstacle.x1
        wall_y = obstacle.y2 - obstacle.y1
        wall_len = math.hypot(wall_x, wall_y)
        if wall_len < 1e-6:
            agent.x = old_x
            agent.y = old_y
            agent.vx = 0.0
            agent.vy = 0.0
            return

        tangent_x = wall_x / wall_len
        tangent_y = wall_y / wall_len
        slide_distance = move_x * tangent_x + move_y * tangent_y
        agent.x = old_x + slide_distance * tangent_x
        agent.y = old_y + slide_distance * tangent_y

        point_x, point_y = self._closest_point_on_obstacle(old_x, old_y, obstacle)
        normal_x = old_x - point_x
        normal_y = old_y - point_y
        normal_len = math.hypot(normal_x, normal_y)
        if normal_len < 1e-6:
            normal_x = -tangent_y
            normal_y = tangent_x
            normal_len = 1.0

        normal_x /= normal_len
        normal_y /= normal_len
        inward_velocity = agent.vx * normal_x + agent.vy * normal_y
        if inward_velocity < 0.0:
            agent.vx -= inward_velocity * normal_x
            agent.vy -= inward_velocity * normal_y

    def _project_out_of_obstacles(self, agent):
        min_clearance = self.static_obstacle_clearance
        for _ in range(2):
            nearest = None
            nearest_dist = float("inf")
            for obstacle in self.obstacles:
                point_x, point_y = self._closest_point_on_obstacle(agent.x, agent.y, obstacle)
                diff_x = agent.x - point_x
                diff_y = agent.y - point_y
                dist = math.hypot(diff_x, diff_y)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest = (diff_x, diff_y)

            if nearest is None or nearest_dist >= min_clearance:
                return

            diff_x, diff_y = nearest
            if nearest_dist < 1e-6:
                return

            normal_x = diff_x / nearest_dist
            normal_y = diff_y / nearest_dist
            correction = min_clearance - nearest_dist
            agent.x += normal_x * correction
            agent.y += normal_y * correction

            inward_velocity = agent.vx * normal_x + agent.vy * normal_y
            if inward_velocity < 0.0:
                agent.vx -= inward_velocity * normal_x
                agent.vy -= inward_velocity * normal_y

    def _closest_point_on_obstacle(self, x, y, obstacle):
        seg_x = obstacle.x2 - obstacle.x1
        seg_y = obstacle.y2 - obstacle.y1
        length_squared = seg_x * seg_x + seg_y * seg_y
        if length_squared < 1e-9:
            return obstacle.x1, obstacle.y1

        rel_x = x - obstacle.x1
        rel_y = y - obstacle.y1
        lam = (rel_x * seg_x + rel_y * seg_y) / length_squared
        lam = max(0.0, min(1.0, lam))
        return obstacle.x1 + lam * seg_x, obstacle.y1 + lam * seg_y

    def _nearest_obstacle(self, x, y):
        nearest = None
        nearest_distance_squared = float("inf")

        for obstacle in self.obstacles:
            point_x, point_y = self._closest_point_on_obstacle(x, y, obstacle)
            diff_x = x - point_x
            diff_y = y - point_y
            distance_squared = diff_x * diff_x + diff_y * diff_y
            if distance_squared < nearest_distance_squared:
                nearest_distance_squared = distance_squared
                nearest = (obstacle, point_x, point_y)

        if nearest is None:
            return None

        obstacle, point_x, point_y = nearest
        return obstacle, point_x, point_y, math.sqrt(nearest_distance_squared)

    def _segments_intersect(self, ax, ay, bx, by, cx, cy, dx, dy):
        eps = 1e-9
        o1 = self._orientation(ax, ay, bx, by, cx, cy)
        o2 = self._orientation(ax, ay, bx, by, dx, dy)
        o3 = self._orientation(cx, cy, dx, dy, ax, ay)
        o4 = self._orientation(cx, cy, dx, dy, bx, by)

        if (
            o1 * o2 < -eps
            and o3 * o4 < -eps
        ):
            return True

        if abs(o1) <= eps and self._on_segment(ax, ay, cx, cy, bx, by):
            return True
        if abs(o2) <= eps and self._on_segment(ax, ay, dx, dy, bx, by):
            return True
        if abs(o3) <= eps and self._on_segment(cx, cy, ax, ay, dx, dy):
            return True
        if abs(o4) <= eps and self._on_segment(cx, cy, bx, by, dx, dy):
            return True
        return False

    def _orientation(self, ax, ay, bx, by, cx, cy):
        return (by - ay) * (cx - bx) - (bx - ax) * (cy - by)

    def _on_segment(self, ax, ay, bx, by, cx, cy):
        return (
            min(ax, cx) - 1e-9 <= bx <= max(ax, cx) + 1e-9
            and min(ay, cy) - 1e-9 <= by <= max(ay, cy) + 1e-9
        )

    def _angle_between(self, ax, ay, bx, by):
        return self._normalize_angle(math.atan2(by, bx) - math.atan2(ay, ax))

    def _normalize_angle(self, value):
        while value <= -math.pi:
            value += 2.0 * math.pi
        while value > math.pi:
            value -= 2.0 * math.pi
        return value

    def _sign(self, value):
        if value > 0.0:
            return 1.0
        if value < 0.0:
            return -1.0
        return 0.0


def main():
    rclpy.init()
    node = ScenarioPedestrianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
