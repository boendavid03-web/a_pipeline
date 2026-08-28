#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /data_collection/episode_event, /data_collection/goal_accepted, /odom
# 检测到的消息类型：Odometry; PointStamped, Twist; String
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：EVENT_SCHEMA, NANOSECONDS_PER_SECOND, RELIABLE, TRANSIENT_LOCAL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo demo_goal_arrival_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-28 04:42:19.339888325 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.635564113 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/episode_goal_picker.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_start_goal_path_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

# 【脚本说明｜演示目标到达检测节点】
# 用途：判断机器人是否到达已接受的导航目标，并且已经稳定停止；满足条件后发出“可开始下一回合”的事件。
# 输入：/data_collection/goal_accepted（geometry_msgs/PointStamped）、/odom（nav_msgs/Odometry）、
#       /cmd_vel（geometry_msgs/Twist）；以及目标距离、停止阈值、停留时间等 ROS 2 参数。
# 输出：向 /data_collection/episode_event（std_msgs/String）发布 JSON 事件，事件格式为
#       semantic_nav_episode_event/v1，event=ready。
# 依赖关系：依赖目标发布节点（例如 semantic_start_goal_path_node.py）、里程计和速度指令发布者；
#            它只做检测，不负责规划路径，也不直接控制机器人。
# 被依赖关系：发布的 ready 事件可被 episode_goal_picker.py、teleop_episode_recorder_controller.py
#              等数据采集/目标选择脚本订阅，用于开启下一目标或下一回合。
# 文件定位：这是 src/semantic_nav_gazebo/scripts/ 下的当前源脚本；install/ 下同名文件只是指向它的符号链接，
#            不是备份或旧版本。重新构建 ROS 2 工作空间时，以本文件为准重新安装。

"""Publish a picker-ready event after a navigation goal is reached and stopped."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


EVENT_SCHEMA = "semantic_nav_episode_event/v1"
NANOSECONDS_PER_SECOND = 1_000_000_000


def velocity_is_stopped(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    linear_threshold: float,
    angular_threshold: float,
) -> bool:
    return (
        math.hypot(float(linear_x), float(linear_y))
        <= float(linear_threshold)
        and abs(float(angular_z)) <= float(angular_threshold)
    )


def time_is_fresh(
    reference_ns: int | None,
    stamp_ns: int | None,
    timeout_seconds: float,
) -> bool:
    if reference_ns is None or stamp_ns is None or timeout_seconds < 0.0:
        return False
    age_ns = int(reference_ns) - int(stamp_ns)
    return 0 <= age_ns <= int(timeout_seconds * NANOSECONDS_PER_SECOND)


class DemoGoalArrival(Node):
    def __init__(self) -> None:
        super().__init__("demo_goal_arrival")
        self.declare_parameter(
            "goal_accepted_topic",
            "/data_collection/goal_accepted",
        )
        self.declare_parameter(
            "episode_event_topic",
            "/data_collection/episode_event",
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("arrival_dwell_sec", 0.5)
        self.declare_parameter("linear_stop_threshold", 0.02)
        self.declare_parameter("angular_stop_threshold", 0.05)
        self.declare_parameter("input_timeout", 0.5)

        for name in (
            "goal_tolerance",
            "arrival_dwell_sec",
            "linear_stop_threshold",
            "angular_stop_threshold",
            "input_timeout",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be a non-negative finite number")

        self.goal_xy: tuple[float, float] | None = None
        self.pose_xy: tuple[float, float] | None = None
        self.odom_velocity: tuple[float, float, float] | None = None
        self.cmd_velocity: tuple[float, float, float] | None = None
        self.odom_receipt_ns: int | None = None
        self.cmd_receipt_ns: int | None = None
        self.arrival_since_ns: int | None = None
        self.last_clock_ns: int | None = None
        self.goal_id = 0
        self.state = "waiting_goal"

        accepted_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.event_pub = self.create_publisher(
            String,
            str(self.get_parameter("episode_event_topic").value),
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("goal_accepted_topic").value),
            self.goal_callback,
            accepted_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_callback,
            10,
        )
        self.create_timer(0.05, self.timer_callback)

    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def observe_clock(self) -> int:
        now_ns = self.now_ns()
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.arrival_since_ns = None
            self.odom_receipt_ns = None
            self.cmd_receipt_ns = None
        self.last_clock_ns = now_ns
        return now_ns

    def goal_callback(self, message: PointStamped) -> None:
        frame = message.header.frame_id.lstrip("/")
        if frame and frame != "map":
            self.get_logger().error(
                f"rejecting accepted goal frame {frame!r}; expected 'map'"
            )
            return
        goal = (float(message.point.x), float(message.point.y))
        if not all(math.isfinite(value) for value in goal):
            self.get_logger().error("rejecting non-finite accepted goal")
            return
        self.goal_id += 1
        self.goal_xy = goal
        self.arrival_since_ns = None
        self.state = "navigating"
        self.get_logger().info(
            f"armed goal {self.goal_id}: ({goal[0]:.2f}, {goal[1]:.2f})"
        )

    def odom_callback(self, message: Odometry) -> None:
        now_ns = self.observe_clock()
        pose = message.pose.pose.position
        twist = message.twist.twist
        values = (
            float(pose.x),
            float(pose.y),
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.pose_xy = None
            self.odom_velocity = None
            self.odom_receipt_ns = None
            self.arrival_since_ns = None
            return
        self.pose_xy = values[:2]
        self.odom_velocity = values[2:]
        self.odom_receipt_ns = now_ns

    def cmd_callback(self, message: Twist) -> None:
        now_ns = self.observe_clock()
        velocity = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in velocity):
            self.cmd_velocity = None
            self.cmd_receipt_ns = None
            self.arrival_since_ns = None
            return
        self.cmd_velocity = velocity
        self.cmd_receipt_ns = now_ns

    def publish_ready(self, now_ns: int) -> None:
        payload = {
            "schema": EVENT_SCHEMA,
            "event": "ready",
            "episode_id": self.goal_id,
            "stamp_ns": int(now_ns),
            "goal": list(self.goal_xy) if self.goal_xy is not None else None,
            "pose": list(self.pose_xy) if self.pose_xy is not None else None,
            "reason": "navigation_goal_reached_and_stopped",
        }
        message = String()
        message.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
        )
        self.event_pub.publish(message)

    def timer_callback(self) -> None:
        now_ns = self.observe_clock()
        if (
            self.state != "navigating"
            or self.goal_xy is None
            or self.pose_xy is None
            or self.odom_velocity is None
            or self.cmd_velocity is None
        ):
            return
        timeout = float(self.get_parameter("input_timeout").value)
        if not (
            time_is_fresh(now_ns, self.odom_receipt_ns, timeout)
            and time_is_fresh(now_ns, self.cmd_receipt_ns, timeout)
        ):
            self.arrival_since_ns = None
            return

        distance = math.hypot(
            self.pose_xy[0] - self.goal_xy[0],
            self.pose_xy[1] - self.goal_xy[1],
        )
        stopped = velocity_is_stopped(
            *self.odom_velocity,
            float(self.get_parameter("linear_stop_threshold").value),
            float(self.get_parameter("angular_stop_threshold").value),
        ) and velocity_is_stopped(
            *self.cmd_velocity,
            float(self.get_parameter("linear_stop_threshold").value),
            float(self.get_parameter("angular_stop_threshold").value),
        )
        if (
            distance <= float(self.get_parameter("goal_tolerance").value)
            and stopped
        ):
            if self.arrival_since_ns is None:
                self.arrival_since_ns = now_ns
                return
            dwell_ns = int(
                float(self.get_parameter("arrival_dwell_sec").value)
                * NANOSECONDS_PER_SECOND
            )
            if now_ns - self.arrival_since_ns >= dwell_ns:
                self.publish_ready(now_ns)
                self.state = "waiting_goal"
                self.arrival_since_ns = None
                self.get_logger().info(
                    f"goal {self.goal_id} reached; opening next-goal picker"
                )
        else:
            self.arrival_since_ns = None


def main() -> None:
    rclpy.init()
    node = DemoGoalArrival()
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
