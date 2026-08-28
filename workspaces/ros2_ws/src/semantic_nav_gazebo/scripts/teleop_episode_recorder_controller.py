#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel_stamped, /data_collection/episode_event, /data_collection/goal_accepted, /odom, /rosbag2_recorder/pause, /rosbag2_recorder/resume
# 检测到的消息类型：Odometry; PointStamped, TwistStamped; String
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：EPISODE_ARMED, EPISODE_CONTROLLER_READY, EPISODE_REACHED, EPISODE_SAVED, EPISODE_STARTED, EVENT_SCHEMA, RELIABLE, TRANSIENT_LOCAL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo teleop_episode_recorder_controller.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 02:06:09.576071483 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.375915200 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh（通过 ros2 run 启动该 ROS 2 节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/teleop_episode_recorder_controller.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜teleop_episode_recorder_controller.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Pause and resume rosbag2 around goal-conditioned teleop episodes."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Pause, Resume
from std_msgs.msg import String


EVENT_SCHEMA = "semantic_nav_episode_event/v1"


def command_is_moving(linear_x, angular_z, linear_threshold, angular_threshold):
    return (
        abs(float(linear_x)) >= float(linear_threshold)
        or abs(float(angular_z)) >= float(angular_threshold)
    )


def robot_is_stopped(linear_x, angular_z, linear_threshold, angular_threshold):
    return (
        abs(float(linear_x)) <= float(linear_threshold)
        and abs(float(angular_z)) <= float(angular_threshold)
    )


class TeleopEpisodeRecorderController(Node):
    def __init__(self):
        super().__init__("teleop_episode_recorder_controller")
        self.declare_parameter(
            "goal_accepted_topic", "/data_collection/goal_accepted"
        )
        self.declare_parameter(
            "episode_event_topic", "/data_collection/episode_event"
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_stamped_topic", "/cmd_vel_stamped")
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("arrival_dwell_sec", 0.5)
        self.declare_parameter("end_event_grace_sec", 0.2)
        self.declare_parameter("motion_linear_threshold", 0.02)
        self.declare_parameter("motion_angular_threshold", 0.05)
        self.declare_parameter("stop_linear_threshold", 0.02)
        self.declare_parameter("stop_angular_threshold", 0.05)
        self.declare_parameter("manage_recorder_pause", True)
        self.declare_parameter(
            "pause_service", "/rosbag2_recorder/pause"
        )
        self.declare_parameter(
            "resume_service", "/rosbag2_recorder/resume"
        )

        self.state = "waiting_goal"
        self.episode_id = 0
        self.goal_xy = None
        self.pose = None
        self.odom_velocity = (0.0, 0.0)
        self.cmd_velocity = (0.0, 0.0)
        self.arrival_since_ns = None
        self.pause_due_ns = None
        self.service_pending = False

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
            TwistStamped,
            str(self.get_parameter("cmd_vel_stamped_topic").value),
            self.cmd_callback,
            10,
        )
        self.pause_client = self.create_client(
            Pause, str(self.get_parameter("pause_service").value)
        )
        self.resume_client = self.create_client(
            Resume, str(self.get_parameter("resume_service").value)
        )
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            "EPISODE_CONTROLLER_READY: waiting for a goal and rosbag2 services"
        )

    def now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def publish_event(self, event, reason=None):
        payload = {
            "schema": EVENT_SCHEMA,
            "event": event,
            "episode_id": self.episode_id,
            "stamp_ns": self.now_ns(),
            "goal": (
                [float(self.goal_xy[0]), float(self.goal_xy[1])]
                if self.goal_xy is not None
                else None
            ),
            "pose": (
                [float(self.pose[0]), float(self.pose[1]), float(self.pose[2])]
                if self.pose is not None
                else None
            ),
        }
        if reason is not None:
            payload["reason"] = reason
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.event_pub.publish(msg)

    def goal_callback(self, msg):
        if self.state in ("recording", "stopping") or self.service_pending:
            self.get_logger().error(
                "Rejecting a new goal while an episode is still recording"
            )
            return
        frame = msg.header.frame_id.lstrip("/")
        if frame and frame != "map":
            self.get_logger().error(
                f"Rejecting accepted goal in frame {frame!r}; expected 'map'"
            )
            return
        goal = (float(msg.point.x), float(msg.point.y))
        if not all(math.isfinite(value) for value in goal):
            self.get_logger().error("Rejecting non-finite accepted goal")
            return
        self.episode_id += 1
        self.goal_xy = goal
        self.arrival_since_ns = None
        self.pause_due_ns = None
        self.state = "armed"
        self.publish_event("armed")
        self.get_logger().info(
            f"EPISODE_ARMED id={self.episode_id} "
            f"goal=({goal[0]:.2f}, {goal[1]:.2f}); move to resume recording"
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (float(pose.position.x), float(pose.position.y), yaw)
        self.odom_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )

    def cmd_callback(self, msg):
        self.cmd_velocity = (
            float(msg.twist.linear.x),
            float(msg.twist.angular.z),
        )
        if (
            self.state == "armed"
            and not self.service_pending
            and command_is_moving(
                *self.cmd_velocity,
                float(self.get_parameter("motion_linear_threshold").value),
                float(self.get_parameter("motion_angular_threshold").value),
            )
        ):
            self.request_resume()

    def request_resume(self):
        if not bool(self.get_parameter("manage_recorder_pause").value):
            self.state = "recording"
            self.publish_event("start", "first_nonzero_teleop_command")
            self.get_logger().info(f"EPISODE_STARTED id={self.episode_id}")
            return
        if not self.resume_client.service_is_ready():
            self.get_logger().warning(
                "rosbag2 resume service is not ready; keeping episode armed"
            )
            return
        self.service_pending = True
        future = self.resume_client.call_async(Resume.Request())
        future.add_done_callback(self.resume_done)

    def resume_done(self, future):
        self.service_pending = False
        if future.exception() is not None:
            self.get_logger().error(f"Could not resume rosbag2: {future.exception()}")
            return
        self.state = "recording"
        self.publish_event("start", "first_nonzero_teleop_command")
        self.get_logger().info(f"EPISODE_STARTED id={self.episode_id}")

    def request_pause(self):
        if not bool(self.get_parameter("manage_recorder_pause").value):
            self.finish_episode()
            return
        if not self.pause_client.service_is_ready():
            self.get_logger().error(
                "rosbag2 pause service is not ready; continuing to record"
            )
            self.pause_due_ns = self.now_ns() + 500_000_000
            return
        self.service_pending = True
        future = self.pause_client.call_async(Pause.Request())
        future.add_done_callback(self.pause_done)

    def pause_done(self, future):
        self.service_pending = False
        if future.exception() is not None:
            self.get_logger().error(f"Could not pause rosbag2: {future.exception()}")
            self.pause_due_ns = self.now_ns() + 500_000_000
            return
        self.finish_episode()

    def finish_episode(self):
        self.state = "waiting_goal"
        self.pause_due_ns = None
        self.arrival_since_ns = None
        reason = (
            "rosbag_paused"
            if bool(self.get_parameter("manage_recorder_pause").value)
            else "episode_closed"
        )
        self.publish_event("ready", reason)
        self.get_logger().info(
            f"EPISODE_SAVED id={self.episode_id}; publish the next /goal_pose"
        )

    def timer_callback(self):
        now_ns = self.now_ns()
        if (
            self.state == "stopping"
            and not self.service_pending
            and self.pause_due_ns is not None
            and now_ns >= self.pause_due_ns
        ):
            self.request_pause()
            return
        if self.state != "recording" or self.goal_xy is None or self.pose is None:
            return

        distance = math.hypot(
            self.pose[0] - self.goal_xy[0],
            self.pose[1] - self.goal_xy[1],
        )
        stopped = robot_is_stopped(
            *self.odom_velocity,
            float(self.get_parameter("stop_linear_threshold").value),
            float(self.get_parameter("stop_angular_threshold").value),
        ) and robot_is_stopped(
            *self.cmd_velocity,
            float(self.get_parameter("stop_linear_threshold").value),
            float(self.get_parameter("stop_angular_threshold").value),
        )
        if distance <= float(self.get_parameter("goal_tolerance").value) and stopped:
            if self.arrival_since_ns is None:
                self.arrival_since_ns = now_ns
            dwell_ns = int(
                round(
                    float(self.get_parameter("arrival_dwell_sec").value)
                    * 1_000_000_000.0
                )
            )
            if now_ns - self.arrival_since_ns >= dwell_ns:
                self.publish_event("end", "goal_reached_and_stopped")
                self.state = "stopping"
                self.pause_due_ns = now_ns + int(
                    round(
                        float(self.get_parameter("end_event_grace_sec").value)
                        * 1_000_000_000.0
                    )
                )
                self.get_logger().info(
                    f"EPISODE_REACHED id={self.episode_id}; finalizing and pausing"
                )
        else:
            self.arrival_since_ns = None


def main():
    rclpy.init()
    node = TeleopEpisodeRecorderController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
