#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel
# 检测到的消息类型：Twist
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：HELP, MOVE_BINDINGS, SPEED_BINDINGS, TCSADRAIN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo continuous_teleop.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 05:53:26.577644987 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.635564113 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh（通过 ros2 run 启动该 ROS 2 节点）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/continuous_teleop.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/closed_loop_demo_recorder.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_goal_arrival_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

# 【脚本说明｜连续键盘遥操作节点】
# 用途：读取终端键盘按键，把运动方向和速度转换为持续发布的机器人速度指令。
# 输入：终端键盘；ROS 2 参数 cmd_topic、publish_rate、speed、turn。
# 输出：向 cmd_topic（默认 /cmd_vel）发布 geometry_msgs/Twist；Ctrl-C 退出前会发布零速度。
# 依赖关系：依赖终端输入和 ROS 2；需要机器人底盘、仿真桥接或其他控制节点订阅 /cmd_vel 执行运动。
# 被依赖关系：通常由人工测试、数据采集或演示流程直接启动；它不依赖 closed_loop_demo_recorder.py
#              或 demo_goal_arrival_node.py，三者可以同时运行但职责不同。
# 文件定位：这是 src/semantic_nav_gazebo/scripts/ 下的当前源脚本；install/ 下同名文件只是指向它的符号链接，
#            不是备份或旧版本。重新构建 ROS 2 工作空间时，以本文件为准重新安装。

"""Keyboard teleoperation that continuously publishes the active Twist."""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOVE_BINDINGS = {
    "u": (1, 0, 1),
    "i": (1, 0, 0),
    "o": (1, 0, -1),
    "j": (0, 0, 1),
    "l": (0, 0, -1),
    "m": (-1, 0, -1),
    ",": (-1, 0, 0),
    ".": (-1, 0, 1),
    "U": (1, 1, 0),
    "I": (1, 0, 0),
    "O": (1, -1, 0),
    "J": (0, 1, 0),
    "L": (0, -1, 0),
    "M": (-1, 1, 0),
    "<": (-1, 0, 0),
    ">": (-1, -1, 0),
    "t": (0, 0, 0, 1),
    "b": (0, 0, 0, -1),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}

HELP = """
Continuous teleop keys:
  u/i/o forward-left/forward/forward-right
  j/l turn left/right, m/,/. backward-left/backward/backward-right
  J/L strafe left/right, k or space stop
  q/z both speed up/down, w/x linear speed up/down, e/c angular speed up/down

Direction commands remain active and are published continuously. Press k to stop.
Ctrl-C exits and publishes zero velocity.
"""


class ContinuousTeleop(Node):
    def __init__(self):
        super().__init__("continuous_teleop")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)
        self.declare_parameter("subscriber_timeout", 2.0)

        self.speed = max(0.01, float(self.get_parameter("speed").value))
        self.turn = max(0.01, float(self.get_parameter("turn").value))
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.subscriber_timeout = max(
            0.1, float(self.get_parameter("subscriber_timeout").value)
        )
        self.last_subscriber_time = time.monotonic()
        self.subscriber_loss_reported = False
        self.command = Twist()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10
        )
        self.create_timer(1.0 / publish_rate, self._publish)

    def handle_key(self, key):
        if key in MOVE_BINDINGS:
            values = MOVE_BINDINGS[key]
            self.x = values[0]
            self.y = values[1]
            self.z = values[3] if len(values) == 4 else 0.0
            self.th = values[2]
            self._update_command()
        elif key in SPEED_BINDINGS:
            self.speed *= SPEED_BINDINGS[key][0]
            self.turn *= SPEED_BINDINGS[key][1]
            self._update_command()
            self.get_logger().info(
                f"speed={self.speed:.2f}, turn={self.turn:.2f}"
            )
        else:
            self.stop()

    def stop(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.command = Twist()
        self._publish()

    def _update_command(self):
        self.command.linear.x = self.x * self.speed
        self.command.linear.y = self.y * self.speed
        self.command.linear.z = self.z * self.speed
        self.command.angular.z = self.th * self.turn

    def _publish(self):
        now = time.monotonic()
        if self.publisher.get_subscription_count() > 0:
            self.last_subscriber_time = now
            self.subscriber_loss_reported = False
        elif now - self.last_subscriber_time >= self.subscriber_timeout:
            moving = any(
                abs(value) > 1.0e-9
                for value in (self.x, self.y, self.z, self.th)
            )
            if moving:
                self.x = self.y = self.z = self.th = 0.0
                self.command = Twist()
            if not self.subscriber_loss_reported:
                self.get_logger().warning(
                    "cmd_vel subscriber disappeared; command reset to zero"
                )
                self.subscriber_loss_reported = True
        self.publisher.publish(self.command)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ContinuousTeleop()
    print(HELP)
    print(f"currently: speed {node.speed:.2f}, turn {node.turn:.2f}")

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = sys.stdin.read(1)
                if key == "\x03":
                    break
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
