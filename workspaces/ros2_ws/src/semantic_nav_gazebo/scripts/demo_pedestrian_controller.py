#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--reptype, --req, --reqtype, --timeout
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：WORLD
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo demo_pedestrian_controller.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:04.635564113 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/demo_pedestrian_controller.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜demo_pedestrian_controller.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import math
import subprocess
import time

import rclpy
from rclpy.node import Node


class DemoPedestrianController(Node):
    def __init__(self):
        super().__init__("demo_pedestrian_controller")

        self.declare_parameter("world_name", "default")
        self.declare_parameter("model_file", "")
        self.declare_parameter("spawn_delay", 4.0)
        self.declare_parameter("update_rate", 5.0)
        self.declare_parameter("speed", 0.8)

        self.world_name = self.get_parameter("world_name").value
        self.model_file = self.get_parameter("model_file").value
        self.spawn_delay = float(self.get_parameter("spawn_delay").value)
        self.update_rate = max(0.5, float(self.get_parameter("update_rate").value))
        self.speed = max(0.05, float(self.get_parameter("speed").value))

        self.paths = [
            ("pedestrian_1", (2.5, 15.5), (7.5, 15.5), 0.0),
            ("pedestrian_2", (10.0, 10.0), (15.5, 10.0), 0.6),
            ("pedestrian_3", (12.0, 16.5), (18.0, 16.5), 1.2),
            ("pedestrian_4", (20.0, 10.2), (26.0, 10.2), 1.8),
            ("pedestrian_5", (18.0, 19.5), (23.5, 19.5), 2.4),
        ]

        self.spawned = False
        self.start_time = None
        self.spawn_timer = self.create_timer(self.spawn_delay, self._spawn_once)
        self.move_timer = None

    def _spawn_once(self):
        self.spawn_timer.cancel()
        if not self.model_file:
            self.get_logger().error("model_file parameter is empty; cannot spawn pedestrians")
            return

        self.get_logger().info("Spawning 5 demo pedestrians sequentially")
        for name, start, _, _ in self.paths:
            if not self._spawn_model(name, start[0], start[1]):
                self.get_logger().error(f"Failed to spawn {name}")
                continue
            self.get_logger().info(f"Spawned {name} at x={start[0]:.2f}, y={start[1]:.2f}")

        self.spawned = True
        self.start_time = time.monotonic()
        self.move_timer = self.create_timer(1.0 / self.update_rate, self._update_poses)

    def _spawn_model(self, name, x, y):
        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            self.world_name,
            "-file",
            self.model_file,
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

        result = subprocess.run(command, capture_output=True, text=True, timeout=8.0)
        if result.returncode == 0:
            return True

        output = (result.stdout + result.stderr).strip()
        self.get_logger().error(output or f"{name} create command returned {result.returncode}")
        return False

    def _update_poses(self):
        if not self.spawned or self.start_time is None:
            return

        elapsed = time.monotonic() - self.start_time
        request_parts = []
        for name, start, end, phase in self.paths:
            x, y, yaw = self._pose_on_path(start, end, elapsed, phase)
            request_parts.append(
                "pose { "
                f'name: "{name}" '
                f"position {{ x: {x:.3f} y: {y:.3f} z: 0.0 }} "
                f"orientation {{ z: {math.sin(yaw / 2.0):.6f} w: {math.cos(yaw / 2.0):.6f} }} "
                "}"
            )

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
            "200",
            "--req",
            " ".join(request_parts),
        ]

        result = subprocess.run(command, capture_output=True, text=True, timeout=1.0)
        if result.returncode != 0:
            output = (result.stdout + result.stderr).strip()
            self.get_logger().warn(output or "set_pose_vector failed")

    def _pose_on_path(self, start, end, elapsed, phase):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = max(0.001, math.hypot(dx, dy))
        omega = self.speed * math.pi / distance
        wave = math.sin(elapsed * omega + phase)
        alpha = 0.5 + 0.5 * wave

        x = start[0] + dx * alpha
        y = start[1] + dy * alpha

        forward_yaw = math.atan2(dy, dx)
        moving_forward = math.cos(elapsed * omega + phase) >= 0.0
        yaw = forward_yaw if moving_forward else forward_yaw + math.pi
        return x, y, yaw


def main():
    rclpy.init()
    node = DemoPedestrianController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
