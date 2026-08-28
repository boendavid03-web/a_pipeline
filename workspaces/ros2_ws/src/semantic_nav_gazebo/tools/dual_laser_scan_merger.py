#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/scan_01, /scan_02, /scan_merged
# 检测到的消息类型：LaserScan
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/dual_laser_scan_merger.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.640301645 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.377915240 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/tools/dual_laser_scan_merger.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜dual_laser_scan_merger.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_to_ns(msg: LaserScan) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def rpy_matrix(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[Tuple[float, float, float], ...]:
    """返回 Rz(yaw) * Ry(pitch) * Rx(roll) 旋转矩阵。"""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
        ),
        (
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
        ),
        (
            -sp,
            cp * sr,
            cp * cr,
        ),
    )


class DualLaserScanMerger(Node):
    def __init__(self) -> None:
        super().__init__("dual_laser_scan_merger")

        self.scan_01: Optional[LaserScan] = None
        self.scan_02: Optional[LaserScan] = None
        self.last_pair: Optional[Tuple[int, int]] = None
        self.publish_count = 0

        # 输出虚拟扫描参数
        self.output_frame = "base_link"
        self.output_topic = "/scan_merged"
        self.output_samples = 720
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (
            self.angle_max - self.angle_min
        ) / (self.output_samples - 1)

        self.range_min = 0.10
        self.range_max = 50.0

        # Keep the established fixed-beam self-filter envelope used by the
        # dataset and inference contract. It intentionally includes padding
        # beyond the visual-mesh collision envelope.
        self.self_filter_half_x = 0.36
        self.self_filter_half_y = 0.32

        # 两路雷达最大允许时间差
        self.max_sync_difference = 0.20

        # 老师真实安装位姿
        self.scan_01_pose = (
            0.2,
            0.13,
            0.208,
            3.14,
            0.0,
            0.0,
        )
        self.scan_02_pose = (
            -0.2,
            -0.13,
            0.208,
            3.14,
            0.0,
            3.14,
        )

        self.scan_01_rotation = rpy_matrix(*self.scan_01_pose[3:])
        self.scan_02_rotation = rpy_matrix(*self.scan_02_pose[3:])

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.subscription_01 = self.create_subscription(
            LaserScan,
            "/scan_01",
            self.scan_01_callback,
            qos_profile_sensor_data,
        )

        self.subscription_02 = self.create_subscription(
            LaserScan,
            "/scan_02",
            self.scan_02_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "双雷达融合节点已启动："
            "/scan_01 + /scan_02 -> /scan_merged，frame=base_link"
        )

    def scan_01_callback(self, msg: LaserScan) -> None:
        self.scan_01 = msg
        self.try_publish()

    def scan_02_callback(self, msg: LaserScan) -> None:
        self.scan_02 = msg
        self.try_publish()

    def add_scan_points(
        self,
        scan: LaserScan,
        pose: Tuple[float, float, float, float, float, float],
        rotation: Tuple[Tuple[float, float, float], ...],
        output_ranges: list,
    ) -> None:
        tx, ty, tz = pose[:3]

        for index, measured_range in enumerate(scan.ranges):
            value = float(measured_range)

            if not math.isfinite(value):
                continue

            if value < max(float(scan.range_min), self.range_min):
                continue

            if value > min(float(scan.range_max), self.range_max):
                continue

            source_angle = (
                float(scan.angle_min)
                + index * float(scan.angle_increment)
            )

            source_x = value * math.cos(source_angle)
            source_y = value * math.sin(source_angle)
            source_z = 0.0

            base_x = (
                tx
                + rotation[0][0] * source_x
                + rotation[0][1] * source_y
                + rotation[0][2] * source_z
            )
            base_y = (
                ty
                + rotation[1][0] * source_x
                + rotation[1][1] * source_y
                + rotation[1][2] * source_z
            )
            base_z = (
                tz
                + rotation[2][0] * source_x
                + rotation[2][1] * source_y
                + rotation[2][2] * source_z
            )

            # 明确位于机器人底盘内部的回波不作为环境障碍。
            if (
                abs(base_x) <= self.self_filter_half_x
                and abs(base_y) <= self.self_filter_half_y
            ):
                continue

            output_range = math.hypot(base_x, base_y)

            if output_range < self.range_min:
                continue

            if output_range > self.range_max:
                continue

            output_angle = math.atan2(base_y, base_x)

            output_index = int(
                round(
                    (output_angle - self.angle_min)
                    / self.angle_increment
                )
            )

            if not 0 <= output_index < self.output_samples:
                continue

            if output_range < output_ranges[output_index]:
                output_ranges[output_index] = output_range

    def try_publish(self) -> None:
        if self.scan_01 is None or self.scan_02 is None:
            return

        stamp_01 = stamp_to_ns(self.scan_01)
        stamp_02 = stamp_to_ns(self.scan_02)

        pair = (stamp_01, stamp_02)

        if pair == self.last_pair:
            return

        time_difference = abs(stamp_01 - stamp_02) / 1_000_000_000.0

        if time_difference > self.max_sync_difference:
            return

        output_ranges = [float("inf")] * self.output_samples

        self.add_scan_points(
            self.scan_01,
            self.scan_01_pose,
            self.scan_01_rotation,
            output_ranges,
        )

        self.add_scan_points(
            self.scan_02,
            self.scan_02_pose,
            self.scan_02_rotation,
            output_ranges,
        )

        output = LaserScan()

        # 使用两帧中较新的时间戳
        if stamp_01 >= stamp_02:
            output.header.stamp = self.scan_01.header.stamp
        else:
            output.header.stamp = self.scan_02.header.stamp

        output.header.frame_id = self.output_frame
        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment
        output.time_increment = 0.0
        output.scan_time = 0.1
        output.range_min = self.range_min
        output.range_max = self.range_max
        output.ranges = output_ranges
        output.intensities = []

        self.publisher.publish(output)
        self.last_pair = pair
        self.publish_count += 1

        if self.publish_count % 50 == 0:
            finite_count = sum(
                1 for value in output_ranges if math.isfinite(value)
            )
            self.get_logger().info(
                f"已发布 {self.publish_count} 帧 /scan_merged，"
                f"当前有效点 {finite_count}/{self.output_samples}，"
                f"两雷达时间差 {time_difference:.3f}s"
            )


def main() -> None:
    rclpy.init()
    node = DualLaserScanMerger()

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
