#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：E402, PROJECT_ROOT, SCRIPT_DIR
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_teleop_episode_controller.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 02:12:00.143745661 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:53.597198074 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_teleop_episode_controller.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Pure-function tests for automatic episode motion and stop thresholds."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = (
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "scripts"
)
sys.path.insert(0, str(SCRIPT_DIR))

from teleop_episode_recorder_controller import (  # noqa: E402
    command_is_moving,
    robot_is_stopped,
)


class MotionThresholdTest(unittest.TestCase):
    def test_linear_or_angular_motion_starts_episode(self):
        self.assertTrue(command_is_moving(0.02, 0.0, 0.02, 0.05))
        self.assertTrue(command_is_moving(0.0, -0.05, 0.02, 0.05))
        self.assertFalse(command_is_moving(0.019, 0.049, 0.02, 0.05))

    def test_robot_must_stop_both_linear_and_angular_motion(self):
        self.assertTrue(robot_is_stopped(0.02, -0.05, 0.02, 0.05))
        self.assertFalse(robot_is_stopped(0.021, 0.0, 0.02, 0.05))
        self.assertFalse(robot_is_stopped(0.0, 0.051, 0.02, 0.05))


if __name__ == "__main__":
    unittest.main()
