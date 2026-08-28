#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：MAX_SIMULATION_CLOCK_GAP_SECONDS, MODULE, MODULE_PATH, NANOSECONDS_PER_SECOND, SPEC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_scenario_pedestrian_time.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 10:09:30.547124498 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.377915240 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_scenario_pedestrian_time.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/scenario_pedestrian_controller.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import importlib.util
import math
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "scenario_pedestrian_controller.py"
)
SPEC = importlib.util.spec_from_file_location(
    "scenario_pedestrian_controller", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_simulation_clock_delta_states():
    assert MODULE.simulation_clock_delta(None, 100) == ("initialize", 0.0)
    assert MODULE.simulation_clock_delta(100, 100) == ("paused", 0.0)
    assert MODULE.simulation_clock_delta(101, 100) == ("reset", 0.0)

    status, elapsed = MODULE.simulation_clock_delta(
        1_000_000_000, 1_050_000_000
    )
    assert status == "advance"
    assert math.isclose(elapsed, 0.05, rel_tol=0.0, abs_tol=1e-12)


def test_simulation_clock_large_jump_is_rejected():
    status, elapsed = MODULE.simulation_clock_delta(
        0,
        int(
            (MODULE.MAX_SIMULATION_CLOCK_GAP_SECONDS + 0.1)
            * MODULE.NANOSECONDS_PER_SECOND
        ),
    )
    assert status == "jump"
    assert elapsed > MODULE.MAX_SIMULATION_CLOCK_GAP_SECONDS


def test_integration_steps_preserve_all_elapsed_time():
    steps = MODULE.integration_steps(0.23, 0.05)
    assert len(steps) == 5
    assert all(0.0 < step <= 0.05 for step in steps)
    assert math.isclose(sum(steps), 0.23, rel_tol=0.0, abs_tol=1e-12)


def test_integration_steps_reject_invalid_step():
    try:
        MODULE.integration_steps(0.1, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("integration_steps accepted a non-positive max_step")


def test_robot_freshness_uses_simulation_time():
    now = 2_000_000_000
    assert MODULE.simulation_stamp_is_fresh(now, 1_000_000_000, 1.0)
    assert not MODULE.simulation_stamp_is_fresh(now, 999_999_999, 1.0)
    assert not MODULE.simulation_stamp_is_fresh(now, 2_000_000_001, 1.0)
    assert not MODULE.simulation_stamp_is_fresh(None, 1_000_000_000, 1.0)
