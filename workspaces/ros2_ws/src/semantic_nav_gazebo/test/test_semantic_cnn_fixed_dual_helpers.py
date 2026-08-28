#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：MODULE, MODULE_PATH, NANOSECONDS_PER_SECOND, SPEC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_semantic_cnn_fixed_dual_helpers.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:59:28.585708590 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.377915240 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_semantic_cnn_fixed_dual_helpers.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "semantic_cnn_fixed_dual_inference_node.py"
)
SPEC = importlib.util.spec_from_file_location(
    "semantic_cnn_fixed_dual_inference_node",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class SemanticCnnFixedDualHelperTests(unittest.TestCase):
    def test_checkpoint_goal_normalization_uses_recorded_vector_contract(self):
        mean, std, source = MODULE.checkpoint_goal_normalization({
            "normalization": {
                "source": "stats_json",
                "sub_goal_mean": [0.8, -0.05],
                "sub_goal_std": [0.55, 0.32],
            }
        })
        np.testing.assert_allclose(mean, [0.8, -0.05])
        np.testing.assert_allclose(std, [0.55, 0.32])
        self.assertEqual(source, "stats_json")

    def test_checkpoint_goal_normalization_preserves_legacy_fallback(self):
        mean, std, source = MODULE.checkpoint_goal_normalization({})
        np.testing.assert_allclose(mean, [MODULE.GOAL_MU, MODULE.GOAL_MU])
        np.testing.assert_allclose(std, [MODULE.GOAL_STD, MODULE.GOAL_STD])
        self.assertEqual(source, "legacy_constants")

    def test_checkpoint_goal_normalization_rejects_invalid_std(self):
        with self.assertRaisesRegex(ValueError, "std must be positive"):
            MODULE.checkpoint_goal_normalization({
                "normalization": {
                    "sub_goal_mean": [0.0, 0.0],
                    "sub_goal_std": [1.0, 0.0],
                }
            })

    def test_freshness_rejects_stale_and_future_scan(self):
        second = MODULE.NANOSECONDS_PER_SECOND
        self.assertTrue(MODULE.time_is_fresh(2 * second, second, 1.0))
        self.assertFalse(
            MODULE.time_is_fresh(2 * second, second - 1, 1.0)
        )
        self.assertFalse(MODULE.time_is_fresh(second, second + 1, 1.0))

    def test_latest_causal_subgoal_rejects_future_and_stale_samples(self):
        second = MODULE.NANOSECONDS_PER_SECOND
        samples = [
            (second, "stale"),
            (2 * second, "causal"),
            (2 * second + 100, "future"),
        ]
        selected = MODULE.latest_causal_sample(
            samples,
            2 * second + 50,
            0.3,
        )
        self.assertEqual(selected, ("causal", 2 * second))
        self.assertEqual(
            MODULE.latest_causal_sample(
                [
                    (2 * second - 20, "older"),
                    (2 * second - 10, "newest"),
                    (2 * second + 10, "future"),
                ],
                2 * second,
                0.3,
            ),
            ("newest", 2 * second - 10),
        )
        self.assertIsNone(
            MODULE.latest_causal_sample(
                [(second, "stale"), (3 * second, "future")],
                2 * second,
                0.3,
            )
        )
        boundary_stamp = 4 * second
        self.assertEqual(
            MODULE.latest_causal_sample(
                [(boundary_stamp - 300_000_000, "boundary")],
                boundary_stamp,
                0.3,
            ),
            ("boundary", boundary_stamp - 300_000_000),
        )
        self.assertIsNone(
            MODULE.latest_causal_sample(
                [(boundary_stamp - 300_000_001, "too-old")],
                boundary_stamp,
                0.3,
            )
        )
        self.assertIsNone(
            MODULE.latest_causal_sample([], boundary_stamp, 0.3)
        )

    def test_clock_rollback_detection(self):
        self.assertTrue(MODULE.clock_rolled_back(101, 100))
        self.assertFalse(MODULE.clock_rolled_back(100, 100))


if __name__ == "__main__":
    unittest.main()
