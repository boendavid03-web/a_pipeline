#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：EXPECTED_BEAMS, IGNORE_LABEL, LABEL_COLORS, MODULE, NUM_CLASSES, SCRIPT, SPEC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_s3net_fixed_dual_helpers.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:37:27.318342081 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.376915220 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_s3net_fixed_dual_helpers.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Pure helper tests for the fixed-dual S3-Net perception node."""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "s3net_fixed_dual_inference_node.py"
)
SPEC = importlib.util.spec_from_file_location("s3net_fixed_dual_helpers", SCRIPT)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot import {SCRIPT}")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class S3NetFixedDualHelperTests(unittest.TestCase):
    def test_scan_layout_is_exactly_two_thousand_beams_and_eight_meters(self):
        increment = 2.0 * math.pi / (MODULE.EXPECTED_BEAMS - 1)
        MODULE.validate_scan_layout(
            MODULE.EXPECTED_BEAMS,
            0.1,
            8.0,
            -math.pi,
            math.pi,
            increment,
        )
        with self.assertRaisesRegex(ValueError, "expected 2000 raw beams"):
            MODULE.validate_scan_layout(
                MODULE.EXPECTED_BEAMS - 1,
                0.1,
                8.0,
                -math.pi,
                math.pi,
                increment,
            )
        with self.assertRaisesRegex(ValueError, "range_max must be 8"):
            MODULE.validate_scan_layout(
                MODULE.EXPECTED_BEAMS,
                0.1,
                50.0,
                -math.pi,
                math.pi,
                increment,
            )

    def test_range_mask_caps_values_at_training_limit(self):
        ranges = np.asarray(
            [0.09, 0.1, 7.99, 8.0, 8.01, np.inf, np.nan],
            dtype=np.float32,
        )
        angles = np.zeros_like(ranges)
        valid = MODULE.range_valid_mask(ranges, angles, 0.1, 50.0)
        np.testing.assert_array_equal(
            valid,
            np.asarray([False, True, True, True, False, False, False]),
        )

    def test_normalization_matches_training_cleanup_and_masks_invalid(self):
        ranges = np.asarray([1.0, np.nan, 5.0], dtype=np.float32)
        angles = np.asarray([-1.0, 0.0, 1.0], dtype=np.float32)
        valid = np.asarray([True, False, True])
        stats = MODULE.NormalizationStats(
            scan_mean=1.0,
            scan_std=2.0,
            incidence_mean=0.5,
            incidence_std=0.5,
        )

        def fake_incidence(_ranges, _angles):
            return np.asarray([0.5, np.nan, 1.5], dtype=np.float32)

        scan, intensity, incidence = MODULE.normalize_s3_inputs(
            ranges,
            angles,
            valid,
            stats,
            fake_incidence,
        )
        np.testing.assert_allclose(scan, [0.0, 0.0, 2.0])
        np.testing.assert_allclose(intensity, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(incidence, [0.0, 0.0, 2.0])

    def test_projection_builds_footprint_self_mask(self):
        ranges = np.asarray([0.2, 1.0], dtype=np.float32)
        angles = np.asarray([0.0, 0.0], dtype=np.float32)
        valid = np.asarray([True, True])
        points, self_mask = MODULE.project_sensor_geometry(
            ranges,
            angles,
            valid,
            np.eye(3),
            np.zeros(3),
        )
        np.testing.assert_allclose(points[:, 0], [0.2, 1.0])
        np.testing.assert_array_equal(self_mask, [True, False])

    def test_machine_label_rows_preserve_unknown_and_color_classes(self):
        first = np.full(MODULE.EXPECTED_BEAMS, MODULE.IGNORE_LABEL, dtype=np.int16)
        second = np.zeros(MODULE.EXPECTED_BEAMS, dtype=np.int16)
        first[0] = 6
        rows = MODULE.stack_label_rows(first, second)
        self.assertEqual(rows.shape, (2, MODULE.EXPECTED_BEAMS))
        self.assertEqual(int(rows[0, 1]), MODULE.IGNORE_LABEL)
        rgb = MODULE.labels_to_rgb(rows)
        np.testing.assert_array_equal(rgb[0, 1], [0, 0, 0])
        np.testing.assert_array_equal(rgb[0, 0], MODULE.LABEL_COLORS[6])

        second[0] = MODULE.NUM_CLASSES
        with self.assertRaisesRegex(ValueError, "labels must be in"):
            MODULE.stack_label_rows(first, second)

    def test_checkpoint_and_sampling_contracts_are_explicit(self):
        MODULE.validate_checkpoint_contract(
            {
                "feature_mode": "range_incidence",
                "input_channels": 2,
                "num_output_channels": 7,
            }
        )
        with self.assertRaisesRegex(ValueError, "feature_mode"):
            MODULE.validate_checkpoint_contract(
                {
                    "feature_mode": "range_intensity_incidence",
                    "input_channels": 3,
                    "num_output_channels": 7,
                }
            )
        self.assertIsNone(
            MODULE.sampling_seed_for_frame("contract", 1337, 10)
        )
        self.assertIsNone(
            MODULE.sampling_seed_for_frame("seeded_sequence", 1337, 10)
        )
        self.assertEqual(
            MODULE.sampling_seed_for_frame("frame_seeded", 1337, 10),
            1347,
        )
        with self.assertRaisesRegex(ValueError, "sampling_strategy"):
            MODULE.sampling_seed_for_frame("posterior_mean", 1337, 0)


if __name__ == "__main__":
    unittest.main()
