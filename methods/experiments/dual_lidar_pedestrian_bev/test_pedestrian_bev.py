#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：CONFIRMED
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/test_pedestrian_bev.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:40:44.727897842 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.378546463 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/test_pedestrian_bev.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Unit tests for neural pedestrian detection and tracking helpers.

This stays beside the model because it intentionally requires the isolated
PyTorch training interpreter, while the pipeline's core tests do not.
"""

from __future__ import annotations

import itertools
import unittest

import numpy as np
import torch

from methods.experiments.dual_lidar_pedestrian_bev.dataset import (
    BEVSpec,
    encode_temporal_bev,
)
from methods.experiments.dual_lidar_pedestrian_bev.model import (
    DecodedDetection,
    TemporalBEVPedestrianDetector,
    decode_detections,
    detection_loss,
)
from methods.experiments.dual_lidar_pedestrian_bev.tracker import (
    MapDetection,
    PedestrianTracker,
    detections_base_to_map,
    linear_sum_assignment,
)


class BEVModelTest(unittest.TestCase):
    def test_current_plus_deltas_encoding(self) -> None:
        temporal = np.asarray(
            [
                [[[0.0]], [[1.0]]],
                [[[2.0]], [[4.0]]],
                [[[5.0]], [[9.0]]],
            ],
            dtype=np.float32,
        ).reshape(6, 1, 1)
        encoded = encode_temporal_bev(temporal, "current_plus_deltas")
        np.testing.assert_array_equal(
            encoded[:, 0, 0],
            np.asarray([5.0, 9.0, 2.0, 3.0, 3.0, 5.0]),
        )

    def test_grid_round_trip(self) -> None:
        spec = BEVSpec(extent_m=8.0, resolution_m=0.1)
        points = np.asarray([[-7.95, -1.25], [0.05, 2.15], [7.85, 7.75]])
        grid_x, grid_y = spec.metric_to_grid(points)
        reconstructed = spec.grid_to_metric(grid_x, grid_y)
        np.testing.assert_allclose(reconstructed, points, atol=1e-10)

    def test_model_loss_and_decode_contract(self) -> None:
        torch.manual_seed(0)
        model = TemporalBEVPedestrianDetector(history_frames=2, base_channels=8)
        inputs = torch.rand(2, 4, 32, 32)
        outputs = model(inputs)
        self.assertEqual(tuple(outputs["heatmap_logits"].shape), (2, 1, 32, 32))
        batch = {
            "heatmap": torch.zeros(2, 1, 32, 32),
            "offset": torch.zeros(2, 2, 32, 32),
            "velocity": torch.zeros(2, 2, 32, 32),
            "regression_mask": torch.zeros(2, 1, 32, 32),
        }
        batch["heatmap"][:, :, 16, 16] = 1.0
        batch["regression_mask"][:, :, 16, 16] = 1.0
        losses = detection_loss(outputs, batch)
        self.assertTrue(bool(torch.isfinite(losses["loss"])))
        decoded = decode_detections(
            outputs,
            BEVSpec(extent_m=1.6, resolution_m=0.1),
            confidence_threshold=0.0,
            topk=3,
        )
        self.assertEqual(len(decoded), 2)
        self.assertEqual(len(decoded[0]), 3)


class AssignmentAndTrackingTest(unittest.TestCase):
    def test_tracker_defaults_match_selected_evaluation(self) -> None:
        tracker = PedestrianTracker()
        self.assertEqual(tracker.position_gate_m, 0.5)
        self.assertEqual(tracker.velocity_gate_mps, 1.5)
        self.assertEqual(tracker.tentative_timeout_s, 0.33)
        self.assertEqual(tracker.confirmed_timeout_s, 1.0)
        self.assertEqual(tracker.acceleration_sigma_mps2, 3.0)
        self.assertEqual(tracker.position_measurement_scale, 0.75)
        self.assertEqual(tracker.velocity_measurement_scale, 2.0)
        self.assertEqual(tracker.association_velocity_weight, 0.4)

    def test_tracker_measurement_scales_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            PedestrianTracker(position_measurement_scale=0.0)
        with self.assertRaises(ValueError):
            PedestrianTracker(velocity_measurement_scale=-1.0)
        with self.assertRaises(ValueError):
            PedestrianTracker(association_velocity_weight=-0.1)

    def test_hungarian_matches_brute_force(self) -> None:
        costs = np.asarray(
            [[4.0, 1.0, 3.0], [2.0, 0.0, 5.0], [3.0, 2.0, 2.0]]
        )
        rows, cols = linear_sum_assignment(costs)
        assigned_cost = float(costs[rows, cols].sum())
        brute = min(
            sum(costs[row, col] for row, col in enumerate(permutation))
            for permutation in itertools.permutations(range(3))
        )
        self.assertEqual(assigned_cost, brute)

    def test_base_map_conversion_and_stable_track_id(self) -> None:
        local = [
            DecodedDetection(
                position_xy_base=np.asarray([1.0, 0.0]),
                velocity_xy_robot_axes_absolute=np.asarray([0.5, 0.0]),
                confidence=0.9,
            )
        ]
        converted = detections_base_to_map(
            local, np.asarray([10.0, 20.0, np.pi / 2.0])
        )
        np.testing.assert_allclose(
            converted[0].position_xy_map, [10.0, 21.0], atol=1e-9
        )
        np.testing.assert_allclose(
            converted[0].velocity_xy_map_absolute, [0.0, 0.5], atol=1e-9
        )

        tracker = PedestrianTracker()
        ids = []
        for frame in range(6):
            timestamp_ns = int(frame * 0.066 * 1e9)
            tracks = tracker.update(
                [
                    MapDetection(
                        position_xy_map=np.asarray([2.0 + frame * 0.033, 1.0]),
                        velocity_xy_map_absolute=np.asarray([0.5, 0.0]),
                        confidence=0.9,
                    )
                ],
                timestamp_ns,
            )
            ids.append(tracks[0].track_id)
        self.assertEqual(ids, [1] * 6)
        self.assertEqual(tracks[0].track_state, "CONFIRMED")


if __name__ == "__main__":
    unittest.main()
