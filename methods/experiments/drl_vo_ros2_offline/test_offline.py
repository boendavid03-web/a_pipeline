from __future__ import annotations
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/home/user/navigation_project/a_pipeline/, /home/user/navigation_project/a_pipeline/runs/
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPZ, PT
# 可能使用的关键环境变量：MODEL, SAMPLE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_offline.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 11:42:54.368400241 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_offline.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import math
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch

from drlvo_model import (
    load_policy_strict,
    load_semantic_policy,
    load_trained_semantic_policy,
)
from observation_adapter import (
    ObservationAdapter,
    compress_scan_history,
    compress_semantic_history,
    normalized_to_physical,
    pedestrian_risk_proxies,
    rotate_map_to_base,
)
from train_behavior_cloning import (
    make_blocked_split,
    remove_pedestrian_velocity,
    seed_split_indices,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]


class AdapterTests(unittest.TestCase):
    def test_rotation_map_to_base(self) -> None:
        vector = np.asarray([[1.0, 0.0]], dtype=np.float32)
        result = rotate_map_to_base(vector, math.pi / 2.0)
        np.testing.assert_allclose(result, [[0.0, -1.0]], atol=1e-6)

    def test_scan_compression_matches_legacy_loop(self) -> None:
        scans = np.arange(7200, dtype=np.float32).reshape(10, 720)
        expected = np.zeros((20, 80), dtype=np.float32)
        for n in range(10):
            for i in range(80):
                group = scans[n, i * 9 : (i + 1) * 9]
                expected[2 * n, i] = np.min(group)
                expected[2 * n + 1, i] = np.mean(group)
        expected = np.tile(expected.reshape(-1), 4)
        np.testing.assert_array_equal(compress_scan_history(scans), expected)

    def test_semantic_compression_uses_nearest_and_majority(self) -> None:
        scans = np.zeros((10, 720), dtype=np.float32)
        semantics = np.full((10, 720), -1, dtype=np.int64)
        scans[:, :3] = [2.0, 1.0, 3.0]
        semantics[:, :3] = [2, 3, 2]
        result = compress_semantic_history(scans, semantics)
        self.assertEqual(result.shape, (80, 80))
        for row in (0, 20, 40, 60):
            self.assertEqual(int(result[row, 0]), 3)
            self.assertEqual(int(result[row + 1, 0]), 2)
            self.assertEqual(int(result[row, 1]), -1)

    def test_action_mapping(self) -> None:
        np.testing.assert_allclose(
            normalized_to_physical(np.asarray([-1.0, -1.0])),
            [0.0, -2.0],
        )
        np.testing.assert_allclose(
            normalized_to_physical(np.asarray([1.0, 1.0])),
            [0.5, 2.0],
        )

    def test_blocked_split_is_disjoint_and_purged(self) -> None:
        splits = make_blocked_split(1000, block_size=100, purge_frames=20)
        sets = {name: set(indices.tolist()) for name, indices in splits.items()}
        self.assertFalse(sets["train"] & sets["val"])
        self.assertFalse(sets["train"] & sets["test"])
        self.assertFalse(sets["val"] & sets["test"])
        self.assertNotIn(99, set().union(*sets.values()))
        self.assertNotIn(100, set().union(*sets.values()))

    def test_seed_split_indices_map_dev_to_validation(self) -> None:
        splits = seed_split_indices(
            np.asarray(["train", "dev", "test", "train"])
        )
        np.testing.assert_array_equal(splits["train"], [0, 3])
        np.testing.assert_array_equal(splits["val"], [1])
        np.testing.assert_array_equal(splits["test"], [2])

    def test_remove_pedestrian_velocity_preserves_scan_and_goal(self) -> None:
        observations = np.arange(2 * 19202, dtype=np.float32).reshape(2, 19202)
        result = remove_pedestrian_velocity(observations)
        np.testing.assert_array_equal(result[:, :12800], 0.0)
        np.testing.assert_array_equal(result[:, 12800:], observations[:, 12800:])
        self.assertTrue(np.any(observations[:, :12800] != 0.0))

    def test_adapter_reset_clears_both_histories(self) -> None:
        adapter = ObservationAdapter(include_semantics=True)
        adapter._scan_history.append(np.ones(720, dtype=np.float32))
        adapter._semantic_history.append(np.zeros(720, dtype=np.int64))
        adapter._sequence_key = ("bag", 2)
        adapter.reset()
        self.assertEqual(adapter._scan_history, [])
        self.assertEqual(adapter._semantic_history, [])
        self.assertIsNone(adapter._sequence_key)

    def test_pedestrian_ttc(self) -> None:
        ttc, closest_distance, closest_time = pedestrian_risk_proxies(
            np.asarray([[2.0, 0.0]], dtype=np.float32),
            np.asarray([[-1.0, 0.0]], dtype=np.float32),
            np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            robot_linear_velocity=0.0,
            collision_radius=0.6,
        )
        self.assertAlmostEqual(ttc, 1.4, places=5)
        self.assertAlmostEqual(closest_distance, 0.0, places=5)
        self.assertAlmostEqual(closest_time, 2.0, places=5)


class IntegrationTests(unittest.TestCase):
    MODEL = PROJECT_ROOT / "github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip"
    SAMPLE = (
        PROJECT_ROOT
        / "runs/20260717_042135_v7_dual/datasets/"
        "20260727_three_bag_online_seed_split_v1/fixed_slots/"
        "20260727_074611-v7-fixed-dual-v3-2000x2000-"
        "converted-pedgt-v1-sgonline/samples/0000000.npz"
    )

    def test_strict_load_and_single_frame(self) -> None:
        policy, count = load_policy_strict(self.MODEL)
        self.assertEqual(count, 163)
        frame = ObservationAdapter().adapt(self.SAMPLE)
        self.assertEqual(frame.observation.shape, (19202,))
        self.assertTrue(np.isfinite(frame.observation).all())
        with torch.inference_mode():
            action = policy.deterministic_action(
                torch.from_numpy(frame.observation).unsqueeze(0)
            )
        self.assertEqual(tuple(action.shape), (1, 2))
        self.assertTrue(torch.isfinite(action).all())

        semantic_policy, semantic_count = load_semantic_policy(
            self.MODEL,
            semantic_num_classes=7,
        )
        semantic_frame = ObservationAdapter(include_semantics=True).adapt(self.SAMPLE)
        self.assertEqual(semantic_count, 163)
        self.assertEqual(semantic_frame.semantic_map.shape, (80, 80))
        self.assertTrue(np.issubdtype(semantic_frame.semantic_map.dtype, np.integer))
        with torch.inference_mode():
            semantic_action = semantic_policy.deterministic_action(
                torch.from_numpy(semantic_frame.observation).unsqueeze(0),
                torch.from_numpy(semantic_frame.semantic_map).unsqueeze(0),
            )
        # The residual semantic projection is initialized to zero, so enabling
        # the interface cannot silently change the pretrained policy.
        torch.testing.assert_close(semantic_action, action)

        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "semantic.pt"
            torch.save(semantic_policy.state_dict(), checkpoint)
            reloaded, state_items = load_trained_semantic_policy(checkpoint, 7)
        self.assertEqual(state_items, 172)
        with torch.inference_mode():
            reloaded_action = reloaded.deterministic_action(
                torch.from_numpy(semantic_frame.observation).unsqueeze(0),
                torch.from_numpy(semantic_frame.semantic_map).unsqueeze(0),
            )
        torch.testing.assert_close(reloaded_action, semantic_action)

    def test_history_resets_when_episode_changes(self) -> None:
        session = self.SAMPLE.parents[1]
        metadata = json.loads(
            (session / "metadata.json").read_text(encoding="utf-8")
        )
        first_episode = int(metadata["episodes"][0]["episode_id"])
        second_episode = int(metadata["episodes"][1]["episode_id"])
        second_index = int(metadata["episodes"][0]["sample_count"])
        adapter = ObservationAdapter(include_semantics=True)
        first = adapter.adapt(self.SAMPLE, sequence_id="bag")
        adapter.adapt(self.SAMPLE.with_name("0000001.npz"), sequence_id="bag")
        self.assertEqual(first.episode_id, first_episode)
        self.assertEqual(len(adapter._scan_history), 2)
        changed = adapter.adapt(
            self.SAMPLE.with_name(f"{second_index:07d}.npz"),
            sequence_id="bag",
        )
        self.assertEqual(changed.episode_id, second_episode)
        self.assertEqual(len(adapter._scan_history), 1)
        self.assertEqual(len(adapter._semantic_history), 1)


if __name__ == "__main__":
    unittest.main()
