#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --map-yaml, --output-root, --samples-01, --samples-02, --semantic-label, --session-name
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PNG, YAML
# 可能使用的关键环境变量：E402, PROJECT_ROOT, TOOLS_DIR
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_fixed_dual_online_subgoals.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 01:28:23.386097626 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:53.597198074 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_fixed_dual_online_subgoals.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Pure-function tests for fixed dual-LiDAR subgoal selection."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
TOOLS_DIR = PROJECT_ROOT / "scripts" / "validation" / "ros2_workspace_tools"
sys.path.insert(0, str(TOOLS_DIR))

from v7_rosbag_to_fixed_dual_lidar_dataset import (  # noqa: E402
    causal_hold_last_subgoals,
    causal_hold_last_subgoals_by_episode,
    episode_ids_for_stamps,
    episode_intervals_from_events,
    common_valid_time_range,
    local_subgoals_by_episode,
    map_episode_events_to_sim_time,
    parse_args,
    reverse_motion_keep_mask,
    successful_episode_intervals,
    terminal_goal_stop_keep_mask,
    split_filenames_by_episode,
    subgoal_contract_metadata,
    trim_safe_boundary_cmd_vel_stamped,
)


class CausalSubgoalAlignmentTest(unittest.TestCase):
    def test_causal_hold_last_uses_latest_non_future_message(self):
        matches, audit = causal_hold_last_subgoals(
            [150, 250],
            [(100, (1.0, 0.0)), (200, (2.0, 0.0)), (300, (3.0, 0.0))],
            100,
        )
        self.assertEqual(matches[0], (100, (1.0, 0.0), 50))
        self.assertEqual(matches[1], (200, (2.0, 0.0), 50))
        self.assertEqual(audit["leading_unmatched_frames_dropped"], 0)

    def test_future_message_is_not_used_and_leading_frame_is_dropped(self):
        matches, audit = causal_hold_last_subgoals(
            [99, 100],
            [(100, (1.0, 0.0))],
            300,
        )
        self.assertIsNone(matches[0])
        self.assertEqual(matches[1], (100, (1.0, 0.0), 0))
        self.assertEqual(audit["leading_unmatched_frames_dropped"], 1)

    def test_max_age_is_inclusive_and_stale_frame_is_dropped(self):
        matches, _ = causal_hold_last_subgoals(
            [400],
            [(100, (1.0, 0.0))],
            300,
        )
        self.assertEqual(matches[0][2], 300)
        matches, audit = causal_hold_last_subgoals(
            [401],
            [(100, (1.0, 0.0))],
            300,
        )
        self.assertEqual(matches, [None])
        self.assertEqual(audit["stale_frames_dropped"], 1)

    def test_online_missing_messages_fails(self):
        matches, audit = causal_hold_last_subgoals([100], [], 300)
        self.assertEqual(matches, [None])
        self.assertEqual(audit["missing_causal_frames_dropped"], 1)


class SubgoalArgumentTest(unittest.TestCase):
    def test_hindsight_is_the_default_source(self):
        args = parse_args(
            [
                "--bag",
                "bag",
                "--output-root",
                "output",
                "--session-name",
                "session",
                "--map-yaml",
                "map.yaml",
                "--semantic-label",
                "label.png",
                "--samples-01",
                "360",
                "--samples-02",
                "360",
            ]
        )
        self.assertEqual(args.subgoal_source, "hindsight")
        self.assertEqual(args.subgoal_max_age_ms, 300.0)
        self.assertFalse(args.exclude_reverse_linear_x)
        self.assertEqual(args.reverse_linear_x_epsilon, 1e-3)
        self.assertEqual(args.reverse_recovery_frames, 15)

    def test_metadata_records_actual_online_source(self):
        metadata = subgoal_contract_metadata(
            "online",
            lookahead=20,
            max_age_ms=300.0,
            leading_unmatched_frames=2,
            ages_ns=[10_000_000, 20_000_000],
        )
        self.assertEqual(metadata["subgoal_source"], "online")
        self.assertEqual(metadata["subgoal_matched_samples"], 2)
        self.assertEqual(
            metadata["subgoal_unmatched_leading_frames_dropped"], 2
        )


class ReverseMotionFilterTest(unittest.TestCase):
    def test_reverse_and_following_recovery_frames_are_removed(self):
        commands = [
            (0.2, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.2, 0.0, 0.0),
            (0.2, 0.0, 0.0),
        ]
        keep, audit = reverse_motion_keep_mask(
            commands, [1] * len(commands), 1e-3, 2
        )
        self.assertEqual(keep, [True, False, False, False, False, True])
        self.assertEqual(audit["reverse_frames_removed"], 2)
        self.assertEqual(audit["recovery_frames_removed"], 2)
        self.assertEqual(audit["affected_episode_ids"], [1])

    def test_recovery_does_not_cross_episode_boundary(self):
        commands = [
            (-0.5, 0.0, 0.0),
            (0.2, 0.0, 0.0),
            (0.2, 0.0, 0.0),
        ]
        keep, audit = reverse_motion_keep_mask(
            commands, [1, 2, 2], 1e-3, 15
        )
        self.assertEqual(keep, [False, True, True])
        self.assertEqual(audit["recovery_frames_removed"], 0)

    def test_small_negative_noise_is_tolerated(self):
        keep, audit = reverse_motion_keep_mask(
            [(0.2, 0.0, 0.0), (-5e-4, 0.0, 0.0)],
            [1, 1],
            1e-3,
            15,
        )
        self.assertEqual(keep, [True, True])
        self.assertEqual(audit["total_frames_removed"], 0)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive and finite"):
            reverse_motion_keep_mask([(0.0, 0.0, 0.0)], [1], 0.0, 1)
        with self.assertRaisesRegex(ValueError, "non-negative"):
            reverse_motion_keep_mask([(0.0, 0.0, 0.0)], [1], 1e-3, -1)


class EpisodeBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.events = [
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "armed",
                "episode_id": 1,
                "stamp_ns": 50,
                "goal": [1.0, 0.0],
            },
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "start",
                "episode_id": 1,
                "stamp_ns": 100,
                "goal": [1.0, 0.0],
                "pose": [0.0, 0.0, 0.0],
            },
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "end",
                "episode_id": 1,
                "stamp_ns": 200,
                "goal": [1.0, 0.0],
                "reason": "goal_reached_and_stopped",
            },
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "start",
                "episode_id": 2,
                "stamp_ns": 300,
                "goal": [2.0, 0.0],
            },
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "end",
                "episode_id": 2,
                "stamp_ns": 400,
                "goal": [2.0, 0.0],
                "reason": "goal_reached_and_stopped",
            },
        ]

    def test_intervals_and_timestamp_assignment(self):
        intervals = episode_intervals_from_events(self.events)
        self.assertEqual([item["episode_id"] for item in intervals], [1, 2])
        self.assertEqual(
            episode_ids_for_stamps([99, 100, 200, 201, 300, 400, 401], intervals),
            [0, 1, 1, 0, 2, 2, 0],
        )

    def test_incomplete_episode_fails(self):
        with self.assertRaisesRegex(RuntimeError, "no recorded end"):
            episode_intervals_from_events(self.events[:-1])

    def test_success_filter_preserves_source_episode_ids(self):
        self.events[-1]["reason"] = "stuck_no_progress"
        intervals = successful_episode_intervals(
            episode_intervals_from_events(self.events)
        )
        self.assertEqual([item["episode_id"] for item in intervals], [1])

    def test_terminal_goal_stop_filter_removes_only_success_tail(self):
        keep, audit = terminal_goal_stop_keep_mask(
            [
                (0.3, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ],
            [1, 1, 1, 2],
            {1},
        )
        self.assertEqual(keep, [True, False, False, True])
        self.assertEqual(audit["total_frames_removed"], 2)

    def test_online_subgoal_alignment_resets_at_episode_boundary(self):
        intervals = episode_intervals_from_events(self.events)
        matches, audit = causal_hold_last_subgoals_by_episode(
            [100, 110, 300, 310],
            [1, 1, 2, 2],
            [(105, (1.0, 0.0)), (305, (2.0, 0.0))],
            intervals,
            20,
        )
        self.assertIsNone(matches[0])
        self.assertEqual(matches[1], (105, (1.0, 0.0), 5))
        self.assertIsNone(matches[2])
        self.assertEqual(matches[3], (305, (2.0, 0.0), 5))
        self.assertEqual(audit["leading_unmatched_frames_dropped"], 2)

    def test_hindsight_and_splits_do_not_cross_episode(self):
        import numpy as np

        positions = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [11.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        )
        subgoals = local_subgoals_by_episode(positions, [1, 1, 2, 2], 20)
        self.assertTrue(np.allclose(subgoals[1], [0.0, 0.0]))
        self.assertTrue(np.allclose(subgoals[3], [0.0, 0.0]))
        splits = split_filenames_by_episode(
            ["a", "b", "c", "d"],
            [1, 1, 2, 2],
            0.5,
            0.0,
            7,
        )
        split_sets = [set(values) for values in splits]
        self.assertFalse(
            any({"a", "b"} & values and {"a", "b"} - values for values in split_sets)
        )
        self.assertFalse(
            any({"c", "d"} & values and {"c", "d"} - values for values in split_sets)
        )

    def test_small_episode_corpus_keeps_train_and_dev_windows_possible(self):
        filenames = ["episode_1", "episode_2", "episode_3"]
        train, dev, test = split_filenames_by_episode(
            filenames, [1, 2, 3], 0.7, 0.1, 7
        )
        self.assertEqual((len(train), len(dev), len(test)), (1, 1, 1))


class TimeDomainTest(unittest.TestCase):
    def test_episode_storage_time_maps_through_clock(self):
        clocks = [(1_000, 100), (2_000, 200)]
        events = [
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "start",
                "episode_id": 2,
                "stamp_ns": 1_490,
                "_storage_stamp_ns": 1_500,
                "goal": [1.0, 2.0],
            },
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": "end",
                "episode_id": 2,
                "stamp_ns": 1_790,
                "_storage_stamp_ns": 1_800,
            },
        ]
        mapped, audit = map_episode_events_to_sim_time(events, clocks)
        intervals = episode_intervals_from_events(mapped)
        self.assertEqual(intervals[0]["start_stamp_ns"], 150)
        self.assertEqual(intervals[0]["end_stamp_ns"], 180)
        self.assertEqual(intervals[0]["start_payload_stamp_ns"], 1_490)
        self.assertEqual(intervals[0]["start_storage_stamp_ns"], 1_500)
        self.assertEqual(audit["payload_storage_delta_ns_max_abs"], 10)

    def test_episode_mapping_refuses_clock_extrapolation(self):
        event = {
            "schema": "semantic_nav_episode_event/v1",
            "event": "start",
            "episode_id": 1,
            "stamp_ns": 999,
            "_storage_stamp_ns": 999,
            "goal": [1.0, 0.0],
        }
        with self.assertRaisesRegex(RuntimeError, "outside"):
            map_episode_events_to_sim_time([event], [(1_000, 100), (2_000, 200)])

    def test_common_time_range_uses_all_required_streams(self):
        start, end, ranges = common_valid_time_range(
            [(10, 100), (20, 500)],
            [(90, None), (480, None)],
            [(110, None), (490, None)],
            [(120, None), (470, None)],
            [(115, None), (460, None)],
        )
        self.assertEqual((start, end), (120, 460))
        self.assertEqual(ranges["scan_02"]["start_ns"], 110)

    def test_zero_command_outside_clock_is_trimmed_after_frame_clipping(self):
        retained, audit = trim_safe_boundary_cmd_vel_stamped(
            [(90, (0.0, 0.0, 0.0)), (120, (0.5, 0.0, 0.0))],
            [(1, 100), (2, 200)],
            [(130, object(), 130, object())],
        )
        self.assertEqual(retained, [(120, (0.5, 0.0, 0.0))])
        self.assertEqual(audit["trimmed_count"], 1)
        self.assertTrue(audit["applied"])

    def test_nonzero_command_outside_clock_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "nonzero"):
            trim_safe_boundary_cmd_vel_stamped(
                [(90, (0.1, 0.0, 0.0)), (120, (0.5, 0.0, 0.0))],
                [(1, 100), (2, 200)],
                [(130, object(), 130, object())],
            )


if __name__ == "__main__":
    unittest.main()
