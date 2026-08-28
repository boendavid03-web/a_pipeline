from __future__ import annotations
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：COASTING, CONFIRMED, OBSERVATION_SIZE, TENTATIVE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_predicted_ped_map.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 01:13:09.110731627 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_predicted_ped_map.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import inspect
import math
import unittest

import numpy as np

from methods.experiments.drl_vo_ros2_offline.observation_adapter import (
    OBSERVATION_SIZE,
    ObservationAdapter,
    observation_with_pedestrian_map,
    tracks_to_drl_vo_ped_map,
    tracks_to_drl_vo_ped_map_with_diagnostics,
)
from methods.experiments.drl_vo_ros2_offline.predicted_ped_map_replay import (
    infer_tracks_without_ground_truth,
)
from methods.experiments.dual_lidar_pedestrian_bev.tracker import (
    MapDetection,
    PedestrianTracker,
)


def make_track(
    track_id: int,
    position: tuple[float, float],
    velocity: tuple[float, float] = (0.0, 0.0),
    *,
    state: str = "CONFIRMED",
    age_s: float = 0.0,
    confidence: float = 0.9,
) -> dict[str, object]:
    return {
        "track_id": track_id,
        "position_xy_map": np.asarray(position, dtype=np.float64),
        "velocity_xy_map_absolute": np.asarray(velocity, dtype=np.float64),
        "confidence": confidence,
        "track_state": state,
        "time_since_update_s": age_s,
    }


class PedestrianMapTests(unittest.TestCase):
    def test_yaw_zero_position_and_channel_order(self) -> None:
        pedestrian_map = tracks_to_drl_vo_ped_map(
            [make_track(1, (1.0, 0.0), (0.4, -0.2))],
            np.asarray([0.0, 0.0, 0.0]),
        )
        self.assertEqual(pedestrian_map.shape, (2, 80, 80))
        self.assertEqual(pedestrian_map.dtype, np.float32)
        self.assertAlmostEqual(float(pedestrian_map[0, 4, 40]), 0.4)
        self.assertAlmostEqual(float(pedestrian_map[1, 4, 40]), -0.2)

    def test_yaw_90_rotates_position_and_absolute_velocity(self) -> None:
        pedestrian_map = tracks_to_drl_vo_ped_map(
            [make_track(1, (0.0, 1.0), (1.0, 0.0))],
            np.asarray([0.0, 0.0, math.pi / 2.0]),
        )
        self.assertAlmostEqual(float(pedestrian_map[0, 4, 40]), 0.0, places=6)
        self.assertAlmostEqual(float(pedestrian_map[1, 4, 40]), -1.0, places=6)

    def test_robot_velocity_is_not_subtracted(self) -> None:
        pedestrian_map = tracks_to_drl_vo_ped_map(
            [make_track(1, (2.0, 0.0), (0.3, -0.4))],
            np.asarray([0.0, 0.0, 0.0]),
        )
        np.testing.assert_allclose(pedestrian_map[:, 8, 40], [0.3, -0.4])

    def test_inclusive_boundaries(self) -> None:
        tracks = [
            make_track(1, (0.0, 0.0), (1.0, 0.0)),
            make_track(2, (20.0, 9.0), (2.0, 0.0)),
            make_track(3, (10.0, 10.0), (3.0, 0.0)),
            make_track(4, (10.0, -10.0), (4.0, 0.0)),
        ]
        pedestrian_map, diagnostics = (
            tracks_to_drl_vo_ped_map_with_diagnostics(
                tracks, np.asarray([0.0, 0.0, 0.0])
            )
        )
        self.assertEqual(len(diagnostics["written_track_ids"]), 4)
        self.assertEqual(float(pedestrian_map[0, 0, 40]), 1.0)
        self.assertEqual(float(pedestrian_map[0, 79, 4]), 2.0)
        self.assertEqual(float(pedestrian_map[0, 40, 0]), 3.0)
        self.assertEqual(float(pedestrian_map[0, 40, 79]), 4.0)

    def test_out_of_range_tracks_are_dropped(self) -> None:
        tracks = [
            make_track(1, (-0.001, 0.0)),
            make_track(2, (20.001, 0.0)),
            make_track(3, (1.0, 10.001)),
            make_track(4, (1.0, -10.001)),
        ]
        pedestrian_map, diagnostics = (
            tracks_to_drl_vo_ped_map_with_diagnostics(
                tracks, np.asarray([0.0, 0.0, 0.0])
            )
        )
        self.assertEqual(diagnostics["written_track_ids"], [])
        self.assertEqual(diagnostics["dropped_track_count"], 4)
        self.assertFalse(np.any(pedestrian_map))

    def test_normalization_clip_and_c_order_flatten(self) -> None:
        base = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        pedestrian_map = np.zeros((2, 80, 80), dtype=np.float32)
        pedestrian_map[0, 0, 1] = 4.0
        pedestrian_map[1, 79, 78] = -3.0
        observation = observation_with_pedestrian_map(base, pedestrian_map)
        expected = np.clip(pedestrian_map / 2.0, -1.0, 1.0).reshape(-1)
        np.testing.assert_array_equal(observation[:12800], expected)
        np.testing.assert_array_equal(observation[12800:], 0.0)
        self.assertEqual(observation.shape, (19202,))

    def test_track_state_filtering(self) -> None:
        tracks = [
            make_track(1, (1.0, 0.0), (1.0, 0.0), state="CONFIRMED"),
            make_track(2, (2.0, 0.0), (2.0, 0.0), state="COASTING"),
            make_track(3, (3.0, 0.0), (3.0, 0.0), state="TENTATIVE"),
        ]
        _, diagnostics = tracks_to_drl_vo_ped_map_with_diagnostics(
            tracks, np.asarray([0.0, 0.0, 0.0])
        )
        self.assertEqual(diagnostics["written_track_ids"], [1, 2])
        self.assertEqual(
            diagnostics["excluded_tracks"][0]["reason"],
            "tentative_disabled",
        )
        _, with_tentative = tracks_to_drl_vo_ped_map_with_diagnostics(
            tracks,
            np.asarray([0.0, 0.0, 0.0]),
            include_tentative=True,
        )
        self.assertEqual(with_tentative["written_track_ids"], [1, 2, 3])

    def test_expired_coasting_track_is_dropped(self) -> None:
        _, diagnostics = tracks_to_drl_vo_ped_map_with_diagnostics(
            [
                make_track(
                    1,
                    (1.0, 0.0),
                    state="COASTING",
                    age_s=0.500001,
                )
            ],
            np.asarray([0.0, 0.0, 0.0]),
        )
        self.assertEqual(diagnostics["written_track_ids"], [])
        self.assertEqual(
            diagnostics["excluded_tracks"][0]["reason"],
            "coasting_timeout",
        )

    def test_same_cell_priority_is_deterministic(self) -> None:
        tracks = [
            make_track(
                1,
                (1.01, 0.01),
                (1.0, 0.0),
                state="COASTING",
                age_s=0.1,
                confidence=0.99,
            ),
            make_track(
                9,
                (1.02, 0.02),
                (9.0, 0.0),
                state="CONFIRMED",
                confidence=0.1,
            ),
        ]
        first_map, first = tracks_to_drl_vo_ped_map_with_diagnostics(
            tracks, np.asarray([0.0, 0.0, 0.0])
        )
        second_map, second = tracks_to_drl_vo_ped_map_with_diagnostics(
            list(reversed(tracks)), np.asarray([0.0, 0.0, 0.0])
        )
        np.testing.assert_array_equal(first_map, second_map)
        self.assertEqual(first["written_track_ids"], [9])
        self.assertEqual(second["written_track_ids"], [9])
        self.assertEqual(first["same_cell_conflict_count"], 1)
        self.assertEqual(float(first_map[0, 4, 39]), 9.0)

    def test_nan_inf_protection_returns_finite_map(self) -> None:
        tracks = [
            make_track(1, (math.nan, 0.0), (1.0, 0.0)),
            make_track(2, (1.0, 0.0), (math.inf, 0.0)),
        ]
        pedestrian_map, diagnostics = (
            tracks_to_drl_vo_ped_map_with_diagnostics(
                tracks, np.asarray([0.0, 0.0, 0.0])
            )
        )
        self.assertTrue(np.isfinite(pedestrian_map).all())
        self.assertFalse(np.any(pedestrian_map))
        self.assertEqual(diagnostics["dropped_track_count"], 2)
        invalid_pose_map, _ = tracks_to_drl_vo_ped_map_with_diagnostics(
            [make_track(3, (1.0, 0.0))],
            np.asarray([0.0, math.nan, 0.0]),
        )
        self.assertTrue(np.isfinite(invalid_pose_map).all())
        self.assertFalse(np.any(invalid_pose_map))

    def test_stationary_track_is_written_but_not_expressible(self) -> None:
        pedestrian_map, diagnostics = (
            tracks_to_drl_vo_ped_map_with_diagnostics(
                [make_track(1, (1.0, 0.0), (0.0, 0.0))],
                np.asarray([0.0, 0.0, 0.0]),
            )
        )
        self.assertEqual(diagnostics["written_track_ids"], [1])
        self.assertFalse(np.any(pedestrian_map))

    def test_repeat_run_is_byte_identical(self) -> None:
        tracks = [
            make_track(2, (2.0, 1.0), (0.2, -0.1)),
            make_track(1, (1.0, -1.0), (-0.4, 0.3)),
        ]
        first = tracks_to_drl_vo_ped_map(
            tracks, np.asarray([0.5, -0.5, 0.2])
        )
        second = tracks_to_drl_vo_ped_map(
            tracks, np.asarray([0.5, -0.5, 0.2])
        )
        self.assertEqual(first.tobytes(), second.tobytes())


class StateAndIsolationTests(unittest.TestCase):
    def test_reset_clears_observation_and_tracker_state(self) -> None:
        adapter = ObservationAdapter()
        adapter._scan_history.append(np.ones(720, dtype=np.float32))
        adapter._last_timestamp_ns = 10
        tracker = PedestrianTracker()
        tracker.update(
            [
                MapDetection(
                    position_xy_map=np.asarray([1.0, 0.0]),
                    velocity_xy_map_absolute=np.asarray([0.0, 0.0]),
                    confidence=0.9,
                )
            ],
            10,
        )
        adapter.reset()
        tracker.reset()
        self.assertEqual(adapter._scan_history, [])
        self.assertIsNone(adapter._last_timestamp_ns)
        self.assertEqual(tracker.tracks, [])
        self.assertIsNone(tracker.last_timestamp_ns)

    def test_tracker_rejects_non_increasing_timestamps(self) -> None:
        tracker = PedestrianTracker()
        tracker.update([], 10)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            tracker.update([], 10)

    def test_perception_function_has_no_truth_field_access(self) -> None:
        source = inspect.getsource(infer_tracks_without_ground_truth)
        forbidden = (
            "pedestrian_positions",
            "pedestrian_velocities",
            "semantic_label",
            "pedestrian_xy_map",
            "pedestrian_velocity_map",
        )
        for field in forbidden:
            self.assertNotIn(field, source)


if __name__ == "__main__":
    unittest.main()
