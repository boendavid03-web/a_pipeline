#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT
# 可能使用的关键环境变量：MODULE, MODULE_PATH, NANOSECONDS_PER_SECOND, PROJECT_ROOT, SPEC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_drl_vo_fixed_dual_helpers.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 23:39:37.231401963 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:22.376915220 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/test/test_drl_vo_fixed_dual_helpers.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_fixed_dual_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import importlib.util
import inspect
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "drl_vo_fixed_dual_inference_node.py"
)
if str(MODULE_PATH.parent) not in sys.path:
    sys.path.insert(0, str(MODULE_PATH.parent))
SPEC = importlib.util.spec_from_file_location(
    "drl_vo_fixed_dual_inference_node",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class DrlVoFixedDualHelperTests(unittest.TestCase):
    def test_dr_spaam_is_an_external_track_source(self):
        self.assertIn("dr_spaam", MODULE.EXTERNAL_TRACK_SOURCES)
        self.assertIn("tracks", MODULE.EXTERNAL_TRACK_SOURCES)
        self.assertNotIn("oracle", MODULE.EXTERNAL_TRACK_SOURCES)

    def test_external_tracks_adapt_to_existing_pedestrian_map(self):
        track = SimpleNamespace(
            track_id=7,
            position=SimpleNamespace(x=1.0, y=0.0),
            velocity=SimpleNamespace(x=0.5, y=-0.25),
            confidence=0.98,
            state="CONFIRMED",
            time_since_update=0.0,
        )
        records = MODULE.tracked_pedestrian_records(
            SimpleNamespace(tracks=[track])
        )
        pedestrian_map, diagnostics = (
            MODULE.tracks_to_drl_vo_ped_map_with_diagnostics(
                records,
                np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            )
        )
        self.assertEqual(diagnostics["written_track_ids"], [7])
        self.assertEqual(pedestrian_map.shape, (2, 80, 80))
        self.assertAlmostEqual(float(pedestrian_map[0, 4, 40]), 0.5)
        self.assertAlmostEqual(float(pedestrian_map[1, 4, 40]), -0.25)

    def test_external_track_callback_has_no_ground_truth_access(self):
        source = inspect.getsource(
            MODULE.DrlVoFixedDualInference.pedestrian_tracks_callback
        )
        self.assertNotIn("ground_truth", source)
        self.assertNotIn("pedestrian_xy", source)
        self.assertNotIn("pedestrian_velocity", source)

    def test_freshness_and_clock_rollback(self):
        second = MODULE.NANOSECONDS_PER_SECOND
        self.assertTrue(MODULE.time_is_fresh(2 * second, second, 1.0))
        self.assertFalse(
            MODULE.time_is_fresh(2 * second, second - 1, 1.0)
        )
        self.assertFalse(MODULE.time_is_fresh(second, second + 1, 1.0))
        self.assertTrue(MODULE.clock_rolled_back(101, 100))
        self.assertFalse(MODULE.clock_rolled_back(100, 100))

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
        self.assertEqual(
            MODULE.latest_causal_sample(
                [
                    (boundary_stamp - 200_000_000, "newer"),
                    (boundary_stamp - 300_000_000, "boundary"),
                ],
                boundary_stamp,
                0.3,
            ),
            ("newer", boundary_stamp - 200_000_000),
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

    def test_observation_contract_is_exact(self):
        pedestrian_map = np.full((2, 80, 80), 2.0, dtype=np.float32)
        scan_history = np.full((10, 720), 15.0, dtype=np.float32)
        observation = MODULE.build_observation(
            pedestrian_map,
            scan_history,
            np.asarray([2.0, -2.0], dtype=np.float32),
        )
        self.assertEqual(observation.shape, (19202,))
        self.assertTrue(np.isfinite(observation).all())
        np.testing.assert_array_equal(observation[:12800], 1.0)
        np.testing.assert_array_equal(observation[12800:19200], 0.0)
        np.testing.assert_array_equal(observation[-2:], [1.0, -1.0])

    def test_temporal_perception_bev_contract(self):
        frames = []
        for timestamp in range(1, 13):
            frames.append(
                MODULE.TemporalLidarFrame(
                    points_xy_base=np.asarray(
                        [[1.0, 0.0], [0.0, 1.0]], dtype=np.float32
                    ),
                    sensor_indices=np.asarray([0, 1], dtype=np.int64),
                    robot_pose_map=np.asarray(
                        [0.0, 0.0, 0.0], dtype=np.float32
                    ),
                    timestamp_ns=timestamp,
                )
            )
        bev = MODULE.build_temporal_lidar_bev(
            frames, MODULE.BEVSpec(8.0, 0.1), 12
        )
        self.assertEqual(bev.shape, (24, 160, 160))
        self.assertEqual(bev.dtype, np.float32)
        self.assertTrue(np.isfinite(bev).all())
        self.assertGreater(float(bev[0].sum()), 0.0)
        self.assertGreater(float(bev[1].sum()), 0.0)

    def test_online_perception_helper_has_no_truth_field_access(self):
        source = inspect.getsource(
            MODULE.DrlVoFixedDualInference._predicted_pedestrian_map
        )
        for forbidden in (
            "pedestrian_xy",
            "pedestrian_yaw",
            "pedestrian_velocity",
            "semantic_label",
        ):
            self.assertNotIn(forbidden, source)

    def test_empty_front_is_explicit(self):
        ranges = np.asarray([np.nan, 1.0], dtype=np.float32)
        angles = np.asarray([0.0, 1.0], dtype=np.float32)
        valid = np.asarray([False, True])
        self.assertIsNone(
            MODULE.minimum_front_range(ranges, angles, valid, 0.35)
        )
        self.assertAlmostEqual(
            MODULE.minimum_front_range(
                np.asarray([0.7], dtype=np.float32),
                np.asarray([0.1], dtype=np.float32),
                np.asarray([True]),
                0.35,
            ),
            0.7,
            places=6,
        )

    def test_positive_infinity_is_open_space_at_effective_range_max(self):
        ranges, angles, valid, footprint = MODULE.transform_scan_to_base(
            np.full(5, np.inf, dtype=np.float32),
            angle_min=-0.2,
            angle_increment=0.1,
            scan_range_min=0.05,
            scan_range_max=12.0,
            translation=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            configured_range_min=0.1,
            configured_range_max=8.0,
            frozen_self_mask=None,
        )
        np.testing.assert_array_equal(valid, np.ones(5, dtype=np.bool_))
        np.testing.assert_array_equal(footprint, np.zeros(5, dtype=np.bool_))
        np.testing.assert_allclose(ranges, 8.0, atol=1e-6)
        np.testing.assert_allclose(
            angles,
            np.asarray([-0.2, -0.1, 0.0, 0.1, 0.2]),
            atol=1e-6,
        )
        self.assertAlmostEqual(
            MODULE.minimum_front_range(ranges, angles, valid, 0.35),
            8.0,
            places=6,
        )
        _raw, command, stopped = MODULE.limit_physical_action(
            np.asarray([0.0, 0.0], dtype=np.float32),
            max_linear=0.3,
            max_angular=1.5,
            front_min=8.0,
            front_stop_distance=0.5,
            local_goal_y=0.0,
            front_stop_angular_deadband=0.05,
            front_stop_min_angular=0.35,
        )
        self.assertFalse(stopped)
        self.assertGreater(float(command[0]), 0.0)

    def test_nan_negative_infinity_and_empty_scan_remain_fail_safe(self):
        for raw in (
            np.full(3, np.nan, dtype=np.float32),
            np.full(3, -np.inf, dtype=np.float32),
            np.empty(0, dtype=np.float32),
        ):
            ranges, angles, valid, _footprint = (
                MODULE.transform_scan_to_base(
                    raw,
                    angle_min=-0.1,
                    angle_increment=0.1,
                    scan_range_min=0.05,
                    scan_range_max=12.0,
                    translation=(0.0, 0.0, 0.0),
                    quaternion=(0.0, 0.0, 0.0, 1.0),
                    configured_range_min=0.1,
                    configured_range_max=8.0,
                    frozen_self_mask=None,
                )
            )
            self.assertFalse(np.any(valid))
            self.assertIsNone(
                MODULE.minimum_front_range(ranges, angles, valid, 0.35)
            )

    def test_stale_scan_stamp_remains_fail_safe(self):
        second = MODULE.NANOSECONDS_PER_SECOND
        self.assertFalse(
            MODULE.time_is_fresh(
                reference_ns=2 * second,
                stamp_ns=second,
                timeout_seconds=0.5,
            )
        )

    def test_fixed_self_mask_and_virtualization(self):
        ranges, angles, valid, footprint = MODULE.transform_scan_to_base(
            np.asarray([0.2, 1.0], dtype=np.float32),
            angle_min=0.0,
            angle_increment=0.0,
            scan_range_min=0.1,
            scan_range_max=8.0,
            translation=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            configured_range_min=0.1,
            configured_range_max=8.0,
            frozen_self_mask=None,
        )
        np.testing.assert_array_equal(footprint, [True, False])
        np.testing.assert_array_equal(valid, [False, True])
        self.assertTrue(math.isnan(float(ranges[0])))
        self.assertAlmostEqual(float(ranges[1]), 1.0, places=6)
        self.assertAlmostEqual(float(angles[1]), 0.0, places=6)

    def test_oracle_leg_overlays_person_label(self):
        label_image = np.zeros((10, 10), dtype=np.int64)
        unknown = MODULE.semantic_labels_for_virtual_scan(
            np.asarray([1.0], dtype=np.float32),
            np.asarray([0.0], dtype=np.float32),
            np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            label_image,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            pedestrian_xy=np.empty((0, 2), dtype=np.float32),
            pedestrian_yaw=np.empty(0, dtype=np.float32),
            static_filter_radius=0,
        )
        self.assertEqual(int(unknown[0]), -1)

        person = MODULE.semantic_labels_for_virtual_scan(
            np.asarray([1.0], dtype=np.float32),
            np.asarray([0.0], dtype=np.float32),
            np.asarray([0.0, 0.0, 0.0], dtype=np.float32),
            label_image,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            pedestrian_xy=np.asarray([[1.0, 0.07]], dtype=np.float32),
            pedestrian_yaw=np.asarray([0.0], dtype=np.float32),
            static_filter_radius=0,
        )
        self.assertEqual(int(person[0]), 6)

    def test_front_stop_overrides_forward_action(self):
        ranges, angles, valid, _footprint = MODULE.transform_scan_to_base(
            np.asarray([np.inf, 0.4, np.inf], dtype=np.float32),
            angle_min=-0.1,
            angle_increment=0.1,
            scan_range_min=0.1,
            scan_range_max=8.0,
            translation=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            configured_range_min=0.1,
            configured_range_max=8.0,
            frozen_self_mask=None,
        )
        front_min = MODULE.minimum_front_range(
            ranges,
            angles,
            valid,
            0.35,
        )
        self.assertAlmostEqual(front_min, 0.4, places=6)
        raw, command, stopped = MODULE.limit_physical_action(
            np.asarray([0.0, 0.0], dtype=np.float32),
            max_linear=0.3,
            max_angular=1.5,
            front_min=front_min,
            front_stop_distance=0.5,
            local_goal_y=-1.0,
            front_stop_angular_deadband=0.05,
            front_stop_min_angular=0.35,
        )
        np.testing.assert_allclose(raw, [0.25, 0.0], atol=1e-7)
        np.testing.assert_allclose(command, [0.0, -0.35], atol=1e-7)
        self.assertTrue(stopped)

    def test_actuation_deadlock_requires_full_frozen_window(self):
        samples = [
            MODULE.ActuationSample(
                stamp_ns=int(index * 0.5e9),
                command_linear=0.0,
                command_angular=0.105,
                x=2.0,
                y=2.0,
                yaw=0.0,
            )
            for index in range(6)
        ]
        self.assertTrue(
            MODULE.actuation_deadlock_detected(
                samples,
                minimum_window_sec=2.5,
                minimum_command_ratio=0.8,
                linear_command_threshold=0.02,
                angular_command_threshold=0.05,
                maximum_displacement_m=0.02,
                maximum_yaw_progress_rad=0.03,
            )
        )
        self.assertFalse(
            MODULE.actuation_deadlock_detected(
                samples[:-1],
                minimum_window_sec=2.5,
                minimum_command_ratio=0.8,
                linear_command_threshold=0.02,
                angular_command_threshold=0.05,
                maximum_displacement_m=0.02,
                maximum_yaw_progress_rad=0.03,
            )
        )

    def test_actuation_response_prevents_false_deadlock(self):
        base = [
            MODULE.ActuationSample(
                stamp_ns=int(index * 0.5e9),
                command_linear=0.0,
                command_angular=0.35,
                x=2.0,
                y=2.0,
                yaw=0.01 * index,
            )
            for index in range(6)
        ]
        self.assertFalse(
            MODULE.actuation_deadlock_detected(
                base,
                2.5,
                0.8,
                0.02,
                0.05,
                0.02,
                0.03,
            )
        )
        translated = [
            MODULE.ActuationSample(
                **{
                    **sample.__dict__,
                    "x": 2.0 + 0.005 * index,
                    "yaw": 0.0,
                }
            )
            for index, sample in enumerate(base)
        ]
        self.assertFalse(
            MODULE.actuation_deadlock_detected(
                translated,
                2.5,
                0.8,
                0.02,
                0.05,
                0.02,
                0.03,
            )
        )

    def test_actuation_deadlock_command_ratio_is_effective(self):
        samples = [
            MODULE.ActuationSample(
                stamp_ns=int(index * 0.625e9),
                command_linear=0.0,
                command_angular=0.105 if index != 2 else 0.0,
                x=2.0,
                y=2.0,
                yaw=0.0,
            )
            for index in range(5)
        ]
        self.assertTrue(
            MODULE.actuation_deadlock_detected(
                samples, 2.5, 0.8, 0.02, 0.05, 0.02, 0.03
            )
        )
        two_inactive = [
            MODULE.ActuationSample(
                **{
                    **sample.__dict__,
                    "command_angular": (
                        0.0 if index in (1, 2) else sample.command_angular
                    ),
                }
            )
            for index, sample in enumerate(samples)
        ]
        self.assertFalse(
            MODULE.actuation_deadlock_detected(
                two_inactive, 2.5, 0.8, 0.02, 0.05, 0.02, 0.03
            )
        )
    def test_actuation_deadlock_detector_is_wired_to_control_loop(self):
        scan_source = inspect.getsource(
            MODULE.DrlVoFixedDualInference.scan_callback
        )
        reset_source = inspect.getsource(
            MODULE.DrlVoFixedDualInference.episode_reset_callback
        )
        self.assertIn("_observe_actuation_deadlock", scan_source)
        self.assertIn("_clear_actuation_deadlock_state", reset_source)

    def test_episode_reset_rearm_is_wired_to_fresh_goal_contract(self):
        reset_source = inspect.getsource(
            MODULE.DrlVoFixedDualInference.episode_reset_callback
        )
        goal_source = inspect.getsource(
            MODULE.DrlVoFixedDualInference.final_goal_callback
        )
        self.assertIn("actions_inhibited_after_reset = True", reset_source)
        self.assertIn("final_goal_rearms_after_reset", goal_source)
        self.assertIn("actions_inhibited_after_reset = False", goal_source)

    def test_episode_reset_keeps_replayed_goal_locked_and_new_goal_rearms(self):
        calls = []

        class Logger:
            def info(self, message):
                calls.append(("info", message))

            def error(self, message):
                calls.append(("error", message))

        fake = SimpleNamespace(
            final_goal=np.asarray([1.0, 2.0], dtype=np.float32),
            final_goal_stamp_ns=0,
            reset_goal=None,
            actions_inhibited_after_reset=False,
            subgoal=np.asarray([0.5, 0.0], dtype=np.float32),
            subgoal_stamp_ns=10,
            subgoal_history=[(10, np.asarray([0.5, 0.0], dtype=np.float32))],
            last_scan_clock_ns=123,
            _observe_clock=lambda: None,
            _clear_actuation_deadlock_state=lambda: calls.append("deadlock"),
            _clear_history=lambda: calls.append("history"),
            publish_stop=lambda _reason="stop": calls.append("stop"),
            get_parameter=lambda name: SimpleNamespace(value="map"),
            get_logger=lambda: Logger(),
        )
        def clear_subgoal_state():
            calls.append("subgoal")
            fake.subgoal = None
            fake.subgoal_stamp_ns = None
            fake.subgoal_history.clear()

        fake._clear_subgoal_state = clear_subgoal_state
        MODULE.DrlVoFixedDualInference.episode_reset_callback(
            fake, MODULE.Empty()
        )
        self.assertTrue(fake.actions_inhibited_after_reset)
        np.testing.assert_array_equal(fake.reset_goal, [1.0, 2.0])
        self.assertIsNone(fake.last_scan_clock_ns)
        self.assertIsNone(fake.subgoal)
        self.assertIsNone(fake.subgoal_stamp_ns)
        self.assertEqual(fake.subgoal_history, [])

        replay = MODULE.PointStamped()
        replay.header.frame_id = "map"
        replay.point.x = 1.0
        replay.point.y = 2.0
        MODULE.DrlVoFixedDualInference.final_goal_callback(fake, replay)
        self.assertTrue(fake.actions_inhibited_after_reset)
        np.testing.assert_array_equal(fake.reset_goal, [1.0, 2.0])

        fresh = MODULE.PointStamped()
        fresh.header.frame_id = "map"
        fresh.point.x = 1.001
        fresh.point.y = 2.0
        fake.subgoal = np.asarray([0.25, 0.0], dtype=np.float32)
        fake.subgoal_stamp_ns = 20
        fake.subgoal_history = [
            (20, np.asarray([0.25, 0.0], dtype=np.float32))
        ]
        subgoal_calls_before = calls.count("subgoal")
        MODULE.DrlVoFixedDualInference.final_goal_callback(fake, fresh)
        self.assertFalse(fake.actions_inhibited_after_reset)
        self.assertIsNone(fake.reset_goal)
        self.assertEqual(calls.count("subgoal"), subgoal_calls_before)
        np.testing.assert_array_equal(fake.subgoal, [0.25, 0.0])
        self.assertEqual(fake.subgoal_stamp_ns, 20)
        self.assertEqual(len(fake.subgoal_history), 1)
        self.assertTrue(
            any(
                isinstance(call, tuple)
                and call[0] == "info"
                and "re-armed" in call[1]
                for call in calls
            )
        )

    def test_changed_valid_goal_preserves_subgoal_state(self):
        calls = []
        fake = SimpleNamespace(
            final_goal=np.asarray([1.0, 2.0], dtype=np.float32),
            final_goal_stamp_ns=0,
            actions_inhibited_after_reset=False,
            subgoal=np.asarray([0.5, 0.0], dtype=np.float32),
            subgoal_stamp_ns=10,
            subgoal_history=[(10, np.asarray([0.5, 0.0], dtype=np.float32))],
            _observe_clock=lambda: None,
            _clear_history=lambda: calls.append("history"),
            _clear_subgoal_state=lambda: calls.append("subgoal"),
            publish_stop=lambda _reason="stop": calls.append("stop"),
            get_parameter=lambda name: SimpleNamespace(value="map"),
            get_logger=lambda: SimpleNamespace(error=lambda message: None),
        )
        changed = MODULE.PointStamped()
        changed.header.frame_id = "map"
        changed.point.x = 1.5
        changed.point.y = 2.0

        MODULE.DrlVoFixedDualInference.final_goal_callback(fake, changed)

        self.assertEqual(calls, ["history", "stop"])
        np.testing.assert_array_equal(fake.subgoal, [0.5, 0.0])
        self.assertEqual(fake.subgoal_stamp_ns, 10)
        self.assertEqual(len(fake.subgoal_history), 1)

    def test_formal_checkpoints_strict_load(self):
        task_root = (
            MODULE.PROJECT_ROOT
            / "runs"
            / "20260717_042135_v7_dual"
            / "datasets"
            / "20260727_three_bag_online_seed_split_v1"
            / "training"
            / "drl_vo"
        )
        base_path = (
            task_root
            / "base_bc"
            / "20260727_114455"
            / "checkpoints"
            / "best.pt"
        )
        semantic_path = (
            task_root
            / "semantic_bc"
            / "20260727_115227"
            / "checkpoints"
            / "best.pt"
        )
        original_path = (
            MODULE.PROJECT_ROOT
            / "github_src"
            / "drl_vo_nav-drl_vo"
            / "drl_vo"
            / "src"
            / "model"
            / "drl_vo.zip"
        )
        perception_path = (
            MODULE.PROJECT_ROOT
            / "runs"
            / "20260717_042135_v7_dual"
            / "training"
            / "dual_lidar_pedestrian_bev"
            / "20260731_opt_velw100_h12_c24_v1"
            / "checkpoints"
            / "epoch_014.pt"
        )
        original, original_count = MODULE.load_policy_checkpoint(
            original_path,
            "original",
            "cpu",
        )
        base, base_count = MODULE.load_policy_checkpoint(
            base_path,
            "base",
            "cpu",
        )
        semantic, semantic_count = MODULE.load_policy_checkpoint(
            semantic_path,
            "semantic",
            "cpu",
        )
        perception, bev_spec, history_frames = (
            MODULE.load_perception_checkpoint(perception_path, "cpu")
        )
        self.assertEqual(original_count, 163)
        self.assertEqual(base_count, 163)
        self.assertEqual(semantic_count, 172)
        self.assertEqual(history_frames, 12)
        self.assertEqual(bev_spec.size, 160)
        observation = torch.zeros((1, 19202), dtype=torch.float32)
        semantic_map = torch.full((1, 80, 80), -1, dtype=torch.int64)
        with torch.inference_mode():
            original_action = original.deterministic_action(observation)
            base_action = base.deterministic_action(observation)
            semantic_action = semantic.deterministic_action(
                observation,
                semantic_map,
            )
            perception_outputs = perception(
                torch.zeros((1, 24, 160, 160), dtype=torch.float32)
            )
        self.assertEqual(tuple(original_action.shape), (1, 2))
        self.assertEqual(tuple(base_action.shape), (1, 2))
        self.assertEqual(tuple(semantic_action.shape), (1, 2))
        self.assertTrue(torch.isfinite(base_action).all())
        self.assertTrue(torch.isfinite(semantic_action).all())
        self.assertTrue(
            all(torch.isfinite(value).all() for value in perception_outputs.values())
        )


if __name__ == "__main__":
    unittest.main()
