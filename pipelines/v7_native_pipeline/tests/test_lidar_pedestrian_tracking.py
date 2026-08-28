#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PGM, WORLD, YAML
# 可能使用的关键环境变量：COASTING, CONFIRMED, DELETED, E402, MEASUREMENT_DOUBLE, MEASUREMENT_MERGED_BODY, MEASUREMENT_SINGLE, SCRIPTS_DIR, TENTATIVE_STATIC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_lidar_pedestrian_tracking.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-24 08:10:56.788737276 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:53.597198074 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_lidar_pedestrian_tracking.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Regression tests for the offline dual-lidar pedestrian baseline."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from evaluate_lidar_pedestrian_tracking import VelocityReferenceBuilder  # noqa: E402
from lidar_pedestrian_tracking_core import (  # noqa: E402
    EstimatorFrame,
    OccupancyMap,
    LidarPedestrianEstimator,
    MEASUREMENT_DOUBLE,
    MEASUREMENT_MERGED_BODY,
    MEASUREMENT_SINGLE,
    PersonMeasurement,
    TrackerConfig,
    _Track,
    _measurement_covariance,
)


class _TemporaryMap:
    def __init__(self, *, origin=(-1.0, -4.0, 0.0), resolution=0.05) -> None:
        self._directory = tempfile.TemporaryDirectory()
        root = Path(self._directory.name)
        image = np.full((160, 160), 254, dtype=np.uint8)
        image[[0, -1], :] = 0
        image[:, [0, -1]] = 0
        image[20, 20] = 205
        Image.fromarray(image).save(root / "map.pgm")
        (root / "map.yaml").write_text(
            "\n".join(
                [
                    "image: map.pgm",
                    f"resolution: {resolution}",
                    f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.196",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        self.path = root / "map.yaml"

    def cleanup(self) -> None:
        self._directory.cleanup()


def _frame_from_clusters(
    timestamp: float,
    clusters_xy: list[tuple[float, float]],
    *,
    skew: float = 0.0,
    duplicate_across_sensors: bool = True,
) -> EstimatorFrame:
    points: list[tuple[float, float]] = []
    sources: list[int] = []
    offsets = [
        (-0.018, -0.008),
        (-0.012, 0.008),
        (-0.005, 0.0),
        (0.005, -0.006),
        (0.012, 0.007),
        (0.018, 0.0),
    ]
    for x, y in clusters_xy:
        for index, (dx, dy) in enumerate(offsets):
            points.append((x + dx, y + dy))
            sources.append(index % 2 if duplicate_across_sensors else 0)
    xy = np.asarray(points, dtype=np.float64)
    source = np.asarray(sources, dtype=np.int8)
    ranges = np.hypot(xy[:, 0], xy[:, 1])
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    return EstimatorFrame(
        frame_index=int(round(timestamp * 100.0)),
        scan_ranges=ranges,
        virtual_angles=angles,
        valid_mask=np.ones(ranges.shape, dtype=bool),
        source_sensor=source,
        robot_pose_map=np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float64),
        scan_timestamp_lidar_1=int(round(timestamp * 1e9)),
        scan_timestamp_lidar_2=int(round((timestamp + skew) * 1e9)),
    )


class MapAndFusionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = _TemporaryMap(origin=(1.2, -0.7, 0.37))

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def test_se2_world_grid_round_trip_and_unknown(self) -> None:
        occupancy_map = OccupancyMap.from_yaml(self.fixture.path)
        rows = np.asarray([10, 60, 120], dtype=np.int64)
        cols = np.asarray([15, 70, 130], dtype=np.int64)
        xy = occupancy_map.grid_to_world(rows, cols)
        round_rows, round_cols, inside = occupancy_map.world_to_grid(xy)
        np.testing.assert_array_equal(round_rows, rows)
        np.testing.assert_array_equal(round_cols, cols)
        self.assertTrue(bool(np.all(inside)))
        self.assertTrue(bool(occupancy_map.occupied_mask[0, 0]))
        self.assertTrue(bool(occupancy_map.unknown_mask[20, 20]))
        self.assertFalse(bool(occupancy_map.free_mask[20, 20]))

    def test_sensor_skew_uses_only_newer_scan(self) -> None:
        estimator = LidarPedestrianEstimator(
            OccupancyMap.from_yaml(self.fixture.path),
            TrackerConfig(max_sensor_time_skew_s=0.01, static_distance_threshold_m=0.001),
        )
        frame = _frame_from_clusters(1.0, [(2.0, 0.0)], skew=0.02)
        result = estimator.process(frame)
        self.assertEqual(result.fusion_mode, "single_degraded")
        self.assertEqual(result.active_sensor, 1)
        self.assertGreater(len(result.clusters), 0)
        self.assertTrue(all(cluster.source_mask == 2 for cluster in result.clusters))


class TrackingBehaviorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = _TemporaryMap()
        self.config = TrackerConfig(
            static_distance_threshold_m=0.001,
            cluster_epsilon_m=0.10,
            cluster_min_points=3,
        )

    def tearDown(self) -> None:
        self.fixture.cleanup()

    def _run(self, trajectories: list[list[tuple[float, float]]]):
        estimator = LidarPedestrianEstimator(OccupancyMap.from_yaml(self.fixture.path), self.config)
        results = []
        for frame_index, clusters in enumerate(trajectories):
            results.append(estimator.process(_frame_from_clusters(frame_index * 0.066, clusters)))
        return results

    def test_stationary_single_leg_never_confirms(self) -> None:
        results = self._run([[(2.0, 0.0)]] * 18)
        live_states = {
            track.track_state
            for result in results
            for track in result.tracks
            if track.track_state != "DELETED"
        }
        self.assertIn("TENTATIVE_STATIC", live_states)
        self.assertNotIn("CONFIRMED", live_states)
        self.assertNotIn("COASTING", live_states)
        self.assertTrue(
            any(
                track.track_state == "DELETED"
                for result in results
                for track in result.tracks
            )
        )

    def test_moving_single_leg_confirms(self) -> None:
        results = self._run([[(2.0 + 0.04 * index, 0.0)] for index in range(12)])
        live_states = [
            track.track_state
            for result in results
            for track in result.tracks
            if track.track_state != "DELETED"
        ]
        self.assertIn("CONFIRMED", live_states)

    def test_cluster_use_is_unique_and_conditioned_update_is_direct(self) -> None:
        results = self._run(
            [
                [(2.0, -0.4), (2.0, 0.4)],
                [(2.04, -0.4), (2.04, 0.4)],
                [(2.08, -0.4), (2.08, 0.4)],
            ]
        )
        for result in results:
            referenced = [
                cluster_id
                for measurement in result.measurements
                for cluster_id in measurement.cluster_ids
            ]
            self.assertEqual(len(referenced), len(set(referenced)))
            for association in result.associations:
                if association.stage == "conditioned_measurement":
                    measurement = next(
                        item
                        for item in result.measurements
                        if item.measurement_id == association.measurement_id
                    )
                    self.assertEqual(
                        measurement.conditioned_track_id,
                        association.track_id,
                    )

    def test_covariance_order_and_deterministic_replay(self) -> None:
        self.assertLess(self.config.double_sigma_m, self.config.single_sigma_m)
        self.assertLess(self.config.single_sigma_m, self.config.merged_sigma_m)
        trajectory = [[(2.0 + 0.03 * index, 0.1)] for index in range(10)]
        first = self._run(trajectory)
        second = self._run(trajectory)

        def canonical(results):
            return json.dumps(
                [
                    {
                        "clusters": [
                            [item.cluster_id, np.round(item.centroid, 9).tolist()]
                            for item in result.clusters
                        ],
                        "measurements": [
                            [
                                item.measurement_id,
                                item.mode,
                                list(item.cluster_ids),
                                np.round(item.position, 9).tolist(),
                            ]
                            for item in result.measurements
                        ],
                        "tracks": [
                            [
                                item.track_id,
                                item.track_state,
                                np.round(item.updated_state, 9).tolist(),
                            ]
                            for item in result.tracks
                        ],
                    }
                    for result in results
                ],
                sort_keys=True,
                separators=(",", ":"),
            )

        self.assertEqual(canonical(first), canonical(second))

    def test_measurement_noise_changes_mahalanobis_gate(self) -> None:
        seed = PersonMeasurement(
            measurement_id=1,
            mode=MEASUREMENT_SINGLE,
            cluster_ids=(0,),
            position=np.asarray([0.0, 0.0]),
            covariance=np.eye(2) * 1e-6,
            confidence=0.5,
        )
        track = _Track(1, seed, 0)
        track.covariance[:2, :2] = np.eye(2) * 1e-6
        probe = np.asarray([0.1, 0.0])
        d2_by_mode = {}
        for mode in (
            MEASUREMENT_DOUBLE,
            MEASUREMENT_SINGLE,
            MEASUREMENT_MERGED_BODY,
        ):
            _, d2, _, _ = track.gate(
                probe,
                _measurement_covariance(mode, self.config),
                self.config,
            )
            d2_by_mode[mode] = d2
        self.assertGreater(
            d2_by_mode[MEASUREMENT_DOUBLE],
            d2_by_mode[MEASUREMENT_SINGLE],
        )
        self.assertGreater(
            d2_by_mode[MEASUREMENT_SINGLE],
            d2_by_mode[MEASUREMENT_MERGED_BODY],
        )

    def test_estimator_frame_has_strict_input_boundary(self) -> None:
        self.assertEqual(
            tuple(EstimatorFrame.__dataclass_fields__),
            (
                "frame_index",
                "scan_ranges",
                "virtual_angles",
                "valid_mask",
                "source_sensor",
                "robot_pose_map",
                "scan_timestamp_lidar_1",
                "scan_timestamp_lidar_2",
            ),
        )

    def test_variable_dt_changes_prediction(self) -> None:
        short = self._run(
            [[(2.0, 0.0)], [(2.05, 0.0)], [(2.10, 0.0)], [(2.15, 0.0)]]
        )
        estimator = LidarPedestrianEstimator(OccupancyMap.from_yaml(self.fixture.path), self.config)
        timestamps = [0.0, 0.066, 0.132, 0.330]
        long = [
            estimator.process(_frame_from_clusters(stamp, [(2.0 + 0.05 * index, 0.0)]))
            for index, stamp in enumerate(timestamps)
        ]
        short_prediction = short[-1].tracks[0].predicted_state[0]
        long_prediction = long[-1].tracks[0].predicted_state[0]
        self.assertGreater(long_prediction, short_prediction)


class _FakeReader:
    def __init__(self, timestamps: list[float], positions: list[tuple[float, float]]) -> None:
        self.frames = list(range(len(timestamps)))
        self.scan_1_timestamps = np.asarray(
            [int(round(timestamp * 1e9)) for timestamp in timestamps],
            dtype=np.int64,
        )
        self._truth = [
            {
                "truth_timestamp_ns": int(round(timestamp * 1e9)),
                "ids": np.asarray(["7"]),
                "positions": np.asarray([position], dtype=np.float64),
            }
            for timestamp, position in zip(timestamps, positions)
        ]

    def truth_frame(self, index: int) -> dict:
        return self._truth[index]


class VelocityReferenceTest(unittest.TestCase):
    def test_quadratic_fit_recovers_center_derivative(self) -> None:
        timestamps = np.linspace(0.0, 1.0, 21).tolist()
        positions = [(2.0 * t + 0.5 * t * t, -t) for t in timestamps]
        reference = VelocityReferenceBuilder(
            _FakeReader(timestamps, positions),
            half_window_s=0.25,
            min_samples=5,
            min_span_s=0.30,
            max_gap_s=0.15,
            max_residual_m=0.12,
            high_dynamics_acceleration_mps2=3.0,
        ).reference("7", int(0.5e9))
        self.assertTrue(reference.reference_valid)
        self.assertFalse(reference.edge_reference)
        np.testing.assert_allclose(reference.velocity_map, [2.5, -1.0], atol=1e-7)
        self.assertLess(reference.fit_residual_m, 1e-8)

    def test_long_truth_gap_invalidates_reference(self) -> None:
        timestamps = [0.0, 0.05, 0.10, 0.15, 0.35, 0.40, 0.45, 0.50]
        positions = [(t, 0.0) for t in timestamps]
        reference = VelocityReferenceBuilder(
            _FakeReader(timestamps, positions),
            half_window_s=0.25,
            min_samples=5,
            min_span_s=0.30,
            max_gap_s=0.15,
            max_residual_m=0.12,
            high_dynamics_acceleration_mps2=3.0,
        ).reference("7", int(0.25e9))
        self.assertFalse(reference.reference_valid)
        self.assertGreater(reference.max_time_gap_s, 0.15)


if __name__ == "__main__":
    unittest.main()
