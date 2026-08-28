#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--acceleration-sigmas-mps2, --association-velocity-weights, --confidence-threshold, --confirmed-timeout-s, --dataset-root, --detections-jsonl, --match-gate-m, --max-continuity-gap-s, --output-json, --position-gates-m, --position-measurement-scales, --tentative-timeout-s, --velocity-gates-mps, --velocity-measurement-scales
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON
# 可能使用的关键环境变量：COASTING, CONFIRMED, JSONL, PASS, RMSE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/tune_tracker.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 01:14:20.922816372 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.378546463 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/tune_tracker.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Replay cached truth-free detections to tune the deterministic tracker.

This is an offline evaluator: pedestrian truth is used only after each tracker
update to score a configuration.  The tracker itself receives detections,
timestamps, and robot poses only.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

from .model import DecodedDetection
from .tracker import (
    MapDetection,
    PedestrianTracker,
    detections_base_to_map,
    linear_sum_assignment,
)


@dataclass(frozen=True)
class ReplayFrame:
    episode: tuple[str, int]
    timestamp_ns: int
    detections: tuple[MapDetection, ...]
    truth_ids: tuple[str, ...]
    truth_positions_map: np.ndarray
    truth_velocities_map: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--detections-jsonl", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--confidence-threshold", type=float, default=0.4)
    parser.add_argument("--position-gates-m", default="0.8")
    parser.add_argument("--velocity-gates-mps", default="2.5")
    parser.add_argument("--acceleration-sigmas-mps2", default="3.0")
    parser.add_argument("--position-measurement-scales", default="1.0")
    parser.add_argument("--velocity-measurement-scales", default="1.0")
    parser.add_argument("--association-velocity-weights", default="0.15")
    parser.add_argument("--tentative-timeout-s", type=float, default=0.33)
    parser.add_argument("--confirmed-timeout-s", type=float, default=1.0)
    parser.add_argument("--match-gate-m", type=float, default=0.5)
    parser.add_argument("--max-continuity-gap-s", type=float, default=1.0)
    return parser.parse_args()


def _float_list(text: str) -> List[float]:
    values = sorted({float(item.strip()) for item in text.split(",") if item.strip()})
    if not values or values[0] <= 0.0:
        raise ValueError("grid values must be positive")
    return values


def _nonnegative_float_list(text: str) -> List[float]:
    values = sorted({float(item.strip()) for item in text.split(",") if item.strip()})
    if not values or values[0] < 0.0:
        raise ValueError("grid values must be nonnegative")
    return values


def _load_frames(
    dataset_root: Path,
    detections_jsonl: Path,
    confidence_threshold: float,
) -> List[ReplayFrame]:
    sessions = {
        path.name: path
        for path in dataset_root.resolve().iterdir()
        if path.is_dir() and (path / "metadata.json").is_file()
    }
    frames = []
    with detections_jsonl.open(encoding="utf-8") as predictions:
        for line in predictions:
            record = json.loads(line)
            session_dir = sessions[record["session"]]
            name = record["name"]
            timestamp_ns = int(record["timestamp_ns"])
            robot_pose_map = np.load(
                session_dir / "positions" / name, allow_pickle=False
            ).astype(np.float64)
            decoded = [
                DecodedDetection(
                    position_xy_base=np.asarray(
                        item["position_xy_base"], dtype=np.float64
                    ),
                    velocity_xy_robot_axes_absolute=np.asarray(
                        item["velocity_xy_robot_axes_absolute"], dtype=np.float64
                    ),
                    confidence=float(item["confidence"]),
                )
                for item in record["detections"]
                if float(item["confidence"]) >= confidence_threshold
            ]
            detections = tuple(detections_base_to_map(decoded, robot_pose_map))

            truth_positions = np.load(
                session_dir / "pedestrian_positions" / name, allow_pickle=False
            ).astype(np.float64)
            truth_velocities = np.load(
                session_dir / "pedestrian_velocities" / name, allow_pickle=False
            ).astype(np.float64)
            truth_timestamp_ns = int(
                np.load(
                    session_dir / "pedestrian_truth_timestamps" / name,
                    allow_pickle=False,
                )
            )
            truth_positions += truth_velocities * (
                (timestamp_ns - truth_timestamp_ns) / 1e9
            )
            truth_ids = tuple(
                np.load(
                    session_dir / "pedestrian_ids" / name, allow_pickle=False
                ).astype(str)
            )
            frames.append(
                ReplayFrame(
                    episode=(record["session"], int(record["episode_id"])),
                    timestamp_ns=timestamp_ns,
                    detections=detections,
                    truth_ids=truth_ids,
                    truth_positions_map=truth_positions,
                    truth_velocities_map=truth_velocities,
                )
            )
    if not frames:
        raise ValueError("detections JSONL contains no frames")
    return frames


def _empty_error_stats() -> Dict[str, object]:
    return {
        "matches": 0,
        "position_squared_error": 0.0,
        "velocity_squared_error": 0.0,
        "position_error_sum": np.zeros(2, dtype=np.float64),
        "velocity_error_sum": np.zeros(2, dtype=np.float64),
    }


def _finalize_error_stats(stats: Dict[str, object]) -> Dict[str, object]:
    matches = int(stats["matches"])
    return {
        "matches": matches,
        "position_vector_rmse_m": math.sqrt(
            float(stats["position_squared_error"]) / max(1, matches)
        ),
        "velocity_vector_rmse_mps": math.sqrt(
            float(stats["velocity_squared_error"]) / max(1, matches)
        ),
        "position_bias_xy_m": (
            np.asarray(stats["position_error_sum"]) / max(1, matches)
        ).tolist(),
        "velocity_bias_xy_mps": (
            np.asarray(stats["velocity_error_sum"]) / max(1, matches)
        ).tolist(),
    }


def _score_configuration(
    frames: Sequence[ReplayFrame],
    config: Dict[str, float],
    *,
    tentative_timeout_s: float,
    confirmed_timeout_s: float,
    match_gate_m: float,
    max_continuity_gap_s: float,
) -> Dict[str, object]:
    tracker = PedestrianTracker(
        position_gate_m=config["position_gate_m"],
        velocity_gate_mps=config["velocity_gate_mps"],
        tentative_timeout_s=tentative_timeout_s,
        confirmed_timeout_s=confirmed_timeout_s,
        acceleration_sigma_mps2=config["acceleration_sigma_mps2"],
        position_measurement_scale=config["position_measurement_scale"],
        velocity_measurement_scale=config["velocity_measurement_scale"],
        association_velocity_weight=config["association_velocity_weight"],
    )
    state_stats = {
        "CONFIRMED": _empty_error_stats(),
        "COASTING": _empty_error_stats(),
        "ALL": _empty_error_stats(),
    }
    eligible_snapshots = 0
    matched_snapshots = 0
    id_switches = 0
    last_episode = None
    last_assignment: Dict[str, int] = {}
    last_matched_timestamp: Dict[str, int] = {}

    for frame in frames:
        if frame.episode != last_episode:
            tracker.reset()
            last_episode = frame.episode
            last_assignment = {}
            last_matched_timestamp = {}
        tracks = [
            item
            for item in tracker.update(frame.detections, frame.timestamp_ns)
            if item.track_state in {"CONFIRMED", "COASTING"}
        ]
        eligible_snapshots += len(tracks)
        if tracks:
            predicted_positions = np.asarray(
                [item.position_xy_map for item in tracks], dtype=np.float64
            )
            distances = np.linalg.norm(
                frame.truth_positions_map[:, None, :]
                - predicted_positions[None, :, :],
                axis=2,
            )
            rows, cols = linear_sum_assignment(distances)
            accepted = distances[rows, cols] <= match_gate_m
            rows = rows[accepted]
            cols = cols[accepted]
        else:
            rows = np.empty(0, dtype=np.int64)
            cols = np.empty(0, dtype=np.int64)
        matched_snapshots += len(rows)
        current_assignment = {}
        for row, col in zip(rows.tolist(), cols.tolist()):
            track = tracks[col]
            gt_id = frame.truth_ids[row]
            current_assignment[gt_id] = track.track_id
            gap_s = (
                (frame.timestamp_ns - last_matched_timestamp[gt_id]) / 1e9
                if gt_id in last_matched_timestamp
                else None
            )
            if (
                gt_id in last_assignment
                and last_assignment[gt_id] != track.track_id
                and gap_s is not None
                and gap_s <= max_continuity_gap_s
            ):
                id_switches += 1
            last_matched_timestamp[gt_id] = frame.timestamp_ns

            position_error = (
                np.asarray(track.position_xy_map) - frame.truth_positions_map[row]
            )
            velocity_error = (
                np.asarray(track.velocity_xy_map_absolute)
                - frame.truth_velocities_map[row]
            )
            for state in (track.track_state, "ALL"):
                stats = state_stats[state]
                stats["matches"] = int(stats["matches"]) + 1
                stats["position_squared_error"] = float(
                    stats["position_squared_error"]
                ) + float(position_error @ position_error)
                stats["velocity_squared_error"] = float(
                    stats["velocity_squared_error"]
                ) + float(velocity_error @ velocity_error)
                stats["position_error_sum"] += position_error
                stats["velocity_error_sum"] += velocity_error
        last_assignment = current_assignment

    return {
        "config": config,
        "eligible_confirmed_or_coasting_track_snapshots": eligible_snapshots,
        "matched_track_snapshots": matched_snapshots,
        "matched_track_precision_proxy": matched_snapshots
        / max(1, eligible_snapshots),
        "id_switches_within_continuity_gap": id_switches,
        "by_state": {
            state: _finalize_error_stats(stats)
            for state, stats in state_stats.items()
        },
    }


def main() -> int:
    args = parse_args()
    if args.output_json.exists():
        raise FileExistsError(f"refusing to overwrite {args.output_json}")
    if not 0.0 < args.confidence_threshold < 1.0:
        raise ValueError("confidence_threshold must be inside (0, 1)")
    frames = _load_frames(
        args.dataset_root, args.detections_jsonl, args.confidence_threshold
    )
    grids = [
        _float_list(args.position_gates_m),
        _float_list(args.velocity_gates_mps),
        _float_list(args.acceleration_sigmas_mps2),
        _float_list(args.position_measurement_scales),
        _float_list(args.velocity_measurement_scales),
        _nonnegative_float_list(args.association_velocity_weights),
    ]
    configurations = [
        {
            "position_gate_m": values[0],
            "velocity_gate_mps": values[1],
            "acceleration_sigma_mps2": values[2],
            "position_measurement_scale": values[3],
            "velocity_measurement_scale": values[4],
            "association_velocity_weight": values[5],
        }
        for values in itertools.product(*grids)
    ]
    started = time.perf_counter()
    results = []
    for index, config in enumerate(configurations, start=1):
        result = _score_configuration(
            frames,
            config,
            tentative_timeout_s=args.tentative_timeout_s,
            confirmed_timeout_s=args.confirmed_timeout_s,
            match_gate_m=args.match_gate_m,
            max_continuity_gap_s=args.max_continuity_gap_s,
        )
        results.append(result)
        confirmed = result["by_state"]["CONFIRMED"]
        print(
            f"config={index}/{len(configurations)} "
            f"velocity_rmse={confirmed['velocity_vector_rmse_mps']:.6f} "
            f"position_rmse={confirmed['position_vector_rmse_m']:.6f} "
            f"precision={result['matched_track_precision_proxy']:.6f}",
            flush=True,
        )
    results.sort(
        key=lambda item: (
            item["by_state"]["CONFIRMED"]["velocity_vector_rmse_mps"],
            item["by_state"]["CONFIRMED"]["position_vector_rmse_m"],
            -item["matched_track_precision_proxy"],
            item["id_switches_within_continuity_gap"],
        )
    )
    report = {
        "schema": "dual-lidar-pedestrian-tracker-grid/v1",
        "status": "PASS",
        "dataset_root": str(args.dataset_root.resolve()),
        "detections_jsonl": str(args.detections_jsonl.resolve()),
        "frames": len(frames),
        "confidence_threshold": args.confidence_threshold,
        "tentative_timeout_s": args.tentative_timeout_s,
        "confirmed_timeout_s": args.confirmed_timeout_s,
        "match_gate_m": args.match_gate_m,
        "max_continuity_gap_s": args.max_continuity_gap_s,
        "configurations": len(configurations),
        "elapsed_seconds": time.perf_counter() - started,
        "selection_note": (
            "Rows are ordered by confirmed-track velocity RMSE. Check position "
            "error, precision proxy, matched snapshots, and ID switches before "
            "selecting a deployable configuration."
        ),
        "results": results,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"best": results[0], "output": str(args.output_json)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
