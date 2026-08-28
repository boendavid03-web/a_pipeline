#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --max-continuity-gap-s, --output-json, --position-gate-m, --predictions-jsonl
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON
# 可能使用的关键环境变量：COASTING, CONFIRMED, JSONL, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate_tracks.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 13:09:53.951447641 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.377546444 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate_tracks.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Evaluate persistent IDs in an inference JSONL against offline GT IDs."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from .tracker import linear_sum_assignment


def _new_error_stats() -> dict[str, object]:
    return {
        "matches": 0,
        "position_squared_error": 0.0,
        "velocity_squared_error": 0.0,
        "position_error_sum": np.zeros(2, dtype=np.float64),
        "velocity_error_sum": np.zeros(2, dtype=np.float64),
    }


def _finalize_error_stats(stats: dict[str, object]) -> dict[str, object]:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--predictions-jsonl", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--position-gate-m", type=float, default=0.5)
    parser.add_argument("--max-continuity-gap-s", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output_json.exists():
        raise FileExistsError(f"refusing to overwrite {args.output_json}")
    sessions = {
        path.name: path
        for path in Path(args.dataset_root).resolve().iterdir()
        if path.is_dir() and (path / "metadata.json").is_file()
    }
    current_episode = None
    last_assignment = {}
    last_matched_timestamp = {}
    id_switches = 0
    short_gap_fragments = 0
    matched_track_snapshots = 0
    eligible_track_snapshots = 0
    frames = 0
    confirmed_frames = 0
    track_ids_by_episode = defaultdict(set)
    gt_track_ids = defaultdict(set)
    error_stats = {
        "CONFIRMED": _new_error_stats(),
        "COASTING": _new_error_stats(),
        "ALL": _new_error_stats(),
    }
    with args.predictions_jsonl.open(encoding="utf-8") as predictions:
        for line in predictions:
            record = json.loads(line)
            frames += 1
            episode = (record["session"], int(record["episode_id"]))
            if episode != current_episode:
                current_episode = episode
                last_assignment = {}
                last_matched_timestamp = {}
            session_dir = sessions[record["session"]]
            name = record["name"]
            timestamp_ns = int(record["timestamp_ns"])
            positions = np.load(
                session_dir / "pedestrian_positions" / name, allow_pickle=False
            ).astype(np.float64)
            velocities = np.load(
                session_dir / "pedestrian_velocities" / name, allow_pickle=False
            ).astype(np.float64)
            truth_timestamp_ns = int(
                np.load(
                    session_dir / "pedestrian_truth_timestamps" / name,
                    allow_pickle=False,
                )
            )
            pedestrian_ids = np.load(
                session_dir / "pedestrian_ids" / name, allow_pickle=False
            ).astype(str)
            positions += velocities * (
                (timestamp_ns - truth_timestamp_ns) / 1e9
            )
            tracks = [
                item
                for item in record["tracks"]
                if item["track_state"] in {"CONFIRMED", "COASTING"}
            ]
            eligible_track_snapshots += len(tracks)
            if any(item["track_state"] == "CONFIRMED" for item in tracks):
                confirmed_frames += 1
            for item in tracks:
                track_ids_by_episode[episode].add(int(item["track_id"]))
            if tracks:
                track_positions = np.asarray(
                    [item["position_xy_map"] for item in tracks],
                    dtype=np.float64,
                )
                distances = np.linalg.norm(
                    positions[:, None, :] - track_positions[None, :, :],
                    axis=2,
                )
                rows, cols = linear_sum_assignment(distances)
                accepted = distances[rows, cols] <= args.position_gate_m
                rows = rows[accepted]
                cols = cols[accepted]
            else:
                rows = np.empty(0, dtype=np.int64)
                cols = np.empty(0, dtype=np.int64)
            matched_track_snapshots += len(rows)
            current_assignment = {}
            for row, col in zip(rows, cols):
                pedestrian_id = str(pedestrian_ids[row])
                track_id = int(tracks[col]["track_id"])
                position_error = (
                    np.asarray(tracks[col]["position_xy_map"], dtype=np.float64)
                    - positions[row]
                )
                velocity_error = (
                    np.asarray(
                        tracks[col]["velocity_xy_map_absolute"],
                        dtype=np.float64,
                    )
                    - velocities[row]
                )
                for state in (tracks[col]["track_state"], "ALL"):
                    stats = error_stats[state]
                    stats["matches"] = int(stats["matches"]) + 1
                    stats["position_squared_error"] = float(
                        stats["position_squared_error"]
                    ) + float(position_error @ position_error)
                    stats["velocity_squared_error"] = float(
                        stats["velocity_squared_error"]
                    ) + float(velocity_error @ velocity_error)
                    stats["position_error_sum"] += position_error
                    stats["velocity_error_sum"] += velocity_error
                current_assignment[pedestrian_id] = track_id
                gt_track_ids[(episode, pedestrian_id)].add(track_id)
                gap_s = (
                    (timestamp_ns - last_matched_timestamp[pedestrian_id]) / 1e9
                    if pedestrian_id in last_matched_timestamp
                    else None
                )
                if (
                    pedestrian_id in last_assignment
                    and last_assignment[pedestrian_id] != track_id
                    and gap_s is not None
                    and gap_s <= args.max_continuity_gap_s
                ):
                    id_switches += 1
                if (
                    pedestrian_id in last_matched_timestamp
                    and pedestrian_id not in last_assignment
                    and gap_s is not None
                    and gap_s <= args.max_continuity_gap_s
                ):
                    short_gap_fragments += 1
                last_matched_timestamp[pedestrian_id] = timestamp_ns
            last_assignment = current_assignment

    track_counts_per_gt = [len(value) for value in gt_track_ids.values()]
    report = {
        "schema": "dual-lidar-pedestrian-track-evaluation/v1",
        "status": "PASS",
        "predictions_jsonl": str(args.predictions_jsonl.resolve()),
        "frames": frames,
        "position_match_gate_m": args.position_gate_m,
        "max_continuity_gap_s": args.max_continuity_gap_s,
        "confirmed_frames": confirmed_frames,
        "eligible_confirmed_or_coasting_track_snapshots": eligible_track_snapshots,
        "matched_track_snapshots": matched_track_snapshots,
        "matched_track_precision_proxy": (
            matched_track_snapshots / max(1, eligible_track_snapshots)
        ),
        "id_switches_within_continuity_gap": id_switches,
        "short_gap_fragments": short_gap_fragments,
        "confirmed_track_ids_total_across_episodes": sum(
            len(value) for value in track_ids_by_episode.values()
        ),
        "persistent_gt_ids_matched": len(gt_track_ids),
        "gt_ids_with_multiple_track_ids": sum(
            count > 1 for count in track_counts_per_gt
        ),
        "mean_track_ids_per_matched_gt": float(
            np.mean(track_counts_per_gt) if track_counts_per_gt else 0.0
        ),
        "position_velocity_errors_by_state": {
            state: _finalize_error_stats(stats)
            for state, stats in error_stats.items()
        },
        "note": (
            "Precision proxy matches confirmed/coasting track snapshots to all "
            "GT centers. Recall is intentionally omitted because this report "
            "does not put invisible GT pedestrians in the denominator."
        ),
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
