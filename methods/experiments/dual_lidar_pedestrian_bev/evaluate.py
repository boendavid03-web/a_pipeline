#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --checkpoint, --confidence-thresholds, --dataset-root, --device, --max-frames, --nms-radius-m, --num-workers, --output-json, --position-gate-m, --skip-initial-windows-per-episode, --split, --topk
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：CUDA, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 13:03:58.213224779 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.568386320 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/train.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/train.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Evaluate pedestrian position/velocity detections against offline truth."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List

import numpy as np
import torch
from torch.utils.data import DataLoader

from .dataset import BEVSpec, TemporalDualLidarDataset
from .model import TemporalBEVPedestrianDetector, decode_detections
from .tracker import linear_sum_assignment


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "dev", "test"), default="dev")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument(
        "--confidence-thresholds",
        default="0.05,0.10,0.15,0.20,0.25,0.30,0.40,0.50",
    )
    parser.add_argument("--position-gate-m", type=float, default=0.5)
    parser.add_argument("--topk", type=int, default=30)
    parser.add_argument("--nms-radius-m", type=float, default=0.30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument(
        "--skip-initial-windows-per-episode",
        type=int,
        default=0,
        help="Exclude initial model windows so different history lengths can be compared on identical frame names",
    )
    parser.add_argument("--device", default="cuda")
    return parser.parse_args()


def _target_arrays(
    batch: Dict[str, object], batch_index: int, spec: BEVSpec
) -> tuple[np.ndarray, np.ndarray]:
    mask = batch["regression_mask"][batch_index, 0].bool()
    rows, cols = torch.where(mask)
    if len(rows) == 0:
        return np.empty((0, 2)), np.empty((0, 2))
    offsets = batch["offset"][batch_index, :, rows, cols].T.numpy()
    grid = np.column_stack((cols.numpy(), rows.numpy())) + offsets
    positions = spec.grid_to_metric(grid[:, 0], grid[:, 1])
    velocities = batch["velocity"][batch_index, :, rows, cols].T.numpy()
    return positions, velocities


def _new_stats() -> Dict[str, float]:
    return {
        "tp": 0,
        "fp": 0,
        "fn": 0,
        "position_squared_error": 0.0,
        "velocity_squared_error": 0.0,
    }


def _update_stats(
    stats: Dict[str, float],
    gt_positions: np.ndarray,
    gt_velocities: np.ndarray,
    detections,
    position_gate_m: float,
) -> None:
    predicted_positions = np.asarray(
        [item.position_xy_base for item in detections], dtype=np.float64
    ).reshape((-1, 2))
    predicted_velocities = np.asarray(
        [item.velocity_xy_robot_axes_absolute for item in detections],
        dtype=np.float64,
    ).reshape((-1, 2))
    if len(gt_positions) and len(predicted_positions):
        distances = np.linalg.norm(
            gt_positions[:, None, :] - predicted_positions[None, :, :], axis=2
        )
        rows, cols = linear_sum_assignment(distances)
        accepted = distances[rows, cols] <= position_gate_m
        rows = rows[accepted]
        cols = cols[accepted]
    else:
        rows = np.empty(0, dtype=np.int64)
        cols = np.empty(0, dtype=np.int64)
    true_positives = len(rows)
    stats["tp"] += true_positives
    stats["fp"] += len(predicted_positions) - true_positives
    stats["fn"] += len(gt_positions) - true_positives
    if true_positives:
        stats["position_squared_error"] += float(
            np.square(gt_positions[rows] - predicted_positions[cols]).sum()
        )
        stats["velocity_squared_error"] += float(
            np.square(gt_velocities[rows] - predicted_velocities[cols]).sum()
        )


def _finalize(stats: Dict[str, float]) -> Dict[str, float | int]:
    tp = int(stats["tp"])
    fp = int(stats["fp"])
    fn = int(stats["fn"])
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": 2.0 * precision * recall / max(1e-12, precision + recall),
        "position_vector_rmse_m": math.sqrt(
            stats["position_squared_error"] / max(1, tp)
        ),
        "position_coordinate_rmse_m": math.sqrt(
            stats["position_squared_error"] / max(1, 2 * tp)
        ),
        "velocity_vector_rmse_mps": math.sqrt(
            stats["velocity_squared_error"] / max(1, tp)
        ),
        "velocity_coordinate_rmse_mps": math.sqrt(
            stats["velocity_squared_error"] / max(1, 2 * tp)
        ),
    }


def main() -> int:
    args = parse_args()
    if args.output_json.exists():
        raise FileExistsError(f"refusing to overwrite {args.output_json}")
    thresholds = sorted(
        {
            float(value.strip())
            for value in args.confidence_thresholds.split(",")
            if value.strip()
        }
    )
    if not thresholds or thresholds[0] <= 0.0 or thresholds[-1] >= 1.0:
        raise ValueError("confidence thresholds must be inside (0, 1)")
    if args.skip_initial_windows_per_episode < 0:
        raise ValueError("skip_initial_windows_per_episode cannot be negative")
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    device = torch.device(args.device)
    checkpoint = torch.load(
        args.checkpoint, map_location=device, weights_only=False
    )
    spec = BEVSpec(
        float(checkpoint["bev_extent_m"]),
        float(checkpoint["bev_resolution_m"]),
    )
    dataset = TemporalDualLidarDataset(
        args.dataset_root,
        args.split,
        history_frames=int(checkpoint["history_frames"]),
        bev_spec=spec,
        build_targets=True,
        input_encoding=str(checkpoint.get("input_encoding", "occupancy")),
        max_samples=args.max_frames,
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.num_workers > 0,
    )
    model = TemporalBEVPedestrianDetector(
        history_frames=int(checkpoint["history_frames"]),
        base_channels=int(checkpoint["base_channels"]),
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.eval()

    threshold_stats = {threshold: _new_stats() for threshold in thresholds}
    frame_max_scores: List[float] = []
    gt_cell_scores: List[float] = []
    frame_count = 0
    target_count = 0
    episode_window_counts: Dict[tuple[str, int], int] = {}
    with torch.inference_mode():
        for batch_index, batch in enumerate(loader):
            outputs = model(batch["input"].to(device, non_blocking=True))
            scores = torch.sigmoid(outputs["heatmap_logits"])
            candidates = decode_detections(
                outputs,
                spec,
                confidence_threshold=thresholds[0],
                topk=args.topk,
                nms_radius_m=args.nms_radius_m,
            )
            for item_index in range(scores.shape[0]):
                episode = (
                    str(batch["session_name"][item_index]),
                    int(batch["episode_id"][item_index]),
                )
                episode_index = episode_window_counts.get(episode, 0)
                episode_window_counts[episode] = episode_index + 1
                if episode_index < args.skip_initial_windows_per_episode:
                    continue
                frame_max_scores.append(
                    float(scores[item_index].amax().cpu())
                )
                gt_positions, gt_velocities = _target_arrays(
                    batch, item_index, spec
                )
                target_count += len(gt_positions)
                mask = batch["regression_mask"][item_index, 0].bool()
                rows, cols = torch.where(mask)
                if len(rows):
                    gt_cell_scores.extend(
                        scores[
                            item_index,
                            0,
                            rows.to(device),
                            cols.to(device),
                        ]
                        .cpu()
                        .tolist()
                    )
                for threshold in thresholds:
                    accepted = [
                        item
                        for item in candidates[item_index]
                        if item.confidence >= threshold
                    ]
                    _update_stats(
                        threshold_stats[threshold],
                        gt_positions,
                        gt_velocities,
                        accepted,
                        args.position_gate_m,
                    )
                frame_count += 1
            if (batch_index + 1) % 100 == 0:
                print(f"evaluated_frames={frame_count}", flush=True)

    metrics = {
        str(threshold): _finalize(threshold_stats[threshold])
        for threshold in thresholds
    }
    best_threshold = max(
        thresholds, key=lambda threshold: metrics[str(threshold)]["f1"]
    )
    report = {
        "schema": "dual-lidar-pedestrian-bev-evaluation/v1",
        "status": "PASS",
        "dataset_root": str(Path(args.dataset_root).resolve()),
        "checkpoint": str(Path(args.checkpoint).resolve()),
        "checkpoint_epoch": int(checkpoint["epoch"]),
        "checkpoint_dev_loss": float(checkpoint["dev_loss"]),
        "split": args.split,
        "frames": frame_count,
        "ground_truth_targets": target_count,
        "position_match_gate_m": args.position_gate_m,
        "topk": args.topk,
        "nms_radius_m": args.nms_radius_m,
        "skip_initial_windows_per_episode": (
            args.skip_initial_windows_per_episode
        ),
        "score_distributions": {
            "frame_max": {
                "min": float(np.min(frame_max_scores)),
                "median": float(np.median(frame_max_scores)),
                "p90": float(np.quantile(frame_max_scores, 0.9)),
                "p99": float(np.quantile(frame_max_scores, 0.99)),
                "max": float(np.max(frame_max_scores)),
            },
            "gt_cell": {
                "p10": float(np.quantile(gt_cell_scores, 0.1)),
                "median": float(np.median(gt_cell_scores)),
                "p90": float(np.quantile(gt_cell_scores, 0.9)),
            },
        },
        "threshold_metrics": metrics,
        "best_f1_threshold": best_threshold,
        "best_f1_metrics": metrics[str(best_threshold)],
        "test_split_used": args.split == "test",
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
