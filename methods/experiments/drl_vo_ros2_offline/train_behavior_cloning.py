#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --block-size, --drop-pedestrian-velocity, --epochs, --feature-batch-size, --include-semantics, --learning-rate, --model, --output-root, --patience, --purge-frames, --replay-dir, --seed, --semantic-num-classes, --semantic-person-class, --smoke, --use-semantics, --zero-goal, --zero-sensors
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, NPZ, PT, TXT
# 可能使用的关键环境变量：FAIL, PASS, PEDESTRIAN_OBSERVATION_SIZE, SPLIT_CYCLE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 11:47:13.850251671 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/replay.py（导入其函数、类或模型）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_semantic_no_ped_overnight.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/replay.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/run_semantic_no_ped_overnight.sh; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_formal_training_queue.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Offline behavior-cloning feasibility test for the adapted DRL-VO policy."""

from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import random
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
from torch import nn

from drlvo_model import load_policy_strict, load_semantic_policy


SPLIT_CYCLE = ("train", "train", "train", "val", "test")
PEDESTRIAN_OBSERVATION_SIZE = 2 * 80 * 80


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--feature-batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--block-size", type=int, default=100)
    parser.add_argument("--purge-frames", type=int, default=20)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Treat finite optimization/checkpoint contracts as gates; report quality metrics diagnostically.",
    )
    parser.add_argument(
        "--zero-goal",
        action="store_true",
        help="Set the two goal inputs to zero for a sensor-only ablation.",
    )
    parser.add_argument(
        "--zero-sensors",
        action="store_true",
        help="Set pedestrian and lidar inputs to zero for a goal-only ablation.",
    )
    parser.add_argument(
        "--use-semantics",
        action="store_true",
        help="Train the optional semantic late-fusion branch from replay semantic_maps.",
    )
    parser.add_argument(
        "--drop-pedestrian-velocity",
        action="store_true",
        help=(
            "Zero both 80x80 pedestrian vx/vy channels so categorical semantics "
            "are the policy's only pedestrian input. Requires --use-semantics."
        ),
    )
    parser.add_argument(
        "--semantic-num-classes",
        type=int,
        default=7,
        help="Number of categorical labels; valid map values are -1 and 0..N-1.",
    )
    parser.add_argument(
        "--semantic-person-class",
        type=int,
        help="Optional Person class ID for a ground-truth Person masking ablation.",
    )
    return parser.parse_args()


def make_blocked_split(
    sample_count: int,
    block_size: int,
    purge_frames: int,
) -> dict[str, np.ndarray]:
    if block_size <= 2 * purge_frames:
        raise ValueError("block_size must exceed twice purge_frames")
    result: dict[str, list[int]] = {"train": [], "val": [], "test": []}
    for block_index, start in enumerate(range(0, sample_count, block_size)):
        end = min(start + block_size, sample_count)
        split = SPLIT_CYCLE[block_index % len(SPLIT_CYCLE)]
        kept_start = start + (purge_frames if start > 0 else 0)
        kept_end = end - (purge_frames if end < sample_count else 0)
        result[split].extend(range(kept_start, max(kept_start, kept_end)))
    arrays = {key: np.asarray(value, dtype=np.int64) for key, value in result.items()}
    if any(not len(indices) for indices in arrays.values()):
        raise ValueError(f"Empty blocked split: { {k: len(v) for k, v in arrays.items()} }")
    return arrays


def seed_split_indices(labels: np.ndarray) -> dict[str, np.ndarray]:
    labels = np.asarray(labels).astype(str)
    if set(labels.tolist()) != {"train", "dev", "test"}:
        raise ValueError(
            "replay split_labels must contain train, dev, and test"
        )
    return {
        "train": np.flatnonzero(labels == "train").astype(np.int64),
        "val": np.flatnonzero(labels == "dev").astype(np.int64),
        "test": np.flatnonzero(labels == "test").astype(np.int64),
    }


def read_recorded_actions(predictions_csv: Path) -> np.ndarray:
    actions = []
    with predictions_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            actions.append(
                [float(row["recorded_linear_x"]), float(row["recorded_angular_z"])]
            )
    return np.asarray(actions, dtype=np.float32)


def physical_to_normalized(recorded: np.ndarray) -> np.ndarray:
    result = np.empty_like(recorded, dtype=np.float32)
    result[:, 0] = np.clip(4.0 * recorded[:, 0] - 1.0, -1.0, 1.0)
    result[:, 1] = np.clip(recorded[:, 1] / 2.0, -1.0, 1.0)
    return result


def remove_pedestrian_velocity(observations: np.ndarray) -> np.ndarray:
    if observations.ndim != 2 or observations.shape[1] != 19202:
        raise ValueError(
            f"observations must have shape (N, 19202), got {observations.shape}"
        )
    result = observations.copy()
    result[:, :PEDESTRIAN_OBSERVATION_SIZE] = 0.0
    return result


def normalized_to_physical(actions: np.ndarray) -> np.ndarray:
    clipped = np.clip(actions, -1.0, 1.0)
    return np.column_stack(((clipped[:, 0] + 1.0) * 0.25, clipped[:, 1] * 2.0))


def cache_features(
    feature_extractor: nn.Module,
    observations: np.ndarray,
    batch_size: int,
) -> np.ndarray:
    chunks = []
    feature_extractor.eval()
    with torch.inference_mode():
        for start in range(0, len(observations), batch_size):
            batch = torch.from_numpy(observations[start : start + batch_size])
            chunks.append(feature_extractor(batch).numpy())
    return np.concatenate(chunks).astype(np.float32)


def class_balanced_weights(targets: np.ndarray, indices: np.ndarray) -> np.ndarray:
    labels = [tuple(np.round(row, decimals=3)) for row in targets[indices]]
    counts = Counter(labels)
    class_count = len(counts)
    weights = np.asarray(
        [
            math.sqrt(len(labels) / (class_count * counts[label]))
            for label in labels
        ],
        dtype=np.float32,
    )
    return weights / np.mean(weights)


def predict_head(
    policy_net: nn.Module,
    action_net: nn.Module,
    features: np.ndarray,
    semantic_fusion: nn.Module | None = None,
    semantic_maps: np.ndarray | None = None,
    batch_size: int = 256,
) -> np.ndarray:
    if (semantic_fusion is None) != (semantic_maps is None):
        raise ValueError("semantic_fusion and semantic_maps must be provided together")
    chunks = []
    policy_net.eval()
    action_net.eval()
    if semantic_fusion is not None:
        semantic_fusion.eval()
    with torch.inference_mode():
        for start in range(0, len(features), batch_size):
            batch = torch.from_numpy(features[start : start + batch_size])
            if semantic_fusion is not None and semantic_maps is not None:
                semantic_batch = torch.from_numpy(
                    semantic_maps[start : start + batch_size]
                )
                batch = semantic_fusion(batch, semantic_batch)
            chunks.append(action_net(policy_net(batch)).numpy())
    return np.concatenate(chunks)


def split_metrics(
    predicted_normalized: np.ndarray,
    target_normalized: np.ndarray,
    indices: np.ndarray,
) -> dict[str, object]:
    prediction = np.clip(predicted_normalized[indices], -1.0, 1.0)
    target = target_normalized[indices]
    error = prediction - target
    prediction_physical = normalized_to_physical(prediction)
    target_physical = normalized_to_physical(target)
    linear_pred_moving = prediction_physical[:, 0] > 0.25
    linear_true_moving = target_physical[:, 0] > 0.25
    linear_accuracy = float(np.mean(linear_pred_moving == linear_true_moving))
    class_recalls = []
    for class_value in (False, True):
        mask = linear_true_moving == class_value
        if np.any(mask):
            class_recalls.append(float(np.mean(linear_pred_moving[mask] == class_value)))
    return {
        "samples": int(len(indices)),
        "mse_normalized": float(np.mean(np.square(error))),
        "mae_normalized": np.mean(np.abs(error), axis=0).tolist(),
        "mae_physical": np.mean(
            np.abs(prediction_physical - target_physical),
            axis=0,
        ).tolist(),
        "linear_moving_accuracy": linear_accuracy,
        "linear_moving_balanced_accuracy": float(np.mean(class_recalls)),
        "prediction_std_physical": np.std(prediction_physical, axis=0).tolist(),
        "saturation_fraction": np.mean(np.abs(prediction) >= 0.999, axis=0).tolist(),
    }


def all_metrics(
    predicted: np.ndarray,
    target: np.ndarray,
    splits: dict[str, np.ndarray],
) -> dict[str, dict[str, object]]:
    return {
        split: split_metrics(predicted, target, indices)
        for split, indices in splits.items()
    }


def main() -> int:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(max(1, min(torch.get_num_threads(), 8)))
    if args.zero_goal and args.zero_sensors:
        raise ValueError("--zero-goal and --zero-sensors are mutually exclusive")
    if args.drop_pedestrian_velocity and not args.use_semantics:
        raise ValueError("--drop-pedestrian-velocity requires --use-semantics")
    if args.drop_pedestrian_velocity and args.zero_sensors:
        raise ValueError(
            "--drop-pedestrian-velocity and --zero-sensors are mutually exclusive"
        )

    with np.load(args.replay_dir / "observations.npz", allow_pickle=False) as payload:
        observations = payload["observations"].astype(np.float32)
        timestamps_ns = payload["timestamps_ns"].astype(np.int64)
        split_labels = (
            payload["split_labels"].astype(str)
            if "split_labels" in payload
            else None
        )
        recorded_from_replay = (
            payload["recorded_actions_physical"].astype(np.float32)
            if "recorded_actions_physical" in payload
            else None
        )
        if args.use_semantics:
            if "semantic_maps" not in payload:
                raise KeyError(
                    "Replay has no semantic_maps; rerun replay.py with "
                    "--include-semantics"
                )
            # Keep the replay's compact representation in host memory. The
            # semantic encoder converts each mini-batch to torch.long itself.
            semantic_maps = payload["semantic_maps"].astype(np.int16, copy=False)
        else:
            semantic_maps = None
    if args.semantic_num_classes <= 0:
        raise ValueError("--semantic-num-classes must be positive")
    if args.semantic_person_class is not None and not (
        0 <= args.semantic_person_class < args.semantic_num_classes
    ):
        raise ValueError("--semantic-person-class must be in 0..N-1")
    if semantic_maps is not None:
        if semantic_maps.shape != (len(observations), 80, 80):
            raise ValueError(
                "semantic_maps must have shape "
                f"({len(observations)}, 80, 80), got {semantic_maps.shape}"
            )
        if np.min(semantic_maps) < -1 or np.max(semantic_maps) >= args.semantic_num_classes:
            raise ValueError(
                f"semantic_maps labels must be in [-1, {args.semantic_num_classes - 1}]"
            )
    if args.zero_goal:
        observations[:, -2:] = 0.0
    if args.zero_sensors:
        observations[:, :-2] = 0.0
    if args.drop_pedestrian_velocity:
        observations[:, :PEDESTRIAN_OBSERVATION_SIZE] = 0.0
    recorded = (
        recorded_from_replay
        if recorded_from_replay is not None
        else read_recorded_actions(args.replay_dir / "predictions.csv")
    )
    if len(recorded) != len(observations):
        raise ValueError("Observation and action counts differ")
    targets = physical_to_normalized(recorded)
    splits = (
        seed_split_indices(split_labels)
        if split_labels is not None
        else make_blocked_split(
            len(observations),
            args.block_size,
            args.purge_frames,
        )
    )
    adapted_recorded = normalized_to_physical(targets)
    action_clipped = ~np.isclose(
        adapted_recorded, recorded, rtol=0.0, atol=1e-6
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / timestamp
    checkpoints_dir = output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=False)

    if args.use_semantics:
        policy, weight_count = load_semantic_policy(
            args.model,
            args.semantic_num_classes,
        )
        semantic_fusion = policy.semantic_fusion
    else:
        policy, weight_count = load_policy_strict(args.model)
        semantic_fusion = None
    feature_started = time.perf_counter()
    features = cache_features(
        policy.features_extractor,
        observations,
        args.feature_batch_size,
    )
    feature_seconds = time.perf_counter() - feature_started
    pretrained_predictions = predict_head(
        policy.mlp_extractor.policy_net,
        policy.action_net,
        features,
    )
    pretrained_metrics = all_metrics(pretrained_predictions, targets, splits)

    constant_prediction = np.broadcast_to(
        np.mean(targets[splits["train"]], axis=0),
        targets.shape,
    )
    constant_metrics = all_metrics(constant_prediction, targets, splits)

    train_features = torch.from_numpy(features[splits["train"]])
    train_semantics = (
        torch.from_numpy(semantic_maps[splits["train"]])
        if semantic_maps is not None
        else None
    )
    train_targets = torch.from_numpy(targets[splits["train"]])
    train_weights = torch.from_numpy(
        class_balanced_weights(targets, splits["train"])
    )
    val_features = torch.from_numpy(features[splits["val"]])
    val_semantics = (
        torch.from_numpy(semantic_maps[splits["val"]])
        if semantic_maps is not None
        else None
    )
    val_targets = torch.from_numpy(targets[splits["val"]])

    parameters = [
        *policy.mlp_extractor.policy_net.parameters(),
        *policy.action_net.parameters(),
    ]
    if semantic_fusion is not None:
        parameters.extend(semantic_fusion.parameters())
    optimizer = torch.optim.Adam(parameters, lr=args.learning_rate)
    element_loss = nn.SmoothL1Loss(reduction="none")
    generator = torch.Generator().manual_seed(args.seed)
    best_val_loss = math.inf
    best_epoch = -1
    epochs_without_improvement = 0
    metric_rows = []
    train_started = time.perf_counter()

    for epoch in range(args.epochs):
        policy.mlp_extractor.policy_net.train()
        policy.action_net.train()
        if semantic_fusion is not None:
            semantic_fusion.train()
        permutation = torch.randperm(len(train_features), generator=generator)
        epoch_loss_sum = 0.0
        for start in range(0, len(permutation), args.batch_size):
            batch_indices = permutation[start : start + args.batch_size]
            batch_features = train_features[batch_indices]
            if semantic_fusion is not None and train_semantics is not None:
                batch_features = semantic_fusion(
                    batch_features,
                    train_semantics[batch_indices],
                )
            prediction = policy.action_net(
                policy.mlp_extractor.policy_net(batch_features)
            )
            per_sample = element_loss(
                prediction,
                train_targets[batch_indices],
            ).mean(dim=1)
            loss = torch.mean(per_sample * train_weights[batch_indices])
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            epoch_loss_sum += float(loss.detach()) * len(batch_indices)

        policy.mlp_extractor.policy_net.eval()
        policy.action_net.eval()
        if semantic_fusion is not None:
            semantic_fusion.eval()
        with torch.inference_mode():
            val_fused = val_features
            if semantic_fusion is not None and val_semantics is not None:
                val_fused = semantic_fusion(val_features, val_semantics)
            val_prediction = policy.action_net(
                policy.mlp_extractor.policy_net(val_fused)
            )
            val_loss = float(element_loss(val_prediction, val_targets).mean())
        train_loss = epoch_loss_sum / len(train_features)
        metric_rows.append(
            {"epoch": epoch + 1, "train_loss": train_loss, "val_loss": val_loss}
        )
        if val_loss < best_val_loss - 1e-7:
            best_val_loss = val_loss
            best_epoch = epoch + 1
            epochs_without_improvement = 0
            torch.save(policy.state_dict(), checkpoints_dir / "best.pt")
        else:
            epochs_without_improvement += 1
        if epochs_without_improvement >= args.patience:
            break

    train_seconds = time.perf_counter() - train_started
    torch.save(policy.state_dict(), checkpoints_dir / "last.pt")
    best_state = torch.load(
        checkpoints_dir / "best.pt",
        map_location="cpu",
        weights_only=True,
    )
    policy.load_state_dict(best_state, strict=True)
    finetuned_predictions = predict_head(
        policy.mlp_extractor.policy_net,
        policy.action_net,
        features,
        semantic_fusion=semantic_fusion,
        semantic_maps=semantic_maps,
    )
    finetuned_metrics = all_metrics(finetuned_predictions, targets, splits)
    semantic_ablation = None
    semantic_projection_l2 = None
    if semantic_fusion is not None and semantic_maps is not None:
        unknown_semantics = np.full_like(semantic_maps, -1)
        unknown_predictions = predict_head(
            policy.mlp_extractor.policy_net,
            policy.action_net,
            features,
            semantic_fusion=semantic_fusion,
            semantic_maps=unknown_semantics,
        )
        unknown_metrics = all_metrics(unknown_predictions, targets, splits)
        del unknown_semantics, unknown_predictions

        shuffled_semantics = semantic_maps.copy()
        shuffle_generator = np.random.default_rng(args.seed + 1)
        for indices in splits.values():
            shuffled_semantics[indices] = semantic_maps[
                shuffle_generator.permutation(indices)
            ]
        shuffled_predictions = predict_head(
            policy.mlp_extractor.policy_net,
            policy.action_net,
            features,
            semantic_fusion=semantic_fusion,
            semantic_maps=shuffled_semantics,
        )
        shuffled_metrics = all_metrics(shuffled_predictions, targets, splits)
        del shuffled_semantics, shuffled_predictions
        person_masked_metrics = None
        if args.semantic_person_class is not None:
            person_masked_semantics = semantic_maps.copy()
            person_masked_semantics[
                person_masked_semantics == args.semantic_person_class
            ] = -1
            person_masked_predictions = predict_head(
                policy.mlp_extractor.policy_net,
                policy.action_net,
                features,
                semantic_fusion=semantic_fusion,
                semantic_maps=person_masked_semantics,
            )
            person_masked_metrics = all_metrics(
                person_masked_predictions,
                targets,
                splits,
            )
            del person_masked_semantics, person_masked_predictions
        semantic_ablation = {
            "correct_semantics": finetuned_metrics,
            "all_unknown": unknown_metrics,
            "within_split_random_permutation": shuffled_metrics,
            "person_masked": person_masked_metrics,
            "test_mse_gain_vs_all_unknown": (
                unknown_metrics["test"]["mse_normalized"]
                - finetuned_metrics["test"]["mse_normalized"]
            ),
            "test_mse_gain_vs_random_permutation": (
                shuffled_metrics["test"]["mse_normalized"]
                - finetuned_metrics["test"]["mse_normalized"]
            ),
            "test_mse_gain_vs_person_masked": (
                person_masked_metrics["test"]["mse_normalized"]
                - finetuned_metrics["test"]["mse_normalized"]
                if person_masked_metrics is not None
                else None
            ),
        }
        semantic_projection_l2 = float(
            torch.linalg.vector_norm(semantic_fusion.projection.weight).detach()
        )

    with (output_dir / "train_metrics.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=["epoch", "train_loss", "val_loss"])
        writer.writeheader()
        writer.writerows(metric_rows)

    split_lookup = np.full(len(observations), "purged", dtype="<U6")
    for split, indices in splits.items():
        split_lookup[indices] = split
    with (output_dir / "eval_predictions.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        fields = [
            "frame",
            "timestamp_ns",
            "split",
            "target_linear_x",
            "target_angular_z",
            "pretrained_linear_x",
            "pretrained_angular_z",
            "finetuned_linear_x",
            "finetuned_angular_z",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        target_physical = normalized_to_physical(targets)
        pretrained_physical = normalized_to_physical(pretrained_predictions)
        finetuned_physical = normalized_to_physical(finetuned_predictions)
        for index in range(len(observations)):
            writer.writerow(
                {
                    "frame": index,
                    "timestamp_ns": int(timestamps_ns[index]),
                    "split": split_lookup[index],
                    "target_linear_x": float(target_physical[index, 0]),
                    "target_angular_z": float(target_physical[index, 1]),
                    "pretrained_linear_x": float(pretrained_physical[index, 0]),
                    "pretrained_angular_z": float(pretrained_physical[index, 1]),
                    "finetuned_linear_x": float(finetuned_physical[index, 0]),
                    "finetuned_angular_z": float(finetuned_physical[index, 1]),
                }
            )

    split_document = {
        "strategy": (
            "whole-bag seed split from replay manifest"
            if split_labels is not None
            else "legacy blocked cyclic split with boundary purging"
        ),
        "cycle": None if split_labels is not None else list(SPLIT_CYCLE),
        "block_size": None if split_labels is not None else args.block_size,
        "purge_frames_each_boundary": (
            0 if split_labels is not None else args.purge_frames
        ),
        "reason": (
            "Prevents seed, bag, and episode leakage."
            if split_labels is not None
            else "Legacy compatibility path."
        ),
        "indices": {key: value.tolist() for key, value in splits.items()},
    }
    (output_dir / "dataset_split.json").write_text(
        json.dumps(split_document, indent=2) + "\n",
        encoding="utf-8",
    )

    checks = {
        "strict_pretrained_load": weight_count == 163,
        "finite_train_loss": all(
            math.isfinite(row["train_loss"]) for row in metric_rows
        ),
        "finite_validation_loss": all(
            math.isfinite(row["val_loss"]) for row in metric_rows
        ),
        "strict_best_checkpoint_reload": True,
        "seed_splits_nonempty": all(len(indices) > 0 for indices in splits.values()),
    }
    if args.drop_pedestrian_velocity:
        checks["pedestrian_velocity_input_is_zero"] = bool(
            np.count_nonzero(
                observations[:, :PEDESTRIAN_OBSERVATION_SIZE]
            )
            == 0
        )
    quality_diagnostics = {
        "training_loss_decreased": metric_rows[-1]["train_loss"]
        < metric_rows[0]["train_loss"],
        "validation_improved_over_pretrained": finetuned_metrics["val"][
            "mse_normalized"
        ]
        < pretrained_metrics["val"]["mse_normalized"],
        "test_improved_over_pretrained": finetuned_metrics["test"]["mse_normalized"]
        < pretrained_metrics["test"]["mse_normalized"],
        "test_improved_over_constant": finetuned_metrics["test"]["mse_normalized"]
        < constant_metrics["test"]["mse_normalized"],
        "test_action_not_constant": all(
            value > 1e-3
            for value in finetuned_metrics["test"]["prediction_std_physical"]
        ),
    }
    if not args.smoke:
        checks.update(quality_diagnostics)
    if semantic_fusion is not None:
        checks["semantic_projection_trained"] = bool(
            semantic_projection_l2 is not None
            and math.isfinite(semantic_projection_l2)
            and semantic_projection_l2 > 0.0
        )
    summary = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "method": "; ".join(
            [
                "frozen pretrained DRL-VO feature extractor",
                (
                    "trainable categorical semantic late-fusion branch"
                    if args.use_semantics
                    else "semantic branch disabled"
                ),
                (
                    "pedestrian vx/vy removed"
                    if args.drop_pedestrian_velocity
                    else "pedestrian vx/vy retained"
                ),
                "fine-tuned policy MLP and action head",
                (
                    "goal zeroed"
                    if args.zero_goal
                    else "sensors zeroed"
                    if args.zero_sensors
                    else "scan, semantics, and goal retained"
                    if args.drop_pedestrian_velocity
                    else "all inputs retained"
                ),
            ]
        ),
        "goal_zeroed": args.zero_goal,
        "sensors_zeroed": args.zero_sensors,
        "pedestrian_velocity_removed": args.drop_pedestrian_velocity,
        "semantic_interface": {
            "enabled": args.use_semantics,
            "num_classes": args.semantic_num_classes if args.use_semantics else None,
            "person_class": args.semantic_person_class if args.use_semantics else None,
            "map_shape": [80, 80] if args.use_semantics else None,
            "unknown_label": -1 if args.use_semantics else None,
            "fusion": "zero-initialized residual late fusion"
            if args.use_semantics
            else None,
            "only_pedestrian_input": bool(
                args.use_semantics and args.drop_pedestrian_velocity
            ),
        },
        "frames_total": len(observations),
        "frames_used": int(sum(len(value) for value in splits.values())),
        "frames_purged": int(
            len(observations) - sum(len(value) for value in splits.values())
        ),
        "recorded_action_adapter": {
            "physical_limits": {
                "linear_x": [0.0, 0.5],
                "angular_z": [-2.0, 2.0],
            },
            "linear_clipped_count": int(np.count_nonzero(action_clipped[:, 0])),
            "linear_clipped_fraction": float(np.mean(action_clipped[:, 0])),
            "angular_clipped_count": int(np.count_nonzero(action_clipped[:, 1])),
            "angular_clipped_fraction": float(np.mean(action_clipped[:, 1])),
            "raw_recorded_actions_preserved": True,
        },
        "split_counts": {key: int(len(value)) for key, value in splits.items()},
        "weight_items": weight_count,
        "best_epoch": best_epoch,
        "epochs_run": len(metric_rows),
        "best_val_smooth_l1": best_val_loss,
        "timing": {
            "feature_cache_seconds": feature_seconds,
            "training_seconds": train_seconds,
        },
        "metrics": {
            "constant_baseline": constant_metrics,
            "pretrained": pretrained_metrics,
            "finetuned": finetuned_metrics,
        },
        "semantic_ablation": semantic_ablation,
        "semantic_projection_weight_l2": semantic_projection_l2,
        "checks": checks,
        "quality_diagnostics": quality_diagnostics,
        "smoke_mode": args.smoke,
        "limitations": [
            "Three seed-disjoint bags cannot establish navigation success or multi-map generalization.",
            "Recorded reverse commands are clipped to the DRL-VO [0, 0.5] m/s action range.",
            "This is supervised offline adaptation, not PPO environment interaction.",
            (
                "Semantic Person labels come from simulator ground truth in this replay; "
                "online deployment needs the same labels from perception or must mask them."
                if args.use_semantics
                else "Semantic inputs were disabled."
            ),
            (
                "Pedestrian vx/vy channels were fixed at zero; scan history, semantics, "
                "and local goal remained available."
                if args.drop_pedestrian_velocity
                else "Pedestrian vx/vy channels remained available."
            ),
        ],
        "safety": {
            "ros_used": False,
            "topics_published": [],
            "simulation_started": False,
            "source_model_overwritten": False,
        },
    }
    (output_dir / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "run_command.txt").write_text(
        " ".join([sys.executable, *sys.argv]) + "\n",
        encoding="utf-8",
    )
    environment = {
        "python": sys.version,
        "platform": platform.platform(),
        "torch": torch.__version__,
        "numpy": np.__version__,
        "cuda_available": torch.cuda.is_available(),
        "seed": args.seed,
    }
    (output_dir / "environment.json").write_text(
        json.dumps(environment, indent=2) + "\n",
        encoding="utf-8",
    )
    report = [
        "# DRL-VO offline behavior-cloning feasibility",
        "",
        f"- Status: **{summary['status']}**",
        f"- Best epoch: {best_epoch}/{len(metric_rows)}",
        f"- Split counts: `{summary['split_counts']}`",
        f"- Purged frames: {summary['frames_purged']}",
        f"- Semantic interface: {'enabled' if args.use_semantics else 'disabled'}",
        f"- Pedestrian vx/vy input: "
        f"{'removed (all zeros)' if args.drop_pedestrian_velocity else 'retained'}",
        f"- Test MSE, pretrained: {pretrained_metrics['test']['mse_normalized']:.6f}",
        f"- Test MSE, constant: {constant_metrics['test']['mse_normalized']:.6f}",
        f"- Test MSE, fine-tuned: {finetuned_metrics['test']['mse_normalized']:.6f}",
        *(
            [
                f"- Test MSE, all semantics unknown: "
                f"{semantic_ablation['all_unknown']['test']['mse_normalized']:.6f}",
                f"- Test MSE, semantics randomly permuted within split: "
                f"{semantic_ablation['within_split_random_permutation']['test']['mse_normalized']:.6f}",
                *(
                    [
                        f"- Test MSE, Person semantics masked: "
                        f"{semantic_ablation['person_masked']['test']['mse_normalized']:.6f}"
                    ]
                    if semantic_ablation["person_masked"] is not None
                    else []
                ),
            ]
            if semantic_ablation is not None
            else []
        ),
        "",
        "## Checks",
        "",
        *[f"- {name}: {'PASS' if passed else 'FAIL'}" for name, passed in checks.items()],
        "",
        "This run only tests whether the recorded data contains learnable control signal.",
        "It does not demonstrate closed-loop navigation success.",
    ]
    (output_dir / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps({"output_dir": str(output_dir), **summary}, indent=2))
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
