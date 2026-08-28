#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--epochs, --learning-rate, --output-root, --patience, --replay-dir, --seed, --training-dir
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, NPZ, PT, TXT
# 可能使用的关键环境变量：PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/analyze_goal_shortcut.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 11:49:13.900278933 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.812228331 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/analyze_goal_shortcut.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Measure how much hindsight subgoal alone explains the recorded commands."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
from torch import nn

from train_behavior_cloning import (
    all_metrics,
    physical_to_normalized,
    read_recorded_actions,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--training-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=500)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=50)
    parser.add_argument("--seed", type=int, default=1337)
    return parser.parse_args()


def load_splits(path: Path) -> dict[str, np.ndarray]:
    document = json.loads(path.read_text(encoding="utf-8"))
    return {
        key: np.asarray(value, dtype=np.int64)
        for key, value in document["indices"].items()
    }


def load_full_predictions(path: Path, sample_count: int) -> np.ndarray:
    result = np.zeros((sample_count, 2), dtype=np.float32)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            index = int(row["frame"])
            linear = float(row["finetuned_linear_x"])
            angular = float(row["finetuned_angular_z"])
            result[index] = [4.0 * linear - 1.0, angular / 2.0]
    return result


def moving_balanced_accuracy(
    predicted_moving: np.ndarray,
    true_moving: np.ndarray,
) -> float:
    recalls = []
    for class_value in (False, True):
        mask = true_moving == class_value
        if np.any(mask):
            recalls.append(float(np.mean(predicted_moving[mask] == class_value)))
    return float(np.mean(recalls))


def optimize_goal_norm_rule(
    goal_norm: np.ndarray,
    targets: np.ndarray,
    train_indices: np.ndarray,
) -> tuple[float, np.ndarray]:
    true_moving = targets[:, 0] > 0.0
    candidates = np.linspace(
        float(np.min(goal_norm[train_indices])),
        float(np.max(goal_norm[train_indices])),
        500,
    )
    best_threshold = 0.0
    best_score = -math.inf
    for threshold in candidates:
        score = moving_balanced_accuracy(
            goal_norm[train_indices] > threshold,
            true_moving[train_indices],
        )
        if score > best_score:
            best_score = score
            best_threshold = float(threshold)
    prediction = np.zeros_like(targets)
    prediction[:, 0] = np.where(goal_norm > best_threshold, 1.0, -1.0)
    return best_threshold, prediction


def main() -> int:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(max(1, min(torch.get_num_threads(), 8)))

    with np.load(args.replay_dir / "observations.npz", allow_pickle=False) as payload:
        observations = payload["observations"].astype(np.float32)
    goals = observations[:, -2:]
    goal_features = np.column_stack((goals, np.linalg.norm(goals, axis=1))).astype(
        np.float32
    )
    recorded = read_recorded_actions(args.replay_dir / "predictions.csv")
    targets = physical_to_normalized(recorded)
    splits = load_splits(args.training_dir / "dataset_split.json")
    full_predictions = load_full_predictions(
        args.training_dir / "eval_predictions.csv",
        len(observations),
    )
    full_metrics = all_metrics(full_predictions, targets, splits)

    best_threshold, rule_predictions = optimize_goal_norm_rule(
        goal_features[:, 2],
        targets,
        splits["train"],
    )
    rule_metrics = all_metrics(rule_predictions, targets, splits)

    model = nn.Sequential(
        nn.Linear(3, 32),
        nn.Tanh(),
        nn.Linear(32, 32),
        nn.Tanh(),
        nn.Linear(32, 2),
    )
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    loss_fn = nn.SmoothL1Loss()
    x_train = torch.from_numpy(goal_features[splits["train"]])
    y_train = torch.from_numpy(targets[splits["train"]])
    x_val = torch.from_numpy(goal_features[splits["val"]])
    y_val = torch.from_numpy(targets[splits["val"]])
    best_state = None
    best_val = math.inf
    best_epoch = -1
    stale = 0
    rows = []
    for epoch in range(args.epochs):
        model.train()
        prediction = model(x_train)
        train_loss = loss_fn(prediction, y_train)
        optimizer.zero_grad(set_to_none=True)
        train_loss.backward()
        optimizer.step()
        model.eval()
        with torch.inference_mode():
            val_loss = float(loss_fn(model(x_val), y_val))
        rows.append(
            {
                "epoch": epoch + 1,
                "train_loss": float(train_loss.detach()),
                "val_loss": val_loss,
            }
        )
        if val_loss < best_val - 1e-7:
            best_val = val_loss
            best_epoch = epoch + 1
            best_state = {
                key: value.detach().clone()
                for key, value in model.state_dict().items()
            }
            stale = 0
        else:
            stale += 1
        if stale >= args.patience:
            break
    if best_state is None:
        raise RuntimeError("Goal-only model did not produce a checkpoint")
    model.load_state_dict(best_state, strict=True)
    model.eval()
    with torch.inference_mode():
        goal_only_predictions = model(torch.from_numpy(goal_features)).numpy()
    goal_only_metrics = all_metrics(goal_only_predictions, targets, splits)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    torch.save(best_state, output_dir / "goal_only_best.pt")
    with (output_dir / "train_metrics.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=["epoch", "train_loss", "val_loss"])
        writer.writeheader()
        writer.writerows(rows)

    full_test_mse = float(full_metrics["test"]["mse_normalized"])
    goal_test_mse = float(goal_only_metrics["test"]["mse_normalized"])
    full_linear_bacc = float(
        full_metrics["test"]["linear_moving_balanced_accuracy"]
    )
    goal_linear_bacc = float(
        goal_only_metrics["test"]["linear_moving_balanced_accuracy"]
    )
    shortcut_risk = bool(
        goal_test_mse <= 1.5 * full_test_mse
        or goal_linear_bacc >= full_linear_bacc - 0.05
    )
    summary = {
        "status": "PASS",
        "purpose": "diagnostic ablation, not a navigation-success test",
        "best_epoch": best_epoch,
        "epochs_run": len(rows),
        "goal_norm_rule_threshold_normalized": best_threshold,
        "metrics": {
            "goal_norm_rule": rule_metrics,
            "goal_only_mlp": goal_only_metrics,
            "full_frozen_feature_finetune": full_metrics,
        },
        "shortcut_risk": shortcut_risk,
        "interpretation": (
            "Hindsight subgoal alone explains nearly as much held-out behavior as the "
            "full adapted observation. Record online planner subgoals before treating "
            "behavior-cloning accuracy as sensor-driven learning."
            if shortcut_risk
            else "Full adapted observations materially outperform the hindsight-goal-only ablation."
        ),
    }
    (output_dir / "ablation_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "report.md").write_text(
        "\n".join(
            [
                "# Hindsight-subgoal shortcut ablation",
                "",
                f"- Shortcut risk: **{'YES' if shortcut_risk else 'NO'}**",
                f"- Goal rule threshold: {best_threshold:.4f} normalized",
                f"- Goal-only test MSE: {goal_test_mse:.6f}",
                f"- Full-input test MSE: {full_test_mse:.6f}",
                f"- Goal-only linear balanced accuracy: {goal_linear_bacc:.4f}",
                f"- Full-input linear balanced accuracy: {full_linear_bacc:.4f}",
                "",
                summary["interpretation"],
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (output_dir / "run_command.txt").write_text(
        " ".join([sys.executable, *sys.argv]) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output_dir": str(output_dir), **summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
