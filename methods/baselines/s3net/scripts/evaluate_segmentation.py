#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --output-json, --split, --stats-json
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：TXT
# 可能使用的关键环境变量：DEFAULT_LABEL_NAMES, IGNORE_LABEL, S3NET_FEATURE_MODE, SEED1
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.734305629 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:51.814067218 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/evaluate_segmentation.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Evaluate an S3-Net checkpoint on a Semantic2D-style split."""

import argparse
import json
import os
from pathlib import Path

import numpy as np
import torch
from tqdm import tqdm

from model import (
    IGNORE_LABEL,
    S3Net,
    VaeTestDataset,
    feature_mode_num_channels,
    set_seed,
    SEED1,
)


DEFAULT_LABEL_NAMES = [
    "_background_", "Chair", "Door", "Elevator", "Person", "Pillar",
    "Sofa", "Table", "Trash bin", "Wall",
]


def load_label_names(dataset_root):
    path = Path(dataset_root) / "label_names.txt"
    if path.exists():
        names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
        names = [name for name in names if name]
        if len(names) >= 2 and names[0] == "_background_":
            return names
        raise ValueError(f"invalid label names file: {path}")
    return list(DEFAULT_LABEL_NAMES)


def update_confusion(confusion, truth, pred, num_classes):
    mask = (truth != IGNORE_LABEL) & (truth >= 0) & (truth < num_classes)
    encoded = num_classes * truth[mask].astype(np.int64) + pred[mask].astype(np.int64)
    confusion += np.bincount(encoded, minlength=num_classes * num_classes).reshape(num_classes, num_classes)
    return int(truth.size - mask.sum())


def summarize(confusion):
    total = int(confusion.sum())
    correct = int(np.trace(confusion))
    accuracy = correct / total if total else 0.0
    ious = []
    for cls in range(confusion.shape[0]):
        tp = confusion[cls, cls]
        fp = confusion[:, cls].sum() - tp
        fn = confusion[cls, :].sum() - tp
        denom = tp + fp + fn
        ious.append(float(tp / denom) if denom else None)
    valid_ious = [v for v in ious if v is not None]
    return {
        "beam_accuracy": accuracy,
        "mean_iou_present_classes": float(np.mean(valid_ious)) if valid_ious else 0.0,
        "per_class_iou": ious,
        "confusion": confusion.tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model_path")
    parser.add_argument("dataset_root")
    parser.add_argument("--split", default="dev")
    parser.add_argument("--stats-json")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--output-json")
    args = parser.parse_args()

    set_seed(SEED1)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dataset = VaeTestDataset(args.dataset_root, args.split, stats_path=args.stats_json)
    loader = torch.utils.data.DataLoader(dataset, batch_size=args.batch_size, shuffle=False, drop_last=False)

    checkpoint = torch.load(args.model_path, map_location=device)
    feature_mode = checkpoint.get(
        "feature_mode", os.environ.get("S3NET_FEATURE_MODE", "range_intensity_incidence")
    )
    label_names = load_label_names(args.dataset_root)
    num_classes = int(checkpoint.get("num_output_channels", len(label_names)))
    if len(label_names) != num_classes:
        raise ValueError(
            f"checkpoint expects {num_classes} classes but dataset defines {len(label_names)}"
        )
    model = S3Net(
        input_channels=feature_mode_num_channels(feature_mode),
        output_channels=num_classes,
        feature_mode=feature_mode,
    ).to(device)
    model.load_state_dict(checkpoint.get("model_state_dict", checkpoint["model"]))
    model.eval()

    confusion = np.zeros((num_classes, num_classes), dtype=np.int64)
    ignored_label_count = 0
    with torch.no_grad():
        for batch in tqdm(loader, total=len(loader)):
            scans = batch["scan"].to(device)
            intensities = batch["intensity"].to(device)
            angles = batch["angle_incidence"].to(device)
            labels = batch["label"].cpu().numpy().astype(np.int64)

            _, semantic_channels, _ = model(scans, intensities, angles)
            preds = semantic_channels.argmax(dim=1).cpu().numpy().astype(np.int64)
            ignored_label_count += update_confusion(confusion, labels.reshape(-1), preds.reshape(-1), num_classes)

    report = summarize(confusion)
    report.update({
        "model_path": str(Path(args.model_path).resolve()),
        "dataset_root": str(Path(args.dataset_root).resolve()),
        "split": args.split,
        "samples": len(dataset),
        "ignore_label": IGNORE_LABEL,
        "ignored_label_count": ignored_label_count,
        "label_names": label_names,
        "num_classes": num_classes,
    })

    print("samples:", report["samples"])
    print("ignored_label_count:", report["ignored_label_count"])
    print("beam_accuracy:", round(report["beam_accuracy"], 6))
    print("mean_iou_present_classes:", round(report["mean_iou_present_classes"], 6))
    for cls, iou in enumerate(report["per_class_iou"]):
        print(f"class_{cls}_iou:", "None" if iou is None else round(iou, 6))

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print("wrote", output_path)


if __name__ == "__main__":
    main()
