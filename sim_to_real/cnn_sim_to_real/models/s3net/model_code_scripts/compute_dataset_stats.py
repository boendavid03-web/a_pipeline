#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：--ignore-class-ids, --num-classes, --split
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：NPY, TXT
# 可能使用的关键环境变量：IGNORE_LABEL
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.733305586 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:51.814067218 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08_smoke_train.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/compute_dataset_stats.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08_smoke_train.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Compute S3-Net normalization stats and class weights for a converted dataset."""

import argparse
import json
from pathlib import Path

import numpy as np

from model import IGNORE_LABEL, angle_incidence_from_scan


def parse_class_ids(value):
    if value is None:
        return []
    ids = []
    for item in str(value).split(","):
        item = item.strip()
        if item:
            ids.append(int(item))
    return ids


class RunningStats:
    def __init__(self, std_floor=1e-6):
        self.count = 0
        self.total = 0.0
        self.total_sq = 0.0
        self.std_floor = float(std_floor)

    def update(self, values):
        values = np.asarray(values, dtype=np.float64)
        values = values[np.isfinite(values)]
        if values.size == 0:
            return
        self.count += int(values.size)
        self.total += float(values.sum())
        self.total_sq += float(np.square(values).sum())

    def as_dict(self):
        mean = self.total / max(self.count, 1)
        variance = max(self.total_sq / max(self.count, 1) - mean * mean, 0.0)
        return {"mean": mean, "std": max(float(np.sqrt(variance)), self.std_floor), "count": self.count}


def iter_sample_names(dataset_root, split):
    root = Path(dataset_root)
    for folder in root.joinpath("dataset.txt").read_text().splitlines():
        folder = folder.strip()
        if not folder:
            continue
        split_file = root / folder / f"{split}.txt"
        if not split_file.exists():
            continue
        for name in split_file.read_text().splitlines():
            name = name.strip()
            if name.endswith(".npy"):
                yield root / folder, name


def compute_stats(dataset_root, split, num_classes, ignore_class_ids=None):
    ignore_class_ids = set(ignore_class_ids or [])
    scan_stats = RunningStats()
    intensity_stats = RunningStats(std_floor=1.0)
    angle_stats = RunningStats()
    class_counts = np.zeros(num_classes, dtype=np.int64)
    samples = 0
    native_samples = 0
    ignored_label_count = 0
    total_beams = 0
    valid_stat_beams = 0
    beam_counts = []

    for session_root, name in iter_sample_names(dataset_root, split):
        scan = np.load(session_root / "scans_lidar" / name).astype(np.float32).reshape(-1)
        intensity = np.load(session_root / "intensities_lidar" / name).astype(np.float32).reshape(-1)
        label = np.nan_to_num(
            np.load(session_root / "semantic_label" / name),
            nan=IGNORE_LABEL,
            posinf=IGNORE_LABEL,
            neginf=IGNORE_LABEL,
        ).astype(np.int64).reshape(-1)
        if ignore_class_ids:
            label[np.isin(label, list(ignore_class_ids))] = IGNORE_LABEL

        if intensity.shape[0] != scan.shape[0]:
            intensity = np.zeros(scan.shape, dtype=np.float32)
        if label.shape[0] != scan.shape[0]:
            raise ValueError(f"semantic_label shape does not match scan shape: {session_root / 'semantic_label' / name}")

        angles_path = session_root / "angles_lidar" / name
        valid_mask_path = session_root / "valid_mask_lidar" / name
        is_native = angles_path.exists() and valid_mask_path.exists()
        if is_native:
            angles = np.load(angles_path).astype(np.float32).reshape(-1)
            valid_mask = np.load(valid_mask_path).astype(np.bool_).reshape(-1)
            if angles.shape[0] != scan.shape[0] or valid_mask.shape[0] != scan.shape[0]:
                raise ValueError(f"native angle/mask shape does not match scan shape: {name}")
            native_samples += 1
        else:
            angles = None
            valid_mask = np.isfinite(scan)

        source_sensor_path = session_root / "source_sensor" / name
        source_sensor = None
        if source_sensor_path.exists():
            source_sensor = np.load(source_sensor_path).reshape(-1)
            if source_sensor.shape != scan.shape:
                raise ValueError(f"source_sensor shape does not match scan shape: {name}")
        angle = angle_incidence_from_scan(scan, angles, source_sensor=source_sensor)
        finite_mask = np.isfinite(scan) & np.isfinite(intensity) & np.isfinite(angle)
        label_mask = (label >= 0) & (label < num_classes)
        stat_mask = valid_mask & finite_mask & label_mask

        scan = np.nan_to_num(scan, nan=0.0, posinf=0.0, neginf=0.0)
        intensity = np.nan_to_num(intensity, nan=0.0, posinf=0.0, neginf=0.0)
        angle = np.nan_to_num(angle, nan=0.0, posinf=0.0, neginf=0.0)

        scan_stats.update(scan[stat_mask])
        intensity_stats.update(intensity[stat_mask])
        angle_stats.update(angle[stat_mask])
        class_counts += np.bincount(label[stat_mask], minlength=num_classes)
        ignored_label_count += int(label.size - stat_mask.sum())
        total_beams += int(label.size)
        valid_stat_beams += int(stat_mask.sum())
        beam_counts.append(int(scan.shape[0]))
        samples += 1

    freqs = class_counts.astype(np.float64) / max(int(class_counts.sum()), 1)
    present = freqs > 0
    median_freq = float(np.median(freqs[present])) if np.any(present) else 0.0
    class_weights = np.zeros(num_classes, dtype=np.float64)
    class_weights[present] = median_freq / freqs[present]

    return {
        "dataset_root": str(Path(dataset_root).resolve()),
        "split": split,
        "samples": samples,
        "native_lidar": native_samples > 0,
        "native_samples": native_samples,
        "ignore_label": IGNORE_LABEL,
        "ignored_class_ids": sorted(ignore_class_ids),
        "ignored_label_count": ignored_label_count,
        "total_beams": total_beams,
        "valid_stat_beams": valid_stat_beams,
        "valid_stat_beam_fraction": valid_stat_beams / max(total_beams, 1),
        "beam_count_unique": sorted(set(beam_counts)),
        "normalization": {
            "scan": scan_stats.as_dict(),
            "intensity": intensity_stats.as_dict(),
            "angle_incidence": angle_stats.as_dict(),
        },
        "class_counts": class_counts.tolist(),
        "class_frequencies": freqs.tolist(),
        "class_weights": class_weights.tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root")
    parser.add_argument("output_json")
    parser.add_argument("--split", default="train")
    parser.add_argument("--num-classes", type=int, default=10)
    parser.add_argument(
        "--ignore-class-ids",
        default="",
        help="Comma-separated semantic class ids to treat as ignore_label, e.g. 0 for Other/Background.",
    )
    args = parser.parse_args()

    stats = compute_stats(
        args.dataset_root,
        args.split,
        args.num_classes,
        ignore_class_ids=parse_class_ids(args.ignore_class_ids),
    )
    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n")

    print("wrote", output_path)
    print("samples:", stats["samples"])
    print("normalization:", stats["normalization"])
    print("class_counts:", stats["class_counts"])
    print("class_weights:", [round(v, 6) for v in stats["class_weights"]])


if __name__ == "__main__":
    main()
