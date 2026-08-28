#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--stats
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, TXT
# 可能使用的关键环境变量：ALL_CLASSES, CLASS_NAMES, DEFAULT_LABEL_NAMES, ERROR, ERRORS, FOUND, GRAND, ISSUES, JSON, RESULT, STATS, SUMMARY, VALUES, WARNING, ZERO
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_native_dataset.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.912035720 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_native_dataset.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_s3net_native_dataset.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""
check_s3net_native_dataset.py — 汇总检查 native LiDAR dataset 的完整性。

检查项:
  - dataset root + dataset.txt 是否可读
  - 每个 session 的 split (train/dev/test) 样本数
  - beam count、intensity 是否全零
  - label 分布、缺失类别 (2,3,4,8)
  - stats JSON 是否匹配 (可选)
  - 汇总

用法:
  python self/scripts/check_s3net_native_dataset.py /path/to/semantic2d_native_lidar [--stats stats.json]
"""

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

DEFAULT_LABEL_NAMES = [
    "_background_", "Chair", "Door", "Elevator", "Person", "Pillar",
    "Sofa", "Table", "Trash bin", "Wall",
]
ALL_CLASSES = list(range(len(DEFAULT_LABEL_NAMES)))
CLASS_NAMES = dict(enumerate(DEFAULT_LABEL_NAMES))


def load_label_names(dataset_root: Path):
    path = dataset_root / "label_names.txt"
    if path.exists():
        names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
        names = [name for name in names if name]
        if len(names) >= 2 and names[0] == "_background_":
            return names
        raise ValueError(f"invalid label names file: {path}")
    return list(DEFAULT_LABEL_NAMES)


def check_session(session_dir: Path) -> dict:
    """检查单个 session 并返回汇总信息."""
    info = {
        "name": session_dir.name,
        "exists": session_dir.is_dir(),
        "splits": {},
        "total_samples": 0,
        "beam_count": None,
        "intensity_all_zero": None,
        "label_distribution": {},
        "missing_classes": [],
        "valid_mask_fraction": None,
        "errors": [],
    }

    if not session_dir.is_dir():
        info["errors"].append("directory missing")
        return info

    # 检查 split 文件
    for split in ("train", "dev", "test"):
        split_file = session_dir / f"{split}.txt"
        if split_file.is_file():
            lines = [l.strip() for l in split_file.read_text().splitlines() if l.strip()]
            info["splits"][split] = len(lines)
        else:
            info["splits"][split] = 0
            info["errors"].append(f"missing {split}.txt")

    total = sum(info["splits"].values())
    info["total_samples"] = total

    if total == 0:
        info["errors"].append("no samples")
        return info

    scan_dir = session_dir / "scans_lidar"
    intensity_dir = session_dir / "intensities_lidar"
    angles_dir = session_dir / "angles_lidar"
    valid_mask_dir = session_dir / "valid_mask_lidar"
    label_dir = session_dir / "semantic_label"

    scan_files = sorted(scan_dir.glob("*.npy"))
    if not scan_files:
        info["errors"].append("no scan files")
        return info

    # beam count: 检查第一个样本
    first_scan = np.load(str(scan_files[0]))
    info["beam_count"] = int(first_scan.shape[0])

    # 检查所有 beam count 是否一致
    for sf in scan_files[1:]:
        s = np.load(str(sf))
        if s.shape[0] != info["beam_count"]:
            info["errors"].append(f"inconsistent beam count: {sf.name} has {s.shape[0]}, expected {info['beam_count']}")

    # intensity 全量扫描
    intensity_files = sorted(intensity_dir.glob("*.npy"))
    all_zero = True
    for ip in intensity_files:
        iv = np.load(str(ip))
        if not np.all(iv == 0):
            all_zero = False
            break
    info["intensity_all_zero"] = all_zero
    info["intensity_files_scanned"] = len(intensity_files)

    # valid_mask: 检查第一个样本
    vm_path = valid_mask_dir / f"{scan_files[0].stem}.npy"
    if vm_path.is_file():
        vm = np.load(str(vm_path))
        info["valid_mask_fraction"] = float(vm.sum() / len(vm))

    # label 分布 (扫描全部)
    label_counter = Counter()
    for lp in sorted(label_dir.glob("*.npy")):
        l = np.load(str(lp))
        unique, counts = np.unique(l, return_counts=True)
        for u, c in zip(unique, counts):
            label_counter[int(u)] += c

    info["label_distribution"] = dict(sorted(label_counter.items()))
    present = set(label_counter.keys())
    info["missing_classes"] = sorted(set(ALL_CLASSES) - present)

    return info


def check_stats(stats_path: Path) -> dict:
    """检查 stats JSON."""
    info = {"exists": stats_path.is_file(), "keys": [], "ignore_label": None,
            "normalization": None, "class_weights": None}
    if not stats_path.is_file():
        return info
    with open(stats_path) as f:
        stats = json.load(f)
    info["keys"] = sorted(stats.keys())
    info["ignore_label"] = stats.get("ignore_label")
    info["class_weights"] = stats.get("class_weights")
    # stats 实际结构: {"normalization": {"intensity": {...}, "scan": {...}, "angle_incidence": {...}}}
    norm = stats.get("normalization", {})
    info["normalization"] = {
        k: {"mean": v.get("mean"), "std": v.get("std")}
        for k, v in norm.items()
    } if norm else None
    return info


def main():
    global ALL_CLASSES, CLASS_NAMES
    parser = argparse.ArgumentParser(description="Check native LiDAR dataset for S3-Net training")
    parser.add_argument("dataset_root", type=str, help="Path to semantic2d_native_lidar root")
    parser.add_argument("--stats", type=str, default=None, help="Optional path to stats JSON")
    args = parser.parse_args()

    root = Path(args.dataset_root)
    if not root.is_dir():
        print(f"ERROR: dataset root not found: {root}")
        sys.exit(1)
    label_names = load_label_names(root)
    ALL_CLASSES = list(range(len(label_names)))
    CLASS_NAMES = dict(enumerate(label_names))

    index_file = root / "dataset.txt"
    if not index_file.is_file():
        print(f"ERROR: dataset.txt not found in {root}")
        sys.exit(1)

    sessions = [line.strip() for line in index_file.read_text().splitlines() if line.strip()]

    print("=" * 72)
    print(f"Dataset root : {root}")
    print(f"Sessions     : {len(sessions)}")
    print()

    # 逐 session 检查
    all_info = []
    grand_train = 0
    grand_dev = 0
    grand_test = 0
    global_label_counter = Counter()
    global_missing = set(ALL_CLASSES)

    for i, name in enumerate(sessions):
        info = check_session(root / name)
        all_info.append(info)

        status = "OK" if not info["errors"] else f"ERRORS: {info['errors']}"
        print(f"[{i+1}/{len(sessions)}] {name}")
        print(f"    train={info['splits'].get('train', 0)}  "
              f"dev={info['splits'].get('dev', 0)}  "
              f"test={info['splits'].get('test', 0)}  "
              f"total={info['total_samples']}")
        print(f"    beam={info['beam_count']}  "
              f"intensity_all_zero={info['intensity_all_zero']}"
              f" (scanned {info.get('intensity_files_scanned', '?')} files)  "
              f"valid_mask_frac={info['valid_mask_fraction']}")
        print(f"    labels={info['label_distribution']}")
        print(f"    missing_classes={info['missing_classes']}")
        if info["errors"]:
            print(f"    ERRORS: {info['errors']}")
        print()

        grand_train += info["splits"].get("train", 0)
        grand_dev += info["splits"].get("dev", 0)
        grand_test += info["splits"].get("test", 0)
        for k, v in info["label_distribution"].items():
            global_label_counter[k] += v
        global_missing &= set(info["missing_classes"])

    # 汇总
    grand_total = grand_train + grand_dev + grand_test
    print("=" * 72)
    print("GRAND SUMMARY")
    print("=" * 72)
    print(f"  Sessions       : {len(sessions)}")
    print(f"  Total samples  : {grand_total}")
    print(f"  Train          : {grand_train} ({grand_train/grand_total*100:.1f}%)" if grand_total else "  Train: 0")
    print(f"  Dev            : {grand_dev} ({grand_dev/grand_total*100:.1f}%)" if grand_total else "  Dev: 0")
    print(f"  Test           : {grand_test} ({grand_test/grand_total*100:.1f}%)" if grand_total else "  Test: 0")
    print()

    # Beam count 一致性
    beam_counts = set(info["beam_count"] for info in all_info if info["beam_count"] is not None)
    print(f"  Beam count(s)  : {sorted(beam_counts)}")
    if len(beam_counts) > 1:
        print("  ⚠ WARNING: inconsistent beam counts across sessions!")

    # Intensity
    all_intensity_zero = all(info["intensity_all_zero"] for info in all_info if info["intensity_all_zero"] is not None)
    print(f"  Intensity      : {'ALL ZERO' if all_intensity_zero else 'HAS NON-ZERO VALUES'}")

    # Label 分布
    print(f"  Global labels  : {dict(sorted(global_label_counter.items()))}")
    print(f"  Global missing : {sorted(global_missing)}")
    print(f"  Present classes: {sorted(set(global_label_counter.keys()))}")
    print()

    # 缺失类别说明
    missing_names = [f"{cid}={CLASS_NAMES.get(cid, '?')}" for cid in sorted(global_missing)]
    present_names = [f"{cid}={CLASS_NAMES.get(cid, '?')}" for cid in sorted(global_label_counter.keys())]
    print(f"  Present : {', '.join(present_names)}")
    print(f"  Missing : {', '.join(missing_names)}")
    print()

    # 类别不平衡提示
    if global_label_counter:
        max_class = max(global_label_counter.items(), key=lambda x: x[1])
        min_class = min(global_label_counter.items(), key=lambda x: x[1])
        print(f"  Most frequent  : class {max_class[0]} ({CLASS_NAMES.get(max_class[0], '?')}) "
              f"= {max_class[1]} ({max_class[1]/sum(global_label_counter.values())*100:.1f}%)")
        print(f"  Least frequent : class {min_class[0]} ({CLASS_NAMES.get(min_class[0], '?')}) "
              f"= {min_class[1]} ({min_class[1]/sum(global_label_counter.values())*100:.1f}%)")
    print()

    # Stats
    if args.stats:
        print("=" * 72)
        print("STATS JSON")
        print("=" * 72)
        stats_info = check_stats(Path(args.stats))
        print(f"  Exists         : {stats_info['exists']}")
        print(f"  Keys           : {stats_info['keys']}")
        print(f"  ignore_label   : {stats_info['ignore_label']}")
        print(f"  class_weights  : {stats_info['class_weights']}")
        print(f"  normalization  : {stats_info['normalization']}")
        print()

    # 结论
    print("=" * 72)
    has_errors = any(info["errors"] for info in all_info)
    if has_errors:
        print("RESULT: ISSUES FOUND — check ERRORS above")
    elif global_missing:
        print("RESULT: OK — dataset ready for training")
        print(f"  Note: {len(global_missing)} classes missing from labels ({', '.join(missing_names)})")
        print(f"  This is expected for limited-coverage data; ignore_label=-1 will handle it.")
    else:
        print("RESULT: OK — dataset ready for training (all 10 classes present)")

    return 0 if not has_errors else 1


if __name__ == "__main__":
    sys.exit(main())
