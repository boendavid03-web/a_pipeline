#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：DB3, NPY, PGM, PNG, TXT, YAML
# 可能使用的关键环境变量：PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/verify_smoke_example.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:46:55.725904249 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.850391663 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（source 载入公共环境/函数/变量）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/verify_smoke_example.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜verify_smoke_example.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Validate the bundled 35-frame semantic LiDAR smoke example."""

from collections import Counter
from pathlib import Path
import sys

import numpy as np
from PIL import Image


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "examples/smoke").resolve()
    semantic_map = root / "semantic_map"
    session = root / "converted_dataset" / "20260716-085329-semantic-converted"

    occupancy = np.asarray(Image.open(semantic_map / "occupancy.pgm"))
    label_map = np.asarray(Image.open(semantic_map / "label.png"))
    names = (semantic_map / "label_names.txt").read_text().splitlines()
    if occupancy.shape != (482, 632) or label_map.shape != occupancy.shape:
        raise SystemExit(f"map shape mismatch: occupancy={occupancy.shape}, label={label_map.shape}")
    if not set(np.unique(label_map)).issubset(set(range(len(names)))):
        raise SystemExit("semantic map contains an invalid class id")

    required = (
        "scans_lidar",
        "intensities_lidar",
        "angles_lidar",
        "valid_mask_lidar",
        "semantic_label",
        "positions",
        "velocities",
        "sub_goals_local",
    )
    files = {name: sorted((session / name).glob("*.npy")) for name in required}
    counts = {name: len(paths) for name, paths in files.items()}
    if any(count != 35 for count in counts.values()):
        raise SystemExit(f"expected 35 files per field, got {counts}")

    histogram: Counter[int] = Counter()
    for index in range(35):
        scan = np.load(files["scans_lidar"][index])
        angles = np.load(files["angles_lidar"][index])
        valid = np.load(files["valid_mask_lidar"][index])
        labels = np.load(files["semantic_label"][index])
        if not (scan.shape == angles.shape == valid.shape == labels.shape == (360,)):
            raise SystemExit(f"frame {index} beam shape mismatch")
        histogram.update(int(value) for value in labels.tolist())

    expected = {-1: 402, 1: 111, 5: 204, 6: 325, 7: 197, 9: 11361}
    if dict(sorted(histogram.items())) != expected:
        raise SystemExit(f"semantic histogram mismatch: {dict(sorted(histogram.items()))}")
    if not (root / "test_rosbag" / "metadata.yaml").is_file():
        raise SystemExit("smoke rosbag metadata is missing")
    if len(list((root / "test_rosbag").glob("*.db3"))) != 1:
        raise SystemExit("smoke rosbag must contain exactly one db3 file")

    print("PASS: smoke map is 632x482 with valid semantic ids")
    print(f"PASS: 35 native LiDAR frames x 360 beams, histogram={expected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
