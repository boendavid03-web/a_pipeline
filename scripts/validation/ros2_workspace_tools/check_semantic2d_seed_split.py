#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --report-json, --seed-split-manifest
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, TXT
# 可能使用的关键环境变量：FAIL, PASS, SPLITS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_seed_split.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 10:02:59.226631524 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:37.993044099 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_seed_split.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_semantic2d_seed_split.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Check a whole-bag seed split across exported Semantic2D sessions."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path


SPLITS = ("train", "dev", "test")


def read_lines(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def valid_window_count(records: list[dict], names: set[str], length: int = 10) -> int:
    count = 0
    for end in range(length - 1, len(records)):
        window = records[end - length + 1 : end + 1]
        if any(record["name"] not in names for record in window):
            continue
        if len({int(record["episode_id"]) for record in window}) != 1:
            continue
        stamps = [int(record["scan_01_stamp_ns"]) for record in window]
        if all(
            45_000_000 <= right - left <= 85_000_000
            for left, right in zip(stamps, stamps[1:])
        ):
            count += 1
    return count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--seed-split-manifest", required=True, type=Path)
    parser.add_argument("--report-json", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    errors: list[str] = []
    manifest = json.loads(args.seed_split_manifest.read_text(encoding="utf-8"))
    if manifest.get("trajectory_leakage_allowed") is not False:
        errors.append("seed split manifest must explicitly forbid trajectory leakage")

    expected: dict[str, dict] = {}
    seed_owners: dict[int, str] = {}
    for split in SPLITS:
        entries = manifest.get("splits", {}).get(split, [])
        if not entries:
            errors.append(f"seed split manifest has no {split} entry")
        for entry in entries:
            bag = str(entry.get("bag", ""))
            seed = int(entry.get("seed", -1))
            if not bag or bag in expected:
                errors.append(f"invalid or duplicate manifest bag: {bag!r}")
            expected[bag] = {"split": split, "seed": seed}
            if seed in seed_owners:
                errors.append(
                    f"seed {seed} is shared by {seed_owners[seed]} and {split}"
                )
            seed_owners[seed] = split

    sessions = read_lines(args.dataset_root / "dataset.txt")
    if len(sessions) != len(set(sessions)):
        errors.append("dataset.txt contains duplicate sessions")
    if len(sessions) != len(expected):
        errors.append(
            f"dataset.txt has {len(sessions)} sessions, expected {len(expected)}"
        )

    all_names: list[str] = []
    observed_bags: set[str] = set()
    split_samples = Counter()
    split_windows = Counter()
    session_reports = []
    episode_owners: dict[tuple[str, int], str] = {}
    for session_name in sessions:
        session = args.dataset_root / session_name
        metadata_path = session / "metadata.json"
        if not metadata_path.is_file():
            errors.append(f"{session_name}: missing metadata.json")
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        source_metadata_path = Path(metadata["source_npz_session"]) / "metadata.json"
        if not source_metadata_path.is_file():
            errors.append(f"{session_name}: source metadata is unavailable")
            continue
        source_metadata = json.loads(source_metadata_path.read_text(encoding="utf-8"))
        bag = Path(source_metadata["bag"]).name
        observed_bags.add(bag)
        expected_entry = expected.get(bag)
        role = metadata.get("split_role")
        if expected_entry is None:
            errors.append(f"{session_name}: bag {bag} is not allowlisted by the split manifest")
        elif role != expected_entry["split"]:
            errors.append(
                f"{session_name}: split role {role!r} != manifest {expected_entry['split']!r}"
            )

        records = metadata.get("frames", [])
        record_names = [str(record.get("name", "")) for record in records]
        split_sets = {}
        for split in SPLITS:
            split_names = read_lines(session / f"{split}.txt")
            split_sets[split] = set(split_names)
            split_samples[split] += len(split_names)
            split_windows[split] += valid_window_count(
                records, split_sets[split]
            )
        memberships = Counter(
            name for names in split_sets.values() for name in names
        )
        if set(memberships) != set(record_names) or any(
            count != 1 for count in memberships.values()
        ):
            errors.append(f"{session_name}: samples are not indexed exactly once")
        for split, names in split_sets.items():
            for record in records:
                if record["name"] not in names:
                    continue
                key = (bag, int(record["episode_id"]))
                previous = episode_owners.setdefault(key, split)
                if previous != split:
                    errors.append(
                        f"{session_name}: episode {key} crosses {previous}/{split}"
                    )
        all_names.extend(record_names)
        session_reports.append(
            {
                "session": session_name,
                "bag": bag,
                "seed": expected_entry["seed"] if expected_entry else None,
                "split": role,
                "samples": len(records),
                "episodes": len({int(record["episode_id"]) for record in records}),
            }
        )

    duplicate_names = sorted(
        name for name, count in Counter(all_names).items() if count != 1
    )
    if duplicate_names:
        errors.append(
            f"global exported filenames are not unique: {duplicate_names[:5]}"
        )
    if observed_bags != set(expected):
        errors.append(
            "observed bags differ from seed split manifest: "
            f"observed={sorted(observed_bags)}, expected={sorted(expected)}"
        )
    for split in SPLITS:
        if split_samples[split] == 0:
            errors.append(f"global {split} split has no samples")
        if split_windows[split] == 0:
            errors.append(f"global {split} split has no valid 10-frame windows")

    report = {
        "status": "PASS" if not errors else "FAIL",
        "sessions": session_reports,
        "total_samples": len(all_names),
        "unique_filenames": len(set(all_names)),
        "split_samples": dict(split_samples),
        "split_windows": dict(split_windows),
        "episode_count": len(episode_owners),
        "seed_count": len(seed_owners),
        "error_count": len(errors),
        "errors": errors,
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if errors:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
