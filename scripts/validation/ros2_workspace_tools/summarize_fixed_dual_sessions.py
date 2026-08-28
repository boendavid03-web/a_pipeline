#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--fixed-session-root, --report-json
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, NPZ
# 可能使用的关键环境变量：PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/summarize_fixed_dual_sessions.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 10:10:50.556176866 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.950217933 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到其他项目脚本的直接调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他项目脚本直接调用本文件；它可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/summarize_fixed_dual_sessions.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜summarize_fixed_dual_sessions.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Summarize fixed-dual samples per bag and episode without modifying them."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixed-session-root", required=True, type=Path)
    parser.add_argument("--report-json", required=True, type=Path)
    return parser.parse_args()


class Accumulator:
    def __init__(self, class_count: int) -> None:
        self.samples = 0
        self.class_beams = np.zeros(class_count, dtype=np.int64)
        self.valid_beams = 0
        self.person_frames = 0
        self.cmd_sum = np.zeros(3, dtype=np.float64)
        self.cmd_min = np.full(3, np.inf)
        self.cmd_max = np.full(3, -np.inf)
        self.stop = 0
        self.left = 0
        self.right = 0
        self.reverse = 0
        self.lateral = 0
        self.subgoal_age_sum = 0
        self.subgoal_age_min = np.inf
        self.subgoal_age_max = -np.inf
        self.cmd_age_sum = 0
        self.cmd_age_min = np.inf
        self.cmd_age_max = -np.inf
        self.drl_linear_clipped = 0
        self.drl_angular_clipped = 0

    def add(self, sample: np.lib.npyio.NpzFile, person_id: int) -> None:
        labels = sample["semantic_label"]
        valid_labels = labels[labels >= 0].astype(np.int64)
        if len(valid_labels):
            self.class_beams += np.bincount(
                valid_labels, minlength=len(self.class_beams)
            )[: len(self.class_beams)]
        self.valid_beams += len(valid_labels)
        self.person_frames += int(np.any(labels == person_id))

        cmd = sample["cmd_velocity"].astype(np.float64)
        self.cmd_sum += cmd
        self.cmd_min = np.minimum(self.cmd_min, cmd)
        self.cmd_max = np.maximum(self.cmd_max, cmd)
        moving = bool(np.any(np.abs(cmd) > 1e-6))
        self.stop += int(not moving)
        self.left += int(cmd[2] > 1e-6)
        self.right += int(cmd[2] < -1e-6)
        self.reverse += int(cmd[0] < -1e-6)
        self.lateral += int(abs(cmd[1]) > 1e-6)
        self.drl_linear_clipped += int(cmd[0] < 0.0 or cmd[0] > 0.5)
        self.drl_angular_clipped += int(cmd[2] < -2.0 or cmd[2] > 2.0)

        subgoal_age = int(sample["local_subgoal_age_ns"])
        self.subgoal_age_sum += subgoal_age
        self.subgoal_age_min = min(self.subgoal_age_min, subgoal_age)
        self.subgoal_age_max = max(self.subgoal_age_max, subgoal_age)
        cmd_age = int(sample["cmd_vel_age_ns"])
        self.cmd_age_sum += cmd_age
        self.cmd_age_min = min(self.cmd_age_min, cmd_age)
        self.cmd_age_max = max(self.cmd_age_max, cmd_age)
        self.samples += 1

    def report(self, label_names: list[str]) -> dict:
        if self.samples == 0:
            raise ValueError("cannot report an empty accumulator")
        return {
            "samples": self.samples,
            "semantic_class_beams": {
                name: int(self.class_beams[index])
                for index, name in enumerate(label_names)
            },
            "valid_semantic_beams": self.valid_beams,
            "person": {
                "beam_count": int(self.class_beams[label_names.index("Person")]),
                "fraction_of_valid_beams": float(
                    self.class_beams[label_names.index("Person")]
                    / self.valid_beams
                ),
                "frames_with_person": self.person_frames,
                "frame_fraction": self.person_frames / self.samples,
            },
            "command": {
                "linear_x": {
                    "min": float(self.cmd_min[0]),
                    "max": float(self.cmd_max[0]),
                    "mean": float(self.cmd_sum[0] / self.samples),
                },
                "linear_y": {
                    "min": float(self.cmd_min[1]),
                    "max": float(self.cmd_max[1]),
                    "mean": float(self.cmd_sum[1] / self.samples),
                },
                "angular_z": {
                    "min": float(self.cmd_min[2]),
                    "max": float(self.cmd_max[2]),
                    "mean": float(self.cmd_sum[2] / self.samples),
                },
                "stop_count": self.stop,
                "stop_fraction": self.stop / self.samples,
                "moving_count": self.samples - self.stop,
                "moving_fraction": (self.samples - self.stop) / self.samples,
                "left_turn_count": self.left,
                "right_turn_count": self.right,
                "reverse_count": self.reverse,
                "lateral_count": self.lateral,
            },
            "online_subgoal_age_ms": {
                "min": self.subgoal_age_min / 1e6,
                "max": self.subgoal_age_max / 1e6,
                "mean": self.subgoal_age_sum / self.samples / 1e6,
            },
            "causal_cmd_age_ms": {
                "min": self.cmd_age_min / 1e6,
                "max": self.cmd_age_max / 1e6,
                "mean": self.cmd_age_sum / self.samples / 1e6,
            },
            "drl_vo_label_adapter": {
                "linear_range_mps": [0.0, 0.5],
                "angular_range_radps": [-2.0, 2.0],
                "linear_clipped_count": self.drl_linear_clipped,
                "linear_clipped_fraction": self.drl_linear_clipped / self.samples,
                "angular_clipped_count": self.drl_angular_clipped,
                "angular_clipped_fraction": self.drl_angular_clipped / self.samples,
                "raw_action_modified": False,
            },
        }


def main() -> None:
    args = parse_args()
    bag_reports = []
    global_accumulator = None
    expected_total = 0
    for metadata_path in sorted(args.fixed_session_root.glob("*/metadata.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        label_names = list(metadata["label_names"])
        if "Person" not in label_names:
            raise ValueError(f"{metadata_path}: Person class is unavailable")
        person_id = label_names.index("Person")
        bag_accumulator = Accumulator(len(label_names))
        episodes: dict[int, Accumulator] = defaultdict(
            lambda: Accumulator(len(label_names))
        )
        sample_paths = sorted((metadata_path.parent / "samples").glob("*.npz"))
        if len(sample_paths) != int(metadata["samples"]):
            raise ValueError(f"{metadata_path.parent}: sample count mismatch")
        for sample_path in sample_paths:
            with np.load(sample_path, allow_pickle=False) as sample:
                episode_id = int(sample["episode_id"])
                bag_accumulator.add(sample, person_id)
                episodes[episode_id].add(sample, person_id)
        if global_accumulator is None:
            global_accumulator = Accumulator(len(label_names))
        for episode_accumulator in episodes.values():
            global_accumulator.samples += episode_accumulator.samples
            global_accumulator.class_beams += episode_accumulator.class_beams
            global_accumulator.valid_beams += episode_accumulator.valid_beams
            global_accumulator.person_frames += episode_accumulator.person_frames
            global_accumulator.cmd_sum += episode_accumulator.cmd_sum
            global_accumulator.cmd_min = np.minimum(
                global_accumulator.cmd_min, episode_accumulator.cmd_min
            )
            global_accumulator.cmd_max = np.maximum(
                global_accumulator.cmd_max, episode_accumulator.cmd_max
            )
            for name in (
                "stop",
                "left",
                "right",
                "reverse",
                "lateral",
                "subgoal_age_sum",
                "cmd_age_sum",
                "drl_linear_clipped",
                "drl_angular_clipped",
            ):
                setattr(
                    global_accumulator,
                    name,
                    getattr(global_accumulator, name)
                    + getattr(episode_accumulator, name),
                )
            global_accumulator.subgoal_age_min = min(
                global_accumulator.subgoal_age_min,
                episode_accumulator.subgoal_age_min,
            )
            global_accumulator.subgoal_age_max = max(
                global_accumulator.subgoal_age_max,
                episode_accumulator.subgoal_age_max,
            )
            global_accumulator.cmd_age_min = min(
                global_accumulator.cmd_age_min, episode_accumulator.cmd_age_min
            )
            global_accumulator.cmd_age_max = max(
                global_accumulator.cmd_age_max, episode_accumulator.cmd_age_max
            )
        expected_total += len(sample_paths)
        bag_reports.append(
            {
                "bag": Path(metadata["bag"]).name,
                "session": metadata_path.parent.name,
                "drop_counts": metadata.get("drop_counts", {}),
                "summary": bag_accumulator.report(label_names),
                "episodes": {
                    str(episode_id): accumulator.report(label_names)
                    for episode_id, accumulator in sorted(episodes.items())
                },
            }
        )
    if global_accumulator is None or global_accumulator.samples != expected_total:
        raise ValueError("global sample accumulation failed")
    report = {
        "status": "PASS",
        "map_resolution_m_per_cell": 0.05,
        "inflation_radius_m": None,
        "inflation_radius_note": "unknown (user-intended 0.55 m)",
        "label_names": label_names,
        "global": global_accumulator.report(label_names),
        "bags": bag_reports,
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "status": report["status"],
                "samples": report["global"]["samples"],
                "bags": len(bag_reports),
                "episodes": sum(len(bag["episodes"]) for bag in bag_reports),
                "drl_vo_label_adapter": report["global"]["drl_vo_label_adapter"],
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
