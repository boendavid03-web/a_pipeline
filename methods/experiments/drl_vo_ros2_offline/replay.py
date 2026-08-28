#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --fixed-session-root, --include-semantics, --limit, --limit-per-split, --model, --output-root, --samples, --seed-split-manifest
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, NPZ, TXT
# 可能使用的关键环境变量：DOMAIN_DIFFERENCES, FAIL, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/replay.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-26 11:42:54.289397507 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.813228349 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py（导入其函数、类或模型）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/replay.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/train_behavior_cloning.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Run DRL-VO shadow inference over validated samples without ROS side effects."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import time
from collections import Counter
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

from drlvo_model import load_policy_strict
from observation_adapter import ObservationAdapter, normalized_to_physical


DOMAIN_DIFFERENCES = [
    "Current input fuses two lidars; the original policy used one lidar.",
    "Current lidar range is 8 m; the original scan normalization uses 30 m.",
    "Current subgoal is the causal online planner subgoal recorded in each bag.",
    "Current robot geometry differs from TurtleBot2.",
    "Recorded reverse commands are outside the policy's [0, 0.5] m/s linear range.",
    "The three bags use separate train/dev/test seeds but still cover one map family.",
    "Fixed bags support offline adaptation/replay, not standard online PPO training.",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--samples", type=Path)
    source.add_argument(
        "--fixed-session-root",
        type=Path,
        help="Root containing fixed-slot sessions selected by --seed-split-manifest.",
    )
    parser.add_argument("--seed-split-manifest", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--limit", type=int)
    parser.add_argument(
        "--limit-per-split",
        type=int,
        help="Bound each train/dev/test seed independently for a representative smoke.",
    )
    parser.add_argument(
        "--include-semantics",
        action="store_true",
        help="Also save categorical 80x80 semantic history maps for optional training.",
    )
    return parser.parse_args()


def seed_split_records(
    fixed_session_root: Path,
    manifest_path: Path,
    limit_per_split: int | None = None,
) -> tuple[list[tuple[Path, str, str]], int]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    requested = {
        str(entry["bag"]): split
        for split in ("train", "dev", "test")
        for entry in manifest.get("splits", {}).get(split, [])
    }
    if len(requested) != 3 or set(requested.values()) != {"train", "dev", "test"}:
        raise ValueError("seed split manifest must select exactly one bag per split")
    sessions = {}
    for metadata_path in sorted(fixed_session_root.glob("*/metadata.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        bag = Path(metadata["bag"]).name
        if bag in requested:
            if bag in sessions:
                raise ValueError(f"multiple fixed sessions match bag {bag}")
            if metadata.get("subgoal_source") != "online":
                raise ValueError(f"{bag}: DRL-VO replay requires online subgoals")
            sessions[bag] = metadata_path.parent
    if set(sessions) != set(requested):
        raise ValueError(
            "fixed sessions do not match seed split manifest: "
            f"found={sorted(sessions)}, expected={sorted(requested)}"
        )

    records = []
    full_count = 0
    for split in ("train", "dev", "test"):
        bag = next(name for name, role in requested.items() if role == split)
        paths = sorted((sessions[bag] / "samples").glob("*.npz"))
        full_count += len(paths)
        if limit_per_split is not None:
            if limit_per_split <= 0:
                raise ValueError("--limit-per-split must be positive")
            paths = paths[:limit_per_split]
        records.extend((path, split, bag) for path in paths)
    return records, full_count


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_stats(values: np.ndarray) -> dict[str, float | None]:
    values = values[np.isfinite(values)]
    if not len(values):
        return {"min": None, "max": None, "mean": None, "std": None}
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
    }


def main() -> int:
    args = parse_args()
    if args.fixed_session_root:
        if args.seed_split_manifest is None:
            raise ValueError(
                "--fixed-session-root requires --seed-split-manifest"
            )
        records, expected_total = seed_split_records(
            args.fixed_session_root,
            args.seed_split_manifest,
            args.limit_per_split,
        )
    else:
        if args.seed_split_manifest is not None or args.limit_per_split is not None:
            raise ValueError(
                "--seed-split-manifest/--limit-per-split require --fixed-session-root"
            )
        sample_paths = sorted(args.samples.glob("*.npz"))
        records = [(path, "unspecified", args.samples.parent.name) for path in sample_paths]
        expected_total = len(records)
    if args.limit is not None:
        records = records[: args.limit]
    if not records:
        raise FileNotFoundError("No NPZ samples selected for replay")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)

    policy, weight_count = load_policy_strict(args.model)
    adapter = ObservationAdapter(include_semantics=args.include_semantics)
    observations: list[np.ndarray] = []
    semantic_maps: list[np.ndarray] = []
    frame_metadata = []
    split_labels = []
    bag_names = []
    history_reset_count = 0
    previous_sequence = None
    adapt_started = time.perf_counter()
    for path, split, bag in records:
        frame = adapter.adapt(path, sequence_id=bag)
        sequence = (bag, frame.episode_id)
        if sequence != previous_sequence:
            history_reset_count += 1
            previous_sequence = sequence
        observations.append(frame.observation)
        split_labels.append(split)
        bag_names.append(bag)
        if frame.semantic_map is not None:
            semantic_maps.append(frame.semantic_map)
        # Keep only fields used by the report. Holding every observation,
        # pedestrian grid, and semantic map twice makes full replay needlessly
        # memory-heavy.
        frame_metadata.append(
            replace(
                frame,
                observation=np.empty(0, dtype=np.float32),
                pedestrian_map=np.empty(0, dtype=np.float32),
                semantic_map=None,
            )
        )
    observation_array = np.stack(observations)
    semantic_array = (
        np.stack(semantic_maps).astype(np.int16)
        if args.include_semantics
        else None
    )
    observations.clear()
    semantic_maps.clear()
    adapt_seconds = time.perf_counter() - adapt_started

    raw_action_chunks = []
    value_chunks = []
    inference_started = time.perf_counter()
    with torch.inference_mode():
        for start in range(0, len(observation_array), args.batch_size):
            batch = torch.from_numpy(
                observation_array[start : start + args.batch_size]
            )
            action_mean, value = policy(batch)
            raw_action_chunks.append(action_mean.numpy())
            value_chunks.append(value.numpy().reshape(-1))
    inference_seconds = time.perf_counter() - inference_started
    raw_actions = np.concatenate(raw_action_chunks)
    normalized_actions = np.clip(raw_actions, -1.0, 1.0)
    values = np.concatenate(value_chunks)
    physical_actions = np.stack(
        [normalized_to_physical(action) for action in normalized_actions]
    )
    guarded_actions = np.zeros_like(physical_actions)
    guard_reasons = []
    for index, (frame, action) in enumerate(zip(frame_metadata, physical_actions)):
        goal_distance = float(np.linalg.norm(frame.goal_local))
        central_scan = frame.front_scan[180:540]
        central_nonzero = central_scan[central_scan > 0.0]
        central_minimum = (
            float(np.min(central_nonzero)) if len(central_nonzero) else 10.0
        )
        if goal_distance <= 0.9:
            guard_reasons.append("goal_within_0.9m")
        elif central_minimum <= 0.4:
            guarded_actions[index] = [0.0, 0.7]
            guard_reasons.append("obstacle_within_0.4m")
        else:
            guarded_actions[index] = action
            guard_reasons.append("policy")

    predictions_path = output_dir / "predictions.csv"
    fieldnames = [
        "frame",
        "timestamp_ns",
        "predicted_linear_x",
        "predicted_angular_z",
        "guarded_linear_x",
        "guarded_angular_z",
        "guard_reason",
        "raw_action_mean_0",
        "raw_action_mean_1",
        "normalized_action_0",
        "normalized_action_1",
        "recorded_linear_x",
        "recorded_angular_z",
        "subgoal_x",
        "subgoal_y",
        "nearest_obstacle_m",
        "nearest_pedestrian_m",
        "pedestrian_ttc_0p6_s",
        "closest_approach_distance_m",
        "time_to_closest_approach_s",
        "front_scan_coverage",
        "value_estimate",
    ]
    with predictions_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, (frame, action, guarded, raw, normalized, value, guard_reason) in enumerate(
            zip(
                frame_metadata,
                physical_actions,
                guarded_actions,
                raw_actions,
                normalized_actions,
                values,
                guard_reasons,
            )
        ):
            writer.writerow(
                {
                    "frame": index,
                    "timestamp_ns": frame.timestamp_ns,
                    "predicted_linear_x": float(action[0]),
                    "predicted_angular_z": float(action[1]),
                    "guarded_linear_x": float(guarded[0]),
                    "guarded_angular_z": float(guarded[1]),
                    "guard_reason": guard_reason,
                    "raw_action_mean_0": float(raw[0]),
                    "raw_action_mean_1": float(raw[1]),
                    "normalized_action_0": float(normalized[0]),
                    "normalized_action_1": float(normalized[1]),
                    "recorded_linear_x": float(frame.recorded_cmd[0]),
                    "recorded_angular_z": float(frame.recorded_cmd[2]),
                    "subgoal_x": float(frame.goal_local[0]),
                    "subgoal_y": float(frame.goal_local[1]),
                    "nearest_obstacle_m": frame.nearest_obstacle_m,
                    "nearest_pedestrian_m": frame.nearest_pedestrian_m,
                    "pedestrian_ttc_0p6_s": frame.pedestrian_ttc_0p6_s,
                    "closest_approach_distance_m": frame.closest_approach_distance_m,
                    "time_to_closest_approach_s": frame.time_to_closest_approach_s,
                    "front_scan_coverage": frame.scan_coverage,
                    "value_estimate": float(value),
                }
            )

    recorded_actions = np.asarray(
        [[frame.recorded_cmd[0], frame.recorded_cmd[2]] for frame in frame_metadata],
        dtype=np.float32,
    )
    adapted_recorded_actions = recorded_actions.copy()
    adapted_recorded_actions[:, 0] = np.clip(
        adapted_recorded_actions[:, 0], 0.0, 0.5
    )
    adapted_recorded_actions[:, 1] = np.clip(
        adapted_recorded_actions[:, 1], -2.0, 2.0
    )
    clipped = adapted_recorded_actions != recorded_actions
    observation_payload = {
        "observations": observation_array,
        "timestamps_ns": np.asarray([f.timestamp_ns for f in frame_metadata]),
        "episode_ids": np.asarray(
            [f.episode_id for f in frame_metadata], dtype=np.int32
        ),
        "split_labels": np.asarray(split_labels),
        "bag_names": np.asarray(bag_names),
        "predicted_actions_normalized": normalized_actions,
        "predicted_actions_physical": physical_actions,
        "raw_action_means": raw_actions,
        "guarded_actions_physical": guarded_actions,
        "recorded_actions_physical": recorded_actions,
        "recorded_actions_drl_vo_physical": adapted_recorded_actions,
    }
    if semantic_array is not None:
        observation_payload["semantic_maps"] = semantic_array
    np.savez_compressed(
        output_dir / "observations.npz",
        **observation_payload,
    )

    coverage = np.asarray([f.scan_coverage for f in frame_metadata])
    pedestrian_ttc = np.asarray(
        [f.pedestrian_ttc_0p6_s for f in frame_metadata],
        dtype=np.float64,
    )
    closest_approach = np.asarray(
        [f.closest_approach_distance_m for f in frame_metadata],
        dtype=np.float64,
    )
    action_finite = bool(np.isfinite(physical_actions).all())
    action_in_bounds = bool(
        np.all((physical_actions[:, 0] >= 0.0) & (physical_actions[:, 0] <= 0.5))
        and np.all(
            (physical_actions[:, 1] >= -2.0) & (physical_actions[:, 1] <= 2.0)
        )
    )
    saturation = np.mean(np.abs(normalized_actions) >= 0.999, axis=0)
    diversity_pass = bool(
        np.all(np.std(physical_actions, axis=0) > 1e-4)
        and np.all(saturation < 0.95)
    )
    inference_hz = len(records) / inference_seconds
    expected_full_run = args.limit is None and args.limit_per_split is None
    checks = {
        "all_frames_processed": len(records) == expected_total if expected_full_run else True,
        "observation_shape": observation_array.shape[1:] == (19202,),
        "observations_finite": bool(np.isfinite(observation_array).all()),
        "strict_weight_load": weight_count == 163,
        "actions_finite": action_finite,
        "actions_in_bounds": action_in_bounds,
        "inference_at_least_15_hz": inference_hz >= 15.0,
        "history_reset_at_sequence_boundary": history_reset_count > 0,
        "all_seed_splits_readable": (
            set(split_labels) == {"train", "dev", "test"}
            if args.fixed_session_root
            else True
        ),
    }
    summary = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "input": {
            "samples": str(args.samples.resolve()) if args.samples else None,
            "fixed_session_root": (
                str(args.fixed_session_root.resolve())
                if args.fixed_session_root
                else None
            ),
            "seed_split_manifest": (
                str(args.seed_split_manifest.resolve())
                if args.seed_split_manifest
                else None
            ),
            "model": str(args.model.resolve()),
            "model_sha256": sha256_file(args.model),
            "input_read_only": True,
        },
        "frames": len(records),
        "split_counts": dict(Counter(split_labels)),
        "episode_history_resets": history_reset_count,
        "observation_shape": list(observation_array.shape),
        "semantic_interface": {
            "enabled": args.include_semantics,
            "shape": list(semantic_array.shape) if semantic_array is not None else None,
            "label_min": int(np.min(semantic_array))
            if semantic_array is not None
            else None,
            "label_max": int(np.max(semantic_array))
            if semantic_array is not None
            else None,
        },
        "weight_items": weight_count,
        "timing": {
            "adapt_seconds": adapt_seconds,
            "inference_seconds": inference_seconds,
            "inference_hz": inference_hz,
            "batch_size": args.batch_size,
        },
        "predicted_linear_x": finite_stats(physical_actions[:, 0]),
        "predicted_angular_z": finite_stats(physical_actions[:, 1]),
        "raw_action_mean_0": finite_stats(raw_actions[:, 0]),
        "raw_action_mean_1": finite_stats(raw_actions[:, 1]),
        "normalized_saturation_fraction": saturation.tolist(),
        "original_guard_reason_counts": dict(Counter(guard_reasons)),
        "guarded_linear_x": finite_stats(guarded_actions[:, 0]),
        "guarded_angular_z": finite_stats(guarded_actions[:, 1]),
        "recorded_action_adapter": {
            "physical_limits": {
                "linear_x": [0.0, 0.5],
                "angular_z": [-2.0, 2.0],
            },
            "linear_clipped_count": int(np.count_nonzero(clipped[:, 0])),
            "linear_clipped_fraction": float(np.mean(clipped[:, 0])),
            "angular_clipped_count": int(np.count_nonzero(clipped[:, 1])),
            "angular_clipped_fraction": float(np.mean(clipped[:, 1])),
            "raw_actions_preserved_in_predictions_csv": True,
        },
        "front_scan_coverage": finite_stats(coverage),
        "pedestrian_ttc_0p6_s": finite_stats(pedestrian_ttc),
        "pedestrian_ttc_finite_frames": int(np.isfinite(pedestrian_ttc).sum()),
        "closest_approach_distance_m": finite_stats(closest_approach),
        "checks": checks,
        "policy_diagnostics": {
            "action_diversity": diversity_pass,
            "linear_action_is_saturated": bool(saturation[0] >= 0.95),
            "not_a_replay_contract_failure": True,
        },
        "domain_differences": DOMAIN_DIFFERENCES,
        "safety": {
            "ros_used": False,
            "topics_published": [],
            "simulation_started": False,
            "source_modified": False,
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    command = " ".join([sys.executable, *sys.argv])
    (output_dir / "run_command.txt").write_text(command + "\n", encoding="utf-8")
    environment = {
        "python": sys.version,
        "platform": platform.platform(),
        "torch": torch.__version__,
        "numpy": np.__version__,
        "cuda_available": torch.cuda.is_available(),
    }
    (output_dir / "environment.json").write_text(
        json.dumps(environment, indent=2) + "\n",
        encoding="utf-8",
    )
    report_lines = [
        "# DRL-VO offline shadow replay",
        "",
        f"- Status: **{summary['status']}**",
        f"- Frames: {len(records)}",
        f"- Observation shape: `{tuple(observation_array.shape)}`",
        (
            f"- Semantic map shape: `{tuple(semantic_array.shape)}`"
            if semantic_array is not None
            else "- Semantic interface: disabled"
        ),
        f"- Strict weight items: {weight_count}",
        f"- Inference: {inference_hz:.2f} Hz (batch size {args.batch_size})",
        f"- Front scan coverage mean: {float(np.mean(coverage)):.3f}",
        "",
        "## Checks",
        "",
        *[f"- {name}: {'PASS' if passed else 'FAIL'}" for name, passed in checks.items()],
        "",
        "## Domain differences",
        "",
        *[f"- {item}" for item in DOMAIN_DIFFERENCES],
        "",
        "This is shadow inference only. No ROS node was started and no control topic was published.",
    ]
    (output_dir / "report.md").write_text(
        "\n".join(report_lines) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output_dir": str(output_dir), **summary}, indent=2))
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
