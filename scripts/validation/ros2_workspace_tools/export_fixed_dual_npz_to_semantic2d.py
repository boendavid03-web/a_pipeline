#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--dev-ratio, --filename-prefix, --frame-period-tolerance-ms, --output-session, --pool-range-max, --range-max-01, --range-max-02, --session-name, --source-session, --split-role, --train-ratio
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, NPZ, TXT
# 可能使用的关键环境变量：FIELD_MAP, PEDESTRIAN_FIELD_MAP
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.837309994 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:46.950217933 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/export_fixed_dual_npz_to_semantic2d.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜export_fixed_dual_npz_to_semantic2d.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Export fixed-slot dual-LiDAR NPZ samples as aligned Semantic2D NPY fields."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


FIELD_MAP = {
    "scans_lidar": ("raw_ranges", np.float32),
    "angles_lidar": ("raw_angles_sensor", np.float32),
    "virtual_ranges_lidar": ("virtual_ranges", np.float32),
    "virtual_angles_lidar": ("virtual_angles", np.float32),
    "range_valid_mask_lidar": ("range_valid_mask", np.bool_),
    "self_mask_lidar": ("self_mask", np.bool_),
    "valid_mask_lidar": ("valid_mask", np.bool_),
    "semantic_label": ("semantic_label", np.int16),
    "source_sensor": ("source_sensor", np.uint8),
    "raw_beam_index": ("raw_beam_index", np.int32),
    "positions": ("position", np.float32),
    "velocities": ("velocity", np.float32),
    "cmd_velocities": ("cmd_velocity", np.float32),
    "sub_goals_local": ("sub_goal_local", np.float32),
}

PEDESTRIAN_FIELD_MAP = {
    "pedestrian_ids": ("pedestrian_ids", np.str_),
    "pedestrian_positions": ("pedestrian_xy_map", np.float32),
    "pedestrian_velocities": ("pedestrian_velocity_map", np.float32),
    "pedestrian_yaws": ("pedestrian_yaw_map", np.float32),
    "pedestrian_leg_positions": ("pedestrian_leg_xy_map", np.float32),
    "pedestrian_truth_timestamps": ("pedestrian_truth_stamp_ns", np.int64),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-session", required=True, type=Path)
    parser.add_argument("--output-session", required=True, type=Path)
    parser.add_argument("--session-name", required=True)
    parser.add_argument("--range-max-01", type=float)
    parser.add_argument("--range-max-02", type=float)
    parser.add_argument("--pool-range-max", type=float)
    parser.add_argument("--frame-period-tolerance-ms", type=float, default=20.0)
    parser.add_argument("--train-ratio", type=float, default=0.7)
    parser.add_argument("--dev-ratio", type=float, default=0.1)
    parser.add_argument(
        "--split-role",
        choices=("preserve", "train", "dev", "test"),
        default="preserve",
        help="Preserve source splits or assign the complete bag to one seed-level split.",
    )
    parser.add_argument("--filename-prefix")
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def positive_finite(value, name):
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite, got {value!r}")


def source_range_max(source_metadata, layout_key, override):
    layout = source_metadata.get(layout_key)
    source_value = layout.get("range_max") if isinstance(layout, dict) else None
    if source_value is not None:
        source_value = float(source_value)
        positive_finite(source_value, f"source metadata {layout_key}.range_max")
    if override is not None:
        override = float(override)
        option_name = {
            "scan_01_layout": "--range-max-01",
            "scan_02_layout": "--range-max-02",
        }[layout_key]
        positive_finite(override, option_name)
        if source_value is not None and not math.isclose(
            override, source_value, rel_tol=0.0, abs_tol=1e-6
        ):
            raise ValueError(
                f"explicit range max {override} does not match source metadata "
                f"{layout_key}.range_max={source_value}"
            )
    value = source_value if source_value is not None else override
    if value is None:
        raise ValueError(
            f"source metadata has no {layout_key}.range_max; pass the matching "
            "--range-max-01/--range-max-02 values explicitly"
        )
    return float(value)


def resolve_range_config(source_metadata, args):
    range_max_01 = source_range_max(
        source_metadata, "scan_01_layout", args.range_max_01
    )
    range_max_02 = source_range_max(
        source_metadata, "scan_02_layout", args.range_max_02
    )
    pool_range_max = (
        max(range_max_01, range_max_02)
        if args.pool_range_max is None
        else float(args.pool_range_max)
    )
    positive_finite(pool_range_max, "--pool-range-max")
    if pool_range_max + 1e-9 < max(range_max_01, range_max_02):
        raise ValueError(
            "pool range max must cover both sensor range maxima; got "
            f"scan_01={range_max_01}, scan_02={range_max_02}, pool={pool_range_max}"
        )
    return range_max_01, range_max_02, pool_range_max


def frame_period_from_records(frame_metadata, tolerance_ms, stamp_key):
    positive_finite(float(tolerance_ms), "--frame-period-tolerance-ms")
    if len(frame_metadata) < 2:
        raise ValueError("at least two synchronized samples are required to infer frame period")
    stamps = np.asarray(
        [record[stamp_key] for record in frame_metadata], dtype=np.int64
    )
    deltas_ms = np.diff(stamps).astype(np.float64) / 1_000_000.0
    if np.any(~np.isfinite(deltas_ms)) or np.any(deltas_ms <= 0.0):
        raise ValueError("scan_01 timestamps must be strictly increasing to infer frame period")
    return {
        "expected_ms": float(np.median(deltas_ms)),
        "tolerance_ms": float(tolerance_ms),
        "min_ms": float(np.min(deltas_ms)),
        "max_ms": float(np.max(deltas_ms)),
    }


def contiguous_source_files(source_session):
    files = sorted((source_session / "samples").glob("*.npz"))
    if not files:
        raise FileNotFoundError(f"no NPZ samples under {source_session / 'samples'}")
    expected = [f"{index:07d}.npz" for index in range(len(files))]
    actual = [path.name for path in files]
    if actual != expected:
        raise ValueError("source NPZ filenames are not a continuous zero-based sequence")
    return files


def write_split(path, names):
    path.write_text("".join(f"{name}\n" for name in names), encoding="utf-8")


def read_split(path):
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def main():
    args = parse_args()
    if args.output_session.exists():
        raise FileExistsError(f"refusing to overwrite output session: {args.output_session}")
    source_metadata_path = args.source_session / "metadata.json"
    if not source_metadata_path.is_file():
        raise FileNotFoundError(source_metadata_path)
    source_metadata = json.loads(source_metadata_path.read_text(encoding="utf-8"))
    source_files = contiguous_source_files(args.source_session)
    source_splits = {
        split: read_split(args.source_session / f"{split}.txt")
        for split in ("train", "dev", "test")
    }
    source_split_lookup = {}
    for split, names in source_splits.items():
        for name in names:
            if name in source_split_lookup:
                raise ValueError(f"source sample appears in multiple splits: {name}")
            source_split_lookup[name] = split
    if set(source_split_lookup) != {path.name for path in source_files}:
        raise ValueError("source train/dev/test split union does not match NPZ samples")
    prefix = args.filename_prefix
    if not prefix:
        bag_name = Path(source_metadata["bag"]).name
        prefix = bag_name.split("_v7_dual_teleop_bag", 1)[0]
    if not prefix or any(
        character
        not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
        for character in prefix
    ):
        raise ValueError(
            "--filename-prefix must contain only letters, digits, hyphens, and underscores"
        )
    total_slots = int(source_metadata["total_slots"])
    samples_01 = int(source_metadata["samples_01"])
    samples_02 = int(source_metadata["samples_02"])
    if total_slots != samples_01 + samples_02:
        raise ValueError("source metadata total_slots does not match sensor sample counts")
    if samples_01 != samples_02:
        raise ValueError(
            "the current S3-Net training loader batches both sensor streams together and "
            f"requires equal beam counts, got scan_01={samples_01}, scan_02={samples_02}; "
            "the raw 07b fixed-slot session remains valid, but it cannot yet be exported "
            "through the shared 07c training path"
        )
    range_max_01, range_max_02, pool_range_max = resolve_range_config(
        source_metadata, args
    )
    with np.load(source_files[0]) as first_sample:
        available_pedestrian_fields = {
            source for source, _ in PEDESTRIAN_FIELD_MAP.values()
            if source in first_sample
        }
        required_pedestrian_fields = {
            source for source, _ in PEDESTRIAN_FIELD_MAP.values()
        }
        if available_pedestrian_fields and (
            available_pedestrian_fields != required_pedestrian_fields
        ):
            missing = sorted(required_pedestrian_fields - available_pedestrian_fields)
            raise KeyError(
                "source contains a partial pedestrian ground-truth export; "
                f"missing {missing}"
            )
        export_pedestrian_ground_truth = bool(available_pedestrian_fields)
        pedestrian_count = (
            int(len(first_sample["pedestrian_ids"]))
            if export_pedestrian_ground_truth
            else 0
        )
    active_field_map = dict(FIELD_MAP)
    if export_pedestrian_ground_truth:
        active_field_map.update(PEDESTRIAN_FIELD_MAP)

    args.output_session.mkdir(parents=True)
    for directory in list(active_field_map) + ["intensities_lidar"]:
        (args.output_session / directory).mkdir()

    output_names = []
    frame_metadata = []
    for index, source_path in enumerate(source_files):
        with np.load(source_path) as sample:
            missing = [
                source for source, _ in active_field_map.values()
                if source not in sample
            ]
            missing.extend(
                key for key in ("scan_01_stamp_ns", "scan_02_stamp_ns") if key not in sample
            )
            if missing:
                raise KeyError(f"{source_path.name}: missing NPZ fields {sorted(set(missing))}")
            if "episode_id" not in sample:
                raise KeyError(f"{source_path.name}: missing episode_id")
            episode_id = int(sample["episode_id"])
            output_name = f"{prefix}-ep{episode_id:03d}-{index:07d}.npy"
            output_names.append(output_name)
            if sample["raw_ranges"].shape != (total_slots,):
                raise ValueError(
                    f"{source_path.name}: raw_ranges shape {sample['raw_ranges'].shape} "
                    f"!= {(total_slots,)}"
                )
            if export_pedestrian_ground_truth:
                current_count = int(len(sample["pedestrian_ids"]))
                if current_count != pedestrian_count:
                    raise ValueError(
                        f"{source_path.name}: pedestrian count {current_count} "
                        f"!= first sample count {pedestrian_count}"
                    )
            for directory, (source_key, dtype) in active_field_map.items():
                values = np.asarray(sample[source_key], dtype=dtype)
                np.save(args.output_session / directory / output_name, values, allow_pickle=False)
            np.save(
                args.output_session / "intensities_lidar" / output_name,
                np.zeros(total_slots, dtype=np.float32),
                allow_pickle=False,
            )
            frame_metadata.append(
                {
                    "name": output_name,
                    "sequence_index": index,
                    "source_npz": source_path.name,
                    "scan_01_stamp_ns": int(sample["scan_01_stamp_ns"]),
                    "scan_02_stamp_ns": int(sample["scan_02_stamp_ns"]),
                    "episode_id": episode_id,
                    "cmd_vel_stamp_ns": int(sample["cmd_vel_stamp_ns"]),
                    "cmd_vel_age_ns": int(sample["cmd_vel_age_ns"]),
                    "local_subgoal_stamp_ns": int(
                        sample["local_subgoal_stamp_ns"]
                    ),
                    "local_subgoal_age_ns": int(sample["local_subgoal_age_ns"]),
                }
            )

    source_to_output = {
        source.name: output_name
        for source, output_name in zip(source_files, output_names)
    }
    if args.split_role == "preserve":
        split_values = {
            split: [source_to_output[name] for name in names]
            for split, names in source_splits.items()
        }
        split_strategy = "preserved source episode split"
    else:
        split_values = {split: [] for split in ("train", "dev", "test")}
        split_values[args.split_role] = list(output_names)
        split_strategy = f"whole bag assigned to seed-level {args.split_role}"
    train, dev, test = (
        split_values["train"],
        split_values["dev"],
        split_values["test"],
    )
    write_split(args.output_session / "train.txt", train)
    write_split(args.output_session / "dev.txt", dev)
    write_split(args.output_session / "test.txt", test)
    frame_period_01 = frame_period_from_records(
        frame_metadata, args.frame_period_tolerance_ms, "scan_01_stamp_ns"
    )
    frame_period_02 = frame_period_from_records(
        frame_metadata, args.frame_period_tolerance_ms, "scan_02_stamp_ns"
    )

    metadata = {
        "format": "semantic2d-fixed-dual-native-v3",
        "session_name": args.session_name,
        "source_npz_session": str(args.source_session.resolve()),
        "source_metadata_sha256": sha256(source_metadata_path),
        "reverse_motion_filter": source_metadata.get("reverse_motion_filter"),
        "forward_only": bool(
            source_metadata.get("reverse_motion_filter", {}).get("enabled", False)
        ),
        "reverse_linear_x_epsilon": source_metadata.get(
            "reverse_motion_filter", {}
        ).get("linear_x_epsilon"),
        "reverse_recovery_frames": source_metadata.get(
            "reverse_motion_filter", {}
        ).get("recovery_frames_requested"),
        "samples": len(source_files),
        "samples_01": samples_01,
        "samples_02": samples_02,
        "total_slots": total_slots,
        "slot_contract": source_metadata.get("slot_contract"),
        "subgoal_source": source_metadata.get("subgoal_source"),
        "person_label_mode": source_metadata.get("person_label_mode"),
        "cmd_label_interface": source_metadata.get("cmd_label_interface"),
        "cmd_vel_max_age_ms": source_metadata.get("cmd_vel_max_age_ms"),
        "cmd_vel_angular_z_relay_scale": source_metadata.get(
            "cmd_vel_angular_z_relay_scale"
        ),
        "subgoal_lookahead": source_metadata.get("subgoal_lookahead"),
        "field_map": {
            directory: source
            for directory, (source, _) in active_field_map.items()
        },
        "pedestrian_ground_truth_exported": export_pedestrian_ground_truth,
        "pedestrian_count": pedestrian_count,
        "pedestrian_velocity_frame": (
            source_metadata.get("map_frame")
            if export_pedestrian_ground_truth
            else None
        ),
        "intensity_mode": "synthetic_zero_format_compatibility_only",
        "split_strategy": split_strategy,
        "split_role": args.split_role,
        "filename_prefix": prefix,
        "split_counts": {"train": len(train), "dev": len(dev), "test": len(test)},
        "expected_frame_period_ms": frame_period_01["expected_ms"],
        "frame_period_tolerance_ms": frame_period_01["tolerance_ms"],
        "frame_period_source": "median_scan_01_timestamp_delta",
        "frame_period_delta_ms_min": frame_period_01["min_ms"],
        "frame_period_delta_ms_max": frame_period_01["max_ms"],
        "scan_02_expected_frame_period_ms": frame_period_02["expected_ms"],
        "scan_02_frame_period_delta_ms_min": frame_period_02["min_ms"],
        "scan_02_frame_period_delta_ms_max": frame_period_02["max_ms"],
        "semantic_cnn_pool_mode": "global_virtual_angle_80",
        "semantic_cnn_supported_pool_modes": [
            "global_virtual_angle_80",
            "sensor_split_40x2",
        ],
        "pool_angle_min": float(-np.pi / 2.0),
        "pool_angle_max": float(np.pi / 2.0),
        "pool_num_bins": 80,
        "range_max_01": range_max_01,
        "range_max_02": range_max_02,
        "pool_range_max": pool_range_max,
        "pool_range_normalization": "clip_0_range_max_then_divide_by_range_max",
        "frames": frame_metadata,
        "label_names": source_metadata.get("label_names", []),
        "self_mask_mode": source_metadata.get("self_mask_mode"),
        "self_mask_calibration": source_metadata.get("self_mask_calibration"),
    }
    label_names = metadata["label_names"]
    if len(label_names) < 2 or label_names[0] != "_background_":
        raise ValueError("source metadata must define _background_ plus at least one semantic class")
    (args.output_session / "label_names.txt").write_text(
        "\n".join(label_names) + "\n", encoding="utf-8"
    )
    (args.output_session / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "session": str(args.output_session.resolve()),
                "samples": metadata["samples"],
                "samples_01": metadata["samples_01"],
                "samples_02": metadata["samples_02"],
                "range_max_01": metadata["range_max_01"],
                "range_max_02": metadata["range_max_02"],
                "pool_range_max": metadata["pool_range_max"],
                "expected_frame_period_ms": metadata["expected_frame_period_ms"],
                "scan_02_expected_frame_period_ms": metadata[
                    "scan_02_expected_frame_period_ms"
                ],
                "split_counts": metadata["split_counts"],
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
