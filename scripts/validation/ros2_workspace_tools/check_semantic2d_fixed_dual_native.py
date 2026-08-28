#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --expected-rate-01, --expected-rate-02, --rate-tolerance-percent, --report-json, --require-session-listed, --session, --source-session
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, NPZ, TXT
# 可能使用的关键环境变量：FAIL, FIELD_SPECS, PASS, PASS_WITH_WARNINGS, PEDESTRIAN_FIELD_SPECS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.836309951 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:37.993044099 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_semantic2d_fixed_dual_native.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_semantic2d_fixed_dual_native.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Validate an exported Semantic2D fixed-dual native training session."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


FIELD_SPECS = {
    "scans_lidar": (("slots",), np.dtype(np.float32), "raw_ranges"),
    "intensities_lidar": (("slots",), np.dtype(np.float32), None),
    "angles_lidar": (("slots",), np.dtype(np.float32), "raw_angles_sensor"),
    "virtual_ranges_lidar": (("slots",), np.dtype(np.float32), "virtual_ranges"),
    "virtual_angles_lidar": (("slots",), np.dtype(np.float32), "virtual_angles"),
    "range_valid_mask_lidar": (("slots",), np.dtype(np.bool_), "range_valid_mask"),
    "self_mask_lidar": (("slots",), np.dtype(np.bool_), "self_mask"),
    "valid_mask_lidar": (("slots",), np.dtype(np.bool_), "valid_mask"),
    "semantic_label": (("slots",), np.dtype(np.int16), "semantic_label"),
    "source_sensor": (("slots",), np.dtype(np.uint8), "source_sensor"),
    "raw_beam_index": (("slots",), np.dtype(np.int32), "raw_beam_index"),
    "positions": ((3,), np.dtype(np.float32), "position"),
    "velocities": ((2,), np.dtype(np.float32), "velocity"),
    "cmd_velocities": ((3,), np.dtype(np.float32), "cmd_velocity"),
    "sub_goals_local": ((2,), np.dtype(np.float32), "sub_goal_local"),
}

PEDESTRIAN_FIELD_SPECS = {
    "pedestrian_ids": (("pedestrians",), None, "pedestrian_ids"),
    "pedestrian_positions": (
        ("pedestrians", 2),
        np.dtype(np.float32),
        "pedestrian_xy_map",
    ),
    "pedestrian_velocities": (
        ("pedestrians", 2),
        np.dtype(np.float32),
        "pedestrian_velocity_map",
    ),
    "pedestrian_yaws": (
        ("pedestrians",),
        np.dtype(np.float32),
        "pedestrian_yaw_map",
    ),
    "pedestrian_leg_positions": (
        ("pedestrians", 2, 2),
        np.dtype(np.float32),
        "pedestrian_leg_xy_map",
    ),
    "pedestrian_truth_timestamps": (
        (),
        np.dtype(np.int64),
        "pedestrian_truth_stamp_ns",
    ),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--source-session", type=Path)
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--require-session-listed", action="store_true")
    parser.add_argument("--expected-rate-01", type=float)
    parser.add_argument("--expected-rate-02", type=float)
    parser.add_argument("--rate-tolerance-percent", type=float, default=10.0)
    parser.add_argument("--report-json", type=Path)
    return parser.parse_args()


def read_names(path):
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_shape(spec, total_slots, pedestrian_count):
    return tuple(
        total_slots
        if value == "slots"
        else pedestrian_count
        if value == "pedestrians"
        else value
        for value in spec
    )


def arrays_equal(actual, expected):
    if actual.shape != expected.shape:
        return False
    if actual.dtype.kind in ("U", "S", "O") or expected.dtype.kind in ("U", "S", "O"):
        return np.array_equal(actual, expected)
    return np.array_equal(actual, expected, equal_nan=True)


def finite_positive_metadata_value(metadata, key, errors):
    value = metadata.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0.0:
        errors.append(f"metadata.{key} must be a positive finite number, got {value!r}")
        return None
    return float(value)


def fixed_mask_from_calibration(calibration, key, beam_count, errors):
    """Rebuild a fixed raw-beam mask from compact v3 or legacy v2 metadata."""
    runs_key = f"{key}_masked_beam_runs"
    indices_key = f"{key}_masked_beam_indices"
    count_key = f"{key}_masked_beam_count"
    mask = np.zeros(beam_count, dtype=np.bool_)
    runs = calibration.get(runs_key)
    if runs is not None:
        if not isinstance(runs, list):
            errors.append(f"self mask calibration {runs_key} is not a list")
            return None
        previous_end = -1
        for run in runs:
            if (
                not isinstance(run, list)
                or len(run) != 2
                or any(not isinstance(value, int) for value in run)
            ):
                errors.append(f"self mask calibration {runs_key} has an invalid run")
                return None
            start, end = run
            if start < 0 or end < start or end >= beam_count or start <= previous_end:
                errors.append(f"self mask calibration {runs_key} has invalid or overlapping runs")
                return None
            mask[start : end + 1] = True
            previous_end = end
    else:
        indices = calibration.get(indices_key)
        if not isinstance(indices, list) or any(not isinstance(value, int) for value in indices):
            errors.append(
                f"self mask calibration must provide {runs_key} or {indices_key}"
            )
            return None
        if len(set(indices)) != len(indices):
            errors.append(f"self mask calibration {indices_key} contains duplicates")
            return None
        if any(value < 0 or value >= beam_count for value in indices):
            errors.append(f"self mask calibration {indices_key} has out-of-range beam indices")
            return None
        mask[indices] = True
    if calibration.get(count_key) != int(mask.sum()):
        errors.append(f"self mask calibration {count_key} does not match stored beam mask")
        return None
    return mask


def fixed_dual_contract_differences(reference, candidate):
    """Return incompatibilities that would make a mixed fixed-dual root ambiguous."""
    differences = []
    exact_keys = (
        "format",
        "samples_01",
        "samples_02",
        "total_slots",
        "slot_contract",
        "semantic_cnn_pool_mode",
        "pool_num_bins",
        "pool_range_normalization",
        "self_mask_mode",
        "forward_only",
        "reverse_recovery_frames",
        "subgoal_source",
        "person_label_mode",
        "pedestrian_ground_truth_exported",
        "cmd_label_interface",
    )
    for key in exact_keys:
        if reference.get(key) != candidate.get(key):
            differences.append(
                f"{key}: {reference.get(key)!r} != {candidate.get(key)!r}"
            )
    for key in (
        "range_max_01",
        "range_max_02",
        "pool_range_max",
        "pool_angle_min",
        "pool_angle_max",
        "frame_period_tolerance_ms",
        "cmd_vel_max_age_ms",
        "cmd_vel_angular_z_relay_scale",
    ):
        try:
            matches = math.isclose(
                float(reference[key]), float(candidate[key]), rel_tol=0.0, abs_tol=1e-6
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(
                f"{key}: {reference.get(key)!r} != {candidate.get(key)!r}"
            )
    if reference.get("forward_only") is True and candidate.get("forward_only") is True:
        try:
            reverse_epsilon_matches = math.isclose(
                float(reference["reverse_linear_x_epsilon"]),
                float(candidate["reverse_linear_x_epsilon"]),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        except (KeyError, TypeError, ValueError):
            reverse_epsilon_matches = False
        if not reverse_epsilon_matches:
            differences.append(
                "reverse_linear_x_epsilon: "
                f"{reference.get('reverse_linear_x_epsilon')!r} != "
                f"{candidate.get('reverse_linear_x_epsilon')!r}"
            )
    for key in ("expected_frame_period_ms", "scan_02_expected_frame_period_ms"):
        try:
            tolerance = min(
                float(reference["frame_period_tolerance_ms"]),
                float(candidate["frame_period_tolerance_ms"]),
            ) / 4.0
            matches = math.isclose(
                float(reference[key]), float(candidate[key]), rel_tol=0.0, abs_tol=tolerance
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(
                f"{key}: {reference.get(key)!r} != {candidate.get(key)!r}"
            )
    if reference.get("label_names") != candidate.get("label_names"):
        differences.append("label_names differ")
    return differences


def rate_from_period_ms(period_ms):
    return 1000.0 / period_ms


def validate_expected_rate(observed_period_ms, expected_rate, label, args, errors):
    if expected_rate is None:
        return
    if not math.isfinite(expected_rate) or expected_rate <= 0.0:
        errors.append(f"{label} expected rate must be positive and finite")
        return
    observed_rate = rate_from_period_ms(observed_period_ms)
    allowed_error = max(0.25, expected_rate * args.rate_tolerance_percent / 100.0)
    if abs(observed_rate - expected_rate) > allowed_error:
        errors.append(
            f"{label} observed rate {observed_rate:.6g} Hz is not within "
            f"{args.rate_tolerance_percent:g}% of expected {expected_rate:.6g} Hz"
        )


def window_count(names, frame_records, sequence_length, expected_period_ms, tolerance_ms):
    selected = set(names)
    count = 0
    for end in range(len(frame_records)):
        start = end - sequence_length + 1
        if start < 0:
            continue
        window = frame_records[start : end + 1]
        if any(record.get("name") not in selected for record in window):
            continue
        episode_ids = {int(record.get("episode_id", -1)) for record in window}
        if len(episode_ids) != 1 or -1 in episode_ids:
            continue
        stamps = [int(record["scan_01_stamp_ns"]) for record in window]
        deltas = [
            (right - left) / 1_000_000.0
            for left, right in zip(stamps, stamps[1:])
        ]
        if all(abs(delta - expected_period_ms) <= tolerance_ms for delta in deltas):
            count += 1
    return count


def main():
    args = parse_args()
    errors = []
    warnings = []
    if (args.expected_rate_01 is None) != (args.expected_rate_02 is None):
        errors.append("--expected-rate-01 and --expected-rate-02 must be supplied together")
    if not math.isfinite(args.rate_tolerance_percent) or args.rate_tolerance_percent <= 0.0:
        errors.append("--rate-tolerance-percent must be positive and finite")
    metadata_path = args.session / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(metadata_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("format") != "semantic2d-fixed-dual-native-v3":
        errors.append(
            "metadata.format must be semantic2d-fixed-dual-native-v3 for this checker"
        )
    session_label_names_path = args.session / "label_names.txt"
    if not session_label_names_path.is_file():
        errors.append("session is missing label_names.txt")
    elif read_names(session_label_names_path) != metadata.get("label_names"):
        errors.append("session label_names.txt differs from metadata.label_names")
    total_slots = int(metadata.get("total_slots", -1))
    sample_count = int(metadata.get("samples", -1))
    samples_01 = int(metadata.get("samples_01", -1))
    samples_02 = int(metadata.get("samples_02", -1))
    pedestrian_ground_truth_exported = (
        metadata.get("pedestrian_ground_truth_exported") is True
    )
    forward_only = metadata.get("forward_only") is True
    reverse_linear_x_epsilon = metadata.get("reverse_linear_x_epsilon")
    if forward_only and (
        not isinstance(reverse_linear_x_epsilon, (int, float))
        or not math.isfinite(reverse_linear_x_epsilon)
        or reverse_linear_x_epsilon <= 0.0
    ):
        errors.append(
            "forward-only metadata has invalid reverse_linear_x_epsilon"
        )
        reverse_linear_x_epsilon = 0.0
    negative_linear_x_samples = 0
    pedestrian_count = int(metadata.get("pedestrian_count", 0))
    field_specs = dict(FIELD_SPECS)
    if pedestrian_ground_truth_exported:
        if pedestrian_count <= 0:
            errors.append(
                "metadata.pedestrian_count must be positive when ground truth is exported"
            )
        field_specs.update(PEDESTRIAN_FIELD_SPECS)
    if total_slots != samples_01 + samples_02 or total_slots <= 0:
        errors.append("metadata slot counts are inconsistent")

    if metadata.get("semantic_cnn_pool_mode") != "global_virtual_angle_80":
        errors.append("metadata.semantic_cnn_pool_mode is not global_virtual_angle_80")
    if metadata.get("pool_num_bins") != 80:
        errors.append("metadata.pool_num_bins is not 80")
    range_max_01 = finite_positive_metadata_value(metadata, "range_max_01", errors)
    range_max_02 = finite_positive_metadata_value(metadata, "range_max_02", errors)
    pool_range_max = finite_positive_metadata_value(metadata, "pool_range_max", errors)
    if (
        range_max_01 is not None
        and range_max_02 is not None
        and pool_range_max is not None
        and pool_range_max + 1e-9 < max(range_max_01, range_max_02)
    ):
        errors.append("metadata.pool_range_max does not cover both sensor range maxima")
    if not math.isclose(float(metadata.get("pool_angle_min", 0.0)), -math.pi / 2.0, abs_tol=1e-6):
        errors.append("metadata.pool_angle_min is not -pi/2")
    if not math.isclose(float(metadata.get("pool_angle_max", 0.0)), math.pi / 2.0, abs_tol=1e-6):
        errors.append("metadata.pool_angle_max is not pi/2")

    frame_records = metadata.get("frames", [])
    if len(frame_records) != sample_count:
        errors.append("metadata.frames count does not match samples")
    expected_names = [
        str(record.get("name", "")) for record in frame_records
    ]
    if (
        any(not name.endswith(".npy") for name in expected_names)
        or len(set(expected_names)) != len(expected_names)
    ):
        errors.append("metadata.frames contains invalid or duplicate names")
    for directory, (shape_spec, dtype, _) in field_specs.items():
        path = args.session / directory
        if not path.is_dir():
            errors.append(f"missing directory {directory}/")
            continue
        names = [item.name for item in sorted(path.glob("*.npy"))]
        if names != sorted(expected_names):
            errors.append(f"{directory}/ filenames do not match metadata.frames")

    source_session = args.source_session
    if source_session is None and metadata.get("source_npz_session"):
        source_session = Path(metadata["source_npz_session"])
    source_files = sorted((source_session / "samples").glob("*.npz")) if source_session else []
    if source_session and len(source_files) != sample_count:
        errors.append(f"source NPZ count {len(source_files)} != exported sample count {sample_count}")
    if source_session:
        source_metadata_path = source_session / "metadata.json"
        if not source_metadata_path.is_file():
            errors.append("source session is missing metadata.json")
        elif metadata.get("source_metadata_sha256") != sha256(source_metadata_path):
            errors.append("metadata.source_metadata_sha256 does not match source metadata")

    stamps = []
    scan_02_stamps = []
    fixed_self_mask = None
    fixed_self_mask_mode = metadata.get("self_mask_mode")
    if fixed_self_mask_mode == "first-synchronized-pair-fixed-beam-identity":
        calibration = metadata.get("self_mask_calibration")
        if not isinstance(calibration, dict):
            errors.append("fixed self-mask metadata is missing self_mask_calibration")
        else:
            first_mask = fixed_mask_from_calibration(
                calibration, "scan_01", samples_01, errors
            )
            second_mask = fixed_mask_from_calibration(
                calibration, "scan_02", samples_02, errors
            )
            if first_mask is not None and second_mask is not None:
                fixed_self_mask = np.concatenate((first_mask, second_mask))
    for index, name in enumerate(expected_names):
        per_field = {}
        for directory, (shape_spec, dtype, source_key) in field_specs.items():
            path = args.session / directory / name
            if not path.is_file():
                continue
            array = np.load(path, allow_pickle=False)
            per_field[directory] = array
            shape = expected_shape(shape_spec, total_slots, pedestrian_count)
            if array.shape != shape:
                errors.append(f"{directory}/{name}: shape {array.shape} != {shape}")
            if dtype is None and array.dtype.kind != "U":
                errors.append(
                    f"{directory}/{name}: dtype {array.dtype} is not Unicode"
                )
            elif dtype is not None and array.dtype != dtype:
                errors.append(f"{directory}/{name}: dtype {array.dtype} != {dtype}")
            if directory.startswith("pedestrian_") and directory != "pedestrian_ids":
                if np.any(~np.isfinite(array)):
                    errors.append(
                        f"{directory}/{name}: contains non-finite values"
                    )
        if "intensities_lidar" in per_field and np.any(per_field["intensities_lidar"] != 0.0):
            errors.append(f"intensities_lidar/{name}: expected format-compatibility zeros")
        if "cmd_velocities" in per_field:
            negative_linear_x_samples += int(
                float(per_field["cmd_velocities"][0])
                < -float(reverse_linear_x_epsilon or 0.0)
            )
        if all(key in per_field for key in ("valid_mask_lidar", "range_valid_mask_lidar", "self_mask_lidar")):
            expected_valid = per_field["range_valid_mask_lidar"] & ~per_field["self_mask_lidar"]
            if not np.array_equal(per_field["valid_mask_lidar"], expected_valid):
                errors.append(f"{name}: valid mask contract mismatch")
        if all(key in per_field for key in ("semantic_label", "valid_mask_lidar")):
            if np.any(per_field["semantic_label"][~per_field["valid_mask_lidar"]] != -1):
                errors.append(f"{name}: invalid beams do not use semantic ignore label -1")
        if all(key in per_field for key in ("virtual_ranges_lidar", "virtual_angles_lidar", "valid_mask_lidar")):
            valid = per_field["valid_mask_lidar"]
            if np.any(~np.isfinite(per_field["virtual_ranges_lidar"][valid])) or np.any(
                ~np.isfinite(per_field["virtual_angles_lidar"][valid])
            ):
                errors.append(f"{name}: valid slots have non-finite virtual coordinates")
            if np.any(~np.isnan(per_field["virtual_ranges_lidar"][~valid])) or np.any(
                ~np.isnan(per_field["virtual_angles_lidar"][~valid])
            ):
                errors.append(f"{name}: invalid slots do not use NaN virtual coordinates")
        if (
            fixed_self_mask_mode == "first-synchronized-pair-fixed-beam-identity"
            and fixed_self_mask is not None
            and "self_mask_lidar" in per_field
            and not np.array_equal(per_field["self_mask_lidar"], fixed_self_mask)
        ):
            errors.append(f"{name}: self mask differs from the fixed calibration")

        if index < len(frame_records):
            record = frame_records[index]
            if (
                record.get("name") != name
                or record.get("source_npz") != f"{index:07d}.npz"
                or int(record.get("sequence_index", index)) != index
            ):
                errors.append(f"metadata.frames[{index}] filename mapping mismatch")
            stamps.append(int(record.get("scan_01_stamp_ns", -1)))
            scan_02_stamps.append(int(record.get("scan_02_stamp_ns", -1)))

        if index < len(source_files):
            with np.load(source_files[index]) as source:
                for directory, (_, _, source_key) in field_specs.items():
                    if source_key is None or directory not in per_field:
                        continue
                    if source_key not in source:
                        errors.append(f"{source_files[index].name}: source field {source_key} is missing")
                    elif not arrays_equal(per_field[directory], source[source_key]):
                        errors.append(f"{directory}/{name}: NPZ->NPY value mismatch")
                if index < len(frame_records):
                    record = frame_records[index]
                    if int(record.get("scan_01_stamp_ns", -1)) != int(source["scan_01_stamp_ns"]):
                        errors.append(f"metadata.frames[{index}] scan_01 timestamp mismatch")
                    if int(record.get("scan_02_stamp_ns", -1)) != int(source["scan_02_stamp_ns"]):
                        errors.append(f"metadata.frames[{index}] scan_02 timestamp mismatch")

    if forward_only and negative_linear_x_samples:
        errors.append(
            "forward-only training session contains "
            f"{negative_linear_x_samples} negative linear.x sample(s)"
        )

    splits = {}
    for split in ("train", "dev", "test"):
        path = args.session / f"{split}.txt"
        if not path.is_file():
            errors.append(f"missing {split}.txt")
            splits[split] = []
        else:
            splits[split] = read_names(path)
    split_sets = {key: set(value) for key, value in splits.items()}
    if any(
        split_sets[left] & split_sets[right]
        for left, right in (("train", "dev"), ("train", "test"), ("dev", "test"))
    ):
        errors.append("train/dev/test splits overlap")
    if set().union(*split_sets.values()) != set(expected_names):
        errors.append("train/dev/test union does not match exported samples")
    record_order = {name: index for index, name in enumerate(expected_names)}
    for split, names in splits.items():
        indices = [record_order.get(name, -1) for name in names]
        if indices != sorted(indices):
            errors.append(f"{split} split is not in metadata frame order")

    if len(stamps) == sample_count and stamps:
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            errors.append("scan_01 timestamps are not strictly increasing")
    expected_period = finite_positive_metadata_value(
        metadata, "expected_frame_period_ms", errors
    )
    tolerance = finite_positive_metadata_value(
        metadata, "frame_period_tolerance_ms", errors
    )
    if expected_period is None:
        expected_period = 0.0
    if tolerance is None:
        tolerance = 0.0
    if len(stamps) >= 2 and expected_period > 0.0:
        measured_period = float(np.median(np.diff(stamps).astype(np.float64) / 1_000_000.0))
        if not math.isclose(measured_period, expected_period, rel_tol=0.0, abs_tol=1e-6):
            errors.append(
                "metadata.expected_frame_period_ms does not match median scan_01 timestamp delta"
            )
    else:
        measured_period = None
    expected_period_02 = finite_positive_metadata_value(
        metadata, "scan_02_expected_frame_period_ms", errors
    )
    if len(scan_02_stamps) == sample_count and len(scan_02_stamps) >= 2:
        if any(right <= left for left, right in zip(scan_02_stamps, scan_02_stamps[1:])):
            errors.append("scan_02 timestamps are not strictly increasing")
        measured_period_02 = float(
            np.median(np.diff(scan_02_stamps).astype(np.float64) / 1_000_000.0)
        )
        if expected_period_02 is not None and not math.isclose(
            measured_period_02, expected_period_02, rel_tol=0.0, abs_tol=1e-6
        ):
            errors.append(
                "metadata.scan_02_expected_frame_period_ms does not match median "
                "scan_02 timestamp delta"
            )
    else:
        measured_period_02 = None
    if measured_period is not None:
        validate_expected_rate(measured_period, args.expected_rate_01, "scan_01", args, errors)
    if measured_period_02 is not None:
        validate_expected_rate(
            measured_period_02, args.expected_rate_02, "scan_02", args, errors
        )
    window_counts = {
        split: window_count(names, frame_records, 10, expected_period, tolerance)
        if len(frame_records) == sample_count
        else 0
        for split, names in splits.items()
    }
    split_role = metadata.get("split_role", "preserve")
    required_window_splits = (
        ("train", "dev") if split_role == "preserve" else (split_role,)
    )
    for split in required_window_splits:
        if split not in window_counts or window_counts[split] == 0:
            errors.append(
                f"{split} split has no valid continuous 10-frame SemanticCNN window"
            )

    if args.dataset_root:
        if not args.dataset_root.is_dir():
            errors.append(f"dataset root does not exist: {args.dataset_root}")
        else:
            root_label_names = args.dataset_root / "label_names.txt"
            if root_label_names.is_file():
                root_names = read_names(root_label_names)
                if root_names != metadata.get("label_names"):
                    errors.append("dataset root label_names.txt differs from this session")
            for existing in sorted(args.dataset_root.iterdir()):
                if not existing.is_dir() or existing.name.startswith("."):
                    continue
                if existing.resolve() == args.session.resolve():
                    continue
                existing_metadata_path = existing / "metadata.json"
                if not existing_metadata_path.is_file():
                    errors.append(
                        f"dataset root has session without metadata.json: {existing.name}"
                    )
                    continue
                existing_metadata = json.loads(
                    existing_metadata_path.read_text(encoding="utf-8")
                )
                differences = fixed_dual_contract_differences(metadata, existing_metadata)
                if differences:
                    errors.append(
                        f"dataset root session {existing.name} is incompatible: "
                        + "; ".join(differences)
                    )
            if args.require_session_listed:
                index_path = args.dataset_root / "dataset.txt"
                if not index_path.is_file():
                    errors.append("dataset root is missing dataset.txt")
                elif args.session.name not in read_names(index_path):
                    errors.append("checked session is not listed in dataset.txt")

    report = {
        "status": "FAIL" if errors else ("PASS_WITH_WARNINGS" if warnings else "PASS"),
        "session": str(args.session.resolve()),
        "source_session": str(source_session.resolve()) if source_session else None,
        "samples": sample_count,
        "total_slots": total_slots,
        "split_counts": {key: len(value) for key, value in splits.items()},
        "semantic_cnn_window_counts": window_counts,
        "range_max_01": range_max_01,
        "range_max_02": range_max_02,
        "pool_range_max": pool_range_max,
        "expected_frame_period_ms": expected_period,
        "scan_02_expected_frame_period_ms": expected_period_02,
        "frame_period_tolerance_ms": tolerance,
        "observed_scan_01_rate_hz": (
            rate_from_period_ms(measured_period) if measured_period is not None else None
        ),
        "observed_scan_02_rate_hz": (
            rate_from_period_ms(measured_period_02)
            if measured_period_02 is not None
            else None
        ),
        "self_mask_mode": fixed_self_mask_mode,
        "forward_only": forward_only,
        "negative_linear_x_samples": negative_linear_x_samples,
        "reverse_motion_filter": metadata.get("reverse_motion_filter"),
        "warnings": warnings,
        "errors": errors[:200],
        "error_count": len(errors),
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.report_json:
        args.report_json.parent.mkdir(parents=True, exist_ok=True)
        args.report_json.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    if errors:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
