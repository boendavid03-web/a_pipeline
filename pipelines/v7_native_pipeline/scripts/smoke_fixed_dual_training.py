#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --device, --report-json, --s3net-root, --s3net-stats, --semantic-cnn-root
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, TXT
# 可能使用的关键环境变量：PASS, POOL_MODES
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.488295204 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.915035776 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/lovasz_losses.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/lovasz_losses.py; /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜smoke_fixed_dual_training.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Run strict one-batch S3-Net and SemanticCNN smoke checks."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import torch


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--s3net-root", required=True, type=Path)
    parser.add_argument("--semantic-cnn-root", required=True, type=Path)
    parser.add_argument("--s3net-stats", required=True, type=Path)
    parser.add_argument("--report-json", required=True, type=Path)
    parser.add_argument("--device", default="auto")
    return parser.parse_args()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def choose_device(value):
    if value == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(value)


def load_num_classes(dataset_root):
    path = dataset_root / "label_names.txt"
    if not path.is_file():
        raise FileNotFoundError(f"missing dataset label names: {path}")
    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    names = [name for name in names if name]
    if len(names) < 2 or names[0] != "_background_":
        raise ValueError(f"invalid label names file: {path}")
    return len(names)


def dataset_sessions(dataset_root):
    index_path = dataset_root / "dataset.txt"
    names = [line.strip().rstrip("/") for line in index_path.read_text().splitlines()]
    sessions = [dataset_root / name for name in names if name]
    if not sessions:
        raise ValueError(f"dataset index contains no sessions: {index_path}")
    return sessions


def session_metadata(session):
    path = session / "metadata.json"
    if not path.is_file():
        raise FileNotFoundError(f"missing fixed-dual metadata: {path}")
    metadata = json.loads(path.read_text(encoding="utf-8"))
    if metadata.get("format") != "semantic2d-fixed-dual-native-v3":
        raise ValueError(f"unsupported fixed-dual metadata format in {path}")
    if metadata.get("subgoal_source") != "online":
        raise ValueError(f"smoke requires causal online subgoals in {path}")
    frames = metadata.get("frames", [])
    if any(
        key not in frame
        for frame in frames
        for key in (
            "episode_id",
            "cmd_vel_stamp_ns",
            "cmd_vel_age_ns",
            "local_subgoal_stamp_ns",
            "local_subgoal_age_ns",
        )
    ):
        raise ValueError(f"frame audit fields are incomplete in {path}")
    samples_01 = int(metadata.get("samples_01", 0))
    samples_02 = int(metadata.get("samples_02", 0))
    if min(samples_01, samples_02) <= 0:
        raise ValueError(f"invalid sensor sample counts in {path}")
    if samples_01 != samples_02:
        raise ValueError(
            "the current S3-Net trainer batches both sensor streams together and "
            f"requires equal beam counts, got {samples_01} and {samples_02} in {path}"
        )
    return metadata, samples_01, samples_02


def fixed_dual_contract_differences(reference, candidate):
    differences = []
    for key in (
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
        "label_names",
    ):
        if reference.get(key) != candidate.get(key):
            differences.append(key)
    for key in (
        "range_max_01",
        "range_max_02",
        "pool_range_max",
        "pool_angle_min",
        "pool_angle_max",
        "frame_period_tolerance_ms",
    ):
        try:
            matches = math.isclose(
                float(reference[key]), float(candidate[key]), rel_tol=0.0, abs_tol=1e-6
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(key)
    if reference.get("forward_only") is True and candidate.get("forward_only") is True:
        try:
            matches = math.isclose(
                float(reference["reverse_linear_x_epsilon"]),
                float(candidate["reverse_linear_x_epsilon"]),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append("reverse_linear_x_epsilon")
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
            differences.append(key)
    return differences


def require_consistent_session_contracts(sessions):
    reference, _, _ = session_metadata(sessions[0])
    for session in sessions[1:]:
        candidate, _, _ = session_metadata(session)
        differences = fixed_dual_contract_differences(reference, candidate)
        require(
            not differences,
            f"dataset root mixes incompatible fixed-dual sessions: {session.name}: {differences}",
        )
    return reference


def s3net_smoke(args, device):
    num_classes = load_num_classes(args.dataset_root)
    model_module = load_module("fixed_dual_s3_model", args.s3net_root / "scripts/model.py")
    lovasz_module = load_module("fixed_dual_lovasz", args.s3net_root / "scripts/lovasz_losses.py")
    dataset = model_module.VaeTestDataset(
        str(args.dataset_root), "train", stats_path=str(args.s3net_stats)
    )
    sessions = dataset_sessions(args.dataset_root)
    metadata = require_consistent_session_contracts(sessions)
    _, samples_01, samples_02 = session_metadata(sessions[0])
    expected_sensor_samples = 0
    for session in sessions:
        _, current_samples_01, current_samples_02 = session_metadata(session)
        require(
            (current_samples_01, current_samples_02) == (samples_01, samples_02),
            "dataset root mixes fixed dual-LiDAR slot counts",
        )
        expected_sensor_samples += 2 * len(
            [name for name in (session / "train.txt").read_text().splitlines() if name]
        )
    require(
        len(dataset) == expected_sensor_samples,
        f"expected {expected_sensor_samples} sensor samples, got {len(dataset)}",
    )
    require(len(dataset) >= 2, "S3-Net smoke requires at least one paired train frame")
    first = dataset[0]
    second = dataset[1]
    require(first["source_sensor_id"] == 0, "first S3-Net sample is not sensor 0")
    require(second["source_sensor_id"] == 1, "second S3-Net sample is not sensor 1")
    for sample, expected_samples in ((first, samples_01), (second, samples_02)):
        require(
            tuple(sample["scan"].shape) == (expected_samples,),
            f"S3-Net range shape is not ({expected_samples},)",
        )
        require(
            tuple(sample["angle_incidence"].shape) == (expected_samples,),
            f"incidence shape is not ({expected_samples},)",
        )
        require(
            tuple(sample["label"].shape) == (expected_samples,),
            f"S3-Net label shape is not ({expected_samples},)",
        )
        invalid = ~sample["valid_mask"]
        require(torch.all(sample["scan"][invalid] == 0), "invalid normalized ranges are not zero")
        require(
            torch.all(sample["angle_incidence"][invalid] == 0),
            "invalid normalized incidence values are not zero",
        )
        require(torch.isfinite(sample["scan"]).all(), "non-finite range input")
        require(torch.isfinite(sample["angle_incidence"]).all(), "non-finite incidence input")

    loader = torch.utils.data.DataLoader(dataset, batch_size=2, shuffle=False)
    batch = next(iter(loader))
    model = model_module.S3Net(
        input_channels=model_module.feature_mode_num_channels("range_incidence"),
        output_channels=num_classes,
        feature_mode="range_incidence",
    ).to(device)
    scans = batch["scan"].to(device)
    intensities = batch["intensity"].to(device)
    incidence = batch["angle_incidence"].to(device)
    labels = batch["label"].to(device)
    require(torch.isfinite(scans).all(), "non-finite S3-Net batch ranges")
    require(torch.isfinite(incidence).all(), "non-finite S3-Net batch incidence")
    _, logits, kl_loss = model(scans, intensities, incidence)
    require(
        tuple(logits.shape) == (2, num_classes, samples_01),
        f"unexpected S3-Net output {logits.shape}",
    )
    ce_loss = torch.nn.CrossEntropyLoss(ignore_index=-1)(logits, labels)
    lovasz_loss, _ = lovasz_module.LovaszSoftmax(
        reduction="sum", ignore_index=-1
    )(logits, labels)
    loss = ce_loss + 0.01 * kl_loss + lovasz_loss.sum()
    require(torch.isfinite(loss), "S3-Net smoke loss is not finite")
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    optimizer.step()
    checkpoint_dir = args.report_json.parent / f"{args.report_json.stem}_checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = checkpoint_dir / "s3net_smoke.pt"
    torch.save(model.state_dict(), checkpoint)
    reloaded = model_module.S3Net(
        input_channels=model_module.feature_mode_num_channels("range_incidence"),
        output_channels=num_classes,
        feature_mode="range_incidence",
    ).to(device)
    reloaded.load_state_dict(
        torch.load(checkpoint, map_location=device, weights_only=True),
        strict=True,
    )

    session = sessions[0]
    first_name = session_metadata(session)[0]["frames"][0]["name"]
    ranges = np.load(session / "scans_lidar" / first_name)
    angles = np.load(session / "angles_lidar" / first_name)
    source_sensor = np.load(session / "source_sensor" / first_name)
    segmented = model_module.angle_incidence_from_scan(
        ranges, angles, source_sensor=source_sensor
    )
    independent = np.concatenate(
        tuple(
            model_module.angle_incidence_from_scan(
                ranges[source_sensor == sensor_id], angles[source_sensor == sensor_id]
            )
            for sensor_id in (0, 1)
        )
    )
    require(
        np.array_equal(segmented, independent, equal_nan=True),
        "incidence crosses the raw sensor boundary",
    )
    probe = np.asarray([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
    probe_angles = np.linspace(-0.3, 0.3, 4, dtype=np.float32)
    base = model_module.angle_incidence_from_scan(probe, probe_angles)
    changed_last = probe.copy()
    changed_last[-1] = 20.0
    changed_first = probe.copy()
    changed_first[0] = 20.0
    require(
        np.isclose(base[0], model_module.angle_incidence_from_scan(changed_last, probe_angles)[0]),
        "beam 0 incorrectly uses the final beam as a circular neighbor",
    )
    require(
        np.isclose(base[-1], model_module.angle_incidence_from_scan(changed_first, probe_angles)[-1]),
        "final beam incorrectly uses beam 0 as a circular neighbor",
    )

    length_shapes = {}
    model.eval()
    with torch.no_grad():
        for length in sorted({samples_01, samples_02, 1081}):
            values = torch.zeros(1, length, device=device)
            _, regression_logits, _ = model(values, values, values)
            require(
                regression_logits.shape[-1] == length,
                f"S3-Net output length mismatch for {length}",
            )
            length_shapes[str(length)] = list(regression_logits.shape)
    return {
        "feature_mode": "range_incidence",
        "num_classes": num_classes,
        "dataset_sensor_samples": len(dataset),
        "item_shape": [samples_01],
        "batch_output_shape": list(logits.shape),
        "loss": float(loss.detach().cpu()),
        "ce_loss": float(ce_loss.detach().cpu()),
        "lovasz_loss": float(lovasz_loss.sum().detach().cpu()),
        "kl_loss": float(kl_loss.detach().cpu()),
        "incidence_segment_match": True,
        "non_circular_endpoints": True,
        "length_regression_shapes": length_shapes,
        "optimizer_step": True,
        "checkpoint": str(checkpoint.resolve()),
        "strict_checkpoint_reload": True,
    }


def semantic_cnn_smoke(args, device):
    module = load_module(
        "fixed_dual_semantic_cnn_model",
        args.semantic_cnn_root / "training/scripts/model.py",
    )
    sessions = dataset_sessions(args.dataset_root)
    metadata = require_consistent_session_contracts(sessions)
    pool_range_max = float(metadata["pool_range_max"])
    results = {}
    for mode in module.POOL_MODES:
        dataset = module.NavDataset(str(args.dataset_root), "train", pooling_mode=mode)
        require(len(dataset) > 0, f"{mode}: train split has no valid windows")
        sample = dataset[0]
        require(tuple(sample["scan_map"].shape) == (80, 80), "scan_map shape is not (80,80)")
        require(tuple(sample["semantic_map"].shape) == (80, 80), "semantic_map shape is not (80,80)")
        require(tuple(sample["sub_goal"].shape) == (2,), "sub_goal shape is not (2,)")
        require(tuple(sample["target"].shape) == (2,), "target shape is not (2,)")
        require(tuple(sample["bin_valid_mask"].shape) == (10, 80), "bin_valid_mask shape mismatch")
        require(torch.isfinite(sample["scan_map"]).all(), "non-finite SemanticCNN scan map")

        window = dataset.windows[0]
        end_name = window["names"][-1]
        command = np.load(Path(window["root"]) / "cmd_velocities" / end_name)[[0, 2]]
        require(np.array_equal(sample["target"].numpy(), command), "target is not cmd_velocity[[0,2]]")

        loader = torch.utils.data.DataLoader(dataset, batch_size=min(2, len(dataset)), shuffle=False)
        batch = next(iter(loader))
        fusion = torch.stack((batch["scan_map"], batch["semantic_map"]), dim=1)
        require(
            tuple(fusion.shape) == (len(batch["scan_map"]), 2, 80, 80),
            "SemanticCNN batch input is not (B,2,80,80)",
        )
        model = module.SemanticCNN(module.Bottleneck, [2, 1, 1]).to(device)
        output = model(
            batch["scan_map"].to(device),
            batch["semantic_map"].to(device),
            batch["sub_goal"].to(device),
        )
        require(
            tuple(output.shape) == (len(batch["scan_map"]), 2),
            "SemanticCNN output is not (B,2)",
        )
        loss = torch.nn.MSELoss()(output, batch["target"].to(device))
        require(torch.isfinite(loss), "SemanticCNN smoke loss is not finite")
        optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()
        checkpoint_dir = args.report_json.parent / f"{args.report_json.stem}_checkpoints"
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        checkpoint = checkpoint_dir / f"semantic_cnn_{mode}_smoke.pt"
        torch.save(model.state_dict(), checkpoint)
        reloaded = module.SemanticCNN(module.Bottleneck, [2, 1, 1]).to(device)
        reloaded.load_state_dict(
            torch.load(checkpoint, map_location=device, weights_only=True),
            strict=True,
        )
        results[mode] = {
            "windows": len(dataset),
            "scan_map_shape": list(sample["scan_map"].shape),
            "semantic_map_shape": list(sample["semantic_map"].shape),
            "sub_goal_shape": list(sample["sub_goal"].shape),
            "target_shape": list(sample["target"].shape),
            "batch_fusion_shape": list(fusion.shape),
            "model_output_shape": list(output.shape),
            "loss": float(loss.detach().cpu()),
            "target_matches_final_frame_cmd": True,
            "optimizer_step": True,
            "checkpoint": str(checkpoint.resolve()),
            "strict_checkpoint_reload": True,
        }

    empty = np.asarray([], dtype=np.float32)
    pooled = module._pool_virtual_angles(
        empty,
        empty,
        np.asarray([], dtype=np.int16),
        np.asarray([], dtype=np.bool_),
        80,
        pool_range_max,
    )
    require(np.all(pooled[0] == 1.0) and np.all(pooled[1] == 1.0), "empty ranges are not 1.0")
    clipped = module._pool_virtual_angles(
        np.asarray([pool_range_max * 2.0], dtype=np.float32),
        np.asarray([0.0], dtype=np.float32),
        np.asarray([3], dtype=np.int16),
        np.asarray([True]),
        80,
        pool_range_max,
    )
    require(float(clipped[0].min()) == 1.0, "range values are not clipped before normalization")
    ignore_probe = module._pool_virtual_angles(
        np.asarray([1.0, 2.0], dtype=np.float32),
        np.asarray([0.0, 0.001], dtype=np.float32),
        np.asarray([-1, 5], dtype=np.int16),
        np.asarray([True, True]),
        80,
        pool_range_max,
    )
    active = np.flatnonzero(ignore_probe[4])
    require(active.size == 1 and ignore_probe[2][active[0]] == 5, "ignore label polluted nearest semantic")

    window_counts = {
        split: len(module.NavDataset(str(args.dataset_root), split))
        for split in ("train", "dev", "test")
    }
    require(window_counts["train"] > 0, "train split has no valid SemanticCNN windows")
    require(window_counts["dev"] > 0, "dev split has no valid SemanticCNN windows")
    require(window_counts["test"] > 0, "test split has no valid SemanticCNN windows")
    results["window_counts"] = window_counts
    results["empty_range_normalized"] = 1.0
    results["range_clip_max"] = pool_range_max
    results["ignore_label_excluded_from_semantic_pooling"] = True
    return results


def main():
    args = parse_args()
    device = choose_device(args.device)
    report = {
        "status": "PASS",
        "device": str(device),
        "dataset_root": str(args.dataset_root.resolve()),
        "s3net": s3net_smoke(args, device),
        "semantic_cnn": semantic_cnn_smoke(args, device),
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
