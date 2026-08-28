#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--amp, --base-channels, --batch-size, --bev-extent-m, --bev-resolution-m, --dataset-root, --device, --epochs, --history-frames, --init-checkpoint, --input-encoding, --learning-rate, --leg-match-radius-m, --lr-schedule, --max-dev-samples, --max-dev-steps, --max-train-samples, --max-train-steps, --min-person-points, --minimum-learning-rate, --num-workers, --offset-loss-weight, --output-root, --patience, --run-name, --save-every-epochs, --seed, --smoke, --target-sigma-cells, --trainable-components, --velocity-loss-weight, --weight-decay
# 代码中检测到的 ROS 2 话题/路径字符串：/home/user/navigation_project/a_pipeline/runs/
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PT, TXT
# 可能使用的关键环境变量：CUDA, DEFAULT_DATASET_ROOT, DEFAULT_OUTPUT_ROOT, FAIL, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/train.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:38:34.277621037 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.378546463 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/train.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/evaluate.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Train the temporal dual-LiDAR pedestrian position/velocity detector."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import shlex
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict

import numpy as np
import torch
from torch.utils.data import DataLoader

from .dataset import BEVSpec, TemporalDualLidarDataset
from .model import (
    TemporalBEVPedestrianDetector,
    decode_detections,
    detection_loss,
)
from .tracker import PedestrianTracker, detections_base_to_map


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATASET_ROOT = Path(
    PROJECT_ROOT / "runs/"
    "20260717_042135_v7_dual/datasets/"
    "20260727_three_bag_online_seed_split_v1/semantic2d"
)
DEFAULT_OUTPUT_ROOT = Path(
    PROJECT_ROOT / "runs/"
    "20260717_042135_v7_dual/training/"
    "dual_lidar_pedestrian_bev"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train causal dual-LiDAR pedestrian center/velocity detection"
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--history-frames", type=int, default=8)
    parser.add_argument("--bev-extent-m", type=float, default=8.0)
    parser.add_argument("--bev-resolution-m", type=float, default=0.10)
    parser.add_argument("--base-channels", type=int, default=24)
    parser.add_argument("--init-checkpoint", type=Path, default=None)
    parser.add_argument(
        "--trainable-components",
        choices=("all", "velocity_head"),
        default="all",
    )
    parser.add_argument(
        "--input-encoding",
        choices=("occupancy", "current_plus_deltas"),
        default="occupancy",
    )
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument(
        "--lr-schedule",
        choices=("constant", "cosine"),
        default="constant",
    )
    parser.add_argument("--minimum-learning-rate", type=float, default=1e-5)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--offset-loss-weight", type=float, default=1.0)
    parser.add_argument("--velocity-loss-weight", type=float, default=0.5)
    parser.add_argument("--target-sigma-cells", type=float, default=1.5)
    parser.add_argument("--min-person-points", type=int, default=1)
    parser.add_argument("--leg-match-radius-m", type=float, default=0.12)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument(
        "--save-every-epochs",
        type=int,
        default=0,
        help="Also save epoch_XXX.pt every N epochs; zero disables it",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-train-samples", type=int, default=None)
    parser.add_argument("--max-dev-samples", type=int, default=None)
    parser.add_argument("--max-train-steps", type=int, default=None)
    parser.add_argument("--max-dev-steps", type=int, default=None)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--amp", action="store_true")
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Run a bounded real-data optimizer/checkpoint/interface smoke test",
    )
    return parser.parse_args()


def _seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def _json_dump(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _batch_to_device(
    batch: Dict[str, object], device: torch.device
) -> Dict[str, object]:
    result = dict(batch)
    for key in ("input", "heatmap", "offset", "velocity", "regression_mask"):
        result[key] = batch[key].to(device, non_blocking=True)
    return result


def _run_epoch(
    model: TemporalBEVPedestrianDetector,
    loader: DataLoader,
    device: torch.device,
    *,
    optimizer: torch.optim.Optimizer | None,
    amp_enabled: bool,
    max_steps: int | None,
    offset_loss_weight: float,
    velocity_loss_weight: float,
    freeze_batch_norm: bool,
) -> Dict[str, float]:
    training = optimizer is not None
    model.train(training)
    if training and freeze_batch_norm:
        for module in model.modules():
            if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
                module.eval()
    totals = {
        "loss": 0.0,
        "heatmap_loss": 0.0,
        "offset_loss": 0.0,
        "velocity_loss": 0.0,
        "batches": 0,
        "samples": 0,
        "targets": 0,
        "target_collisions": 0,
    }
    context = torch.enable_grad if training else torch.inference_mode
    with context():
        for step, raw_batch in enumerate(loader):
            if max_steps is not None and step >= max_steps:
                break
            batch = _batch_to_device(raw_batch, device)
            if training:
                optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=amp_enabled,
            ):
                outputs = model(batch["input"])
                losses = detection_loss(
                    outputs,
                    batch,
                    offset_weight=offset_loss_weight,
                    velocity_weight=velocity_loss_weight,
                )
            if not torch.isfinite(losses["loss"]):
                raise FloatingPointError(f"non-finite loss at step {step}")
            if training:
                losses["loss"].backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
                optimizer.step()
            batch_size = int(batch["input"].shape[0])
            for key in ("loss", "heatmap_loss", "offset_loss", "velocity_loss"):
                totals[key] += float(losses[key].detach()) * batch_size
            totals["batches"] += 1
            totals["samples"] += batch_size
            totals["targets"] += int(raw_batch["target_count"].sum())
            totals["target_collisions"] += int(
                raw_batch["target_collisions"].sum()
            )
    if totals["samples"] == 0:
        raise RuntimeError("epoch processed no samples")
    samples = totals["samples"]
    return {
        key: float(totals[key] / samples)
        for key in ("loss", "heatmap_loss", "offset_loss", "velocity_loss")
    } | {
        "batches": int(totals["batches"]),
        "samples": int(samples),
        "targets": int(totals["targets"]),
        "target_collisions": int(totals["target_collisions"]),
    }


def _checkpoint_payload(
    model: TemporalBEVPedestrianDetector,
    args: argparse.Namespace,
    bev_spec: BEVSpec,
    epoch: int,
    dev_loss: float,
) -> Dict[str, object]:
    return {
        "schema": "dual-lidar-pedestrian-bev/v1",
        "model_state_dict": model.state_dict(),
        "history_frames": int(args.history_frames),
        "base_channels": int(args.base_channels),
        "bev_extent_m": float(bev_spec.extent_m),
        "bev_resolution_m": float(bev_spec.resolution_m),
        "input_encoding": str(args.input_encoding),
        "epoch": int(epoch),
        "dev_loss": float(dev_loss),
        "output_contract": {
            "position": "position_xy_base_m",
            "velocity": "velocity_xy_robot_axes_absolute_mps",
            "confidence": "0_to_1",
            "track_id": "assigned_by_post_model_kf_hungarian",
        },
    }


def main() -> int:
    args = parse_args()
    if args.offset_loss_weight <= 0.0 or args.velocity_loss_weight <= 0.0:
        raise ValueError("loss weights must be positive")
    if args.minimum_learning_rate < 0.0:
        raise ValueError("minimum_learning_rate cannot be negative")
    if args.target_sigma_cells <= 0.0:
        raise ValueError("target_sigma_cells must be positive")
    if args.save_every_epochs < 0:
        raise ValueError("save_every_epochs cannot be negative")
    if args.smoke:
        args.epochs = 1
        args.batch_size = min(args.batch_size, 2)
        args.num_workers = 0
        args.max_train_samples = 12
        args.max_dev_samples = 6
        args.max_train_steps = 3
        args.max_dev_steps = 2
    _seed_everything(args.seed)
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; pass --device cpu explicitly")
    device = torch.device(args.device)
    amp_enabled = bool(args.amp and device.type == "cuda")
    bev_spec = BEVSpec(args.bev_extent_m, args.bev_resolution_m)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = args.run_name or (
        f"{timestamp}_{'smoke' if args.smoke else 'train'}"
    )
    output_dir = args.output_root.expanduser().resolve() / run_name
    checkpoints_dir = output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "command.txt").write_text(
        shlex.join([sys.executable, "-m", __package__ + ".train", *sys.argv[1:]])
        + "\n",
        encoding="utf-8",
    )

    train_dataset = TemporalDualLidarDataset(
        args.dataset_root,
        "train",
        history_frames=args.history_frames,
        bev_spec=bev_spec,
        build_targets=True,
        min_person_points=args.min_person_points,
        leg_match_radius_m=args.leg_match_radius_m,
        target_sigma_cells=args.target_sigma_cells,
        input_encoding=args.input_encoding,
        max_samples=args.max_train_samples,
    )
    dev_dataset = TemporalDualLidarDataset(
        args.dataset_root,
        "dev",
        history_frames=args.history_frames,
        bev_spec=bev_spec,
        build_targets=True,
        min_person_points=args.min_person_points,
        leg_match_radius_m=args.leg_match_radius_m,
        target_sigma_cells=args.target_sigma_cells,
        input_encoding=args.input_encoding,
        max_samples=args.max_dev_samples,
    )
    generator = torch.Generator().manual_seed(args.seed)
    loader_options = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": device.type == "cuda",
        "persistent_workers": args.num_workers > 0,
    }
    train_loader = DataLoader(
        train_dataset, shuffle=True, generator=generator, **loader_options
    )
    dev_loader = DataLoader(dev_dataset, shuffle=False, **loader_options)
    model = TemporalBEVPedestrianDetector(
        history_frames=args.history_frames,
        base_channels=args.base_channels,
    ).to(device)
    if args.init_checkpoint is not None:
        initial_checkpoint = torch.load(
            args.init_checkpoint, map_location=device, weights_only=False
        )
        expected = {
            "history_frames": args.history_frames,
            "base_channels": args.base_channels,
            "bev_extent_m": args.bev_extent_m,
            "bev_resolution_m": args.bev_resolution_m,
            "input_encoding": args.input_encoding,
        }
        observed = {
            "history_frames": int(initial_checkpoint["history_frames"]),
            "base_channels": int(initial_checkpoint["base_channels"]),
            "bev_extent_m": float(initial_checkpoint["bev_extent_m"]),
            "bev_resolution_m": float(initial_checkpoint["bev_resolution_m"]),
            "input_encoding": str(
                initial_checkpoint.get("input_encoding", "occupancy")
            ),
        }
        if observed != expected:
            raise ValueError(
                f"init checkpoint contract mismatch: expected {expected}, "
                f"observed {observed}"
            )
        model.load_state_dict(
            initial_checkpoint["model_state_dict"], strict=True
        )
    if args.trainable_components == "velocity_head":
        for parameter in model.parameters():
            parameter.requires_grad_(False)
        for parameter in model.velocity_head.parameters():
            parameter.requires_grad_(True)
    trainable_parameters = [
        parameter for parameter in model.parameters() if parameter.requires_grad
    ]
    if not trainable_parameters:
        raise RuntimeError("model has no trainable parameters")
    optimizer = torch.optim.AdamW(
        trainable_parameters,
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    scheduler = (
        torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer,
            T_max=args.epochs,
            eta_min=args.minimum_learning_rate,
        )
        if args.lr_schedule == "cosine"
        else None
    )
    first_parameter_before = (
        trainable_parameters[0].detach().cpu().clone()
    )
    config = {
        "schema": "dual-lidar-pedestrian-bev-training/v1",
        "args": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        "device": str(device),
        "cuda_device": (
            torch.cuda.get_device_name(device) if device.type == "cuda" else None
        ),
        "torch_version": torch.__version__,
        "train_contract": train_dataset.contract_dict(),
        "dev_contract": dev_dataset.contract_dict(),
        "model_parameters": int(sum(p.numel() for p in model.parameters())),
        "trainable_model_parameters": int(
            sum(p.numel() for p in trainable_parameters)
        ),
    }
    _json_dump(output_dir / "config.json", config)

    best_loss = math.inf
    best_velocity_loss = math.inf
    best_epoch = 0
    best_velocity_epoch = 0
    epochs_without_improvement = 0
    metric_rows = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        train_metrics = _run_epoch(
            model,
            train_loader,
            device,
            optimizer=optimizer,
            amp_enabled=amp_enabled,
            max_steps=args.max_train_steps,
            offset_loss_weight=args.offset_loss_weight,
            velocity_loss_weight=args.velocity_loss_weight,
            freeze_batch_norm=args.trainable_components == "velocity_head",
        )
        dev_metrics = _run_epoch(
            model,
            dev_loader,
            device,
            optimizer=None,
            amp_enabled=amp_enabled,
            max_steps=args.max_dev_steps,
            offset_loss_weight=args.offset_loss_weight,
            velocity_loss_weight=args.velocity_loss_weight,
            freeze_batch_norm=False,
        )
        row = {
            "epoch": epoch,
            "learning_rate": float(optimizer.param_groups[0]["lr"]),
            "train": train_metrics,
            "dev": dev_metrics,
        }
        metric_rows.append(row)
        with (output_dir / "metrics.jsonl").open(
            "a", encoding="utf-8"
        ) as handle:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
        print(
            f"epoch={epoch:03d} train={train_metrics['loss']:.5f} "
            f"dev={dev_metrics['loss']:.5f} "
            f"targets={train_metrics['targets']}"
        )
        payload = _checkpoint_payload(
            model, args, bev_spec, epoch, dev_metrics["loss"]
        )
        torch.save(payload, checkpoints_dir / "last.pt")
        if (
            args.save_every_epochs > 0
            and epoch % args.save_every_epochs == 0
        ):
            torch.save(payload, checkpoints_dir / f"epoch_{epoch:03d}.pt")
        if dev_metrics["loss"] < best_loss:
            best_loss = float(dev_metrics["loss"])
            best_epoch = epoch
            epochs_without_improvement = 0
            torch.save(payload, checkpoints_dir / "best.pt")
        else:
            epochs_without_improvement += 1
        if dev_metrics["velocity_loss"] < best_velocity_loss:
            best_velocity_loss = float(dev_metrics["velocity_loss"])
            best_velocity_epoch = epoch
            torch.save(payload, checkpoints_dir / "best_velocity.pt")
        if scheduler is not None:
            scheduler.step()
        if epochs_without_improvement >= args.patience:
            break

    parameter_delta = float(
        torch.linalg.vector_norm(
            trainable_parameters[0].detach().cpu() - first_parameter_before
        )
    )
    checkpoint = torch.load(
        checkpoints_dir / "best.pt",
        map_location=device,
        weights_only=False,
    )
    reloaded = TemporalBEVPedestrianDetector(
        history_frames=int(checkpoint["history_frames"]),
        base_channels=int(checkpoint["base_channels"]),
    ).to(device)
    reloaded.load_state_dict(checkpoint["model_state_dict"], strict=True)
    reloaded.eval()
    smoke_batch = next(iter(dev_loader))
    with torch.inference_mode():
        smoke_outputs = reloaded(smoke_batch["input"].to(device))
    decoded = decode_detections(
        smoke_outputs,
        bev_spec,
        confidence_threshold=0.01 if args.smoke else 0.30,
        topk=5,
    )
    tracker = PedestrianTracker()
    first_detections_map = detections_base_to_map(
        decoded[0], smoke_batch["robot_pose_map"][0].numpy()
    )
    tracked = tracker.update(
        first_detections_map, int(smoke_batch["timestamp_ns"][0])
    )
    elapsed = time.perf_counter() - started
    finite_outputs = all(
        bool(torch.isfinite(value).all()) for value in smoke_outputs.values()
    )
    interface_fields = (
        sorted(
            [
                "position_xy_base",
                "velocity_xy_robot_axes_absolute",
                "confidence",
            ]
        )
        if decoded[0]
        else []
    )
    pipeline_pass = parameter_delta > 0.0 and finite_outputs
    if args.smoke:
        pipeline_pass = pipeline_pass and bool(decoded[0]) and bool(tracked)
    summary = {
        "status": "PASS" if pipeline_pass else "FAIL",
        "output_dir": str(output_dir),
        "best_checkpoint": str(checkpoints_dir / "best.pt"),
        "best_epoch": best_epoch,
        "best_dev_loss": best_loss,
        "best_velocity_checkpoint": str(
            checkpoints_dir / "best_velocity.pt"
        ),
        "best_velocity_epoch": best_velocity_epoch,
        "best_dev_velocity_loss": best_velocity_loss,
        "epochs_completed": len(metric_rows),
        "elapsed_seconds": elapsed,
        "parameter_delta_l2": parameter_delta,
        "checkpoint_strict_reload": True,
        "finite_outputs": finite_outputs,
        "interface_probe_detection_count": len(decoded[0]),
        "interface_probe_track_count": len(tracked),
        "interface_probe_note": (
            "The single probe batch is an interface check, not an accuracy "
            "criterion. Use evaluate.py for split-level detection metrics."
        ),
        "decoded_output_fields": interface_fields,
        "tracked_output_fields": [
            "track_id",
            "position_xy_map",
            "velocity_xy_map_absolute",
            "confidence",
            "track_state",
        ],
        "test_split_used": False,
    }
    _json_dump(output_dir / "training_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
