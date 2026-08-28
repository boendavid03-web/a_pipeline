#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--acceleration-sigma-mps2, --association-velocity-weight, --checkpoint, --confidence-threshold, --confirmed-timeout-s, --dataset-root, --device, --max-frames, --nms-radius-m, --output-jsonl, --position-gate-m, --position-measurement-scale, --skip-initial-windows-per-episode, --split, --tentative-timeout-s, --topk, --velocity-gate-mps, --velocity-measurement-scale
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON
# 可能使用的关键环境变量：CUDA, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/infer_dataset.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:38:34.277621037 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.377546444 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/infer_dataset.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Run truth-free dataset inference and emit detections plus stable track IDs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from .dataset import BEVSpec, TemporalDualLidarDataset
from .model import TemporalBEVPedestrianDetector, decode_detections
from .tracker import PedestrianTracker, detections_base_to_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "dev", "test"), default="dev")
    parser.add_argument("--output-jsonl", type=Path, required=True)
    parser.add_argument("--confidence-threshold", type=float, default=0.30)
    parser.add_argument("--topk", type=int, default=30)
    parser.add_argument("--nms-radius-m", type=float, default=0.30)
    parser.add_argument("--position-gate-m", type=float, default=0.8)
    parser.add_argument("--velocity-gate-mps", type=float, default=2.5)
    parser.add_argument("--tentative-timeout-s", type=float, default=0.33)
    parser.add_argument("--confirmed-timeout-s", type=float, default=1.0)
    parser.add_argument("--acceleration-sigma-mps2", type=float, default=3.0)
    parser.add_argument("--position-measurement-scale", type=float, default=1.0)
    parser.add_argument("--velocity-measurement-scale", type=float, default=1.0)
    parser.add_argument("--association-velocity-weight", type=float, default=0.15)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument(
        "--skip-initial-windows-per-episode",
        type=int,
        default=0,
        help="Skip complete model windows before starting/resetting the tracker",
    )
    parser.add_argument("--device", default="cuda")
    return parser.parse_args()


def _round_list(values) -> list[float]:
    return [round(float(value), 6) for value in values]


def main() -> int:
    args = parse_args()
    if args.output_jsonl.exists():
        raise FileExistsError(f"refusing to overwrite {args.output_jsonl}")
    if args.skip_initial_windows_per_episode < 0:
        raise ValueError("skip_initial_windows_per_episode cannot be negative")
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    device = torch.device(args.device)
    checkpoint = torch.load(
        args.checkpoint, map_location=device, weights_only=False
    )
    bev_spec = BEVSpec(
        float(checkpoint["bev_extent_m"]),
        float(checkpoint["bev_resolution_m"]),
    )
    dataset = TemporalDualLidarDataset(
        args.dataset_root,
        args.split,
        history_frames=int(checkpoint["history_frames"]),
        bev_spec=bev_spec,
        build_targets=False,
        input_encoding=str(checkpoint.get("input_encoding", "occupancy")),
        max_samples=args.max_frames,
    )
    if dataset.contract_dict()["target_ground_truth_inputs"]:
        raise AssertionError("inference dataset unexpectedly reads truth")
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0)
    model = TemporalBEVPedestrianDetector(
        history_frames=int(checkpoint["history_frames"]),
        base_channels=int(checkpoint["base_channels"]),
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.eval()
    tracker = PedestrianTracker(
        position_gate_m=args.position_gate_m,
        velocity_gate_mps=args.velocity_gate_mps,
        tentative_timeout_s=args.tentative_timeout_s,
        confirmed_timeout_s=args.confirmed_timeout_s,
        acceleration_sigma_mps2=args.acceleration_sigma_mps2,
        position_measurement_scale=args.position_measurement_scale,
        velocity_measurement_scale=args.velocity_measurement_scale,
        association_velocity_weight=args.association_velocity_weight,
    )
    prior_sequence = None
    sequence_window_index = 0
    written_frames = 0
    args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
    with args.output_jsonl.open("x", encoding="utf-8") as output:
        with torch.inference_mode():
            for batch in loader:
                sequence = (
                    str(batch["session_name"][0]),
                    int(batch["episode_id"][0]),
                )
                if sequence != prior_sequence:
                    tracker.reset()
                    prior_sequence = sequence
                    sequence_window_index = 0
                if (
                    sequence_window_index
                    < args.skip_initial_windows_per_episode
                ):
                    sequence_window_index += 1
                    continue
                sequence_window_index += 1
                prediction = model(batch["input"].to(device))
                detections_base = decode_detections(
                    prediction,
                    bev_spec,
                    confidence_threshold=args.confidence_threshold,
                    topk=args.topk,
                    nms_radius_m=args.nms_radius_m,
                )[0]
                pose = batch["robot_pose_map"][0].numpy()
                timestamp_ns = int(batch["timestamp_ns"][0])
                detections_map = detections_base_to_map(detections_base, pose)
                tracks = tracker.update(detections_map, timestamp_ns)
                record = {
                    "session": sequence[0],
                    "episode_id": sequence[1],
                    "name": batch["name"][0],
                    "timestamp_ns": timestamp_ns,
                    "detections": [
                        {
                            "position_xy_base": _round_list(
                                item.position_xy_base
                            ),
                            "velocity_xy_robot_axes_absolute": _round_list(
                                item.velocity_xy_robot_axes_absolute
                            ),
                            "confidence": round(item.confidence, 6),
                        }
                        for item in detections_base
                    ],
                    "tracks": [
                        {
                            "track_id": item.track_id,
                            "position_xy_map": _round_list(item.position_xy_map),
                            "velocity_xy_map_absolute": _round_list(
                                item.velocity_xy_map_absolute
                            ),
                            "confidence": round(item.confidence, 6),
                            "track_state": item.track_state,
                            "time_since_update_s": round(
                                item.time_since_update_s, 6
                            ),
                        }
                        for item in tracks
                    ],
                }
                output.write(json.dumps(record, sort_keys=True) + "\n")
                written_frames += 1
    summary = {
        "status": "PASS",
        "frames": written_frames,
        "dataset_windows": len(dataset),
        "output_jsonl": str(args.output_jsonl),
        "checkpoint": str(args.checkpoint.resolve()),
        "split": args.split,
        "confidence_threshold": args.confidence_threshold,
        "nms_radius_m": args.nms_radius_m,
        "topk": args.topk,
        "skip_initial_windows_per_episode": (
            args.skip_initial_windows_per_episode
        ),
        "tracker": {
            "position_gate_m": args.position_gate_m,
            "velocity_gate_mps": args.velocity_gate_mps,
            "tentative_timeout_s": args.tentative_timeout_s,
            "confirmed_timeout_s": args.confirmed_timeout_s,
            "acceleration_sigma_mps2": args.acceleration_sigma_mps2,
            "position_measurement_scale": args.position_measurement_scale,
            "velocity_measurement_scale": args.velocity_measurement_scale,
            "association_velocity_weight": args.association_velocity_weight,
        },
        "ground_truth_inputs": [],
    }
    summary_path = args.output_jsonl.with_suffix(".summary.json")
    if summary_path.exists():
        raise FileExistsError(f"refusing to overwrite {summary_path}")
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
