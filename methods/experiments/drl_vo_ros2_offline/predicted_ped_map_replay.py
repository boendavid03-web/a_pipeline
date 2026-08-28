#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--cell-match-gate-m, --checkpoint, --coasting-max-time-s, --confidence-threshold, --dataset-root, --device, --fixed-session-root, --frames, --high-risk-distance-m, --include-tentative, --max-track-age-s, --nms-radius-m, --output-dir, --policy, --policy-batch-size, --position-match-gate-m, --split, --start-index, --topk
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, NPZ, PNG, TXT
# 可能使用的关键环境变量：CONFIRMED, CUDA, FAIL, OBSERVATION_SIZE, PASS, PED_MAP_SHAPE, PROJECT_ROOT, VARIANTS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/predicted_ped_map_replay.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 01:11:50.369451747 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.813228349 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/github_src/drl_vo_nav-drl_vo/drl_vo/src/drl_vo_inference.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/predicted_ped_map_replay.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/github_src/drl_vo_nav-drl_vo/drl_vo/src/drl_vo_inference.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Truth-isolated pedestrian inference and DRL-VO A/B/C/D shadow replay."""

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
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from methods.experiments.drl_vo_ros2_offline.drlvo_model import (
    load_policy_strict,
)
from methods.experiments.drl_vo_ros2_offline.observation_adapter import (
    OBSERVATION_SIZE,
    PED_MAP_SHAPE,
    ObservationAdapter,
    observation_with_pedestrian_map,
    rotate_map_to_base,
    tracks_to_drl_vo_ped_map_with_diagnostics,
)
from methods.experiments.dual_lidar_pedestrian_bev.dataset import (
    BEVSpec,
    TemporalDualLidarDataset,
)
from methods.experiments.dual_lidar_pedestrian_bev.model import (
    TemporalBEVPedestrianDetector,
    decode_detections,
)
from methods.experiments.dual_lidar_pedestrian_bev.tracker import (
    PedestrianTracker,
    detections_base_to_map,
    linear_sum_assignment,
)


VARIANTS = ("oracle", "predicted", "oracle_8m", "zero")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run truth-free dual-lidar inference, generate legacy DRL-VO "
            "pedestrian maps, and compare policy actions."
        )
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--fixed-session-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "dev", "test"), default="dev")
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--confidence-threshold", type=float, default=0.4)
    parser.add_argument("--topk", type=int, default=30)
    parser.add_argument("--nms-radius-m", type=float, default=0.30)
    parser.add_argument("--coasting-max-time-s", type=float, default=0.5)
    parser.add_argument("--max-track-age-s", type=float, default=1.0)
    parser.add_argument("--include-tentative", action="store_true")
    parser.add_argument("--position-match-gate-m", type=float, default=0.5)
    parser.add_argument("--cell-match-gate-m", type=float, default=0.5)
    parser.add_argument("--high-risk-distance-m", type=float, default=2.0)
    parser.add_argument("--policy-batch-size", type=int, default=32)
    parser.add_argument(
        "--device",
        default="auto",
        help="auto, cpu, or a torch device such as cuda",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_stats(values: Sequence[float]) -> dict[str, float | None]:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {
            "min": None,
            "max": None,
            "mean": None,
            "p95": None,
        }
    return {
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
        "p95": float(np.percentile(finite, 95)),
    }


def _resolve_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device(requested)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    return device


def _synchronize(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def _round_list(values: np.ndarray) -> list[float]:
    return [round(float(value), 6) for value in values]


def _serialize_detection(item: object) -> dict[str, object]:
    return {
        "position_xy_base": _round_list(
            np.asarray(getattr(item, "position_xy_base"))
        ),
        "velocity_xy_robot_axes_absolute": _round_list(
            np.asarray(getattr(item, "velocity_xy_robot_axes_absolute"))
        ),
        "confidence": round(float(getattr(item, "confidence")), 6),
    }


def _serialize_track(item: object) -> dict[str, object]:
    return {
        "track_id": int(getattr(item, "track_id")),
        "position_xy_map": _round_list(
            np.asarray(getattr(item, "position_xy_map"))
        ),
        "velocity_xy_map_absolute": _round_list(
            np.asarray(getattr(item, "velocity_xy_map_absolute"))
        ),
        "confidence": round(float(getattr(item, "confidence")), 6),
        "track_state": str(getattr(item, "track_state")),
        "time_since_update_s": round(
            float(getattr(item, "time_since_update_s")), 6
        ),
    }


def build_fixed_sample_lookup(
    dataset_root: Path,
    fixed_session_root: Path,
    split: str,
) -> dict[tuple[str, str], Path]:
    lookup: dict[tuple[str, str], Path] = {}
    fixed_root = fixed_session_root.expanduser().resolve()
    for metadata_path in sorted(dataset_root.glob("*/metadata.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("split_role") != split:
            continue
        session_name = str(metadata["session_name"])
        source_session = Path(str(metadata["source_npz_session"])).resolve()
        if not source_session.is_relative_to(fixed_root):
            raise ValueError(
                f"source session is outside fixed root: {source_session}"
            )
        for frame in metadata["frames"]:
            key = (session_name, str(frame["name"]))
            path = source_session / "samples" / str(frame["source_npz"])
            if key in lookup:
                raise ValueError(f"duplicate semantic frame key: {key}")
            if not path.is_file():
                raise FileNotFoundError(path)
            lookup[key] = path
    if not lookup:
        raise ValueError(f"no {split} frame mapping found under {dataset_root}")
    return lookup


def load_oracle_tracks_for_evaluation(
    sample_path: Path,
    timestamp_ns: int,
    robot_pose_map: np.ndarray,
) -> tuple[list[dict[str, object]], list[dict[str, object]], float]:
    """Load truth only for oracle maps and metrics, after model inference."""

    with np.load(sample_path, allow_pickle=False) as sample:
        positions = sample["pedestrian_xy_map"].astype(np.float64)
        velocities = sample["pedestrian_velocity_map"].astype(np.float64)
        ids = sample["pedestrian_ids"].astype(np.str_)
        truth_timestamp_ns = int(sample["pedestrian_truth_stamp_ns"])
    if (
        positions.ndim != 2
        or positions.shape[1:] != (2,)
        or velocities.shape != positions.shape
        or ids.shape != (len(positions),)
        or not np.isfinite(positions).all()
        or not np.isfinite(velocities).all()
    ):
        raise ValueError(f"invalid oracle arrays in {sample_path}")
    dt_s = (int(timestamp_ns) - truth_timestamp_ns) / 1e9
    positions = positions + velocities * dt_s
    order = np.argsort(ids, kind="stable")
    pose = np.asarray(robot_pose_map, dtype=np.float64)
    positions_base = rotate_map_to_base(
        positions - pose[:2], float(pose[2])
    )
    tracks = []
    tracks_8m = []
    for track_id, source_index in enumerate(order.tolist(), start=1):
        track = {
            "track_id": track_id,
            "source_pedestrian_id": str(ids[source_index]),
            "position_xy_map": positions[source_index],
            "velocity_xy_map_absolute": velocities[source_index],
            "confidence": 1.0,
            "track_state": "CONFIRMED",
            "time_since_update_s": 0.0,
        }
        tracks.append(track)
        if float(np.linalg.norm(positions_base[source_index])) <= 8.0:
            tracks_8m.append(track)
    nearest = (
        float(np.min(np.linalg.norm(positions_base, axis=1)))
        if len(positions_base)
        else math.nan
    )
    return tracks, tracks_8m, nearest


def infer_tracks_without_ground_truth(
    sample: dict[str, object],
    model: TemporalBEVPedestrianDetector,
    bev_spec: BEVSpec,
    tracker: PedestrianTracker,
    device: torch.device,
    *,
    confidence_threshold: float,
    topk: int,
    nms_radius_m: float,
) -> tuple[list[object], list[object], dict[str, float]]:
    """Model and tracker path. This function never receives an oracle field."""

    inputs = sample["input"]
    if not isinstance(inputs, torch.Tensor) or not torch.isfinite(inputs).all():
        raise ValueError("perception input must be a finite tensor")
    started = time.perf_counter()
    model_started = time.perf_counter()
    with torch.inference_mode():
        outputs = model(inputs.unsqueeze(0).to(device))
    _synchronize(device)
    model_ms = (time.perf_counter() - model_started) * 1000.0
    if any(not torch.isfinite(value).all() for value in outputs.values()):
        raise ValueError("perception model produced NaN or Inf")

    postprocess_started = time.perf_counter()
    detections_base = decode_detections(
        outputs,
        bev_spec,
        confidence_threshold=confidence_threshold,
        topk=topk,
        nms_radius_m=nms_radius_m,
    )[0]
    pose = np.asarray(sample["robot_pose_map"], dtype=np.float64)
    detections_map = detections_base_to_map(detections_base, pose)
    postprocess_ms = (time.perf_counter() - postprocess_started) * 1000.0

    tracker_started = time.perf_counter()
    tracks = tracker.update(detections_map, int(sample["timestamp_ns"]))
    tracker_ms = (time.perf_counter() - tracker_started) * 1000.0
    return detections_base, tracks, {
        "inference_ms": model_ms,
        "postprocess_ms": postprocess_ms,
        "tracker_ms": tracker_ms,
        "end_to_end_ms": (time.perf_counter() - started) * 1000.0,
    }


def _tracks_in_robot_axes(
    tracks: Sequence[object],
    written_track_ids: Sequence[int],
    robot_pose_map: np.ndarray,
) -> list[dict[str, object]]:
    selected = set(map(int, written_track_ids))
    pose = np.asarray(robot_pose_map, dtype=np.float64)
    states = []
    for track in tracks:
        track_id = int(
            track["track_id"] if isinstance(track, dict) else track.track_id
        )
        if track_id not in selected:
            continue
        position_map = np.asarray(
            track["position_xy_map"]
            if isinstance(track, dict)
            else track.position_xy_map,
            dtype=np.float64,
        )
        velocity_map = np.asarray(
            track["velocity_xy_map_absolute"]
            if isinstance(track, dict)
            else track.velocity_xy_map_absolute,
            dtype=np.float64,
        )
        states.append(
            {
                "track_id": track_id,
                "position": rotate_map_to_base(
                    (position_map - pose[:2]).reshape(1, 2), float(pose[2])
                )[0],
                "velocity": rotate_map_to_base(
                    velocity_map.reshape(1, 2), float(pose[2])
                )[0],
            }
        )
    states.sort(key=lambda item: int(item["track_id"]))
    return states


def _match_states(
    predicted: Sequence[dict[str, object]],
    target: Sequence[dict[str, object]],
    gate_m: float,
) -> dict[str, object]:
    if not predicted or not target:
        return {
            "tp": 0,
            "fp": len(predicted),
            "fn": len(target),
            "position_errors_m": [],
            "velocity_errors_mps": [],
            "matches": [],
        }
    predicted_positions = np.stack([item["position"] for item in predicted])
    target_positions = np.stack([item["position"] for item in target])
    distances = np.linalg.norm(
        predicted_positions[:, None, :] - target_positions[None, :, :],
        axis=2,
    )
    invalid = 1e9
    costs = np.where(distances <= gate_m, distances, invalid)
    rows, cols = linear_sum_assignment(costs)
    matches = []
    position_errors = []
    velocity_errors = []
    for row, col in zip(rows.tolist(), cols.tolist()):
        if costs[row, col] >= invalid:
            continue
        position_error = float(distances[row, col])
        velocity_error = float(
            np.linalg.norm(
                np.asarray(predicted[row]["velocity"])
                - np.asarray(target[col]["velocity"])
            )
        )
        position_errors.append(position_error)
        velocity_errors.append(velocity_error)
        matches.append(
            {
                "predicted_track_id": int(predicted[row]["track_id"]),
                "target_track_id": int(target[col]["track_id"]),
                "position_error_m": position_error,
                "velocity_error_mps": velocity_error,
            }
        )
    tp = len(matches)
    return {
        "tp": tp,
        "fp": len(predicted) - tp,
        "fn": len(target) - tp,
        "position_errors_m": position_errors,
        "velocity_errors_mps": velocity_errors,
        "matches": matches,
    }


def _written_cell_states(
    diagnostics: dict[str, object],
) -> list[dict[str, object]]:
    return [
        {
            "track_id": int(item["track_id"]),
            "position": np.asarray(
                [
                    (float(item["row"]) + 0.5) * 0.25,
                    10.0 - (float(item["col"]) + 0.5) * 0.25,
                ],
                dtype=np.float64,
            ),
            "velocity": np.zeros(2, dtype=np.float64),
        }
        for item in diagnostics["written_cells"]
    ]


def _safe_ratio(numerator: int, denominator: int) -> float | None:
    return float(numerator / denominator) if denominator else None


def _frame_match_metrics(
    predicted_tracks: Sequence[object],
    predicted_diagnostics: dict[str, object],
    target_tracks: Sequence[object],
    target_diagnostics: dict[str, object],
    robot_pose_map: np.ndarray,
    *,
    position_gate_m: float,
    cell_gate_m: float,
) -> dict[str, object]:
    predicted_states = _tracks_in_robot_axes(
        predicted_tracks,
        predicted_diagnostics["written_track_ids"],
        robot_pose_map,
    )
    target_states = _tracks_in_robot_axes(
        target_tracks,
        target_diagnostics["written_track_ids"],
        robot_pose_map,
    )
    state_match = _match_states(
        predicted_states, target_states, position_gate_m
    )
    cell_match = _match_states(
        _written_cell_states(predicted_diagnostics),
        _written_cell_states(target_diagnostics),
        cell_gate_m,
    )
    return {
        "state_tp": state_match["tp"],
        "state_fp": state_match["fp"],
        "state_fn": state_match["fn"],
        "position_errors_m": state_match["position_errors_m"],
        "velocity_errors_mps": state_match["velocity_errors_mps"],
        "matches": state_match["matches"],
        "cell_tp": cell_match["tp"],
        "cell_fp": cell_match["fp"],
        "cell_fn": cell_match["fn"],
        "cell_precision": _safe_ratio(
            int(cell_match["tp"]),
            int(cell_match["tp"]) + int(cell_match["fp"]),
        ),
        "cell_recall": _safe_ratio(
            int(cell_match["tp"]),
            int(cell_match["tp"]) + int(cell_match["fn"]),
        ),
    }


def _map_stats(pedestrian_map: np.ndarray, written_count: int) -> dict[str, object]:
    nonzero_cells = int(
        np.count_nonzero(np.any(np.abs(pedestrian_map) > 0.0, axis=0))
    )
    return {
        "written_cell_count": int(written_count),
        "nonzero_cell_count": nonzero_cells,
        "static_or_zero_velocity_written_cells": int(
            written_count - nonzero_cells
        ),
        "max_absolute_velocity_mps": float(np.max(np.abs(pedestrian_map))),
    }


def _aggregate_match_metrics(
    frames: Sequence[dict[str, object]],
    key: str,
) -> dict[str, object]:
    metrics = [frame[key] for frame in frames]
    state_tp = sum(int(item["state_tp"]) for item in metrics)
    state_fp = sum(int(item["state_fp"]) for item in metrics)
    state_fn = sum(int(item["state_fn"]) for item in metrics)
    cell_tp = sum(int(item["cell_tp"]) for item in metrics)
    cell_fp = sum(int(item["cell_fp"]) for item in metrics)
    cell_fn = sum(int(item["cell_fn"]) for item in metrics)
    position_errors = [
        value for item in metrics for value in item["position_errors_m"]
    ]
    velocity_errors = [
        value for item in metrics for value in item["velocity_errors_mps"]
    ]
    return {
        "state_matches": {
            "tp": state_tp,
            "fp": state_fp,
            "fn": state_fn,
            "precision": _safe_ratio(state_tp, state_tp + state_fp),
            "recall": _safe_ratio(state_tp, state_tp + state_fn),
            "position_error_m": finite_stats(position_errors),
            "velocity_error_mps": finite_stats(velocity_errors),
        },
        "cell_spatial_matches": {
            "tp": cell_tp,
            "fp": cell_fp,
            "fn": cell_fn,
            "precision": _safe_ratio(cell_tp, cell_tp + cell_fp),
            "recall": _safe_ratio(cell_tp, cell_tp + cell_fn),
        },
    }


def _turn_direction(angular_velocity: np.ndarray) -> np.ndarray:
    result = np.zeros_like(angular_velocity, dtype=np.int8)
    result[angular_velocity > 0.05] = 1
    result[angular_velocity < -0.05] = -1
    return result


def build_policy_comparison(
    normalized_actions: np.ndarray,
    physical_actions: np.ndarray,
    high_risk_mask: np.ndarray,
) -> dict[str, object]:
    variants = {}
    for index, name in enumerate(VARIANTS):
        variants[name] = {
            "linear_velocity_mps": finite_stats(
                physical_actions[:, index, 0]
            ),
            "angular_velocity_radps": finite_stats(
                physical_actions[:, index, 1]
            ),
            "linear_saturation_fraction": float(
                np.mean(np.abs(normalized_actions[:, index, 0]) >= 0.999)
            ),
            "angular_saturation_fraction": float(
                np.mean(np.abs(normalized_actions[:, index, 1]) >= 0.999)
            ),
        }
    comparisons = {}
    oracle = physical_actions[:, 0]
    oracle_stop = oracle[:, 0] <= 0.05
    oracle_turn = _turn_direction(oracle[:, 1])
    for index, name in enumerate(VARIANTS[1:], start=1):
        candidate = physical_actions[:, index]
        differences = np.linalg.norm(candidate - oracle, axis=1)
        high_risk_differences = differences[high_risk_mask]
        comparisons[f"{name}_vs_oracle"] = {
            "action_l2": finite_stats(differences),
            "stop_move_flip_count": int(
                np.count_nonzero((candidate[:, 0] <= 0.05) != oracle_stop)
            ),
            "turn_direction_flip_count": int(
                np.count_nonzero(
                    _turn_direction(candidate[:, 1]) != oracle_turn
                )
            ),
            "high_risk_frame_count": int(np.count_nonzero(high_risk_mask)),
            "high_risk_action_l2": finite_stats(high_risk_differences),
        }
    return {
        "schema": "drl-vo-ped-map-policy-comparison/v1",
        "variant_order": list(VARIANTS),
        "variants": variants,
        "comparisons": comparisons,
        "stop_threshold_linear_mps": 0.05,
        "turn_deadband_radps": 0.05,
    }


def save_policy_csv(
    path: Path,
    frame_metrics: Sequence[dict[str, object]],
    normalized_actions: np.ndarray,
    physical_actions: np.ndarray,
) -> None:
    fields = ["frame", "timestamp_ns"]
    for name in VARIANTS:
        fields.extend(
            [
                f"{name}_normalized_0",
                f"{name}_normalized_1",
                f"{name}_linear_mps",
                f"{name}_angular_radps",
            ]
        )
    fields.extend(
        [
            "predicted_vs_oracle_action_l2",
            "oracle_8m_vs_oracle_action_l2",
            "zero_vs_oracle_action_l2",
        ]
    )
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame_index, frame in enumerate(frame_metrics):
            row: dict[str, object] = {
                "frame": frame_index,
                "timestamp_ns": frame["timestamp_ns"],
            }
            for variant_index, name in enumerate(VARIANTS):
                row.update(
                    {
                        f"{name}_normalized_0": float(
                            normalized_actions[frame_index, variant_index, 0]
                        ),
                        f"{name}_normalized_1": float(
                            normalized_actions[frame_index, variant_index, 1]
                        ),
                        f"{name}_linear_mps": float(
                            physical_actions[frame_index, variant_index, 0]
                        ),
                        f"{name}_angular_radps": float(
                            physical_actions[frame_index, variant_index, 1]
                        ),
                    }
                )
            for variant_index, name in enumerate(VARIANTS[1:], start=1):
                row[f"{name}_vs_oracle_action_l2"] = float(
                    np.linalg.norm(
                        physical_actions[frame_index, variant_index]
                        - physical_actions[frame_index, 0]
                    )
                )
            writer.writerow(row)


def save_overview_visualization(
    path: Path,
    pedestrian_maps: np.ndarray,
    physical_actions: np.ndarray,
) -> int:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    differences = np.linalg.norm(
        physical_actions[:, 1] - physical_actions[:, 0], axis=1
    )
    frame_index = int(np.argmax(differences))
    oracle_magnitude = np.linalg.norm(
        pedestrian_maps[frame_index, 0], axis=0
    )
    predicted_magnitude = np.linalg.norm(
        pedestrian_maps[frame_index, 1], axis=0
    )
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    maximum = max(
        0.1,
        float(np.max(oracle_magnitude)),
        float(np.max(predicted_magnitude)),
    )
    for axis, image, title in (
        (axes[0], oracle_magnitude, "A: GT/oracle speed"),
        (axes[1], predicted_magnitude, "B: predicted speed"),
    ):
        view = axis.imshow(
            image.T,
            origin="upper",
            extent=(0.0, 20.0, -10.0, 10.0),
            aspect="auto",
            vmin=0.0,
            vmax=maximum,
            cmap="viridis",
        )
        axis.set_xlabel("x_base [m]")
        axis.set_ylabel("y_base [m]")
        axis.set_title(title)
        figure.colorbar(view, ax=axis, label="speed [m/s]")
    x = np.arange(len(VARIANTS))
    axes[2].bar(
        x - 0.18,
        physical_actions[frame_index, :, 0],
        width=0.36,
        label="linear [m/s]",
    )
    axes[2].bar(
        x + 0.18,
        physical_actions[frame_index, :, 1],
        width=0.36,
        label="angular [rad/s]",
    )
    axes[2].set_xticks(x, VARIANTS, rotation=20)
    axes[2].set_title(
        "DRL actions\n"
        f"|B-A|={differences[frame_index]:.4f}, frame={frame_index}"
    )
    axes[2].legend()
    axes[2].grid(axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return frame_index


def _policy_candidates(policy_dir: Path) -> list[dict[str, object]]:
    candidates = []
    for path in sorted(policy_dir.glob("*.zip")):
        candidates.append(
            {
                "path": str(path.resolve()),
                "sha256": sha256_file(path),
                "input_contract": [OBSERVATION_SIZE],
                "pedestrian_prefix_shape": list(PED_MAP_SHAPE),
            }
        )
    return candidates


def main() -> int:
    args = parse_args()
    if args.frames < 1 or args.start_index < 0:
        raise ValueError("--frames must be positive and --start-index nonnegative")
    if args.policy_batch_size < 1:
        raise ValueError("--policy-batch-size must be positive")
    if args.output_dir.exists():
        raise FileExistsError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    visualizations_dir = args.output_dir / "visualizations"
    visualizations_dir.mkdir()

    device = _resolve_device(args.device)
    torch.use_deterministic_algorithms(True)
    if torch.backends.cudnn.is_available():
        torch.backends.cudnn.benchmark = False
    checkpoint = torch.load(
        args.checkpoint, map_location=device, weights_only=False
    )
    required_checkpoint_fields = {
        "model_state_dict",
        "history_frames",
        "base_channels",
        "bev_extent_m",
        "bev_resolution_m",
    }
    missing = required_checkpoint_fields - set(checkpoint)
    if missing:
        raise KeyError(f"checkpoint fields missing: {sorted(missing)}")
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
    )
    contract = dataset.contract_dict()
    if contract["inference_ground_truth_inputs"] or contract[
        "target_ground_truth_inputs"
    ]:
        raise AssertionError("perception inference dataset reads ground truth")
    end_index = min(len(dataset), args.start_index + args.frames)
    indices = list(range(args.start_index, end_index))
    if len(indices) != args.frames:
        raise ValueError(
            f"requested {args.frames} frames from {args.start_index}, "
            f"but dataset contains {len(dataset)} windows"
        )
    sample_lookup = build_fixed_sample_lookup(
        args.dataset_root, args.fixed_session_root, args.split
    )

    model = TemporalBEVPedestrianDetector(
        history_frames=int(checkpoint["history_frames"]),
        base_channels=int(checkpoint["base_channels"]),
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.eval()
    policy, policy_weight_items = load_policy_strict(args.policy)
    policy = policy.to(device)
    policy.eval()
    tracker = PedestrianTracker()
    observation_adapter = ObservationAdapter(include_semantics=False)

    config = {
        "schema": "predicted-ped-map-shadow-replay/v1",
        "args": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        "resolved": {
            "dataset_root": str(args.dataset_root.resolve()),
            "fixed_session_root": str(args.fixed_session_root.resolve()),
            "checkpoint": str(args.checkpoint.resolve()),
            "checkpoint_sha256": sha256_file(args.checkpoint),
            "policy": str(args.policy.resolve()),
            "policy_sha256": sha256_file(args.policy),
            "device": str(device),
        },
        "checkpoint_contract": {
            "schema": checkpoint.get("schema"),
            "history_frames": int(checkpoint["history_frames"]),
            "input_shape": contract["input_shape"],
            "bev_extent_m": bev_spec.extent_m,
            "bev_resolution_m": bev_spec.resolution_m,
            "output_contract": checkpoint.get("output_contract"),
        },
        "perception_input_contract": contract,
        "tracker": {
            "position_gate_m": tracker.position_gate_m,
            "velocity_gate_mps": tracker.velocity_gate_mps,
            "tentative_timeout_s": tracker.tentative_timeout_s,
            "confirmed_timeout_s": tracker.confirmed_timeout_s,
            "acceleration_sigma_mps2": tracker.acceleration_sigma_mps2,
            "position_measurement_scale": tracker.position_measurement_scale,
            "velocity_measurement_scale": tracker.velocity_measurement_scale,
            "association_velocity_weight": (
                tracker.association_velocity_weight
            ),
        },
        "drl_vo_contract": {
            "pedestrian_map_shape": list(PED_MAP_SHAPE),
            "channels": [
                "vx_base_absolute",
                "vy_base_absolute",
            ],
            "velocity_transform": (
                "rotate map absolute velocity to robot axes; "
                "do not subtract robot velocity"
            ),
            "fov_m": {"x": [0.0, 20.0], "y": [-10.0, 10.0]},
            "resolution_m": 0.25,
            "normalization": "clip(map_mps / 2.0, -1, 1)",
            "flatten_order": "C",
            "observation_shape": [OBSERVATION_SIZE],
        },
        "policy_selection": {
            "selected_reason": (
                "drl_vo.zip is the default in drl_vo_inference.py and "
                "drl_vo_inference.launch; semantic_no_ped checkpoints are excluded"
            ),
            "weight_items": policy_weight_items,
            "candidates": _policy_candidates(args.policy.parent),
        },
        "range_ablation": {
            "oracle_8m_definition": "Euclidean base-frame distance <= 8.0 m",
            "no_rescaling": True,
        },
    }
    (args.output_dir / "config.json").write_text(
        json.dumps(config, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    all_maps = []
    all_observations = []
    frame_metrics: list[dict[str, object]] = []
    timestamps = []
    sequence_resets = 0
    perception_errors = 0
    previous_sequence: tuple[str, int] | None = None
    timing_values: dict[str, list[float]] = {
        "inference_ms": [],
        "postprocess_ms": [],
        "tracker_ms": [],
        "end_to_end_ms": [],
    }

    tracks_path = args.output_dir / "predicted_tracks.jsonl"
    with tracks_path.open("x", encoding="utf-8") as tracks_stream:
        for frame_number, dataset_index in enumerate(indices):
            sample = dataset[dataset_index]
            session = str(sample["session_name"])
            episode_id = int(sample["episode_id"])
            sequence = (session, episode_id)
            if sequence != previous_sequence:
                tracker.reset()
                sequence_resets += 1
                previous_sequence = sequence
            fixed_path = sample_lookup[(session, str(sample["name"]))]
            pose = np.asarray(sample["robot_pose_map"], dtype=np.float64)
            timestamp_ns = int(sample["timestamp_ns"])
            timestamps.append(timestamp_ns)

            sensor_frame = observation_adapter.adapt_with_pedestrian_map(
                fixed_path,
                np.zeros(PED_MAP_SHAPE, dtype=np.float32),
                sequence_id=session,
            )
            if sensor_frame.timestamp_ns != timestamp_ns:
                raise ValueError(
                    f"timestamp mismatch for {sample['name']}: "
                    f"{sensor_frame.timestamp_ns} != {timestamp_ns}"
                )

            detections: list[object] = []
            tracks: list[object] = []
            perception_error: str | None = None
            timing = {
                "inference_ms": 0.0,
                "postprocess_ms": 0.0,
                "tracker_ms": 0.0,
                "end_to_end_ms": 0.0,
            }
            perception_pipeline_started = time.perf_counter()
            try:
                detections, tracks, timing = infer_tracks_without_ground_truth(
                    sample,
                    model,
                    bev_spec,
                    tracker,
                    device,
                    confidence_threshold=args.confidence_threshold,
                    topk=args.topk,
                    nms_radius_m=args.nms_radius_m,
                )
                predicted_map, predicted_diagnostics = (
                    tracks_to_drl_vo_ped_map_with_diagnostics(
                        tracks,
                        pose,
                        coasting_max_time_s=args.coasting_max_time_s,
                        max_track_age_s=args.max_track_age_s,
                        include_tentative=args.include_tentative,
                    )
                )
            except Exception as error:
                perception_errors += 1
                perception_error = f"{type(error).__name__}: {error}"
                tracker.reset()
                predicted_map = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
                predicted_diagnostics = {
                    "written_track_ids": [],
                    "written_cells": [],
                    "excluded_tracks": [
                        {
                            "track_id": None,
                            "reason": "perception_exception",
                            "detail": perception_error,
                        }
                    ],
                    "same_cell_conflict_count": 0,
                    "dropped_track_count": 0,
                }
            timing["end_to_end_ms"] = (
                time.perf_counter() - perception_pipeline_started
            ) * 1000.0
            for key, value in timing.items():
                timing_values[key].append(float(value))

            oracle_tracks, oracle_8m_tracks, nearest_oracle = (
                load_oracle_tracks_for_evaluation(
                    fixed_path, timestamp_ns, pose
                )
            )
            oracle_map, oracle_diagnostics = (
                tracks_to_drl_vo_ped_map_with_diagnostics(
                    oracle_tracks, pose
                )
            )
            oracle_8m_map, oracle_8m_diagnostics = (
                tracks_to_drl_vo_ped_map_with_diagnostics(
                    oracle_8m_tracks, pose
                )
            )
            zero_map = np.zeros(PED_MAP_SHAPE, dtype=np.float32)
            maps = np.stack(
                (oracle_map, predicted_map, oracle_8m_map, zero_map)
            )
            observations = np.stack(
                [
                    observation_with_pedestrian_map(
                        sensor_frame.observation, pedestrian_map
                    )
                    for pedestrian_map in maps
                ]
            )
            if not np.isfinite(maps).all() or not np.isfinite(observations).all():
                raise RuntimeError("non-finite map or observation")
            all_maps.append(maps)
            all_observations.append(observations)

            predicted_vs_oracle = _frame_match_metrics(
                tracks,
                predicted_diagnostics,
                oracle_tracks,
                oracle_diagnostics,
                pose,
                position_gate_m=args.position_match_gate_m,
                cell_gate_m=args.cell_match_gate_m,
            )
            predicted_vs_oracle_8m = _frame_match_metrics(
                tracks,
                predicted_diagnostics,
                oracle_8m_tracks,
                oracle_8m_diagnostics,
                pose,
                position_gate_m=args.position_match_gate_m,
                cell_gate_m=args.cell_match_gate_m,
            )
            frame_metric = {
                "frame": frame_number,
                "dataset_index": dataset_index,
                "session": session,
                "episode_id": episode_id,
                "name": str(sample["name"]),
                "fixed_sample": str(fixed_path),
                "timestamp_ns": timestamp_ns,
                "robot_pose_map": _round_list(pose),
                "input_tracks": [_serialize_track(item) for item in tracks],
                "predicted_map_diagnostics": predicted_diagnostics,
                "oracle_map_diagnostics": oracle_diagnostics,
                "oracle_8m_map_diagnostics": oracle_8m_diagnostics,
                "map_stats": {
                    "oracle": _map_stats(
                        oracle_map,
                        len(oracle_diagnostics["written_track_ids"]),
                    ),
                    "predicted": _map_stats(
                        predicted_map,
                        len(predicted_diagnostics["written_track_ids"]),
                    ),
                    "oracle_8m": _map_stats(
                        oracle_8m_map,
                        len(oracle_8m_diagnostics["written_track_ids"]),
                    ),
                    "zero": _map_stats(zero_map, 0),
                },
                "predicted_vs_oracle": predicted_vs_oracle,
                "predicted_vs_oracle_8m": predicted_vs_oracle_8m,
                "oracle_nearest_distance_m": nearest_oracle,
                "high_risk_close_scene": bool(
                    math.isfinite(nearest_oracle)
                    and nearest_oracle <= args.high_risk_distance_m
                ),
                "timing": timing,
                "perception_error": perception_error,
            }
            frame_metrics.append(frame_metric)
            tracks_record = {
                "frame": frame_number,
                "session": session,
                "episode_id": episode_id,
                "name": str(sample["name"]),
                "timestamp_ns": timestamp_ns,
                "detections": [
                    _serialize_detection(item) for item in detections
                ],
                "tracks": [_serialize_track(item) for item in tracks],
                "perception_error": perception_error,
            }
            tracks_stream.write(
                json.dumps(
                    tracks_record,
                    sort_keys=True,
                    allow_nan=False,
                )
                + "\n"
            )

    pedestrian_maps = np.stack(all_maps).astype(np.float32)
    observation_array = np.stack(all_observations).astype(np.float32)
    flattened = observation_array.reshape(-1, OBSERVATION_SIZE)
    raw_action_chunks = []
    policy_started = time.perf_counter()
    with torch.inference_mode():
        for start in range(0, len(flattened), args.policy_batch_size):
            batch = torch.from_numpy(
                flattened[start : start + args.policy_batch_size]
            ).to(device)
            raw_action, _value = policy(batch)
            raw_action_chunks.append(raw_action.detach().cpu().numpy())
    _synchronize(device)
    policy_seconds = time.perf_counter() - policy_started
    raw_actions = np.concatenate(raw_action_chunks).reshape(
        len(frame_metrics), len(VARIANTS), 2
    )
    normalized_actions = np.clip(raw_actions, -1.0, 1.0)
    physical_actions = np.empty_like(normalized_actions)
    physical_actions[..., 0] = (
        normalized_actions[..., 0] + 1.0
    ) * 0.25
    physical_actions[..., 1] = normalized_actions[..., 1] * 2.0
    high_risk_mask = np.asarray(
        [bool(frame["high_risk_close_scene"]) for frame in frame_metrics]
    )
    policy_comparison = build_policy_comparison(
        normalized_actions, physical_actions, high_risk_mask
    )
    policy_comparison["policy_inference"] = {
        "seconds": policy_seconds,
        "frames_per_second_all_four_variants": (
            len(frame_metrics) / policy_seconds
        ),
        "batch_size": args.policy_batch_size,
    }
    (args.output_dir / "policy_comparison.json").write_text(
        json.dumps(
            policy_comparison,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    save_policy_csv(
        args.output_dir / "policy_actions.csv",
        frame_metrics,
        normalized_actions,
        physical_actions,
    )
    for frame_index, frame in enumerate(frame_metrics):
        actions = {}
        for variant_index, name in enumerate(VARIANTS):
            actions[name] = {
                "normalized": _round_list(
                    normalized_actions[frame_index, variant_index]
                ),
                "physical_linear_angular": _round_list(
                    physical_actions[frame_index, variant_index]
                ),
            }
        frame["policy_actions"] = actions
        frame["policy_action_l2_vs_oracle"] = {
            name: float(
                np.linalg.norm(
                    physical_actions[frame_index, variant_index]
                    - physical_actions[frame_index, 0]
                )
            )
            for variant_index, name in enumerate(VARIANTS[1:], start=1)
        }

    np.savez_compressed(
        args.output_dir / "pedestrian_maps.npz",
        variant_order=np.asarray(VARIANTS),
        pedestrian_maps_mps=pedestrian_maps,
        timestamps_ns=np.asarray(timestamps, dtype=np.int64),
        raw_policy_actions=raw_actions,
        normalized_policy_actions=normalized_actions,
        physical_policy_actions=physical_actions,
    )
    with (args.output_dir / "frame_metrics.jsonl").open(
        "x", encoding="utf-8"
    ) as metrics_stream:
        for frame in frame_metrics:
            metrics_stream.write(
                json.dumps(frame, sort_keys=True, allow_nan=False) + "\n"
            )

    overview_frame = save_overview_visualization(
        visualizations_dir / "action_difference_overview.png",
        pedestrian_maps,
        physical_actions,
    )
    checks = {
        "real_checkpoint_strictly_loaded": True,
        "perception_ground_truth_inputs_empty": (
            contract["inference_ground_truth_inputs"] == []
            and contract["target_ground_truth_inputs"] == []
        ),
        "perception_errors_zero": perception_errors == 0,
        "map_shape_is_2x80x80": pedestrian_maps.shape[2:] == PED_MAP_SHAPE,
        "maps_finite": bool(np.isfinite(pedestrian_maps).all()),
        "observation_shape_is_19202": observation_array.shape[2:]
        == (OBSERVATION_SIZE,),
        "observations_finite": bool(np.isfinite(observation_array).all()),
        "timestamps_strictly_increasing_within_sequence": all(
            timestamps[index] < timestamps[index + 1]
            or frame_metrics[index]["episode_id"]
            != frame_metrics[index + 1]["episode_id"]
            for index in range(len(timestamps) - 1)
        ),
        "tracker_and_scan_history_reset": sequence_resets > 0,
        "original_policy_strict_weight_items": policy_weight_items == 163,
        "actions_finite": bool(np.isfinite(physical_actions).all()),
        "all_requested_frames_processed": len(frame_metrics) == args.frames,
    }
    predicted_exclusion_reasons = Counter(
        str(excluded["reason"])
        for frame in frame_metrics
        for excluded in frame["predicted_map_diagnostics"]["excluded_tracks"]
    )
    predicted_written_cells = sum(
        int(frame["map_stats"]["predicted"]["written_cell_count"])
        for frame in frame_metrics
    )
    oracle_written_cells = sum(
        int(frame["map_stats"]["oracle"]["written_cell_count"])
        for frame in frame_metrics
    )
    oracle_8m_written_cells = sum(
        int(frame["map_stats"]["oracle_8m"]["written_cell_count"])
        for frame in frame_metrics
    )
    summary = {
        "schema": "predicted-ped-map-shadow-replay-summary/v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "frames": len(frame_metrics),
        "split": args.split,
        "sequence_resets": sequence_resets,
        "perception_errors": perception_errors,
        "map_array_shape": list(pedestrian_maps.shape),
        "observation_array_shape": list(observation_array.shape),
        "checks": checks,
        "timing": {
            key: finite_stats(value)
            for key, value in timing_values.items()
        },
        "map_totals": {
            name: {
                "written_cells": int(
                    sum(
                        int(frame["map_stats"][name]["written_cell_count"])
                        for frame in frame_metrics
                    )
                ),
                "nonzero_cells": int(
                    sum(
                        int(frame["map_stats"][name]["nonzero_cell_count"])
                        for frame in frame_metrics
                    )
                ),
            }
            for name in VARIANTS
        },
        "predicted_track_filtering": {
            "same_cell_conflict_count": int(
                sum(
                    int(
                        frame["predicted_map_diagnostics"][
                            "same_cell_conflict_count"
                        ]
                    )
                    for frame in frame_metrics
                )
            ),
            "dropped_track_count": int(
                sum(
                    int(
                        frame["predicted_map_diagnostics"][
                            "dropped_track_count"
                        ]
                    )
                    for frame in frame_metrics
                )
            ),
            "exclusion_reason_counts": dict(
                sorted(predicted_exclusion_reasons.items())
            ),
        },
        "map_coverage_and_count_bias": {
            "predicted_written_cells": predicted_written_cells,
            "oracle_written_cells": oracle_written_cells,
            "oracle_8m_written_cells": oracle_8m_written_cells,
            "predicted_minus_oracle_written_cells": (
                predicted_written_cells - oracle_written_cells
            ),
            "predicted_minus_oracle_8m_written_cells": (
                predicted_written_cells - oracle_8m_written_cells
            ),
            "predicted_coverage_vs_oracle": _safe_ratio(
                predicted_written_cells, oracle_written_cells
            ),
            "predicted_coverage_vs_oracle_8m": _safe_ratio(
                predicted_written_cells, oracle_8m_written_cells
            ),
        },
        "predicted_vs_oracle": _aggregate_match_metrics(
            frame_metrics, "predicted_vs_oracle"
        ),
        "predicted_vs_oracle_8m": _aggregate_match_metrics(
            frame_metrics, "predicted_vs_oracle_8m"
        ),
        "policy_comparison": policy_comparison,
        "visualization_max_action_difference_frame": overview_frame,
        "capability_boundaries": [
            (
                "The perception input is limited to approximately 8 m while "
                "the unchanged DRL map spans 20 m forward; cells beyond sensed "
                "range naturally remain zero."
            ),
            (
                "A stationary pedestrian has vx=vy=0 and is therefore "
                "indistinguishable from an empty cell in the two-channel "
                "legacy representation."
            ),
            (
                "No occupancy channel or epsilon velocity was added because "
                "that would change the pretrained policy contract."
            ),
        ],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "torch": torch.__version__,
            "numpy": np.__version__,
            "cuda_available": torch.cuda.is_available(),
            "device": str(device),
        },
        "outputs": {
            "config": "config.json",
            "summary": "summary.json",
            "tracks": "predicted_tracks.jsonl",
            "maps": "pedestrian_maps.npz",
            "frame_metrics": "frame_metrics.jsonl",
            "policy_comparison": "policy_comparison.json",
            "policy_actions": "policy_actions.csv",
            "visualization": "visualizations/action_difference_overview.png",
        },
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "run_command.txt").write_text(
        " ".join([sys.executable, *sys.argv]) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": summary["status"],
                "output_dir": str(args.output_dir),
                "frames": len(frame_metrics),
                "checks": checks,
            },
            indent=2,
        )
    )
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
