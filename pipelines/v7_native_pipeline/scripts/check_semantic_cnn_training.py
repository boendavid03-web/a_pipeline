#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --device, --log, --model-code, --model-dir, --out-dir, --periodic-max
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, NPY, PNG, PT, TXT
# 可能使用的关键环境变量：BAD_RE, BEST_RE, CUDA, DATASET_ROOT, DEV_RE, EPOCH_RE, EPOCH_STEP_RE, IGNORECASE, LOG_DIR, LOG_RE, MODEL_DIR, OUT_DIR, PERIODIC_MAX, PROJECT_ROOT, RMSE, RUN_ROOT, SAVED_BEST_RE, SAVED_PERIODIC_RE, SCRIPT_DIR, TRAIN_RE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.488295204 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.913035738 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_semantic_cnn_training_check_helpers.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_semantic_cnn_training_check_helpers.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_semantic_cnn_training.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
from collections import defaultdict
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
RUN_ROOT = os.environ.get("RUN_ROOT") or str(PROJECT_ROOT / "experiments" / "legacy_runs")
DATASET_ROOT = os.path.join(RUN_ROOT, "datasets", "semantic2d_native_lidar")
MODEL_DIR = os.path.join(RUN_ROOT, "training", "semantic_cnn")
LOG_DIR = os.path.join(RUN_ROOT, "logs")
SCRIPT_DIR = os.path.join(RUN_ROOT, "training", "work", "semantic_cnn_v7", "scripts")
OUT_DIR = os.path.join(MODEL_DIR, "eval_reports")
PERIODIC_MAX = 300

LOG_RE = re.compile(r"(semantic_cnn_overnight_301_|10_train_semantic_cnn_).*\.log$")
EPOCH_STEP_RE = re.compile(r"Epoch \[(\d+)/(\d+)\], Step")
EPOCH_RE = re.compile(r"Epoch \[(\d+)/(\d+)\]\s*$")
TRAIN_RE = re.compile(r"Train set: Average loss: ([0-9.eE+-]+)")
DEV_RE = re.compile(r"Validation set: Average loss: ([0-9.eE+-]+)")
BEST_RE = re.compile(r"Best dev loss: ([0-9.eE+-]+)")
SAVED_BEST_RE = re.compile(r"Saved best_dev: (yes|no)")
SAVED_PERIODIC_RE = re.compile(r"Saved periodic checkpoint: (.+)")
BAD_RE = re.compile(
    r"Traceback|CUDA out of memory|killed|exception|error|(^|[^a-z])nan([^a-z]|$)|(^|[^a-z])inf([^a-z]|$)",
    re.IGNORECASE,
)


def jdump(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, sort_keys=True)


def read_lines(path):
    with open(path, "r", errors="replace") as f:
        return f.read().splitlines()


def latest_log():
    logs = []
    if os.path.isdir(LOG_DIR):
        for name in os.listdir(LOG_DIR):
            if LOG_RE.match(name):
                path = os.path.join(LOG_DIR, name)
                logs.append((os.path.getmtime(path), path))
    if not logs:
        raise FileNotFoundError("no semantic_cnn_overnight_301_*.log found")
    return sorted(logs, reverse=True)[0][1]


def parse_log(path):
    rows = []
    cur = None
    last_epoch = None
    bad = []
    with open(path, "r", errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            stripped = line.strip()
            if BAD_RE.search(stripped):
                bad.append({"line": lineno, "text": stripped[:240]})
            m = EPOCH_STEP_RE.search(stripped)
            if m:
                last_epoch = int(m.group(1))
                cur = {
                    "epoch": last_epoch,
                    "num_epochs": int(m.group(2)),
                    "train_loss": None,
                    "dev_loss": None,
                    "best_dev_loss": None,
                    "saved_best_dev": None,
                    "periodic_checkpoint": "",
                }
                continue
            m = TRAIN_RE.search(stripped)
            if m and cur is not None:
                cur["train_loss"] = float(m.group(1))
                continue
            m = DEV_RE.search(stripped)
            if m and cur is not None:
                cur["dev_loss"] = float(m.group(1))
                continue
            m = BEST_RE.search(stripped)
            if m and cur is not None:
                cur["best_dev_loss"] = float(m.group(1))
                continue
            m = SAVED_BEST_RE.search(stripped)
            if m and cur is not None:
                cur["saved_best_dev"] = (m.group(1) == "yes")
                continue
            m = SAVED_PERIODIC_RE.search(stripped)
            if m and cur is not None:
                value = m.group(1)
                cur["periodic_checkpoint"] = "" if value == "no" else value
                rows.append(cur)
                cur = None
                continue
            m = EPOCH_RE.search(stripped)
            if m:
                last_epoch = int(m.group(1))
    final_saved = any("Saved final:" in line for line in read_lines(path)[-200:])
    run_model_saved = any("Saved run model:" in line for line in read_lines(path)[-200:])
    return {
        "path": path,
        "rows": rows,
        "bad_matches": bad,
        "last_epoch": last_epoch,
        "final_saved_in_log": final_saved,
        "run_model_saved_in_log": run_model_saved,
    }


def write_loss_csv(rows, path):
    fields = ["epoch", "num_epochs", "train_loss", "dev_loss", "best_dev_loss", "saved_best_dev", "periodic_checkpoint"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_loss(rows, path):
    epochs = [r["epoch"] for r in rows]
    train = [r["train_loss"] for r in rows]
    dev = [r["dev_loss"] for r in rows]
    best_rows = [r for r in rows if r["saved_best_dev"]]
    plt.figure(figsize=(11, 6))
    plt.plot(epochs, train, label="train loss", linewidth=1.4)
    plt.plot(epochs, dev, label="dev loss", linewidth=1.4)
    if best_rows:
        plt.scatter([r["epoch"] for r in best_rows], [r["dev_loss"] for r in best_rows], s=18, label="best_dev update")
        best = min(rows, key=lambda r: r["dev_loss"])
        plt.axvline(best["epoch"], color="black", linestyle="--", linewidth=0.9, alpha=0.6)
    plt.yscale("log")
    plt.xlabel("epoch")
    plt.ylabel("loss (log scale)")
    plt.title("SemanticCNN native cmd training loss")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def checkpoint_summary(path):
    ckpt = torch.load(path, map_location="cpu")
    model_state = ckpt.get("model_state_dict", ckpt.get("model", {}))
    return {
        "path": path,
        "exists": os.path.exists(path),
        "size_bytes": os.path.getsize(path),
        "sha256": sha256_file(path),
        "keys": sorted(list(ckpt.keys())),
        "model_state_dict_key_count": len(model_state),
        "model_state_dict_first_keys": list(model_state.keys())[:10],
        "epoch": ckpt.get("epoch"),
        "train_loss": ckpt.get("train_loss"),
        "dev_loss": ckpt.get("dev_loss"),
        "best_dev_loss": ckpt.get("best_dev_loss"),
        "dataset_root": ckpt.get("dataset_root"),
        "target": ckpt.get("target"),
        "batch_size": ckpt.get("batch_size"),
        "num_epochs": ckpt.get("num_epochs"),
        "normalization": ckpt.get("normalization"),
        "timestamp": ckpt.get("timestamp"),
    }


def model_tensors_equal(path_a, path_b):
    a = torch.load(path_a, map_location="cpu").get("model_state_dict")
    b = torch.load(path_b, map_location="cpu").get("model_state_dict")
    if a is None or b is None or a.keys() != b.keys():
        return False
    for key in a.keys():
        if not torch.equal(a[key], b[key]):
            return False
    return True


def session_dirs(dataset_root):
    with open(os.path.join(dataset_root, "dataset.txt"), "r") as f:
        names = [line.strip().rstrip("/") for line in f if line.strip()]
    return [os.path.join(dataset_root, name) for name in names]


def npy_count(path):
    if not os.path.isdir(path):
        return 0
    return sum(1 for name in os.listdir(path) if name.endswith(".npy"))


def safe_load(path):
    arr = np.load(path).astype(np.float64).reshape(-1)
    return np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)


def split_file_names(session_dir, split):
    path = os.path.join(session_dir, split + ".txt")
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, "r") as f:
        for line in f:
            name = line.strip()
            if name.endswith(".npy"):
                out.append(os.path.basename(name))
    return out


def target_for_dataset_sample(session_dir, sample_name):
    sample_name = os.path.basename(str(sample_name))
    stem, extension = os.path.splitext(sample_name)
    if extension != ".npy":
        raise ValueError(f"split sample must end with .npy: {sample_name}")

    if not stem.isdigit():
        raw = safe_load(
            os.path.join(session_dir, "cmd_velocities", sample_name)
        )
        if raw.shape[0] < 3:
            raise ValueError(
                "cmd_velocities target must have 3 values: "
                f"{session_dir}/{sample_name}"
            )
        return raw[[0, 2]]

    idx_num = int(stem)
    frame_count = npy_count(os.path.join(session_dir, "scans_lidar"))
    seq_len = 10
    if idx_num + seq_len <= frame_count:
        idx_s = idx_num
    elif idx_num - seq_len >= 0:
        idx_s = idx_num - seq_len
    else:
        idx_s = frame_count // 2
    end_name = f"{idx_s + seq_len - 1:07d}.npy"
    raw = safe_load(os.path.join(session_dir, "cmd_velocities", end_name))
    if raw.shape[0] < 3:
        raise ValueError(f"cmd_velocities target must have 3 values: {session_dir}/{end_name}")
    return raw[[0, 2]]


def summarize_targets(values):
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return {}
    lin = arr[:, 0]
    ang = arr[:, 1]
    speed_abs = np.abs(lin)
    turn_abs = np.abs(ang)
    nonzero = (speed_abs >= 1e-4) | (turn_abs >= 1e-4)
    stop = (speed_abs < 1e-4) & (turn_abs < 1e-4)
    move = speed_abs >= 1e-4
    turn = turn_abs >= 1e-4
    forward = lin > 1e-4
    back = lin < -1e-4
    left = ang > 1e-4
    right = ang < -1e-4
    return {
        "samples": int(arr.shape[0]),
        "linear_x": stats_1d(lin),
        "angular_z": stats_1d(ang),
        "nonzero_ratio_any": float(nonzero.mean()),
        "near_zero_ratio_any": float(stop.mean()),
        "linear_near_zero_ratio": float((speed_abs < 1e-4).mean()),
        "angular_near_zero_ratio": float((turn_abs < 1e-4).mean()),
        "forward_ratio": float(forward.mean()),
        "back_ratio": float(back.mean()),
        "turn_ratio": float(turn.mean()),
        "left_turn_ratio": float(left.mean()),
        "right_turn_ratio": float(right.mean()),
        "stop_ratio": float(stop.mean()),
        "move_ratio": float(move.mean()),
    }


def stats_1d(x):
    x = np.asarray(x, dtype=np.float64)
    return {
        "min": float(np.min(x)),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
        "max": float(np.max(x)),
    }


def dataset_stats(dataset_root):
    sessions = session_dirs(dataset_root)
    out = {
        "dataset_root": dataset_root,
        "sessions": [],
        "splits": {},
        "all_split_targets": {},
    }
    for session in sessions:
        entry = {"session": os.path.basename(session)}
        for folder in ["scans_lidar", "cmd_velocities", "sub_goals_local", "semantic_label", "angles_lidar", "valid_mask_lidar"]:
            entry[folder + "_count"] = npy_count(os.path.join(session, folder))
        entry["scan_cmd_file_count_match"] = entry["scans_lidar_count"] == entry["cmd_velocities_count"]
        for split in ["train", "dev", "test"]:
            entry[split + "_samples"] = len(split_file_names(session, split))
        out["sessions"].append(entry)

    for split in ["train", "dev", "test"]:
        targets = []
        for session in sessions:
            for sample_name in split_file_names(session, split):
                targets.append(target_for_dataset_sample(session, sample_name))
        out["splits"][split] = summarize_targets(targets)
    all_targets = []
    for split in ["train", "dev", "test"]:
        for session in sessions:
            for sample_name in split_file_names(session, split):
                all_targets.append(target_for_dataset_sample(session, sample_name))
    out["all_split_targets"] = summarize_targets(all_targets)
    return out


def import_training_model():
    sys.path.insert(0, SCRIPT_DIR)
    from model import Bottleneck, NavDataset, SemanticCNN
    return Bottleneck, NavDataset, SemanticCNN


def load_model(path, device):
    Bottleneck, _, SemanticCNN = import_training_model()
    model = SemanticCNN(Bottleneck, [2, 1, 1])
    ckpt = torch.load(path, map_location=device)
    state = ckpt.get("model_state_dict", ckpt.get("model"))
    state = {k.replace("module.", "", 1): v for k, v in state.items()}
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    return model, ckpt


def corrcoef_safe(a, b):
    if len(a) < 2 or np.std(a) == 0 or np.std(b) == 0:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def metrics_for_arrays(targets, preds):
    targets = np.asarray(targets, dtype=np.float64)
    preds = np.asarray(preds, dtype=np.float64)
    err = preds - targets
    mse_by_dim = np.mean(err ** 2, axis=0)
    mae_by_dim = np.mean(np.abs(err), axis=0)
    return {
        "samples": int(targets.shape[0]),
        "mse": float(np.mean(err ** 2)),
        "rmse": float(math.sqrt(np.mean(err ** 2))),
        "mae": float(np.mean(np.abs(err))),
        "linear_x": {
            "mse": float(mse_by_dim[0]),
            "rmse": float(math.sqrt(mse_by_dim[0])),
            "mae": float(mae_by_dim[0]),
            "correlation": corrcoef_safe(targets[:, 0], preds[:, 0]),
        },
        "angular_z": {
            "mse": float(mse_by_dim[1]),
            "rmse": float(math.sqrt(mse_by_dim[1])),
            "mae": float(mae_by_dim[1]),
            "correlation": corrcoef_safe(targets[:, 1], preds[:, 1]),
        },
        "prediction": {
            "linear_x": stats_1d(preds[:, 0]),
            "angular_z": stats_1d(preds[:, 1]),
        },
        "target": {
            "linear_x": stats_1d(targets[:, 0]),
            "angular_z": stats_1d(targets[:, 1]),
        },
        "error": {
            "linear_x": stats_1d(err[:, 0]),
            "angular_z": stats_1d(err[:, 1]),
        },
        "control_behavior": control_behavior_metrics(targets, preds),
    }


def subset_metrics(targets, preds):
    targets = np.asarray(targets, dtype=np.float64)
    preds = np.asarray(preds, dtype=np.float64)
    lin_abs = np.abs(targets[:, 0])
    ang_abs = np.abs(targets[:, 1])
    masks = {
        "nonzero_any": (lin_abs >= 1e-4) | (ang_abs >= 1e-4),
        "stop": (lin_abs < 1e-4) & (ang_abs < 1e-4),
        "move": lin_abs >= 1e-4,
        "turn": ang_abs >= 1e-4,
    }
    out = {}
    for name, mask in masks.items():
        out[name] = metrics_for_arrays(targets[mask], preds[mask]) if int(mask.sum()) else {"samples": 0}
    return out


def control_behavior_metrics(targets, preds, threshold=1e-4):
    targets = np.asarray(targets, dtype=np.float64)
    preds = np.asarray(preds, dtype=np.float64)
    linear_active = np.abs(targets[:, 0]) >= threshold
    angular_active = np.abs(targets[:, 1]) >= threshold
    target_stop = ~linear_active & ~angular_active
    pred_stop = (np.abs(preds[:, 0]) < threshold) & (np.abs(preds[:, 1]) < threshold)

    def direction_accuracy(mask, dim):
        if not int(mask.sum()):
            return None
        return float(np.mean(np.sign(preds[mask, dim]) == np.sign(targets[mask, dim])))

    forward = targets[:, 0] >= threshold
    reverse = targets[:, 0] <= -threshold
    left_turn = targets[:, 1] >= threshold
    right_turn = targets[:, 1] <= -threshold
    return {
        "threshold": threshold,
        "linear_x": {
            "active_target_samples": int(linear_active.sum()),
            "forward_target_samples": int(forward.sum()),
            "reverse_target_samples": int(reverse.sum()),
            "active_direction_accuracy": direction_accuracy(linear_active, 0),
            "forward_direction_accuracy": direction_accuracy(forward, 0),
            "reverse_direction_accuracy": direction_accuracy(reverse, 0),
        },
        "angular_z": {
            "turn_target_samples": int(angular_active.sum()),
            "left_turn_target_samples": int(left_turn.sum()),
            "right_turn_target_samples": int(right_turn.sum()),
            "turn_direction_accuracy": direction_accuracy(angular_active, 1),
            "left_turn_direction_accuracy": direction_accuracy(left_turn, 1),
            "right_turn_direction_accuracy": direction_accuracy(right_turn, 1),
        },
        "target_stop": {
            "samples": int(target_stop.sum()),
            "predicted_stop_ratio": float(pred_stop[target_stop].mean()) if int(target_stop.sum()) else None,
        },
    }


def load_split_cache(dataset_root, split):
    _, NavDataset, _ = import_training_model()
    dataset = NavDataset(dataset_root, split)
    loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=False, drop_last=False, pin_memory=False)
    targets = []
    scan_norm_values = []
    subgoal_norm_values = []
    scans = []
    semantics = []
    goals = []
    total_batches = int(math.ceil(len(dataset) / 64.0))
    for batch_idx, batch in enumerate(loader, 1):
        scans.append(batch["scan_map"].clone())
        semantics.append(batch["semantic_map"].clone())
        goals.append(batch["sub_goal"].clone())
        targets.append(batch["velocity"].numpy())
        scan_norm_values.append(batch["scan_map"].numpy().reshape(-1))
        subgoal_norm_values.append(batch["sub_goal"].numpy().reshape(-1))
        if batch_idx == 1 or batch_idx % 5 == 0 or batch_idx == total_batches:
            print(f"loaded {split} cache batch {batch_idx}/{total_batches}", flush=True)
    return {
        "scans": torch.cat(scans, dim=0),
        "semantics": torch.cat(semantics, dim=0),
        "goals": torch.cat(goals, dim=0),
        "targets": np.vstack(targets),
        "scan_norm": np.concatenate(scan_norm_values),
        "subgoal_norm": np.concatenate(subgoal_norm_values),
        "normalization": {
            "source": dataset.normalization_source,
            "stats_json": dataset.normalization_stats_path,
            "sub_goal_mean": np.asarray(dataset.g_mu, dtype=np.float64),
            "sub_goal_std": np.asarray(dataset.g_std, dtype=np.float64),
        },
    }


def evaluate_checkpoint(path, split_cache, device, split, pred_csv_path=None, figure_prefix=None):
    model, ckpt = load_model(path, device)
    preds = []
    batch_size = 64
    print(f"evaluating {os.path.basename(path)} on {device}", flush=True)
    with torch.no_grad():
        total = split_cache["scans"].shape[0]
        total_batches = int(math.ceil(total / float(batch_size)))
        for batch_idx, start in enumerate(range(0, total, batch_size), 1):
            end = min(start + batch_size, total)
            scan = split_cache["scans"][start:end].to(device)
            semantic = split_cache["semantics"][start:end].to(device)
            goal = split_cache["goals"][start:end].to(device)
            out = model(scan, semantic, goal).detach().cpu().numpy()
            preds.append(out)
            if batch_idx == 1 or batch_idx % 5 == 0 or batch_idx == total_batches:
                print(f"  forward batch {batch_idx}/{total_batches}", flush=True)
    preds = np.vstack(preds)
    targets = split_cache["targets"]
    metrics = metrics_for_arrays(targets, preds)
    metrics["subsets"] = subset_metrics(targets, preds)
    metrics["checkpoint_metadata"] = {
        "epoch": ckpt.get("epoch"),
        "dev_loss": ckpt.get("dev_loss"),
        "train_loss": ckpt.get("train_loss"),
        "best_dev_loss": ckpt.get("best_dev_loss"),
        "timestamp": ckpt.get("timestamp"),
    }
    metrics["collapse_checks"] = collapse_checks(targets, preds)
    scan_norm = split_cache["scan_norm"]
    subgoal_norm = split_cache["subgoal_norm"]
    metrics[f"{split}_input_stats_model_normalized"] = {
        "scan_map": stats_1d(scan_norm),
        "sub_goal": stats_1d(subgoal_norm),
    }
    goal_mean = split_cache["normalization"]["sub_goal_mean"]
    goal_std = split_cache["normalization"]["sub_goal_std"]
    subgoal_pre = (
        subgoal_norm.reshape(-1, 2) * goal_std.reshape(1, 2)
        + goal_mean.reshape(1, 2)
    ).reshape(-1)
    metrics[f"{split}_input_stats_pre_normalization_inferred"] = {
        "scan_map": stats_1d(scan_norm),
        "sub_goal": stats_1d(subgoal_pre),
    }
    metrics["normalization"] = {
        "source": split_cache["normalization"]["source"],
        "stats_json": split_cache["normalization"]["stats_json"],
        "sub_goal_mean": goal_mean.tolist(),
        "sub_goal_std": goal_std.tolist(),
        "scan_map": "range clipped to pool_range_max and divided by pool_range_max",
    }
    if pred_csv_path:
        write_predictions_csv(pred_csv_path, targets, preds)
    if figure_prefix:
        make_pred_figures(figure_prefix, targets, preds, split)
    return metrics, targets, preds


def collapse_checks(targets, preds):
    pred_std = np.std(preds, axis=0)
    target_std = np.std(targets, axis=0)
    pred_abs_mean = np.mean(np.abs(preds), axis=0)
    near_zero_ratio = np.mean(np.abs(preds) < 1e-4, axis=0)
    return {
        "pred_std": {"linear_x": float(pred_std[0]), "angular_z": float(pred_std[1])},
        "target_std": {"linear_x": float(target_std[0]), "angular_z": float(target_std[1])},
        "pred_abs_mean": {"linear_x": float(pred_abs_mean[0]), "angular_z": float(pred_abs_mean[1])},
        "pred_near_zero_ratio": {"linear_x": float(near_zero_ratio[0]), "angular_z": float(near_zero_ratio[1])},
        "linear_x_near_constant": bool(pred_std[0] < max(1e-4, 0.05 * target_std[0])),
        "angular_z_near_constant": bool(pred_std[1] < max(1e-4, 0.05 * target_std[1])),
        "all_near_zero": bool(np.mean(np.linalg.norm(preds, axis=1) < 1e-4) > 0.95),
    }


def write_predictions_csv(path, targets, preds):
    with open(path, "w", newline="") as f:
        fields = ["sample_index", "target_linear_x", "target_angular_z", "pred_linear_x", "pred_angular_z", "err_linear_x", "err_angular_z", "class"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for i, (t, p) in enumerate(zip(targets, preds)):
            if abs(t[0]) < 1e-4 and abs(t[1]) < 1e-4:
                cls = "stop"
            elif abs(t[1]) >= 1e-4:
                cls = "turn"
            else:
                cls = "move"
            writer.writerow({
                "sample_index": i,
                "target_linear_x": float(t[0]),
                "target_angular_z": float(t[1]),
                "pred_linear_x": float(p[0]),
                "pred_angular_z": float(p[1]),
                "err_linear_x": float(p[0] - t[0]),
                "err_angular_z": float(p[1] - t[1]),
                "class": cls,
            })


def make_pred_figures(prefix, targets, preds, split):
    for dim, name in enumerate(["linear_x", "angular_z"]):
        plt.figure(figsize=(6, 6))
        plt.scatter(targets[:, dim], preds[:, dim], s=9, alpha=0.45)
        lo = float(min(np.min(targets[:, dim]), np.min(preds[:, dim])))
        hi = float(max(np.max(targets[:, dim]), np.max(preds[:, dim])))
        plt.plot([lo, hi], [lo, hi], color="black", linewidth=0.9)
        plt.xlabel(f"target {name}")
        plt.ylabel(f"pred {name}")
        plt.title(f"{split} pred vs target: {name}")
        plt.grid(True, alpha=0.25)
        plt.tight_layout()
        plt.savefig(f"{prefix}_pred_target_scatter_{name}.png", dpi=160)
        plt.close()

        plt.figure(figsize=(7, 4))
        plt.hist(preds[:, dim] - targets[:, dim], bins=60)
        plt.xlabel(f"prediction error {name}")
        plt.ylabel("count")
        plt.title(f"{split} error histogram: {name}")
        plt.grid(True, alpha=0.25)
        plt.tight_layout()
        plt.savefig(f"{prefix}_error_hist_{name}.png", dpi=160)
        plt.close()


def checkpoint_presence():
    present = {}
    for name in ["latest", "best_dev", "final"]:
        filename = f"semantic_cnn_native_cmd_{name}.pth"
        present[filename] = os.path.exists(os.path.join(MODEL_DIR, filename))
    periodic = {}
    for epoch in range(10, PERIODIC_MAX + 1, 10):
        filename = f"semantic_cnn_native_cmd_epoch_{epoch:04d}.pth"
        periodic[filename] = os.path.exists(os.path.join(MODEL_DIR, filename))
    return {"named": present, "periodic": periodic}


def write_comparison_csv(path, rows):
    fields = ["checkpoint", "epoch", "metadata_dev_loss", "computed_dev_mse", "linear_x_rmse", "angular_z_rmse", "pred_std_linear_x", "pred_std_angular_z", "suspected_collapse"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(rows, fields):
    out = []
    out.append("| " + " | ".join(fields) + " |")
    out.append("| " + " | ".join(["---"] * len(fields)) + " |")
    for row in rows:
        out.append("| " + " | ".join(str(row.get(f, "")) for f in fields) + " |")
    return "\n".join(out)


def fmt(x, nd=6):
    if x is None:
        return "n/a"
    if isinstance(x, float):
        return f"{x:.{nd}g}"
    return str(x)


def write_reports(out):
    full_path = os.path.join(OUT_DIR, "semantic_cnn_full_check_report.md")
    effect_path = os.path.join(OUT_DIR, "semantic_cnn_training_effect_report.md")
    log = out["log"]
    rows = log["rows"]
    best_row = min(rows, key=lambda r: r["dev_loss"]) if rows else None
    final_row = rows[-1] if rows else None
    comparison = out["checkpoint_eval_comparison"]
    best_metrics = out["best_dev_metrics"]
    test_metrics = out["best_dev_test_metrics"]
    ds = out["dataset_stats"]
    train_first = rows[0]["train_loss"] if rows else None
    train_last = rows[-1]["train_loss"] if rows else None
    dev_first = rows[0]["dev_loss"] if rows else None
    dev_last = rows[-1]["dev_loss"] if rows else None
    recent_dev_losses = [row["dev_loss"] for row in rows[-3:]]
    late_dev_strictly_improving = (
        len(recent_dev_losses) == 3
        and recent_dev_losses[0] > recent_dev_losses[1] > recent_dev_losses[2]
    )
    periodic_missing = [k for k, v in out["checkpoint_presence"]["periodic"].items() if not v]
    named_missing = [k for k, v in out["checkpoint_presence"]["named"].items() if not v]
    periodic_label = f"0010..{PERIODIC_MAX:04d}" if PERIODIC_MAX >= 10 else "none expected (<10 epochs)"
    final_worse_pct = None
    if best_row and final_row:
        final_worse_pct = 100.0 * (final_row["dev_loss"] - best_row["dev_loss"]) / best_row["dev_loss"]
    collapse = best_metrics["collapse_checks"]
    target_all = ds["all_split_targets"]
    target_dev = ds["splits"].get("dev", {})
    target_test = ds["splits"].get("test", {})

    if not rows:
        effect_interpretation = ["- No parsed epoch metrics are available; do not draw an overfitting conclusion."]
    elif final_worse_pct is None:
        effect_interpretation = ["- Best-vs-final dev comparison is unavailable; do not draw an overfitting conclusion."]
    elif final_worse_pct > 5.0:
        effect_interpretation = [
            "- Final dev loss is materially worse than the best-dev point; this run shows overfitting after best_dev.",
            "- Use best_dev for held-out test evaluation and downstream deployment.",
        ]
    elif late_dev_strictly_improving:
        effect_interpretation = [
            "- The final three dev losses strictly improve; a separate longer experiment may be considered.",
            "- Do not call that separate experiment a resume unless checkpoint-resume support is verified.",
        ]
    else:
        effect_interpretation = [
            "- Final dev loss is close to the best-dev point; this run does not establish a clear late overfitting or improvement trend.",
            "- Use best_dev for held-out test evaluation and downstream deployment.",
        ]

    effect = [
        "# SemanticCNN Training Effect Report",
        "",
        f"- Log: `{log['path']}`",
        f"- Parsed epochs: {len(rows)}, last epoch: {log['last_epoch']}",
        f"- Train loss: {fmt(train_first)} -> {fmt(train_last)}",
        f"- Dev loss: {fmt(dev_first)} -> {fmt(dev_last)}",
        f"- Best dev: epoch {best_row['epoch'] if best_row else 'n/a'}, loss {fmt(best_row['dev_loss'] if best_row else None)}",
        f"- Final dev vs best_dev: {fmt(final_worse_pct, 4)}% worse" if final_worse_pct is not None else "- Final dev vs best_dev: n/a",
        "",
        "Interpretation:",
    ] + effect_interpretation
    with open(effect_path, "w") as f:
        f.write("\n".join(effect) + "\n")

    cmp_rows = []
    for item in comparison:
        cmp_rows.append({
            "checkpoint": item["checkpoint"],
            "epoch": item["epoch"],
            "metadata_dev_loss": fmt(item["metadata_dev_loss"]),
            "computed_dev_mse": fmt(item["computed_dev_mse"]),
            "linear_x_rmse": fmt(item["linear_x_rmse"]),
            "angular_z_rmse": fmt(item["angular_z_rmse"]),
            "pred_std": f"{fmt(item['pred_std_linear_x'])}/{fmt(item['pred_std_angular_z'])}",
            "collapse": item["suspected_collapse"],
        })

    session_rows = []
    for s in ds["sessions"]:
        session_rows.append({
            "session": s["session"],
            "train": s["train_samples"],
            "dev": s["dev_samples"],
            "test": s["test_samples"],
            "scans": s["scans_lidar_count"],
            "cmd": s["cmd_velocities_count"],
            "match": s["scan_cmd_file_count_match"],
        })

    full = [
        "# SemanticCNN Native Cmd Full Check",
        "",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        "",
        "## 1. Training End Status",
        f"- Training log: `{log['path']}`",
        f"- Training process still running: {out['training_process_running']}",
        f"- Normal final save found in log: {log['final_saved_in_log']}",
        f"- Parsed epochs: {len(rows)}; actual last epoch: {log['last_epoch']}",
        f"- Bad keyword matches: {len(log['bad_matches'])}",
        "",
        "## 2. Checkpoints",
        f"- Missing named checkpoints: {', '.join(named_missing) if named_missing else 'none'}",
        f"- Missing periodic checkpoints {periodic_label}: {', '.join(periodic_missing) if periodic_missing else 'none'}",
        f"- latest and final model tensors equal: {out['latest_final_model_tensors_equal']}",
        f"- best_dev epoch: {out['checkpoints']['best_dev']['epoch']}, best_dev_loss: {fmt(out['checkpoints']['best_dev']['best_dev_loss'])}",
        f"- final epoch: {out['checkpoints']['final']['epoch']}, final dev_loss: {fmt(out['checkpoints']['final']['dev_loss'])}",
        f"- final vs best_dev dev loss: {fmt(final_worse_pct, 4)}% worse" if final_worse_pct is not None else "- final vs best_dev dev loss: n/a",
        "",
        "Checkpoint metadata files:",
        f"- `{os.path.join(OUT_DIR, 'semantic_cnn_checkpoint_metadata.json')}`",
        "",
        "## 3. Loss Trend",
        f"- Train loss: {fmt(train_first)} -> {fmt(train_last)}",
        f"- Dev loss: {fmt(dev_first)} -> {fmt(dev_last)}",
        f"- Best dev was {'late' if best_row and best_row['epoch'] > 0.66 * best_row['num_epochs'] else 'not late'}: epoch {best_row['epoch'] if best_row else 'n/a'} / {best_row['num_epochs'] if best_row else 'n/a'}",
        f"- Final three dev losses strictly improve: {late_dev_strictly_improving}",
        f"- Loss CSV: `{os.path.join(OUT_DIR, 'semantic_cnn_loss_curve.csv')}`",
        f"- Loss plot: `{os.path.join(OUT_DIR, 'semantic_cnn_loss_curve.png')}`",
        "",
        "## 4. Dataset And Target Distribution",
        markdown_table(session_rows, ["session", "train", "dev", "test", "scans", "cmd", "match"]),
        "",
        f"- All split target samples: {target_all.get('samples')}",
        f"- All stop ratio: {fmt(target_all.get('stop_ratio'))}; move ratio: {fmt(target_all.get('move_ratio'))}; turn ratio: {fmt(target_all.get('turn_ratio'))}",
        f"- Dev stop ratio: {fmt(target_dev.get('stop_ratio'))}; move ratio: {fmt(target_dev.get('move_ratio'))}; turn ratio: {fmt(target_dev.get('turn_ratio'))}",
        f"- Test stop ratio: {fmt(target_test.get('stop_ratio'))}; move ratio: {fmt(target_test.get('move_ratio'))}; turn ratio: {fmt(target_test.get('turn_ratio'))}",
        f"- Dev linear_x std: {fmt(target_dev.get('linear_x', {}).get('std'))}; angular_z std: {fmt(target_dev.get('angular_z', {}).get('std'))}",
        "- A constant/all-zero predictor can look deceptively acceptable if stop or near-zero targets dominate; here stop exists but move/turn samples are substantial, so stratified metrics are needed.",
        "",
        "## 5. Best Dev Prediction Metrics",
        f"- MSE/RMSE/MAE: {fmt(best_metrics['mse'])} / {fmt(best_metrics['rmse'])} / {fmt(best_metrics['mae'])}",
        f"- linear_x RMSE/MAE/corr: {fmt(best_metrics['linear_x']['rmse'])} / {fmt(best_metrics['linear_x']['mae'])} / {fmt(best_metrics['linear_x']['correlation'])}",
        f"- angular_z RMSE/MAE/corr: {fmt(best_metrics['angular_z']['rmse'])} / {fmt(best_metrics['angular_z']['mae'])} / {fmt(best_metrics['angular_z']['correlation'])}",
        f"- Prediction std linear/angular: {fmt(collapse['pred_std']['linear_x'])} / {fmt(collapse['pred_std']['angular_z'])}",
        f"- Target std linear/angular: {fmt(collapse['target_std']['linear_x'])} / {fmt(collapse['target_std']['angular_z'])}",
        f"- Collapse suspected: {collapse['linear_x_near_constant'] or collapse['angular_z_near_constant'] or collapse['all_near_zero']}",
        "",
        "## 6. Held-Out Test Metrics (best_dev checkpoint)",
        "- The checkpoint was selected using dev only; the following metrics are a single held-out test forward pass.",
        f"- MSE/RMSE/MAE: {fmt(test_metrics['mse'])} / {fmt(test_metrics['rmse'])} / {fmt(test_metrics['mae'])}",
        f"- linear_x RMSE/MAE/corr: {fmt(test_metrics['linear_x']['rmse'])} / {fmt(test_metrics['linear_x']['mae'])} / {fmt(test_metrics['linear_x']['correlation'])}",
        f"- angular_z RMSE/MAE/corr: {fmt(test_metrics['angular_z']['rmse'])} / {fmt(test_metrics['angular_z']['mae'])} / {fmt(test_metrics['angular_z']['correlation'])}",
        f"- Turn direction accuracy (left/right): {fmt(test_metrics['control_behavior']['angular_z']['turn_direction_accuracy'])} ({fmt(test_metrics['control_behavior']['angular_z']['left_turn_direction_accuracy'])} / {fmt(test_metrics['control_behavior']['angular_z']['right_turn_direction_accuracy'])})",
        f"- Linear direction accuracy (forward/reverse): {fmt(test_metrics['control_behavior']['linear_x']['active_direction_accuracy'])} ({fmt(test_metrics['control_behavior']['linear_x']['forward_direction_accuracy'])} / {fmt(test_metrics['control_behavior']['linear_x']['reverse_direction_accuracy'])})",
        f"- Target-stop samples: {test_metrics['control_behavior']['target_stop']['samples']}; predicted-stop ratio on target-stop: {fmt(test_metrics['control_behavior']['target_stop']['predicted_stop_ratio'])}",
        f"- Collapse suspected: {test_metrics['collapse_checks']['linear_x_near_constant'] or test_metrics['collapse_checks']['angular_z_near_constant'] or test_metrics['collapse_checks']['all_near_zero']}",
        f"- Test prediction CSV: `{os.path.join(OUT_DIR, 'semantic_cnn_best_dev_test_pred_vs_target.csv')}`",
        "",
        "## 7. Checkpoint Dev Comparison",
        markdown_table(cmp_rows, ["checkpoint", "epoch", "metadata_dev_loss", "computed_dev_mse", "linear_x_rmse", "angular_z_rmse", "pred_std", "collapse"]),
        "",
        "## 8. Normalization Check",
        f"- Normalization source: {best_metrics['normalization']['source']}; stats JSON: `{best_metrics['normalization']['stats_json']}`.",
        f"- sub_goal mean: {best_metrics['normalization']['sub_goal_mean']}; std: {best_metrics['normalization']['sub_goal_std']}.",
        "- scan_map uses per-frame range clipping and division by pool_range_max; no dataset mean/std is applied.",
        "- semantic_map is passed as numeric label IDs without standardization.",
        "- target `[linear_x, angular_z]` is not normalized.",
        f"- Dev inferred pre-normalized scan_map mean/std: {fmt(best_metrics['dev_input_stats_pre_normalization_inferred']['scan_map']['mean'])} / {fmt(best_metrics['dev_input_stats_pre_normalization_inferred']['scan_map']['std'])}",
        f"- Dev inferred pre-normalized sub_goal mean/std: {fmt(best_metrics['dev_input_stats_pre_normalization_inferred']['sub_goal']['mean'])} / {fmt(best_metrics['dev_input_stats_pre_normalization_inferred']['sub_goal']['std'])}",
        "- The checkpoint records the exact normalization metadata used for reproducible evaluation.",
        "",
        "## 9. Recommendation",
        "- Use `semantic_cnn_native_cmd_best_dev.pth` for held-out test evaluation and downstream tests.",
    ]
    if late_dev_strictly_improving:
        full.extend([
            "- The final three dev values strictly improve without a final-vs-best regression; a separate timestamped longer experiment is eligible.",
            "- Verify checkpoint-resume behavior before calling any future longer experiment a resume.",
        ])
    else:
        full.extend([
            "- Do not continue the same run blindly; use early stopping around best_dev behavior and add stratified move/turn/stop evaluation.",
            "- Next most useful step: native normalization stats plus motion-state-balanced dev/test evaluation.",
        ])
    with open(full_path, "w") as f:
        f.write("\n".join(full) + "\n")


def main():
    global DATASET_ROOT, MODEL_DIR, SCRIPT_DIR, OUT_DIR, PERIODIC_MAX
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default=OUT_DIR)
    parser.add_argument("--dataset-root", default=DATASET_ROOT)
    parser.add_argument("--model-dir", default=MODEL_DIR)
    parser.add_argument("--model-code", default=SCRIPT_DIR)
    parser.add_argument("--log", default="")
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"])
    parser.add_argument("--periodic-max", type=int, default=PERIODIC_MAX)
    args = parser.parse_args()
    DATASET_ROOT = args.dataset_root
    MODEL_DIR = args.model_dir
    SCRIPT_DIR = args.model_code
    OUT_DIR = args.out_dir
    PERIODIC_MAX = args.periodic_max
    os.makedirs(args.out_dir, exist_ok=True)

    log_path = args.log or latest_log()
    log = parse_log(log_path)
    write_loss_csv(log["rows"], os.path.join(args.out_dir, "semantic_cnn_loss_curve.csv"))
    plot_loss(log["rows"], os.path.join(args.out_dir, "semantic_cnn_loss_curve.png"))

    ckpt_paths = {
        "latest": os.path.join(args.model_dir, "semantic_cnn_native_cmd_latest.pth"),
        "best_dev": os.path.join(args.model_dir, "semantic_cnn_native_cmd_best_dev.pth"),
        "final": os.path.join(args.model_dir, "semantic_cnn_native_cmd_final.pth"),
    }
    ckpts = {name: checkpoint_summary(path) for name, path in ckpt_paths.items()}
    jdump(os.path.join(args.out_dir, "semantic_cnn_checkpoint_metadata.json"), ckpts)

    ds_stats = dataset_stats(args.dataset_root)
    jdump(os.path.join(args.out_dir, "semantic_cnn_native_dataset_target_stats.json"), ds_stats)

    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    torch.set_num_threads(max(1, min(8, os.cpu_count() or 1)))
    dev_cache = load_split_cache(args.dataset_root, "dev")
    comparison = []
    best_metrics = None
    for name in ["best_dev", "latest", "final"]:
        pred_csv = None
        prefix = None
        if name == "best_dev":
            pred_csv = os.path.join(args.out_dir, "semantic_cnn_best_dev_pred_vs_target.csv")
            prefix = os.path.join(args.out_dir, "semantic_cnn_best_dev")
        metrics, _, _ = evaluate_checkpoint(ckpt_paths[name], dev_cache, device, "dev", pred_csv, prefix)
        jdump(os.path.join(args.out_dir, f"semantic_cnn_{name}_eval_metrics.json"), metrics)
        if name == "best_dev":
            best_metrics = metrics
        collapse = metrics["collapse_checks"]
        comparison.append({
            "checkpoint": name,
            "epoch": metrics["checkpoint_metadata"]["epoch"],
            "metadata_dev_loss": metrics["checkpoint_metadata"]["dev_loss"],
            "computed_dev_mse": metrics["mse"],
            "linear_x_rmse": metrics["linear_x"]["rmse"],
            "angular_z_rmse": metrics["angular_z"]["rmse"],
            "pred_std_linear_x": collapse["pred_std"]["linear_x"],
            "pred_std_angular_z": collapse["pred_std"]["angular_z"],
            "suspected_collapse": bool(collapse["linear_x_near_constant"] or collapse["angular_z_near_constant"] or collapse["all_near_zero"]),
        })
    write_comparison_csv(os.path.join(args.out_dir, "semantic_cnn_checkpoint_dev_comparison.csv"), comparison)

    test_cache = load_split_cache(args.dataset_root, "test")
    test_pred_csv = os.path.join(args.out_dir, "semantic_cnn_best_dev_test_pred_vs_target.csv")
    test_prefix = os.path.join(args.out_dir, "semantic_cnn_best_dev_test")
    best_dev_test_metrics, _, _ = evaluate_checkpoint(
        ckpt_paths["best_dev"], test_cache, device, "test", test_pred_csv, test_prefix
    )
    jdump(
        os.path.join(args.out_dir, "semantic_cnn_best_dev_test_eval_metrics.json"),
        best_dev_test_metrics,
    )

    out = {
        "device": str(device),
        "training_process_running": False,
        "log": log,
        "checkpoint_presence": checkpoint_presence(),
        "checkpoints": ckpts,
        "latest_final_model_tensors_equal": model_tensors_equal(ckpt_paths["latest"], ckpt_paths["final"]),
        "dataset_stats": ds_stats,
        "best_dev_metrics": best_metrics,
        "best_dev_test_metrics": best_dev_test_metrics,
        "checkpoint_eval_comparison": comparison,
    }
    jdump(os.path.join(args.out_dir, "semantic_cnn_full_check_summary.json"), out)
    write_reports(out)
    print(json.dumps({
        "out_dir": args.out_dir,
        "log": log_path,
        "epochs": len(log["rows"]),
        "last_epoch": log["last_epoch"],
        "best_dev_epoch": ckpts["best_dev"]["epoch"],
        "best_dev_loss": ckpts["best_dev"]["dev_loss"],
        "best_dev_mse": best_metrics["mse"],
        "best_dev_test_mse": best_dev_test_metrics["mse"],
        "report": os.path.join(args.out_dir, "semantic_cnn_full_check_report.md"),
    }, indent=2))


if __name__ == "__main__":
    main()
