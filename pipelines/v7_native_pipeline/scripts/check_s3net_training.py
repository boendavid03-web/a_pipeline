#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：--dataset-root, --device, --log, --model-code, --model-dir, --out-dir, --periodic-max, --stats-json
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：CSV, JSON, PNG, PT, TXT
# 可能使用的关键环境变量：BAD_RE, BEST_RE, CLASS_NAMES, CUDA, DATASET_ROOT, DEFAULT_LABEL_NAMES, DEV_RE, EPOCH_END_RE, EPOCH_STEP_RE, IGNORECASE, IGNORE_LABEL, LOG_DIR, LOG_RE, MODEL_DIR, NUM_CLASSES, NUM_INPUT_CHANNELS, OUT_DIR, PERIODIC_MAX, PROJECT_ROOT, RUN_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_training.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.912035720 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_s3net_training.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_s3net_training.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Check S3-Net training result: parse log, inspect checkpoints, eval dev, generate reports."""

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
from tqdm import tqdm
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
RUN_ROOT = os.environ.get("RUN_ROOT") or str(PROJECT_ROOT / "experiments" / "legacy_runs")
DATASET_ROOT = os.path.join(RUN_ROOT, "datasets", "semantic2d_native_lidar")
MODEL_DIR = os.path.join(RUN_ROOT, "training", "s3net")
LOG_DIR = os.path.join(RUN_ROOT, "logs")
SCRIPT_DIR = os.path.join(RUN_ROOT, "training", "work", "s3_net_v7", "scripts")
OUT_DIR = os.path.join(MODEL_DIR, "eval_reports")
PERIODIC_MAX = 300

LOG_RE = re.compile(r"09_train_s3net_native_stats_.*\.log$")
EPOCH_STEP_RE = re.compile(r"Epoch \[(\d+)/(\d+)\], Step")
EPOCH_END_RE = re.compile(r"Epoch \[(\d+)/(\d+)\]$")
TRAIN_RE = re.compile(r"Train set: Average loss: ([0-9.eE+-]+)")
DEV_RE = re.compile(r"Validation set: Average loss: ([0-9.eE+-]+)")
BEST_RE = re.compile(r"Best dev loss: ([0-9.eE+-]+)")
SAVED_BEST_RE = re.compile(r"Saved best_dev: (yes|no)")
SAVED_PERIODIC_RE = re.compile(r"Saved periodic checkpoint: (.+)")
BAD_RE = re.compile(
    r"Traceback|CUDA out of memory|killed|exception|error|(^|[^a-z])nan([^a-z]|$)|(^|[^a-z])inf([^a-z]|$)",
    re.IGNORECASE,
)

# S3-Net class names. Dynamic runs load this mapping from dataset_root.
DEFAULT_LABEL_NAMES = [
    "_background_", "Chair", "Door", "Elevator", "Person", "Pillar",
    "Sofa", "Table", "Trash bin", "Wall",
]
CLASS_NAMES = list(DEFAULT_LABEL_NAMES)
NUM_CLASSES = len(CLASS_NAMES)
NUM_INPUT_CHANNELS = 3
IGNORE_LABEL = -1


def load_dataset_label_names(dataset_root):
    path = Path(dataset_root) / "label_names.txt"
    if path.exists():
        names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
        names = [name for name in names if name]
        if len(names) >= 2 and names[0] == "_background_":
            return names
        raise ValueError(f"invalid label names file: {path}")
    return list(DEFAULT_LABEL_NAMES)


def jdump(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
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
        raise FileNotFoundError("no 09_train_s3net_native_stats_*.log found")
    return sorted(logs, reverse=True)[0][1]


def parse_log(path):
    """Parse training log for per-epoch train/dev loss."""
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
            m = EPOCH_END_RE.search(stripped)
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
    os.makedirs(os.path.dirname(path), exist_ok=True)
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
    plt.title("S3-Net native stats training loss")
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
        "stats_json": ckpt.get("stats_json"),
        "batch_size": ckpt.get("batch_size"),
        "num_epochs": ckpt.get("num_epochs"),
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


def import_s3net_model():
    sys.path.insert(0, SCRIPT_DIR)
    from model import IGNORE_LABEL as ign, S3Net, VaeTestDataset, set_seed, SEED1
    return ign, S3Net, VaeTestDataset, set_seed, SEED1


def load_model(path, device):
    _, S3Net, _, set_seed, SEED1 = import_s3net_model()
    set_seed(SEED1)
    ckpt = torch.load(path, map_location=device)
    num_classes = int(ckpt.get("num_output_channels", NUM_CLASSES))
    if num_classes != NUM_CLASSES:
        raise ValueError(
            f"checkpoint expects {num_classes} classes but dataset defines {NUM_CLASSES}"
        )
    model = S3Net(
        input_channels=int(ckpt.get("input_channels", NUM_INPUT_CHANNELS)),
        output_channels=num_classes,
        feature_mode=ckpt.get("feature_mode"),
    )
    state = ckpt.get("model_state_dict", ckpt.get("model"))
    state = {k.replace("module.", "", 1): v for k, v in state.items()}
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    return model, ckpt


def update_confusion(confusion, truth, pred, num_classes):
    mask = (truth != IGNORE_LABEL) & (truth >= 0) & (truth < num_classes)
    encoded = num_classes * truth[mask].astype(np.int64) + pred[mask].astype(np.int64)
    confusion += np.bincount(encoded, minlength=num_classes * num_classes).reshape(num_classes, num_classes)
    return int(truth.size - mask.sum())


def summarize_confusion(confusion):
    total = int(confusion.sum())
    correct = int(np.trace(confusion))
    accuracy = correct / total if total else 0.0
    ious = []
    for cls in range(confusion.shape[0]):
        tp = confusion[cls, cls]
        fp = confusion[:, cls].sum() - tp
        fn = confusion[cls, :].sum() - tp
        denom = tp + fp + fn
        ious.append(float(tp / denom) if denom else None)
    valid_ious = [v for v in ious if v is not None]
    return {
        "beam_accuracy": accuracy,
        "mean_iou_present_classes": float(np.mean(valid_ious)) if valid_ious else 0.0,
        "per_class_iou": ious,
        "confusion": confusion.tolist(),
    }


def evaluate_checkpoint(path, dataset_root, stats_json, device, batch_size=128):
    """Run dev eval on a single checkpoint, returning segmentation metrics."""
    _, S3Net, VaeTestDataset, set_seed, SEED1 = import_s3net_model()
    set_seed(SEED1)
    dataset = VaeTestDataset(dataset_root, "dev", stats_path=stats_json)
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=False, drop_last=False)

    checkpoint = torch.load(path, map_location=device)
    num_classes = int(checkpoint.get("num_output_channels", NUM_CLASSES))
    if num_classes != NUM_CLASSES:
        raise ValueError(
            f"checkpoint expects {num_classes} classes but dataset defines {NUM_CLASSES}"
        )
    model = S3Net(
        input_channels=int(checkpoint.get("input_channels", NUM_INPUT_CHANNELS)),
        output_channels=num_classes,
        feature_mode=checkpoint.get("feature_mode"),
    ).to(device)
    state = checkpoint.get("model_state_dict", checkpoint.get("model"))
    state = {k.replace("module.", "", 1): v for k, v in state.items()}
    model.load_state_dict(state)
    model.eval()

    confusion = np.zeros((num_classes, num_classes), dtype=np.int64)
    ignored_label_count = 0
    with torch.no_grad():
        for batch in tqdm(loader, total=len(loader), desc=f"eval {os.path.basename(path)}"):
            scans = batch["scan"].to(device)
            intensities = batch["intensity"].to(device)
            angles = batch["angle_incidence"].to(device)
            labels = batch["label"].cpu().numpy().astype(np.int64)

            _, semantic_channels, _ = model(scans, intensities, angles)
            preds = semantic_channels.argmax(dim=1).cpu().numpy().astype(np.int64)
            ignored_label_count += update_confusion(confusion, labels.reshape(-1), preds.reshape(-1), num_classes)

    report = summarize_confusion(confusion)
    report.update({
        "model_path": str(os.path.abspath(path)),
        "dataset_root": str(os.path.abspath(dataset_root)),
        "split": "dev",
        "samples": len(dataset),
        "ignore_label": IGNORE_LABEL,
        "ignored_label_count": ignored_label_count,
        "label_names": CLASS_NAMES,
        "num_classes": num_classes,
        "checkpoint_metadata": {
            "epoch": checkpoint.get("epoch"),
            "dev_loss": checkpoint.get("dev_loss"),
            "train_loss": checkpoint.get("train_loss"),
            "best_dev_loss": checkpoint.get("best_dev_loss"),
            "timestamp": checkpoint.get("timestamp"),
        },
    })
    return report


def _list_periodic_checkpoints(model_dir):
    """Glob s3net_native_stats_epoch_*.pth and return sorted list of (epoch, filename, path)."""
    import glob
    pattern = os.path.join(model_dir, "s3net_native_stats_epoch_*.pth")
    found = []
    for path in sorted(glob.glob(pattern)):
        base = os.path.basename(path)
        # extract epoch number from s3net_native_stats_epoch_NNNN.pth
        m = re.match(r"s3net_native_stats_epoch_(\d+)\.pth", base)
        if m:
            found.append((int(m.group(1)), base, path))
    return sorted(found, key=lambda x: x[0])


def checkpoint_presence(model_dir):
    present = {}
    for name in ["latest", "best_dev", "final"]:
        filename = f"s3net_native_stats_{name}.pth"
        present[filename] = os.path.exists(os.path.join(model_dir, filename))
    periodic = {}
    for epoch, filename, path in _list_periodic_checkpoints(model_dir):
        periodic[filename] = True
    return {"named": present, "periodic": periodic}


def write_comparison_csv(path, rows):
    fields = [
        "checkpoint", "epoch", "metadata_dev_loss", "beam_accuracy",
        "mean_iou_present_classes", "samples", "ignored_label_count",
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_per_class_iou(per_class_iou, path):
    present = [(i, v) for i, v in enumerate(per_class_iou) if v is not None]
    if not present:
        return
    indices = [p[0] for p in present]
    values = [p[1] for p in present]
    labels = [CLASS_NAMES[i] for i in indices]
    plt.figure(figsize=(10, 6))
    bars = plt.bar(range(len(labels)), values, color="steelblue")
    plt.axhline(y=np.mean(values), color="red", linestyle="--", linewidth=1.0, label=f"mean IoU: {np.mean(values):.4f}")
    plt.xticks(range(len(labels)), labels, rotation=45, ha="right")
    plt.ylabel("IoU")
    plt.title("S3-Net best_dev per-class IoU")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.25, axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_confusion_matrix(confusion, path):
    confusion = np.array(confusion, dtype=np.float64)
    # Normalize rows
    row_sums = confusion.sum(axis=1, keepdims=True)
    row_sums[row_sums == 0] = 1.0
    cm_norm = confusion / row_sums

    fig, ax = plt.subplots(figsize=(10, 8))
    im = ax.imshow(cm_norm, cmap="Blues", vmin=0, vmax=1)
    plt.colorbar(im, ax=ax)
    ax.set_xticks(range(NUM_CLASSES))
    ax.set_yticks(range(NUM_CLASSES))
    ax.set_xticklabels(CLASS_NAMES, rotation=45, ha="right", fontsize=9)
    ax.set_yticklabels(CLASS_NAMES, fontsize=9)
    ax.set_xlabel("Predicted")
    ax.set_ylabel("True")
    ax.set_title("S3-Net best_dev confusion matrix (row-normalized)")
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


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


def write_reports(out, eval_dir):
    full_path = os.path.join(eval_dir, "s3net_full_check_report.md")
    log = out["log"]
    rows = log["rows"]
    best_row = min(rows, key=lambda r: r["dev_loss"]) if rows else None
    final_row = rows[-1] if rows else None
    comparison = out["checkpoint_eval_comparison"]
    best_metrics = out["best_dev_metrics"]
    train_first = rows[0]["train_loss"] if rows else None
    train_last = rows[-1]["train_loss"] if rows else None
    dev_first = rows[0]["dev_loss"] if rows else None
    dev_last = rows[-1]["dev_loss"] if rows else None
    periodic_missing = [k for k, v in out["checkpoint_presence"]["periodic"].items() if not v]
    named_missing = [k for k, v in out["checkpoint_presence"]["named"].items() if not v]
    periodic_found = [k for k, v in out["checkpoint_presence"]["periodic"].items() if v]
    periodic_label = ", ".join(periodic_found) if periodic_found else "none"
    final_worse_pct = None
    if best_row and final_row:
        final_worse_pct = 100.0 * (final_row["dev_loss"] - best_row["dev_loss"]) / best_row["dev_loss"]

    cmp_rows = []
    for item in comparison:
        cmp_rows.append({
            "checkpoint": item["checkpoint"],
            "epoch": item["epoch"],
            "metadata_dev_loss": fmt(item["metadata_dev_loss"]),
            "beam_accuracy": fmt(item["beam_accuracy"]),
            "mean_iou": fmt(item["mean_iou_present_classes"]),
            "samples": item["samples"],
            "ignored": item["ignored_label_count"],
        })

    # Per-class IoU table
    iou_rows = []
    if best_metrics:
        for cls_idx, iou in enumerate(best_metrics.get("per_class_iou", [])):
            iou_rows.append({"class": CLASS_NAMES[cls_idx], "iou": fmt(iou)})

    full = [
        "# S3-Net Native Stats Full Check",
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
        f"- Periodic checkpoints found: {periodic_label}",
        f"- latest and final model tensors equal: {out['latest_final_model_tensors_equal']}",
        f"- best_dev epoch: {out['checkpoints']['best_dev']['epoch']}, best_dev_loss: {fmt(out['checkpoints']['best_dev']['best_dev_loss'])}",
        f"- final epoch: {out['checkpoints']['final']['epoch']}, final dev_loss: {fmt(out['checkpoints']['final']['dev_loss'])}",
        f"- final vs best_dev dev loss: {fmt(final_worse_pct, 4)}% worse" if final_worse_pct is not None else "- final vs best_dev dev loss: n/a",
        "",
        "Checkpoint metadata files:",
        f"- `{os.path.join(eval_dir, 's3net_checkpoint_metadata.json')}`",
        "",
        "## 3. Loss Trend",
        f"- Train loss: {fmt(train_first)} -> {fmt(train_last)}",
        f"- Dev loss: {fmt(dev_first)} -> {fmt(dev_last)}",
        f"- Best dev was {'late' if best_row and best_row['epoch'] > 0.66 * best_row['num_epochs'] else 'not late'}: epoch {best_row['epoch'] if best_row else 'n/a'} / {best_row['num_epochs'] if best_row else 'n/a'}",
        f"- Loss CSV: `{os.path.join(eval_dir, 's3net_training_loss_curve.csv')}`",
        f"- Loss plot: `{os.path.join(eval_dir, 's3net_training_loss_curve.png')}`",
        "",
        "## 4. Best Dev Segmentation Metrics",
        f"- Beam accuracy: {fmt(best_metrics['beam_accuracy'])}",
        f"- Mean IoU (present classes): {fmt(best_metrics['mean_iou_present_classes'])}",
        f"- Samples: {best_metrics['samples']}",
        f"- Ignored label count: {best_metrics['ignored_label_count']}",
        "",
        "### Per-class IoU",
        markdown_table(iou_rows, ["class", "iou"]),
        "",
        f"- Per-class IoU plot: `{os.path.join(eval_dir, 's3net_per_class_iou_best_dev.png')}`",
        f"- Confusion matrix plot: `{os.path.join(eval_dir, 's3net_confusion_best_dev.png')}`",
        "",
        "## 5. Checkpoint Dev Comparison",
        markdown_table(cmp_rows, [
            "checkpoint", "epoch", "metadata_dev_loss", "beam_accuracy",
            "mean_iou", "samples", "ignored",
        ]),
        "",
        "## 6. How to Choose the Model",
        '- "best" is chosen by **minimum dev loss**, same rule as SemanticCNN.',
        "- mIoU is reported for information but is NOT used to select best.",
        "- **Deploy / downstream experiments**: default to `s3net_native_stats_best_dev.pth`.",
        "- `s3net_native_stats_final.pth` may have higher dev loss due to overfitting.",
        "",
        "## 7. Recommendation",
        "- Use `s3net_native_stats_best_dev.pth` for downstream tests and deployment.",
        "- If best_dev and final dev loss are close, final can also be acceptable.",
        "- Consider early stopping if dev loss rises consistently after best_dev.",
    ]
    with open(full_path, "w") as f:
        f.write("\n".join(full) + "\n")


def main():
    global DATASET_ROOT, MODEL_DIR, SCRIPT_DIR, OUT_DIR, PERIODIC_MAX, CLASS_NAMES, NUM_CLASSES
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default=OUT_DIR)
    parser.add_argument("--dataset-root", default=DATASET_ROOT)
    parser.add_argument("--model-dir", default=MODEL_DIR)
    parser.add_argument("--model-code", default=SCRIPT_DIR)
    parser.add_argument("--stats-json", default="")
    parser.add_argument("--log", default="")
    parser.add_argument("--device", default="auto", choices=["cpu", "cuda", "auto"])
    parser.add_argument("--periodic-max", type=int, default=PERIODIC_MAX)
    args = parser.parse_args()
    DATASET_ROOT = args.dataset_root
    MODEL_DIR = args.model_dir
    SCRIPT_DIR = args.model_code
    OUT_DIR = args.out_dir
    PERIODIC_MAX = args.periodic_max
    CLASS_NAMES = load_dataset_label_names(args.dataset_root)
    NUM_CLASSES = len(CLASS_NAMES)
    stats_json = args.stats_json
    os.makedirs(args.out_dir, exist_ok=True)

    # ---- Parse log ----
    log_path = args.log or latest_log()
    log = parse_log(log_path)
    write_loss_csv(log["rows"], os.path.join(args.out_dir, "s3net_training_loss_curve.csv"))
    plot_loss(log["rows"], os.path.join(args.out_dir, "s3net_training_loss_curve.png"))

    # ---- Inspect checkpoints ----
    ckpt_paths = {
        "latest": os.path.join(args.model_dir, "s3net_native_stats_latest.pth"),
        "best_dev": os.path.join(args.model_dir, "s3net_native_stats_best_dev.pth"),
        "final": os.path.join(args.model_dir, "s3net_native_stats_final.pth"),
    }
    ckpts = {name: checkpoint_summary(path) for name, path in ckpt_paths.items()}
    jdump(os.path.join(args.out_dir, "s3net_checkpoint_metadata.json"), ckpts)

    # ---- Device ----
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    torch.set_num_threads(max(1, min(8, os.cpu_count() or 1)))

    # ---- Evaluate named checkpoints ----
    comparison = []
    best_metrics = None
    for name in ["best_dev", "latest", "final"]:
        path = ckpt_paths[name]
        if not os.path.exists(path):
            print(f"WARNING: checkpoint not found: {path}", file=sys.stderr)
            continue
        print(f"\n===== Evaluating {name} checkpoint =====", flush=True)
        metrics = evaluate_checkpoint(path, args.dataset_root, stats_json, device, batch_size=128)
        jdump(os.path.join(args.out_dir, f"s3net_{name}_eval_metrics.json"), metrics)
        if name == "best_dev":
            best_metrics = metrics
        comparison.append({
            "checkpoint": name,
            "epoch": metrics["checkpoint_metadata"]["epoch"],
            "metadata_dev_loss": metrics["checkpoint_metadata"]["dev_loss"],
            "beam_accuracy": metrics["beam_accuracy"],
            "mean_iou_present_classes": metrics["mean_iou_present_classes"],
            "samples": metrics["samples"],
            "ignored_label_count": metrics["ignored_label_count"],
        })

    # ---- Evaluate periodic checkpoints (glob actual files, not hardcoded step) ----
    for epoch, filename, path in _list_periodic_checkpoints(args.model_dir):
        print(f"\n===== Evaluating periodic checkpoint epoch {epoch} =====", flush=True)
        try:
            metrics = evaluate_checkpoint(path, args.dataset_root, stats_json, device, batch_size=128)
            jdump(os.path.join(args.out_dir, f"s3net_epoch_{epoch:04d}_eval_metrics.json"), metrics)
            comparison.append({
                "checkpoint": f"epoch_{epoch:04d}",
                "epoch": metrics["checkpoint_metadata"]["epoch"],
                "metadata_dev_loss": metrics["checkpoint_metadata"]["dev_loss"],
                "beam_accuracy": metrics["beam_accuracy"],
                "mean_iou_present_classes": metrics["mean_iou_present_classes"],
                "samples": metrics["samples"],
                "ignored_label_count": metrics["ignored_label_count"],
            })
        except Exception as e:
            print(f"WARNING: failed to evaluate {filename}: {e}", file=sys.stderr)

    write_comparison_csv(os.path.join(args.out_dir, "s3net_checkpoint_dev_comparison.csv"), comparison)

    # ---- Per-class IoU and confusion matrix plots for best_dev ----
    if best_metrics:
        if best_metrics.get("per_class_iou"):
            plot_per_class_iou(
                best_metrics["per_class_iou"],
                os.path.join(args.out_dir, "s3net_per_class_iou_best_dev.png"),
            )
        if best_metrics.get("confusion"):
            plot_confusion_matrix(
                best_metrics["confusion"],
                os.path.join(args.out_dir, "s3net_confusion_best_dev.png"),
            )

    # ---- Write reports ----
    out = {
        "device": str(device),
        "training_process_running": False,
        "log": log,
        "checkpoint_presence": checkpoint_presence(args.model_dir),
        "checkpoints": ckpts,
        "latest_final_model_tensors_equal": (
            model_tensors_equal(ckpt_paths["latest"], ckpt_paths["final"])
            if os.path.exists(ckpt_paths["latest"]) and os.path.exists(ckpt_paths["final"])
            else False
        ),
        "best_dev_metrics": best_metrics,
        "checkpoint_eval_comparison": comparison,
    }
    jdump(os.path.join(args.out_dir, "s3net_full_check_summary.json"), out)
    write_reports(out, args.out_dir)

    print(json.dumps({
        "out_dir": args.out_dir,
        "log": log_path,
        "epochs": len(log["rows"]),
        "last_epoch": log["last_epoch"],
        "best_dev_epoch": ckpts["best_dev"]["epoch"],
        "best_dev_loss": ckpts["best_dev"]["dev_loss"],
        "best_dev_miou": best_metrics["mean_iou_present_classes"] if best_metrics else None,
        "report": os.path.join(args.out_dir, "s3net_full_check_report.md"),
    }, indent=2))


if __name__ == "__main__":
    main()
