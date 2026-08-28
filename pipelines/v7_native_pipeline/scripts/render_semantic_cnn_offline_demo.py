#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--batch-size, --dataset-root, --device, --display-range, --fps, --max-frames, --model, --model-code, --output-dir, --session, --start-frame, --stride
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, TXT
# 可能使用的关键环境变量：PIPE, SEMANTIC_COLORS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 23:59:08.281109352 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.915035776 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py（导入其函数、类或模型）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜render_semantic_cnn_offline_demo.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Render an offline, non-actuating SemanticCNN replay video from test windows.

This intentionally reuses the training snapshot's NavDataset so the demo
uses the exact fixed-dual 10-frame window, virtual-angle pooling, and input
normalization contract of the selected checkpoint.  It never starts ROS or
publishes a velocity command.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont


SEMANTIC_COLORS = {
    -1: (90, 90, 90),
    0: (155, 155, 155),
    1: (80, 180, 255),
    2: (255, 190, 70),
    3: (185, 100, 230),
    4: (90, 215, 150),
    5: (225, 225, 225),
    6: (240, 80, 80),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-code", type=Path, required=True,
                        help="SemanticCNN model_code_scripts snapshot directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--session", default="",
                        help="Optional training-session directory name; default is all test windows")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="0 means all selected legal test windows")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="Skip this many selected legal test windows before rendering")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--batch-size", type=int, default=32,
                        help="Inference batch size; rendering remains frame-by-frame")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--display-range", type=float, default=8.0,
                        help="Front-view range in meters")
    return parser.parse_args()


def require_path(path: Path, description: str) -> None:
    if not path.exists():
        raise FileNotFoundError(f"{description} does not exist: {path}")


def load_training_module(model_code: Path):
    model_file = model_code / "model.py"
    require_path(model_file, "model snapshot")
    spec = importlib.util.spec_from_file_location("semantic_cnn_demo_model", model_file)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import model snapshot: {model_file}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_model(module, model_path: Path, device: str):
    model = module.SemanticCNN(module.Bottleneck, [2, 1, 1])
    checkpoint = torch.load(model_path, map_location=device)
    if "model" not in checkpoint:
        raise KeyError(f"checkpoint has no 'model' state dict: {model_path}")
    model.load_state_dict(checkpoint["model"])
    model.to(device)
    model.eval()
    return model


def get_font(size: int):
    for font_path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if os.path.isfile(font_path):
            return ImageFont.truetype(font_path, size)
    return ImageFont.load_default()


def draw_text(draw: ImageDraw.ImageDraw, xy, text: str, font, fill=(235, 235, 235)) -> None:
    draw.text(xy, text, font=font, fill=fill)


def draw_scan_panel(draw, box, ranges, angles, semantic, valid_mask, display_range, font):
    left, top, right, bottom = box
    draw.rectangle(box, fill=(20, 25, 30), outline=(95, 105, 115), width=2)
    draw_text(draw, (left + 12, top + 10), "Latest fixed-dual virtual scan (front 180 deg)", font)
    plot_top = top + 45
    plot_bottom = bottom - 25
    center_x = (left + right) // 2
    sensor_y = plot_bottom
    scale = min((right - left - 50) / (2 * display_range), (plot_bottom - plot_top - 20) / display_range)

    for meters in range(1, int(display_range) + 1):
        radius = meters * scale
        draw.arc((center_x - radius, sensor_y - radius, center_x + radius, sensor_y + radius),
                 180, 360, fill=(55, 65, 75), width=1)
        draw_text(draw, (center_x + 4, int(sensor_y - radius) - 12), f"{meters}m", font, (125, 135, 145))
    draw.line((left + 18, sensor_y, right - 18, sensor_y), fill=(55, 65, 75), width=1)
    draw.line((center_x, plot_top, center_x, sensor_y), fill=(55, 65, 75), width=1)
    draw.ellipse((center_x - 5, sensor_y - 5, center_x + 5, sensor_y + 5), fill=(255, 255, 255))

    ranges = np.asarray(ranges).reshape(-1)
    angles = np.asarray(angles).reshape(-1)
    semantic = np.asarray(semantic).reshape(-1)
    valid_mask = np.asarray(valid_mask, dtype=bool).reshape(-1)
    keep = (
        valid_mask & np.isfinite(ranges) & np.isfinite(angles)
        & (ranges > 0.0) & (ranges <= display_range)
        & (angles >= -math.pi / 2) & (angles < math.pi / 2)
    )
    for distance, angle, label in zip(ranges[keep], angles[keep], semantic[keep]):
        x = center_x + float(distance * math.sin(angle) * scale)
        y = sensor_y - float(distance * math.cos(angle) * scale)
        color = SEMANTIC_COLORS.get(int(label), (255, 0, 255))
        draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=color)

    legend = [(1, "Chair"), (2, "Pillar"), (3, "Sofa"), (4, "Table"), (5, "Wall"), (6, "Person")]
    legend_x = left + 14
    legend_y = bottom - 18
    for label, name in legend:
        color = SEMANTIC_COLORS[label]
        draw.rectangle((legend_x, legend_y, legend_x + 10, legend_y + 10), fill=color)
        draw_text(draw, (legend_x + 14, legend_y - 3), name, font, (205, 205, 205))
        legend_x += 82


def draw_history(draw, box, targets, predictions, title, component, scale, font):
    left, top, right, bottom = box
    draw.rectangle(box, fill=(20, 25, 30), outline=(95, 105, 115), width=2)
    draw_text(draw, (left + 12, top + 9), title, font)
    plot = (left + 36, top + 36, right - 14, bottom - 25)
    pl, pt, pr, pb = plot
    mid = (pt + pb) // 2
    draw.line((pl, mid, pr, mid), fill=(85, 95, 105), width=1)
    draw_text(draw, (pl, pt - 1), f"+{scale:.2f}", font, (145, 145, 145))
    draw_text(draw, (pl, pb - 13), f"-{scale:.2f}", font, (145, 145, 145))
    if len(targets) < 2:
        return
    start = max(0, len(targets) - 120)
    target_values = np.asarray(targets[start:])[:, component]
    prediction_values = np.asarray(predictions[start:])[:, component]
    width = max(1, pr - pl)

    def point(index, value, count):
        x = pl + index * width / max(1, count - 1)
        y = mid - float(np.clip(value, -scale, scale)) * (pb - pt) / (2 * scale)
        return int(x), int(y)

    count = len(target_values)
    target_points = [point(i, value, count) for i, value in enumerate(target_values)]
    pred_points = [point(i, value, count) for i, value in enumerate(prediction_values)]
    draw.line(target_points, fill=(90, 215, 150), width=2)
    draw.line(pred_points, fill=(255, 175, 65), width=2)
    draw_text(draw, (pr - 162, top + 9), "green=target  orange=pred", font, (185, 185, 185))


def render_frame(window, target, prediction, target_history, prediction_history, args, font, small_font):
    width, height = 1280, 760
    image = Image.new("RGB", (width, height), (12, 16, 21))
    draw = ImageDraw.Draw(image)
    root = Path(window["root"])
    frame_name = window["names"][-1]
    frame_index = int(Path(frame_name).stem)
    draw_text(draw, (22, 14), "SemanticCNN offline test replay — no ROS, no robot control", font)
    draw_text(draw, (22, 42), f"session: {root.name}   frame: {frame_index:07d}", small_font, (190, 200, 210))

    ranges = np.load(root / "virtual_ranges_lidar" / frame_name)
    angles = np.load(root / "virtual_angles_lidar" / frame_name)
    semantic = np.load(root / "semantic_label" / frame_name)
    valid_mask = np.load(root / "valid_mask_lidar" / frame_name)
    draw_scan_panel(draw, (20, 75, 790, 730), ranges, angles, semantic, valid_mask,
                    args.display_range, small_font)

    right_left = 815
    draw.rectangle((right_left, 75, 1260, 232), fill=(20, 25, 30), outline=(95, 105, 115), width=2)
    draw_text(draw, (right_left + 14, 88), "Control command at this 10-frame window", font)
    draw_text(draw, (right_left + 18, 128), f"linear x     target {target[0]:+0.3f} m/s    prediction {prediction[0]:+0.3f} m/s", small_font,
              (90, 215, 150))
    draw_text(draw, (right_left + 18, 158), f"angular z    target {target[1]:+0.3f} rad/s  prediction {prediction[1]:+0.3f} rad/s", small_font,
              (255, 175, 65))
    target_near_stop = float(np.linalg.norm(target)) <= 0.03
    pred_near_stop = float(np.linalg.norm(prediction)) <= 0.03
    state = "near-stop target" if target_near_stop else "moving target"
    if target_near_stop and not pred_near_stop:
        state += " — model still commands motion"
        color = (255, 100, 90)
    else:
        color = (210, 210, 210)
    draw_text(draw, (right_left + 18, 196), state, small_font, color)

    draw_history(draw, (815, 252, 1260, 475), target_history, prediction_history,
                 "linear x history (last 120 frames)", 0, 0.5, small_font)
    draw_history(draw, (815, 500, 1260, 730), target_history, prediction_history,
                 "angular z history (last 120 frames)", 1, 1.2, small_font)
    return image


def start_encoder(output_path: Path, fps: float):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to encode the MP4 but is not installed")
    command = [
        ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", "1280x760", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p", str(output_path),
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def main() -> None:
    args = parse_args()
    if args.stride < 1:
        raise ValueError("--stride must be >= 1")
    if args.max_frames < 0:
        raise ValueError("--max-frames must be >= 0")
    if args.start_frame < 0:
        raise ValueError("--start-frame must be >= 0")
    if args.fps <= 0:
        raise ValueError("--fps must be > 0")
    if args.batch_size < 1:
        raise ValueError("--batch-size must be >= 1")
    require_path(args.dataset_root / "dataset.txt", "dataset index")
    require_path(args.model, "model checkpoint")
    require_path(args.model_code, "model code directory")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(f"output directory must be new or empty: {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    module = load_training_module(args.model_code)
    dataset = module.NavDataset(str(args.dataset_root), "test", pooling_mode="global_virtual_angle_80")
    selected = [
        (index, window) for index, window in enumerate(dataset.windows)
        if not args.session or Path(window["root"]).name == args.session
    ]
    selected = selected[::args.stride]
    selected = selected[args.start_frame:]
    if args.max_frames:
        selected = selected[:args.max_frames]
    if not selected:
        raise RuntimeError("no legal test windows matched the requested session/limits")

    device = torch.device(args.device)
    model = load_model(module, args.model, str(device))
    records = []
    with torch.no_grad():
        for start in range(0, len(selected), args.batch_size):
            batch = selected[start:start + args.batch_size]
            data_items = [dataset[index] for index, _ in batch]
            prediction_batch = model(
                torch.stack([item["scan_map"] for item in data_items]).to(device),
                torch.stack([item["semantic_map"] for item in data_items]).to(device),
                torch.stack([item["sub_goal"] for item in data_items]).to(device),
            ).cpu().numpy().astype(np.float32)
            for (_, window), item, prediction in zip(batch, data_items, prediction_batch):
                records.append((window, item["target"].numpy().astype(np.float32), prediction))
            print(f"inferred {len(records)}/{len(selected)} frames", flush=True)

    output_video = args.output_dir / "semantic_cnn_test_replay.mp4"
    encoder = start_encoder(output_video, args.fps)
    font = get_font(22)
    small_font = get_font(15)
    targets, predictions = [], []
    try:
        for frame_no, (window, target, prediction) in enumerate(records, start=1):
            targets.append(target)
            predictions.append(prediction)
            image = render_frame(window, target, prediction, targets, predictions, args, font, small_font)
            assert encoder.stdin is not None
            encoder.stdin.write(image.tobytes())
            if frame_no % 100 == 0 or frame_no == len(records):
                print(f"rendered {frame_no}/{len(records)} frames", flush=True)
    finally:
        if encoder.stdin is not None:
            encoder.stdin.close()
    stderr = encoder.stderr.read().decode("utf-8", errors="replace") if encoder.stderr else ""
    if encoder.wait() != 0:
        raise RuntimeError(f"ffmpeg failed while writing {output_video}: {stderr}")

    target_array = np.asarray(targets)
    prediction_array = np.asarray(predictions)
    summary = {
        "purpose": "offline fixed-dual test replay; no ROS or actuation",
        "dataset_root": str(args.dataset_root.resolve()),
        "model": str(args.model.resolve()),
        "model_code": str(args.model_code.resolve()),
        "session_filter": args.session or None,
        "test_windows_rendered": int(len(selected)),
        "start_frame": args.start_frame,
        "stride": args.stride,
        "fps": args.fps,
        "video": str(output_video.resolve()),
        "mse_rendered_subset": float(np.mean((prediction_array - target_array) ** 2)),
        "rmse_rendered_subset": float(np.sqrt(np.mean((prediction_array - target_array) ** 2))),
        "mae_rendered_subset": float(np.mean(np.abs(prediction_array - target_array))),
    }
    summary_path = args.output_dir / "demo_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
