#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--fps, --goal-x, --goal-y, --map-yaml, --max-frames, --output-mp4, --trajectory-csv
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：NEAREST, PIPE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_closed_loop_trajectory_video.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-18 00:58:08.047245575 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.914035757 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_closed_loop_trajectory_video.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜render_closed_loop_trajectory_video.py】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Render an MP4 from a closed_loop_demo_recorder CSV trajectory."""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory-csv", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--output-mp4", type=Path, required=True)
    parser.add_argument("--goal-x", type=float, default=16.0)
    parser.add_argument("--goal-y", type=float, default=16.0)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--max-frames", type=int, default=180)
    return parser.parse_args()


def font(size):
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    return ImageFont.truetype(path, size) if Path(path).is_file() else ImageFont.load_default()


def load_map(path: Path):
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    image = Image.open(image_path).convert("RGB")
    origin = metadata["origin"]
    return image, float(metadata["resolution"]), float(origin[0]), float(origin[1])


def main() -> None:
    args = arguments()
    if args.output_mp4.exists():
        raise FileExistsError(f"refusing to overwrite existing video: {args.output_mp4}")
    with args.trajectory_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise RuntimeError("trajectory needs at least two recorded poses")
    map_image, resolution, origin_x, origin_y = load_map(args.map_yaml)
    scale = 2
    map_image = map_image.resize((map_image.width * scale, map_image.height * scale), Image.Resampling.NEAREST)
    width, height = map_image.size
    positions = [(float(row["x"]), float(row["y"])) for row in rows]
    stride = max(1, math.ceil(len(rows) / max(1, args.max_frames)))
    frame_indices = list(range(0, len(rows), stride))
    if frame_indices[-1] != len(rows) - 1:
        frame_indices.append(len(rows) - 1)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    args.output_mp4.parent.mkdir(parents=True, exist_ok=True)
    command = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height + 80}", "-r", str(args.fps), "-i", "-", "-an", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", str(args.output_mp4)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    title_font, label_font = font(26), font(18)

    def pixel(x, y):
        return (
            (x - origin_x) / resolution * scale,
            80 + (map_image.height - 1) - (y - origin_y) / resolution * scale,
        )

    try:
        for frame_number, index in enumerate(frame_indices, start=1):
            canvas = Image.new("RGB", (width, height + 80), (15, 18, 23))
            canvas.paste(map_image, (0, 80))
            draw = ImageDraw.Draw(canvas)
            draw.text((16, 18), "SemanticCNN fixed-dual closed-loop Gazebo demo", font=title_font, fill=(245, 245, 245))
            row = rows[index]
            draw.text((16, 49), f"start (2.0, 2.0)   goal ({args.goal_x:.1f}, {args.goal_y:.1f})   current ({float(row['x']):.2f}, {float(row['y']):.2f})   goal distance {float(row['goal_distance']):.2f} m", font=label_font, fill=(205, 215, 225))
            trace = [pixel(x, y) for x, y in positions[:index + 1]]
            if len(trace) > 1:
                draw.line(trace, fill=(30, 130, 255), width=4)
            sx, sy = pixel(2.0, 2.0)
            gx, gy = pixel(args.goal_x, args.goal_y)
            cx, cy = trace[-1]
            draw.ellipse((sx - 8, sy - 8, sx + 8, sy + 8), fill=(60, 220, 110), outline=(0, 0, 0), width=2)
            draw.ellipse((gx - 9, gy - 9, gx + 9, gy + 9), fill=(240, 70, 70), outline=(0, 0, 0), width=2)
            draw.ellipse((cx - 9, cy - 9, cx + 9, cy + 9), fill=(255, 215, 40), outline=(0, 0, 0), width=2)
            draw.text((sx + 12, sy - 10), "start", font=label_font, fill=(30, 110, 50))
            draw.text((gx + 12, gy - 10), "goal", font=label_font, fill=(180, 30, 30))
            assert process.stdin is not None
            process.stdin.write(canvas.tobytes())
    finally:
        if process.stdin is not None:
            process.stdin.close()
    stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
    if process.wait() != 0:
        raise RuntimeError(stderr)
    print(f"wrote {args.output_mp4} with {len(frame_indices)} frames")


if __name__ == "__main__":
    main()
