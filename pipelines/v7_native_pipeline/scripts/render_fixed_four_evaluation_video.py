#!/usr/bin/env python3
"""Render a synchronized fixed-four evaluation MP4 from real recorded data."""

from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import json
import math
import shutil
import struct
import subprocess
from array import array
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont


LIDAR_MAGIC = b"FFVLIDAR1\n"
LIDAR_RECORD = struct.Struct("<qqII8f")
MERGED_LIDAR_MAGIC = b"FFVMERGED1\n"
MERGED_LIDAR_RECORD = struct.Struct("<qI4f")
COLORS = {
    "cmd_linear": (36, 125, 255),
    "applied_linear": (255, 174, 45),
    "actual_linear": (44, 190, 112),
    "cmd_angular": (183, 96, 255),
    "actual_angular": (245, 92, 92),
}
RESAMPLE_NEAREST = getattr(getattr(Image, "Resampling", Image), "NEAREST")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation-dir", type=Path, required=True)
    parser.add_argument("--output-mp4", type=Path)
    parser.add_argument("--capture-dir", type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--playback-rate", type=float, default=4.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--save-episode-screenshots", action="store_true")
    return parser.parse_args()


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def load_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(row: dict | None, key: str):
    if not row:
        return None
    text = row.get(key, "")
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def timed_rows(rows: list[dict], key: str = "simulation_time_sec"):
    accepted = []
    for row in rows:
        timestamp = number(row, key)
        if timestamp is not None:
            accepted.append((timestamp, row))
    accepted.sort(key=lambda item: item[0])
    return accepted


def causal(items, timestamp: float):
    if not items:
        return None
    index = bisect.bisect_right([item[0] for item in items], timestamp) - 1
    return items[index][1] if index >= 0 else None


def load_map(path: Path):
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    image = Image.open(image_path).convert("RGB")
    origin = metadata["origin"]
    return image, float(metadata["resolution"]), float(origin[0]), float(origin[1])


def load_navigation_events(path: Path):
    paths = []
    goals = []
    if not path.is_file():
        return paths, goals
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
                timestamp = float(event["simulation_time_sec"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                continue
            if (
                event.get("type") == "global_path"
                and event.get("points")
                and str(event.get("frame_id", "")).lstrip("/") == "map"
            ):
                paths.append((timestamp, event["points"]))
            elif event.get("type") == "accepted_goal" and event.get("goal"):
                goals.append((timestamp, event["goal"]))
    return paths, goals


def read_lidar_record(stream):
    raw = stream.read(LIDAR_RECORD.size)
    if not raw:
        return None
    if len(raw) != LIDAR_RECORD.size:
        raise RuntimeError("truncated dual lidar record header")
    values = LIDAR_RECORD.unpack(raw)
    time_01_ns, time_02_ns, count_01, count_02 = values[:4]
    byte_count_01 = count_01 * 4
    byte_count_02 = count_02 * 4
    raw_01 = stream.read(byte_count_01)
    raw_02 = stream.read(byte_count_02)
    if len(raw_01) != byte_count_01 or len(raw_02) != byte_count_02:
        raise RuntimeError("truncated dual lidar ranges")
    ranges_01 = array("f")
    ranges_02 = array("f")
    ranges_01.frombytes(raw_01)
    ranges_02.frombytes(raw_02)
    return {
        "time": min(time_01_ns, time_02_ns) * 1.0e-9,
        "time_01": time_01_ns * 1.0e-9,
        "time_02": time_02_ns * 1.0e-9,
        "ranges_01": ranges_01,
        "ranges_02": ranges_02,
        "layout_01": values[4:8],
        "layout_02": values[8:12],
    }


def select_lidar_samples(path: Path, target_times: list[float]):
    selected = [None] * len(target_times)
    if not path.is_file() or not target_times:
        return selected
    with gzip.open(path, "rb") as stream:
        if stream.read(len(LIDAR_MAGIC)) != LIDAR_MAGIC:
            raise RuntimeError(f"unsupported lidar capture format: {path}")
        latest = None
        pending = read_lidar_record(stream)
        for index, timestamp in enumerate(target_times):
            while pending is not None and pending["time"] <= timestamp:
                latest = pending
                pending = read_lidar_record(stream)
            selected[index] = latest
    return selected


def read_merged_lidar_record(stream):
    raw = stream.read(MERGED_LIDAR_RECORD.size)
    if not raw:
        return None
    if len(raw) != MERGED_LIDAR_RECORD.size:
        raise RuntimeError("truncated merged lidar record header")
    timestamp_ns, count, angle_min, increment, range_min, range_max = (
        MERGED_LIDAR_RECORD.unpack(raw)
    )
    byte_count = count * 4
    raw_ranges = stream.read(byte_count)
    if len(raw_ranges) != byte_count:
        raise RuntimeError("truncated merged lidar ranges")
    ranges = array("f")
    ranges.frombytes(raw_ranges)
    return {
        "time": timestamp_ns * 1.0e-9,
        "ranges": ranges,
        "layout": (angle_min, increment, range_min, range_max),
    }


def select_merged_lidar_samples(path: Path, target_times: list[float]):
    selected = [None] * len(target_times)
    if not path.is_file() or not target_times:
        return selected
    with gzip.open(path, "rb") as stream:
        if stream.read(len(MERGED_LIDAR_MAGIC)) != MERGED_LIDAR_MAGIC:
            raise RuntimeError(f"unsupported merged lidar capture format: {path}")
        latest = None
        pending = read_merged_lidar_record(stream)
        for index, timestamp in enumerate(target_times):
            while pending is not None and pending["time"] <= timestamp:
                latest = pending
                pending = read_merged_lidar_record(stream)
            selected[index] = latest
    return selected


def project_merged_lidar_to_map(sample, robot_pose_row):
    """Project one real base_link LaserScan into map coordinates."""
    if sample is None or robot_pose_row is None:
        return []
    robot_x = number(robot_pose_row, "x")
    robot_y = number(robot_pose_row, "y")
    robot_yaw = number(robot_pose_row, "yaw")
    if robot_x is None or robot_y is None or robot_yaw is None:
        return []
    angle_min, increment, range_min, range_max = sample["layout"]
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    points = []
    for index, measured_range in enumerate(sample["ranges"]):
        distance = float(measured_range)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * increment
        base_x = distance * math.cos(angle)
        base_y = distance * math.sin(angle)
        points.append(
            (
                robot_x + cos_yaw * base_x - sin_yaw * base_y,
                robot_y + sin_yaw * base_x + cos_yaw * base_y,
            )
        )
    return points


def pedestrian_snapshots(rows: list[dict]):
    snapshots = []
    current_time = None
    current = []
    for row in rows:
        timestamp = number(row, "simulation_time_sec")
        x, y = number(row, "x"), number(row, "y")
        if timestamp is None or x is None or y is None:
            continue
        if current_time is None or not math.isclose(timestamp, current_time, abs_tol=1.0e-9):
            if current:
                snapshots.append((current_time, current))
            current_time, current = timestamp, []
        current.append((x, y, row.get("pedestrian_id", "")))
    if current:
        snapshots.append((current_time, current))
    return snapshots


def build_frame_schedule(episodes: list[dict], fps: float, playback_rate: float, max_frames: int):
    frames = []
    step = playback_rate / fps
    for episode_index, episode in enumerate(episodes, start=1):
        start = float(episode["experiment"]["simulation_time_start"])
        end = float(episode["experiment"]["simulation_time_end"])
        timestamp = start
        while timestamp < end:
            frames.append((episode_index, timestamp))
            timestamp += step
        frames.append((episode_index, end))
    if max_frames > 0 and len(frames) > max_frames:
        boundary_indices = {
            index
            for index in range(len(frames))
            if index == len(frames) - 1 or frames[index + 1][0] != frames[index][0]
        }
        stride = (len(frames) - 1) / max(1, max_frames - 1)
        indices = set(round(index * stride) for index in range(max_frames)) | boundary_indices
        while len(indices) > max_frames:
            removable = sorted(indices - boundary_indices)
            if not removable:
                break
            indices.remove(removable[len(removable) // 2])
        indices = sorted(indices)
        frames = [frames[index] for index in indices]
    return frames


def plot_series(draw, box, series, current_time, title, unit, unavailable=()):
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=8, fill=(23, 28, 36), outline=(73, 82, 96), width=1)
    draw.text((left + 10, top + 7), f"{title} [{unit}]", font=font(16), fill=(232, 236, 242))
    plot_left, plot_top = left + 42, top + 34
    plot_right, plot_bottom = right - 10, bottom - 25
    visible_values = []
    for _, items in series:
        visible_values.extend(value for timestamp, value in items if timestamp <= current_time and value is not None)
    limit = max((abs(value) for value in visible_values), default=1.0)
    limit = max(limit, 0.2)
    start_time = min((items[0][0] for _, items in series if items), default=current_time)
    duration = max(current_time - start_time, 1.0)
    zero_y = (plot_top + plot_bottom) / 2
    draw.line((plot_left, zero_y, plot_right, zero_y), fill=(82, 90, 103), width=1)
    for label, items in series:
        color = COLORS[label]
        points = []
        for timestamp, value in items:
            if timestamp > current_time or value is None:
                continue
            x = plot_left + (timestamp - start_time) / duration * (plot_right - plot_left)
            y = zero_y - value / limit * (plot_bottom - plot_top) * 0.46
            points.append((x, y))
        if len(points) > 1:
            draw.line(points, fill=color, width=2)
    legend_x = left + 10
    legend_y = bottom - 20
    for label, _ in series:
        color = COLORS[label]
        text = {
            "cmd_linear": "commanded",
            "applied_linear": "applied",
            "actual_linear": "actual",
            "cmd_angular": "commanded",
            "actual_angular": "actual",
        }[label]
        if label in unavailable:
            text += " (unavailable)"
        draw.rectangle((legend_x, legend_y + 4, legend_x + 10, legend_y + 14), fill=color)
        draw.text((legend_x + 14, legend_y), text, font=font(12), fill=(200, 207, 217))
        legend_x += 170


def draw_lidar(draw, box, sample, slot: int):
    left, top, right, bottom = box
    color = (40, 150, 255) if slot == 1 else (255, 157, 54)
    draw.rounded_rectangle(box, radius=8, fill=(23, 28, 36), outline=color, width=2)
    draw.text((left + 10, top + 7), f"lidar slot {slot}", font=font(16), fill=color)
    if sample is None:
        draw.text((left + 18, (top + bottom) / 2), "unavailable", font=font(20), fill=(235, 100, 100))
        return
    ranges = sample[f"ranges_0{slot}"]
    angle_min, increment, range_min, range_max = sample[f"layout_0{slot}"]
    center_x = (left + right) / 2
    center_y = bottom - 18
    radius_px = min((right - left) * 0.46, (bottom - top) * 0.72)
    display_max = min(float(range_max), 12.0)
    points = []
    stride = max(1, len(ranges) // 600)
    for index in range(0, len(ranges), stride):
        distance = float(ranges[index])
        if not math.isfinite(distance) or distance < range_min or distance > display_max:
            continue
        angle = angle_min + index * increment
        scale = distance / display_max * radius_px
        points.append((center_x - math.sin(angle) * scale, center_y - math.cos(angle) * scale))
    draw.arc((center_x - radius_px, center_y - radius_px, center_x + radius_px, center_y + radius_px), 180, 360, fill=(83, 92, 106))
    for x, y in points:
        draw.point((x, y), fill=color)
    draw.ellipse((center_x - 4, center_y - 4, center_x + 4, center_y + 4), fill=(250, 250, 250))
    draw.text((left + 10, bottom - 18), f"t={sample[f'time_0{slot}']:.3f}s", font=font(11), fill=(170, 178, 190))


def main() -> None:
    args = arguments()
    if args.fps <= 0 or args.playback_rate <= 0:
        raise ValueError("fps and playback-rate must be positive")
    evaluation_dir = args.evaluation_dir.expanduser().resolve()
    session_path = evaluation_dir / "session_summary.json"
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if int(session.get("episode_count", 0)) != 4 or len(session.get("episodes", [])) != 4:
        raise RuntimeError("fixed-four video requires exactly four completed episodes")
    episodes = []
    for entry in session["episodes"]:
        episode_dir = evaluation_dir / entry["directory"]
        summary = json.loads((episode_dir / "episode_summary.json").read_text(encoding="utf-8"))
        episodes.append(summary)

    map_yaml = args.map_yaml
    if map_yaml is None:
        provenance = episodes[0].get("map_provenance", {})
        value = provenance.get("map_yaml_path")
        map_yaml = Path(value) if value else None
    if map_yaml is None or not map_yaml.is_file():
        raise FileNotFoundError("map YAML is required and was not found in episode provenance")
    map_image, resolution, origin_x, origin_y = load_map(map_yaml)

    video_dir = evaluation_dir / "video"
    output_mp4 = (args.output_mp4 or (video_dir / "evaluation_video.mp4")).expanduser().resolve()
    if output_mp4.exists() and not args.overwrite:
        raise FileExistsError(f"refusing to overwrite existing video: {output_mp4}")
    output_mp4.parent.mkdir(parents=True, exist_ok=True)
    capture_dir = (args.capture_dir or (video_dir / "sync")).expanduser().resolve()
    navigation_events = capture_dir / "navigation_events.jsonl"
    lidar_path = capture_dir / "dual_lidar.bin.gz"
    merged_lidar_path = capture_dir / "merged_lidar.bin.gz"
    capture_summary_path = capture_dir / "capture_summary.json"
    if not merged_lidar_path.is_file():
        raise FileNotFoundError(
            "merged_lidar.bin.gz is required for the map-frame LiDAR overlay"
        )
    if capture_summary_path.is_file():
        capture_summary = json.loads(capture_summary_path.read_text(encoding="utf-8"))
        merged_frame = str(capture_summary.get("frames", {}).get("scan_merged", ""))
        if merged_frame.lstrip("/") != "base_link":
            raise RuntimeError(
                f"map overlay requires /scan_merged in base_link, got {merged_frame!r}"
            )
    path_samples, _goal_samples = load_navigation_events(navigation_events)

    schedule = build_frame_schedule(episodes, args.fps, args.playback_rate, args.max_frames)
    lidar_samples = select_lidar_samples(lidar_path, [timestamp for _, timestamp in schedule])
    merged_lidar_samples = select_merged_lidar_samples(
        merged_lidar_path, [timestamp for _, timestamp in schedule]
    )
    if not any(sample is not None for sample in merged_lidar_samples):
        raise RuntimeError(
            "/scan_merged capture contains no sample at or before the video schedule"
        )
    episode_data = []
    for index in range(1, 5):
        directory = evaluation_dir / f"episode_{index:04d}"
        trajectory = timed_rows(load_csv(directory / "trajectory.csv"))
        commands = timed_rows(load_csv(directory / "commands.csv"))
        simulator = timed_rows(load_csv(directory / "simulator_actuation.csv"))
        decisions = timed_rows(load_csv(directory / "actuation_decisions.csv"))
        pedestrians = pedestrian_snapshots(load_csv(directory / "pedestrian_trace.csv"))
        episode_data.append(
            {"trajectory": trajectory, "commands": commands, "simulator": simulator,
             "decisions": decisions, "pedestrians": pedestrians}
        )

    ffmpeg = shutil.which("ffmpeg")
    process = None
    cv_writer = None
    if ffmpeg is not None:
        command = [
            ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-", "-an",
            "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", str(output_mp4),
        ]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        encoder = "ffmpeg/libx264"
    else:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("video encoding requires ffmpeg or OpenCV") from exc
        encoder = None
        for fourcc, label in (("avc1", "opencv/avc1 (H.264)"), ("mp4v", "opencv/mp4v")):
            cv_writer = cv2.VideoWriter(
                str(output_mp4),
                cv2.VideoWriter_fourcc(*fourcc),
                args.fps,
                (args.width, args.height),
            )
            if cv_writer.isOpened():
                encoder = label
                break
            cv_writer.release()
        if encoder is None:
            raise RuntimeError("OpenCV could not open an H.264 or MPEG-4 MP4 encoder")
    title_font, body_font, small_font = font(24), font(16), font(13)
    map_box = (16, 82, 1010, 832)
    map_left, map_top, map_right, map_bottom = map_box
    map_width, map_height = map_right - map_left, map_bottom - map_top
    scale = min(map_width / map_image.width, map_height / map_image.height)
    drawn_map = map_image.resize(
        (max(1, round(map_image.width * scale)), max(1, round(map_image.height * scale))),
        RESAMPLE_NEAREST,
    )
    paste_x = map_left + (map_width - drawn_map.width) // 2
    paste_y = map_top + (map_height - drawn_map.height) // 2

    def pixel(x, y):
        return (
            paste_x + (x - origin_x) / resolution * scale,
            paste_y + drawn_map.height - 1 - (y - origin_y) / resolution * scale,
        )

    episode_frame_counts = [0, 0, 0, 0]
    final_canvases = {}
    cumulative_before = [0.0]
    for summary in episodes[:-1]:
        cumulative_before.append(cumulative_before[-1] + float(summary["episode"]["navigation_time_sec"]))

    try:
        for frame_index, ((episode_number, timestamp), lidar, merged_lidar) in enumerate(
            zip(schedule, lidar_samples, merged_lidar_samples)
        ):
            summary = episodes[episode_number - 1]
            data = episode_data[episode_number - 1]
            trajectory_row = causal(data["trajectory"], timestamp)
            command_row = causal(data["commands"], timestamp)
            simulator_row = causal(data["simulator"], timestamp)
            decision_row = causal(data["decisions"], timestamp)
            pedestrians = causal(data["pedestrians"], timestamp) or []
            start_time = float(summary["experiment"]["simulation_time_start"])
            end_time = float(summary["experiment"]["simulation_time_end"])
            elapsed = max(0.0, timestamp - start_time)
            session_elapsed = cumulative_before[episode_number - 1] + elapsed
            goal = summary["experiment"]["accepted_goal"]
            method = summary["method"].get("name") or "unknown policy"
            scene = summary["experiment"].get("scene_id") or "scene id unavailable"
            canvas = Image.new("RGB", (args.width, args.height), (12, 15, 20))
            draw = ImageDraw.Draw(canvas)
            draw.text((18, 12), f"{method} fixed_four_goals evaluation", font=title_font, fill=(244, 247, 250))
            draw.text(
                (18, 46),
                f"episode {episode_number}/4   goal {episode_number}: ({goal[0]:.2f}, {goal[1]:.2f})   "
                f"simulation_time_sec={timestamp:.3f}   elapsed={elapsed:.2f}s   session elapsed={session_elapsed:.2f}s",
                font=body_font,
                fill=(190, 202, 217),
            )
            canvas.paste(drawn_map, (paste_x, paste_y))
            draw.rectangle(map_box, outline=(78, 89, 103), width=2)

            trail = []
            for row_time, row in data["trajectory"]:
                if row_time > timestamp:
                    break
                x, y = number(row, "x"), number(row, "y")
                if x is not None and y is not None:
                    trail.append(pixel(x, y))
            if len(trail) > 1:
                draw.line(trail, fill=(32, 135, 255), width=4)
            lidar_pose_row = causal(data["trajectory"], merged_lidar["time"]) if merged_lidar else None
            map_lidar_points = project_merged_lidar_to_map(merged_lidar, lidar_pose_row)
            for point_x, point_y in map_lidar_points:
                px, py = pixel(point_x, point_y)
                if map_left <= px <= map_right and map_top <= py <= map_bottom:
                    draw.ellipse(
                        (px - 2, py - 2, px + 2, py + 2),
                        fill=(255, 45, 210),
                    )
            actual_path = causal(path_samples, timestamp)
            if actual_path:
                path_pixels = [pixel(float(point[0]), float(point[1])) for point in actual_path]
                if len(path_pixels) > 1:
                    draw.line(path_pixels, fill=(50, 205, 125), width=3)
                plan_status = "actual published global plan"
            else:
                plan_status = "global plan unavailable"
            for x, y, _identifier in pedestrians:
                px, py = pixel(x, y)
                draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(255, 138, 45), outline=(60, 30, 10))
            gx, gy = pixel(float(goal[0]), float(goal[1]))
            draw.ellipse((gx - 9, gy - 9, gx + 9, gy + 9), fill=(245, 70, 70), outline=(255, 255, 255), width=2)
            if trajectory_row:
                robot_x, robot_y = number(trajectory_row, "x"), number(trajectory_row, "y")
                yaw = number(trajectory_row, "yaw") or 0.0
                if robot_x is not None and robot_y is not None:
                    rx, ry = pixel(robot_x, robot_y)
                    draw.ellipse((rx - 9, ry - 9, rx + 9, ry + 9), fill=(255, 220, 50), outline=(20, 20, 20), width=2)
                    draw.line((rx, ry, rx + 18 * math.cos(yaw), ry - 18 * math.sin(yaw)), fill=(20, 20, 20), width=3)
            else:
                robot_x = robot_y = None

            goal_distance = number(trajectory_row, "goal_distance_m")
            static_clearance = number(trajectory_row, "static_clearance_m")
            human_clearance = number(trajectory_row, "nearest_human_body_clearance_m")
            proxy_flags = []
            if static_clearance is not None and static_clearance < 0:
                proxy_flags.append("static overlap proxy")
            if human_clearance is not None and human_clearance < 0:
                proxy_flags.append("human overlap proxy")
            at_end = timestamp >= end_time - 1.0e-6
            if at_end:
                status = "goal reached" if summary["episode"].get("goal_reached") else summary["episode"].get("termination_reason", "finished")
            else:
                status = "running"
            draw.rounded_rectangle((28, 94, 590, 204), radius=8, fill=(12, 16, 22), outline=(75, 84, 98))
            draw.text((42, 104), f"status: {status}   collision/contact truth: unavailable", font=body_font, fill=(246, 218, 92))
            draw.text((42, 129), f"proxy state: {', '.join(proxy_flags) if proxy_flags else 'none at current sample'}", font=small_font, fill=(222, 170, 115))
            pose_text = "unavailable" if robot_x is None else f"({robot_x:.2f}, {robot_y:.2f}, yaw={yaw:.2f})"
            distance_text = "unavailable" if goal_distance is None else f"{goal_distance:.2f} m"
            draw.text((42, 151), f"robot pose: {pose_text}   goal distance: {distance_text}", font=small_font, fill=(212, 220, 230))
            raw_linear = number(decision_row, "raw_linear_x_mps")
            raw_angular = number(decision_row, "raw_angular_z_radps")
            action_text = "unavailable" if raw_linear is None or raw_angular is None else f"v={raw_linear:.3f} m/s, w={raw_angular:.3f} rad/s"
            draw.text((42, 174), f"actual model action: {action_text}", font=small_font, fill=(207, 214, 224))
            lidar_text = (
                f"live /scan_merged: {len(map_lidar_points)} map-frame points"
                if merged_lidar is not None
                else "live /scan_merged: unavailable"
            )
            draw.rectangle((42, 197, 54, 209), fill=(255, 45, 210))
            draw.text((62, 194), lidar_text, font=small_font, fill=(240, 220, 238))
            draw.text((30, 842), f"map frame | {plan_status} | model predicted trajectory: unavailable (velocity action only) | {scene}", font=small_font, fill=(225, 230, 236))

            command_linear = [(time, number(row, "linear_x_mps")) for time, row in data["commands"]]
            command_angular = [(time, number(row, "angular_z_radps")) for time, row in data["commands"]]
            if data["simulator"]:
                applied_linear = [(time, number(row, "applied_linear_x_mps")) for time, row in data["simulator"]]
                actual_linear = [(time, number(row, "actual_linear_x_mps")) for time, row in data["simulator"]]
                actual_angular = [(time, number(row, "actual_angular_z_radps")) for time, row in data["simulator"]]
                unavailable = ()
            else:
                applied_linear = []
                actual_linear = [(time, number(row, "odom_linear_x_mps")) for time, row in data["trajectory"]]
                actual_angular = [(time, number(row, "odom_angular_z_radps")) for time, row in data["trajectory"]]
                unavailable = ("applied_linear",)
            plot_series(
                draw, (1024, 82, 1584, 282),
                [("cmd_linear", command_linear), ("applied_linear", applied_linear), ("actual_linear", actual_linear)],
                timestamp, "linear velocity", "m/s", unavailable=unavailable,
            )
            plot_series(
                draw, (1024, 294, 1584, 494),
                [("cmd_angular", command_angular), ("actual_angular", actual_angular)],
                timestamp, "angular velocity", "rad/s",
            )
            draw_lidar(draw, (1024, 506, 1297, 884), lidar, 1)
            draw_lidar(draw, (1311, 506, 1584, 884), lidar, 2)

            if process is not None:
                assert process.stdin is not None
                process.stdin.write(canvas.tobytes())
            else:
                import cv2
                import numpy as np
                cv_writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
            episode_frame_counts[episode_number - 1] += 1
            final_canvases[episode_number] = canvas
    finally:
        if process is not None and process.stdin is not None:
            process.stdin.close()
        if cv_writer is not None:
            cv_writer.release()
    if process is not None:
        stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        if process.wait() != 0:
            raise RuntimeError(stderr)

    screenshots = []
    if args.save_episode_screenshots:
        for episode_number, canvas in final_canvases.items():
            path = output_mp4.parent / f"episode_{episode_number:02d}_final.png"
            canvas.save(path)
            screenshots.append(str(path))
    simulator_samples = sum(len(data["simulator"]) for data in episode_data)
    summary = {
        "schema": "fixed_four_evaluation_video/v1",
        "evaluation_dir": str(evaluation_dir),
        "output_mp4": str(output_mp4),
        "episode_count": 4,
        "frame_count": len(schedule),
        "episode_frame_counts": episode_frame_counts,
        "fps": args.fps,
        "playback_rate": args.playback_rate,
        "encoder": encoder,
        "time_basis": "simulation_time_sec; causal latest sample at or before each frame",
        "episode_time_behavior": "absolute simulation time is monotonic across the four sequential goals; inter-goal gaps are not rendered",
        "coordinate_semantics": "trajectory, pedestrians, accepted goals, published global path and live /scan_merged returns are drawn in the shared map frame; raw lidar slots are also retained in sensor-local polar panels",
        "data_availability": {
            "map": "available",
            "actual_trajectory": "available",
            "pedestrian_trace": "available",
            "commanded_velocity": "available",
            "applied_velocity": "available" if simulator_samples else "unavailable",
            "actual_velocity": "simulator_actuation" if simulator_samples else "odometry",
            "dual_lidar": "available" if lidar_path.is_file() else "unavailable",
            "map_frame_lidar_overlay": "available",
            "actual_published_global_path": "available" if path_samples else "unavailable",
            "model_predicted_trajectory": "unavailable",
            "physical_collision_truth": "unavailable",
        },
        "definitions": {
            "commanded_velocity": "commands.csv /cmd_vel final command",
            "applied_velocity": "simulator_actuation.csv applied command",
            "actual_velocity": "simulator telemetry when present, otherwise odometry twist",
            "model_output": "actuation_decisions.csv raw velocity action; not converted into a fabricated trajectory",
        },
        "screenshots": screenshots,
    }
    summary_path = output_mp4.parent / "video_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output_mp4} ({len(schedule)} frames) and {summary_path}")


if __name__ == "__main__":
    main()
