#!/usr/bin/env python3
"""Render virtual 360 LiDAR rays in the map frame as an MP4 video."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont


RESAMPLE_NEAREST = getattr(getattr(Image, "Resampling", Image), "NEAREST")
SENSOR_COLORS = {1: (0, 76, 170), 2: (196, 72, 0)}
OVERLAP_COLOR = (178, 83, 186)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--virtual360", required=True, type=Path)
    parser.add_argument("--evaluation-dir", required=True, type=Path)
    parser.add_argument("--output-mp4", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--playback-rate", type=float, default=4.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--ray-alpha", type=int, default=105)
    parser.add_argument("--max-range-m", type=float, default=50.0)
    parser.add_argument("--robot-half-x-m", type=float, default=0.31237000644207)
    parser.add_argument("--robot-half-y-m", type=float, default=0.2435245481133461)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_number(row: dict | None, key: str):
    if row is None:
        return None
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def timed_rows(rows: list[dict[str, str]], key: str = "simulation_time_sec"):
    result = []
    for row in rows:
        timestamp = finite_number(row, key)
        if timestamp is not None:
            result.append((timestamp, row))
    return sorted(result, key=lambda item: item[0])


def causal(items, timestamp: float):
    if not items:
        return None
    times = [item[0] for item in items]
    index = bisect.bisect_right(times, timestamp) - 1
    return items[index][1] if index >= 0 else None


def pedestrian_snapshots(rows: list[dict[str, str]]):
    snapshots = []
    timestamp = None
    people = []
    for row in rows:
        current = finite_number(row, "simulation_time_sec")
        x, y = finite_number(row, "x"), finite_number(row, "y")
        if current is None or x is None or y is None:
            continue
        if timestamp is None or not math.isclose(current, timestamp, abs_tol=1.0e-9):
            if people:
                snapshots.append((timestamp, people))
            timestamp, people = current, []
        people.append((x, y))
    if people:
        snapshots.append((timestamp, people))
    return snapshots


def load_map(path: Path):
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    image = Image.open(image_path).convert("RGB")
    origin = metadata["origin"]
    return image, float(metadata["resolution"]), float(origin[0]), float(origin[1])


def build_schedule(episodes: list[dict], fps: float, playback_rate: float):
    step = playback_rate / fps
    schedule = []
    for episode_number, summary in enumerate(episodes, start=1):
        start = float(summary["experiment"]["simulation_time_start"])
        end = float(summary["experiment"]["simulation_time_end"])
        timestamp = start
        while timestamp < end:
            schedule.append((episode_number, timestamp))
            timestamp += step
        schedule.append((episode_number, end))
    return schedule


def clip_segment(x0, y0, x1, y1, left, top, right, bottom):
    """Liang-Barsky clip in pixel coordinates."""
    dx, dy = x1 - x0, y1 - y0
    p = (-dx, dx, -dy, dy)
    q = (x0 - left, right - x0, y0 - top, bottom - y0)
    low, high = 0.0, 1.0
    for pi, qi in zip(p, q):
        if pi == 0.0:
            if qi < 0.0:
                return None
            continue
        ratio = qi / pi
        if pi < 0.0:
            low = max(low, ratio)
        else:
            high = min(high, ratio)
        if low > high:
            return None
    return x0 + low * dx, y0 + low * dy, x0 + high * dx, y0 + high * dy


def open_writer(path: Path, fps: float, size: tuple[int, int]):
    for codec, label in (("avc1", "opencv/avc1"), ("mp4v", "opencv/mp4v")):
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), fps, size)
        if writer.isOpened():
            return writer, label
        writer.release()
    raise RuntimeError("OpenCV could not open an H.264 or MPEG-4 writer")


def composite_sensor_masks(
    canvas: Image.Image,
    masks: dict[int, Image.Image],
    alpha: int,
) -> Image.Image:
    """Blend two sensor masks without giving either draw-order priority."""
    array = np.asarray(canvas.convert("RGB"), dtype=np.float32).copy()
    first = np.asarray(masks[1], dtype=np.uint8) > 0
    second = np.asarray(masks[2], dtype=np.uint8) > 0
    weight = float(alpha) / 255.0
    for region, color in (
        (first & ~second, SENSOR_COLORS[1]),
        (second & ~first, SENSOR_COLORS[2]),
        (first & second, OVERLAP_COLOR),
    ):
        if np.any(region):
            array[region] = array[region] * (1.0 - weight) + np.asarray(color) * weight
    return Image.fromarray(np.clip(array, 0, 255).astype(np.uint8), mode="RGB")


def oriented_box_map(
    robot_x: float,
    robot_y: float,
    yaw: float,
    half_x: float,
    half_y: float,
) -> np.ndarray:
    corners = np.asarray(
        [
            [-half_x, -half_y],
            [half_x, -half_y],
            [half_x, half_y],
            [-half_x, half_y],
        ],
        dtype=np.float64,
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return corners @ rotation.T + np.asarray([robot_x, robot_y])


def draw_local_sensor_inset(
    canvas: Image.Image,
    bounds: tuple[int, int, int, int],
    segment_groups: list[tuple[np.ndarray, np.ndarray, int]],
    sensor_origins: list[tuple[np.ndarray, int]],
    half_x: float,
    half_y: float,
    title: str,
    extent_m: float = 1.15,
    show_start_points: bool = False,
    ray_stride: int = 1,
) -> Image.Image:
    """Draw robot-frame ray geometry large enough to expose self occlusion."""
    left, top, right, bottom = bounds
    layer = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    draw.rounded_rectangle(
        bounds,
        radius=9,
        fill=(20, 26, 35, 245),
        outline=(74, 86, 102, 255),
        width=1,
    )
    draw.text((left + 10, top + 8), title, font=font(13), fill=(240, 244, 248, 255))
    plot_left, plot_top = left + 11, top + 31
    plot_right, plot_bottom = right - 11, bottom - 11
    scale = min(
        (plot_right - plot_left) / (2.0 * extent_m),
        (plot_bottom - plot_top) / (2.0 * extent_m),
    )
    center_x = (plot_left + plot_right) * 0.5
    center_y = (plot_top + plot_bottom) * 0.5

    def local_pixel(point: np.ndarray | tuple[float, float]):
        return center_x + float(point[0]) * scale, center_y - float(point[1]) * scale

    for offset in (-1.0, -0.5, 0.0, 0.5, 1.0):
        if abs(offset) > extent_m:
            continue
        x, _ = local_pixel((offset, 0.0))
        _, y = local_pixel((0.0, offset))
        grid_color = (82, 94, 108, 85 if offset else 145)
        draw.line((x, plot_top, x, plot_bottom), fill=grid_color, width=1)
        draw.line((plot_left, y, plot_right, y), fill=grid_color, width=1)

    canvas = Image.alpha_composite(canvas.convert("RGBA"), layer).convert("RGB")
    ray_masks = {sensor: Image.new("L", canvas.size, 0) for sensor in (1, 2)}
    ray_draws = {sensor: ImageDraw.Draw(ray_masks[sensor]) for sensor in (1, 2)}
    start_masks = {sensor: Image.new("L", canvas.size, 0) for sensor in (1, 2)}
    start_draws = {sensor: ImageDraw.Draw(start_masks[sensor]) for sensor in (1, 2)}

    ray_stride = max(1, int(ray_stride))
    for starts, ends, sensor in segment_groups:
        for start, end in zip(starts[::ray_stride], ends[::ray_stride]):
            sx, sy = local_pixel(start)
            ex, ey = local_pixel(end)
            clipped = clip_segment(
                sx, sy, ex, ey, plot_left, plot_top, plot_right, plot_bottom
            )
            if clipped is not None:
                ray_draws[sensor].line(clipped, fill=255, width=1)
            if show_start_points and plot_left <= sx <= plot_right and plot_top <= sy <= plot_bottom:
                start_draws[sensor].ellipse((sx - 1, sy - 1, sx + 1, sy + 1), fill=255)

    canvas = composite_sensor_masks(canvas, ray_masks, 78)
    if show_start_points:
        canvas = composite_sensor_masks(canvas, start_masks, 230)
    annotation = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(annotation)

    box = [
        local_pixel((-half_x, -half_y)),
        local_pixel((half_x, -half_y)),
        local_pixel((half_x, half_y)),
        local_pixel((-half_x, half_y)),
    ]
    draw.polygon(box, fill=(58, 66, 78, 255), outline=(245, 230, 120, 255), width=2)
    for origin, sensor in sensor_origins:
        ox, oy = local_pixel(origin)
        color = SENSOR_COLORS.get(sensor, (220, 220, 220))
        draw.ellipse(
            (ox - 4, oy - 4, ox + 4, oy + 4),
            fill=(*color, 255),
            outline=(255, 255, 255, 255),
            width=1,
        )
    cx, cy = local_pixel((0.0, 0.0))
    draw.line((cx, cy, cx + 18, cy), fill=(255, 226, 62, 255), width=3)
    draw.polygon(
        [(cx + 18, cy), (cx + 12, cy - 4), (cx + 12, cy + 4)],
        fill=(255, 226, 62, 255),
    )
    return Image.alpha_composite(canvas.convert("RGBA"), annotation).convert("RGB")


def main() -> None:
    args = arguments()
    if args.fps <= 0.0 or args.playback_rate <= 0.0:
        raise ValueError("fps and playback-rate must be positive")
    if not 0 <= args.ray_alpha <= 255 or args.max_range_m <= 0.0:
        raise ValueError("invalid ray alpha or range")
    output = args.output_mp4.expanduser().resolve()
    if output.exists() and not args.overwrite:
        raise FileExistsError(f"refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    evaluation = args.evaluation_dir.expanduser().resolve()
    session = json.loads((evaluation / "session_summary.json").read_text())
    episodes = []
    episode_data = []
    for entry in session["episodes"]:
        directory = evaluation / entry["directory"]
        summary = json.loads((directory / "episode_summary.json").read_text())
        episodes.append(summary)
        episode_data.append(
            {
                "trajectory": timed_rows(load_csv(directory / "trajectory.csv")),
                "pedestrians": pedestrian_snapshots(
                    load_csv(directory / "pedestrian_trace.csv")
                ),
            }
        )
    map_path = Path(episodes[0]["map_provenance"]["map_yaml_path"])
    map_image, resolution, origin_x, origin_y = load_map(map_path)

    with np.load(args.virtual360, allow_pickle=False) as data:
        scan_times = data["stamp_ns"].astype(np.float64) * 1.0e-9
        ranges = data["ranges_m"].copy()
        valid = data["valid_mask"].copy()
        angles = data["angles_rad"].copy()
        sensors = data["source_sensor"].copy()
        source_masks = data["source_sensor_mask"].copy()
    if (
        ranges.ndim != 2
        or ranges.shape != valid.shape
        or ranges.shape != sensors.shape
        or ranges.shape != source_masks.shape
    ):
        raise RuntimeError("virtual360 range, validity and source arrays do not match")
    if ranges.shape[1] != angles.size:
        raise RuntimeError("virtual360 angle count does not match range width")

    schedule = build_schedule(episodes, args.fps, args.playback_rate)
    scan_indices = np.searchsorted(
        scan_times, np.asarray([item[1] for item in schedule]), side="right"
    ) - 1
    writer, encoder = open_writer(output, args.fps, (args.width, args.height))

    title_font, body_font, small_font = font(23), font(16), font(13)
    map_box = (16, 68, args.width - 250, args.height - 18)
    map_left, map_top, map_right, map_bottom = map_box
    map_width, map_height = map_right - map_left, map_bottom - map_top
    scale = min(map_width / map_image.width, map_height / map_image.height)
    drawn_map = map_image.resize(
        (
            max(1, round(map_image.width * scale)),
            max(1, round(map_image.height * scale)),
        ),
        RESAMPLE_NEAREST,
    )
    paste_x = map_left + (map_width - drawn_map.width) // 2
    paste_y = map_top + (map_height - drawn_map.height) // 2

    def pixel(x: float, y: float):
        return (
            paste_x + (x - origin_x) / resolution * scale,
            paste_y + drawn_map.height - 1 - (y - origin_y) / resolution * scale,
        )

    final_frames = {}
    frame_counts = [0] * len(episodes)
    try:
        for frame_number, ((episode_number, timestamp), scan_index) in enumerate(
            zip(schedule, scan_indices)
        ):
            summary = episodes[episode_number - 1]
            current_data = episode_data[episode_number - 1]
            pose = causal(current_data["trajectory"], timestamp)
            people = causal(current_data["pedestrians"], timestamp) or []
            canvas = Image.new("RGB", (args.width, args.height), (11, 15, 21))
            canvas.paste(drawn_map, (paste_x, paste_y))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle(map_box, outline=(80, 91, 105), width=2)
            draw.text(
                (16, 10),
                "Dual-LiDAR virtual 360 rays in map frame",
                font=title_font,
                fill=(242, 246, 250),
            )
            draw.text(
                (16, 41),
                f"episode {episode_number}/4   simulation time {timestamp:.3f}s   "
                f"playback x{args.playback_rate:g}",
                font=small_font,
                fill=(190, 202, 218),
            )

            trail = []
            for row_time, row in current_data["trajectory"]:
                if row_time > timestamp:
                    break
                x, y = finite_number(row, "x"), finite_number(row, "y")
                if x is not None and y is not None:
                    trail.append(pixel(x, y))
            if len(trail) > 1:
                draw.line(trail, fill=(37, 205, 118), width=3)

            robot_x = finite_number(pose, "x")
            robot_y = finite_number(pose, "y")
            robot_yaw = finite_number(pose, "yaw")
            ray_count = 0
            selected_01 = 0
            selected_02 = 0
            dual_candidate_slots = 0
            local_segments = []
            if (
                scan_index >= 0
                and robot_x is not None
                and robot_y is not None
                and robot_yaw is not None
            ):
                rx, ry = pixel(robot_x, robot_y)
                ray_masks = {
                    sensor: Image.new("L", canvas.size, 0) for sensor in (1, 2)
                }
                ray_draws = {
                    sensor: ImageDraw.Draw(ray_masks[sensor]) for sensor in (1, 2)
                }
                endpoint_masks = {
                    sensor: Image.new("L", canvas.size, 0) for sensor in (1, 2)
                }
                endpoint_draws = {
                    sensor: ImageDraw.Draw(endpoint_masks[sensor]) for sensor in (1, 2)
                }
                frame_valid = valid[scan_index] & np.isfinite(ranges[scan_index])
                frame_valid &= ranges[scan_index] <= args.max_range_m
                dual_candidate_slots = int(np.count_nonzero(source_masks[scan_index] == 3))
                local_ends_by_sensor = {1: [], 2: []}
                for beam in np.flatnonzero(frame_valid):
                    distance = float(ranges[scan_index, beam])
                    sensor = int(sensors[scan_index, beam])
                    angle = robot_yaw + float(angles[beam])
                    endpoint_x = robot_x + distance * math.cos(angle)
                    endpoint_y = robot_y + distance * math.sin(angle)
                    local_ends_by_sensor.setdefault(sensor, []).append(
                        [
                            distance * math.cos(float(angles[beam])),
                            distance * math.sin(float(angles[beam])),
                        ]
                    )
                    px, py = pixel(endpoint_x, endpoint_y)
                    clipped = clip_segment(
                        rx, ry, px, py, map_left, map_top, map_right, map_bottom
                    )
                    if clipped is None:
                        continue
                    ray_draws[sensor].line(clipped, fill=255, width=1)
                    if map_left <= px <= map_right and map_top <= py <= map_bottom:
                        endpoint_draws[sensor].ellipse(
                            (px - 1.5, py - 1.5, px + 1.5, py + 1.5),
                            fill=255,
                        )
                    ray_count += 1
                    selected_01 += sensor == 1
                    selected_02 += sensor == 2
                for sensor, local_ends in local_ends_by_sensor.items():
                    if local_ends:
                        ends_array = np.asarray(local_ends, dtype=np.float64)
                        local_segments.append(
                            (np.zeros_like(ends_array), ends_array, sensor)
                        )
                canvas = composite_sensor_masks(canvas, ray_masks, args.ray_alpha)
                canvas = composite_sensor_masks(canvas, endpoint_masks, 225)
                draw = ImageDraw.Draw(canvas)

                rx, ry = pixel(robot_x, robot_y)
                footprint = oriented_box_map(
                    robot_x,
                    robot_y,
                    robot_yaw,
                    args.robot_half_x_m,
                    args.robot_half_y_m,
                )
                footprint_pixels = [pixel(float(x), float(y)) for x, y in footprint]
                draw.polygon(
                    footprint_pixels,
                    fill=(58, 66, 78),
                    outline=(245, 230, 120),
                    width=2,
                )
                draw.ellipse(
                    (rx - 8, ry - 8, rx + 8, ry + 8),
                    fill=(255, 226, 62),
                    outline=(20, 20, 20),
                    width=2,
                )
                draw.line(
                    (
                        rx,
                        ry,
                        rx + 18 * math.cos(robot_yaw),
                        ry - 18 * math.sin(robot_yaw),
                    ),
                    fill=(20, 20, 20),
                    width=3,
                )

            for x, y in people:
                px, py = pixel(x, y)
                draw.ellipse(
                    (px - 5, py - 5, px + 5, py + 5),
                    fill=(255, 76, 190),
                    outline=(80, 20, 60),
                )
            goal = summary["experiment"]["accepted_goal"]
            gx, gy = pixel(float(goal[0]), float(goal[1]))
            draw.ellipse(
                (gx - 8, gy - 8, gx + 8, gy + 8),
                fill=(245, 69, 69),
                outline=(255, 255, 255),
                width=2,
            )

            panel_x = args.width - 224
            draw.rounded_rectangle(
                (panel_x, 82, args.width - 16, 390),
                radius=9,
                fill=(20, 26, 35),
                outline=(74, 86, 102),
                width=1,
            )
            draw.text((panel_x + 14, 98), "virtual scan", font=body_font, fill=(240, 244, 248))
            scan_time = scan_times[scan_index] if scan_index >= 0 else math.nan
            age_ms = (timestamp - scan_time) * 1000.0 if scan_index >= 0 else math.nan
            lines = [
                f"rays: {ray_count}/360",
                f"sensor 1 wins: {selected_01}",
                f"sensor 2 wins: {selected_02}",
                f"both candidates: {dual_candidate_slots}",
                "robot footprint: 0.625 x 0.487 m",
                f"scan age: {age_ms:.1f} ms",
                "",
                "blue: sensor 1 winner",
                "orange: sensor 2 winner",
                "purple: ray-pixel overlap",
                "pink: pedestrians",
                "green: robot trail",
                "red: accepted goal",
            ]
            for line_index, text in enumerate(lines):
                color = (197, 210, 225)
                if line_index == 7:
                    color = SENSOR_COLORS[1]
                elif line_index == 8:
                    color = SENSOR_COLORS[2]
                elif line_index == 9:
                    color = OVERLAP_COLOR
                elif line_index == 10:
                    color = (255, 76, 190)
                draw.text(
                    (panel_x + 14, 130 + line_index * 20),
                    text,
                    font=small_font,
                    fill=color,
                )
            canvas = draw_local_sensor_inset(
                canvas,
                (panel_x, 406, args.width - 16, args.height - 18),
                local_segments,
                [],
                args.robot_half_x_m,
                args.robot_half_y_m,
                "robot-frame zoom (+x right)",
            )
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (args.width - 178, 42),
                f"frame {frame_number + 1}/{len(schedule)}",
                font=small_font,
                fill=(165, 178, 194),
            )
            writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
            final_frames[episode_number] = canvas
            frame_counts[episode_number - 1] += 1
    finally:
        writer.release()

    screenshots = []
    for episode_number, canvas in sorted(final_frames.items()):
        screenshot = output.parent / f"episode_{episode_number:02d}_virtual360_rays.png"
        canvas.save(screenshot)
        screenshots.append(str(screenshot))
    summary = {
        "schema": "virtual360_map_ray_video/v1",
        "input_npz": str(args.virtual360.resolve()),
        "evaluation_dir": str(evaluation),
        "map_yaml": str(map_path),
        "output_mp4": str(output),
        "encoder": encoder,
        "fps": args.fps,
        "playback_rate": args.playback_rate,
        "frame_count": len(schedule),
        "episode_frame_counts": frame_counts,
        "ray_count_per_scan": ranges.shape[1],
        "ray_semantics": "base_link virtual360 endpoints transformed by nearest causal map-frame robot pose",
        "robot_collision_box_m": [
            2.0 * args.robot_half_x_m,
            2.0 * args.robot_half_y_m,
        ],
        "local_inset": {"frame": "base_link", "extent_m": 1.15},
        "ray_colors": {
            "sensor_1_winner_only": "blue",
            "sensor_2_winner_only": "orange",
            "pixel_overlap": "purple",
        },
        "screenshots": screenshots,
    }
    (output.parent / "virtual360_ray_video_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(f"wrote {output} with {len(schedule)} frames using {encoder}")


if __name__ == "__main__":
    main()
