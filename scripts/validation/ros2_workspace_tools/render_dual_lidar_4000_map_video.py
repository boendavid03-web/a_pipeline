#!/usr/bin/env python3
"""Render both original 2000-beam LiDAR scans as map-frame rays."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from PIL import Image, ImageDraw
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from convert_rosbag2_dual_lidar_to_virtual360 import (
    build_static_graph,
    lookup_static_transform,
    normalize_frame,
    pair_scans,
    stamp_ns,
)
from render_virtual360_map_video import (
    OVERLAP_COLOR,
    SENSOR_COLORS,
    RESAMPLE_NEAREST,
    build_schedule,
    causal,
    clip_segment,
    composite_sensor_masks,
    draw_local_sensor_inset,
    finite_number,
    font,
    load_csv,
    load_map,
    open_writer,
    oriented_box_map,
    pedestrian_snapshots,
    timed_rows,
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--evaluation-dir", required=True, type=Path)
    parser.add_argument("--output-mp4", required=True, type=Path)
    parser.add_argument("--scan-01-topic", default="/scan_01")
    parser.add_argument("--scan-02-topic", default="/scan_02")
    parser.add_argument(
        "--sensor",
        choices=("both", "1", "2"),
        default="both",
        help="render both raw scans or isolate one physical sensor",
    )
    parser.add_argument(
        "--self-occlusion-calibration",
        type=Path,
        help="dataset metadata.json containing fixed scan_01/scan_02 self-mask runs",
    )
    parser.add_argument("--output-frame", default="base_link")
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--playback-rate", type=float, default=4.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--ray-alpha", type=int, default=118)
    parser.add_argument("--max-range-m", type=float, default=50.0)
    parser.add_argument("--robot-half-x-m", type=float, default=0.31237000644207)
    parser.add_argument("--robot-half-y-m", type=float, default=0.2435245481133461)
    parser.add_argument("--ray-origin-epsilon-m", type=float, default=0.01)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def load_bag(args: argparse.Namespace):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic in (args.scan_01_topic, args.scan_02_topic):
        if topic_types.get(topic) != "sensor_msgs/msg/LaserScan":
            raise RuntimeError(f"missing LaserScan topic {topic!r}")
    scans_01, scans_02, static_messages = [], [], []
    while reader.has_next():
        topic, data, _storage_time = reader.read_next()
        if topic == args.scan_01_topic:
            message = deserialize_message(data, LaserScan)
            scans_01.append((stamp_ns(message), message))
        elif topic == args.scan_02_topic:
            message = deserialize_message(data, LaserScan)
            scans_02.append((stamp_ns(message), message))
        elif topic == "/tf_static":
            static_messages.append(deserialize_message(data, TFMessage))
    tolerance_ns = round(args.sync_tolerance_ms * 1_000_000.0)
    pairs, skipped_01, skipped_02 = pair_scans(scans_01, scans_02, tolerance_ns)
    if not pairs:
        raise RuntimeError("no synchronized dual-LiDAR pairs")
    graph, _edges = build_static_graph(static_messages)
    frames = {
        normalize_frame(scan.header.frame_id)
        for pair in pairs
        for scan in (pair[1], pair[3])
    }
    transforms = {
        frame: lookup_static_transform(graph, frame, args.output_frame)
        for frame in frames
    }
    return pairs, transforms, skipped_01, skipped_02


def ray_start_offsets_outside_box(
    sensor_xy: np.ndarray,
    directions_xy: np.ndarray,
    half_extents_xy: np.ndarray,
    minimum_offset: float,
    epsilon: float,
) -> np.ndarray:
    """Match the Isaac producer's per-beam start beyond the robot box."""
    directions = directions_xy / np.linalg.norm(directions_xy, axis=1)[:, None]
    boundaries = np.where(directions >= 0.0, half_extents_xy, -half_extents_xy)
    with np.errstate(divide="ignore", invalid="ignore"):
        axis_exit = (boundaries - sensor_xy) / directions
    axis_exit[np.abs(directions) <= 1.0e-12] = np.inf
    exit_offsets = np.min(axis_exit, axis=1)
    if np.any(~np.isfinite(exit_offsets)) or np.any(exit_offsets < 0.0):
        raise RuntimeError("failed to reconstruct finite ray starts outside robot box")
    return np.maximum(float(minimum_offset), exit_offsets + float(epsilon))


def load_fixed_self_masks(path: Path | None, beam_count: int = 2000):
    if path is None:
        return {1: np.zeros(beam_count, dtype=bool), 2: np.zeros(beam_count, dtype=bool)}
    metadata = json.loads(path.expanduser().resolve().read_text())
    calibration = metadata.get("self_mask_calibration")
    if not isinstance(calibration, dict):
        raise RuntimeError(f"{path}: missing self_mask_calibration")
    masks = {}
    for sensor in (1, 2):
        key = f"scan_0{sensor}_masked_beam_runs"
        runs = calibration.get(key)
        if not isinstance(runs, list):
            raise RuntimeError(f"{path}: missing {key}")
        mask = np.zeros(beam_count, dtype=bool)
        for start, end in runs:
            start, end = int(start), int(end)
            if start < 0 or end < start or end >= beam_count:
                raise RuntimeError(f"{path}: invalid {key} run {[start, end]}")
            mask[start : end + 1] = True
        expected = calibration.get(f"scan_0{sensor}_masked_beam_count")
        if expected is not None and int(mask.sum()) != int(expected):
            raise RuntimeError(f"{path}: {key} count mismatch")
        masks[sensor] = mask
    return masks


def scan_geometry(
    scan: LaserScan,
    source_to_base: np.ndarray,
    max_range_m: float,
    robot_half_extents: np.ndarray,
    ray_origin_epsilon_m: float,
    fixed_self_mask: np.ndarray | None = None,
):
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = float(scan.angle_min) + np.arange(ranges.size) * float(
        scan.angle_increment
    )
    low = float(scan.range_min)
    high = min(float(scan.range_max), max_range_m)
    valid = np.isfinite(ranges) & (ranges >= low) & (ranges <= high)
    if fixed_self_mask is not None:
        fixed_self_mask = np.asarray(fixed_self_mask, dtype=bool)
        if fixed_self_mask.shape != ranges.shape:
            raise RuntimeError(
                f"fixed self mask shape {fixed_self_mask.shape} does not match scan {ranges.shape}"
            )
        valid &= ~fixed_self_mask
    ranges, angles = ranges[valid], angles[valid]
    directions_sensor = np.column_stack((np.cos(angles), np.sin(angles)))
    directions_base = directions_sensor @ source_to_base[:2, :2].T
    sensor_origin_base = source_to_base[:2, 3]
    start_offsets = ray_start_offsets_outside_box(
        sensor_origin_base,
        directions_base,
        robot_half_extents,
        float(scan.range_min),
        ray_origin_epsilon_m,
    )
    ray_origins_base = sensor_origin_base + directions_base * start_offsets[:, None]
    points_sensor = np.column_stack(
        (
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.zeros(ranges.size, dtype=np.float64),
            np.ones(ranges.size, dtype=np.float64),
        )
    )
    points_base = points_sensor @ source_to_base.T
    return sensor_origin_base, ray_origins_base, points_base[:, :2]


def base_to_map(points: np.ndarray, robot_x: float, robot_y: float, yaw: float):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return points @ rotation.T + np.asarray([robot_x, robot_y])


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

    pairs, transforms, skipped_01, skipped_02 = load_bag(args)
    fixed_self_masks = load_fixed_self_masks(args.self_occlusion_calibration)
    pair_times = np.asarray(
        [max(first_stamp, second_stamp) for first_stamp, _, second_stamp, _ in pairs],
        dtype=np.float64,
    ) * 1.0e-9

    evaluation = args.evaluation_dir.expanduser().resolve()
    session = json.loads((evaluation / "session_summary.json").read_text())
    episodes, episode_data = [], []
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
    schedule = build_schedule(episodes, args.fps, args.playback_rate)
    pair_indices = np.searchsorted(
        pair_times, np.asarray([item[1] for item in schedule]), side="right"
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
    valid_counts_01, valid_counts_02 = [], []
    enabled_sensors = (1, 2) if args.sensor == "both" else (int(args.sensor),)
    title_prefix = "Self-occluded" if args.self_occlusion_calibration else "Recorded"
    title = (
        f"{title_prefix} dual LiDAR: 2000 + 2000 map-frame rays"
        if args.sensor == "both"
        else f"{title_prefix} LiDAR scan_0{args.sensor}: 2000 map-frame rays"
    )
    try:
        for frame_number, ((episode_number, timestamp), pair_index) in enumerate(
            zip(schedule, pair_indices)
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
                title,
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

            robot_x, robot_y = finite_number(pose, "x"), finite_number(pose, "y")
            robot_yaw = finite_number(pose, "yaw")
            counts = [0, 0]
            age_ms = math.nan
            sensor_origins_map = []
            local_segments = []
            local_sensor_origins = []
            if (
                pair_index >= 0
                and robot_x is not None
                and robot_y is not None
                and robot_yaw is not None
            ):
                pair = pairs[pair_index]
                age_ms = (timestamp - pair_times[pair_index]) * 1000.0
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
                for sensor_index, scan in enumerate((pair[1], pair[3]), start=1):
                    if sensor_index not in enabled_sensors:
                        continue
                    transform = transforms[normalize_frame(scan.header.frame_id)]
                    origin_base, ray_origins_base, points_base = scan_geometry(
                        scan,
                        transform,
                        args.max_range_m,
                        np.asarray(
                            [args.robot_half_x_m, args.robot_half_y_m],
                            dtype=np.float64,
                        ),
                        args.ray_origin_epsilon_m,
                        fixed_self_masks[sensor_index],
                    )
                    origin_map = base_to_map(
                        origin_base.reshape(1, 2), robot_x, robot_y, robot_yaw
                    )[0]
                    points_map = base_to_map(points_base, robot_x, robot_y, robot_yaw)
                    ray_origins_map = base_to_map(
                        ray_origins_base, robot_x, robot_y, robot_yaw
                    )
                    ox, oy = pixel(float(origin_map[0]), float(origin_map[1]))
                    sensor_origins_map.append((ox, oy, sensor_index))
                    local_sensor_origins.append((origin_base.copy(), sensor_index))
                    local_segments.append(
                        (ray_origins_base.copy(), points_base.copy(), sensor_index)
                    )
                    for ray_origin, endpoint in zip(ray_origins_map, points_map):
                        start_x, start_y = pixel(
                            float(ray_origin[0]), float(ray_origin[1])
                        )
                        px, py = pixel(float(endpoint[0]), float(endpoint[1]))
                        clipped = clip_segment(
                            start_x,
                            start_y,
                            px,
                            py,
                            map_left,
                            map_top,
                            map_right,
                            map_bottom,
                        )
                        if clipped is None:
                            continue
                        ray_draws[sensor_index].line(clipped, fill=255, width=1)
                        if map_left <= px <= map_right and map_top <= py <= map_bottom:
                            endpoint_draws[sensor_index].point((px, py), fill=255)
                        counts[sensor_index - 1] += 1
                canvas = composite_sensor_masks(canvas, ray_masks, args.ray_alpha)
                canvas = composite_sensor_masks(canvas, endpoint_masks, 210)
                draw = ImageDraw.Draw(canvas)
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
                for ox, oy, sensor_index in sensor_origins_map:
                    color = SENSOR_COLORS[sensor_index]
                    draw.ellipse(
                        (ox - 5, oy - 5, ox + 5, oy + 5),
                        fill=color,
                        outline=(255, 255, 255),
                        width=1,
                    )
                rx, ry = pixel(robot_x, robot_y)
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
            valid_counts_01.append(counts[0])
            valid_counts_02.append(counts[1])

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
                (panel_x, 82, args.width - 16, 428),
                radius=9,
                fill=(20, 26, 35),
                outline=(74, 86, 102),
                width=1,
            )
            panel_title = "raw dual scan" if args.sensor == "both" else f"raw scan_0{args.sensor} only"
            draw.text((panel_x + 14, 98), panel_title, font=body_font, fill=(240, 244, 248))
            input_slots = "2000 + 2000" if args.sensor == "both" else "2000"
            lines = [
                (f"input slots: {input_slots}", (197, 210, 225)),
                ("raw angular span: 360 deg", (197, 210, 225)),
                (
                    "fixed self-occlusion applied"
                    if args.self_occlusion_calibration
                    else "recorded scan; self hits absent",
                    (197, 210, 225),
                ),
            ]
            for sensor_index in enabled_sensors:
                lines.append(
                    (
                        f"valid sensor {sensor_index}: {counts[sensor_index - 1]}",
                        SENSOR_COLORS[sensor_index],
                    )
                )
                if args.self_occlusion_calibration:
                    lines.append(
                        (
                            f"masked sensor {sensor_index}: {int(fixed_self_masks[sensor_index].sum())}",
                            (197, 210, 225),
                        )
                    )
                    visible_degrees = (
                        360.0
                        * (len(fixed_self_masks[sensor_index]) - int(fixed_self_masks[sensor_index].sum()))
                        / len(fixed_self_masks[sensor_index])
                    )
                    lines.append(
                        (
                            f"visible support: {visible_degrees:.1f} deg",
                            (197, 210, 225),
                        )
                    )
            lines.extend(
                [
                    (f"drawn total: {sum(counts)}", (197, 210, 225)),
                    (f"scan age: {age_ms:.1f} ms", (197, 210, 225)),
                    ("", (197, 210, 225)),
                ]
            )
            if 1 in enabled_sensors:
                lines.append(("blue: scan_01 rays", SENSOR_COLORS[1]))
            if 2 in enabled_sensors:
                lines.append(("orange: scan_02 rays", SENSOR_COLORS[2]))
            if args.sensor == "both":
                lines.append(("purple: ray-pixel overlap", OVERLAP_COLOR))
            lines.extend(
                [
                    ("small dots: sensor origins", (197, 210, 225)),
                    ("pink: pedestrians", (255, 76, 190)),
                    ("green: robot trail", (197, 210, 225)),
                    ("red: accepted goal", (197, 210, 225)),
                ]
            )
            for line_index, (text, color) in enumerate(lines):
                draw.text(
                    (panel_x + 14, 130 + line_index * 19),
                    text,
                    font=small_font,
                    fill=color,
                )
            canvas = draw_local_sensor_inset(
                canvas,
                (panel_x, 436, args.width - 16, args.height - 18),
                local_segments,
                local_sensor_origins,
                args.robot_half_x_m,
                args.robot_half_y_m,
                (
                    "visible rays, 1/4 (+x)"
                    if args.self_occlusion_calibration
                    else "recorded starts, 1/4 (+x)"
                ),
                show_start_points=True,
                ray_stride=4,
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
    screenshot_tag = "raw4000" if args.sensor == "both" else f"scan_0{args.sensor}_only"
    for episode_number, canvas in sorted(final_frames.items()):
        screenshot = output.parent / f"episode_{episode_number:02d}_{screenshot_tag}_rays.png"
        canvas.save(screenshot)
        screenshots.append(str(screenshot))
    summary = {
        "schema": "raw_dual_lidar_map_ray_video/v1",
        "source_bag": str(args.bag.resolve()),
        "evaluation_dir": str(evaluation),
        "map_yaml": str(map_path),
        "output_mp4": str(output),
        "encoder": encoder,
        "fps": args.fps,
        "playback_rate": args.playback_rate,
        "frame_count": len(schedule),
        "episode_frame_counts": frame_counts,
        "paired_scans": len(pairs),
        "skipped_scan_01": skipped_01,
        "skipped_scan_02": skipped_02,
        "input_beams_per_pair": [2000 for _ in enabled_sensors],
        "rendered_sensors": list(enabled_sensors),
        "self_occlusion_calibration": (
            str(args.self_occlusion_calibration.expanduser().resolve())
            if args.self_occlusion_calibration
            else None
        ),
        "fixed_self_masked_beams": {
            f"scan_0{sensor}": int(fixed_self_masks[sensor].sum())
            for sensor in enabled_sensors
        },
        "mean_valid_drawn": [
            float(np.mean((valid_counts_01, valid_counts_02)[sensor - 1]))
            for sensor in enabled_sensors
        ],
        "ray_semantics": "published range endpoints use each physical sensor; rendered segments start at the producer's per-beam origin outside the robot collision box",
        "robot_collision_box_m": [
            2.0 * args.robot_half_x_m,
            2.0 * args.robot_half_y_m,
        ],
        "ray_origin_epsilon_m": args.ray_origin_epsilon_m,
        "local_inset": {
            "frame": "base_link",
            "extent_m": 1.15,
            "ray_stride": 4,
            "shows_sensor_mounts": True,
            "shows_reconstructed_query_origins": True,
        },
        "ray_colors": {
            "scan_01_only": "blue",
            "scan_02_only": "orange",
            "pixel_overlap": "purple",
        },
        "screenshots": screenshots,
    }
    summary_name = (
        "raw4000_ray_video_summary.json"
        if args.sensor == "both"
        else f"scan_0{args.sensor}_only_ray_video_summary.json"
    )
    (output.parent / summary_name).write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(f"wrote {output} with {len(schedule)} frames using {encoder}")


if __name__ == "__main__":
    main()
