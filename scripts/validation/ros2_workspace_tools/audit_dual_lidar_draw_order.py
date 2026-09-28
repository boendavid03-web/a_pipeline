#!/usr/bin/env python3
"""Compare dual-LiDAR draw order on representative rosbag2 recordings."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


BLUE = np.asarray([170, 76, 0], dtype=np.uint8)  # OpenCV BGR
ORANGE = np.asarray([0, 72, 196], dtype=np.uint8)
PURPLE = np.asarray([186, 83, 178], dtype=np.uint8)
BACKGROUND = np.asarray([21, 15, 11], dtype=np.uint8)
SENSOR_POSES = {
    "/scan_01": (0.20, 0.13, 0.0),
    "/scan_02": (-0.20, -0.13, math.pi),
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", action="append", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--extent-m", type=float, default=8.0)
    parser.add_argument("--panel-size", type=int, default=246)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def topic_counts(metadata: dict) -> dict[str, int]:
    return {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in metadata.get("topics_with_message_count", [])
    }


def load_middle_scans(bag: Path) -> tuple[dict[str, LaserScan], dict]:
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    counts = topic_counts(metadata)
    topics = tuple(SENSOR_POSES)
    for topic in topics:
        if counts.get(topic, 0) <= 0:
            raise RuntimeError(f"{bag}: missing non-empty {topic}")
    targets = {topic: counts[topic] // 2 for topic in topics}
    seen = {topic: 0 for topic in topics}
    scans: dict[str, LaserScan] = {}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=str(metadata["storage_identifier"])
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    while reader.has_next() and len(scans) < len(topics):
        topic, data, _ = reader.read_next()
        if topic not in seen:
            continue
        if seen[topic] == targets[topic]:
            scans[topic] = deserialize_message(data, LaserScan)
        seen[topic] += 1
    return scans, metadata


def scan_points(scan: LaserScan, pose: tuple[float, float, float]):
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = float(scan.angle_min) + np.arange(ranges.size) * float(scan.angle_increment)
    valid = np.isfinite(ranges)
    valid &= ranges >= float(scan.range_min)
    valid &= ranges <= float(scan.range_max)
    ranges, angles = ranges[valid], angles[valid]
    sx, sy, yaw = pose
    world_angles = angles + yaw
    ends = np.column_stack(
        (sx + ranges * np.cos(world_angles), sy + ranges * np.sin(world_angles))
    )
    starts = np.repeat(np.asarray([[sx, sy]], dtype=np.float64), len(ends), axis=0)
    return starts, ends, ranges, int(valid.sum())


def pixel(point: np.ndarray, size: int, extent_m: float) -> tuple[int, int]:
    scale = (size - 18) / (2.0 * extent_m)
    return (
        int(round(size * 0.5 + float(point[0]) * scale)),
        int(round(size * 0.5 - float(point[1]) * scale)),
    )


def ray_mask(
    starts: np.ndarray,
    ends: np.ndarray,
    size: int,
    extent_m: float,
) -> np.ndarray:
    mask = np.zeros((size, size), dtype=np.uint8)
    rectangle = (0, 0, size, size)
    for start, end in zip(starts, ends):
        p0, p1 = pixel(start, size, extent_m), pixel(end, size, extent_m)
        clipped, q0, q1 = cv2.clipLine(rectangle, p0, p1)
        if clipped:
            cv2.line(mask, q0, q1, 255, 1, cv2.LINE_8)
    return mask


def base_panel(mask_01: np.ndarray, mask_02: np.ndarray, mode: str) -> np.ndarray:
    image = np.empty((*mask_01.shape, 3), dtype=np.uint8)
    image[:] = BACKGROUND
    first, second = mask_01 > 0, mask_02 > 0
    if mode == "scan_01":
        image[first] = BLUE
    elif mode == "scan_02":
        image[second] = ORANGE
    elif mode == "blue_then_orange":
        image[first] = BLUE
        image[second] = ORANGE
    elif mode == "orange_then_blue":
        image[second] = ORANGE
        image[first] = BLUE
    elif mode == "order_independent":
        image[first & ~second] = BLUE
        image[second & ~first] = ORANGE
        image[first & second] = PURPLE
    else:
        raise ValueError(mode)
    return image


def decorate_panel(
    image: np.ndarray,
    endpoints: list[np.ndarray],
    size: int,
    extent_m: float,
) -> np.ndarray:
    colors = [(170, 76, 0), (0, 72, 196)]
    for sensor, points in enumerate(endpoints):
        for point in points:
            px, py = pixel(point, size, extent_m)
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(image, (px, py), 1, colors[sensor], -1, cv2.LINE_8)
    hx, hy = 0.31237000644207, 0.2435245481133461
    p0 = pixel(np.asarray([-hx, -hy]), size, extent_m)
    p1 = pixel(np.asarray([hx, hy]), size, extent_m)
    cv2.rectangle(image, (p0[0], p1[1]), (p1[0], p0[1]), (120, 230, 245), 1)
    for topic, color in zip(SENSOR_POSES, colors):
        px, py = pixel(np.asarray(SENSOR_POSES[topic][:2]), size, extent_m)
        cv2.circle(image, (px, py), 3, color, -1, cv2.LINE_AA)
        cv2.circle(image, (px, py), 4, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def main() -> None:
    args = arguments()
    output_dir = args.output_dir.resolve()
    contact_path = output_dir / "dual_lidar_draw_order_contact_sheet.png"
    summary_path = output_dir / "draw_order_audit.json"
    if not args.overwrite and (contact_path.exists() or summary_path.exists()):
        raise FileExistsError(f"refusing to overwrite {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    modes = [
        ("scan_01", "scan_01 only"),
        ("scan_02", "scan_02 only"),
        ("blue_then_orange", "blue first, orange last"),
        ("orange_then_blue", "orange first, blue last"),
        ("order_independent", "fair: overlap = purple"),
    ]
    margin, header, row_label = 14, 88, 282
    cell_gap, row_height = 10, args.panel_size + 62
    width = row_label + len(modes) * (args.panel_size + cell_gap) + margin
    height = header + len(args.bag) * row_height + margin
    sheet = Image.new("RGB", (width, height), (11, 15, 21))
    draw = ImageDraw.Draw(sheet)
    draw.text((margin, 10), "Dual-LiDAR draw-order audit across rosbags", font=font(23), fill=(242, 246, 250))
    draw.text((margin, 39), "Same +/-8 m base frame; ray endpoints are bright; yellow box is robot footprint", font=font(13), fill=(185, 199, 216))
    for column, (_, title) in enumerate(modes):
        x = row_label + column * (args.panel_size + cell_gap)
        draw.text((x, header - 20), title, font=font(12), fill=(211, 220, 231))

    bag_summaries = []
    for row, bag in enumerate(args.bag):
        bag = bag.resolve()
        scans, metadata = load_middle_scans(bag)
        geometry = {}
        for topic in SENSOR_POSES:
            geometry[topic] = scan_points(scans[topic], SENSOR_POSES[topic])
        starts_01, ends_01, ranges_01, valid_01 = geometry["/scan_01"]
        starts_02, ends_02, ranges_02, valid_02 = geometry["/scan_02"]
        mask_01 = ray_mask(starts_01, ends_01, args.panel_size, args.extent_m)
        mask_02 = ray_mask(starts_02, ends_02, args.panel_size, args.extent_m)
        pixels_01, pixels_02 = mask_01 > 0, mask_02 > 0
        overlap = pixels_01 & pixels_02
        union = pixels_01 | pixels_02
        hx, hy = 0.31237000644207, 0.2435245481133461
        self_01 = int(np.count_nonzero((np.abs(ends_01[:, 0]) <= hx) & (np.abs(ends_01[:, 1]) <= hy)))
        self_02 = int(np.count_nonzero((np.abs(ends_02[:, 0]) <= hx) & (np.abs(ends_02[:, 1]) <= hy)))
        row_y = header + row * row_height
        short_name = bag.name if bag.name != "rosbag" else bag.parent.name
        draw.text((margin, row_y + 8), short_name[:36], font=font(14), fill=(240, 244, 248))
        draw.text((margin, row_y + 31), f"valid beams: {valid_01} + {valid_02}", font=font(12), fill=(192, 204, 219))
        draw.text((margin, row_y + 50), f"ray-pixel overlap: {int(overlap.sum())}/{int(union.sum())} ({overlap.sum()/max(1,union.sum()):.1%})", font=font(12), fill=(192, 204, 219))
        draw.text((margin, row_y + 69), f"endpoints in robot box: {self_01} + {self_02}", font=font(12), fill=(192, 204, 219))
        draw.text((margin, row_y + 88), f"range_min: {scans['/scan_01'].range_min:g}, {scans['/scan_02'].range_min:g} m", font=font(12), fill=(192, 204, 219))
        for column, (mode, _) in enumerate(modes):
            panel = base_panel(mask_01, mask_02, mode)
            shown_endpoints = {
                "scan_01": [ends_01, np.empty((0, 2))],
                "scan_02": [np.empty((0, 2)), ends_02],
            }.get(mode, [ends_01, ends_02])
            panel = decorate_panel(
                panel, shown_endpoints, args.panel_size, args.extent_m
            )
            panel_rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
            x = row_label + column * (args.panel_size + cell_gap)
            sheet.paste(Image.fromarray(panel_rgb), (x, row_y + 18))
        bag_summaries.append(
            {
                "bag": str(bag),
                "middle_scan": {
                    topic: {
                        "frame_id": scans[topic].header.frame_id,
                        "beam_count": len(scans[topic].ranges),
                        "valid_beams": geometry[topic][3],
                        "range_min_m": float(scans[topic].range_min),
                        "range_max_m": float(scans[topic].range_max),
                    }
                    for topic in SENSOR_POSES
                },
                "ray_pixels_scan_01": int(pixels_01.sum()),
                "ray_pixels_scan_02": int(pixels_02.sum()),
                "ray_pixels_overlap": int(overlap.sum()),
                "ray_pixels_union": int(union.sum()),
                "overlap_over_union": float(overlap.sum() / max(1, union.sum())),
                "draw_order_sensitive_pixels": int(overlap.sum()),
                "endpoints_inside_robot_box": {
                    "scan_01": self_01,
                    "scan_02": self_02,
                },
            }
        )
    sheet.save(contact_path)
    summary_path.write_text(
        json.dumps(
            {
                "schema": "dual_lidar_draw_order_audit/v1",
                "sensor_poses_base_link": SENSOR_POSES,
                "extent_m": args.extent_m,
                "diagnosis": "last-drawn color owns every overlapping ray pixel in the two ordered panels; the fair panel marks overlap purple",
                "bags": bag_summaries,
                "contact_sheet": str(contact_path),
            },
            indent=2,
        )
        + "\n"
    )
    print(contact_path)
    print(summary_path)


if __name__ == "__main__":
    main()
