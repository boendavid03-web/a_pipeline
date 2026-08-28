#!/usr/bin/env python3
"""Render all automatically collected navigation episodes on one map.

The bag reader intentionally consumes only the episode/goal/path/odom/TF/clock
topics needed by this report.  In particular, it does not use a merged scan.
ROS 2 imports are kept inside ``read_bag`` so the map and episode helpers can
be unit-tested on machines that do not have ROS sourced.
"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import html
import io
import json
import math
import os
import sys
from bisect import bisect_left
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ROS_TOOLS = PROJECT_ROOT / "workspaces" / "ros2_ws" / "tools"
VALIDATION_TOOLS = PROJECT_ROOT / "scripts" / "validation" / "ros2_workspace_tools"
sys.path.insert(0, str(PROJECT_ROOT / "workspaces" / "ros2_ws" / "src" / "semantic_nav_gazebo" / "scripts"))

SUCCESS_REASONS = frozenset(("goal_reached_and_stopped", "goal_tolerance_reached"))
PATH_GOAL_TOLERANCE_M = 0.5
REQUIRED_TOPICS = (
    "/data_collection/episode_event",
    "/data_collection/goal_accepted",
    "/semantic_cnn/global_path",
    "/semantic_cnn/final_goal",
    "/odom",
    "/tf",
    "/tf_static",
    "/clock",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def path_length(points) -> float | None:
    if points is None or len(points) < 2:
        return 0.0 if points is not None and len(points) == 1 else None
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])))


def map_world_to_pixel(x: float, y: float, width: int, height: int, resolution: float,
                       origin_x: float, origin_y: float) -> tuple[float, float]:
    """Continuous map-image pixel coordinates, with the project's Y flip."""
    return ((float(x) - origin_x) / resolution,
            height - (float(y) - origin_y) / resolution)


def map_pixel_to_world(px: float, py: float, width: int, height: int, resolution: float,
                       origin_x: float, origin_y: float) -> tuple[float, float]:
    return (origin_x + float(px) * resolution,
            origin_y + (height - float(py)) * resolution)


def load_map_geometry(map_yaml: Path) -> dict:
    """Load map metadata using the same YAML/image convention as navigation core."""
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = Path(metadata["image"]).expanduser()
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    origin = metadata["origin"]
    return {
        "yaml": map_yaml.resolve(),
        "image_path": image_path.resolve(),
        "image": image,
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "resolution": float(metadata["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "origin_yaw": float(origin[2]) if len(origin) > 2 else 0.0,
        "negate": int(metadata.get("negate", 0)),
    }


def build_episode_intervals(events: list[dict], sim_end_ns: int) -> list[dict]:
    """Build ordered intervals, retaining a final start without an end as incomplete."""
    relevant = []
    for event in events:
        if not isinstance(event, dict):
            raise RuntimeError("episode event payload is not a JSON object")
        # Older Isaac bridge bags can contain command-dwell teleop markers on
        # the same topic.  They explicitly have no navigation goal and must
        # neither masquerade as nor invalidate formal auto-capture episodes.
        if event.get("schema") == "isaac_manual_teleop_episode/v1":
            continue
        if event.get("schema") != "semantic_nav_episode_event/v1":
            raise RuntimeError(f"unsupported episode event schema: {event.get('schema')!r}")
        if event.get("event") not in ("armed", "start", "end", "ready"):
            raise RuntimeError(f"unknown episode event kind: {event.get('event')!r}")
        if event.get("event") in ("start", "end"):
            relevant.append(event)
    relevant.sort(key=lambda item: int(item.get("stamp_ns", -1)))
    intervals = []
    active = None
    previous_id = 0
    for event in relevant:
        try:
            episode_id = int(event["episode_id"])
            stamp_ns = int(event["stamp_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("episode event has invalid id or timestamp") from exc
        if episode_id <= 0 or stamp_ns < 0:
            raise RuntimeError("episode id must be positive and timestamp non-negative")
        if event["event"] == "start":
            if active is not None:
                raise RuntimeError(f"episode {episode_id} starts before episode {active['episode_id']} ends")
            if episode_id <= previous_id:
                raise RuntimeError("episode ids are not strictly increasing")
            goal = event.get("goal")
            if (not isinstance(goal, list) or len(goal) != 2 or
                    not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in goal)):
                raise RuntimeError(f"episode {episode_id} start has an invalid goal")
            active = {
                "episode_id": episode_id,
                "start_stamp_ns": stamp_ns,
                "end_stamp_ns": None,
                "goal": [float(goal[0]), float(goal[1])],
                "start_pose": event.get("pose"),
                "end_pose": None,
                "reason": None,
                "has_end": False,
            }
        else:
            if active is None or episode_id != active["episode_id"]:
                raise RuntimeError(f"episode end id {episode_id} does not match the active episode")
            if stamp_ns <= active["start_stamp_ns"]:
                raise RuntimeError(f"episode {episode_id} end is not after its start")
            active["end_stamp_ns"] = stamp_ns
            active["end_pose"] = event.get("pose")
            active["reason"] = event.get("reason")
            active["has_end"] = True
            intervals.append(active)
            previous_id = episode_id
            active = None
    if active is not None:
        active["end_stamp_ns"] = max(int(sim_end_ns), int(active["start_stamp_ns"]))
        active["reason"] = "incomplete_missing_end_event"
        intervals.append(active)
    return intervals


def _interp_storage_clock(clocks: list[tuple[int, int]], storage_ns: int) -> int:
    if not clocks:
        raise RuntimeError("/clock is required to map episode event timestamps")
    values = sorted((int(a), int(b)) for a, b in clocks)
    if storage_ns < values[0][0] or storage_ns > values[-1][0]:
        raise RuntimeError("episode event storage timestamp is outside /clock range")
    times = [item[0] for item in values]
    index = bisect_left(times, int(storage_ns))
    if index == 0:
        return values[0][1]
    if index == len(values):
        return values[-1][1]
    before, after = values[index - 1], values[index]
    if after[0] == before[0]:
        return before[1]
    fraction = (storage_ns - before[0]) / float(after[0] - before[0])
    return int(round(before[1] + fraction * (after[1] - before[1])))


def _transform_xy(tf, x: float, y: float) -> tuple[float, float]:
    tx, ty, yaw = tf
    c, s = math.cos(yaw), math.sin(yaw)
    return (tx + c * float(x) - s * float(y), ty + s * float(x) + c * float(y))


def _load_ros_helpers():
    sys.path.insert(0, str(VALIDATION_TOOLS))
    # Both directories contain a historical file with the same basename.
    # The workspace tools version is the canonical ROS bag/TF implementation.
    sys.path.insert(0, str(ROS_TOOLS))
    import rosbag2_py  # type: ignore
    from rclpy.serialization import deserialize_message  # type: ignore
    from rosidl_runtime_py.utilities import get_message  # type: ignore
    from convert_rosbag2_to_semantic2d_native_lidar import (  # type: ignore
        TfIndex, msg_time_ns, normalize_frame, stamp_to_ns, tf_from_transform_stamped,
    )
    from v7_rosbag_to_fixed_dual_lidar_dataset import map_episode_events_to_sim_time  # type: ignore
    return (rosbag2_py, deserialize_message, get_message, TfIndex, msg_time_ns,
            normalize_frame, stamp_to_ns, tf_from_transform_stamped,
            map_episode_events_to_sim_time)


def read_bag(bag: Path, map_geometry: dict, allow_identity_map_odom: bool = False) -> dict:
    (rosbag2_py, deserialize_message, get_message, TfIndex, msg_time_ns,
     normalize_frame, stamp_to_ns, tf_from_transform_stamped,
     map_episode_events_to_sim_time) = _load_ros_helpers()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in REQUIRED_TOPICS if topic not in topic_types]
    if missing:
        raise RuntimeError("bag is missing required topic(s): " + ", ".join(missing))
    expected_types = {
        "/data_collection/episode_event": "std_msgs/msg/String",
        "/data_collection/goal_accepted": "geometry_msgs/msg/PointStamped",
        "/semantic_cnn/global_path": "nav_msgs/msg/Path",
        "/semantic_cnn/final_goal": "geometry_msgs/msg/PointStamped",
        "/odom": "nav_msgs/msg/Odometry",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/clock": "rosgraph_msgs/msg/Clock",
    }
    wrong = [f"{topic}={topic_types[topic]!r}" for topic in REQUIRED_TOPICS
             if topic_types[topic] != expected_types[topic]]
    if wrong:
        raise RuntimeError("required topic type mismatch: " + "; ".join(wrong))
    message_types = {topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS}
    events, accepted, paths, final_goals, odoms, clocks = [], [], [], [], [], []
    tf_index = TfIndex()
    topic_counts = {topic: 0 for topic in topic_types}
    while reader.has_next():
        topic, raw, storage_ns = reader.read_next()
        if topic not in message_types:
            continue
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        msg = deserialize_message(raw, message_types[topic])
        if topic == "/data_collection/episode_event":
            payload = json.loads(msg.data)
            payload["_storage_stamp_ns"] = int(storage_ns)
            events.append(payload)
        elif topic == "/data_collection/goal_accepted":
            accepted.append((msg_time_ns(msg, storage_ns), normalize_frame(msg.header.frame_id),
                             (float(msg.point.x), float(msg.point.y))))
        elif topic == "/semantic_cnn/global_path":
            points = [(float(item.pose.position.x), float(item.pose.position.y)) for item in msg.poses]
            if points:
                paths.append((msg_time_ns(msg, storage_ns), normalize_frame(msg.header.frame_id), points))
        elif topic == "/semantic_cnn/final_goal":
            final_goals.append((msg_time_ns(msg, storage_ns), normalize_frame(msg.header.frame_id),
                                (float(msg.point.x), float(msg.point.y))))
        elif topic == "/odom":
            pose = msg.pose.pose
            odoms.append((msg_time_ns(msg, storage_ns), normalize_frame(msg.header.frame_id),
                          (float(pose.position.x), float(pose.position.y))))
        elif topic == "/clock":
            clocks.append((int(storage_ns), stamp_to_ns(msg.clock)))
        else:
            for transform in msg.transforms:
                tf_index.add(transform.header.frame_id, transform.child_frame_id,
                             msg_time_ns(transform, storage_ns),
                             tf_from_transform_stamped(transform),
                             is_static=(topic == "/tf_static"))
    def sort_stamp(item):
        if isinstance(item, dict):
            return int(item.get("stamp_ns", -1))
        return int(item[0])

    for values in (events, accepted, paths, final_goals, odoms, clocks):
        values.sort(key=sort_stamp)
    tf_index.finalize()
    sim_clock = [item[1] for item in clocks]
    sim_start_ns, sim_end_ns = min(sim_clock), max(sim_clock)
    raw_event_count = len(events)
    ignored_manual_event_count = sum(
        event.get("schema") == "isaac_manual_teleop_episode/v1"
        for event in events
    )
    mapped_events, mapping_summary = map_episode_events_to_sim_time(events, clocks)
    intervals = build_episode_intervals(mapped_events, sim_end_ns)

    transformed_odom = []
    tf_failures = []
    tf_samples = []
    for stamp_ns, frame, point in odoms:
        if frame == "map":
            tf = (0.0, 0.0, 0.0)
        else:
            tf = tf_index.lookup("map", frame, stamp_ns)
        if tf is None and allow_identity_map_odom and frame == "odom":
            tf = (0.0, 0.0, 0.0)
        if tf is None:
            tf_failures.append((stamp_ns, frame))
        else:
            transformed_odom.append((stamp_ns, _transform_xy(tf, *point)))
            if frame == "odom":
                tf_samples.append(tf)
    if tf_failures:
        raise RuntimeError(f"could not validate map->{tf_failures[0][1]} TF for {len(tf_failures)} odom samples")

    transformed_paths = []
    for stamp_ns, frame, points in paths:
        if frame in ("", "map"):
            tf = (0.0, 0.0, 0.0)
        else:
            tf = tf_index.lookup("map", frame, stamp_ns)
            if tf is None and allow_identity_map_odom and frame == "odom":
                tf = (0.0, 0.0, 0.0)
            if tf is None:
                raise RuntimeError(f"could not validate map->{frame} TF for global path at {stamp_ns}")
        transformed_paths.append((stamp_ns, [_transform_xy(tf, *point) for point in points]))

    # The event helper is the authoritative storage->simulation mapping.  Goal
    # and path/odom header stamps are already simulation-clock stamps in ROS 2.
    previous_end_ns = sim_start_ns
    for interval in intervals:
        start_ns = int(interval["start_stamp_ns"])
        goal = tuple(interval["goal"])
        # Goal acceptance normally precedes the start event by about one
        # second.  Bound the lookup to this episode's transition window so a
        # repeated goal cannot accidentally select a future or stale event.
        transition_end_ns = min(int(interval["end_stamp_ns"]),
                                start_ns + 2_000_000_000)
        window = [item for item in accepted
                  if previous_end_ns <= item[0] <= transition_end_ns]
        matching = [item for item in window
                    if math.hypot(item[2][0] - goal[0], item[2][1] - goal[1])
                    <= PATH_GOAL_TOLERANCE_M]
        candidates = matching or window
        selected = max(candidates, key=lambda item: item[0]) if candidates else None
        interval["goal_accepted_stamp_ns"] = selected[0] if selected else None
        interval["goal_accepted_goal_consistent"] = bool(selected and selected in matching)
        interval["association_window_start_ns"] = int(previous_end_ns)
        previous_end_ns = int(interval["end_stamp_ns"])
    return {
        "events": mapped_events,
        "raw_event_count": raw_event_count,
        "ignored_manual_event_count": ignored_manual_event_count,
        "intervals": intervals,
        "accepted": accepted,
        "paths": transformed_paths,
        "final_goals": final_goals,
        "odom": transformed_odom,
        "clocks": clocks,
        "sim_start_ns": int(sim_start_ns),
        "sim_end_ns": int(sim_end_ns),
        "event_mapping": mapping_summary,
        "topic_types": topic_types,
        "topic_counts": topic_counts,
        "tf_index": tf_index,
        "tf_samples": tf_samples,
        "identity_fallback": bool(allow_identity_map_odom and not tf_samples),
    }


def _episode_data(bag_data: dict) -> list[dict]:
    odom = bag_data["odom"]
    paths = bag_data["paths"]
    result = []
    for interval in bag_data["intervals"]:
        episode_id = int(interval["episode_id"])
        start_ns, end_ns = int(interval["start_stamp_ns"]), int(interval["end_stamp_ns"])
        actual = [point for stamp, point in odom if start_ns <= stamp <= end_ns]
        accepted_ns = interval.get("goal_accepted_stamp_ns")
        # The global path is commonly first published between goal acceptance
        # and the episode start.  Require its endpoint to agree with this
        # episode's goal; this rejects a latched path from the prior episode.
        lower_ns = (int(accepted_ns) if accepted_ns is not None
                    else int(interval.get("association_window_start_ns", start_ns)))
        candidates = [(stamp, points) for stamp, points in paths
                      if lower_ns <= stamp <= end_ns]
        consistent = [(stamp, points) for stamp, points in candidates
                      if math.hypot(points[-1][0] - interval["goal"][0],
                                    points[-1][1] - interval["goal"][1])
                      <= PATH_GOAL_TOLERANCE_M]
        selected_path = consistent[0] if consistent else None
        planned_stamp_ns = selected_path[0] if selected_path else None
        planned = selected_path[1] if selected_path else []
        start_xy = actual[0] if actual else (tuple(interval["start_pose"][:2]) if isinstance(interval.get("start_pose"), list) else None)
        end_xy = actual[-1] if actual else (tuple(interval["end_pose"][:2]) if isinstance(interval.get("end_pose"), list) else None)
        goal_xy = tuple(interval["goal"])
        planned_len, actual_len = path_length(planned), path_length(actual)
        straight = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1]) if start_xy else None
        min_goal = min((math.hypot(point[0] - goal_xy[0], point[1] - goal_xy[1]) for point in actual), default=None)
        reason = interval.get("reason") or "incomplete_missing_end_event"
        status = "success" if reason in SUCCESS_REASONS else ("incomplete" if not interval["has_end"] else "failed")
        result.append({
            "episode_id": episode_id,
            "status": status,
            "reason": reason,
            "start_xy": list(start_xy) if start_xy else None,
            "end_xy": list(end_xy) if end_xy else None,
            "goal_xy": list(goal_xy),
            "planned": planned,
            "actual": actual,
            "planned_path_length_m": planned_len,
            "actual_path_length_m": actual_len,
            "duration_sec": max(0.0, (end_ns - start_ns) / 1e9),
            "minimum_goal_distance_m": min_goal,
            "planned_path_efficiency": (straight / planned_len if straight is not None and planned_len and planned_len > 0 else None),
            "actual_path_efficiency": (straight / actual_len if straight is not None and actual_len and actual_len > 0 else None),
            "actual_vs_planned_efficiency": (planned_len / actual_len if planned_len and actual_len and actual_len > 0 else None),
            "odom_sample_count": len(actual),
            "global_path_sample_count": len(planned),
            "planned_path_sample_count": len(planned),
            "start_stamp_ns": start_ns,
            "end_stamp_ns": end_ns,
            "has_end": bool(interval["has_end"]),
            "goal_accepted_stamp_ns": accepted_ns,
            "goal_accepted_goal_consistent": bool(interval.get("goal_accepted_goal_consistent")),
            "planned_path_stamp_ns": planned_stamp_ns,
            "planned_path_goal_distance_m": (
                math.hypot(planned[-1][0] - goal_xy[0], planned[-1][1] - goal_xy[1])
                if planned else None
            ),
            "data_quality": {
                "has_end_event": bool(interval["has_end"]),
                "has_goal_acceptance": accepted_ns is not None,
                "goal_acceptance_matches_episode_goal": bool(interval.get("goal_accepted_goal_consistent")),
                "has_planned_path": bool(planned),
                "planned_path_endpoint_matches_goal": bool(planned),
                "has_actual_odom_path": bool(actual),
            },
        })
    return result


def _fmt(value, digits=3):
    return "—" if value is None else f"{float(value):.{digits}f}"


def _color(index):
    return matplotlib.colors.to_hex(plt.get_cmap("tab10")(index % 10))


def _arrow_indices(points, count=3):
    if len(points) < 2:
        return []
    return np.linspace(0, len(points) - 2, min(count, len(points) - 1), dtype=int).tolist()


def render_static(episodes: list[dict], map_geometry: dict, semantic_path: Path | None,
                  semantic_alpha: float, stride: int, dpi: int, output_png: Path, output_svg: Path,
                  title: str) -> None:
    fig, ax = plt.subplots(figsize=(14, 10), constrained_layout=True)
    image = map_geometry["image"]
    extent = (map_geometry["origin_x"], map_geometry["origin_x"] + map_geometry["width"] * map_geometry["resolution"],
              map_geometry["origin_y"], map_geometry["origin_y"] + map_geometry["height"] * map_geometry["resolution"])
    ax.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255, interpolation="nearest", zorder=0)
    if semantic_path is not None:
        label = np.asarray(Image.open(semantic_path).convert("P"))
        if label.shape != image.shape:
            raise RuntimeError(f"semantic label shape {label.shape} does not match map {image.shape}")
        ax.imshow(label, cmap="tab20", origin="upper", extent=extent, alpha=semantic_alpha,
                  interpolation="nearest", zorder=1)
    handles = [Line2D([0], [0], color="#333333", lw=2, ls="--", label="planned"),
               Line2D([0], [0], color="#333333", lw=2, label="actual"),
               Line2D([0], [0], marker="o", color="none", markeredgecolor="#333333", label="start"),
               Line2D([0], [0], marker="*", color="none", markeredgecolor="#333333", label="goal"),
               Line2D([0], [0], marker="*", color="none", markeredgecolor="green", label="success"),
               Line2D([0], [0], marker="x", color="red", label="failed"),
               Patch(facecolor="orange", edgecolor="orange", alpha=.8, label="incomplete")]
    for index, episode in enumerate(episodes):
        color = _color(index)
        planned = episode["planned"][::max(1, stride)]
        actual = episode["actual"][::max(1, stride)]
        if episode["planned"] and planned[-1] != episode["planned"][-1]: planned.append(episode["planned"][-1])
        if episode["actual"] and actual[-1] != episode["actual"][-1]: actual.append(episode["actual"][-1])
        if len(planned) > 1:
            ax.plot([p[0] for p in planned], [p[1] for p in planned], "--", color=color, lw=1.8, alpha=.9, zorder=3)
            for i in _arrow_indices(planned):
                ax.annotate("", xy=planned[i + 1], xytext=planned[i], arrowprops={"arrowstyle": "->", "color": color, "lw": 1.0}, zorder=4)
        if len(actual) > 1:
            ax.plot([p[0] for p in actual], [p[1] for p in actual], "-", color=color, lw=2.2, alpha=.95, zorder=4)
            for i in _arrow_indices(actual):
                ax.annotate("", xy=actual[i + 1], xytext=actual[i], arrowprops={"arrowstyle": "->", "color": color, "lw": 1.0}, zorder=5)
        if episode["start_xy"]:
            sx, sy = episode["start_xy"]
            ax.scatter([sx], [sy], s=105, facecolors="white", edgecolors=color, linewidths=2, zorder=7)
            ax.annotate(f"S{episode['episode_id']}", (sx, sy), xytext=(6 + 5 * (index % 2), 7 + 5 * (index % 3)), textcoords="offset points", color=color, fontsize=9, weight="bold", zorder=8)
        gx, gy = episode["goal_xy"]
        edge = "green" if episode["status"] == "success" else ("orange" if episode["status"] == "incomplete" else "red")
        ax.scatter([gx], [gy], s=170, marker="*", facecolors="white", edgecolors=edge, linewidths=2, zorder=7)
        if episode["status"] == "failed":
            ax.scatter([gx], [gy], s=110, marker="x", color="red", linewidths=2, zorder=8)
        ax.annotate(f"G{episode['episode_id']}", (gx, gy), xytext=(7 + 5 * (index % 2), -14 - 5 * (index % 3)), textcoords="offset points", color=edge, fontsize=9, weight="bold", zorder=8)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(title, fontsize=13)
    ax.legend(handles=handles, loc="upper left", framealpha=.92, fontsize=9)
    fig.savefig(output_png, dpi=dpi)
    fig.savefig(output_svg)
    plt.close(fig)


def render_episode_png(episode: dict, map_geometry: dict, semantic_path: Path | None,
                       semantic_alpha: float, stride: int, dpi: int, output: Path) -> None:
    """Render one episode even when one of its trajectory sources is absent."""
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    image = map_geometry["image"]
    extent = (
        map_geometry["origin_x"],
        map_geometry["origin_x"] + map_geometry["width"] * map_geometry["resolution"],
        map_geometry["origin_y"],
        map_geometry["origin_y"] + map_geometry["height"] * map_geometry["resolution"],
    )
    ax.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255,
              interpolation="nearest", zorder=0)
    if semantic_path is not None:
        label = np.asarray(Image.open(semantic_path).convert("P"))
        if label.shape != image.shape:
            raise RuntimeError(f"semantic label shape {label.shape} does not match map {image.shape}")
        ax.imshow(label, cmap="tab20", origin="upper", extent=extent, alpha=semantic_alpha,
                  interpolation="nearest", zorder=1)

    planned = episode["planned"][::max(1, stride)]
    actual = episode["actual"][::max(1, stride)]
    if episode["planned"] and planned[-1] != episode["planned"][-1]:
        planned.append(episode["planned"][-1])
    if episode["actual"] and actual[-1] != episode["actual"][-1]:
        actual.append(episode["actual"][-1])
    if len(planned) > 1:
        ax.plot([p[0] for p in planned], [p[1] for p in planned], "--",
                color="#2b6cb0", lw=2.0, label="global planned path", zorder=3)
    if len(actual) > 1:
        ax.plot([p[0] for p in actual], [p[1] for p in actual], "-",
                color="#d94801", lw=2.2, label="actual odom path", zorder=4)
    if episode["start_xy"]:
        sx, sy = episode["start_xy"]
        ax.scatter([sx], [sy], s=90, marker="o", facecolors="white",
                   edgecolors="#111111", linewidths=1.8, label="start", zorder=6)
    gx, gy = episode["goal_xy"]
    ax.scatter([gx], [gy], s=150, marker="*", facecolors="#f6e05e",
               edgecolors="#111111", linewidths=1.4, label="goal", zorder=6)
    if episode["end_xy"]:
        ex, ey = episode["end_xy"]
        end_color = {"success": "#238b45", "failed": "#d62728",
                     "incomplete": "#f28e2b"}[episode["status"]]
        ax.scatter([ex], [ey], s=85, marker="X", color=end_color,
                   edgecolors="white", linewidths=.8, label="recorded end", zorder=7)

    missing = []
    if not planned:
        missing.append("global planned path missing")
    if not actual:
        missing.append("actual odom path missing")
    details = (f"reason={episode['reason']} | duration={episode['duration_sec']:.2f} s | "
               f"planned={_fmt(episode['planned_path_length_m'])} m | "
               f"actual={_fmt(episode['actual_path_length_m'])} m")
    if missing:
        details += "\nDATA: " + "; ".join(missing)
    ax.set_title(f"Episode {episode['episode_id']:04d} — {episode['status']}\n{details}", fontsize=10)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc="upper left", framealpha=.92, fontsize=8)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def render_episode_index(episodes: list[dict], output: Path, bag_name: str) -> None:
    status_counts = Counter(item["status"] for item in episodes)
    reason_counts = Counter(item["reason"] for item in episodes)
    rows = []
    for episode in episodes:
        quality = episode["data_quality"]
        quality_text = ", ".join(
            label for key, label in (
                ("has_end_event", "end event"),
                ("has_planned_path", "planned path"),
                ("has_actual_odom_path", "odom path"),
            ) if quality[key]
        ) or "no trajectory data"
        image_path = html.escape(episode["episode_png"], quote=True)
        rows.append(
            "<tr>"
            f"<td>{episode['episode_id']}</td><td>{html.escape(episode['status'])}</td>"
            f"<td>{html.escape(str(episode['reason']))}</td>"
            f"<td>{episode['duration_sec']:.2f}</td>"
            f"<td>{_fmt(episode['planned_path_length_m'])}</td>"
            f"<td>{_fmt(episode['actual_path_length_m'])}</td>"
            f"<td>{html.escape(quality_text)}</td>"
            f"<td><a href=\"{image_path}\"><img src=\"{image_path}\" alt=\"episode {episode['episode_id']}\"></a></td>"
            "</tr>"
        )
    reasons = "".join(
        f"<li><code>{html.escape(str(reason))}</code>: {count}</li>"
        for reason, count in sorted(reason_counts.items())
    )
    doc = f'''<!doctype html>
<html><head><meta charset="utf-8"><title>{html.escape(bag_name)} episode index</title>
<style>body{{font-family:system-ui,sans-serif;margin:24px;color:#222}} table{{border-collapse:collapse;width:100%}} th,td{{border:1px solid #ccc;padding:6px;text-align:left;vertical-align:top}} th{{position:sticky;top:0;background:#eee}} img{{width:260px;height:auto}} code{{white-space:nowrap}}</style></head>
<body><h1>{html.escape(bag_name)} — episode trajectories</h1>
<p>Total {len(episodes)}; success {status_counts['success']}; failed {status_counts['failed']}; incomplete {status_counts['incomplete']}.</p>
<h2>End reasons</h2><ul>{reasons}</ul>
<table><thead><tr><th>ID</th><th>Status</th><th>Reason</th><th>Duration (s)</th><th>Planned (m)</th><th>Actual (m)</th><th>Data</th><th>Visualization</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></body></html>'''
    output.write_text(doc, encoding="utf-8")


def _png_data_uri(image: Image.Image) -> str:
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def _svg_path(points, geometry):
    if not points:
        return ""
    coords = [map_world_to_pixel(x, y, geometry["width"], geometry["height"], geometry["resolution"], geometry["origin_x"], geometry["origin_y"]) for x, y in points]
    return "M " + " L ".join(f"{x:.3f},{y:.3f}" for x, y in coords)


def render_html(episodes: list[dict], geometry: dict, semantic_path: Path | None, semantic_alpha: float,
                stride: int, output: Path, title: str) -> None:
    width, height = geometry["width"], geometry["height"]
    occupancy_uri = _png_data_uri(Image.fromarray(geometry["image"], mode="L"))
    label_uri = _png_data_uri(Image.open(semantic_path).convert("RGBA")) if semantic_path else ""
    groups = []
    controls = []
    for index, episode in enumerate(episodes):
        eid, color = episode["episode_id"], _color(index)
        planned = episode["planned"][::max(1, stride)]
        actual = episode["actual"][::max(1, stride)]
        for source, points, dash in (("planned", planned, "stroke-dasharray=5 3"), ("actual", actual, "")):
            if points:
                tip = html.escape(
                    f"episode {eid} {source}; time={episode['start_stamp_ns'] / 1e9:.3f}..{episode['end_stamp_ns'] / 1e9:.3f} s; "
                    f"start={episode['start_xy']}; goal={episode['goal_xy']}; reason={episode['reason']}",
                    quote=True,
                )
                groups.append(f'<path class="ep ep-{eid} {source}" data-episode="{eid}" d="{_svg_path(points, geometry)}" fill="none" stroke="{color}" stroke-width="2" {dash}><title>{tip}</title></path>')
        sx = episode["start_xy"]
        if sx:
            px, py = map_world_to_pixel(*sx, width, height, geometry["resolution"], geometry["origin_x"], geometry["origin_y"])
            groups.append(f'<circle class="marker ep-{eid}" data-episode="{eid}" cx="{px:.3f}" cy="{py:.3f}" r="4" fill="white" stroke="{color}" stroke-width="2"><title>S{eid}: {sx[0]:.3f}, {sx[1]:.3f}</title></circle>')
        gx, gy = episode["goal_xy"]
        px, py = map_world_to_pixel(gx, gy, width, height, geometry["resolution"], geometry["origin_x"], geometry["origin_y"])
        edge = "#238b45" if episode["status"] == "success" else ("#f28e2b" if episode["status"] == "incomplete" else "#d62728")
        reason_text = html.escape(str(episode["reason"]), quote=True)
        groups.append(f'<text class="label ep-{eid}" data-episode="{eid}" x="{px + 6:.3f}" y="{py - 6:.3f}" fill="{edge}">G{eid}</text>')
        groups.append(f'<path class="goal ep-{eid}" data-episode="{eid}" d="M {px:.3f},{py-6:.3f} L {px+5.7:.3f},{py+4.8:.3f} L {px-5.7:.3f},{py-1.8:.3f} L {px+5.7:.3f},{py-1.8:.3f} L {px-5.7:.3f},{py+4.8:.3f} Z" fill="white" stroke="{edge}" stroke-width="2"><title>G{eid}: {gx:.3f}, {gy:.3f}; {reason_text}</title></path>')
        if episode["status"] == "failed":
            groups.append(f'<path class="goal ep-{eid}" data-episode="{eid}" d="M {px-4:.3f},{py-4:.3f} L {px+4:.3f},{py+4:.3f} M {px+4:.3f},{py-4:.3f} L {px-4:.3f},{py+4:.3f}" stroke="red" stroke-width="2"/>')
        controls.append(f'<label class="episode-row"><input type="checkbox" checked data-episode-toggle="{eid}"> <span style="color:{color}">Episode {eid}</span> — {episode["status"]}</label>')
    legend = "".join(f'<button class="legend-item" data-toggle-class="{name}"><span class="swatch {name}"></span>{name}</button>' for name in ("planned", "actual"))
    doc = f'''<!doctype html>
<html><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>body{{font-family:system-ui,sans-serif;margin:0;background:#f5f5f5;color:#222}} main{{display:flex;gap:14px;padding:14px}} aside{{width:220px;background:white;padding:12px;border-radius:6px;box-shadow:0 1px 4px #aaa;height:max-content}} svg{{background:#ddd;max-width:calc(100vw - 280px);height:auto;image-rendering:auto}} .episode-row,.legend-item{{display:block;margin:7px 0;cursor:pointer}} .legend-item{{border:0;background:none;padding:0;font:inherit;text-align:left}} .swatch{{display:inline-block;width:22px;border-top:3px solid #333;margin:0 6px 3px 0}} .swatch.planned{{border-top-style:dashed}} .swatch.actual{{border-top-style:solid}} .muted{{opacity:.28}} text{{font-size:10px;font-weight:700;pointer-events:none}}</style></head>
<body><main><aside><h3>Trajectory controls</h3><label><input type="checkbox" checked id="show-planned"> planned</label><label><input type="checkbox" checked id="show-actual"> actual</label><hr>{legend}<hr>{''.join(controls)}</aside>
<svg id="map" viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg"><image href="{occupancy_uri}" x="0" y="0" width="{width}" height="{height}" preserveAspectRatio="none"/>{f'<image id="semantic" href="{label_uri}" x="0" y="0" width="{width}" height="{height}" opacity="{semantic_alpha}" preserveAspectRatio="none"/>' if semantic_path else ''}{''.join(groups)}</svg></main>
<script>
const q=s=>document.querySelector(s), qa=s=>document.querySelectorAll(s);
function update(){{const planned=q('#show-planned').checked, actual=q('#show-actual').checked; qa('.planned').forEach(e=>e.classList.toggle('muted',!planned)); qa('.actual').forEach(e=>e.classList.toggle('muted',!actual)); qa('[data-episode-toggle]').forEach(c=>qa('.ep-'+c.dataset.episodeToggle).forEach(e=>e.classList.toggle('muted',!c.checked)));}}
q('#show-planned').onchange=update; q('#show-actual').onchange=update; qa('[data-episode-toggle]').forEach(e=>e.onchange=update); qa('[data-toggle-class]').forEach(e=>e.onclick=()=>{{const name=e.dataset.toggleClass, hidden=qa('.'+name)[0].classList.contains('muted'); qa('.'+name).forEach(x=>x.classList.toggle('muted',hidden));}}); update();
</script></body></html>'''
    output.write_text(doc, encoding="utf-8")


def write_metrics(episodes: list[dict], output: Path) -> None:
    fields = ["episode_id", "status", "reason", "start_xy", "goal_xy", "end_xy", "planned_path_length_m", "actual_path_length_m", "duration_sec", "minimum_goal_distance_m", "planned_path_efficiency", "actual_path_efficiency", "actual_vs_planned_efficiency", "odom_sample_count", "global_path_sample_count", "episode_png", "has_end_event", "has_goal_acceptance", "goal_acceptance_matches_episode_goal", "has_planned_path", "planned_path_endpoint_matches_goal", "has_actual_odom_path"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for episode in episodes:
            row = {field: episode.get(field) for field in fields}
            row.update(episode["data_quality"])
            row["start_xy"] = json.dumps(row["start_xy"], separators=(",", ":"))
            row["goal_xy"] = json.dumps(row["goal_xy"], separators=(",", ":"))
            row["end_xy"] = json.dumps(row["end_xy"], separators=(",", ":"))
            writer.writerow(row)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-label", required=True, type=Path)
    parser.add_argument("--status-json", type=Path,
                        help="optional scheduler status; omitted for interrupted/failed captures")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--successful-only", action="store_true")
    parser.add_argument("--episode-ids", default="", help="comma-separated source episode IDs")
    parser.add_argument("--semantic-alpha", type=float, default=0.22)
    parser.add_argument("--trajectory-stride", type=int, default=1)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--no-interactive", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--allow-identity-map-odom", action="store_true", help="explicitly allow identity fallback if bag has no map->odom TF")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    if not 0.0 <= args.semantic_alpha <= 1.0:
        raise ValueError("--semantic-alpha must be in [0, 1]")
    if args.trajectory_stride < 1 or args.dpi < 1:
        raise ValueError("--trajectory-stride and --dpi must be positive")
    bag, map_yaml, semantic = [path.expanduser().resolve()
                               for path in (args.bag, args.map_yaml, args.semantic_label)]
    status = args.status_json.expanduser().resolve() if args.status_json is not None else None
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    names = ("trajectory_overview.png", "trajectory_overview.svg", "trajectory_overview.html",
             "trajectory_summary.json", "episode_metrics.csv", "episode_index.html")
    existing = [output_dir / name for name in names if (output_dir / name).exists()]
    if (output_dir / "episodes").exists():
        existing.append(output_dir / "episodes")
    if existing and not args.overwrite:
        raise FileExistsError("refusing to overwrite existing output(s): " + ", ".join(str(p) for p in existing))
    geometry = load_map_geometry(map_yaml)
    if not semantic.is_file():
        raise FileNotFoundError(semantic)
    bag_data = read_bag(bag, geometry, args.allow_identity_map_odom)
    all_episodes = _episode_data(bag_data)
    selected_ids = {int(value) for value in args.episode_ids.split(",") if value.strip()} if args.episode_ids.strip() else None
    episodes = [item for item in all_episodes if (selected_ids is None or item["episode_id"] in selected_ids) and (not args.successful_only or item["status"] == "success")]
    if not episodes:
        raise RuntimeError("episode filters selected no episodes")
    status_data = None
    if status is not None:
        with status.open("r", encoding="utf-8") as stream:
            status_data = json.load(stream)
    successes = sum(item["status"] == "success" for item in all_episodes)
    failures = sum(item["status"] == "failed" for item in all_episodes)
    incompletes = sum(item["status"] == "incomplete" for item in all_episodes)
    sim_duration = (bag_data["sim_end_ns"] - bag_data["sim_start_ns"]) / 1e9
    title = (f"{bag.name} — {len(all_episodes)} episodes | success {successes} | failed {failures} | "
             f"incomplete {incompletes} | sim {sim_duration:.2f} s")
    render_static(episodes, geometry, semantic, args.semantic_alpha, args.trajectory_stride, args.dpi,
                  output_dir / "trajectory_overview.png", output_dir / "trajectory_overview.svg", title)
    html_output = None
    if not args.no_interactive:
        html_output = output_dir / "trajectory_overview.html"
        render_html(episodes, geometry, semantic, args.semantic_alpha, args.trajectory_stride, html_output, title)
    episodes_dir = output_dir / "episodes"
    episodes_dir.mkdir(parents=True, exist_ok=True)
    for episode in episodes:
        filename = f"episode_{episode['episode_id']:04d}_{episode['status']}.png"
        episode["episode_png"] = f"episodes/{filename}"
        render_episode_png(episode, geometry, semantic, args.semantic_alpha,
                           args.trajectory_stride, args.dpi, episodes_dir / filename)
    for episode in all_episodes:
        episode.setdefault("episode_png", None)
    episode_index = output_dir / "episode_index.html"
    render_episode_index(episodes, episode_index, bag.name)
    tf_index = bag_data["tf_index"]
    if bag_data["identity_fallback"]:
        tf_source = "explicit --allow-identity-map-odom fallback; no map->odom TF was available"
    elif ("map", "odom") in tf_index.static_edges:
        tf_source = "tf_static direct map->odom, validated for every /odom sample"
    else:
        tf_source = "TF graph map->odom, validated for every /odom sample"
    summary = {
        "schema": "auto_capture_trajectory_visualization/v1",
        "input": {"bag": str(bag), "map_yaml": str(map_yaml), "map_image": str(geometry["image_path"]), "semantic_label": str(semantic), "status_json": str(status) if status else None},
        "sha256": {"map_yaml": sha256_file(map_yaml), "map_image": sha256_file(geometry["image_path"]), "semantic_label": sha256_file(semantic)},
        "map": {"resolution_m_per_pixel": geometry["resolution"], "origin": [geometry["origin_x"], geometry["origin_y"], geometry["origin_yaw"]], "size_px": [geometry["width"], geometry["height"]], "y_axis": "image row is flipped from increasing map y"},
        "coordinate_transform": {"map_frame": "map", "odom_frame": "odom", "source": tf_source, "identity_fallback": bag_data["identity_fallback"], "tf_sample_count": len(bag_data["tf_samples"])},
        "topics": {"read": list(REQUIRED_TOPICS), "counts": {topic: bag_data["topic_counts"].get(topic, 0) for topic in REQUIRED_TOPICS}, "scan_merged_read": False},
        "event_check": {"event_messages": bag_data["raw_event_count"], "ignored_manual_event_messages": bag_data["ignored_manual_event_count"], "boundary_events_used": len([e for e in bag_data["events"] if e.get("schema") == "semantic_nav_episode_event/v1"]), "start_events": len([e for e in bag_data["events"] if e.get("schema") == "semantic_nav_episode_event/v1" and e.get("event") == "start"]), "end_events": len([e for e in bag_data["events"] if e.get("schema") == "semantic_nav_episode_event/v1" and e.get("event") == "end"]), "episode_ids": [item["episode_id"] for item in all_episodes], "status_file_available": status is not None, "status_file": status_data},
        "episode_count": len(all_episodes), "success_count": successes, "failure_count": failures, "incomplete_count": incompletes,
        "reason_counts": dict(sorted(Counter(item["reason"] for item in all_episodes).items())),
        "failure_reason_counts": dict(sorted(Counter(item["reason"] for item in all_episodes if item["status"] == "failed").items())),
        "rendered_episode_ids": [item["episode_id"] for item in episodes], "simulation_duration_sec": sim_duration,
        "episodes": [{key: value for key, value in item.items() if key not in ("planned", "actual")} for item in all_episodes],
        "output": {"png": str(output_dir / "trajectory_overview.png"), "svg": str(output_dir / "trajectory_overview.svg"), "html": str(html_output) if html_output else None, "summary": str(output_dir / "trajectory_summary.json"), "metrics_csv": str(output_dir / "episode_metrics.csv"), "episode_index": str(episode_index), "episodes_dir": str(episodes_dir)},
        "metric_definitions": {"planned_path_efficiency": "straight-line start-goal distance / planned polyline length", "actual_path_efficiency": "straight-line start-goal distance / actual odom polyline length", "actual_vs_planned_efficiency": "planned polyline length / actual odom polyline length"},
    }
    (output_dir / "trajectory_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_metrics(episodes, output_dir / "episode_metrics.csv")
    print(json.dumps({"output_dir": str(output_dir), "episode_count": len(all_episodes), "success_count": successes, "failure_count": failures, "incomplete_count": incompletes, "files": summary["output"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
