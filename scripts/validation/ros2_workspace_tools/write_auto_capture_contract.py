#!/usr/bin/env python3
"""Seal reproducibility inputs and quality settings for an automatic bag."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


SCHEMA = "semantic_nav_auto_capture_contract/v1"
ASSET_SCHEMA = "semantic_nav_supervision_asset_snapshot/v1"
STATUS_SCHEMA = "semantic_nav_auto_capture_status/v1"
RECORDED_TOPICS = [
    "/scan_merged",
    "/scan_01",
    "/scan_02",
    "/odom",
    "/tf",
    "/tf_static",
    "/cmd_vel",
    "/cmd_vel_stamped",
    "/clock",
    "/pedestrian_ground_truth",
    "/semantic_cnn/global_path",
    "/semantic_cnn/local_subgoal",
    "/semantic_cnn/final_goal",
    "/data_collection/goal_accepted",
    "/data_collection/episode_event",
    "/data_collection/auto_capture_status",
    "/drl_vo/raw_model_cmd",
    "/drl_vo/control_event",
    "/drl_vo/episode_reset",
    "/navigation_evaluation/inference_metrics",
]
EXPECTED_TOPIC_TYPES = {
    "/scan_merged": "sensor_msgs/msg/LaserScan",
    "/scan_01": "sensor_msgs/msg/LaserScan",
    "/scan_02": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/cmd_vel_stamped": "geometry_msgs/msg/TwistStamped",
    "/clock": "rosgraph_msgs/msg/Clock",
    "/pedestrian_ground_truth": (
        "semantic_nav_gazebo/msg/PedestrianStateArray"
    ),
    "/semantic_cnn/global_path": "nav_msgs/msg/Path",
    "/semantic_cnn/local_subgoal": "geometry_msgs/msg/PointStamped",
    "/semantic_cnn/final_goal": "geometry_msgs/msg/PointStamped",
    "/data_collection/goal_accepted": "geometry_msgs/msg/PointStamped",
    "/data_collection/episode_event": "std_msgs/msg/String",
    "/data_collection/auto_capture_status": "std_msgs/msg/String",
    "/drl_vo/raw_model_cmd": "geometry_msgs/msg/Twist",
    "/drl_vo/control_event": "std_msgs/msg/String",
    "/drl_vo/episode_reset": "std_msgs/msg/Empty",
    "/navigation_evaluation/inference_metrics": (
        "navigation_evaluation_msgs/msg/InferenceMetrics"
    ),
}
OPTIONAL_EMPTY_TOPICS = frozenset(("/drl_vo/control_event",))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_setting(text: str):
    if "=" not in text:
        raise ValueError(f"setting must be key=value, got {text!r}")
    key, raw = text.split("=", 1)
    if not key:
        raise ValueError("setting key must be non-empty")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw
    return key, value


def parse_settings(items):
    settings = {}
    for item in items:
        key, value = parse_setting(item)
        if key in settings:
            raise ValueError(f"duplicate setting key: {key}")
        settings[key] = value
    return settings


def require_inside(path: Path, directory: Path, label: str) -> None:
    if not path.is_relative_to(directory):
        raise ValueError(f"{label} must be inside the finalized bag")


def validate_asset_manifest(manifest_path: Path, assets: dict) -> list[dict]:
    if not isinstance(assets, dict) or assets.get("schema") != ASSET_SCHEMA:
        raise ValueError("unsupported supervision asset manifest schema")
    snapshot = assets.get("snapshot")
    expected_keys = {
        "map_yaml",
        "occupancy_image",
        "semantic_label",
        "label_names",
    }
    if not isinstance(snapshot, dict) or set(snapshot) != expected_keys:
        raise ValueError("asset manifest snapshot entries are incomplete")
    verified = []
    snapshot_root = manifest_path.parent.resolve()
    for key in sorted(expected_keys):
        entry = snapshot[key]
        if not isinstance(entry, dict):
            raise ValueError(f"invalid asset manifest entry: {key}")
        relative = entry.get("file")
        expected_hash = entry.get("sha256")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).name != relative
        ):
            raise ValueError(f"asset {key} must use a direct relative filename")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise ValueError(f"asset {key} has an invalid SHA-256")
        path = (snapshot_root / relative).resolve()
        if path.parent != snapshot_root or not path.is_file():
            raise ValueError(f"asset snapshot file is missing or unsafe: {key}")
        actual_hash = sha256(path)
        if actual_hash != expected_hash:
            raise ValueError(f"asset snapshot checksum mismatch: {key}")
        verified.append(
            {
                "kind": key,
                "file": relative,
                "size_bytes": path.stat().st_size,
                "sha256": actual_hash,
            }
        )
    return verified


def bag_data_files(bag: Path, metadata_path: Path) -> list[Path]:
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    try:
        relative_paths = metadata[
            "rosbag2_bagfile_information"
        ]["relative_file_paths"]
    except (KeyError, TypeError) as exc:
        raise ValueError("metadata.yaml has no rosbag relative_file_paths") from exc
    if not isinstance(relative_paths, list) or not relative_paths:
        raise ValueError("rosbag metadata contains no data files")
    result = []
    for raw in relative_paths:
        if not isinstance(raw, str) or not raw:
            raise ValueError("rosbag metadata has an invalid data filename")
        relative = Path(raw)
        if relative.is_absolute():
            raise ValueError("rosbag data filename must be relative")
        path = (bag / relative).resolve()
        require_inside(path, bag, "rosbag data file")
        if not path.is_file():
            raise FileNotFoundError(path)
        result.append(path)
    if len(set(result)) != len(result):
        raise ValueError("rosbag metadata repeats a data file")
    return result


def validate_recorded_topics(metadata_path: Path) -> list[dict]:
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    try:
        entries = metadata[
            "rosbag2_bagfile_information"
        ]["topics_with_message_count"]
    except (KeyError, TypeError) as exc:
        raise ValueError("metadata.yaml has no topic inventory") from exc
    if not isinstance(entries, list):
        raise ValueError("rosbag topic inventory is not a list")
    actual = {}
    for entry in entries:
        try:
            topic_metadata = entry["topic_metadata"]
            name = topic_metadata["name"]
            message_type = topic_metadata["type"]
            count = int(entry["message_count"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("rosbag topic inventory entry is invalid") from exc
        if name in actual:
            raise ValueError(f"rosbag topic inventory repeats {name}")
        actual[name] = {"name": name, "type": message_type, "message_count": count}
    records = []
    for name in RECORDED_TOPICS:
        record = actual.get(name)
        if record is None:
            raise ValueError(f"required recorded topic is missing: {name}")
        expected_type = EXPECTED_TOPIC_TYPES[name]
        if record["type"] != expected_type:
            raise ValueError(
                f"recorded topic type mismatch for {name}: "
                f"{record['type']} != {expected_type}"
            )
        if record["message_count"] < 0 or (
            name not in OPTIONAL_EMPTY_TOPICS
            and record["message_count"] == 0
        ):
            raise ValueError(f"required recorded topic is empty: {name}")
        if (
            name == "/data_collection/auto_capture_status"
            and record["message_count"] != 1
        ):
            raise ValueError(
                "automatic capture bag must contain exactly one final status"
            )
        records.append(record)
    return records


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--status-json", required=True, type=Path)
    parser.add_argument("--asset-manifest", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--step-id", required=True)
    parser.add_argument("--pedestrian-seed", required=True, type=int)
    parser.add_argument("--goal-seed", required=True, type=int)
    parser.add_argument("--pedestrian-count", required=True, type=int)
    parser.add_argument("--setting", action="append", default=[])
    return parser.parse_args()


def main():
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    status_path = args.status_json.expanduser().resolve()
    asset_manifest_path = args.asset_manifest.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    output = args.output.expanduser().resolve()
    metadata_path = bag / "metadata.yaml"
    for path in (
        metadata_path,
        status_path,
        asset_manifest_path,
        checkpoint,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
    status = json.loads(status_path.read_text(encoding="utf-8"))
    assets = json.loads(asset_manifest_path.read_text(encoding="utf-8"))
    require_inside(status_path, bag, "scheduler status")
    require_inside(asset_manifest_path, bag, "asset manifest")
    require_inside(checkpoint, bag, "teacher checkpoint")
    require_inside(output, bag, "capture contract")
    if status.get("schema") != STATUS_SCHEMA:
        raise ValueError("unsupported scheduler status schema")
    if status.get("outcome") != "complete":
        raise ValueError("only a complete scheduler status can be sealed")
    if status.get("quality_quota_met") is not True:
        raise ValueError("scheduler quality quota was not met")
    if status.get("duration_deadline_reached") is not True:
        raise ValueError("scheduler duration deadline was not reached")
    verified_assets = validate_asset_manifest(asset_manifest_path, assets)
    data_files = bag_data_files(bag, metadata_path)
    recorded_topics = validate_recorded_topics(metadata_path)
    settings = parse_settings(args.setting)
    contract = {
        "schema": SCHEMA,
        "step_id": args.step_id,
        "bag": str(bag),
        "bag_metadata_sha256": sha256(metadata_path),
        "bag_data_files": [
            {
                "file": str(path.relative_to(bag)),
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in data_files
        ],
        "pedestrian_seed": args.pedestrian_seed,
        "goal_seed": args.goal_seed,
        "pedestrian_count": args.pedestrian_count,
        "teacher_checkpoint": {
            "path": str(checkpoint),
            "sha256": sha256(checkpoint),
        },
        "supervision_assets": {
            "manifest": str(asset_manifest_path),
            "manifest_sha256": sha256(asset_manifest_path),
            "schema": assets.get("schema"),
            "snapshot": assets.get("snapshot"),
            "verified_files": verified_assets,
        },
        "scheduler_status": {
            "path": str(status_path),
            "sha256": sha256(status_path),
            "outcome": status.get("outcome"),
            "reason": status.get("reason"),
            "success_count": status.get("success_count"),
            "failure_count": status.get("failure_count"),
            "quality_requirements": status.get("quality_requirements"),
        },
        "recorded_topics": recorded_topics,
        "settings": settings,
    }
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    temporary.write_text(
        json.dumps(contract, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)
    print(json.dumps(contract, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
