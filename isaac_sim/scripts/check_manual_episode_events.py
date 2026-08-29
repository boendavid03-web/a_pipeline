#!/usr/bin/env python3
"""Validate complete start/end intervals for Isaac keyboard teleoperation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


SCHEMA = "isaac_manual_teleop_episode/v1"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    events = []
    while reader.has_next():
        topic, data, storage_stamp_ns = reader.read_next()
        if topic != "/data_collection/episode_event":
            continue
        message = deserialize_message(data, String)
        payload = json.loads(message.data)
        if payload.get("schema") != SCHEMA:
            raise RuntimeError(
                f"unsupported manual episode schema: {payload.get('schema')!r}"
            )
        payload["_storage_stamp_ns"] = int(storage_stamp_ns)
        events.append(payload)

    active = None
    completed = []
    previous_id = 0
    for event in events:
        kind = event.get("event")
        episode_id = int(event.get("episode_id", -1))
        stamp_ns = int(event.get("stamp_ns", -1))
        sim_time = float(event.get("sim_time", -1.0))
        if kind not in ("start", "end"):
            raise RuntimeError(f"unsupported manual episode event: {kind!r}")
        if episode_id <= 0 or stamp_ns < 0 or sim_time < 0.0:
            raise RuntimeError("episode id/timestamp/simulation time is invalid")
        if kind == "start":
            if active is not None:
                raise RuntimeError("an episode started before the previous one ended")
            if episode_id <= previous_id:
                raise RuntimeError("episode ids are not strictly increasing")
            active = event
        else:
            if active is None:
                raise RuntimeError("an episode ended without a recorded start")
            if episode_id != int(active["episode_id"]):
                raise RuntimeError("episode end id differs from its start id")
            if stamp_ns <= int(active["stamp_ns"]):
                raise RuntimeError("episode end wall timestamp is not after start")
            if sim_time <= float(active["sim_time"]):
                raise RuntimeError("episode end simulation time is not after start")
            completed.append((active, event))
            previous_id = episode_id
            active = None
    if active is not None:
        raise RuntimeError(f"episode {active['episode_id']} has no recorded end")
    if not completed:
        raise RuntimeError("no complete manual teleoperation episode was recorded")

    print(f"manual_episode_event_count: {len(events)}")
    print(f"manual_complete_episode_count: {len(completed)}")
    for start, end in completed:
        duration = float(end["sim_time"]) - float(start["sim_time"])
        print(
            f"PASS episode_id={start['episode_id']} "
            f"sim_duration_sec={duration:.6f}"
        )
    print("manual_episode_intervals: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
