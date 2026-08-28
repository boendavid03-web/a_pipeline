#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --topic
# 代码中检测到的 ROS 2 话题/路径字符串：/data_collection/episode_event
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG, JSON
# 可能使用的关键环境变量：EPISODE_EVENTS_PASS, EVENT_SCHEMA, EVENT_TYPE, JSON, PASS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 02:10:36.321755356 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:37.992044080 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_episode_event_bag.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_episode_event_bag.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Read-only validation of automatic teleop episode boundaries in a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


EVENT_SCHEMA = "semantic_nav_episode_event/v1"
STATUS_SCHEMA = "semantic_nav_auto_capture_status/v1"
EVENT_TYPE = "std_msgs/msg/String"
SUCCESS_REASONS = frozenset(
    ("goal_reached_and_stopped", "goal_tolerance_reached")
)
ALLOWED_EVENTS = frozenset(("armed", "start", "end"))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument(
        "--topic", default="/data_collection/episode_event"
    )
    parser.add_argument("--status-json", type=Path)
    parser.add_argument(
        "--require-complete-status", action="store_true"
    )
    parser.add_argument(
        "--minimum-successful-episodes", type=int, default=0
    )
    parser.add_argument(
        "--minimum-successful-duration-sec", type=float, default=0.0
    )
    return parser.parse_args()


def complete_episode_intervals(events):
    armed_by_id = {}
    for event in events:
        if event.get("schema") != EVENT_SCHEMA:
            raise RuntimeError(
                f"unsupported event schema: {event.get('schema')!r}"
            )
        if event.get("event") not in ALLOWED_EVENTS:
            raise RuntimeError(
                f"unsupported episode event: {event.get('event')!r}"
            )
        try:
            episode_id = int(event["episode_id"])
            stamp_ns = int(event["stamp_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "event has invalid episode_id or stamp_ns"
            ) from exc
        if episode_id <= 0 or stamp_ns < 0:
            raise RuntimeError(
                "episode_id must be positive and stamp_ns non-negative"
            )
        if event.get("event") == "armed":
            if episode_id in armed_by_id:
                raise RuntimeError(f"duplicate armed event for episode {episode_id}")
            goal = event.get("goal")
            if (
                not isinstance(goal, list)
                or len(goal) != 2
                or not all(isinstance(value, (int, float)) for value in goal)
                or not all(math.isfinite(float(value)) for value in goal)
            ):
                raise RuntimeError(
                    f"armed event for episode {episode_id} has invalid goal"
                )
            armed_by_id[episode_id] = {
                "stamp_ns": stamp_ns,
                "goal": [float(goal[0]), float(goal[1])],
            }
    relevant = [
        event for event in events if event.get("event") in ("start", "end")
    ]
    relevant.sort(key=lambda event: int(event.get("stamp_ns", -1)))
    intervals = []
    active = None
    previous_id = 0
    previous_end_stamp_ns = -1
    for event in relevant:
        try:
            episode_id = int(event["episode_id"])
            stamp_ns = int(event["stamp_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("event has invalid episode_id or stamp_ns") from exc
        if event["event"] == "start":
            goal = event.get("goal")
            if (
                active is not None
                or episode_id <= previous_id
                or episode_id not in armed_by_id
                or armed_by_id[episode_id]["stamp_ns"] > stamp_ns
                or armed_by_id[episode_id]["stamp_ns"] < previous_end_stamp_ns
                or not isinstance(goal, list)
                or len(goal) != 2
                or not all(isinstance(value, (int, float)) for value in goal)
                or not all(math.isfinite(float(value)) for value in goal)
            ):
                raise RuntimeError(f"invalid start event for episode {episode_id}")
            if math.hypot(
                float(goal[0]) - armed_by_id[episode_id]["goal"][0],
                float(goal[1]) - armed_by_id[episode_id]["goal"][1],
            ) > 1e-6:
                raise RuntimeError(
                    f"episode {episode_id} armed/start goals disagree"
                )
            active = {
                "episode_id": episode_id,
                "start_stamp_ns": stamp_ns,
                "goal": [float(goal[0]), float(goal[1])],
            }
        else:
            if active is None or episode_id != active["episode_id"]:
                raise RuntimeError(f"unmatched end event for episode {episode_id}")
            if stamp_ns <= active["start_stamp_ns"]:
                raise RuntimeError(f"episode {episode_id} has a non-positive duration")
            reason = event.get("reason")
            if not isinstance(reason, str) or not reason:
                raise RuntimeError(f"episode {episode_id} end reason is missing")
            active["end_stamp_ns"] = stamp_ns
            active["finish_reason"] = reason
            intervals.append(active)
            previous_id = episode_id
            previous_end_stamp_ns = stamp_ns
            active = None
    if active is not None:
        raise RuntimeError(f"episode {active['episode_id']} has no end event")
    if not intervals:
        raise RuntimeError("no complete start/end episode pair was recorded")
    return intervals


def main():
    args = parse_args()
    if args.minimum_successful_episodes < 0:
        raise ValueError("minimum-successful-episodes must be non-negative")
    if (
        not math.isfinite(args.minimum_successful_duration_sec)
        or args.minimum_successful_duration_sec < 0.0
    ):
        raise ValueError(
            "minimum-successful-duration-sec must be finite and non-negative"
        )
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    actual_type = types.get(args.topic)
    if actual_type != EVENT_TYPE:
        raise RuntimeError(
            f"{args.topic} has type {actual_type!r}, expected {EVENT_TYPE!r}"
        )
    message_type = get_message(actual_type)
    events = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, message_type)
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"{args.topic} contains invalid JSON") from exc
        if not isinstance(payload, dict):
            raise RuntimeError(f"{args.topic} payload is not a JSON object")
        events.append(payload)
    intervals = complete_episode_intervals(events)
    print(f"episode_event_messages: {len(events)}")
    print(f"complete_episodes: {len(intervals)}")
    successful = []
    discarded = []
    for interval in intervals:
        duration = (
            interval["end_stamp_ns"] - interval["start_stamp_ns"]
        ) / 1_000_000_000.0
        is_success = interval["finish_reason"] in SUCCESS_REASONS
        (successful if is_success else discarded).append((interval, duration))
        print(
            f"{'SUCCESS' if is_success else 'DISCARDED'} "
            f"episode={interval['episode_id']} "
            f"duration_sec={duration:.3f} goal={interval['goal']} "
            f"reason={interval['finish_reason']}"
        )
    successful_duration = sum(duration for _, duration in successful)
    discarded_duration = sum(duration for _, duration in discarded)
    if len(successful) < args.minimum_successful_episodes:
        raise RuntimeError(
            f"successful episode count {len(successful)} is below required "
            f"{args.minimum_successful_episodes}"
        )
    if successful_duration + 1e-9 < args.minimum_successful_duration_sec:
        raise RuntimeError(
            f"successful duration {successful_duration:.3f}s is below "
            f"required {args.minimum_successful_duration_sec:.3f}s"
        )
    if args.status_json is not None:
        status = json.loads(args.status_json.read_text(encoding="utf-8"))
        if not isinstance(status, dict):
            raise RuntimeError("status JSON must contain an object")
        if status.get("schema") != STATUS_SCHEMA:
            raise RuntimeError(
                f"unsupported capture status schema: {status.get('schema')!r}"
            )
        if args.require_complete_status and status.get("outcome") != "complete":
            raise RuntimeError(
                f"capture status is not complete: {status.get('outcome')!r}"
            )
        if (
            args.require_complete_status
            and status.get("duration_deadline_reached") is not True
        ):
            raise RuntimeError(
                "complete capture status did not reach the duration deadline"
            )
        if int(status.get("success_count", -1)) != len(successful):
            raise RuntimeError("status success_count disagrees with events")
        if int(status.get("failure_count", -1)) != len(discarded):
            raise RuntimeError("status failure_count disagrees with events")
        expected_failure_reasons = Counter(
            interval["finish_reason"] for interval, _ in discarded
        )
        if status.get("failure_reasons", {}) != dict(
            sorted(expected_failure_reasons.items())
        ):
            raise RuntimeError("status failure_reasons disagrees with events")
        status_success_duration = status.get(
            "successful_episode_duration_sec"
        )
        if status_success_duration is None:
            raise RuntimeError(
                "status successful episode duration is missing"
            )
        if not math.isclose(
            float(status_success_duration),
            successful_duration,
            rel_tol=0.0,
            abs_tol=0.15,
        ):
            raise RuntimeError(
                "status successful episode duration disagrees with events"
            )
        status_discarded_duration = status.get(
            "discarded_episode_duration_sec"
        )
        if status_discarded_duration is None:
            raise RuntimeError(
                "status discarded episode duration is missing"
            )
        if not math.isclose(
            float(status_discarded_duration),
            discarded_duration,
            rel_tol=0.0,
            abs_tol=0.15,
        ):
            raise RuntimeError(
                "status discarded episode duration disagrees with events"
            )
        if (
            args.require_complete_status
            and status.get("quality_quota_met") is not True
        ):
            raise RuntimeError("capture status reports an unmet quality quota")
    print(
        "EPISODE_EVENTS_PASS "
        f"successful_episodes={len(successful)} "
        f"discarded_episodes={len(discarded)} "
        f"successful_duration_sec={successful_duration:.3f} "
        f"discarded_duration_sec={discarded_duration:.3f}"
    )


if __name__ == "__main__":
    main()
