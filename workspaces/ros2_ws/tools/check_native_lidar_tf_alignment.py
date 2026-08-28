#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：--bag, --base-frame, --cmd-vel-topic, --map-frame, --map-yaml, --odom-topic, --report-md, --scan-topic
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_01, /scan_02, /scan_merged, /tf, /tf_static
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：BAG
# 可能使用的关键环境变量：FINAL_STATUS, JSON_RESULT, WARNING
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_native_lidar_tf_alignment.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.685303552 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:31.258092381 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_native_lidar_tf_alignment.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/check_native_lidar_tf_alignment.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/check_native_lidar_tf_alignment.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜check_native_lidar_tf_alignment.py】
# 用途：ROS 2 工作空间辅助工具，用于数据检查、转换、调试或运行时辅助。
# 输入输出：输入通常是命令行参数、bag、数据集或 ROS 2 状态；输出为检查结果、转换文件、日志或辅助话题。
# 关系：被 pipeline 或人工命令调用，依赖 ROS 2 环境和对应数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Check TF and native LiDAR geometry assumptions before Semantic2D conversion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from convert_rosbag2_to_semantic2d_native_lidar import (
    normalize_frame,
    read_bag,
)


def load_map_yaml(path: Path):
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return {
        "resolution": float(data["resolution"]),
        "origin": [float(v) for v in data["origin"]],
    }


def can_lookup_any(tf_index, parent: str, child: str, stamps):
    for stamp_ns in stamps:
        if tf_index.lookup(parent, child, stamp_ns) is not None:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--report-md", type=Path)
    args = parser.parse_args()

    scans, odoms, _cmds, topic_types, topic_counts, tf_index, scan_summaries = read_bag(
        args.bag,
        args.scan_topic,
        args.odom_topic,
        args.cmd_vel_topic,
        inspect_scan_topics=["/scan_01", "/scan_02", args.scan_topic],
        include_tf=True,
        return_extras=True,
    )
    if not scans:
        raise RuntimeError(f"No scans found on {args.scan_topic}")

    sample_stamps = [scans[0][0], scans[len(scans) // 2][0], scans[-1][0]]
    map_frame = normalize_frame(args.map_frame)
    base_frame = normalize_frame(args.base_frame)

    merged_info = scan_summaries.get(args.scan_topic, {})
    merged_frames = merged_info.get("frame_ids", {})
    scan_frame = next(iter(merged_frames.keys()), "") if len(merged_frames) == 1 else ""

    has_tf = "/tf" in topic_types and topic_counts.get("/tf", 0) > 0
    has_tf_static = "/tf_static" in topic_types and topic_counts.get("/tf_static", 0) > 0
    has_map_base = can_lookup_any(tf_index, map_frame, base_frame, sample_stamps)
    has_map_scan = bool(scan_frame) and can_lookup_any(tf_index, map_frame, scan_frame, sample_stamps)
    has_odom_base = can_lookup_any(tf_index, "odom", base_frame, sample_stamps)

    warnings = []
    status = "safe"
    if not has_tf and not has_tf_static:
        warnings.append("Bag has no /tf or /tf_static; projection would rely only on odom.")
    if not scan_frame:
        warnings.append(f"{args.scan_topic} has missing or multiple frame_id values: {merged_frames}")
        status = "unsafe"
    elif scan_frame != base_frame and not has_map_scan:
        warnings.append(
            f"{args.scan_topic} frame '{scan_frame}' is not '{base_frame}', and {map_frame}->{scan_frame} is unavailable."
        )
        status = "unsafe"
    elif has_map_scan:
        status = "safe"
    elif has_map_base and scan_frame == base_frame:
        status = "safe"
    elif has_odom_base or odoms:
        warnings.append("Only odom-frame pose is available; conversion assumes odom and map are aligned.")
        status = "warning"
    else:
        warnings.append("No usable map/base/odom pose source was found.")
        status = "unsafe"

    for topic in ("/scan_01", "/scan_02", args.scan_topic):
        info = scan_summaries.get(topic, {})
        if info.get("count", 0) and len(info.get("frame_ids", {})) > 1:
            warnings.append(f"{topic} has multiple frame_id values: {info.get('frame_ids')}")
            if topic == args.scan_topic:
                status = "unsafe"
        if info.get("count", 0) and len(info.get("beam_count_unique", [])) > 1:
            warnings.append(f"{topic} beam_count changes across the bag: {info.get('beam_count_unique')}")

    map_info = load_map_yaml(args.map_yaml)
    result = {
        "bag": str(args.bag),
        "has_tf": has_tf,
        "has_tf_static": has_tf_static,
        "scan_merged_frame_id": scan_frame or merged_frames,
        "scan_topics": scan_summaries,
        "can_resolve_map_to_base_link": has_map_base,
        "can_resolve_map_to_scan_frame": has_map_scan,
        "can_resolve_odom_to_base_link": has_odom_base,
        "map": map_info,
        "tf_dynamic_edges": [list(edge) for edge in sorted(tf_index.dynamic_edges)],
        "tf_static_edges": [list(edge) for edge in sorted(tf_index.static_edges)],
        "warnings": sorted(set(warnings)),
        "final_status": status,
    }

    lines = [
        "# Native LiDAR TF Alignment Check",
        "",
        f"- bag: `{result['bag']}`",
        f"- has /tf: {has_tf}",
        f"- has /tf_static: {has_tf_static}",
        f"- map frame: `{map_frame}`",
        f"- base frame: `{base_frame}`",
        f"- /scan_merged frame_id: `{result['scan_merged_frame_id']}`",
        f"- can resolve map->base_link: {has_map_base}",
        f"- can resolve map->scan_frame: {has_map_scan}",
        f"- can resolve odom->base_link: {has_odom_base}",
        f"- map resolution: {map_info['resolution']}",
        f"- map origin: {map_info['origin']}",
        "",
        "## Scan Geometry",
    ]
    for topic in ("/scan_01", "/scan_02", args.scan_topic):
        info = scan_summaries.get(topic, {"count": 0})
        lines.extend(
            [
                f"### {topic}",
                f"- count: {info.get('count', 0)}",
                f"- frame_ids: `{info.get('frame_ids', {})}`",
                f"- angle_min: {info.get('angle_min_min')} .. {info.get('angle_min_max')}",
                f"- angle_max: {info.get('angle_max_min')} .. {info.get('angle_max_max')}",
                f"- angle_increment: {info.get('angle_increment_min')} .. {info.get('angle_increment_max')}",
                f"- beam_count_unique: {info.get('beam_count_unique')}",
                f"- range_min: {info.get('range_min_min')} .. {info.get('range_min_max')}",
                f"- range_max: {info.get('range_max_min')} .. {info.get('range_max_max')}",
                "",
            ]
        )
    lines.extend(
        [
            "## TF Edges",
            f"- dynamic: `{result['tf_dynamic_edges']}`",
            f"- static: `{result['tf_static_edges']}`",
            "",
            "## Assessment",
        ]
    )
    if result["warnings"]:
        lines.extend(f"- WARNING: {warning}" for warning in result["warnings"])
    else:
        lines.append("- No obvious coordinate-frame risk detected by this check.")
    lines.extend(["", f"FINAL_STATUS={status}", ""])

    report = "\n".join(lines)
    if args.report_md:
        args.report_md.parent.mkdir(parents=True, exist_ok=True)
        args.report_md.write_text(report, encoding="utf-8")

    print(report)
    print("JSON_RESULT=" + json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
