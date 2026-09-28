#!/usr/bin/env python3
"""Stage 2-A: validate the Isaac5 custom-experience dual LiDAR contract."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from basic_navigation import (
    PHYSICS_DT,
    ROBOT_USD,
    add_lidar_obstacles,
    make_runtime,
    parse_pose,
)
from lidar_sensor import DualPhysxRaycastLidar


HERE = Path(__file__).resolve().parent
BACKEND_ROOT = HERE.parents[0]
GENERATED_ROOT = BACKEND_ROOT / "generated"
SAMPLE_COUNT = 2000
RATE_HZ = 15.0
PUBLISH_EVERY_STEPS = round(1.0 / (PHYSICS_DT * RATE_HZ))
DURATION_S = 3.0


def main() -> int:
    app = world = robot = controller = ros = None
    result: dict[str, object] = {"status": "FAIL", "stage": "2-A"}
    received: dict[str, list[dict[str, object]]] = {"/scan_01": [], "/scan_02": []}
    try:
        app, world, robot, controller, ros = make_runtime(
            "lidar", ROBOT_USD, use_ros=True, dual_scan=True
        )
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data

        def record(topic: str):
            def callback(message: LaserScan) -> None:
                stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                    message.header.stamp.nanosec
                )
                received[topic].append(
                    {
                        "stamp_ns": stamp_ns,
                        "frame_id": message.header.frame_id,
                        "beam_count": len(message.ranges),
                        "angle_min": float(message.angle_min),
                        "angle_increment": float(message.angle_increment),
                        "range_min": float(message.range_min),
                        "range_max": float(message.range_max),
                        "receive_wall": time.monotonic(),
                    }
                )

            return callback

        ros.node.create_subscription(
            LaserScan, "/scan_01", record("/scan_01"), qos_profile_sensor_data
        )
        ros.node.create_subscription(
            LaserScan, "/scan_02", record("/scan_02"), qos_profile_sensor_data
        )
        sensor = DualPhysxRaycastLidar(
            sample_count=SAMPLE_COUNT,
            rate_hz=RATE_HZ,
            range_min=0.5,
            range_max=50.0,
        )
        samples: list[dict[str, object]] = []
        sim_time = 0.0
        steps = int(round(DURATION_S / PHYSICS_DT))
        wall_start = time.monotonic()
        for step in range(steps):
            ros.spin_once()
            controller.apply(0.0, 0.0, 0.0)
            world.step(render=False, step_sim=True)
            sim_time = float(world.current_time)
            position, orientation, _ = parse_pose(robot)
            ros.publish_clock(sim_time)
            ros.publish_state(sim_time, position, orientation, (0.0, 0.0, 0.0))
            if step % PUBLISH_EVERY_STEPS == 0:
                pair = sensor.sample(sim_time, position, orientation)
                ros.publish_dual_scan(pair)
                samples.append(
                    {
                        "sim_time": sim_time,
                        "stamp_equal": pair.scan_01.sim_time == pair.scan_02.sim_time,
                        "scan_01": {
                            "frame_id": pair.scan_01.frame_id,
                            "beam_count": len(pair.scan_01.ranges),
                            "finite_beams": sum(math.isfinite(value) for value in pair.scan_01.ranges),
                            "self_hit_beams": sum(
                                path.startswith("/World/Robot") for path in pair.scan_01.hit_paths
                            ),
                        },
                        "scan_02": {
                            "frame_id": pair.scan_02.frame_id,
                            "beam_count": len(pair.scan_02.ranges),
                            "finite_beams": sum(math.isfinite(value) for value in pair.scan_02.ranges),
                            "self_hit_beams": sum(
                                path.startswith("/World/Robot") for path in pair.scan_02.hit_paths
                            ),
                        },
                    }
                )
            time.sleep(max(0.0, PHYSICS_DT - (time.monotonic() - wall_start - sim_time)))

        ros.spin_once()
        wall_duration = time.monotonic() - wall_start
        pair_count = min(len(received["/scan_01"]), len(received["/scan_02"]))
        paired = list(zip(received["/scan_01"], received["/scan_02"]))
        timestamp_deltas = [
            abs(int(first["stamp_ns"]) - int(second["stamp_ns"]))
            for first, second in paired
        ]
        stamp_steps = {
            topic: [
                (int(rows[index]["stamp_ns"]) - int(rows[index - 1]["stamp_ns"])) / 1.0e9
                for index in range(1, len(rows))
            ]
            for topic, rows in received.items()
        }
        wall_rates = {
            topic: (
                (len(rows) - 1) / (float(rows[-1]["receive_wall"]) - float(rows[0]["receive_wall"]))
                if len(rows) >= 2
                and float(rows[-1]["receive_wall"]) > float(rows[0]["receive_wall"])
                else None
            )
            for topic, rows in received.items()
        }
        checks = {
            "publish_samples": len(samples) >= 3,
            "received_scan_01": len(received["/scan_01"]) >= 3,
            "received_scan_02": len(received["/scan_02"]) >= 3,
            "beam_count_2000": all(
                int(row["beam_count"]) == SAMPLE_COUNT
                for topic in received.values()
                for row in topic
            ),
            "frame_ids_correct": all(
                row["frame_id"] == topic.rsplit("/", 1)[-1].replace("scan", "base_scan")
                for topic, rows in received.items()
                for row in rows
            ),
            "timestamps_synchronized": bool(timestamp_deltas)
            and max(timestamp_deltas) == 0,
            "timestamps_strictly_increase": all(
                deltas and all(delta > 0.0 for delta in deltas)
                for deltas in stamp_steps.values()
            ),
            "layout_stable": len(
                {
                    (
                        row["frame_id"],
                        row["beam_count"],
                        row["angle_min"],
                        row["angle_increment"],
                        row["range_min"],
                        row["range_max"],
                    )
                    for rows in received.values()
                    for row in rows
                }
            )
            == 2,
            "frequency_near_15hz": all(
                deltas
                and abs(sum(deltas) / len(deltas) - 1.0 / RATE_HZ) <= PHYSICS_DT * 0.25
                for deltas in stamp_steps.values()
            )
            and all(rate is not None and rate >= 12.0 for rate in wall_rates.values()),
            "static_geometry_hits": all(
                int(sample[topic]["finite_beams"]) > 0
                for sample in samples
                for topic in ("scan_01", "scan_02")
            ),
            "robot_self_hits_filtered": all(
                int(sample[topic]["self_hit_beams"]) == 0
                for sample in samples
                for topic in ("scan_01", "scan_02")
            ),
        }
        result = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "stage": "2-A",
            "backend": "physx_scene_query",
            "experience": str(GENERATED_ROOT / "minimal_core_physx_no_rtx.kit"),
            "sample_count": SAMPLE_COUNT,
            "requested_rate_hz": RATE_HZ,
            "sim_time_s": sim_time,
            "wall_time_s": wall_duration,
            "published_samples": len(samples),
            "received_samples": {
                topic: len(rows) for topic, rows in received.items()
            },
            "max_pair_timestamp_delta_ns": max(timestamp_deltas)
            if timestamp_deltas
            else None,
            "observed_wall_rates_hz": wall_rates,
            "observed_stamp_periods_s": {
                topic: (sum(values) / len(values) if values else None)
                for topic, values in stamp_steps.items()
            },
            "checks": checks,
            "sample_layout": samples[:2],
            "teardown": None,
        }
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "stage": "2-A", "error": repr(exc)}
        return 1
    finally:
        teardown = {"ros_close": False, "world_stop": False, "app_close": False, "errors": []}
        if ros is not None:
            try:
                ros.close()
                teardown["ros_close"] = True
            except Exception as exc:
                teardown["errors"].append(f"ros_close: {exc!r}")
        if world is not None:
            try:
                world.stop()
                teardown["world_stop"] = True
            except Exception as exc:
                teardown["errors"].append(f"world_stop: {exc!r}")
        if app is not None:
            try:
                app.close()
                teardown["app_close"] = True
            except Exception as exc:
                teardown["errors"].append(f"app_close: {exc!r}")
        teardown["status"] = (
            "PASS" if not teardown["errors"] and teardown["app_close"] else "FAIL"
        )
        result["teardown"] = teardown
        print("ISAAC5_STAGE2A_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
