#!/usr/bin/env python3
"""Read-only preflight for the deferred standalone Level 3 runtime."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from runtime_common import default_report_path, load_alignment, write_report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wall-timeout", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def transform_key(transform: Any) -> tuple[Any, ...]:
    item = transform.transform
    return (
        transform.header.frame_id,
        transform.child_frame_id,
        round(float(item.translation.x), 6),
        round(float(item.translation.y), 6),
        round(float(item.translation.z), 6),
        round(float(item.rotation.x), 6),
        round(float(item.rotation.y), 6),
        round(float(item.rotation.z), 6),
        round(float(item.rotation.w), 6),
    )


class RuntimePreflight(Node):
    def __init__(self) -> None:
        super().__init__(
            "level3_runtime_preflight",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_message: OccupancyGrid | None = None
        self.odom_message: Odometry | None = None
        self.scan_message: LaserScan | None = None
        self.global_costmap: OccupancyGrid | None = None
        self.local_costmap: OccupancyGrid | None = None
        self.clock_values: list[float] = []
        self.static_transforms: set[tuple[Any, ...]] = set()
        self.dynamic_pairs: set[tuple[str, str]] = set()
        self.create_subscription(OccupancyGrid, "/map", self._map, latched)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(LaserScan, "/scan_merged", self._scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._global_costmap, latched
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._local_costmap, latched
        )
        self.create_subscription(TFMessage, "/tf_static", self._tf_static, latched)
        self.create_subscription(TFMessage, "/tf", self._tf, 50)
        # Node.__init__ owns an instance attribute named ``_clock``.  Using that
        # name for a callback resolves to ROSClock rather than this class method.
        self.create_subscription(Clock, "/clock", self._on_clock, 20)

    def _map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _odom(self, message: Odometry) -> None:
        self.odom_message = message

    def _scan(self, message: LaserScan) -> None:
        self.scan_message = message

    def _global_costmap(self, message: OccupancyGrid) -> None:
        self.global_costmap = message

    def _local_costmap(self, message: OccupancyGrid) -> None:
        self.local_costmap = message

    def _on_clock(self, message: Clock) -> None:
        self.clock_values.append(
            float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        )

    def _tf_static(self, message: TFMessage) -> None:
        self.static_transforms.update(transform_key(item) for item in message.transforms)

    def _tf(self, message: TFMessage) -> None:
        self.dynamic_pairs.update(
            (item.header.frame_id, item.child_frame_id) for item in message.transforms
        )


def lifecycle_state(node: RuntimePreflight, name: str) -> str:
    client = node.create_client(GetState, f"/{name}/get_state")
    if not client.wait_for_service(timeout_sec=2.0):
        return "SERVICE_UNAVAILABLE"
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.result() is None:
        return "NO_RESPONSE"
    return str(future.result().current_state.label)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RuntimePreflight()
    try:
        alignment = load_alignment()["map_to_odom"]
        expected_yaw = float(alignment["yaw_rad"])
        expected_map_odom = (
            "map",
            "odom",
            round(float(alignment["x_m"]), 6),
            round(float(alignment["y_m"]), 6),
            0.0,
            0.0,
            0.0,
            round(math.sin(0.5 * expected_yaw), 6),
            round(math.cos(0.5 * expected_yaw), 6),
        )
        required_sensor_transforms = {
            ("base_link", "base_scan_01"),
            ("base_link", "base_scan_02"),
        }
        expected_tf_static_publishers = [
            "/isaac_6_udp_ros_bridge",
            "/level3_ground_truth_map_to_odom",
        ]
        deadline = time.monotonic() + args.wall_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            observed_static_pairs = {item[:2] for item in node.static_transforms}
            observed_tf_static_publishers = [
                f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                for info in node.get_publishers_info_by_topic("/tf_static")
            ]
            if (
                node.map_message is not None
                and node.odom_message is not None
                and node.scan_message is not None
                and node.global_costmap is not None
                and node.local_costmap is not None
                and len(node.clock_values) >= 2
                and ("odom", "base_link") in node.dynamic_pairs
                and expected_map_odom in node.static_transforms
                and required_sensor_transforms <= observed_static_pairs
                and sorted(observed_tf_static_publishers)
                == sorted(expected_tf_static_publishers)
                and len(observed_tf_static_publishers) == 2
            ):
                break

        map_odom_transforms = {
            item for item in node.static_transforms if item[:2] == ("map", "odom")
        }
        static_pairs = {item[:2] for item in node.static_transforms}
        tf_static_publisher_list = [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/tf_static")
        ]
        tf_publisher_list = [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/tf")
        ]
        scan = node.scan_message
        occupancy_map = node.map_message
        odom = node.odom_message
        odom_yaw = (
            2.0
            * math.atan2(
                float(odom.pose.pose.orientation.z),
                float(odom.pose.pose.orientation.w),
            )
            if odom is not None
            else math.nan
        )
        lifecycle_nodes = (
            "map_server",
            "planner_server",
            "controller_server",
            "behavior_server",
            "bt_navigator",
            "velocity_smoother",
        )
        lifecycle = {name: lifecycle_state(node, name) for name in lifecycle_nodes}
        navigate_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        follow_client = ActionClient(node, FollowPath, "/follow_path")
        cmd_vel_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/cmd_vel")
        }
        cmd_vel_nav_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/cmd_vel_nav")
        }
        scan_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/scan_merged")
        }
        odom_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/odom")
        }
        clock_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_publishers_info_by_topic("/clock")
        }
        cmd_vel_subscribers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_subscriptions_info_by_topic("/cmd_vel")
        }
        goal_pose_subscribers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in node.get_subscriptions_info_by_topic("/goal_pose")
        }
        runtime_graph_empty = (
            all(value == "SERVICE_UNAVAILABLE" for value in lifecycle.values())
            and not tf_static_publisher_list
            and not tf_publisher_list
            and not cmd_vel_publishers
            and not cmd_vel_nav_publishers
            and not scan_publishers
            and not odom_publishers
            and not clock_publishers
            and not goal_pose_subscribers
            and not node.get_publishers_info_by_topic("/map")
        )
        checks = {
            "runtime_graph_present": not runtime_graph_empty,
            "clock_advancing": (
                len(node.clock_values) >= 2
                and max(node.clock_values) - min(node.clock_values) > 0.01
            ),
            "map_contract": (
                occupancy_map is not None
                and occupancy_map.header.frame_id == "map"
                and abs(float(occupancy_map.info.resolution) - 0.05) < 1.0e-6
                and occupancy_map.info.width == 1024
                and occupancy_map.info.height == 768
            ),
            "odom_contract": (
                odom is not None
                and odom.header.frame_id == "odom"
                and odom.child_frame_id == "base_link"
            ),
            "default_spawn_pose": (
                odom is not None
                and abs(float(odom.pose.pose.position.x) - 2.0) <= 0.05
                and abs(float(odom.pose.pose.position.y) - 2.0) <= 0.05
                and abs(math.atan2(math.sin(odom_yaw), math.cos(odom_yaw))) <= 0.05
            ),
            "merged_scan_contract": (
                scan is not None
                and scan.header.frame_id == "base_link"
                and len(scan.ranges) >= 90
                and scan.angle_min <= -3.13
                and scan.angle_max >= 3.13
                and scan.range_min <= 0.5
                and scan.range_max >= 49.0
                and any(math.isfinite(float(value)) for value in scan.ranges)
            ),
            "map_to_odom_exact_and_unique": map_odom_transforms == {expected_map_odom},
            "map_to_odom_not_dynamic": ("map", "odom") not in node.dynamic_pairs,
            "tf_static_publishers_exact": (
                sorted(tf_static_publisher_list)
                == sorted(expected_tf_static_publishers)
                and len(tf_static_publisher_list) == 2
            ),
            "odom_to_base_dynamic": ("odom", "base_link") in node.dynamic_pairs,
            "tf_dynamic_only_from_bridge": tf_publisher_list
            == ["/isaac_6_udp_ros_bridge"],
            "sensor_static_tf": required_sensor_transforms <= static_pairs,
            "global_costmap_available": (
                node.global_costmap is not None
                and node.global_costmap.header.frame_id == "map"
            ),
            "local_costmap_available": (
                node.local_costmap is not None
                and node.local_costmap.header.frame_id == "odom"
            ),
            "all_lifecycle_nodes_active": all(value == "active" for value in lifecycle.values()),
            "navigate_to_pose_available": navigate_client.wait_for_server(timeout_sec=1.0),
            "follow_path_available": follow_client.wait_for_server(timeout_sec=1.0),
            "goal_pose_interface_available": goal_pose_subscribers == {"/bt_navigator"},
            "cmd_vel_only_from_smoother": cmd_vel_publishers == {"/velocity_smoother"},
            "cmd_vel_nav_only_from_nav2": cmd_vel_nav_publishers
            == {"/controller_server", "/behavior_server"},
            "single_map_publisher": len(node.get_publishers_info_by_topic("/map")) == 1,
            "merged_scan_only_from_merger": scan_publishers
            == {"/v7_dual_laser_scan_merger"},
            "odom_only_from_bridge": odom_publishers == {"/isaac_6_udp_ros_bridge"},
            "clock_only_from_bridge": clock_publishers == {"/isaac_6_udp_ros_bridge"},
            "cmd_vel_consumed_by_bridge": "/isaac_6_udp_ros_bridge"
            in cmd_vel_subscribers,
        }
        passed = all(checks.values())
        output = (args.output or default_report_path("preflight")).resolve()
        write_report(
            output,
            {
                "schema": "a_pipeline_level3_runtime_preflight/v1",
                "status": "PASS" if passed else "FAIL",
                "runtime_graph_empty": runtime_graph_empty,
                "checks": checks,
                "lifecycle_states": lifecycle,
                "map_to_odom_transforms_seen": [list(item) for item in map_odom_transforms],
                "tf_static_publishers": sorted(tf_static_publisher_list),
                "tf_publishers": tf_publisher_list,
                "static_tf_pairs": sorted([list(item) for item in static_pairs]),
                "dynamic_tf_pairs": sorted([list(item) for item in node.dynamic_pairs]),
                "cmd_vel_publishers": sorted(cmd_vel_publishers),
                "cmd_vel_nav_publishers": sorted(cmd_vel_nav_publishers),
                "scan_merged_publishers": sorted(scan_publishers),
                "odom_publishers": sorted(odom_publishers),
                "clock_publishers": sorted(clock_publishers),
                "cmd_vel_subscribers": sorted(cmd_vel_subscribers),
                "goal_pose_subscribers": sorted(goal_pose_subscribers),
                "runtime_tests_not_run": [
                    "NavigateToPose motion",
                    "static-obstacle route",
                    "Omni FollowPath motion",
                    "Isaac collision result",
                ],
            },
        )
        if runtime_graph_empty:
            print("RUNTIME_GRAPH_EMPTY=FAIL")
            print(
                "ERROR: no Isaac bridge or Level 3 Nav2 endpoints were discovered; "
                "keep Terminal 1 and Terminal 2 running before retrying this preflight"
            )
        for name, value in checks.items():
            print(f"TEST_{name.upper()}={'PASS' if value else 'FAIL'}")
        print(f"LEVEL3_RUNTIME_PREFLIGHT={'PASS' if passed else 'FAIL'}")
        print(f"RESULT_FILE={output}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
