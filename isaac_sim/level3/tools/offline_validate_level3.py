#!/usr/bin/env python3
"""Static, non-runtime validation of the standalone Isaac Level 3 files."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import math
import os
import inspect
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
LEVEL3_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = LEVEL3_ROOT.parents[1]
CONFIG_FILE = LEVEL3_ROOT / "config/nav2_level3.yaml"
ALIGNMENT_FILE = LEVEL3_ROOT / "config/map_alignment.yaml"
ALIGNMENT_REPORT = LEVEL3_ROOT / "reports/map_alignment.json"
ROUTES_FILE = LEVEL3_ROOT / "config/test_routes.yaml"
ROUTES_REPORT = LEVEL3_ROOT / "reports/test_routes.json"
LAUNCH_FILE = LEVEL3_ROOT / "launch/standalone_level3.launch.py"
OFFLINE_REPORT = LEVEL3_ROOT / "reports/offline_validation.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def expect(checks: dict[str, bool], name: str, value: Any) -> None:
    checks[name] = bool(value)


def contains_key(value: Any, target: str) -> bool:
    if isinstance(value, dict):
        return target in value or any(contains_key(item, target) for item in value.values())
    if isinstance(value, list):
        return any(contains_key(item, target) for item in value)
    return False


def load_launch_description() -> Any:
    spec = importlib.util.spec_from_file_location("level3_static_launch", LAUNCH_FILE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {LAUNCH_FILE}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def import_runtime_modules() -> None:
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        for name in (
            "runtime_common",
            "goal_pose_adapter",
            "send_test_goal",
            "send_omni_follow_path",
            "validate_level3_runtime",
            "check_isaac_collision_result",
        ):
            __import__(name)
    finally:
        sys.path.pop(0)


def self_attributes_assigned(tree: ast.AST) -> set[str]:
    """Return attributes written as ``self.<name>`` in an AST."""
    attributes: set[str] = set()
    for node in ast.walk(tree):
        targets: list[ast.expr] = []
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        elif isinstance(node, ast.AugAssign):
            targets = [node.target]
        for target in targets:
            for item in ast.walk(target):
                if (
                    isinstance(item, ast.Attribute)
                    and isinstance(item.value, ast.Name)
                    and item.value.id == "self"
                ):
                    attributes.add(item.attr)
    return attributes


def subscription_callback_names(tree: ast.AST) -> set[str]:
    """Return ``self.<callback>`` names passed to create_subscription."""
    callbacks: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 3:
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "create_subscription":
            continue
        callback = node.args[2]
        if (
            isinstance(callback, ast.Attribute)
            and isinstance(callback.value, ast.Name)
            and callback.value.id == "self"
        ):
            callbacks.add(callback.attr)
    return callbacks


def main() -> int:
    checks: dict[str, bool] = {}
    required_files = (
        CONFIG_FILE,
        ALIGNMENT_FILE,
        ALIGNMENT_REPORT,
        ROUTES_FILE,
        ROUTES_REPORT,
        LEVEL3_ROOT / "reports/map_alignment_overlay.png",
        LAUNCH_FILE,
        LEVEL3_ROOT / "start_standalone_level3.sh",
        LEVEL3_ROOT / "send_first_goal.sh",
        LEVEL3_ROOT / "send_obstacle_goal.sh",
        LEVEL3_ROOT / "send_omni_test.sh",
        LEVEL3_ROOT / "check_runtime_preflight.sh",
        SCRIPT_DIR / "send_test_goal.py",
        SCRIPT_DIR / "goal_pose_adapter.py",
        SCRIPT_DIR / "send_omni_follow_path.py",
        SCRIPT_DIR / "validate_level3_runtime.py",
        SCRIPT_DIR / "check_isaac_collision_result.py",
        SCRIPT_DIR / "validate_test_routes.py",
    )
    expect(checks, "required_files_exist", all(path.is_file() for path in required_files))
    expect(
        checks,
        "custom_scene_deterministic_check",
        os.environ.get("LEVEL3_CUSTOM_SCENE_CHECK") == "PASS",
    )

    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    alignment = yaml.safe_load(ALIGNMENT_FILE.read_text(encoding="utf-8"))
    report = json.loads(ALIGNMENT_REPORT.read_text(encoding="utf-8"))
    expect(checks, "alignment_report_pass", report.get("status") == "PASS")
    route_config = yaml.safe_load(ROUTES_FILE.read_text(encoding="utf-8"))
    route_report = json.loads(ROUTES_REPORT.read_text(encoding="utf-8"))
    expect(checks, "test_route_geometry_pass", route_report.get("status") == "PASS")
    expect(
        checks,
        "alignment_sample_and_region_contract",
        report["fit"]["sample_count"] == 79 * 4 * report["fit"]["samples_per_edge"]
        and set(report["residuals"]["regions_by_world_x_tertile"])
        == {"west", "center", "east"}
        and max(
            region["p90_m"]
            for region in report["residuals"]["regions_by_world_x_tertile"].values()
        )
        <= 0.30,
    )
    expect(
        checks,
        "alignment_repeatability",
        all(
            spread <= limit
            for spread, limit in zip(
                report["fit"]["repeatability_peak_to_peak"],
                (0.002, 0.002, 0.0002),
            )
        ),
    )
    frozen = alignment["map_to_odom"]
    fitted = report["transform"]
    expect(
        checks,
        "frozen_transform_matches_report",
        all(
            abs(float(a) - float(b)) <= 5.0e-9
            for a, b in (
                (frozen["x_m"], fitted["x_m"]),
                (frozen["y_m"], fitted["y_m"]),
                (frozen["yaw_rad"], fitted["yaw_rad"]),
            )
        ),
    )
    input_paths = {
        "map_yaml": Path(report["inputs"]["map_yaml"]),
        "map_image": Path(report["inputs"]["map_image"]),
        "world": Path(report["inputs"]["world"]),
        "usda": Path(report["inputs"]["usda"]),
    }
    expect(checks, "alignment_inputs_exist", all(path.is_file() for path in input_paths.values()))
    expect(
        checks,
        "alignment_input_hashes_current",
        all(
            sha256(path) == report["inputs"][f"{name}_sha256"]
            for name, path in input_paths.items()
        ),
    )

    controller = config["controller_server"]["ros__parameters"]
    mppi = controller["FollowPath"]
    expect(checks, "mppi_omni", mppi.get("motion_model") == "Omni")
    expect(checks, "controller_20hz", float(controller["controller_frequency"]) == 20.0)
    expect(checks, "mppi_model_dt_005", float(mppi["model_dt"]) == 0.05)
    expect(checks, "lateral_threshold_small", float(controller["min_y_velocity_threshold"]) <= 0.001)
    expect(checks, "lateral_velocity_enabled", float(mppi["vy_max"]) > 0.0 and float(mppi["vy_std"]) > 0.0)
    expect(
        checks,
        "isaac_velocity_limits_respected",
        math.hypot(float(mppi["vx_max"]), float(mppi["vy_max"])) <= 0.6
        and float(mppi["wz_max"]) <= 1.5,
    )
    expect(
        checks,
        "unsupported_mppi_acceleration_absent",
        not any(key in mppi for key in ("ax_max", "ax_min", "ay_max", "az_max")),
    )
    expect(checks, "prefer_forward_critic_absent", "PreferForwardCritic" not in mppi["critics"])
    expect(
        checks,
        "humble_path_angle_parameters",
        "mode" not in mppi["PathAngleCritic"]
        and mppi["PathAngleCritic"].get("forward_preference") is False,
    )
    expect(checks, "controller_publishes_final_zero", controller.get("publish_zero_velocity") is True)

    footprint_strings = []
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        footprint_strings.append(params["footprint"])
        layers = params["plugins"]
        expect(
            checks,
            f"{costmap_name}_uses_merged_scan_only",
            params["obstacle_layer"]["observation_sources"] == "scan_merged"
            and params["obstacle_layer"]["scan_merged"]["topic"] == "/scan_merged"
            and params["obstacle_layer"]["scan_merged"].get("inf_is_valid") is True
            and "/scan_01" not in json.dumps(params)
            and "/scan_02" not in json.dumps(params),
        )
        expect(checks, f"{costmap_name}_has_obstacle_inflation", {"obstacle_layer", "inflation_layer"} <= set(layers))
        source = params["obstacle_layer"]["scan_merged"]
        expect(
            checks,
            f"{costmap_name}_sensor_range_contract",
            float(source["obstacle_min_range"]) == 0.45
            and float(source["obstacle_max_range"]) == 10.0
            and float(source["raytrace_min_range"]) == 0.45
            and float(source["raytrace_max_range"]) == 12.0
            and float(source["max_obstacle_height"]) == 2.0,
        )
        expect(
            checks,
            f"{costmap_name}_inflation_contract",
            float(params["inflation_layer"]["inflation_radius"]) == 0.55
            and float(params["inflation_layer"]["cost_scaling_factor"]) == 3.0,
        )
        expect(
            checks,
            f"{costmap_name}_transform_tolerance",
            float(params["transform_tolerance"]) == 0.20,
        )
    expect(
        checks,
        "global_static_local_rolling_structure",
        "static_layer" in config["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
        and "static_layer" not in config["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
        and config["local_costmap"]["local_costmap"]["ros__parameters"]["rolling_window"] is True
        and config["global_costmap"]["global_costmap"]["ros__parameters"]["rolling_window"] is False,
    )
    expect(checks, "footprints_identical", len(set(footprint_strings)) == 1)
    footprint = ast.literal_eval(footprint_strings[0])
    footprint_x = max(point[0] for point in footprint) - min(point[0] for point in footprint)
    footprint_y = max(point[1] for point in footprint) - min(point[1] for point in footprint)
    expect(checks, "footprint_dimensions_070x056", abs(footprint_x - 0.70) < 1.0e-9 and abs(footprint_y - 0.56) < 1.0e-9)
    expect(checks, "robot_radius_absent", not contains_key(config, "robot_radius"))
    local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
    expect(
        checks,
        "costmap_frame_and_rate_contract",
        local_params["global_frame"] == "odom"
        and global_params["global_frame"] == "map"
        and local_params["robot_base_frame"] == "base_link"
        and global_params["robot_base_frame"] == "base_link"
        and float(local_params["update_frequency"]) == 10.0
        and float(local_params["publish_frequency"]) == 5.0
        and float(global_params["update_frequency"]) == 2.0
        and float(global_params["publish_frequency"]) == 1.0
        and type(local_params["width"]) is int
        and type(local_params["height"]) is int
        and local_params["width"] == 6
        and local_params["height"] == 6
        and float(local_params["resolution"]) == 0.05
        and float(global_params["resolution"]) == 0.05,
    )
    smoother = config["velocity_smoother"]["ros__parameters"]
    expect(
        checks,
        "omni_velocity_and_smoother_contract",
        [float(value) for value in smoother["max_velocity"]] == [0.45, 0.30, 1.0]
        and [float(value) for value in smoother["min_velocity"]]
        == [-0.25, -0.30, -1.0]
        and float(smoother["smoothing_frequency"]) == 20.0
        and float(smoother["velocity_timeout"]) == 0.50,
    )
    expect(
        checks,
        "planner_known_space_only",
        config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]
        is False,
    )

    map_server = config["map_server"]["ros__parameters"]
    expect(checks, "map_server_sim_time", map_server["use_sim_time"] is True)
    expect(checks, "map_server_frame", map_server["frame_id"] == "map")
    expect(checks, "map_yaml_exists", Path(map_server["yaml_filename"]).is_file())
    expect(
        checks,
        "all_nav_nodes_use_sim_time",
        all(
            section["ros__parameters"].get("use_sim_time") is True
            for name, section in config.items()
            if name not in ("local_costmap", "global_costmap")
        )
        and config["local_costmap"]["local_costmap"]["ros__parameters"]["use_sim_time"] is True
        and config["global_costmap"]["global_costmap"]["ros__parameters"]["use_sim_time"] is True,
    )

    launch_text = LAUNCH_FILE.read_text(encoding="utf-8")
    expect(checks, "launch_has_ground_truth_tf", "level3_ground_truth_map_to_odom" in launch_text)
    expect(checks, "launch_has_no_amcl_slam_arena", all(word not in launch_text for word in ("nav2_amcl", "slam_toolbox", "arena_ws")))
    expect(
        checks,
        "launch_has_velocity_smoother_remap",
        launch_text.count('remappings=[("cmd_vel", "cmd_vel_nav")]') == 2
        and '("cmd_vel_smoothed", "cmd_vel")' in launch_text,
    )
    expect(
        checks,
        "launch_uses_native_goal_pose",
        "goal_pose_adapter.py" not in launch_text
        and 'name="bt_navigator"' in launch_text,
    )
    description = load_launch_description()
    expect(checks, "launch_description_constructs", len(description.entities) == 10)
    expected_nodes = {
        ("tf2_ros", "static_transform_publisher", "level3_ground_truth_map_to_odom"),
        ("nav2_map_server", "map_server", "map_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
        ("nav2_lifecycle_manager", "lifecycle_manager", "lifecycle_manager_level3"),
    }
    actual_nodes = {
        (
            getattr(entity, "_Node__package", None),
            getattr(entity, "_Node__node_executable", None),
            getattr(entity, "_Node__node_name", None),
        )
        for entity in description.entities
        if hasattr(entity, "_Node__package")
    }
    expect(checks, "launch_exact_node_inventory", actual_nodes == expected_nodes)
    lifecycle_node_names = (
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    )
    expect(
        checks,
        "lifecycle_manager_exact_inventory",
        all(f'"{name}"' in launch_text for name in lifecycle_node_names)
        and '"autostart": True' in launch_text,
    )
    forbidden_launch_terms = (
        "nav2_amcl",
        "slam_toolbox",
        "rviz2",
        "gazebo",
        "isaacsim",
        "arena_ws",
    )
    expect(
        checks,
        "launch_forbidden_runtime_absent",
        all(
            term not in launch_text.lower().replace("simulator", "")
            for term in forbidden_launch_terms
        ),
    )
    expect(
        checks,
        "launch_tf_direction_and_frames",
        '"--frame-id", "map"' in launch_text
        and '"--child-frame-id", "odom"' in launch_text,
    )

    bt_plugins = config["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    expect(
        checks,
        "all_bt_plugin_libraries_installed",
        all(
            (Path("/opt/ros/humble/lib") / f"lib{plugin}.so").is_file()
            for plugin in bt_plugins
        ),
    )
    expect(
        checks,
        "bt_internal_clients_use_sim_time",
        all(
            config[name]["ros__parameters"].get("use_sim_time") is True
            for name in (
                "bt_navigator_navigate_through_poses_rclcpp_node",
                "bt_navigator_navigate_to_pose_rclcpp_node",
            )
        ),
    )

    obstacle_wrapper = (LEVEL3_ROOT / "send_obstacle_goal.sh").read_text(encoding="utf-8")
    first_wrapper = (LEVEL3_ROOT / "send_first_goal.sh").read_text(encoding="utf-8")
    goal_client_text = (SCRIPT_DIR / "send_test_goal.py").read_text(encoding="utf-8")
    runtime_common_text = (SCRIPT_DIR / "runtime_common.py").read_text(encoding="utf-8")
    omni_client_text = (SCRIPT_DIR / "send_omni_follow_path.py").read_text(encoding="utf-8")
    preflight_text = (SCRIPT_DIR / "validate_level3_runtime.py").read_text(encoding="utf-8")
    collision_checker_text = (SCRIPT_DIR / "check_isaac_collision_result.py").read_text(encoding="utf-8")
    expect(
        checks,
        "obstacle_test_requires_measured_detour",
        "--require-detour" in obstacle_wrapper
        and "plan_detour_ratio >= 1.02" in goal_client_text
        and '"/plan"' in runtime_common_text,
    )
    expect(
        checks,
        "runtime_goal_acceptance_contract",
        "node.plans" in goal_client_text
        and 'plan.header.frame_id != "map"' in goal_client_text
        and "initial_map.x" in goal_client_text
        and "poses[-1].pose.position.x - args.x" in goal_client_text
        and "position_error <= 0.30" in goal_client_text
        and "yaw_error <= 0.35" in goal_client_text
        and "displacement >= 0.10" in goal_client_text,
    )
    expect(
        checks,
        "runtime_final_stop_contract",
        "last_odom_twist" in runtime_common_text
        and "received_fresh_odom" in runtime_common_text
        and "final_zero" in goal_client_text
        and "final_zero" in omni_client_text,
    )
    expect(
        checks,
        "runtime_omni_acceptance_contract",
        "maximum_abs_vy > 0.05" in omni_client_text
        and "lateral_displacement >= 0.50" in omni_client_text
        and "maximum_yaw_excursion <= 0.25" in omni_client_text,
    )
    expect(
        checks,
        "runtime_tf_authority_contract",
        "map_to_odom_not_dynamic" in preflight_text
        and "tf_static_publishers_exact" in preflight_text
        and "tf_dynamic_only_from_bridge" in preflight_text,
    )
    expect(
        checks,
        "runtime_empty_graph_diagnostic",
        '"runtime_graph_present": not runtime_graph_empty' in preflight_text
        and "RUNTIME_GRAPH_EMPTY=FAIL" in preflight_text
        and "Terminal 1 and Terminal 2" in preflight_text,
    )
    expect(
        checks,
        "runtime_collision_acceptance_contract",
        "robot_collision_protection_enabled" in collision_checker_text
        and "collision_blocked_count_zero" in collision_checker_text
        and "custom_static_scene" in collision_checker_text
        and "goal_and_isaac_motion_agree" in collision_checker_text,
    )
    obstacle_goal = route_config["obstacle_goal"]
    expect(
        checks,
        "obstacle_wrapper_matches_checked_route",
        all(
            str(float(obstacle_goal[key])) in obstacle_wrapper
            for key in ("x_m", "y_m", "yaw_rad")
        ),
    )
    first_goal = route_config["first_goal"]
    expect(
        checks,
        "first_goal_wrapper_matches_checked_route",
        all(
            f"{float(first_goal[key]):.9f}" in first_wrapper
            for key in ("x_m", "y_m", "yaw_rad")
        ),
    )
    readme_text = (LEVEL3_ROOT / "README.md").read_text(encoding="utf-8")
    provenance_terms = (
        "Parameter provenance",
        "First-run engineering baseline",
        "Existing merger and LaserScan contract",
        "Isaac collision proxy",
    )
    expect(
        checks,
        "engineering_parameter_provenance_documented",
        all(term in readme_text for term in provenance_terms),
    )

    python_files = list(LEVEL3_ROOT.rglob("*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    expect(checks, "python_ast", True)
    from rclpy.node import Node

    node_init_ast = ast.parse(textwrap.dedent(inspect.getsource(Node.__init__)))
    node_instance_attributes = self_attributes_assigned(node_init_ast)
    preflight_ast = ast.parse(preflight_text, filename=str(SCRIPT_DIR / "validate_level3_runtime.py"))
    preflight_callbacks = subscription_callback_names(preflight_ast)
    expect(
        checks,
        "runtime_callback_bindings",
        bool(preflight_callbacks)
        and preflight_callbacks.isdisjoint(node_instance_attributes)
        and "_on_clock" in preflight_callbacks,
    )
    import_runtime_modules()
    expect(checks, "runtime_module_imports_without_execution", True)
    plugin_xml_paths = (
        Path("/opt/ros/humble/share/nav2_mppi_controller/mppic.xml"),
        Path("/opt/ros/humble/share/nav2_mppi_controller/critics.xml"),
        Path("/opt/ros/humble/share/nav2_controller/plugins.xml"),
        Path("/opt/ros/humble/share/nav2_behaviors/behavior_plugin.xml"),
        Path("/opt/ros/humble/share/nav2_navfn_planner/global_planner_plugin.xml"),
        Path("/opt/ros/humble/share/nav2_costmap_2d/costmap_plugins.xml"),
    )
    registered_plugin_names: set[str] = set()
    registered_plugin_types: set[str] = set()
    for xml_path in plugin_xml_paths:
        ET.parse(xml_path)
        for plugin_class in ET.parse(xml_path).getroot().iter("class"):
            if plugin_class.get("name"):
                registered_plugin_names.add(str(plugin_class.get("name")))
            if plugin_class.get("type"):
                registered_plugin_types.add(str(plugin_class.get("type")))
    expect(checks, "installed_plugin_xml", True)
    configured_plugin_classes = {
        mppi["plugin"],
        controller["progress_checker"]["plugin"],
        controller["goal_checker"]["plugin"],
        config["planner_server"]["ros__parameters"]["GridBased"]["plugin"],
        *(config["behavior_server"]["ros__parameters"][name]["plugin"] for name in config["behavior_server"]["ros__parameters"]["behavior_plugins"]),
        *(config[map_name][map_name]["ros__parameters"][layer]["plugin"] for map_name in ("local_costmap", "global_costmap") for layer in config[map_name][map_name]["ros__parameters"]["plugins"]),
    }
    expect(
        checks,
        "all_configured_plugins_registered",
        all(
            plugin in registered_plugin_names or plugin in registered_plugin_types
            for plugin in configured_plugin_classes
        ),
    )
    registered_critics = {
        plugin_type.rsplit("::", 1)[-1]
        for plugin_type in registered_plugin_types
        if plugin_type.startswith("mppi::critics::")
    }
    expect(
        checks,
        "all_configured_mppi_critics_registered",
        set(mppi["critics"]) <= registered_critics,
    )
    installed_mppi = (
        Path("/opt/ros/humble/lib/libmppi_controller.so").read_bytes()
        + Path("/opt/ros/humble/lib/libmppi_critics.so").read_bytes()
    )
    required_parameter_symbols = (
        b"motion_model",
        b"vx_max",
        b"vx_min",
        b"vy_max",
        b"wz_max",
        b"retry_attempt_limit",
        b"reset_period",
        b"regenerate_noises",
        b"consider_footprint",
        b"inflation_layer_name",
        b"max_path_occupancy_ratio",
        b"forward_preference",
    )
    expect(
        checks,
        "installed_mppi_parameter_symbols",
        all(symbol in installed_mppi for symbol in required_parameter_symbols),
    )
    system_packages = (
        "nav2_map_server",
        "nav2_planner",
        "nav2_navfn_planner",
        "nav2_controller",
        "nav2_mppi_controller",
        "nav2_costmap_2d",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_velocity_smoother",
        "nav2_lifecycle_manager",
    )
    package_versions = {
        package: ET.parse(f"/opt/ros/humble/share/{package}/package.xml")
        .getroot()
        .findtext("version")
        for package in system_packages
    }
    expect(
        checks,
        "system_nav2_versions_1_1_20",
        set(package_versions.values()) == {"1.1.20"},
    )

    # The old launch files remain untouched, but the new bringup neither imports
    # nor includes any of their identity map->odom publishers.
    known_identity_publishers = [
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/launch"
        / "teleop_fixed_map_capture.launch.py",
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/launch"
        / "semantic_cnn_fixed_dual_start_goal_demo.launch.py",
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/launch"
        / "semantic_cnn_v7_start_goal_demo.launch.py",
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/launch"
        / "tokennav_v7_start_goal_demo.launch.py",
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/launch"
        / "drl_vo_fixed_dual_start_goal_demo.launch.py",
    ]
    expect(checks, "known_identity_publishers_documented", all(path.is_file() for path in known_identity_publishers))
    expect(checks, "standalone_does_not_include_identity_launches", all(path.name not in launch_text for path in known_identity_publishers))

    passed = all(checks.values())
    OFFLINE_REPORT.parent.mkdir(parents=True, exist_ok=True)
    OFFLINE_REPORT.write_text(
        json.dumps(
            {
                "schema": "a_pipeline_offline_level3_validation/v1",
                "status": "PASS" if passed else "FAIL",
                "checks": checks,
                "system_nav2_package_versions": package_versions,
                "runtime_pending": [
                    "current ROS topic and TF rates",
                    "Nav2 lifecycle activation",
                    "map/scan live overlay",
                    "NavigateToPose motion and success",
                    "static-obstacle route",
                    "MPPI Omni nonzero linear.y",
                    "Isaac collision result",
                ],
                "safety": {
                    "isaac_started_by_validator": False,
                    "gazebo_started_by_validator": False,
                    "rviz_started_by_validator": False,
                    "gpu_workload_started_by_validator": False,
                    "training_started_by_validator": False,
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    for name, value in checks.items():
        print(f"STATIC_{name.upper()}={'PASS' if value else 'FAIL'}")
    print(f"OFFLINE_LEVEL3_PREP={'PASS' if passed else 'FAIL'}")
    print(f"REPORT={OFFLINE_REPORT}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
