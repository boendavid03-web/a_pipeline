from pathlib import Path
import importlib.util

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[5]
GOALS_FILE = PROJECT_ROOT / "configs" / "evaluation" / "fixed_four_goals.yaml"
SEQUENCE_SCRIPT = (
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "scripts"
    / "fixed_goal_sequence.py"
)
FIXED_LAUNCH_FILES = (
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "launch"
    / "semantic_cnn_fixed_dual_start_goal_demo.launch.py",
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "launch"
    / "drl_vo_fixed_dual_start_goal_demo.launch.py",
)


def test_fixed_four_goals_are_reproducible_and_ordered():
    payload = yaml.safe_load(GOALS_FILE.read_text(encoding="utf-8"))
    assert payload["schema"] == "fixed_navigation_goal_suite/v1"
    goals = payload["goals"]
    assert len(goals) == 4
    assert [goal["id"] for goal in goals] == [
        "goal_01",
        "goal_02",
        "goal_03",
        "goal_04",
    ]
    assert all(
        isinstance(goal["x"], (int, float)) and isinstance(goal["y"], (int, float))
        for goal in goals
    )


def test_fixed_goal_sequence_is_ros_executable():
    assert SEQUENCE_SCRIPT.is_file()
    assert SEQUENCE_SCRIPT.stat().st_mode & 0o111


def test_fixed_goal_sequence_leaves_use_sim_time_to_rclpy():
    script = SEQUENCE_SCRIPT.read_text(encoding="utf-8")
    assert 'declare_parameter("use_sim_time"' not in script


def load_sequence_module():
    spec = importlib.util.spec_from_file_location("fixed_goal_sequence", SEQUENCE_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fixed_goal_preflight_requires_all_real_inputs():
    module = load_sequence_module()
    assert module.missing_preflight_inputs(set()) == (
        "clock",
        "odom",
        "scan_01",
        "scan_02",
        "pedestrian_ground_truth",
    )
    assert module.missing_preflight_inputs(set(module.REQUIRED_PREFLIGHT_INPUTS)) == ()


def test_fixed_goal_preflight_requires_clock_progress():
    module = load_sequence_module()
    assert not module.clock_has_advanced(None, 1)
    assert not module.clock_has_advanced(1, 1)
    assert not module.clock_has_advanced(2, 1)
    assert module.clock_has_advanced(1, 2)


def test_fixed_launches_auto_shutdown_after_final_goal():
    for launch_file in FIXED_LAUNCH_FILES:
        source = launch_file.read_text(encoding="utf-8")
        assert '"fixed_test_auto_shutdown_delay_sec"' in source
        assert '"auto_shutdown_delay_sec"' in source
        assert 'Shutdown(reason="Fixed goal sequence exited")' in source
