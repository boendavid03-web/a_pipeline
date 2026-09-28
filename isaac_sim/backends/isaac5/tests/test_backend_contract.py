from __future__ import annotations

import math
import sys
from pathlib import Path


BACKEND = Path(__file__).resolve().parents[1]
RUNTIME = BACKEND / "runtime"
sys.path.insert(0, str(RUNTIME))

from contract import finite_pose, generated_output_path, measured_rate  # noqa: E402


def test_generated_output_is_backend_local(tmp_path):
    root = tmp_path / "isaac5"
    assert generated_output_path(root, None) is None
    assert generated_output_path(root, Path("smoke.usda")) == (root / "generated/smoke.usda").resolve()
    try:
        generated_output_path(root, Path("../scenes/escape.usda"))
    except ValueError:
        pass
    else:
        raise AssertionError("output outside generated/ must be rejected")


def test_rate_and_pose_helpers_are_observation_based():
    assert math.isclose(measured_rate([(0.0, 1.0), (0.1, 1.05), (0.2, 1.1)]), 20.0)
    assert measured_rate([(0.0, 0.0)]) is None
    assert finite_pose([0.0, 1.0, 2.0], [1.0, 0.0, 0.0, 0.0])
    assert not finite_pose([float("nan"), 1.0, 2.0], [1.0, 0.0, 0.0, 0.0])


def test_backend_does_not_import_isaac6_only_runtime_apis():
    forbidden = (
        "isaacsim.sensors.experimental." + "physics",
        "isaacsim.sensors.experimental." + "rtx",
        "IRA_" + "Character",
        "SimulationManager." + "setup_simulation",
    )
    files = [path for path in BACKEND.rglob("*") if path.is_file() and path.name != __file__]
    for path in files:
        if path.suffix not in {".py", ".sh", ".md", ".json"}:
            continue
        text = path.read_text(encoding="utf-8")
        assert not any(token in text for token in forbidden), path


def test_people_assets_are_backend_owned_data_only():
    pedestrian = (RUNTIME / "validate_pedestrian.py").read_text(encoding="utf-8")
    crowd = (RUNTIME / "validate_crowd.py").read_text(encoding="utf-8")
    people = BACKEND / "assets/people"
    assert "biped_demo_meters.usd" in pedestrian
    assert "stand_walk_loop_in_place.skelanim.usd" in pedestrian
    assert 'ASSET_ROOT = BACKEND_ROOT / "assets/people"' in pedestrian
    assert "isaacsim-6.0.1" not in pedestrian
    assert "assets-6.0.1" not in pedestrian
    assert (people / "asset_manifest.json").is_file()
    assert (people / "SHA256SUMS").is_file()
    assert (people / "licenses/NVIDIA_OMNIVERSE_LICENSE_SOURCE.txt").is_file()
    assert "omni.anim.retarget.core" in pedestrian
    assert "from isaacsim.replicator.agent" not in pedestrian
    assert "load_arena_pedestrians" in crowd
    assert (BACKEND / "config/arena_eng_lobby_pedestrians.json").is_file()
    assert "map_wide_initial_distribution" in crowd
    assert "cumulative_forward_distance_m" in crowd
    assert "--native-stall-replan-after" in pedestrian
    assert "native_stall_route_replan" in crowd
    assert 'metrics["executed_acceleration_max"]' in crowd
    assert 'controller_summary["maximum_speed_mps"]' in crowd


def test_runtime_and_launch_have_no_direct_isaac6_asset_path():
    forbidden = ("isaacsim-6.0.1", "assets-6.0.1")
    for directory in (RUNTIME, BACKEND / "launch"):
        for path in directory.rglob("*"):
            if path.is_file() and path.suffix in {".py", ".sh"}:
                text = path.read_text(encoding="utf-8")
                assert not any(token in text for token in forbidden), path


def test_gate9_launchers_preserve_bounded_backend_local_control():
    launcher = (BACKEND / "launch/validate_gate81_drlvo_shadow.sh").read_text(encoding="utf-8")
    mobile = (RUNTIME / "gate9_mobile_robot.py").read_text(encoding="utf-8")
    assert "/isaac5/gate9/cmd_vel" in launcher
    assert "/isaac5/gate9/fixed_goal_cmd_vel" in launcher
    assert "-p max_linear:=0.3" in launcher
    assert "np.clip(linear, -0.30, 0.30)" in mobile
    assert "mecanum730_xms5_default_base.usd" in mobile
    assert "authored_visual_pose_on_dynamic_base" in mobile
    assert "minimum_visual_bottom_z" in mobile


def test_hunav_host_uses_task_owned_external_goal_authority():
    launcher = (BACKEND / "launch/run_hunav_host.sh").read_text(encoding="utf-8")
    crowd = (RUNTIME / "validate_crowd.py").read_text(encoding="utf-8")
    adapter = (RUNTIME / "hunav_isaac_adapter.py").read_text(encoding="utf-8")
    assert "build_isaac_goal_sync/hunav_agent_manager/hunav_agent_manager" in launcher
    assert "external_goals_authoritative:=true" in launcher
    assert '"goals": route_state.goals_xy()[:1]' in crowd
    assert '"hunav_external_goal_sync"' in crowd
    assert '"goal_sync_error_m"' in adapter


def test_backend_writes_only_generated_output():
    original_scene = str(Path("isaac_sim") / "scenes")
    original_logs = str(Path("isaac_sim") / "scripts" / "logs")
    for path in (BACKEND / "runtime/run_navigation.py", BACKEND / "runtime/validate_robot.py"):
        text = path.read_text(encoding="utf-8")
        assert original_scene not in text
        assert original_logs not in text
        assert "generated" in text
