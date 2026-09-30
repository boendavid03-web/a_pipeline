"""Contract coverage for the legacy Arena scenario catalog.

These tests deliberately stay ROS/Isaac-free.  They catch schema regressions
before a long Isaac Sim launch and ensure that empty and stationary Arena
entries remain valid inputs to the native Isaac pedestrian importer.
"""

from pathlib import Path
import sys

import pytest


RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))

from arena_scenario import load_arena_pedestrians  # noqa: E402


SCENARIO_ROOT = Path("/home/user/arena_full_ws/src/arena/simulation-setup/worlds")


def _scenario_files() -> list[Path]:
    if not SCENARIO_ROOT.is_dir():
        return []
    return sorted(
        path for path in SCENARIO_ROOT.glob("*/scenarios/*.json")
        if not any(part.startswith(".") for part in path.parts)
    )


@pytest.mark.skipif(not SCENARIO_ROOT.is_dir(), reason="Arena source workspace is unavailable")
def test_every_legacy_arena_scenario_loads() -> None:
    files = _scenario_files()
    assert files
    for path in files:
        pedestrians = load_arena_pedestrians(path)
        assert all(person.waypoint_mode in (0, 1, 2) for person in pedestrians), path
        assert len({person.stable_id for person in pedestrians}) == len(pedestrians), path


@pytest.mark.skipif(not SCENARIO_ROOT.is_dir(), reason="Arena source workspace is unavailable")
def test_empty_and_blocked_corridors_contracts() -> None:
    empty = load_arena_pedestrians(SCENARIO_ROOT / "map_empty/scenarios/empty.json")
    marl = load_arena_pedestrians(SCENARIO_ROOT / "map_empty/scenarios/marl.json")
    blocked = load_arena_pedestrians(
        SCENARIO_ROOT / "map_empty/scenarios/blocked_corridors.json"
    )
    assert empty == ()
    assert marl == ()
    assert len(blocked) == 7
    assert sum(not person.route[1:] for person in blocked) == 6


@pytest.mark.skipif(not SCENARIO_ROOT.is_dir(), reason="Arena source workspace is unavailable")
def test_mode_two_scenario_keeps_all_waypoints() -> None:
    pedestrians = load_arena_pedestrians(SCENARIO_ROOT / "map_empty/scenarios/2.json")
    mode_two = [person for person in pedestrians if person.waypoint_mode == 2]
    assert len(mode_two) == 1
    assert len(mode_two[0].route) == 4
