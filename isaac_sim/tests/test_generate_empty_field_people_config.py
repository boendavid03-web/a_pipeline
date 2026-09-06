from __future__ import annotations

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from generate_empty_field_people_config import make_config, out_and_back_routes  # noqa: E402


EMPTY_SCENE = ROOT / "isaac_sim/scenes/a_pipeline_empty_people.usda"


def test_eight_routes_are_closed_bounded_and_cover_many_headings():
    routes = out_and_back_routes(8, 21)

    assert len(routes) == 8
    assert all(route[0] == route[-1] for route in routes)
    assert all(len(route) == 3 for route in routes)
    assert all(abs(point[0]) <= 11.0 and abs(point[1]) <= 8.5 for route in routes for point in route)
    headings = {
        round(
            math.atan2(
                route[1][1] - route[0][1],
                route[1][0] - route[0][0],
            ),
            3,
        )
        for route in routes
    }
    assert len(headings) == 8


def test_config_uses_exact_speed_and_one_group_per_person():
    config = make_config(EMPTY_SCENE, count=8, seed=21, speed=0.8)
    root = config["isaacsim.replicator.agent"]
    groups = root["character"]["groups"]

    assert root["environment"]["base_stage_asset_path"] == str(EMPTY_SCENE.resolve())
    assert len(groups) == 8
    assert all(group["num"] == 1 for group in groups.values())
    assert all(
        group["routines"][0]["patrol"]["speed_range"] == [0.8, 0.8]
        for group in groups.values()
    )


def test_empty_usd_has_floor_and_navmesh_but_no_static_obstacles():
    source = EMPTY_SCENE.read_text(encoding="utf-8")

    assert 'def NavMeshVolume "NavMeshVolume"' in source
    assert 'def Mesh "Ground"' in source
    assert "int staticObstacleCount = 0" in source
    assert 'def Xform "NavMeshObstacles"' not in source
