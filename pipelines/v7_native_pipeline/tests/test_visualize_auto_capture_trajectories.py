"""Pure tests for auto-capture trajectory segmentation and map geometry."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "visualize_auto_capture_trajectories.py"
SPEC = importlib.util.spec_from_file_location("visualize_auto_capture_trajectories", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_map_world_pixel_conversion_preserves_y_flip_and_round_trip():
    kwargs = dict(width=627, height=477, resolution=0.05, origin_x=-0.0348, origin_y=0.1)
    for world in ((0.0, 0.0), (10.0, 12.5), (30.0, 20.0)):
        pixel = MODULE.map_world_to_pixel(*world, **kwargs)
        recovered = MODULE.map_pixel_to_world(*pixel, **kwargs)
        assert all(math.isclose(actual, expected, abs_tol=1e-12) for actual, expected in zip(recovered, world))
    assert MODULE.map_world_to_pixel(0.0, 1.0, **kwargs)[1] < MODULE.map_world_to_pixel(0.0, 0.0, **kwargs)[1]


def test_episode_intervals_keep_order_and_mark_missing_end_incomplete():
    events = [
        {"schema": "isaac_manual_teleop_episode/v1", "event": "start", "episode_id": 99, "stamp_ns": 0},
        {"schema": "semantic_nav_episode_event/v1", "event": "armed", "episode_id": 1, "stamp_ns": 1},
        {"schema": "semantic_nav_episode_event/v1", "event": "start", "episode_id": 1, "stamp_ns": 2, "goal": [1.0, 1.0]},
        {"schema": "semantic_nav_episode_event/v1", "event": "end", "episode_id": 1, "stamp_ns": 5, "reason": "goal_reached_and_stopped"},
        {"schema": "semantic_nav_episode_event/v1", "event": "start", "episode_id": 2, "stamp_ns": 8, "goal": [2.0, 2.0]},
    ]
    intervals = MODULE.build_episode_intervals(events, sim_end_ns=12)
    assert [item["episode_id"] for item in intervals] == [1, 2]
    assert intervals[0]["has_end"] is True
    assert intervals[0]["reason"] == "goal_reached_and_stopped"
    assert intervals[1]["has_end"] is False
    assert intervals[1]["reason"] == "incomplete_missing_end_event"
    assert intervals[1]["end_stamp_ns"] == 12


def test_unknown_episode_schema_remains_a_hard_error():
    try:
        MODULE.build_episode_intervals(
            [{"schema": "unknown/v1", "event": "start"}], sim_end_ns=1
        )
    except RuntimeError as error:
        assert "unsupported episode event schema" in str(error)
    else:
        raise AssertionError("unknown event schemas must not be silently ignored")


def test_episode_path_selection_rejects_stale_endpoint_and_keeps_actual_path():
    bag_data = {
        "intervals": [{
            "episode_id": 7,
            "start_stamp_ns": 20,
            "end_stamp_ns": 40,
            "goal": [5.0, 5.0],
            "start_pose": [0.0, 0.0],
            "end_pose": [4.9, 5.0],
            "reason": "human_collision_geometry_proxy",
            "has_end": True,
            "goal_accepted_stamp_ns": 18,
            "goal_accepted_goal_consistent": True,
            "association_window_start_ns": 10,
        }],
        "paths": [
            (18, [(0.0, 0.0), (99.0, 99.0)]),
            (19, [(0.0, 0.0), (5.0, 5.0)]),
        ],
        "odom": [(20, (0.0, 0.0)), (30, (2.0, 2.0)), (40, (4.9, 5.0))],
    }
    episode = MODULE._episode_data(bag_data)[0]
    assert episode["status"] == "failed"
    assert episode["planned"][-1] == (5.0, 5.0)
    assert episode["planned_path_stamp_ns"] == 19
    assert episode["actual"][-1] == (4.9, 5.0)
    assert episode["data_quality"]["planned_path_endpoint_matches_goal"] is True


def test_status_json_is_optional_for_interrupted_bags():
    args = MODULE.parse_args([
        "--bag", "bag", "--map-yaml", "map.yaml", "--semantic-label", "label.png",
        "--output-dir", "report",
    ])
    assert args.status_json is None
