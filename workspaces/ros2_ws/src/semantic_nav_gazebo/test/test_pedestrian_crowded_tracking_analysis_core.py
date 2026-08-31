from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from pedestrian_crowded_tracking_analysis_core import (  # noqa: E402
    AnalysisParameters,
    DISTANCE_BINS,
    build_connected_encounters,
    compute_static_line_of_sight,
    normalize_frames,
    assign_scan_support,
    reconstruct_identity_events,
)


def gt(identity, x, y):
    return {"id": identity, "x": x, "y": y}


def frame(t, distance=1.0, match=True, observable=True, tracks=(1, 2)):
    truths = [gt("a", 0.0, 0.0), gt("b", distance, 0.0)]
    return {"timestamp_ns": t, "ground_truth": truths,
            "pairwise_gt": [{"ids": ["a", "b"], "distance_m": distance}],
            "observability": {x: {"observable": observable} for x in ("a", "b")},
            "gt_track_matches": ([{"gt_id": "a", "track_id": tracks[0]}, {"gt_id": "b", "track_id": tracks[1]}] if match else []),
            "tracks": [{"track_id": x, "state": "CONFIRMED"} for x in tracks]}


def test_contract_defaults_and_bins_are_v2_ordered():
    params = AnalysisParameters()
    assert params.roi_radius_m == 8.0 and params.match_threshold_m == 0.5
    assert params.close_distance_m == 1.5 and params.component_gap_sec == 0.25
    assert params.acceptable_scan_rate_hz == (13.5, 16.5)
    assert [x[0] for x in DISTANCE_BINS] == [">=1.50", "1.00-1.50", "0.75-1.00", "0.50-0.75", "<0.50"]


def test_scan_support_nearest_tie_id_and_no_double_count():
    support = assign_scan_support(
        [{"x": 0.1, "y": 0.0}],
        [gt("b", 0.0, 0.0), gt("a", 0.0, 0.0)],
    )
    assert support["a"]["supported"] and not support["b"]["supported"]
    assert len({v["point_index"] for v in support.values() if v["supported"]}) == 1


def test_los_rotated_box_z_touch_endpoint_and_dual_sensor():
    box = {"pose": {"x": 2.0, "y": 0.0, "z": 0.5, "yaw": 0.7853981633974483}, "size": [1.0, 1.0, 1.0]}
    assert not compute_static_line_of_sight({"scan_01": [0, 0, 0.5]}, [4, 0, 0.5], [box])
    # The second exact sensor origin is clear, so one clear sensor suffices.
    assert compute_static_line_of_sight({"scan_01": [0, 0, 0.5], "scan_02": [0, 3, 0.5]}, [4, 0, 0.5], [box])
    # Closed Z range includes the beam plane; endpoint behind the box is not.
    assert compute_static_line_of_sight([[0, 0, 2.0]], [4, 0, 2.0], [box])


def test_normalize_encounter_and_identity_reconstruction():
    frames = normalize_frames([frame(0), frame(100_000_000), frame(200_000_000, match=False), frame(300_000_000, match=True, tracks=(3, 2))])
    encounters = build_connected_encounters(frames)
    assert encounters and encounters[0].minimum_distance_m == 1.0 and encounters[0].censored
    events = reconstruct_identity_events(frames)
    assert any(event.event in {"reacquisition_id_change", "fragmentation"} for event in events)
