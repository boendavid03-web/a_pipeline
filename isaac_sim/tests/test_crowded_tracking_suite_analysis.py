from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "isaac_sim/scripts/analyze_crowded_tracking_suite.py"
spec = importlib.util.spec_from_file_location("crowded_suite", SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_manifest_has_canonical_cells_and_required_entry_contract():
    manifest = json.loads((ROOT / "isaac_sim/config/crowded_tracking_suite_manifest_20260831.json").read_text())
    assert [entry["cell"] for entry in manifest["entries"]] == list(module.EXPECTED_CELLS)
    required = {"bag_files", "metadata", "canonical_hash", "producer_source_sha256", "launcher_sha256", "world_path", "scene_path", "world_sha256", "scene_sha256", "parameters", "output"}
    assert all(required <= set(entry) for entry in manifest["entries"])


def test_preflight_rejects_duplicate_or_missing_without_creating_output(tmp_path):
    manifest = {"schema": "isaac_crowded_tracking_suite/v2", "entries": []}
    path = tmp_path / "manifest.json"; path.write_text(json.dumps(manifest))
    try:
        module.validate_manifest(manifest, path)
    except ValueError as error:
        assert "canonical ten cells" in str(error)
    else:
        raise AssertionError("invalid manifest unexpectedly accepted")
    assert not (tmp_path / "out").exists()


def test_canonical_hash_is_deterministic_and_path_delimited(tmp_path):
    first = tmp_path / "z.db3"; second = tmp_path / "a.db3"
    first.write_bytes(b"one"); second.write_bytes(b"two")
    assert module.canonical_bag_hash([first, second]) == module.canonical_bag_hash([second, first])


def test_aggregator_is_manifest_only_and_v2():
    source = SCRIPT.read_text()
    assert "isaac_crowded_tracking_suite/v2" in source
    assert "canonical_bag_hash" in source and "evaluation_recomputed_v2" in (ROOT / "isaac_sim/config/crowded_tracking_suite_manifest_20260831.json").read_text()


def _frame(stamp, distance, ids=("a", "b"), detection_ids=("a", "b"), track_ids=(1, 2), observable_ids=("a", "b")):
    truth = [{"id": identity, "x": index * distance, "y": 0.0} for index, identity in enumerate(ids)]
    pairs = []
    for left, first in enumerate(ids):
        for right, second in enumerate(ids[left + 1:], start=left + 1):
            pairs.append({"ids": [first, second], "distance_m": abs(right - left) * distance})
    return {
        "schema": "pedestrian_crowded_tracking_trace/v2",
        "timestamp_ns": stamp,
        "ground_truth": truth,
        "pairwise_gt": pairs,
        "observability": {identity: {"observable": identity in observable_ids, "reason": "OBSERVABLE" if identity in observable_ids else "NO_SCAN_SUPPORT"} for identity in ids},
        "gt_detection_matches": [{"gt_id": identity, "detection_index": index, "position_error_m": 0.05} for index, identity in enumerate(detection_ids)],
        "gt_track_matches": [{"gt_id": identity, "track_id": track_ids[index]} for index, identity in enumerate(ids) if track_ids[index] is not None],
    }


def test_distance_boundary_and_completed_separation_event_are_data_driven():
    assert module.distance_bin(1.5) == ">=1.50"
    frames = [
        _frame(0, 1.0),
        _frame(100_000_000, 1.0, detection_ids=("a",)),
        _frame(200_000_000, 1.0, detection_ids=("a",)),
        _frame(300_000_000, 1.0),
    ]
    encounters = module.build_encounters(frames, "C")
    events = module.build_separation_events(frames, encounters)
    assert len(events) == 1
    assert events[0]["event_kind"] == "detector_separation_event"


def test_tracker_native_requires_half_second_detection_continuity():
    frames = [_frame(index * 100_000_000, 1.2, track_ids=(1 if index < 6 else 3, 2)) for index in range(7)]
    encounters = module.build_encounters(frames, "C")
    events = module.reconstruct_identity_events(frames, encounters, [])
    continuous = [event for event in events if event["event_kind"] == "continuous_id_switch"]
    assert len(continuous) == 1
    assert continuous[0]["attribution"] == "TRACKER_ASSOCIATION_FAILURE"


def test_three_person_pair_bins_use_mutual_nearest_neighbour():
    frame = _frame(0, 0.8, ids=("a", "b", "c"), detection_ids=("a", "b", "c"), track_ids=(1, 2, 3), observable_ids=("a", "b", "c"))
    bins = module.aggregate_bins([frame], [])
    assert bins["0.75-1.00"]["observable_pair_frame_denominator"] == 2
    assert bins[">=1.50"]["observable_pair_frame_denominator"] == 0


def test_answers_and_bottleneck_change_with_fixture_data():
    bins = {label: {**module.fresh_bin(), "unique_frame_count": 0, "detector_recall": None, "separation_success_rate": None} for label in module.BIN_ORDER}
    for value in bins.values():
        value.pop("unique_frame_indices")
    far = bins[">=1.50"]
    far.update(observable_pair_frame_denominator=2, separated_pair_frame_numerator=2, separation_success_rate=1.0)
    base_run = {"scenario": "A", "cell": "A1.50", "episode_validity": "VALID", "algorithm_status": "PASS", "detector_status": "PASS", "detector_separation_events": 0, "attribution_counts": {}, "event_counts": {}, "identity_events": []}
    answers_pass = module.derive_answers([base_run], bins)
    assert answers_pass[-1]["answer"] == "NEITHER IN TESTED RANGE"
    failing = {**base_run, "detector_status": "FAIL", "algorithm_status": "FAIL", "identity_events": [{"event_kind": "continuous_id_switch", "attribution": "TRACKER_ASSOCIATION_FAILURE"}]}
    answers_fail = module.derive_answers([failing], bins)
    assert answers_fail[-1]["answer"] == "BOTH"
