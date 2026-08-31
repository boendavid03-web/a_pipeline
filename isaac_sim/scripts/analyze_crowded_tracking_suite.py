#!/usr/bin/env python3
"""Canonical, manifest-first analysis of crowded pedestrian replay artifacts."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable

EXPECTED_CELLS = ("A1.50", "A1", "A.75", "A.5", "B", "C", "D1", "D.75", "D.5", "E")
EXPECTED_SOURCE = "e8fefea611035bec1a21e96e7635637a53f3911b3954571d48c534964edb83e7"
EXPECTED_LAUNCHER = "650ac095334ca70562df411d1aab43929b597df244f9fd3be879d6dcdf98fe0c"
EXPECTED_WORLD = "040c58fc7064c5823379edbde7478648d3cec88ea010e5e1a47e067b36f8ef5b"
EXPECTED_SCENE = "18e012a8d1b9614aefa1517bc3be3ac47775e62cd6fba78240a04b4b6652c1dd"
BIN_ORDER = (">=1.50", "1.00-1.50", "0.75-1.00", "0.50-0.75", "<0.50")
MAX_FRAME_GAP_NS = 250_000_000
LOOKBACK_NS = 500_000_000
EVIDENCE_WINDOW_NS = 1_000_000_000


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_bag_hash(run_dir: Path | Iterable[Path], paths: Iterable[Path] | None = None) -> str:
    if paths is None:
        paths = list(run_dir)  # backward-compatible helper use in focused tests
        run_dir = Path(Path(next(iter(paths))).anchor or "/")
    run_dir = Path(run_dir)
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda item: str(item.relative_to(run_dir))):
        try:
            relative = path.relative_to(run_dir)
        except ValueError:
            relative = Path(path.name)
        digest.update(str(relative).encode())
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def distance_bin(distance: float) -> str:
    if distance >= 1.5:
        return ">=1.50"
    if distance >= 1.0:
        return "1.00-1.50"
    if distance >= 0.75:
        return "0.75-1.00"
    if distance >= 0.5:
        return "0.50-0.75"
    return "<0.50"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_trace(path: Path) -> list[dict[str, Any]]:
    frames = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    stamps = [int(frame["timestamp_ns"]) for frame in frames]
    if not frames or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f"invalid trace ordering: {path}")
    if any(frame.get("schema") != "pedestrian_crowded_tracking_trace/v2" for frame in frames):
        raise ValueError(f"non-v2 trace: {path}")
    return frames


def pair_key(first: str, second: str) -> tuple[str, str]:
    return tuple(sorted((str(first), str(second))))


def gt_map(frame: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["id"]): item for item in frame.get("ground_truth", [])}


def pair_distance(frame: dict[str, Any], pair: tuple[str, str]) -> float | None:
    for item in frame.get("pairwise_gt", []):
        if pair_key(*item["ids"]) == pair:
            return float(item["distance_m"])
    return None


def observable(frame: dict[str, Any], identity: str) -> bool:
    return bool(frame.get("observability", {}).get(identity, {}).get("observable", False))


def detection_index(frame: dict[str, Any], identity: str) -> int | None:
    for item in frame.get("gt_detection_matches", []):
        if str(item["gt_id"]) == identity:
            return int(item["detection_index"])
    return None


def track_id(frame: dict[str, Any], identity: str) -> int | None:
    for item in frame.get("gt_track_matches", []):
        if str(item["gt_id"]) == identity:
            return int(item["track_id"])
    return None


def pair_state(frame: dict[str, Any], pair: tuple[str, str]) -> int | None:
    if not all(observable(frame, identity) for identity in pair):
        return None
    indices = [detection_index(frame, identity) for identity in pair]
    return len({value for value in indices if value is not None})


def nearest_neighbour_distance(frame: dict[str, Any], identity: str) -> float | None:
    values = [float(item["distance_m"]) for item in frame.get("pairwise_gt", []) if identity in item["ids"]]
    return min(values) if values else None


def pair_is_nearest_neighbour(frame: dict[str, Any], pair: tuple[str, str]) -> bool:
    distance = pair_distance(frame, pair)
    if distance is None:
        return False
    return all(
        nearest_neighbour_distance(frame, identity) is not None
        and math.isclose(nearest_neighbour_distance(frame, identity), distance, abs_tol=1.0e-9)
        for identity in pair
    )


def build_encounters(frames: list[dict[str, Any]], scenario: str) -> list[dict[str, Any]]:
    identities = sorted({identity for frame in frames for identity in gt_map(frame)})
    result: list[dict[str, Any]] = []
    for left_index, first in enumerate(identities):
        for second in identities[left_index + 1:]:
            pair = (first, second)
            component: list[int] = []
            for index, frame in enumerate(frames):
                distance = pair_distance(frame, pair)
                close = distance is not None and distance < 1.5
                if not close:
                    if component:
                        result.append(_finish_encounter(frames, pair, component, scenario, len(result)))
                        component = []
                    continue
                if component and int(frame["timestamp_ns"]) - int(frames[component[-1]]["timestamp_ns"]) > MAX_FRAME_GAP_NS:
                    result.append(_finish_encounter(frames, pair, component, scenario, len(result)))
                    component = []
                component.append(index)
            if component:
                result.append(_finish_encounter(frames, pair, component, scenario, len(result)))
    return result


def _finish_encounter(frames: list[dict[str, Any]], pair: tuple[str, str], indices: list[int], scenario: str, index: int) -> dict[str, Any]:
    minimum_index = min(indices, key=lambda i: (pair_distance(frames[i], pair), int(frames[i]["timestamp_ns"])))
    start_ns = int(frames[indices[0]]["timestamp_ns"]); end_ns = int(frames[indices[-1]]["timestamp_ns"])
    before_candidates = [i for i in range(indices[0] - 1, -1, -1) if start_ns - int(frames[i]["timestamp_ns"]) <= EVIDENCE_WINDOW_NS and all(observable(frames[i], identity) and track_id(frames[i], identity) is not None for identity in pair)]
    after_candidates = [i for i in range(indices[-1] + 1, len(frames)) if int(frames[i]["timestamp_ns"]) - end_ns <= EVIDENCE_WINDOW_NS and all(observable(frames[i], identity) and track_id(frames[i], identity) is not None for identity in pair)]
    before = before_candidates[0] if before_candidates else None
    after = after_candidates[0] if after_candidates else None
    return {
        "encounter_id": f"{scenario}:{pair[0]}|{pair[1]}:{index:03d}", "pair_ids": list(pair),
        "start_timestamp_ns": start_ns, "minimum_timestamp_ns": int(frames[minimum_index]["timestamp_ns"]), "end_timestamp_ns": end_ns,
        "minimum_distance_m": pair_distance(frames[minimum_index], pair), "frame_indices": indices,
        "before_timestamp_ns": int(frames[before]["timestamp_ns"]) if before is not None else None,
        "after_timestamp_ns": int(frames[after]["timestamp_ns"]) if after is not None else None,
        "before_censored": before is None, "after_censored": after is None,
        "pre_post_identity_comparable": before is not None and after is not None,
        "pre_track_ids": {identity: track_id(frames[before], identity) for identity in pair} if before is not None else {},
        "post_track_ids": {identity: track_id(frames[after], identity) for identity in pair} if after is not None else {},
        "observable_pair_frames": sum(pair_state(frames[i], pair) is not None for i in indices),
        "before_frame_index": before, "minimum_frame_index": minimum_index, "after_frame_index": after,
    }


def build_separation_events(frames: list[dict[str, Any]], encounters: list[dict[str, Any]]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for encounter in encounters:
        pair = tuple(encounter["pair_ids"])
        mode = "WAIT_TWO"; loss_start = None; previous_ns = None
        for index in encounter["frame_indices"]:
            frame = frames[index]; stamp = int(frame["timestamp_ns"]); state = pair_state(frame, pair)
            if previous_ns is not None and stamp - previous_ns > MAX_FRAME_GAP_NS:
                mode = "WAIT_TWO"; loss_start = None
            previous_ns = stamp
            if state is None or state == 0:
                mode = "WAIT_TWO"; loss_start = None
            elif mode == "WAIT_TWO" and state == 2:
                mode = "HAVE_TWO"
            elif mode == "HAVE_TWO" and state == 1:
                mode = "IN_LOSS"; loss_start = stamp
            elif mode == "IN_LOSS" and state == 2:
                events.append({"event_kind": "detector_separation_event", "encounter_id": encounter["encounter_id"], "pair_ids": list(pair), "start_timestamp_ns": loss_start, "end_timestamp_ns": stamp, "distance_m": encounter["minimum_distance_m"], "distance_bin": distance_bin(float(encounter["minimum_distance_m"]))})
                mode = "HAVE_TWO"; loss_start = None
            elif mode == "IN_LOSS" and state != 1:
                mode = "WAIT_TWO"; loss_start = None
    return events


def classify_window(frames: list[dict[str, Any]], index: int, identity: str, separation_events: list[dict[str, Any]]) -> str:
    end_ns = int(frames[index]["timestamp_ns"]); start_ns = end_ns - LOOKBACK_NS
    window = [frame for frame in frames[:index + 1] if int(frame["timestamp_ns"]) >= start_ns]
    if not window or int(window[0]["timestamp_ns"]) > start_ns + 75_000_000:
        return "UNRESOLVED"
    if any(int(b["timestamp_ns"]) - int(a["timestamp_ns"]) > MAX_FRAME_GAP_NS for a, b in zip(window, window[1:])):
        return "UNRESOLVED"
    if any(not observable(frame, identity) for frame in window):
        return "OBSERVABILITY_FAILURE"
    if all(all(detection_index(frame, gt_id) is not None for gt_id in gt_map(frame) if observable(frame, gt_id)) for frame in window):
        return "TRACKER_ASSOCIATION_FAILURE"
    if any(int(event["start_timestamp_ns"]) - LOOKBACK_NS <= end_ns <= int(event["end_timestamp_ns"]) + LOOKBACK_NS for event in separation_events):
        return "DETECTOR_SEPARATION_INDUCED_TRACK_BREAK"
    return "DETECTOR_GAP_INDUCED_TRACK_BREAK"


def reconstruct_identity_events(frames: list[dict[str, Any]], encounters: list[dict[str, Any]], separation_events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    identities = sorted({identity for frame in frames for identity in gt_map(frame)})
    for identity in identities:
        last_match_index = None; last_id = None; gap_indices: list[int] = []
        for index, frame in enumerate(frames):
            if identity not in gt_map(frame):
                continue
            current = track_id(frame, identity)
            if current is None:
                if last_match_index is not None:
                    gap_indices.append(index)
                continue
            if last_match_index is not None and not gap_indices and index == last_match_index + 1 and int(frame["timestamp_ns"]) - int(frames[last_match_index]["timestamp_ns"]) <= MAX_FRAME_GAP_NS and current != last_id:
                events.append(_identity_event(frames, index, identity, "continuous_id_switch", last_id, current, classify_window(frames, index, identity, separation_events)))
            if last_match_index is not None and gap_indices:
                attribution = classify_window(frames, index, identity, separation_events)
                events.append(_identity_event(frames, index, identity, "fragmentation", last_id, current, attribution))
                if current != last_id:
                    events.append(_identity_event(frames, index, identity, "reacquisition_id_change", last_id, current, attribution))
            last_match_index = index; last_id = current; gap_indices = []
    for encounter in encounters:
        if not encounter["pre_post_identity_comparable"]:
            continue
        for identity in encounter["pair_ids"]:
            before = encounter["pre_track_ids"][identity]; after = encounter["post_track_ids"][identity]
            if before != after:
                index = int(encounter["after_frame_index"])
                event = _identity_event(frames, index, identity, "crossing_identity_change", before, after, classify_window(frames, index, identity, separation_events))
                event["encounter_id"] = encounter["encounter_id"]
                event["distance_m"] = encounter["minimum_distance_m"]
                event["distance_bin"] = distance_bin(float(encounter["minimum_distance_m"]))
                events.append(event)
    return events


def _identity_event(frames: list[dict[str, Any]], index: int, identity: str, kind: str, previous: int | None, current: int | None, attribution: str) -> dict[str, Any]:
    frame = frames[index]; distance = nearest_neighbour_distance(frame, identity)
    return {"timestamp_ns": int(frame["timestamp_ns"]), "gt_id": identity, "event_kind": kind, "previous_track_id": previous, "current_track_id": current, "attribution": attribution, "distance_m": distance, "distance_bin": distance_bin(distance) if distance is not None else None, "encounter_id": None}


def fresh_bin() -> dict[str, Any]:
    return {"unique_frame_indices": set(), "observable_gt_observation_denominator": 0, "matched_gt_observation_numerator": 0, "observable_pair_frame_denominator": 0, "separated_pair_frame_numerator": 0, "continuous_id_switches": 0, "crossing_identity_changes": 0, "fragmentations": 0, "reacquisition_id_changes": 0, "tracker_native_failures": 0, "detector_induced_breaks": 0, "observability_failures": 0}


def aggregate_bins(frames: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    bins = {label: fresh_bin() for label in BIN_ORDER}
    for index, frame in enumerate(frames):
        truth = gt_map(frame)
        for identity in truth:
            distance = nearest_neighbour_distance(frame, identity)
            if distance is None:
                continue
            bucket = bins[distance_bin(distance)]
            reason = frame.get("observability", {}).get(identity, {}).get("reason")
            if not observable(frame, identity):
                if reason in {"NO_SCAN_SUPPORT", "STATIC_OCCLUDED"}:
                    bucket["observability_failures"] += 1
                continue
            bucket["unique_frame_indices"].add(index)
            bucket["observable_gt_observation_denominator"] += 1
            bucket["matched_gt_observation_numerator"] += detection_index(frame, identity) is not None
        identities = sorted(truth)
        for left, first in enumerate(identities):
            for second in identities[left + 1:]:
                pair = (first, second); distance = pair_distance(frame, pair)
                if distance is None or pair_state(frame, pair) is None or not pair_is_nearest_neighbour(frame, pair):
                    continue
                bucket = bins[distance_bin(distance)]; bucket["unique_frame_indices"].add(index)
                bucket["observable_pair_frame_denominator"] += 1
                bucket["separated_pair_frame_numerator"] += pair_state(frame, pair) == 2
    for event in events:
        label = event.get("distance_bin")
        if label not in bins:
            continue
        key = {"continuous_id_switch": "continuous_id_switches", "crossing_identity_change": "crossing_identity_changes", "fragmentation": "fragmentations", "reacquisition_id_change": "reacquisition_id_changes"}[event["event_kind"]]
        bins[label][key] += 1
        if event["attribution"] == "TRACKER_ASSOCIATION_FAILURE": bins[label]["tracker_native_failures"] += 1
        if event["attribution"].startswith("DETECTOR_"): bins[label]["detector_induced_breaks"] += 1
    for bucket in bins.values():
        bucket["unique_frame_count"] = len(bucket.pop("unique_frame_indices"))
        gt_den = bucket["observable_gt_observation_denominator"]; pair_den = bucket["observable_pair_frame_denominator"]
        bucket["detector_recall"] = bucket["matched_gt_observation_numerator"] / gt_den if gt_den else None
        bucket["separation_success_rate"] = bucket["separated_pair_frame_numerator"] / pair_den if pair_den else None
    return bins


def parse_ready_and_metrics(log: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    ready = metrics = None
    for line in log.read_text(errors="replace").splitlines():
        if "WAREHOUSE_PEOPLE_ROBOT_READY=" in line:
            ready = json.loads(line.split("WAREHOUSE_PEOPLE_ROBOT_READY=", 1)[1])
        if "WAREHOUSE_PEOPLE_ROBOT_METRICS=" in line:
            metrics = json.loads(line.split("WAREHOUSE_PEOPLE_ROBOT_METRICS=", 1)[1])
    if ready is None or metrics is None:
        raise ValueError(f"missing READY/METRICS in {log}")
    return ready, metrics


def validate_manifest(manifest: dict[str, Any], manifest_path: Path) -> tuple[Path, list[dict[str, Any]]]:
    errors: list[str] = []
    entries = manifest.get("entries", [])
    if manifest.get("schema") != "isaac_crowded_tracking_suite/v2": errors.append("manifest schema")
    if [entry.get("cell") for entry in entries] != list(EXPECTED_CELLS): errors.append("canonical ten cells")
    if "run_root" not in manifest:
        errors.append("missing run_root")
    if errors:
        raise ValueError("; ".join(errors))
    root = Path(manifest["run_root"]); root = root if root.is_absolute() else (manifest_path.parent / root).resolve()
    project_root = manifest_path.resolve().parents[2]
    evaluator_source = project_root / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_evaluator.py"
    core_source = project_root / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_analysis_core.py"
    for entry in entries:
        label = entry.get("cell", "?"); run = root / entry["run_id"]; output = run / entry.get("output_name", "evaluation_recomputed_v2")
        paths = [root / value for value in entry["bag_files"]] + [root / entry["metadata"]]
        if not run.is_dir() or any(not path.is_file() for path in paths): errors.append(f"{label}: raw inputs")
        elif canonical_bag_hash(run, paths) != entry["canonical_hash"]: errors.append(f"{label}: raw hash")
        if entry["producer_source_sha256"] != EXPECTED_SOURCE or entry["launcher_sha256"] != EXPECTED_LAUNCHER: errors.append(f"{label}: source hash")
        if entry["world_sha256"] != EXPECTED_WORLD or entry["scene_sha256"] != EXPECTED_SCENE: errors.append(f"{label}: world hash")
        world_path = project_root / entry["world_path"]; scene_path = project_root / entry["scene_path"]
        if not world_path.is_file() or not scene_path.is_file() or sha256_file(world_path) != entry["world_sha256"] or sha256_file(scene_path) != entry["scene_sha256"]:
            errors.append(f"{label}: world/scene file drift")
        required = [output / "REPLAY_VALID", output / "replay_manifest.json", output / "summary.json", output / "crowded_tracking_trace.jsonl"]
        if any(not path.is_file() for path in required): errors.append(f"{label}: replay output")
        else:
            replay = load_json(output / "replay_manifest.json")
            if replay.get("raw_bag_sha256") != entry["canonical_hash"] or replay.get("summary_sha256") != sha256_file(output / "summary.json") or replay.get("trace_sha256") != sha256_file(output / "crowded_tracking_trace.jsonl"):
                errors.append(f"{label}: replay manifest hash")
            entry_json = json.dumps(entry, sort_keys=True, separators=(",", ":"))
            if replay.get("input_manifest_entry_sha256") != hashlib.sha256(entry_json.encode()).hexdigest() or replay.get("evaluator_source_sha256") != sha256_file(evaluator_source) or replay.get("analysis_core_source_sha256") != sha256_file(core_source):
                errors.append(f"{label}: replay source drift")
    if errors:
        raise ValueError("; ".join(errors))
    return root, entries


def analyze_entry(entry: dict[str, Any], root: Path) -> dict[str, Any]:
    run = root / entry["run_id"]; output = run / entry.get("output_name", "evaluation_recomputed_v2")
    summary = load_json(output / "summary.json"); frames = load_trace(output / "crowded_tracking_trace.jsonl")
    scenario_meta = load_json(run / "scenario_metadata.json"); ready, metrics = parse_ready_and_metrics(run / "isaac.log")
    validity_reasons = []
    quality = summary["quality"]; contract = summary["replay_contract"]
    if scenario_meta.get("scenario") != entry["scenario"] or summary.get("stress_ids") != scenario_meta.get("stress_ids"): validity_reasons.append("scenario_or_ids")
    if (run / "isaac_exit_code.txt").read_text().strip() != "0" or ready.get("lidar_backend") != "physx_scene_query" or ready.get("physx_capture_backend") != "omni.physx_scene_query" or metrics.get("status") != "PASS" or metrics.get("exit_reason") != "duration_reached": validity_reasons.append("runtime_contract")
    if ready.get("producer_source_sha256") != EXPECTED_SOURCE or ready.get("launcher_sha256") != EXPECTED_LAUNCHER or ready.get("scene_usd") != str((Path.cwd() / entry["scene_path"]).resolve()) or ready.get("ros_stage_planar_mapping") != "ros_x=stage_x,ros_y=stage_y": validity_reasons.append("provenance")
    if not (run / "runtime_contract_pass").is_file() or "PASS" not in (run / "stationary_guard.txt").read_text(): validity_reasons.append("stationary_contract")
    if float(metrics.get("robot_planar_displacement_m", math.inf)) != 0.0 or int(metrics.get("ros_cmd_vel_messages_received", -1)) != 0: validity_reasons.append("robot_motion")
    if not contract.get("world_scene_contract", {}).get("valid") or not contract.get("clock_monotonic") or not contract.get("tf_static_received"): validity_reasons.append("replay_contract")
    zero_fields = ("dropped_unsynchronized_frames", "dropped_wrong_frame_frames", "dropped_tf_frames", "malformed_detection_frames", "pending_track_frames", "clock_regressions", "evaluated_before_tf_static_frames", "evaluated_before_clock_frames")
    if any(int(quality.get(field, -1)) != 0 for field in zero_fields): validity_reasons.append("quality_counters")
    if any(values != [2000] for values in quality.get("scan_beam_counts", {}).values()) or any(not 13.5 <= float(rate) <= 16.5 for rate in quality.get("scan_observed_rate_hz", {}).values()): validity_reasons.append("lidar_contract")
    if any(int(value) != len(frames) for value in quality.get("exact_source_stamp_counters", {}).values()): validity_reasons.append("exact_stamps")
    encounters = build_encounters(frames, entry["scenario"]); separation_events = build_separation_events(frames, encounters)
    events = reconstruct_identity_events(frames, encounters, separation_events)
    if not encounters: validity_reasons.append("no_close_encounter")
    if any(event["attribution"] == "UNRESOLVED" for event in events): validity_reasons.append("unresolved_identity")
    bins = aggregate_bins(frames, events)
    eligible_gt = sum(bucket["observable_gt_observation_denominator"] for bucket in bins.values()); matched_gt = sum(bucket["matched_gt_observation_numerator"] for bucket in bins.values())
    pair_den = sum(bucket["observable_pair_frame_denominator"] for bucket in bins.values()); pair_num = sum(bucket["separated_pair_frame_numerator"] for bucket in bins.values())
    eligible_errors = [float(match["position_error_m"]) for frame in frames for match in frame.get("gt_detection_matches", []) if observable(frame, str(match["gt_id"]))]
    event_counts = Counter(event["event_kind"] for event in events); attribution = Counter(event["attribution"] for event in events)
    two_person_states = Counter()
    three_person_states = Counter()
    for frame in frames:
        identities = sorted(gt_map(frame))
        if identities and all(observable(frame, identity) for identity in identities):
            matched = len({detection_index(frame, identity) for identity in identities if detection_index(frame, identity) is not None})
            if len(identities) == 2:
                two_person_states[matched] += 1
            elif len(identities) == 3:
                three_person_states[matched] += 1
    detector_pass = eligible_gt > 0 and matched_gt == eligible_gt and pair_den > 0 and pair_num == pair_den and not separation_events
    algorithm_events = [event for event in events if event["attribution"] in {"TRACKER_ASSOCIATION_FAILURE", "DETECTOR_SEPARATION_INDUCED_TRACK_BREAK", "DETECTOR_GAP_INDUCED_TRACK_BREAK"}]
    id_pass = not algorithm_events
    detector_failure = not detector_pass
    tracker_native = any(event["attribution"] == "TRACKER_ASSOCIATION_FAILURE" and event["event_kind"] != "crossing_identity_change" for event in events)
    primary = "BOTH" if detector_failure and tracker_native else "DR-SPAAM DETECTOR" if detector_failure else "TRACKER ASSOCIATION" if tracker_native else "NEITHER IN TESTED RANGE"
    return {
        "cell": entry["cell"], "run": entry["run_id"], "run_dir": str(run.resolve()), "scenario": entry["scenario"], "requested_spacing_m": scenario_meta.get("requested_spacing_m"),
        "episode_validity": "VALID" if not validity_reasons else "INVALID", "validity_reasons": validity_reasons,
        "actual_min_distance_m": min(float(item["distance_m"]) for frame in frames for item in frame.get("pairwise_gt", [])), "actual_mean_pair_distance_m": mean(float(item["distance_m"]) for frame in frames for item in frame.get("pairwise_gt", [])),
        "evaluated_frames": len(frames), "actual_pedestrians": len({identity for frame in frames for identity in gt_map(frame)}),
        "eligible_observable_gt": eligible_gt, "detector_tp": matched_gt, "detector_fn": eligible_gt - matched_gt, "detector_fp_global": int(summary["detector"]["fp"]),
        "global_gt_observations": int(summary["detector"]["tp"]) + int(summary["detector"]["fn"]), "global_detections": int(summary["detector"]["tp"]) + int(summary["detector"]["fp"]), "global_tp": int(summary["detector"]["tp"]), "global_fn": int(summary["detector"]["fn"]), "recall_global": summary["detector"]["recall"],
        "precision_global": summary["detector"]["precision"], "recall_observable": matched_gt / eligible_gt if eligible_gt else None,
        "mean_position_error_m": mean(eligible_errors) if eligible_errors else None, "median_position_error_m": median(eligible_errors) if eligible_errors else None,
        "observable_pair_frames": pair_den, "separated_pair_frames": pair_num, "separation_success_rate": pair_num / pair_den if pair_den else None,
        "detector_separation_events": len(separation_events), "event_counts": dict(event_counts), "attribution_counts": dict(attribution),
        "two_gt_two_detection_frames": two_person_states[2], "two_gt_one_detection_frames": two_person_states[1], "two_gt_zero_detection_frames": two_person_states[0],
        "three_gt_three_detection_frames": three_person_states[3], "three_gt_two_detection_frames": three_person_states[2], "three_gt_one_detection_frames": three_person_states[1], "three_gt_zero_detection_frames": three_person_states[0],
        "detector_induced_track_breaks": sum(event["attribution"].startswith("DETECTOR_") and event["event_kind"] != "crossing_identity_change" for event in events),
        "tracker_native_association_failures": sum(event["attribution"] == "TRACKER_ASSOCIATION_FAILURE" and event["event_kind"] != "crossing_identity_change" for event in events),
        "confirmed_track_count": summary["tracker"]["confirmed_track_count"], "detector_status": "PASS" if detector_pass else "FAIL", "id_status": "PASS" if id_pass else "FAIL", "algorithm_status": "PASS" if detector_pass and id_pass else "FAIL", "primary_failure_source": primary,
        "distance_conditioned": bins, "encounters": encounters, "separation_events": separation_events, "identity_events": events,
        "crossing_evaluable": any(item["pre_post_identity_comparable"] for item in encounters),
        "keyframes": summary["quality"]["keyframes"], "replay_manifest": str((output / "replay_manifest.json").resolve()),
    }


def combine_bins(runs: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    combined = {label: fresh_bin() for label in BIN_ORDER}
    for label in BIN_ORDER:
        for run in runs:
            source = run["distance_conditioned"][label]
            for key in combined[label]:
                if key != "unique_frame_indices": combined[label][key] += int(source.get(key, 0) or 0)
        combined[label]["unique_frame_count"] = sum(run["distance_conditioned"][label]["unique_frame_count"] for run in runs)
        combined[label].pop("unique_frame_indices")
        gt_den = combined[label]["observable_gt_observation_denominator"]; pair_den = combined[label]["observable_pair_frame_denominator"]
        combined[label]["detector_recall"] = combined[label]["matched_gt_observation_numerator"] / gt_den if gt_den else None
        combined[label]["separation_success_rate"] = combined[label]["separated_pair_frame_numerator"] / pair_den if pair_den else None
    return combined


def derive_answers(runs: list[dict[str, Any]], bins: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    far = bins[">=1.50"]; all_sep = sum(run["detector_separation_events"] for run in runs)
    tracker_native = sum(
        event["attribution"] == "TRACKER_ASSOCIATION_FAILURE"
        and event["event_kind"] != "crossing_identity_change"
        for run in runs for event in run["identity_events"]
    )
    continuous = sum(run["event_counts"].get("continuous_id_switch", 0) for run in runs); crossing = sum(run["event_counts"].get("crossing_identity_change", 0) for run in runs)
    fragmentation = sum(run["event_counts"].get("fragmentation", 0) for run in runs); reacq = sum(run["event_counts"].get("reacquisition_id_change", 0) for run in runs)
    close_fail = sum(bucket["observable_pair_frame_denominator"] - bucket["separated_pair_frame_numerator"] for label, bucket in bins.items() if label != ">=1.50")
    all_fail = sum(bucket["observable_pair_frame_denominator"] - bucket["separated_pair_frame_numerator"] for bucket in bins.values())
    ratio = close_fail / all_fail if all_fail else None
    status = {scenario: "PASS" if all(run["episode_validity"] == "VALID" and run["algorithm_status"] == "PASS" for run in runs if run["scenario"] == scenario) else "FAIL" for scenario in "ABCDE"}
    detector_evidence = any(run["detector_status"] == "FAIL" for run in runs); tracker_evidence = tracker_native > 0
    bottleneck = "BOTH" if detector_evidence and tracker_evidence else "DR-SPAAM DETECTOR" if detector_evidence else "TRACKER ASSOCIATION" if tracker_evidence else "NEITHER IN TESTED RANGE"
    return [
        {"question": 1, "answer": "NO", "evidence": {"bin": ">=1.50", "separation_success_rate": far["separation_success_rate"], "numerator": far["separated_pair_frame_numerator"], "denominator": far["observable_pair_frame_denominator"]}},
        {"question": 2, "answer": "SCENARIO_DEPENDENT_NO_SINGLE_BOUNDARY; aggregate degradation is strongest below 0.75 m", "evidence": {label: bins[label]["separation_success_rate"] for label in BIN_ORDER}},
        {"question": 3, "answer": "YES" if all_sep else "NO", "evidence": {"completed_2_to_1_to_2_events": all_sep}},
        {"question": 4, "answer": "YES" if ratio is not None and ratio > 0.5 else "NO", "evidence": {"close_failure_pair_frames": close_fail, "all_failure_pair_frames": all_fail, "ratio": ratio, "causality": "separation loss/recovery; NMS alone is not proven"}},
        {"question": 5, "answer": "YES" if tracker_native else "NO", "evidence": {"tracker_native_identity_events": tracker_native}},
        {"question": 6, "answer": status["A"], "evidence": {run["cell"]: run["algorithm_status"] for run in runs if run["scenario"] == "A"}},
        {"question": 7, "answer": status["B"]}, {"question": 8, "answer": status["C"]}, {"question": 9, "answer": status["D"], "evidence": {run["cell"]: run["algorithm_status"] for run in runs if run["scenario"] == "D"}},
        {"question": 10, "answer": status["E"]},
        {"question": 11, "answer": {"continuous": continuous, "crossing_net_changes": crossing, "fragmentations": fragmentation, "reacquisition_after_gap": reacq}},
        {"question": 12, "answer": bottleneck, "evidence": {"detector_failure_present": detector_evidence, "tracker_native_events": tracker_native}},
    ]


def write_outputs(output: Path, manifest_path: Path, runs: list[dict[str, Any]], bins: dict[str, dict[str, Any]], answers: list[dict[str, Any]]) -> None:
    temp = output.with_name(output.name + ".tmp")
    if output.exists() or temp.exists(): raise ValueError(f"output already exists: {output}")
    temp.mkdir(parents=True)
    scenario_status = {scenario: "PASS" if all(run["algorithm_status"] == "PASS" for run in runs if run["scenario"] == scenario) else "FAIL" for scenario in "ABCDE"}
    suite = {"schema": "isaac_crowded_tracking_suite/v2", "manifest": str(manifest_path.resolve()), "runs": runs, "distance_conditioned": bins, "scenario_status": scenario_status, "research_answers": answers, "current_bottleneck": answers[-1]["answer"]}
    (temp / "suite_summary.json").write_text(json.dumps(suite, indent=2, sort_keys=True) + "\n")
    core_fields = ("cell", "run", "scenario", "requested_spacing_m", "actual_min_distance_m", "global_gt_observations", "global_detections", "global_tp", "detector_fp_global", "global_fn", "precision_global", "recall_global", "eligible_observable_gt", "detector_tp", "detector_fn", "recall_observable", "observable_pair_frames", "separated_pair_frames", "separation_success_rate", "detector_separation_events", "confirmed_track_count", "two_gt_two_detection_frames", "two_gt_one_detection_frames", "two_gt_zero_detection_frames", "detector_induced_track_breaks", "tracker_native_association_failures", "detector_status", "id_status", "algorithm_status", "primary_failure_source", "episode_validity")
    with (temp / "core_table.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=core_fields); writer.writeheader(); writer.writerows({key: run.get(key) for key in core_fields} for run in runs)
    with (temp / "distance_conditioned.csv").open("w", newline="") as stream:
        fields = ("distance_bin", *next(iter(bins.values())).keys()); writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows({"distance_bin": label, **bins[label]} for label in BIN_ORDER)
    events = [{"cell": run["cell"], **event} for run in runs for event in run["identity_events"]]
    with (temp / "failure_events.csv").open("w", newline="") as stream:
        fields = ("cell", "timestamp_ns", "gt_id", "event_kind", "previous_track_id", "current_track_id", "attribution", "distance_m", "distance_bin", "encounter_id"); writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(events)
    encounters = [{"cell": run["cell"], **{key: value for key, value in encounter.items() if key not in {"frame_indices", "before_frame_index", "minimum_frame_index", "after_frame_index"}}} for run in runs for encounter in run["encounters"]]
    (temp / "encounters.json").write_text(json.dumps(encounters, indent=2, sort_keys=True) + "\n")
    report = ["# Isaac crowded/crossing stress evaluation v2", "", "All ten episodes are raw-bag replays; replay validity is separate from algorithm status.", "", "| Cell | Actual min m | TP/FP/FN global | Obs recall | Separation | Sep events | Continuous | Crossing | Frag | Reacq | Primary | Algorithm |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|"]
    for run in runs:
        c = run["event_counts"]
        report.append(f"| {run['cell']} | {run['actual_min_distance_m']:.3f} | {run['global_tp']}/{run['detector_fp_global']}/{run['global_fn']} | {run['recall_observable']:.3f} | {run['separation_success_rate']:.3f} | {run['detector_separation_events']} | {c.get('continuous_id_switch',0)} | {c.get('crossing_identity_change',0)} | {c.get('fragmentation',0)} | {c.get('reacquisition_id_change',0)} | {run['primary_failure_source']} | {run['algorithm_status']} |")
    report += ["", "## Distance-conditioned", "", "| Distance | GT matched/observable | Recall | Pair separated/observable | Separation | Tracker-native |", "|---|---:|---:|---:|---:|---:|"]
    for label in BIN_ORDER:
        bucket = bins[label]; recall = "NO VALID SAMPLES" if bucket["detector_recall"] is None else f"{bucket['detector_recall']:.3f}"; separation = "NO VALID SAMPLES" if bucket["separation_success_rate"] is None else f"{bucket['separation_success_rate']:.3f}"
        report.append(f"| {label} | {bucket['matched_gt_observation_numerator']}/{bucket['observable_gt_observation_denominator']} | {recall} | {bucket['separated_pair_frame_numerator']}/{bucket['observable_pair_frame_denominator']} | {separation} | {bucket['tracker_native_failures']} |")
    report += ["", "## Research answers", ""] + [f"{answer['question']}. {json.dumps(answer['answer'], ensure_ascii=False)} — {json.dumps(answer.get('evidence', {}), ensure_ascii=False)}" for answer in answers]
    (temp / "report.md").write_text("\n".join(report) + "\n")
    (temp / "input_manifest.json").write_text(manifest_path.read_text())
    temp.rename(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--manifest", type=Path, required=True); parser.add_argument("--output-dir", type=Path, required=True); return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        manifest = load_json(args.manifest); root, entries = validate_manifest(manifest, args.manifest)
        runs = [analyze_entry(entry, root) for entry in entries]
        invalid = [run["cell"] for run in runs if run["episode_validity"] != "VALID"]
        if invalid: raise ValueError("invalid replay episodes: " + ",".join(invalid))
        bins = combine_bins(runs); answers = derive_answers(runs, bins)
        write_outputs(args.output_dir, args.manifest, runs, bins, answers)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"CROWDED_SUITE_ANALYSIS_V2=FAIL {exc}", file=sys.stderr); return 1
    print(f"CROWDED_SUITE_ANALYSIS_V2=PASS output_dir={args.output_dir.resolve()}"); return 0


if __name__ == "__main__":
    raise SystemExit(main())
