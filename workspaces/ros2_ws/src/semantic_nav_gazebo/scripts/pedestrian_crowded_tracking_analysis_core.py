#!/usr/bin/env python3
"""Pure, offline analysis for the crowded pedestrian tracking contract.

The module deliberately contains no ROS imports.  It consumes the JSON frame
records produced by the evaluator and is consequently also usable by replay
and unit-test tooling.  Ground truth is never used by perception or control;
all functions here are post-hoc analysis functions.
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


DISTANCE_BINS = (
    (">=1.50", 1.50, math.inf),
    ("1.00-1.50", 1.00, 1.50),
    ("0.75-1.00", 0.75, 1.00),
    ("0.50-0.75", 0.50, 0.75),
    ("<0.50", -math.inf, 0.50),
)


@dataclass(frozen=True)
class AnalysisParameters:
    roi_radius_m: float = 8.0
    match_threshold_m: float = 0.5
    close_distance_m: float = 1.5
    continuity_lookback_sec: float = 0.5
    component_gap_sec: float = 0.25
    evidence_window_sec: float = 1.0
    scan_support_radius_m: float = 0.5
    scan_rate_hz: float = 15.0
    scan_rate_tolerance_hz: float = 1.5
    epsilon: float = 1.0e-9

    @property
    def roi_m(self) -> float:
        return self.roi_radius_m

    @property
    def roi_radius(self) -> float:
        return self.roi_radius_m

    @property
    def match_threshold(self) -> float:
        return self.match_threshold_m

    @property
    def close_distance(self) -> float:
        return self.close_distance_m

    @property
    def acceptable_scan_rate_hz(self) -> tuple[float, float]:
        return (self.scan_rate_hz - self.scan_rate_tolerance_hz,
                self.scan_rate_hz + self.scan_rate_tolerance_hz)


@dataclass
class FrameEvidence:
    timestamp_ns: int
    ground_truth: list[dict] = field(default_factory=list)
    detections: list[dict] = field(default_factory=list)
    tracks: list[dict] = field(default_factory=list)
    gt_detection_matches: list[dict] = field(default_factory=list)
    gt_track_matches: list[dict] = field(default_factory=list)
    scan_support: dict[str, dict] = field(default_factory=dict)
    observability: dict[str, dict] = field(default_factory=dict)
    source_stamps: dict[str, int | None] = field(default_factory=dict)
    sensor_origins: dict[str, tuple[float, float, float] | None] = field(default_factory=dict)
    pairwise_gt: list[dict] = field(default_factory=list)
    id_events: list[dict] = field(default_factory=list)
    raw: dict = field(default_factory=dict)

    @property
    def stamp_ns(self) -> int:
        return self.timestamp_ns


@dataclass
class ObservabilityEvidence:
    gt_id: str
    in_roi: bool
    scan_support: bool
    line_of_sight: bool | None
    reason: str | None = None
    observable: bool = False
    unknown: bool = False

    def __post_init__(self) -> None:
        if self.line_of_sight is None:
            self.unknown = True
        self.observable = bool(self.in_roi and self.scan_support and self.line_of_sight is True)
        if self.unknown:
            self.reason = "TF_OR_WORLD_UNKNOWN"
        elif self.observable:
            self.reason = None
        elif self.reason is None:
            self.reason = (
                "OUT_OF_ROI" if not self.in_roi else
                "NO_SCAN_SUPPORT" if not self.scan_support else
                "STATIC_OCCLUDED"
            )


@dataclass
class EncounterEvidence:
    pair: tuple[str, str]
    start_ns: int
    minimum_ns: int
    end_ns: int
    minimum_distance_m: float
    comparable: bool
    censored: bool
    frames: list[int] = field(default_factory=list)
    before_timestamp_ns: int | None = None
    after_timestamp_ns: int | None = None
    evidence: dict[str, Any] = field(default_factory=dict)


@dataclass
class IdentityEvent:
    timestamp_ns: int
    gt_id: str
    event: str
    classification: str
    previous_track_id: int | str | None = None
    current_track_id: int | str | None = None
    reason: str | None = None
    encounter_pair: tuple[str, str] | None = None


@dataclass
class RunAnalysis:
    frames: list[FrameEvidence]
    encounters: list[EncounterEvidence] = field(default_factory=list)
    identity_events: list[IdentityEvent] = field(default_factory=list)
    separation_events: list[dict] = field(default_factory=list)
    distance_conditioned: dict = field(default_factory=dict)
    validity: dict = field(default_factory=dict)
    gates: dict = field(default_factory=dict)
    answers: dict = field(default_factory=dict)
    bottleneck: dict = field(default_factory=dict)


def _value(record: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in record:
            return record[name]
    return default


def _stamp(frame: Mapping[str, Any]) -> int:
    value = _value(frame, "timestamp_ns", "stamp_ns", "timestamp")
    if isinstance(value, Mapping):
        return int(value.get("sec", 0)) * 1_000_000_000 + int(value.get("nanosec", 0))
    return int(value)


def _gt_id(gt: Mapping[str, Any]) -> str:
    return str(_value(gt, "id", "gt_id", "identity"))


def _xy(item: Mapping[str, Any] | Sequence[float]) -> tuple[float, float]:
    if not isinstance(item, Mapping):
        return float(item[0]), float(item[1])
    pose = item.get("pose")
    if isinstance(pose, Mapping):
        position = pose.get("position", pose)
        return float(position.get("x", 0.0)), float(position.get("y", 0.0))
    return float(item.get("x", 0.0)), float(item.get("y", 0.0))


def _pair_distance(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    x1, y1 = _xy(first)
    x2, y2 = _xy(second)
    return math.hypot(x1 - x2, y1 - y2)


def distance_bin(distance: float) -> str:
    if not math.isfinite(distance):
        raise ValueError(f"non-finite distance: {distance}")
    for label, low, high in DISTANCE_BINS:
        if low <= distance < high:
            return label
    raise ValueError(f"distance outside bins: {distance}")


def normalize_frames(frames: Iterable[Mapping[str, Any]], params: AnalysisParameters | None = None) -> list[FrameEvidence]:
    """Normalize v1/v2 JSON records and enforce strictly increasing stamps."""
    params = params or AnalysisParameters()
    result: list[FrameEvidence] = []
    previous = None
    for raw in frames:
        timestamp_ns = _stamp(raw)
        if previous is not None and timestamp_ns <= previous:
            raise ValueError("trace timestamps must be strict and unique")
        previous = timestamp_ns
        gt = [dict(item) for item in raw.get("ground_truth", raw.get("gt", []))]
        detections = [dict(item) for item in raw.get("detections", [])]
        tracks = [dict(item) for item in raw.get("tracks", [])]
        pairs = [dict(item) for item in raw.get("pairwise_gt", [])]
        if not pairs:
            for i, first in enumerate(gt):
                for second in gt[i + 1:]:
                    pairs.append({"ids": sorted((_gt_id(first), _gt_id(second))),
                                  "distance_m": _pair_distance(first, second)})
        source = dict(raw.get("source_stamps", raw.get("exact_source_stamps", {})))
        source.setdefault("tracks", timestamp_ns)
        source.setdefault("detections", timestamp_ns)
        source.setdefault("merged", _value(raw, "scan_merged_stamp_ns", default=timestamp_ns))
        result.append(FrameEvidence(
            timestamp_ns=timestamp_ns,
            ground_truth=gt,
            detections=detections,
            tracks=tracks,
            gt_detection_matches=[dict(x) for x in raw.get("gt_detection_matches", [])],
            gt_track_matches=[dict(x) for x in raw.get("gt_track_matches", [])],
            scan_support={str(k): dict(v) if isinstance(v, Mapping) else {"supported": bool(v)}
                          for k, v in raw.get("scan_support", {}).items()},
            observability={str(k): dict(v) if isinstance(v, Mapping) else {"observable": bool(v)}
                           for k, v in raw.get("observability", {}).items()},
            source_stamps={str(k): (None if v is None else int(v)) for k, v in source.items()},
            sensor_origins={str(k): tuple(v) if v is not None else None
                            for k, v in raw.get("sensor_origins", {}).items()},
            pairwise_gt=pairs,
            id_events=[dict(x) for x in raw.get("id_events", [])],
            raw=dict(raw),
        ))
    return result


def assign_scan_support(
    frame_or_points: Mapping[str, Any] | Sequence[Mapping[str, Any]],
    gt_points: Sequence[Mapping[str, Any]] | None = None,
    radius_m: float = 0.5,
    params: AnalysisParameters | None = None,
) -> dict[str, dict]:
    """Assign each merged scan return to at most one nearest GT (ID tie-break).

    The function accepts either a frame record plus optional explicit GT list,
    or a sequence of scan points and explicit GT points.  It intentionally does
    not use detector/tracker matches.
    """
    if isinstance(frame_or_points, Mapping):
        frame = frame_or_points
        points = frame.get("scan_points", frame.get("merged_scan", []))
        truths = list(gt_points if gt_points is not None else frame.get("ground_truth", []))
        radius_m = (params or AnalysisParameters()).scan_support_radius_m if radius_m == 0.5 else radius_m
    else:
        points = frame_or_points
        truths = list(gt_points or [])
    result = {_gt_id(gt): {
        "supported": False,
        "support_point_count": 0,
        "nearest_distance_m": None,
        "point_index": None,
        "point_indices": [],
    }
              for gt in truths}
    candidates: dict[int, list[tuple[float, str]]] = {}
    for index, point in enumerate(points):
        px, py = _xy(point)
        for gt in truths:
            gx, gy = _xy(gt)
            distance = math.hypot(px - gx, py - gy)
            if distance <= radius_m:
                candidates.setdefault(index, []).append((distance, _gt_id(gt)))
    # Each return supports its nearest GT only. A GT may own many returns.
    for index, point_candidates in sorted(candidates.items()):
        distance, identity = min(point_candidates, key=lambda item: (item[0], item[1]))
        record = result[identity]
        record["supported"] = True
        record["support_point_count"] += 1
        record["point_indices"].append(index)
        if record["point_index"] is None:
            record["point_index"] = index
        nearest = record["nearest_distance_m"]
        record["nearest_distance_m"] = distance if nearest is None else min(nearest, distance)
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


WORLD_SHA256 = "040c58fc7064c5823379edbde7478648d3cec88ea010e5e1a47e067b36f8ef5b"
SCENE_SHA256 = "18e012a8d1b9614aefa1517bc3be3ac47775e62cd6fba78240a04b4b6652c1dd"


def validate_world_scene_contract(world: str | Path, scene: str | Path, expected_boxes: int = 79) -> dict:
    """Audit provenance and planar scene metadata; unknown geometry fails closed."""
    world_path, scene_path = Path(world), Path(scene)
    result = {"status": "VALID", "valid": True, "reason": None, "world_sha256": None,
              "scene_sha256": None, "migrated_static_boxes": None, "skipped_includes": None}
    try:
        world_hash = _sha256(world_path)
        scene_hash = _sha256(scene_path)
        text = scene_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        result.update(status="TF_OR_WORLD_UNKNOWN", valid=False, reason=str(exc))
        return result
    result.update(world_sha256=world_hash, scene_sha256=scene_hash)
    migrated = re.search(r"int migratedStaticBoxes\s*=\s*(\d+)", text)
    skipped = re.search(r"int skippedGazeboIncludes\s*=\s*(\d+)", text)
    checks = [
        world_hash == WORLD_SHA256,
        scene_hash == SCENE_SHA256,
        'string sourceWorld = "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"' in text,
        'string sourceWorldSha256 = "' + WORLD_SHA256 + '"' in text,
        migrated is not None and int(migrated.group(1)) == expected_boxes,
        skipped is not None,
        re.search(r"metersPerUnit\s*=\s*1(?:\D|$)", text) is not None,
        re.search(r'upAxis\s*=\s*"Z"', text) is not None,
        'generatedBy = "isaac_sim/scripts/convert_gazebo_boxes_to_usda.py"' in text,
    ]
    mesh_names = re.findall(r'def\s+Mesh\s+"([^"]+)"', text)
    unknown_tokens = ("model://", "sourceWorld = \"\"")
    if not all(checks) or any(token in text for token in unknown_tokens) or any(name != "Ground" for name in mesh_names):
        result.update(status="TF_OR_WORLD_UNKNOWN", valid=False, reason="world/scene provenance or geometry contract mismatch")
    result["migrated_static_boxes"] = int(migrated.group(1)) if migrated else None
    result["skipped_includes"] = int(skipped.group(1)) if skipped else None
    result["converter_check_expected_boxes"] = expected_boxes
    return result


def _box_values(box: Mapping[str, Any]) -> tuple[float, float, float, float, float, float, float]:
    pose = box.get("pose", box)
    if "position" in pose:
        pose = {**pose, **pose["position"]}
    x, y, z = (float(pose.get(k, 0.0)) for k in ("x", "y", "z"))
    yaw = float(pose.get("yaw", pose.get("yaw_rad", 0.0)))
    size = box.get("size", box.get("dimensions", (0.0, 0.0, 0.0)))
    sx, sy, sz = (float(v) for v in size)
    return x, y, z, yaw, sx, sy, sz


def _ray_box_intersection(origin: tuple[float, float], target: tuple[float, float], box: Mapping[str, Any], epsilon: float) -> bool:
    x, y, _z, yaw, sx, sy, _sz = _box_values(box)
    direction = (target[0] - origin[0], target[1] - origin[1])
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = c * (origin[0] - x) + s * (origin[1] - y), -s * (origin[0] - x) + c * (origin[1] - y)
    dx, dy = c * direction[0] + s * direction[1], -s * direction[0] + c * direction[1]
    t0, t1 = 0.0, 1.0
    for coordinate, delta, half in ((ox, dx, sx * 0.5), (oy, dy, sy * 0.5)):
        if abs(delta) <= epsilon:
            if coordinate < -half - epsilon or coordinate > half + epsilon:
                return False
            continue
        near, far = (-half - coordinate) / delta, (half - coordinate) / delta
        if near > far:
            near, far = far, near
        t0, t1 = max(t0, near), min(t1, far)
        if t0 > t1 + epsilon:
            return False
    # Endpoint contact is not an occlusion; all other tangent/edge/corner
    # contacts are blocked.  t=0 is a valid blocked ray origin.
    return t0 <= 1.0 - epsilon and t1 >= -epsilon


def compute_static_line_of_sight(
    sensor_origins: Mapping[str, Sequence[float]] | Sequence[Sequence[float]],
    target: Sequence[float],
    boxes: Sequence[Mapping[str, Any]],
    *,
    epsilon: float = 1.0e-9,
) -> bool:
    """Return clear when either exact sensor-origin ray reaches the GT point."""
    origins = list(sensor_origins.values()) if isinstance(sensor_origins, Mapping) else list(sensor_origins)
    heights = [float(origin[2]) for origin in origins if origin is not None and len(origin) > 2]
    if not heights or max(heights) - min(heights) > 1.0e-6:
        return False
    beam_z = heights[0]
    target_xyz = (float(target[0]), float(target[1]), beam_z)
    for origin_value in origins:
        if origin_value is None:
            continue
        origin = (float(origin_value[0]), float(origin_value[1]), float(origin_value[2]) if len(origin_value) > 2 else target_xyz[2])
        blocked = False
        for box in boxes:
            x, y, z, yaw, sx, sy, sz = _box_values(box)
            zmin, zmax = z - sz * 0.5, z + sz * 0.5
            if zmin - epsilon <= beam_z <= zmax + epsilon:
                if _ray_box_intersection(origin[:2], target_xyz[:2], box, epsilon):
                    blocked = True
                    break
        if not blocked:
            return True
    return False


def _truth_map(frame: FrameEvidence) -> dict[str, dict]:
    return {_gt_id(gt): gt for gt in frame.ground_truth}


def _pair_from_frame(frame: FrameEvidence, pair: tuple[str, str]) -> float | None:
    values = set(pair)
    for item in frame.pairwise_gt:
        ids = tuple(sorted(str(x) for x in item.get("ids", item.get("pair", []))))
        if set(ids) == values:
            value = _value(item, "distance_m", "distance")
            return None if value is None else float(value)
    truths = _truth_map(frame)
    if values <= truths.keys():
        return _pair_distance(truths[pair[0]], truths[pair[1]])
    return None


def _pair_observable(frame: FrameEvidence, pair: tuple[str, str]) -> bool:
    return all(bool(frame.observability.get(identity, {}).get("observable", False)) for identity in pair)


def _pair_confirmed(frame: FrameEvidence, pair: tuple[str, str]) -> bool:
    matches = {_gt_id(item): item for item in frame.gt_track_matches}
    tracks = {str(_value(item, "track_id", "id")): item for item in frame.tracks}
    for identity in pair:
        match = matches.get(identity)
        if match is None:
            return False
        track = tracks.get(str(_value(match, "track_id")))
        if track is not None and str(track.get("state", "CONFIRMED")).upper() not in {"CONFIRMED", "COASTING"}:
            return False
    return True


def build_connected_encounters(frames: Sequence[FrameEvidence] | Sequence[Mapping[str, Any]], params: AnalysisParameters | None = None) -> list[EncounterEvidence]:
    params = params or AnalysisParameters()
    normalized = list(frames) if (not frames or isinstance(frames[0], FrameEvidence)) else normalize_frames(frames, params)
    pairs = set()
    for frame in normalized:
        identities = sorted(_truth_map(frame))
        pairs.update((first, second) for i, first in enumerate(identities) for second in identities[i + 1:])
    encounters = []
    for pair in sorted(pairs):
        components: list[list[tuple[int, FrameEvidence, float]]] = []
        current: list[tuple[int, FrameEvidence, float]] = []
        previous_close_ns: int | None = None
        intervening_nonclose = False
        for index, frame in enumerate(normalized):
            distance = _pair_from_frame(frame, pair)
            is_close = distance is not None and distance < params.close_distance_m
            if not is_close:
                if current:
                    intervening_nonclose = True
                continue
            item = (index, frame, float(distance))
            if (current and (intervening_nonclose or previous_close_ns is None or
                             frame.timestamp_ns - previous_close_ns > params.component_gap_sec * 1e9)):
                components.append(current)
                current = []
            current.append(item)
            previous_close_ns = frame.timestamp_ns
            intervening_nonclose = False
        if current:
            components.append(current)
        for component in components:
            minimum = min(component, key=lambda x: (x[2], x[1].timestamp_ns))
            boundary_ns = int(params.evidence_window_sec * 1e9)
            before = [f for f in normalized if f.timestamp_ns < component[0][1].timestamp_ns and component[0][1].timestamp_ns - f.timestamp_ns <= boundary_ns]
            after = [f for f in normalized if f.timestamp_ns > component[-1][1].timestamp_ns and f.timestamp_ns - component[-1][1].timestamp_ns <= boundary_ns]
            before = [f for f in before if _pair_observable(f, pair) and _pair_confirmed(f, pair)]
            after = [f for f in after if _pair_observable(f, pair) and _pair_confirmed(f, pair)]
            before_frame = max(before, key=lambda f: f.timestamp_ns, default=None)
            after_frame = min(after, key=lambda f: f.timestamp_ns, default=None)
            comparable = before_frame is not None and after_frame is not None
            encounters.append(EncounterEvidence(
                pair=pair, start_ns=component[0][1].timestamp_ns, end_ns=component[-1][1].timestamp_ns,
                minimum_ns=minimum[1].timestamp_ns, minimum_distance_m=minimum[2], comparable=comparable,
                censored=not comparable, frames=[f.timestamp_ns for _, f, _ in component],
                before_timestamp_ns=before_frame.timestamp_ns if before_frame else None,
                after_timestamp_ns=after_frame.timestamp_ns if after_frame else None,
                evidence={"before": before_frame.raw if before_frame else None, "after": after_frame.raw if after_frame else None},
            ))
    return encounters


def build_detector_separation_events(frames: Sequence[FrameEvidence] | Sequence[Mapping[str, Any]], params: AnalysisParameters | None = None) -> list[dict]:
    params = params or AnalysisParameters()
    normalized = list(frames) if (not frames or isinstance(frames[0], FrameEvidence)) else normalize_frames(frames, params)
    events = []
    active: dict[tuple[str, str], dict] = {}
    for frame in normalized:
        component = frame.raw.get("encounter_component")
        for pair_item in frame.pairwise_gt:
            pair = tuple(sorted(str(x) for x in pair_item.get("ids", pair_item.get("pair", []))))
            if len(pair) != 2 or not _pair_observable(frame, pair) or not _pair_confirmed(frame, pair):
                continue
            state = int(pair_item.get("detector_state", pair_item.get("state", 2 if len(frame.gt_detection_matches) >= len(frame.ground_truth) else 1)))
            current = active.setdefault(pair, {"state": 1, "component": component, "last_ns": frame.timestamp_ns, "first_ns": None})
            if current["component"] is not None and component is not None and current["component"] != component:
                active.pop(pair, None)
                continue
            if frame.timestamp_ns - current["last_ns"] > params.component_gap_sec * 1e9:
                current["state"] = 1
            if state == 2 and current["state"] == 1 and current["first_ns"] is not None:
                events.append({"event": "separation_event", "pair": pair, "timestamp_ns": frame.timestamp_ns, "component": component})
            if state == 1 and current["first_ns"] is None:
                current["first_ns"] = frame.timestamp_ns
            current["state"] = state
            current["last_ns"] = frame.timestamp_ns
    return events


def _match_for(frame: FrameEvidence, identity: str) -> dict | None:
    return next((m for m in frame.gt_track_matches if str(_value(m, "gt_id", "id")) == identity), None)


def reconstruct_identity_events(frames: Sequence[FrameEvidence] | Sequence[Mapping[str, Any]], params: AnalysisParameters | None = None, separation_events: Sequence[Mapping[str, Any]] | None = None) -> list[IdentityEvent]:
    params = params or AnalysisParameters()
    normalized = list(frames) if (not frames or isinstance(frames[0], FrameEvidence)) else normalize_frames(frames, params)
    all_ids = sorted({identity for frame in normalized for identity in _truth_map(frame)})
    separations = {(tuple(item.get("pair", ())), int(item.get("timestamp_ns", -1))) for item in (separation_events or build_detector_separation_events(normalized, params))}
    events: list[IdentityEvent] = []
    for identity in all_ids:
        previous: tuple[int, Any] | None = None
        had_match = False
        gap_start = None
        for frame in normalized:
            observable = bool(frame.observability.get(identity, {}).get("observable", False))
            match = _match_for(frame, identity)
            track_id = _value(match or {}, "track_id")
            if observable and match is not None:
                if previous is not None and track_id != previous[1]:
                    pair = next((tuple(sorted((identity, x))) for x in all_ids if x != identity and tuple(sorted((identity, x))) in {p for p, _ in separations}), None)
                    classification = "DETECTOR_SEPARATION_INDUCED" if pair and any(p == pair and abs(ts - frame.timestamp_ns) <= params.evidence_window_sec * 1e9 for p, ts in separations) else "TRACKER"
                    close_now = any(_pair_from_frame(frame, tuple(sorted((identity, other)))) is not None and _pair_from_frame(frame, tuple(sorted((identity, other)))) < params.close_distance_m for other in all_ids if other != identity)
                    events.append(IdentityEvent(frame.timestamp_ns, identity, "crossing_id_switch" if close_now else "continuous_id_switch", classification, previous[1], track_id))
                elif gap_start is not None:
                    classification = "TRACKER" if had_match and _independent_detection(frame, identity) else "DETECTOR_GAP"
                    events.append(IdentityEvent(frame.timestamp_ns, identity, "fragmentation" if previous is not None and track_id == previous[1] else "reacquisition_id_change", classification, previous[1] if previous else None, track_id))
                previous = (frame.timestamp_ns, track_id)
                had_match = True
                gap_start = None
                continue
            if had_match:
                if not observable:
                    reason = frame.observability.get(identity, {}).get("reason", "TF_OR_WORLD_UNKNOWN")
                    if reason == "TF_OR_WORLD_UNKNOWN":
                        events.append(IdentityEvent(frame.timestamp_ns, identity, "gap", "OBSERVABILITY", previous[1] if previous else None, None, reason))
                    gap_start = frame.timestamp_ns
                elif gap_start is None:
                    gap_start = frame.timestamp_ns
        # A gap with no later matched sample is censored and intentionally has
        # no tracker event; this keeps endpoint censoring separate from errors.
    return events


def _independent_detection(frame: FrameEvidence, identity: str) -> bool:
    return any(str(_value(m, "gt_id", "id")) == identity for m in frame.gt_detection_matches)


def aggregate_distance_conditioned(frames: Sequence[FrameEvidence] | Sequence[Mapping[str, Any]], events: Sequence[IdentityEvent] | None = None, encounters: Sequence[EncounterEvidence] | None = None, params: AnalysisParameters | None = None) -> dict:
    params = params or AnalysisParameters()
    normalized = list(frames) if (not frames or isinstance(frames[0], FrameEvidence)) else normalize_frames(frames, params)
    events = list(events or reconstruct_identity_events(normalized, params))
    result = {label: {"distance_bin": label, "raw_frame_numerator": 0, "raw_frame_denominator": 0, "gt_numerator": 0, "gt_denominator": 0, "pair_numerator": 0, "pair_denominator": 0, "rates": {}, "event_counts": {}, "status": "NO VALID SAMPLES"} for label, _, _ in DISTANCE_BINS}
    for frame in normalized:
        for item in frame.pairwise_gt:
            distance = _value(item, "distance_m", "distance")
            if distance is None:
                continue
            bucket = result[distance_bin(float(distance))]
            bucket["raw_frame_denominator"] += 1
            bucket["raw_frame_numerator"] += int(_pair_observable(frame, tuple(sorted(str(x) for x in item.get("ids", [])))))
            bucket["pair_denominator"] += 1
            bucket["pair_numerator"] += int(_pair_observable(frame, tuple(sorted(str(x) for x in item.get("ids", [])))))
            bucket["status"] = "VALID"
        for truth in frame.ground_truth:
            identity = _gt_id(truth)
            for item in frame.pairwise_gt:
                if identity in [str(x) for x in item.get("ids", [])]:
                    bucket = result[distance_bin(float(_value(item, "distance_m", "distance")))]
                    bucket["gt_denominator"] += 1
                    bucket["gt_numerator"] += int(bool(frame.observability.get(identity, {}).get("observable", False)))
    for event in events:
        for bucket in result.values():
            bucket["event_counts"][event.event] = bucket["event_counts"].get(event.event, 0) + 1
    for bucket in result.values():
        for num, den, name in (("raw_frame_numerator", "raw_frame_denominator", "raw_frame_rate"), ("gt_numerator", "gt_denominator", "gt_rate"), ("pair_numerator", "pair_denominator", "pair_rate")):
            bucket["rates"][name] = bucket[num] / bucket[den] if bucket[den] else None
    return result


def evaluate_episode_validity(metadata: Mapping[str, Any], analysis: RunAnalysis | None = None, *, scenario: str | None = None) -> dict:
    reasons: list[str] = []
    required = ("world_sha256", "scene_sha256", "exit_code", "backend", "topics", "beams_per_sensor", "scan_rate_hz", "stationary", "cmd_vel_publishers", "duration_sec")
    missing = [key for key in required if key not in metadata]
    if missing:
        reasons.append("missing_metadata:" + ",".join(missing))
    if metadata.get("world_sha256") != WORLD_SHA256 or metadata.get("scene_sha256") != SCENE_SHA256:
        reasons.append("provenance")
    if metadata.get("exit_code") != 0 or metadata.get("backend") != "physx_scene_query":
        reasons.append("runtime_contract")
    if metadata.get("beams_per_sensor") != 2000 or not (13.5 <= float(metadata.get("scan_rate_hz", -1)) <= 16.5):
        reasons.append("sensor_contract")
    if metadata.get("stationary") is not True or int(metadata.get("cmd_vel_publishers", 1)) != 0:
        reasons.append("stationary")
    counters = metadata.get("counters", {})
    if any(counters.get(key, 0) != 0 for key in ("missing", "out_of_order", "tf", "time")):
        reasons.append("time_tf_counters")
    if analysis:
        if any(event.classification == "OBSERVABILITY" for event in analysis.identity_events):
            reasons.append("unknown_observability")
        if any(event.classification == "UNRESOLVED" for event in analysis.identity_events):
            reasons.append("unresolved_identity")
        if not any(not encounter.censored for encounter in analysis.encounters):
            reasons.append("no_close_comparable_encounter")
        if scenario in {"B", "C", "D", "E"} and not any(not e.censored for e in analysis.encounters):
            reasons.append("scenario_not_comparable")
    return {"valid": not reasons, "status": "VALID" if not reasons else "INVALID", "reasons": reasons}


def evaluate_algorithm_gates(analysis: RunAnalysis | Mapping[str, Any]) -> dict:
    if isinstance(analysis, Mapping):
        detector_fn = int(analysis.get("detector_fn", analysis.get("fn", 0)))
        separation_num = int(analysis.get("separation_numerator", 0))
        separation_den = int(analysis.get("separation_denominator", 0))
        events = analysis.get("identity_events", [])
    else:
        detector_fn = sum(int(f.raw.get("detector_fn", 0)) for f in analysis.frames)
        separation_num = len(analysis.separation_events)
        separation_den = len(analysis.separation_events)
        events = analysis.identity_events
    def event_field(event: Any, name: str, default: Any = None) -> Any:
        return getattr(event, name, default) if not isinstance(event, Mapping) else event.get(name, default)

    event_names = [event_field(event, "classification") for event in events]
    detector_pass = detector_fn == 0 and separation_num == separation_den and not any(name == "DETECTOR_SEPARATION_INDUCED" for name in event_names)
    id_failures = {"continuous_id_switch", "crossing_id_switch", "fragmentation", "reacquisition_id_change", "tracker_native"}
    id_pass = not any(event_field(event, "event") in id_failures for event in events)
    return {"detector": "PASS" if detector_pass else "FAIL", "id": "PASS" if id_pass else "FAIL", "detector_pass": detector_pass, "id_pass": id_pass}


def derive_research_answers(analysis: RunAnalysis | Mapping[str, Any]) -> dict[str, Any]:
    bins = analysis.distance_conditioned if isinstance(analysis, RunAnalysis) else analysis.get("distance_conditioned", {})
    events = analysis.identity_events if isinstance(analysis, RunAnalysis) else analysis.get("identity_events", [])
    total = len(events)
    detector_events = sum((event.classification if not isinstance(event, Mapping) else event.get("classification")) == "DETECTOR_SEPARATION_INDUCED" for event in events)
    answers = {}
    for index in range(1, 13):
        if index <= 5:
            answers[str(index)] = {"distance_conditioned": bins}
        elif index <= 9:
            answers[str(index)] = {"event_count": total, "detector_separation_events": detector_events}
        else:
            answers[str(index)] = {"data": {"bins": bins, "events": total}}
    return answers


def derive_bottleneck(analysis: RunAnalysis | Mapping[str, Any]) -> dict:
    answers = derive_research_answers(analysis)
    if isinstance(analysis, RunAnalysis):
        eligible = sum(1 for f in analysis.frames for value in f.observability.values() if value.get("observable"))
        detector_failures = sum(int(f.raw.get("detector_fn", 0)) for f in analysis.frames)
    else:
        eligible = int(analysis.get("eligible_observable", 0))
        detector_failures = int(analysis.get("detector_failures", 0))
    ratio = detector_failures / eligible if eligible else None
    return {"q1": answers["1"], "q2": answers["2"], "q3": answers["3"], "q4": {"eligible_observable": eligible, "detector_failure_ratio": ratio, "major_detector_bottleneck": bool(ratio is not None and ratio > 0.5)}}
