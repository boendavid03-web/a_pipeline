"""Pure helpers shared by navigation planning and episode evaluation."""

from __future__ import annotations

import heapq
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def map_asset_paths(map_yaml, semantic_label_path=""):
    """Resolve the map YAML, occupancy image, and optional semantic-label paths."""
    def resolved(value):
        return str(Path(value).expanduser().resolve()) if value else None

    map_yaml_path = resolved(map_yaml)
    occupancy_map_path = None
    if map_yaml_path and Path(map_yaml_path).is_file():
        with Path(map_yaml_path).open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = Path(map_yaml_path).parent / image_path
        occupancy_map_path = str(image_path.resolve())
    return {
        "map_yaml_path": map_yaml_path,
        "occupancy_map_path": occupancy_map_path,
        "semantic_label_path": resolved(semantic_label_path),
    }


def load_map(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    occupancy = np.asarray(Image.open(image_path).convert("L"))
    origin = metadata["origin"]
    return occupancy, float(metadata["resolution"]), float(origin[0]), float(origin[1])


def world_to_grid(x, y, height, resolution, origin_x, origin_y):
    return height - 1 - int(math.floor((y - origin_y) / resolution)), int(
        math.floor((x - origin_x) / resolution)
    )


def grid_to_world(row, col, height, resolution, origin_x, origin_y):
    return (
        origin_x + (col + 0.5) * resolution,
        origin_y + (height - 1 - row + 0.5) * resolution,
    )


def free_space_mask(occupancy, resolution, inflate_radius):
    """Apply the planner's unknown-as-occupied and circular inflation rule."""
    free = occupancy >= 250
    cells = max(0, int(math.ceil(inflate_radius / resolution)))
    if cells == 0:
        return free
    occupied = ~free
    inflated = occupied.copy()
    rows, cols = np.nonzero(occupied)
    height, width = occupied.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            if dr * dr + dc * dc > cells * cells:
                continue
            inflated[np.clip(rows + dr, 0, height - 1), np.clip(cols + dc, 0, width - 1)] = True
    return ~inflated


def snap_to_free(cell, free, resolution, snap_radius):
    row, col = cell
    height, width = free.shape
    if 0 <= row < height and 0 <= col < width and free[row, col]:
        return row, col
    for radius in range(1, int(math.ceil(snap_radius / resolution)) + 1):
        best = None
        best_d2 = None
        for rr in range(max(0, row - radius), min(height - 1, row + radius) + 1):
            for cc in range(max(0, col - radius), min(width - 1, col + radius) + 1):
                if free[rr, cc]:
                    d2 = (rr - row) ** 2 + (cc - col) ** 2
                    if best is None or d2 < best_d2:
                        best, best_d2 = (rr, cc), d2
        if best is not None:
            return best
    raise RuntimeError(f"No free cell within {snap_radius:.2f} m")


def astar(start, goal, free):
    height, width = free.shape
    neighbors = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
                 (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)))
    queue, parent, score = [(0.0, start)], {}, {start: 0.0}
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            result = [current]
            while current in parent:
                current = parent[current]
                result.append(current)
            return list(reversed(result))
        for dr, dc, cost in neighbors:
            candidate = current[0] + dr, current[1] + dc
            if not (0 <= candidate[0] < height and 0 <= candidate[1] < width) or not free[candidate]:
                continue
            trial = score[current] + cost
            if trial < score.get(candidate, float("inf")):
                parent[candidate], score[candidate] = current, trial
                heapq.heappush(queue, (trial + math.hypot(candidate[0] - goal[0], candidate[1] - goal[1]), candidate))
    raise RuntimeError("A* could not find a path")


def planner_reference_path_length(start_xy, goal_xy, free, resolution, origin_x, origin_y, snap_radius):
    """Return the configured A* reference-path length in metres, or ``None``."""
    try:
        height = free.shape[0]
        start = snap_to_free(world_to_grid(*start_xy, height, resolution, origin_x, origin_y), free, resolution, snap_radius)
        goal = snap_to_free(world_to_grid(*goal_xy, height, resolution, origin_x, origin_y), free, resolution, snap_radius)
        cells = astar(start, goal, free)
        return float(sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(cells, cells[1:])) * resolution)
    except (RuntimeError, ValueError, OverflowError):
        return None


def path_length(points, minimum_segment_m=1.0e-4):
    valid = [(float(x), float(y)) for x, y in points if math.isfinite(x) and math.isfinite(y)]
    return sum(
        distance
        for a, b in zip(valid, valid[1:])
        if (distance := math.hypot(b[0] - a[0], b[1] - a[1])) >= minimum_segment_m
    ) if len(valid) > 1 else 0.0


def _finite(value):
    try:
        return float(value) if math.isfinite(float(value)) else None
    except (TypeError, ValueError):
        return None


def _tracking_statistics(upstream, downstream, *, ratio_epsilon=1.0e-3):
    """JSON-safe signed downstream-minus-upstream tracking statistics."""
    pairs = [(float(a), float(b)) for a, b in zip(upstream, downstream)
             if _finite(a) is not None and _finite(b) is not None]
    if not pairs:
        return {"valid": False, "reason": "no_finite_pairs", "sample_count": 0}
    a = np.asarray([pair[0] for pair in pairs], dtype=float)
    b = np.asarray([pair[1] for pair in pairs], dtype=float)
    error = b - a
    ratios = b[np.abs(a) > ratio_epsilon] / a[np.abs(a) > ratio_epsilon]
    zero_hold = np.abs(b[np.abs(a) <= ratio_epsilon])
    correlation = None
    correlation_reason = None
    if len(a) < 3:
        correlation_reason = "fewer_than_three_samples"
    elif np.std(a) <= ratio_epsilon or np.std(b) <= ratio_epsilon:
        correlation_reason = "near_zero_variance"
    else:
        correlation = float(np.corrcoef(a, b)[0, 1])
    return {
        "valid": True, "reason": None, "sample_count": int(len(a)),
        "bias": float(np.mean(error)), "mae": float(np.mean(np.abs(error))),
        "rmse": float(math.sqrt(np.mean(error ** 2))),
        "p50_absolute_error": float(np.percentile(np.abs(error), 50)),
        "p95_absolute_error": float(np.percentile(np.abs(error), 95)),
        "maximum_absolute_error": float(np.max(np.abs(error))),
        "correlation": correlation, "correlation_reason": correlation_reason,
        "actual_to_command_ratio": float(np.mean(ratios)) if len(ratios) else None,
        "actual_to_command_ratio_sample_count": int(len(ratios)),
        "zero_command_hold_error": float(np.mean(zero_hold)) if len(zero_hold) else None,
        "zero_command_hold_sample_count": int(len(zero_hold)),
    }


def align_actuation_series(decisions, states, *, rate_hz=15.0, freshness_sec=0.20,
                           max_delay_sec=0.50, ratio_epsilon=1.0e-3):
    """Causally align policy decisions and simulator states on simulation time.

    Inputs are mappings with ``time``, decision ``raw``/``final``/``gated`` and
    state ``received``/``applied``/``actual`` fields (each a linear-x scalar).
    Duplicate and nonpositive timestamps are counted and never resampled.
    """
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    period = 1.0 / rate_hz
    diagnostics = {
        "decision_duplicate_or_nonpositive": 0,
        "state_duplicate_or_nonpositive": 0,
        "decision_stale": 0,
        "state_stale": 0,
        "decision_sequence_gaps": 0,
        "telemetry_sequence_gaps": 0,
        "command_sequence_gaps": 0,
    }
    def clean(rows, key):
        result, previous, previous_sequence, previous_command_sequence = [], None, None, None
        # Preserve receive order: sorting would hide a simulation reset or
        # out-of-order transport packet and make it look like valid history.
        for row in rows:
            stamp = _finite(row.get("time"))
            if stamp is None:
                diagnostics[f"{key}_duplicate_or_nonpositive"] += 1
            elif previous is not None and stamp <= previous:
                diagnostics[f"{key}_duplicate_or_nonpositive"] += 1
            else:
                sequence = row.get("sequence")
                if sequence is not None:
                    sequence = int(sequence)
                    if previous_sequence is not None and sequence > previous_sequence:
                        diagnostics[
                            "decision_sequence_gaps" if key == "decision" else "telemetry_sequence_gaps"
                        ] += max(0, sequence - previous_sequence - 1)
                    previous_sequence = sequence
                command_sequence = row.get("command_sequence")
                if command_sequence is not None:
                    command_sequence = int(command_sequence)
                    if previous_command_sequence is not None and command_sequence > previous_command_sequence:
                        diagnostics["command_sequence_gaps"] += max(
                            0, command_sequence - previous_command_sequence - 1
                        )
                    previous_command_sequence = command_sequence
                result.append((stamp, row)); previous = stamp
        return result
    decisions = clean(decisions, "decision")
    states = clean(states, "state")
    if not decisions or not states:
        return {"rows": [], "coverage": 0.0, "diagnostics": diagnostics,
                "raw_to_final": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "final_to_actual": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "raw_to_final_angular": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "final_to_actual_angular": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "gated": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "ungated": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "raw_to_final_gated": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "raw_to_final_ungated": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "gated_angular": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "ungated_angular": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "stable_straight_ungated": {"valid": False, "reason": "missing_source", "sample_count": 0},
                "best_causal_delay_sec": None, "delay_reason": "missing_source"}
    start, end = max(decisions[0][0], states[0][0]), min(decisions[-1][0], states[-1][0])
    if end < start:
        return align_actuation_series([], [], rate_hz=rate_hz)
    rows, di, si, samples = [], 0, 0, 0
    t = start
    while t <= end + 1e-9:
        while di + 1 < len(decisions) and decisions[di + 1][0] <= t:
            di += 1
        while si + 1 < len(states) and states[si + 1][0] <= t:
            si += 1
        dt, decision = decisions[di]
        st, state = states[si]
        if t - dt > freshness_sec:
            diagnostics["decision_stale"] += 1; t += period; continue
        if t - st > freshness_sec:
            diagnostics["state_stale"] += 1; t += period; continue
        raw, final = _finite(decision.get("raw")), _finite(decision.get("final"))
        raw_angular = _finite(decision.get("raw_angular"))
        final_angular = _finite(decision.get("final_angular"))
        received, applied, actual = (_finite(state.get(name)) for name in ("received", "applied", "actual"))
        received_angular, applied_angular, actual_angular = (
            _finite(state.get(name))
            for name in ("received_angular", "applied_angular", "actual_angular")
        )
        if None not in (
            final, final_angular, received, applied, actual,
            received_angular, applied_angular, actual_angular,
        ):
            policy_gated = bool(decision.get("gated", False))
            simulator_gated = bool(
                state.get("watchdog_active", False)
                or state.get("collision_protection_active", False)
                or state.get("control_reasons")
            )
            rows.append({"simulation_time_sec": t, "raw_model_linear_x_mps": raw,
                         "final_command_linear_x_mps": final, "received_command_linear_x_mps": received,
                         "applied_command_linear_x_mps": applied, "actual_linear_x_mps": actual,
                         "raw_model_angular_z_radps": raw_angular,
                         "final_command_angular_z_radps": final_angular,
                         "received_command_angular_z_radps": received_angular,
                         "applied_command_angular_z_radps": applied_angular,
                         "actual_angular_z_radps": actual_angular,
                         "policy_gated": policy_gated,
                         "simulator_gated": simulator_gated,
                         "gated": policy_gated or simulator_gated,
                         "decision_age_sec": t - dt, "state_age_sec": t - st})
            samples += 1
        t += period
    raw_final = _tracking_statistics([row["raw_model_linear_x_mps"] for row in rows], [row["final_command_linear_x_mps"] for row in rows], ratio_epsilon=ratio_epsilon)
    final_actual = _tracking_statistics([row["final_command_linear_x_mps"] for row in rows], [row["actual_linear_x_mps"] for row in rows], ratio_epsilon=ratio_epsilon)
    raw_final_angular = _tracking_statistics(
        [row["raw_model_angular_z_radps"] for row in rows],
        [row["final_command_angular_z_radps"] for row in rows],
        ratio_epsilon=ratio_epsilon,
    )
    final_actual_angular = _tracking_statistics(
        [row["final_command_angular_z_radps"] for row in rows],
        [row["actual_angular_z_radps"] for row in rows],
        ratio_epsilon=ratio_epsilon,
    )
    def group(gated, upstream="final_command_linear_x_mps",
              downstream="actual_linear_x_mps", gate_field="gated"):
        selected = [row for row in rows if row[gate_field] is gated]
        return _tracking_statistics(
            [row[upstream] for row in selected],
            [row[downstream] for row in selected],
            ratio_epsilon=ratio_epsilon,
        )
    stable_straight = [
        row for row in rows
        if not row["gated"]
        and abs(row["final_command_linear_x_mps"]) > 0.05
        and abs(row["final_command_angular_z_radps"]) <= 0.05
    ]
    stable_straight_stats = _tracking_statistics(
        [row["final_command_linear_x_mps"] for row in stable_straight],
        [row["actual_linear_x_mps"] for row in stable_straight],
        ratio_epsilon=ratio_epsilon,
    )
    delay, delay_reason = None, None
    if len(rows) < 3 or np.std([row["final_command_linear_x_mps"] for row in rows]) <= ratio_epsilon:
        delay_reason = "insufficient_or_low_variation_command"
    else:
        candidates = []
        max_steps = min(int(max_delay_sec / period), len(rows) - 3)
        # Every candidate is scored over the same downstream time interval.
        # This prevents a larger delay winning merely because it drops a
        # difficult prefix/suffix.  Commands are always sampled at t-delay.
        actual = np.asarray(
            [row["actual_linear_x_mps"] for row in rows[max_steps:]], dtype=float
        )
        for steps in range(max_steps + 1):
            command = np.asarray(
                [
                    rows[index - steps]["final_command_linear_x_mps"]
                    for index in range(max_steps, len(rows))
                ],
                dtype=float,
            )
            if len(command) >= 3:
                candidates.append((math.sqrt(float(np.mean((actual - command) ** 2))), steps * period))
        if candidates:
            delay = min(candidates)[1]
    return {"rows": rows, "coverage": samples / max(1, int(math.floor((end-start) / period)) + 1),
            "diagnostics": diagnostics, "raw_to_final": raw_final, "final_to_actual": final_actual,
            "raw_to_final_angular": raw_final_angular,
            "final_to_actual_angular": final_actual_angular,
            "gated": group(True), "ungated": group(False),
            "raw_to_final_gated": group(True, "raw_model_linear_x_mps", "final_command_linear_x_mps", "policy_gated"),
            "raw_to_final_ungated": group(False, "raw_model_linear_x_mps", "final_command_linear_x_mps", "policy_gated"),
            "gated_angular": group(True, "final_command_angular_z_radps", "actual_angular_z_radps"),
            "ungated_angular": group(False, "final_command_angular_z_radps", "actual_angular_z_radps"),
            "stable_straight_ungated": stable_straight_stats,
            "best_causal_delay_sec": delay,
            "delay_reason": delay_reason}


def goal_spl(goal_reached, reference_length, actual):
    """Compute the SPL-shaped score for a planner reference (not formal SPL)."""
    if goal_reached is None:
        return None
    if goal_reached is False:
        return 0.0
    if reference_length is None or actual is None or reference_length < 0.0 or actual <= 0.0:
        return None
    return reference_length / max(actual, reference_length)


def planner_reference_metadata(resolution, inflation_radius, snap_radius):
    """Describe the A* reference exactly as it is computed by this module."""
    return {
        "algorithm": "astar_8_connected",
        "resolution": float(resolution) if resolution is not None else None,
        "inflation_radius": float(inflation_radius),
        "endpoint_convention": (
            "world_xy_floor_to_grid; snap_to_nearest_free_cell; "
            "sum_cell_center_steps"
        ),
        "snap_radius": float(snap_radius),
    }


def _differentiate(samples, max_dt):
    """Take one finite difference and return samples plus audit counters."""
    derivatives = []
    diagnostics = {
        "duplicate_timestamp_pairs": 0,
        "nonpositive_dt_pairs": 0,
        "abnormal_dt_pairs": 0,
        "nonfinite_value_pairs": 0,
        "nonfinite_timestamp_pairs": 0,
    }
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if not math.isfinite(t0) or not math.isfinite(t1):
            diagnostics["nonfinite_timestamp_pairs"] += 1
        elif dt == 0.0:
            diagnostics["duplicate_timestamp_pairs"] += 1
            diagnostics["nonpositive_dt_pairs"] += 1
        elif dt < 0.0:
            diagnostics["nonpositive_dt_pairs"] += 1
        elif dt > max_dt:
            diagnostics["abnormal_dt_pairs"] += 1
        elif not math.isfinite(v0) or not math.isfinite(v1):
            diagnostics["nonfinite_value_pairs"] += 1
        else:
            # A derivative belongs to the later source timestamp.  Repeating
            # this operation therefore implements velocity -> acceleration ->
            # jerk without assuming uniform sampling.
            derivatives.append((t1, (v1 - v0) / dt))
    return derivatives, diagnostics


def derivative_summary(samples, order, max_dt=2.0):
    """Return RMS/max and timestamp-quality audit data for a derivative.

    ``order=1`` converts velocity to acceleration, and ``order=2`` converts
    velocity to acceleration then jerk.  Duplicate/non-positive and abnormal
    time intervals are skipped rather than silently producing invalid values.
    """
    if order not in (1, 2):
        raise ValueError("order must be 1 (acceleration) or 2 (jerk)")
    if not math.isfinite(max_dt) or max_dt <= 0.0:
        raise ValueError("max_dt must be finite and positive")
    current = [(float(t), float(value)) for t, value in samples]
    diagnostics = {
        "duplicate_timestamp_pairs": 0,
        "nonpositive_dt_pairs": 0,
        "abnormal_dt_pairs": 0,
        "nonfinite_value_pairs": 0,
        "nonfinite_timestamp_pairs": 0,
    }
    final_source = current
    for _ in range(order):
        final_source = current
        current, level_diagnostics = _differentiate(current, max_dt)
        for key, value in level_diagnostics.items():
            diagnostics[key] += value
    values = np.asarray([value for _, value in current], dtype=float)
    finite_values = values[np.isfinite(values)]
    # Derivative samples are attached to the later timestamp.  Integrating a
    # sample over the following valid interval is conservative and, unlike a
    # raw sum, keeps the result meaningful for irregular ROS time sampling.
    integrated_squared = 0.0
    source_by_end = {t1: t1 - t0 for (t0, _), (t1, _) in zip(final_source, final_source[1:])
                     if math.isfinite(t0) and math.isfinite(t1)}
    for t1, value in current:
        dt = source_by_end.get(t1)
        if dt is not None and 0.0 < dt <= max_dt and math.isfinite(value):
            integrated_squared += value * value * dt
    def value_or_none(fn):
        return float(fn(finite_values)) if len(finite_values) else None
    return {
        "minimum": value_or_none(np.min),
        "signed_minimum": value_or_none(np.min),
        "mean": value_or_none(np.mean),
        "signed_mean": value_or_none(np.mean),
        "maximum_signed": value_or_none(np.max),
        "signed_maximum": value_or_none(np.max),
        "mean_abs": value_or_none(lambda a: np.mean(np.abs(a))),
        "rms": value_or_none(lambda a: np.sqrt(np.mean(a ** 2))),
        "p95_abs": value_or_none(lambda a: np.percentile(np.abs(a), 95)),
        "max_abs": value_or_none(lambda a: np.max(np.abs(a))),
        # Existing consumers use ``maximum`` as an absolute maximum.
        "maximum": value_or_none(lambda a: np.max(np.abs(a))),
        "integrated_squared": float(integrated_squared) if len(finite_values) else None,
        "sample_count": int(len(finite_values)),
        **diagnostics,
    }


def time_milestones(goal_accepted_time, reach_time, first_odom_time, first_policy_action_time, first_nonzero_cmd_time):
    """Calculate reach durations from raw ROS-time milestones, preserving 0."""
    def duration(start):
        if start is None or reach_time is None:
            return None
        if not math.isfinite(start) or not math.isfinite(reach_time) or reach_time < start:
            return None
        return float(reach_time - start)

    return {
        "goal_to_reach_sec": duration(goal_accepted_time),
        "first_odom_to_reach_sec": duration(first_odom_time),
        "first_policy_action_to_reach_sec": duration(first_policy_action_time),
        "first_nonzero_cmd_to_reach_sec": duration(first_nonzero_cmd_time),
    }


def ttc_statistics(samples, episode_time_sec, threshold_sec=2.0, max_dt=2.0):
    """Summarize TTC with no-collision samples represented by ``None``/+inf."""
    if not math.isfinite(threshold_sec) or threshold_sec < 0.0:
        raise ValueError("threshold_sec must be finite and non-negative")
    if not math.isfinite(max_dt) or max_dt <= 0.0:
        raise ValueError("max_dt must be finite and positive")
    valid_samples = [(float(timestamp), value) for timestamp, value in samples if timestamp is not None and math.isfinite(timestamp)]
    finite_values = [float(value) for _, value in valid_samples if value is not None and math.isfinite(value)]
    finite_time = below_threshold_time = 0.0
    below_one_time = below_two_time = 0.0
    for (t0, value), (t1, _next_value) in zip(valid_samples, valid_samples[1:]):
        dt = t1 - t0
        if not (0.0 < dt <= max_dt):
            continue
        # ``None`` is deliberately +inf here: it contributes neither finite
        # TTC coverage nor time below the collision-risk threshold.
        if value is not None and math.isfinite(value):
            finite_time += dt
            if value < threshold_sec:
                below_threshold_time += dt
            if value < 1.0:
                below_one_time += dt
            if value < 2.0:
                below_two_time += dt
    valid_episode_time = (
        float(episode_time_sec)
        if episode_time_sec is not None and math.isfinite(episode_time_sec) and episode_time_sec > 0.0
        else None
    )
    return {
        "minimum_finite_ttc_sec": min(finite_values) if finite_values else None,
        "p05_finite_ttc_sec": float(np.percentile(finite_values, 5)) if finite_values else None,
        "finite_ttc_sample_fraction": len(finite_values) / len(valid_samples) if valid_samples else None,
        "finite_ttc_time_fraction": finite_time / valid_episode_time if valid_episode_time else None,
        "time_below_threshold_sec": below_threshold_time,
        "episode_time_fraction_below_threshold_sec": below_threshold_time / valid_episode_time if valid_episode_time else None,
        "time_below_1_sec": below_one_time,
        "time_below_2_sec": below_two_time,
        "episode_time_fraction_below_1_sec": below_one_time / valid_episode_time if valid_episode_time else None,
        "episode_time_fraction_below_2_sec": below_two_time / valid_episode_time if valid_episode_time else None,
        "threshold_sec": float(threshold_sec),
    }


def distribution_summary(values):
    """JSON-safe descriptive distribution statistics for finite values."""
    finite = np.asarray([float(v) for v in values if v is not None and math.isfinite(float(v))], dtype=float)
    if not len(finite):
        return {"count": 0, "minimum": None, "mean": None, "maximum": None,
                "p05": None, "p50": None, "p95": None, "std": None}
    return {"count": int(len(finite)), "minimum": float(np.min(finite)),
            "mean": float(np.mean(finite)), "maximum": float(np.max(finite)),
            "p05": float(np.percentile(finite, 5)), "p50": float(np.percentile(finite, 50)),
            "p95": float(np.percentile(finite, 95)), "std": float(np.std(finite))}


def threshold_exposure(samples, threshold, direction="below"):
    """Return sample and time exposure to a threshold.

    ``samples`` may be values or ``(timestamp, value)`` pairs.  Time is
    accumulated only on valid positive intervals and is therefore explicitly
    unavailable for a single sample.
    """
    if not math.isfinite(float(threshold)):
        raise ValueError("threshold must be finite")
    pairs = list(samples)
    if pairs and isinstance(pairs[0], (tuple, list)) and len(pairs[0]) == 2:
        pairs = [(float(t), None if v is None else float(v)) for t, v in pairs
                 if t is not None and math.isfinite(float(t)) and
                 (v is None or math.isfinite(float(v)) or float(v) == float("inf"))]
        predicate = (lambda v: v < threshold) if direction == "below" else (lambda v: v > threshold)
        total = exposed = 0.0
        exposed_flags = [v is not None and predicate(v) for _, v in pairs]
        for (t0, v0), (t1, _v1) in zip(pairs, pairs[1:]):
            dt = t1 - t0
            if dt > 0.0 and math.isfinite(dt) and v0 is not None:
                total += dt
                if predicate(v0):
                    exposed += dt
        entry_count = sum(1 for before, now in zip([False] + exposed_flags, exposed_flags) if now and not before)
        count = sum(exposed_flags)
        max_penetration = max((threshold - v if direction == "below" else v - threshold)
                              for (_, v), exposed_now in zip(pairs, exposed_flags) if exposed_now) if count else 0.0
        return {"threshold": float(threshold), "direction": direction, "sample_count": len(pairs),
                "exposed_sample_count": int(count), "exposure_time_sec": exposed,
                "exposure_ratio": exposed / total if total > 0.0 else None,
                "entry_count": int(entry_count), "max_penetration": float(max_penetration) if entry_count else 0.0}
    vals = [float(v) for v in pairs if v is not None and math.isfinite(float(v))]
    predicate = (lambda v: v < threshold) if direction == "below" else (lambda v: v > threshold)
    exposed_count = int(sum(predicate(v) for v in vals))
    return {"threshold": float(threshold), "direction": direction, "sample_count": len(vals),
            "exposed_sample_count": exposed_count, "exposure_time_sec": None,
            "exposure_ratio": (exposed_count / len(vals)) if vals else None,
            "entry_count": None, "max_penetration": max((threshold - v if direction == "below" else v - threshold) for v in vals if predicate(v)) if exposed_count else 0.0}


def failure_to_progress_summary(samples, window=5.0, progress=0.2):
    """Detect windows whose goal-distance reduction is below ``progress`` m."""
    if window <= 0.0 or progress < 0.0:
        raise ValueError("window must be positive and progress non-negative")
    pairs = [(float(t), float(d)) for t, d in samples if t is not None and d is not None and math.isfinite(float(t)) and math.isfinite(float(d))]
    evaluations = []
    for index, (start_t, start_d) in enumerate(pairs):
        end_candidates = [(t, d) for t, d in pairs[index + 1:] if t - start_t >= window]
        if not end_candidates:
            continue
        end_t, end_d = end_candidates[0]
        evaluations.append((end_t, start_d - end_d < progress))
    failures = [failed for _timestamp, failed in evaluations]
    event_count = sum(1 for before, current in zip([False] + failures, failures)
                      if current and not before)
    evaluated_duration = stalled_duration = 0.0
    for (t0, stalled), (t1, _next_stalled) in zip(evaluations, evaluations[1:]):
        dt = t1 - t0
        if math.isfinite(dt) and dt > 0.0:
            evaluated_duration += dt
            if stalled:
                stalled_duration += dt
    return {"window_sec": float(window), "progress_threshold_m": float(progress),
            "window_count": len(failures), "event_count": int(event_count),
            "evaluated_duration_sec": float(evaluated_duration),
            "stalled_duration_sec": float(stalled_duration),
            "stalled_duration_ratio": stalled_duration / evaluated_duration if evaluated_duration > 0.0 else None,
            "failed": bool(failures and any(failures)) if failures else None}


def path_irregularity_summary(points, minimum_segment_m=1.0e-4,
                               minimum_path_length_m=1.0e-3):
    """Geometric turning proxy: absolute heading change per metre (rad/m)."""
    valid = [(float(x), float(y)) for x, y in points if x is not None and y is not None and math.isfinite(float(x)) and math.isfinite(float(y))]
    segments = [
        (a, b, math.hypot(b[0] - a[0], b[1] - a[1]))
        for a, b in zip(valid, valid[1:])
    ]
    accepted = [(a, b, length) for a, b, length in segments if length >= minimum_segment_m]
    lengths = [length for _a, _b, length in accepted]
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b, _length in accepted]
    turns = [abs((h1 - h0 + math.pi) % (2.0 * math.pi) - math.pi)
             for h0, h1 in zip(headings, headings[1:])]
    total_turn = float(sum(turns)) if turns else 0.0
    necessary_turn = (abs((headings[-1] - headings[0] + math.pi) % (2.0 * math.pi) - math.pi)
                      if len(headings) >= 2 else 0.0)
    unnecessary_turn = max(0.0, total_turn - necessary_turn)
    length = float(sum(lengths))
    metric_valid = length >= minimum_path_length_m
    return {"turn_count": len(turns), "total_turn_rad": total_turn,
            "necessary_turn_rad": necessary_turn, "unnecessary_turn_rad": unnecessary_turn,
            "path_length_m": length,
            "ignored_micro_segment_count": len(segments) - len(accepted),
            "valid": metric_valid,
            "reason": None if metric_valid else "path_too_short",
            "turning_rad_per_m": (unnecessary_turn / length) if metric_valid else None}


def personal_space_integral(samples, threshold):
    """Integrate threshold violations over valid simulation-time intervals."""
    violation = total = 0.0
    for (t0, d0), (t1, _d1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt > 0.0 and d0 is not None and (math.isfinite(d0) or d0 == float("inf")):
            total += dt
            if math.isfinite(d0) and d0 < threshold:
                violation += dt
    return violation, (violation / total if total > 0.0 else None)


def constant_velocity_ttc(robot_xy, robot_velocity_world, pedestrian_xy, pedestrian_velocity_world, collision_radius):
    """First future contact time for a closing constant-velocity pair."""
    relative_position = np.asarray(pedestrian_xy, dtype=float) - np.asarray(robot_xy, dtype=float)
    relative_velocity = np.asarray(pedestrian_velocity_world, dtype=float) - np.asarray(robot_velocity_world, dtype=float)
    a = float(relative_velocity @ relative_velocity)
    b = 2.0 * float(relative_position @ relative_velocity)
    c = float(relative_position @ relative_position) - collision_radius ** 2
    if not all(math.isfinite(value) for value in (a, b, c)):
        return None
    if c <= 0.0:
        return 0.0
    if b >= 0.0:
        return None
    if a <= 1e-12:
        return None
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None
    roots = ((-b - math.sqrt(discriminant)) / (2.0 * a), (-b + math.sqrt(discriminant)) / (2.0 * a))
    future = [root for root in roots if root >= 0.0]
    return min(future) if future else None
