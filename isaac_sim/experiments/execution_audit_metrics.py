"""Simulator-independent gates for the isolated locomotion audit (SI units)."""
from __future__ import annotations

import math


def json_safe(value):
    """Keep invalid telemetry explicit without writing JSON NaN/Infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def valid_pose(position, quaternion, tolerance=1.0e-3):
    return (
        len(position) == 3 and len(quaternion) == 4
        and all(math.isfinite(v) for v in [*position, *quaternion])
        and abs(math.sqrt(sum(v * v for v in quaternion)) - 1.0) <= tolerance
    )


def summarize(samples, goal, *, goal_tolerance=0.45, freeze_speed=0.05):
    """Use runtime position deltas, simulation seconds and transition counts.

    Authored USD positions and agent-reported navigation velocity are diagnostics,
    never substitutes for measured root displacement. Arrival dwell is not freeze.
    """
    result = dict(samples=len(samples), distance_m=0.0, freeze_sec=0.0,
                  max_freeze_sec=0.0, task_replacement_events=0,
                  task_mismatch_samples=0, invalid_pose_samples=0,
                  task_failure_samples=0,
                  discontinuities=0, time_errors=0, max_sample_gap_sec=0.0,
                  max_usd_runtime_distance_m=0.0, reached=False)
    previous = None
    freeze = 0.0
    mismatch_previous = False
    for row in samples:
        result['task_failure_samples'] += int(str(row.get('issued_task_status', '')).upper().split('.')[-1] == 'FAILED')
        if not valid_pose(row['position'], row['quaternion']):
            result['invalid_pose_samples'] += 1
            previous = None
            freeze = 0.0
            continue
        position = row['position']
        gap = math.dist(position, row['usd_position'])
        result['max_usd_runtime_distance_m'] = max(result['max_usd_runtime_distance_m'], gap)
        arrived = math.dist(position, goal) <= goal_tolerance
        result['reached'] |= arrived
        mismatch = row['active_task_id'] != row['issued_task_id'] and not arrived
        result['task_mismatch_samples'] += int(mismatch)
        result['task_replacement_events'] += int(mismatch and not mismatch_previous)
        mismatch_previous = mismatch
        if previous is not None:
            dt = row['sim_time'] - previous['sim_time']
            if dt <= 0 or not math.isfinite(dt):
                result['time_errors'] += 1
                previous = row
                freeze = 0.0
                continue
            result['max_sample_gap_sec'] = max(result['max_sample_gap_sec'], dt)
            displacement = math.dist(position, previous['position'])
            result['distance_m'] += displacement
            result['discontinuities'] += int(displacement > max(0.5, 3.0 * dt))
            if displacement / dt < freeze_speed and not arrived:
                freeze += dt
                result['freeze_sec'] += dt
                result['max_freeze_sec'] = max(result['max_freeze_sec'], freeze)
            else:
                freeze = 0.0
        previous = row
    result['goal_distance_m'] = math.dist(previous['position'], goal) if previous else None
    result['duration_sec'] = samples[-1]['sim_time'] - samples[0]['sim_time'] if len(samples) > 1 else 0.0
    result['pass'] = bool(
        result['reached'] and len(samples) >= 2
        and result['max_freeze_sec'] <= 2.0 and result['max_sample_gap_sec'] <= 0.25
        and not any(result[k] for k in ('invalid_pose_samples', 'discontinuities',
                                      'task_replacement_events', 'task_failure_samples', 'time_errors'))
    )
    return result
