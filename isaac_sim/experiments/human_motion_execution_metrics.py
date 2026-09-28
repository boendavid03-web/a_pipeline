"""Causal, simulation-time metrics for the opt-in velocity execution benchmark."""
from __future__ import annotations

import bisect
import math

from execution_audit_metrics import valid_pose


THRESHOLDS = dict(reset_error_m=0.15, max_sample_gap_sec=0.025,
                  max_freeze_sec=2.0, steady_vector_rmse_mps=0.20,
                  response_sec=1.0, stop_distance_m=0.30,
                  path_deviation_m=0.50, filter_window_sec=0.20)


def validate_schedule(schedule):
    if not schedule or schedule[0]['t'] != 0:
        raise ValueError('schedule must start at zero')
    previous = -1.0
    for command in schedule:
        values = [command['t'], *command['velocity']]
        if len(command['velocity']) != 2 or not all(math.isfinite(x) for x in values):
            raise ValueError('non-finite or non-planar command')
        if command['t'] <= previous:
            raise ValueError('command times must be strictly increasing')
        previous = command['t']


def command_at(schedule, t):
    index = max(0, bisect.bisect_right([c['t'] for c in schedule], t + 1e-9) - 1)
    return index, schedule[index]


def integrate(schedule, start, end):
    """Exact zero-order-hold command integral, including boundaries inside frames."""
    displacement = [0.0, 0.0]
    for i, c in enumerate(schedule):
        right = schedule[i + 1]['t'] if i + 1 < len(schedule) else end
        dt = max(0.0, min(end, right) - max(start, c['t']))
        for axis in range(2):
            displacement[axis] += dt * c['velocity'][axis]
    return displacement


def segment_feasibility(position, target, project, route_length):
    """Check direct-segment NavMesh coverage, separately from planner detour.

    A generated path can zigzag within free space. Its excess length describes
    the planner response and cannot establish that the input segment is blocked.
    Projection is only observed here; no projected target is sent to the agent.
    """
    distance = math.dist(position, target)
    count = max(1, math.ceil(distance / 0.05))
    errors = []
    islands = []
    for i in range(count + 1):
        point = [a + (b - a) * i / count for a, b in zip(position, target)]
        nearest, island = project(point)
        errors.append(math.dist(nearest[:2], point[:2]) if nearest is not None else math.inf)
        islands.append(island)
    return dict(valid=all(e <= 0.05 for e in errors) and all(i >= 0 for i in islands)
                and (distance < 1e-5 or route_length is not None),
                checked_points=len(errors), max_projection_error_m=max(errors),
                direct_distance_m=distance, planned_length_m=route_length,
                planner_excess_length_m=None if route_length is None else route_length - distance)


def weighted_rmse(rows, key):
    duration = sum(r['dt'] for r in rows)
    return math.sqrt(sum(r['dt'] * r[key] ** 2 for r in rows) / duration) if duration else None


def sustained(rows, condition, dwell):
    first = None
    for row in rows:
        if condition(row):
            if first is None:
                first = row['t0']
            if row['t1'] - first >= dwell - 1e-8:
                return first
        else:
            first = None
    return None


def analyze(intervals, schedule, duration, *, reset_pass=True, synthetic=True):
    """Each row pairs a pre-update command with the following live pose interval.

    Raw frame velocities include root-motion gait variation. The causal 0.2 s
    average uses measured displacement and time, never navigation velocity.
    Input-to-output error uses the scheduled integral; executor error uses the
    command actually written before app.update(). Neither is shifted for fit.
    """
    validate_schedule(schedule)
    result = dict(samples=len(intervals), status='INCOMPLETE', integrity_errors=[],
                  max_freeze_sec=0.0, stop_intent_sec=0.0, thresholds=THRESHOLDS.copy())
    errors = result['integrity_errors']
    rows = []
    reference = [0.0, 0.0]
    displacement = [0.0, 0.0]
    freeze = 0.0
    for i, r in enumerate(intervals):
        dt = r['t1'] - r['t0']
        values = [r['t0'], r['t1'], *r['position0'], *r['position1'], *r['command'], *r['quaternion']]
        if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
            errors.append('nonfinite_interval')
            continue
        if dt <= 0 or dt > THRESHOLDS['max_sample_gap_sec'] + 1e-8:
            errors.append('sample_dt')
            continue
        if i and (abs(r['t0'] - intervals[i - 1]['t1']) > 1e-7
                  or math.dist(r['position0'], intervals[i - 1]['position1']) > 1e-6):
            errors.append('discontinuous_interval_chain')
        if not valid_pose(r['position1'], r['quaternion']):
            errors.append('invalid_pose')
        if r['active_task_id'] != r['issued_task_id'] or not r['task_running']:
            errors.append('task_not_running_or_replaced')
        if str(r['task_status']).upper().split('.')[-1] == 'FAILED':
            errors.append('task_failed')
        if r.get('navmesh_valid') is not True:
            errors.append('target_or_root_navmesh')
        delta = [r['position1'][a] - r['position0'][a] for a in range(2)]
        if math.hypot(*delta) > max(0.5, 3.0 * dt):
            errors.append('pose_jump')
        velocity = [d / dt for d in delta]
        expected = integrate(schedule, r['t0'], r['t1'])
        for axis in range(2):
            reference[axis] += expected[axis]
            displacement[axis] += delta[axis]
        row = dict(r, dt=dt, velocity=velocity, speed=math.hypot(*velocity),
                   executor_error=math.dist(velocity, r['command']),
                   input_error=math.dist(velocity, [x / dt for x in expected]),
                   deviation=math.dist(displacement, reference))
        if math.hypot(*r['command']) > 0.1 and row['speed'] < 0.05:
            freeze += dt
            result['max_freeze_sec'] = max(result['max_freeze_sec'], freeze)
        else:
            freeze = 0.0
        if math.hypot(*r['command']) <= 0.1:
            result['stop_intent_sec'] += dt
        rows.append(row)
    if not reset_pass:
        errors.append('reset')
    if not rows or abs(rows[0]['t0']) > 1e-7 or rows[-1]['t1'] < duration - 1e-7:
        errors.append('duration_coverage')
    # Integrate overlapping measured intervals for a fixed causal time window.
    window = THRESHOLDS['filter_window_sec']
    for row in rows:
        left = max(0.0, row['t1'] - window)
        relevant = [x for x in rows if x['t1'] > left + 1e-9 and x['t0'] < row['t1'] - 1e-9]
        weights = [min(x['t1'], row['t1']) - max(x['t0'], left) for x in relevant]
        span = sum(weights)
        row['filtered_velocity'] = [sum(w * x['velocity'][a] for w, x in zip(weights, relevant)) / span for a in range(2)]
        row['filtered_speed'] = math.hypot(*row['filtered_velocity'])
        applied = [sum(w * x['command'][a] for w, x in zip(weights, relevant)) / span for a in range(2)]
        row['filtered_executor_error'] = math.dist(row['filtered_velocity'], applied)
    phases = []
    if synthetic:
        for i, c in enumerate(schedule):
            end = schedule[i + 1]['t'] if i + 1 < len(schedule) else duration
            part = [r for r in rows if r['t0'] >= c['t'] - 1e-8 and r['t1'] <= end + 1e-8]
            steady = [r for r in part if r['t0'] >= c['t'] + 1.0 - 1e-8]
            moving = math.hypot(*c['velocity']) > 0.1
            onset = sustained(part, lambda r: r['filtered_speed'] > 0.1, 0.2) if moving else None
            response = sustained(part, lambda r: math.dist(r['filtered_velocity'], c['velocity']) <= (0.2 if moving else 0.1), 0.2)
            phase = dict(t=c['t'], end=end, velocity=c['velocity'],
                         onset_sec=None if onset is None else onset - c['t'],
                         response_sec=None if response is None else response - c['t'],
                         steady_vector_rmse_mps=weighted_rmse(steady, 'filtered_executor_error'),
                         observed_sec=sum(r['dt'] for r in part))
            if not moving:
                # Entire commanded stop, including root drift, not net displacement.
                phase['stop_distance_m'] = sum(r['speed'] * r['dt'] for r in part)
            phases.append(phase)
    result.update(integrity_errors=sorted(set(errors)), phases=phases,
                  vector_rmse_mps=weighted_rmse(rows, 'executor_error'),
                  scheduled_input_rmse_mps=weighted_rmse(rows, 'input_error'),
                  filtered_vector_rmse_mps=weighted_rmse(rows, 'filtered_executor_error'),
                  final_deviation_m=rows[-1]['deviation'] if rows else None,
                  max_deviation_m=max((r['deviation'] for r in rows), default=None),
                  distance_m=sum(r['speed'] * r['dt'] for r in rows),
                  observed_duration_sec=sum(r['dt'] for r in rows),
                  max_sample_gap_sec=max((r['dt'] for r in rows), default=None))
    failures = []
    if result['max_freeze_sec'] > THRESHOLDS['max_freeze_sec']:
        failures.append('freeze')
    if result['max_deviation_m'] is None or result['max_deviation_m'] > THRESHOLDS['path_deviation_m']:
        failures.append('path_deviation')
    if not synthetic and (result['filtered_vector_rmse_mps'] is None
                          or result['filtered_vector_rmse_mps'] > THRESHOLDS['steady_vector_rmse_mps']):
        failures.append('replay_tracking')
    for p in phases:
        if p['steady_vector_rmse_mps'] is None or p['steady_vector_rmse_mps'] > THRESHOLDS['steady_vector_rmse_mps']:
            failures.append('steady_tracking')
        if p['response_sec'] is None or p['response_sec'] > THRESHOLDS['response_sec']:
            failures.append('step_response')
        if p.get('stop_distance_m', 0) > THRESHOLDS['stop_distance_m']:
            failures.append('stopping_distance')
    result['tracking_failures'] = sorted(set(failures))
    result['status'] = 'INVALID' if errors else ('FAIL_TRACKING' if failures else 'PASS')
    result['motion_integrity_pass'] = not errors and result['max_freeze_sec'] <= THRESHOLDS['max_freeze_sec']
    return result
