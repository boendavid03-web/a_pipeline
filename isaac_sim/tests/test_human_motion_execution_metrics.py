"""Regression cases that expose causal alignment and false-pass errors."""
import copy
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from human_motion_execution_metrics import analyze, command_at, integrate, segment_feasibility, validate_schedule


def trace(schedule, duration=3, delay=0, frozen=False):
    position = [0.0, 0.0, 0.0]
    rows = []
    dt = 1 / 60
    for i in range(round(duration / dt)):
        t = i * dt
        _, c = command_at(schedule, t)
        delta = [0, 0] if frozen or t < delay else integrate(schedule, max(0, t - delay), max(0, t + dt - delay))
        after = [position[0] + delta[0], position[1] + delta[1], 0.0]
        rows.append(dict(t0=t, t1=t+dt, position0=position, position1=after,
                         quaternion=[0, 0, 0, 1], command=c['velocity'],
                         active_task_id=4, issued_task_id=4, task_running=True,
                         task_status='RUNNING', navmesh_valid=True))
        position = after
    return rows


STRAIGHT = [dict(t=0, velocity=[1, 0])]


def test_exact_vector_tracking_passes():
    result = analyze(trace(STRAIGHT), STRAIGHT, 3)
    assert result['status'] == 'PASS'
    assert result['vector_rmse_mps'] < 1e-10
    assert result['max_deviation_m'] < 1e-10


def test_integral_splits_command_boundary_inside_frame():
    schedule = [dict(t=0, velocity=[1, 0]), dict(t=0.007, velocity=[0, 1])]
    assert integrate(schedule, 0, 0.02) == pytest.approx([0.007, 0.013])


def test_latency_is_not_removed_by_best_fit():
    result = analyze(trace(STRAIGHT, delay=0.5), STRAIGHT, 3)
    assert result['vector_rmse_mps'] > 0.3
    assert 0.4 < result['phases'][0]['onset_sec'] < 0.65
    assert result['final_deviation_m'] == pytest.approx(0.5)


def test_intentional_stop_is_not_freeze():
    stop = [dict(t=0, velocity=[0, 0])]
    result = analyze(trace(stop), stop, 3)
    assert result['status'] == 'PASS'
    assert result['stop_intent_sec'] == pytest.approx(3)
    assert result['max_freeze_sec'] == 0


def test_nonzero_intent_frozen_root_fails():
    result = analyze(trace(STRAIGHT, frozen=True), STRAIGHT, 3)
    assert result['status'] == 'FAIL_TRACKING'
    assert 'freeze' in result['tracking_failures']
    assert result['phases'][0]['response_sec'] is None


@pytest.mark.parametrize('mutation', ['failed', 'replaced', 'not_running', 'nan', 'gap', 'navmesh', 'short'])
def test_integrity_never_passes(mutation):
    rows = copy.deepcopy(trace(STRAIGHT))
    if mutation == 'failed':
        rows[50]['task_status'] = 'BehaviorTaskStatus.FAILED'
    elif mutation == 'replaced':
        rows[50]['active_task_id'] = 9
    elif mutation == 'not_running':
        rows[50]['task_running'] = False
    elif mutation == 'nan':
        rows[50]['position1'][0] = math.nan
    elif mutation == 'gap':
        rows[50]['t1'] += 0.5
    elif mutation == 'navmesh':
        rows[50]['navmesh_valid'] = False
    else:
        rows = rows[:-30]
    assert analyze(rows, STRAIGHT, 3)['status'] == 'INVALID'


def test_reset_failure_rejects_perfect_motion():
    assert analyze(trace(STRAIGHT), STRAIGHT, 3, reset_pass=False)['status'] == 'INVALID'


def test_stop_go_measures_braking_and_restart_separately():
    schedule = [dict(t=0, velocity=[1, 0]), dict(t=2, velocity=[0, 0]), dict(t=4, velocity=[1, 0])]
    result = analyze(trace(schedule, 6, delay=0.5), schedule, 6)
    assert result['phases'][1]['stop_distance_m'] == pytest.approx(0.5)
    assert 0.4 < result['phases'][2]['onset_sec'] < 0.65
    assert 'stopping_distance' in result['tracking_failures']


def test_json_safe_invalid_quaternion_still_rejected():
    rows = trace(STRAIGHT)
    rows[20]['quaternion'][0] = 'nan'
    assert analyze(rows, STRAIGHT, 3)['status'] == 'INVALID'


def test_free_segment_with_planner_detour_is_measured_not_rejected():
    result = segment_feasibility([2, 4, 0], [3, 4, 0], lambda p: (p, 0), 1.1049)
    assert result['valid']
    assert result['planner_excess_length_m'] == pytest.approx(0.1049)


def test_segment_interior_blockage_rejected_despite_valid_endpoints():
    def project(p):
        return ([p[0], p[1] + 0.2, 0], 0) if 2.4 < p[0] < 2.6 else (p, 0)
    assert not segment_feasibility([2, 4, 0], [3, 4, 0], project, 1.5)['valid']


@pytest.mark.parametrize('schedule', [[], [dict(t=1, velocity=[1, 0])],
                                     [dict(t=0, velocity=[math.nan, 0])],
                                     [dict(t=0, velocity=[1, 0]), dict(t=0, velocity=[0, 1])]])
def test_invalid_command_input_rejected(schedule):
    with pytest.raises(ValueError):
        validate_schedule(schedule)
