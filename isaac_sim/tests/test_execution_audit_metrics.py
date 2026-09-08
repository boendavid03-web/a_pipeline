import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location('execution_audit_metrics', Path(__file__).parents[1] / 'experiments/execution_audit_metrics.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def row(t, x, active=4):
    return dict(sim_time=t, position=[x, 0, 0], quaternion=[0, 0, 0, 1],
                usd_position=[0, 0, 0], active_task_id=active, issued_task_id=4)


def test_slow_motion_uses_speed_not_per_frame_distance():
    samples = [row(i / 60, i / 600) for i in range(601)]
    result = audit.summarize(samples, [1, 0, 0])
    assert result['pass']
    assert result['max_freeze_sec'] == 0


def test_arrival_dwell_is_not_freeze():
    result = audit.summarize([row(i / 60, 1) for i in range(601)], [1, 0, 0])
    assert result['max_freeze_sec'] == 0


def test_two_freeze_intervals_accumulate_and_keep_tail():
    rows = [row(0, 0), row(1, 0), row(2, 0), row(3, .2), row(4, .2), row(5, .2), row(6, .2)]
    result = audit.summarize(rows, [5, 0, 0])
    assert result['freeze_sec'] == 5
    assert result['max_freeze_sec'] == 3
    assert not result['pass']


def test_task_mismatch_counts_events_and_occupancy_separately():
    result = audit.summarize([row(i / 60, 0, a) for i, a in enumerate([4, 5, 5, 4, 5])], [5, 0, 0])
    assert result['task_replacement_events'] == 2
    assert result['task_mismatch_samples'] == 3


def test_unit_quaternion_and_finite_values_required():
    assert not audit.valid_pose([0, 0, 0], [0, 0, 0, 0])
    assert not audit.valid_pose([float('nan'), 0, 0], [0, 0, 0, 1])
    assert not audit.valid_pose([0, 0, 0], [0, 0, 0, 2])


def test_jump_and_bad_clock_cannot_pass_with_arrival():
    result = audit.summarize([row(0, 0), row(.02, 5), row(.02, 5)], [5, 0, 0])
    assert result['discontinuities'] == 1
    assert result['time_errors'] == 1
    assert not result['pass']


def test_empty_and_sparse_samples_cannot_pass():
    assert not audit.summarize([], [0, 0, 0])['pass']
    assert not audit.summarize([row(0, 0), row(3, 1)], [1, 0, 0])['pass']


def test_failed_task_cannot_pass_even_if_id_and_position_look_valid():
    rows = [row(0, 1), row(.02, 1)]
    rows[-1]['issued_task_status'] = 'BehaviorTaskStatus.FAILED'
    result = audit.summarize(rows, [1, 0, 0])
    assert result['task_failure_samples'] == 1
    assert not result['pass']


def test_invalid_telemetry_remains_recordable_in_strict_json():
    import json
    value = audit.json_safe({'position': [float('nan'), float('inf'), 0]})
    assert json.loads(json.dumps(value, allow_nan=False))['position'] == ['nan', 'inf', 0]
