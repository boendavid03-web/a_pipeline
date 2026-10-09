"""Offline analysis of actual native SDK clock intervals and recorded SRNN tensors."""
import collections
import json
import math
import pathlib
import statistics
import struct
import sys

case = pathlib.Path(sys.argv[1])
trace = pathlib.Path('[LOCAL_PATH]') / case.name
files = list(trace.glob('sdk_*.jsonl'))
assert len(files) == 1
rows = [json.loads(line) for line in files[0].read_text().splitlines()]
obs = [r for r in rows if r['kind'] == 'obs']
steps = [r for r in rows if r['kind'] == 'step']
inputs = [r for r in rows if r['kind'] == 'actual_neural_input']
observations = {r['seq']: r for r in obs}
intervals = [(a, b, b['sim'] - a['sim']) for a, b in zip(obs, obs[1:])]
# The SDK sends one immediate startup observation before the periodic timer.
# Preserve that interval separately rather than attributing it to steady control.
regular = [v for _, _, v in intervals[1:] if v > 0]
active = [v for a, b, v in intervals if a.get('active') and b.get('active') and v > 0]
errors = []
with_human = 0
missing_state = 0
f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
for row in inputs:
    features = observations[row['seq']]['features']
    px, py, yaw = features['robot_pose']
    theta = yaw % (2.0 * math.pi)
    gx, gy = features['goal_pose'][:2]
    state = features.get('robot_state')
    if state is not None and len(state) >= 4:
        vx, vy = state[2:4]
    else:
        missing_state += 1
        vx, vy = 0.0, 0.0
    expected_robot = [0.0, 0.0, .3, gx - px, gy - py, 1.0, theta]
    expected_velocity = [vx * math.cos(theta) - vy * math.sin(theta),
                         vx * math.sin(theta) + vy * math.cos(theta)]
    relative = sorted(((p[1] - px) ** 2 + (p[2] - py) ** 2, p[1] - px, p[2] - py)
                      for p in features.get('pedestrians') or [])
    with_human += bool(relative)
    observed = row['inputs']
    expected_spatial = [[x, y] for _, x, y in relative[:len(observed['spatial_edges'][0])]]
    expected_spatial += [[15.0, 15.0]] * (len(observed['spatial_edges'][0]) - len(expected_spatial))
    for expected, actual in [(expected_robot, observed['robot_node'][0][0]),
                             (expected_velocity, observed['temporal_edges'][0][0]),
                             *zip(expected_spatial, observed['spatial_edges'][0])]:
        assert len(expected) == len(actual)
        errors.extend(abs(f32(e) - a) for e, a in zip(expected, actual))

result = {
    'case': case.name, 'actual_file': str(files[0]), 'identity': rows[0],
    'counts': dict(collections.Counter(r['kind'] for r in rows)),
    'all_obs_interval_distribution': dict(collections.Counter(round(v, 9) for _, _, v in intervals)),
    'bootstrap_first_interval_s': intervals[0][2] if intervals else None,
    'regular_intervals_count': len(regular),
    'regular_interval_min_s': min(regular) if regular else None,
    'regular_interval_max_s': max(regular) if regular else None,
    'regular_interval_median_s': statistics.median(regular) if regular else None,
    'regular_max_abs_error_from_0_25_s': max((abs(v - .25) for v in regular), default=None),
    'active_to_active_count': len(active),
    'active_interval_distribution': dict(collections.Counter(round(v, 9) for v in active)),
    'actual_neural_inputs_compared': len(inputs), 'inputs_with_real_human': with_human,
    'missing_startup_robot_state_inputs': missing_state,
    'max_float32_input_error': max(errors) if errors else None,
    'actual_loaded_models': [r for r in rows if r['kind'] == 'actual_model_load'],
    'actual_neural_forwards': dict(sum((collections.Counter(r.get('neural_forwards', {}))
                                      for r in steps), collections.Counter())),
    'scope': 'CONFIG_ONLY owned runtime metadata alignment; original baseline remains about 3.75Hz',
    'safety': 'NOT_EVALUATED',
}
(case / 'ACTUAL_TIMING_AND_INPUT_ANALYSIS.json').write_text(json.dumps(result, indent=2))
print(json.dumps({k: v for k, v in result.items()
                  if k not in ('identity', 'actual_loaded_models', 'actual_file')}, indent=2))
