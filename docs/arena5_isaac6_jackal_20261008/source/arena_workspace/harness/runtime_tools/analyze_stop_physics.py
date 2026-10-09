"""Offline comparison of the original hold window across actual runtime layers."""
import collections
import csv
import json
import math
import pathlib
import sys

case = pathlib.Path(sys.argv[1])
data = json.loads((case / 'PROBE_RESULT.json').read_text())
trace = pathlib.Path('[LOCAL_PATH]') / case.name
physics_file = trace / 'physics.jsonl'
physics = [json.loads(line) for line in physics_file.read_text().splitlines()] if physics_file.exists() else []
result = {'case': case.name, 'physics_file': str(physics_file),
          'physics_kinds': dict(collections.Counter(r['kind'] for r in physics)),
          'actual_parameters': data.get('actual_parameters'),
          'actual_endpoints': data.get('actual_endpoints'), 'trials': [],
          'acceptance': 'Original cmd_vel sample requirement unchanged; downstream diagnostics do not replace it',
          'contact': 'NOT_MEASURED', 'safety': 'NOT_EVALUATED'}
csv_rows = []
for trial in data['trials']:
    hold = trial.get('hold')
    if not hold:
        continue
    start, end = hold['wall_start'], hold['wall_end']
    within = lambda r: start <= r['wall'] <= end
    record = {'phase': trial['phase'], 'original_hold': hold, 'actual_topics': {}}
    for key in ('cmd', 'cmd_channels', 'controller_cmd', 'joints', 'odom'):
        topics = collections.defaultdict(list)
        for row in data['rows'].get(key, []):
            if within(row):
                topics[row['topic']].append(row)
        for topic, rows in topics.items():
            values = []
            for row in rows:
                msg = row['msg']
                if key == 'joints':
                    values.extend(abs(v) for v in msg['velocity'])
                else:
                    if key == 'controller_cmd':
                        msg = msg['twist']
                    elif key == 'odom':
                        msg = msg['twist']['twist']
                    values.extend(abs(msg[part][axis]) for part in ('linear', 'angular') for axis in ('x', 'y', 'z'))
            record['actual_topics'][topic] = {
                'layer': key, 'samples': len(rows), 'max_abs_component': max(values) if values else None,
                'first_wall_relative_hold_s': rows[0]['wall'] - start,
                'last_wall_relative_hold_s': rows[-1]['wall'] - start,
                'joint_names': rows[0]['msg'].get('name') if key == 'joints' else None,
                'joint_array_lengths_match': all(len(r['msg']['name']) == len(r['msg']['velocity'])
                                                 for r in rows) if key == 'joints' else None,
            }
    bodies = []
    for row in physics:
        if row['kind'] != 'robot_physics' or not within(row):
            continue
        chassis = next((b for b in row['bodies'] if b['path'].endswith('/chassis_link')), None)
        if chassis and chassis['physx_transform'].get('ret_val'):
            bodies.append((row, chassis))
    if bodies:
        first = bodies[0][1]
        p0 = first['physx_transform']['position']
        u0 = first['usd_position']
        drift = lambda a, b: math.dist(a[:2], b[:2])
        record['actual_chassis'] = {
            'samples': len(bodies), 'path': first['path'],
            'physx_drift_m': max(drift(p0, body['physx_transform']['position']) for _, body in bodies),
            'usd_drift_m': max(drift(u0, body['usd_position']) for _, body in bodies),
            'physx_first': bodies[0][1]['physx_transform'], 'physx_last': bodies[-1][1]['physx_transform'],
            'first_wall_relative_hold_s': bodies[0][0]['wall'] - start,
            'last_wall_relative_hold_s': bodies[-1][0]['wall'] - start,
            'usd_velocity_attr_max_mps': max((abs(v) for _, b in bodies for v in b.get('usd_velocity_attr') or []), default=None),
            'note': 'USD/PhysX and TF are asynchronous samples; origin shifts do not change displacement',
        }
        for row, body in bodies:
            pos = body['physx_transform']['position']
            csv_rows.append({'phase': trial['phase'], 'wall_relative_hold_s': row['wall'] - start,
                             'sim_s': row['sim'], 'x_m': pos[0], 'y_m': pos[1], 'z_m': pos[2],
                             'physx_drift_m': drift(p0, pos)})
    else:
        record['actual_chassis'] = {'samples': 0, 'status': 'UNVERIFIED'}
    result['trials'].append(record)
(case / 'STOP_PHYSICS_ANALYSIS.json').write_text(json.dumps(result, indent=2))
if csv_rows:
    with (case / 'STOP_PHYSICS_TIMESERIES.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
print(json.dumps({'case': case.name, 'physics_kinds': result['physics_kinds'],
                  'trials': result['trials']}, indent=2))
