#!/usr/bin/env python3
"""Prepare immutable inputs or run one bounded, locked, clean-process experiment."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from human_motion_execution_metrics import THRESHOLDS, validate_schedule


ROOT = Path(__file__).resolve().parents[2]
CASES = ['straight', 'lateral', 'direction_switch', 'stop_go', 'replay_raw', 'replay_adapter', 'direction_switch_4hz']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def prepare(output):
    output.mkdir(parents=True, exist_ok=True)
    protocol_path = output / 'protocol.json'
    if protocol_path.exists():
        raise FileExistsError('protocol already exists; never overwrite')
    source = ROOT / 'runs/pedestrian_validation_matrix/evaluation/head_on/pedestrian_social_steering.jsonl'
    data = [json.loads(line) for line in source.open()]
    samples = [r for r in data if r.get('type') == 'sample']
    person = sorted(samples[0]['people'])[0]
    origin = samples[0]['sim_time']
    replay = []
    for row in samples:
        t = row['sim_time'] - origin
        if t >= 6.0:
            break
        d = row['people'][person]
        replay.append(dict(t=t, source_sim_time=row['sim_time'],
                           raw=d['gazebo_raw_velocity_mps'], adapter=d['isaac_adapter_output_velocity_mps'],
                           solver_dt=d['gazebo_solver_dt_sec'], original_position=d['position_m']))
    if samples[-1]['sim_time'] - origin < 6.0:
        raise ValueError('insufficient replay coverage')
    frozen_source = output / 'replay_source.jsonl'
    frozen_source.write_bytes(source.read_bytes())
    write_json(output / 'replay_commands.json', replay)
    schedules = dict(straight=([(0, [1, 0])], 4.0), lateral=([(0, [0, 1])], 3.0),
                     direction_switch=([(0, [1, 0]), (2, [0, 1])], 5.0),
                     stop_go=([(0, [1, 0]), (2, [0, 0]), (4, [1, 0])], 6.0))
    specs = output / 'specs'
    specs.mkdir(exist_ok=False)
    for case in CASES:
        synthetic = not case.startswith('replay_')
        if synthetic:
            entries, duration = schedules[case.removesuffix('_4hz')]
            schedule = [dict(t=t, velocity=v) for t, v in entries]
        else:
            key = case.removeprefix('replay_')
            schedule = [dict(t=r['t'], velocity=r[key]) for r in replay]
            duration = 6.0
        validate_schedule(schedule)
        spec = dict(case=case, duration=duration, schedule=schedule, synthetic=synthetic,
                    config=str(ROOT / 'runs/pedestrian_validation_matrix/configs/single.yaml'),
                    scene=str(ROOT / 'isaac_sim/scenes/a_pipeline_eng_lobby.usda'),
                    anchor=[2.0, 4.0, 0.0], initial_facing=[1.0, 0.0, 0.0],
                    lookahead_m=1.0, target_min_shift_m=0.02,
                    control_period_sec=0.25 if case.endswith('_4hz') else 1 / 60,
                    seed=7, replay_source=str(source) if not synthetic else None,
                    replay_source_sha256=digest(source) if not synthetic else None,
                    replay_person=person if not synthetic else None,
                    replay_source_start_sim_time=origin if not synthetic else None)
        write_json(specs / f'{case}.json', spec)
    code = [Path(__file__), Path(__file__).with_name('behavior_agent_velocity_benchmark.py'),
            Path(__file__).with_name('human_motion_execution_metrics.py'),
            Path(__file__).with_name('execution_audit_metrics.py'), ROOT / 'isaac_sim/scripts/pedestrian_steering.py']
    source_dir = output / 'source'
    source_dir.mkdir(exist_ok=True)
    for path in code:
        (source_dir / path.name).write_bytes(path.read_bytes())
    inputs = code + sorted(specs.glob('*.json')) + [frozen_source, output / 'replay_commands.json',
             ROOT / 'runs/pedestrian_validation_matrix/configs/single.yaml', ROOT / 'isaac_sim/scenes/a_pipeline_eng_lobby.usda']
    protocol = dict(schema='human_motion_execution_protocol/v1', cases=CASES, repeats=3,
                    git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    thresholds=THRESHOLDS, frozen_sha256={str(p): digest(p) for p in inputs},
                    max_wall_sec_per_run=240, no_runtime_parameter_search=True,
                    interpretation='Replay is an open-loop historical stress input from a failed initialization; not a valid head-on re-enactment. Repeats share seed and asset, not population samples.',
                    latency='0.2 s causal pose-velocity window; vector tolerance sustained 0.2 s. Command written before observed interval.',
                    stop_rule='Do not retry ERROR/INVALID configurations; complete independent planned conditions. No production fixes.')
    write_json(protocol_path, protocol)
    print(json.dumps(dict(prepared=str(protocol_path), cases=CASES, repeats=3)))


def run(output, case, repeat):
    protocol = json.loads((output / 'protocol.json').read_text())
    for path, expected in protocol['frozen_sha256'].items():
        if digest(Path(path)) != expected:
            raise RuntimeError(f'frozen input changed: {path}')
    isaac = Path(os.environ.get('ISAAC_SIM_6_ROOT', ROOT / 'isaac_sim/isaacsim-6.0.1'))
    lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'warehouse_people_robot_6_0.{os.getuid()}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Isaac launcher lock is occupied; existing process untouched') from exc
        destination = output / f'{case}_r{repeat}'
        destination.mkdir(exist_ok=False)
        spec = output / 'specs' / f'{case}.json'
        command = [str(isaac / 'python.sh'), str(Path(__file__).with_name('behavior_agent_velocity_benchmark.py')),
                   '--spec', str(spec), '--output', str(destination / 'runtime')]
        manifest = dict(command=command, cwd=str(ROOT), protocol_sha256=digest(output / 'protocol.json'),
                        repeat=repeat, case=case, exit_code=None, status='INCOMPLETE')
        write_json(destination / 'manifest.json', manifest)
        env = os.environ.copy()
        env['ISAACSIM_ASSET_ROOT'] = env.get('ISAACSIM_ASSET_ROOT', str(ROOT / 'isaac_sim/assets-6.0.1/Assets/Isaac/6.0'))
        env['OMNI_KIT_DISABLE_TELEMETRY'] = '1'
        started = time.monotonic()
        log_path = destination / 'isaac.log'
        with log_path.open('w') as log:
            child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait(timeout=protocol['max_wall_sec_per_run'])
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                code = 124 if isinstance(exc, subprocess.TimeoutExpired) else 130
        metrics_path = destination / 'runtime/metrics.json'
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
        warnings = log_path.read_text(errors='replace').count('motion matching returned non-finite/non-unit root transform')
        status = metrics.get('status', 'INCOMPLETE')
        if warnings:
            status = 'FAIL_MOTION_MATCHING_WARNING'
        elif code not in (0, 2):
            status = 'FAIL_PROCESS_EXIT'
        elif code == 2 and status == 'PASS':
            status = 'FAIL_PROCESS_EXIT'
        manifest.update(exit_code=code, status=status, motion_matching_warnings=warnings,
                        wall_elapsed_sec=time.monotonic() - started)
        write_json(destination / 'manifest.json', manifest)
        artifacts = sorted(p for p in destination.rglob('*') if p.is_file())
        (destination / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(destination)}\n' for p in artifacts))
        print(json.dumps(dict(manifest, vector_rmse_mps=metrics.get('vector_rmse_mps'),
                              max_deviation_m=metrics.get('max_deviation_m'), error=metrics.get('error'))), flush=True)
        return 0 if status == 'PASS' else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--case', choices=CASES)
    p.add_argument('--repeat', type=int, choices=[1, 2, 3], default=1)
    args = p.parse_args()
    output = args.output.resolve()
    if args.prepare:
        prepare(output)
        return 0
    if not args.case:
        p.error('--case required for execution')
    return run(output, args.case, args.repeat)


if __name__ == '__main__':
    sys.exit(main())
