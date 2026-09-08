#!/usr/bin/env python3
"""Run exactly one bounded audit in a new directory, with provenance and locking."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    root = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--config', type=Path, default=root / 'runs/pedestrian_validation_matrix/configs/single.yaml')
    p.add_argument('--method', choices=['move_to', 'follow_static', 'follow_moving', 'follow_stable'], default='move_to')
    p.add_argument('--ownership', choices=['before_reset', 'after_reset'], default='before_reset')
    p.add_argument('--reset-mode', choices=['playing', 'paused'], default='playing')
    p.add_argument('--duration', type=float, default=15.0)
    p.add_argument('--target-period-sec', type=float, default=0.0)
    args = p.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error('duration must be finite and positive')
    if not math.isfinite(args.target_period_sec) or args.target_period_sec < 0:
        p.error('target period must be finite and nonnegative')
    isaac = Path(os.environ.get('ISAAC_SIM_6_ROOT', root / 'isaac_sim/isaacsim-6.0.1'))
    scene = root / 'isaac_sim/scenes/a_pipeline_eng_lobby.usda'
    script = Path(__file__).with_name('behavior_agent_execution_audit.py')
    for path in (isaac / 'python.sh', args.config, scene, script):
        if not path.is_file():
            p.error(f'missing input: {path}')
    # Share the production launcher lock without terminating any existing job.
    lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'warehouse_people_robot_6_0.{os.getuid()}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            p.error(f'Isaac is already running: {lock_path}')
        args.output = args.output.resolve()
        args.output.mkdir(parents=True, exist_ok=False)
        command = [str(isaac / 'python.sh'), str(script), '--headless', '--config', str(args.config.resolve()),
                   '--scene', str(scene), '--output', str(args.output / 'runtime'),
                   '--method', args.method, '--ownership', args.ownership,
                   '--reset-mode', args.reset_mode, '--duration', str(args.duration),
                   '--target-period-sec', str(args.target_period_sec)]
        files = [script, Path(__file__).resolve(), script.with_name('execution_audit_metrics.py'), args.config.resolve(), scene]
        manifest = dict(command=command, cwd=str(root), git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                        source_sha256={str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
                        wall_timeout_sec=360, exit_code=None)
        (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        env = os.environ.copy()
        env['ISAACSIM_ASSET_ROOT'] = env.get('ISAACSIM_ASSET_ROOT', str(root / 'isaac_sim/assets-6.0.1/Assets/Isaac/6.0'))
        env['OMNI_KIT_DISABLE_TELEMETRY'] = '1'
        log_path = args.output / 'isaac.log'
        started = time.monotonic()
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=360)
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                # Only the process group created by this runner is terminated.
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                code = 124 if isinstance(exc, subprocess.TimeoutExpired) else 130
        log_text = log_path.read_text(errors='replace')
        warning_count = log_text.count('motion matching returned non-finite/non-unit root transform')
        manifest.update(exit_code=code, wall_elapsed_sec=time.monotonic() - started,
                        motion_matching_warning_count=warning_count)
        metrics_path = args.output / 'runtime/metrics.json'
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
        manifest['decision'] = metrics.get('status', 'INCOMPLETE')
        if code != 0 and manifest['decision'] == 'PASS':
            manifest['decision'] = 'FAIL_PROCESS_EXIT'
        if warning_count:
            manifest['decision'] = 'FAIL_MOTION_MATCHING_WARNING'
        (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        artifacts = sorted(f for f in args.output.rglob('*') if f.is_file() and f.name != 'SHA256SUMS')
        (args.output / 'SHA256SUMS').write_text(''.join(f'{hashlib.sha256(f.read_bytes()).hexdigest()}  {f.relative_to(args.output)}\n' for f in artifacts))
        print(json.dumps(manifest, indent=2))
        return 0 if manifest['decision'] == 'PASS' and code == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
