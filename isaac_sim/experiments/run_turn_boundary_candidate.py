#!/usr/bin/env python3
"""Launch one isolated 180 s candidate through the existing production lock."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir', required=True, type=Path)
    p.add_argument('--config', required=True, type=Path)
    p.add_argument('--source-manifest', required=True, type=Path,
                   help='Recorded candidate manifest; refuse to run a restored or changed implementation.')
    p.add_argument('--prediction', choices=['actual', 'heading'], default='heading')
    args = p.parse_args()
    frozen = json.loads(args.source_manifest.read_text())['frozen_sha256']
    mismatches = []
    checked = 0
    for filename, expected in frozen.items():
        source_path = Path(filename)
        if source_path.parent == ROOT / 'isaac_sim/scripts':
            checked += 1
            if not source_path.exists() or hashlib.sha256(source_path.read_bytes()).hexdigest() != expected:
                mismatches.append(filename)
    if checked < 6 or mismatches:
        raise RuntimeError(f'Candidate source does not match the recorded experiment; no Isaac launch: {mismatches}')
    out = args.run_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    selected = dict(ISAAC_SCENE='custom', ISAAC_PEDESTRIAN_COUNT='15', ISAAC_PEDESTRIAN_SEED='7',
                    ISAAC_PEDESTRIAN_SPEED='1.0', ISAAC_PEDESTRIAN_FIXED_SPEED='0',
                    ISAAC_PEDESTRIAN_ENSURE_ALL_DIRECTIONS='0', ISAAC_PEDESTRIAN_AVOIDANCE_MODE='off',
                    ISAAC_PEDESTRIAN_SOCIAL_MODE='gazebo_social', ISAAC_PHYSX_GPU_DYNAMICS='0',
                    ISAAC_LIDAR_MODE='physx', ROS_DOMAIN_ID='78', ISAAC_ROS_DOMAIN_ID='78',
                    ISAAC_PEDESTRIAN_BOUNDARY_PREDICTION=args.prediction,
                    ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG=str(args.config.resolve()),
                    ISAAC_PEDESTRIAN_SOCIAL_TRACE_PATH=str(out / 'steering.jsonl'))
    env = os.environ.copy()
    env.update(selected)
    command = ['bash', str(ROOT / 'isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh'),
               '--headless', '--no-ros', '--duration', '180']
    source = out / 'source'
    source.mkdir()
    paths = [ROOT / 'isaac_sim/scripts' / name for name in (
        'pedestrian_social.py', 'pedestrian_steering.py', 'show_warehouse_people_robot_6_0.py',
        'generate_free_space_people_config.py', 'people_route_geometry.py',
        'run_isaac_6_0_warehouse_people_robot.sh')]
    paths += [Path(__file__), args.config.resolve()]
    hashes = {}
    for path in paths:
        (source / path.name).write_bytes(path.read_bytes())
        hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = dict(command=command, cwd=str(ROOT), selected_environment=selected,
                    source_manifest=str(args.source_manifest.resolve()),
                    frozen_sha256=hashes, exit_code=None, status='INCOMPLETE', max_wall_sec=480)
    def save():
        (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    save()
    started = time.monotonic()
    with (out / 'console.log').open('w') as log:
        child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        manifest['launcher_pid'] = child.pid
        save()
        try:
            code = child.wait(timeout=480)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            code = 124 if isinstance(exc, subprocess.TimeoutExpired) else 130
    manifest.update(exit_code=code, status='PROCESS_COMPLETED' if code == 0 else 'PROCESS_FAILED',
                    wall_elapsed_sec=time.monotonic()-started)
    save()
    print(json.dumps(manifest), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
