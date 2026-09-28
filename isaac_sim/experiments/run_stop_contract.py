#!/usr/bin/env python3
"""Bounded single-factor stop tests using the existing locked Isaac runner."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from human_motion_execution_metrics import THRESHOLDS
from run_human_motion_execution import ROOT, digest, run, write_json
from stop_contract_policy import MODES


def prepare(output):
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'protocol.json').exists():
        raise FileExistsError('protocol already frozen')
    reference = ROOT / 'runs/human_motion_execution/20260908_101151/validated_geometry_v2/specs/stop_go.json'
    base = json.loads(reference.read_text())
    specs = output / 'specs'
    specs.mkdir(exist_ok=False)
    for mode in MODES:
        write_json(specs / f'{mode}.json', dict(base, case=mode, stop_mode=mode))
    sources = [Path(__file__), *[Path(__file__).with_name(name) for name in (
        'behavior_agent_velocity_benchmark.py', 'stop_contract_policy.py',
        'run_human_motion_execution.py', 'human_motion_execution_metrics.py', 'execution_audit_metrics.py')],
        ROOT / 'isaac_sim/scripts/pedestrian_steering.py', Path(base['config']), Path(base['scene'])]
    source_dir = output / 'source'
    source_dir.mkdir(exist_ok=False)
    for path in sources:
        (source_dir / path.name).write_bytes(path.read_bytes())
    write_json(output / 'protocol.json', dict(
        schema='stop_contract_protocol/v1', cases=list(MODES), repeats=3,
        frozen_sha256={str(p): digest(p) for p in sources + list(specs.glob('*.json'))},
        max_wall_sec_per_run=240, thresholds=THRESHOLDS,
        prior_reference=str(reference), prior_reference_sha256=digest(reference),
        primary_gates=dict(stop_distance_m=0.30, stop_response_sec=1.0, restart_response_sec=1.0,
                           max_unintentional_freeze_sec=1.0, runtime_resets=0),
        full_tracking_gates='Unmodified historical metrics remain reported, including startup and path error.',
        task_contract='continuous/fixed: one follow. idle: initial follow, one idle at stop edge, one follow at restart edge; every observed active task must match its issued epoch.',
        interpretation='One actor is an isolated interface diagnosis, not crowd workload reduction or boundary acceptance.',
        stop_rule='Three predetermined mechanisms; no parameter search. Abort infrastructure/pose failures. Replicate valid conditions in independent processes; no crowd runs until a stop/restart candidate passes.'
    ))
    print(output / 'protocol.json')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--case', choices=MODES)
    p.add_argument('--repeat', type=int, choices=(1, 2, 3), default=1)
    args = p.parse_args()
    if args.prepare:
        prepare(args.output.resolve())
        return 0
    if not args.case:
        p.error('--case required')
    return run(args.output.resolve(), args.case, args.repeat)


if __name__ == '__main__':
    raise SystemExit(main())
