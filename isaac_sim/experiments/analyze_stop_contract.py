#!/usr/bin/env python3
"""Summarize task epochs and live-root stop/restart evidence, without refitting."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from run_human_motion_execution import digest, write_json


def analyze_run(folder, gates):
    manifest = json.loads((folder / 'manifest.json').read_text())
    metrics = json.loads((folder / 'runtime/metrics.json').read_text())
    trace = [json.loads(line) for line in (folder / 'runtime/trace.jsonl').open()]
    intervals = [r for r in trace if r['type'] == 'interval']
    commands = [r for r in trace if r['type'] == 'command']
    stopped = [r for r in intervals if math.hypot(*r['command']) <= 1e-12]
    stop_commands = [r for r in commands if math.hypot(*r['velocity']) <= 1e-12]
    phases = metrics.get('phases', [])
    stop = next((p for p in phases if math.hypot(*p['velocity']) == 0), {})
    restart = next((p for p in phases if p['t'] > stop.get('t', math.inf) and math.hypot(*p['velocity']) > 0), {})
    mode = metrics['stop_mode']
    expected = [('idle', 2.0), ('follow', 4.0)] if mode == 'idle_on_stop' else []
    events = metrics['task_events']
    tasks_ok = (len(events) == len(expected) and
                all(e['action'] == a and abs(e['t'] - t) < 1e-7 for e, (a, t) in zip(events, expected)) and
                metrics['follow_calls'] == (2 if mode == 'idle_on_stop' else 1) and
                metrics['idle_calls'] == (1 if mode == 'idle_on_stop' else 0))
    failures = []
    checks = [('stop_distance_m', stop.get('stop_distance_m'), gates['stop_distance_m']),
              ('stop_response_sec', stop.get('response_sec'), gates['stop_response_sec']),
              ('restart_response_sec', restart.get('response_sec'), gates['restart_response_sec']),
              ('freeze_sec', metrics.get('max_freeze_sec'), gates['max_unintentional_freeze_sec'])]
    for name, value, limit in checks:
        if value is None or value > limit + 1e-8:
            failures.append(name)
    integrity_ok = (metrics.get('motion_integrity_pass') is True and tasks_ok and
                    metrics['reset_calls'] == 1 and manifest['motion_matching_warnings'] == 0 and
                    manifest['exit_code'] in (0, 2) and manifest['status'] in ('PASS', 'FAIL_TRACKING'))
    if not integrity_ok:
        failures.append('integrity')
    result = dict(run=folder.name, mode=mode, primary_contract_pass=not failures,
                  primary_failures=failures, full_tracking_status=metrics['status'],
                  full_tracking_failures=metrics.get('tracking_failures'),
                  integrity_pass=integrity_ok, task_epoch_contract_pass=tasks_ok,
                  reset_calls=metrics['reset_calls'], follow_calls=metrics['follow_calls'], idle_calls=metrics['idle_calls'],
                  stop_distance_m=stop.get('stop_distance_m'), stop_response_sec=stop.get('response_sec'),
                  restart_response_sec=restart.get('response_sec'), restart_onset_sec=restart.get('onset_sec'),
                  max_freeze_sec=metrics.get('max_freeze_sec'), max_deviation_m=metrics.get('max_deviation_m'),
                  max_sample_gap_sec=metrics.get('max_sample_gap_sec'),
                  max_sample_jump_m=max(math.dist(r['position0'][:2], r['position1'][:2]) for r in intervals),
                  stop_command_target_writes=sum(r['target_written'] for r in stop_commands),
                  stop_speed_readback_after_write=sorted(set(r['reported_speed_after_write'] for r in stop_commands)),
                  stop_speed_readback_after_update=sorted(set(r['reported_speed'] for r in stopped)),
                  actor_path=metrics.get('actor_path'), observed_duration_sec=metrics.get('observed_duration_sec'))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    protocol = json.loads((args.output / 'protocol.json').read_text())
    rows = [analyze_run(f.parent, protocol['primary_gates']) for f in sorted(args.output.glob('*_r*/manifest.json'))]
    result = dict(schema='stop_contract_analysis/v1', runs=rows,
                  protocol_sha256=digest(args.output / 'protocol.json'),
                  completeness={mode: sum(r['mode'] == mode for r in rows) for mode in protocol['cases']})
    write_json(args.output / 'analysis.json', result)
    for row in rows:
        print(json.dumps(row))


if __name__ == '__main__':
    main()
