#!/usr/bin/env python3
"""Recompute recorded metrics, summarize all repeats, and export research figures."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

from human_motion_execution_metrics import analyze, integrate


def load(path):
    return json.loads(path.read_text())


def extent(values):
    values = [v for v in values if v is not None]
    return dict(mean=statistics.mean(values), min=min(values), max=max(values)) if values else None


def equivalent(left, right):
    """Kit Python 3.12 and system Python use different floating summation paths.

    Compare numerical metrics at 1e-10 SI tolerance; statuses and coverage stay
    exact. This is several orders below the stored experimental precision.
    """
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(equivalent(left[k], right[k]) for k in left)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(equivalent(a, b) for a, b in zip(left, right))
    if type(left) is float and type(right) is float:
        return math.isclose(left, right, rel_tol=1e-10, abs_tol=1e-10)
    return left == right


def sha_check(folder):
    path = folder / 'SHA256SUMS'
    if not path.exists():
        return False
    for line in path.read_text().splitlines():
        expected, name = line.split('  ', 1)
        if hashlib.sha256((folder / name).read_bytes()).hexdigest() != expected:
            return False
    return True


def causal_velocity(rows, window=0.2):
    result = []
    for row in rows:
        end = row['t1']
        start = max(0.0, end - window)
        selected = [r for r in rows if r['t1'] > start + 1e-9 and r['t0'] < end - 1e-9]
        displacement = [0.0, 0.0]
        for r in selected:
            weight = (min(end, r['t1']) - max(start, r['t0'])) / (r['t1'] - r['t0'])
            for axis in range(2):
                displacement[axis] += weight * (r['position1'][axis] - r['position0'][axis])
        result.append([x / (end - start) for x in displacement])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='prepared protocol directory')
    args = parser.parse_args()
    base = args.output.resolve()
    protocol = load(base / 'protocol.json')
    analysis = dict(protocol=str(base / 'protocol.json'), conditions={}, runs=[],
                    offline_metric_comparison=dict(float_abs_tol=1e-10, float_rel_tol=1e-10, statuses='exact'),
                    repeat_interpretation='Three clean processes with the same seed and asset; no population inference.')
    figures = {}
    keys = ['vector_rmse_mps', 'filtered_vector_rmse_mps', 'scheduled_input_rmse_mps',
            'final_deviation_m', 'max_deviation_m', 'distance_m', 'max_freeze_sec']
    for case in protocol['cases']:
        metrics = []
        for repeat in range(1, protocol['repeats'] + 1):
            directory = base / f'{case}_r{repeat}'
            if not (directory / 'manifest.json').exists():
                continue
            manifest = load(directory / 'manifest.json')
            path = directory / 'runtime/metrics.json'
            m = load(path) if path.exists() else {}
            rows = [load_line for load_line in (json.loads(s) for s in (directory / 'runtime/trace.jsonl').open())
                    if load_line['type'] == 'interval'] if (directory / 'runtime/trace.jsonl').exists() else []
            verification = None
            if m.get('samples'):
                recalculated = analyze(rows, m['spec']['schedule'], m['spec']['duration'],
                                       reset_pass=m.get('reset_pass', False), synthetic=m['spec']['synthetic'])
                verification = all(equivalent(m.get(k), v) for k, v in recalculated.items())
            applied_commands = [json.loads(s) for s in (directory / 'runtime/trace.jsonl').open()
                                if '"type": "command"' in s] if rows else []
            # Only the FIRST application of each source sample estimates delivery delay.
            first = {}
            for c in applied_commands:
                first.setdefault(c['index'], c['t'] - c['source_t'])
            extra = dict(command_delivery_delay_sec=extent(list(first.values())),
                         max_planner_excess_length_m=max((r.get('navmesh_probe', {}).get('planner_excess_length_m') or 0 for r in rows), default=None),
                         max_navmesh_projection_error_m=max((r.get('navmesh_probe', {}).get('max_projection_error_m') or 0 for r in rows), default=None),
                         facing_final_deg=math.degrees(math.atan2(rows[-1]['facing'][1], rows[-1]['facing'][0])) if rows else None,
                         reported_velocity_all_zero=all(math.hypot(*r['reported_velocity'][:2]) < 1e-9 for r in rows) if rows else None)
            entry = dict(case=case, repeat=repeat, status=manifest['status'],
                         warnings=manifest.get('motion_matching_warnings'),
                         process_exit_code=manifest.get('exit_code'),
                         sha256_pass=sha_check(directory), recomputed_metrics_match=verification,
                         phases=m.get('phases'), tracking_failures=m.get('tracking_failures'),
                         integrity_errors=m.get('integrity_errors'), actor_path=m.get('actor_path'),
                         reset_error_m=m.get('reset_error_m'), **extra,
                         **{k: m.get(k) for k in keys})
            analysis['runs'].append(entry)
            metrics.append(entry)
            if repeat == 1 and verification:
                figures[case] = (rows, m['spec'], causal_velocity(rows))
        analysis['conditions'][case] = dict(completed=len(metrics), planned=protocol['repeats'],
                                           statuses=[m['status'] for m in metrics],
                                           **{k: extent([m[k] for m in metrics]) for k in keys})
    replay = load(base / 'replay_commands.json')
    square_sum = sum(((replay[i+1]['t'] if i+1 < len(replay) else 6.0) - r['t']) * math.dist(r['raw'], r['adapter'])**2 for i, r in enumerate(replay))
    analysis['historical_raw_to_adapter_vector_rmse_mps'] = math.sqrt(square_sum / 6.0)
    analysis['all_run_sha256_pass'] = all(r['sha256_pass'] for r in analysis['runs'])
    analysis['all_completed_metrics_recomputed'] = all(r['recomputed_metrics_match'] is True for r in analysis['runs'])
    analysis['all_planned_runs_completed'] = len(analysis['runs']) == protocol['repeats'] * len(protocol['cases'])
    (base / 'analysis.json').write_text(json.dumps(analysis, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not figures:
        raise RuntimeError('No complete, verified traces available to plot; analysis.json preserved')
    fig, axes = plt.subplots(len(figures), 2, figsize=(12, 2.4 * len(figures)), squeeze=False, constrained_layout=True)
    for index, (case, (rows, spec, velocities)) in enumerate(figures.items()):
        ax, path_ax = axes[index]
        times = [r['t1'] for r in rows]
        for component, color in [(0, '#2463a5'), (1, '#dd741c')]:
            ax.step([0] + times, [rows[0]['command'][component]] + [r['command'][component] for r in rows],
                    where='pre', color=color, ls='--', alpha=0.7, label=f'command v{"xy"[component]}')
            ax.plot(times, [v[component] for v in velocities], color=color, label=f'live v{"xy"[component]} (0.2 s)')
        ax.set(title=case, xlabel='Simulation time (s)', ylabel='Velocity (m/s)')
        ax.grid(alpha=0.2)
        if index == 0:
            ax.legend(fontsize=7, ncol=2)
        start = rows[0]['position0'][:2]
        actual = [start] + [r['position1'][:2] for r in rows]
        ideal = [[start[a] + d[a] for a in range(2)] for d in [integrate(spec['schedule'], 0, t) for t in [0] + times]]
        path_ax.plot([p[0] for p in ideal], [p[1] for p in ideal], '--', color='#888888', label='velocity integral')
        path_ax.plot([p[0] for p in actual], [p[1] for p in actual], color='#2463a5', label='live root')
        path_ax.scatter(*start, color='black', s=12)
        path_ax.set(xlabel='World X (m)', ylabel='World Y (m)')
        path_ax.axis('equal')
        path_ax.grid(alpha=0.2)
        if index == 0:
            path_ax.legend(fontsize=7)
    fig.suptitle('Human motion execution benchmark — preselected repeat 1\nSame seed 7 / asset; headless app 60 Hz; no ROS, robot, or live Social Force', fontsize=12)
    fig.savefig(base / 'velocity_and_trajectory.png', dpi=160)
    fig.savefig(base / 'velocity_and_trajectory.pdf')
    plt.close(fig)
    lines = ['# Human Motion Execution — measured results', '',
             'Three independent processes per condition, identical seed/asset. RMSE is a time-weighted planar vector error.', '',
             '| Experiment | Input | Result | Error | Conclusion |', '|---|---|---|---|---|']
    for case, c in analysis['conditions'].items():
        spec = load(base / 'specs' / f'{case}.json')
        signal = 'historical ' + case.removeprefix('replay_') if not spec['synthetic'] else ' → '.join(str(s['velocity']) for s in spec['schedule'])
        errors = c['filtered_vector_rmse_mps']
        error_text = 'unavailable' if errors is None else f"filtered RMSE {errors['mean']:.3f} [{errors['min']:.3f}, {errors['max']:.3f}] m/s"
        status = ', '.join(c['statuses'])
        failures = sorted({f for r in analysis['runs'] if r['case'] == case for f in (r['tracking_failures'] or [])})
        conclusion = ', '.join(failures) or ('within declared gates' if c['completed'] == c['planned'] else 'incomplete')
        lines.append(f'| {case} | {signal} | {status} | {error_text} | {conclusion} |')
    lines += ['', '![Preselected repeat 1](velocity_and_trajectory.png)', '',
              'The raw replay is a historical stress sequence from an invalid head-on initialization. It does not validate a human interaction or reproduce its full execution context.',
              'Raw-to-adapter replay RMS difference: %.6f m/s. Paired replays have different inputs and references; their errors cannot be subtracted to estimate a causal adapter contribution.' % analysis['historical_raw_to_adapter_vector_rmse_mps'],
              '', 'Full per-run metrics, gates, phases, and artifact verification: [analysis.json](analysis.json).']
    (base / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print(json.dumps({k: analysis[k] for k in ['all_run_sha256_pass', 'all_completed_metrics_recomputed', 'all_planned_runs_completed']}))


if __name__ == '__main__':
    main()
