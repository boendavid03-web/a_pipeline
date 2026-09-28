#!/usr/bin/env python3
"""Extract strict boundary gates from a completed, naturally terminated run."""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path


def analyze(folder):
    final = None
    warnings = 0
    for line in (folder / 'console.log').open(errors='replace'):
        if line.startswith('WAREHOUSE_PEOPLE_ROBOT_RESULT='):
            final = json.loads(line.split('=', 1)[1])
        warnings += line.count('motion matching returned non-finite/non-unit root transform')
    if final is None:
        raise ValueError(f'no final result in {folder}')
    motion = final['pedestrian_social_motion']
    guard = final['pedestrian_free_space_intrusions']
    duration = final['timeline_elapsed_sec']
    sources = Counter()
    previous_t = None
    close_time = 0.0
    sample_count = 0
    maximum_gap = 0.0
    minimum_pair_distance = math.inf
    zero_command_samples = 0
    for line in (folder / 'steering.jsonl').open():
        row = json.loads(line)
        if row.get('type') != 'sample':
            continue
        t = row['sim_time']
        dt = 0 if previous_t is None else t - previous_t
        maximum_gap = max(maximum_gap, dt)
        people = list(row['people'].values())
        distance = min(math.dist(a['position_m'], b['position_m'])
                       for i, a in enumerate(people) for b in people[i + 1:])
        minimum_pair_distance = min(minimum_pair_distance, distance)
        # Same sampled right-endpoint interval estimate used for comparison.
        if distance < .5:
            close_time += dt
        sources.update(p.get('predictive_boundary_source', 'actual_velocity') for p in people)
        zero_command_samples += sum(p['set_speed_command_mps'] <= 1e-12 for p in people)
        previous_t = t
        sample_count += 1
    b004 = next(p for p in final['people_max_displacement_m'] if '/gazebo_b_004/' in p)
    guard_hz = guard['sample_frames'] / duration
    control_hz = motion['update_count'] / duration
    result = dict(folder=str(folder), status=final['status'], exit_reason=final['exit_reason'],
                  duration_sec=duration, people_moving=final['people_moving'],
                  unsafe_person_samples=guard['unsafe_person_samples'],
                  sustained_intrusion_count=guard['sustained_intrusion_count'],
                  active_intrusions=guard['active_intrusions'],
                  estimated_unsafe_person_time_sec=guard['unsafe_person_samples']/guard_hz,
                  guard_hz=guard_hz, control_hz=control_hz, maximum_control_gap_sec=maximum_gap,
                  maximum_sample_jump_m=motion['maximum_sample_displacement_m'],
                  maximum_freeze_sec=max(motion['maximum_freeze_sec_by_person'].values()),
                  close_below_0_5_time_sec=close_time, minimum_pair_distance_m=minimum_pair_distance,
                  b004_displacement_m=final['people_max_displacement_m'][b004],
                  b004_cursor_advance=motion['patrol_cursors'][b004]['advance_count'],
                  active_yielders=final['pedestrian_social_yielding']['active_yielders'],
                  active_latches=motion['predictive_boundary_active_latches'],
                  follow_restart_count=motion['follow_restart_count'],
                  runtime_reset_count=final['pedestrian_runtime_reset_count'],
                  free_space_reset_count=final['pedestrian_free_space_reset_count'],
                  recovery_count=final['pedestrian_free_space_recovery_count'],
                  physics_execution_device=final['physics_execution_device'],
                  physics_steps=final['physics_steps_main_loop'],
                  physics_gpu_dynamics=final['physics_gpu_dynamics'],
                  motion_matching_warnings=warnings, prediction_sources=dict(sources),
                  zero_command_person_samples=zero_command_samples, sampled_control_frames=sample_count,
                  parameters=motion['parameters'], preferred_speeds_mps=motion['preferred_speeds_mps'])
    failures = []
    for key in ('unsafe_person_samples', 'sustained_intrusion_count', 'runtime_reset_count',
                'free_space_reset_count', 'recovery_count', 'follow_restart_count', 'motion_matching_warnings'):
        if result[key] != 0:
            failures.append(key)
    for key in ('active_intrusions', 'active_yielders'):
        if result[key]:
            failures.append(key)
    for key, limit in [('maximum_sample_jump_m', .297), ('maximum_freeze_sec', 1), ('close_below_0_5_time_sec', 2)]:
        if result[key] > limit:
            failures.append(key)
    for key, limit in [('guard_hz', 24.28), ('control_hz', 14.25), ('b004_displacement_m', 15), ('b004_cursor_advance', 100)]:
        if result[key] < limit:
            failures.append(key)
    if result['status'] != 'PASS' or result['exit_reason'] != 'duration_reached' or duration < 180:
        failures.append('natural_duration_completion')
    if result['people_moving'] != 15 or result['physics_execution_device'] != 'cpu' or result['physics_gpu_dynamics']:
        failures.append('workload')
    result['strict_failures'] = failures
    result['strict_pass'] = not failures
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.run)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('parameters', 'preferred_speeds_mps')}, indent=2))


if __name__ == '__main__':
    main()
