#!/usr/bin/env python3
"""Read-only reconstruction of sampled boundary entries and prediction coverage."""
from __future__ import annotations

import argparse
from collections import defaultdict, deque
import cProfile
import hashlib
import json
import math
from pathlib import Path
import pstats
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'isaac_sim/scripts'))
from convert_gazebo_boxes_to_usda import load_static_boxes
from generate_free_space_people_config import FreeSpaceMap


def load_guard():
    boxes, _ = load_static_boxes(ROOT / 'workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world')
    return FreeSpaceMap(ROOT / 'workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml', 0.20, boxes)


def endpoint(position, velocity):
    speed = math.hypot(*velocity)
    # Exact retained runtime settings: response=2*0.35 s, max acceleration=4 m/s2.
    return [position[i] + velocity[i] * (0.70 + speed / 8.0) for i in range(2)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    guard = load_guard()
    source = ROOT / 'runs/pedestrian_boundary_guard/20260908_round2_latched_release/steering.jsonl'
    histories = defaultdict(deque)
    active = {}
    entries = []
    queries = []
    total = heading_only = 0
    samples = []
    for line in source.open():
        record = json.loads(line)
        if record.get('type') != 'sample':
            continue
        t = record['sim_time']
        for person, d in record['people'].items():
            pos = d['position_m']
            actual = d['actual_navigation_velocity_mps']
            speed = d['set_speed_command_mps']
            heading = d['heading_direction']
            norm = math.hypot(*heading)
            hv = [x / norm * max(speed, math.hypot(*actual)) for x in heading] if norm > 1e-12 else actual
            actual_end = endpoint(pos, actual)
            heading_end = endpoint(pos, hv)
            actual_risk = not guard.segment_world_free(*pos, *actual_end)
            heading_risk = not guard.segment_world_free(*pos, *heading_end)
            safe = guard.contains_world(*pos)
            row = dict(t=t, person=person, position=pos, actual=actual, heading=heading,
                       adapter=d['isaac_adapter_output_velocity_mps'], speed=speed,
                       applied_target=d['applied_target_m'], decision=d['free_space_decision'],
                       cursor=d['route_cursor_target_index'], actual_end=actual_end, heading_end=heading_end,
                       actual_risk=actual_risk, heading_risk=heading_risk, safe=safe,
                       recorded_prediction=d['predictive_boundary_raw_prediction_constrained'])
            h = histories[person]
            while h and t - h[0]['t'] > 1.0 + 1e-9:
                h.popleft()
            if not safe and not active.get(person, False):
                entries.append(dict(entry=row, preceding_1s=list(h)))
            active[person] = not safe
            h.append(row)
            total += 1
            heading_only += bool(safe and heading_risk and not actual_risk)
            # Frozen stratified workload, independent of query result.
            if total % 8 == 0:
                queries.extend([pos + actual_end, pos + heading_end, pos + d['applied_target_m']])
            samples.append(row)
    workload = np.asarray(queries, dtype=np.float64)
    np.save(args.output / 'segment_workload.npy', workload)
    expected = np.asarray([guard.segment_world_free(*q) for q in workload], dtype=bool)
    np.save(args.output / 'segment_expected.npy', expected)
    np.savez_compressed(args.output / 'guard_grid.npz', free=np.asarray(sorted(guard.free)), width=guard.width,
                        height=guard.height, resolution=guard.resolution, origin=[guard.origin_x, guard.origin_y])
    profile = cProfile.Profile()
    profile.enable()
    for q in workload[:3000]:
        guard.segment_world_free(*q)
    profile.disable()
    with (args.output / 'geometry_profile.txt').open('w') as stream:
        pstats.Stats(profile, stream=stream).sort_stats('cumtime').print_stats(20)
    timings = []
    for _ in range(3):
        started = time.perf_counter()
        actual_results = [guard.segment_world_free(*q) for q in workload]
        timings.append(time.perf_counter() - started)
        assert np.array_equal(actual_results, expected)
    summary = dict(schema='turn_boundary_audit/v1', source=str(source),
                   source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   control_person_samples=total, safe_samples_heading_only_risk=heading_only,
                   unsafe_control_samples=sum(not r['safe'] for r in samples),
                   sampled_entries=len(entries), entries=entries,
                   workload_queries=len(workload), baseline_query_wall_sec=timings,
                   settings=dict(clearance=0.20, response_sec=0.70, maximum_acceleration_mps2=4.0),
                   interpretation='Observed trajectory only. Earlier risk detection does not establish counterfactual avoidance or continuous safety.')
    (args.output / 'audit.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (args.output / 'sample_diagnostics.jsonl').open('w') as f:
        for row in samples:
            f.write(json.dumps(row) + '\n')
    print(json.dumps({k:v for k,v in summary.items() if k != 'entries'}), flush=True)
    for e in entries:
        before = e['preceding_1s']
        def lead(key):
            return max((e['entry']['t']-r['t'] for r in before if r[key] and r['safe']), default=None)
        print(json.dumps(dict(t=e['entry']['t'], person=e['entry']['person'].split('/')[3],
                              actual_lead_sec=lead('actual_risk'), heading_lead_sec=lead('heading_risk'))))


if __name__ == '__main__':
    main()
