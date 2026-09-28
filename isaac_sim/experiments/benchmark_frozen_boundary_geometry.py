#!/usr/bin/env python3
"""Recheck archived broadphase against the restored guard without changing it."""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import numpy as np

from audit_turn_boundary import load_guard
from people_route_geometry import edge_is_continuously_safe


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--experiment', required=True, type=Path)
    args = p.parse_args()
    root = args.experiment
    source = root / 'round1_heading_validated/source/people_route_geometry.py'
    spec = importlib.util.spec_from_file_location('archived_boundary_geometry', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    guard = load_guard()
    broadphase = module.StaticBoxBroadphase(guard.static_boxes, guard.clearance)
    workload = np.load(root / 'offline/segment_workload.npy')
    expected = np.load(root / 'offline/segment_expected.npy')
    grid = np.load(root / 'offline/guard_grid.npz')
    assert np.array_equal(np.asarray(sorted(guard.free)), grid['free'])

    def indexed(q):
        return guard._segment_raster_free(*q) and edge_is_continuously_safe(
            tuple(q[:2]), tuple(q[2:]), broadphase.candidates(tuple(q[:2]), tuple(q[2:])),
            guard.bounds, guard.clearance)

    timings = {'original': [], 'archived_index': []}
    for repeat in range(3):
        for name in (('original', 'archived_index') if repeat % 2 == 0 else ('archived_index', 'original')):
            started = time.perf_counter()
            result = np.asarray([guard.segment_world_free(*q) if name == 'original' else indexed(q) for q in workload])
            timings[name].append(time.perf_counter() - started)
            assert np.array_equal(result, expected), name
    result = dict(queries=len(workload),mismatches=0, free_cells_match=True,
                  wall_seconds=timings, production_mutated=False,
                  interpretation='Offline predicate equivalence and query timing only; runtime candidate was rejected.')
    (root / 'offline/geometry_recheck.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
