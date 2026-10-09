"""Read-only DS-RNN per-tick parent observation trace for one guarded case."""
import json
import pathlib
import time

import numpy as np


class Recorder:
    def __init__(self, path):
        self._stream = pathlib.Path(path).open('x', buffering=1)

    def record(self, seq, sim_time, features):
        plan = features.get('global_plan')
        path = np.asarray(plan, dtype=float).tolist() if plan is not None else None
        pose = features.get('robot_pose')
        goal = features.get('goal_pose')
        grid = features.get('costmap')
        costmap = None
        if grid is not None:
            def value(xy):
                if xy is None:
                    return None
                col = int((float(xy[0]) - grid.origin_x) / grid.resolution)
                row = int((float(xy[1]) - grid.origin_y) / grid.resolution)
                if 0 <= row < grid.data.shape[0] and 0 <= col < grid.data.shape[1]:
                    return int(grid.data[row, col])
                return None
            chord = []
            if pose is not None and goal is not None:
                for fraction in np.linspace(0.0, 1.0, 21):
                    xy = (float(pose[0]) + fraction * (float(goal[0]) - float(pose[0])),
                          float(pose[1]) + fraction * (float(goal[1]) - float(pose[1])))
                    chord.append(value(xy))
            costmap = {'shape': list(grid.data.shape), 'resolution': grid.resolution,
                       'origin_xy': [grid.origin_x, grid.origin_y],
                       'robot_cost': value(pose), 'subgoal_cost': value(goal),
                       'chord_costs_21': chord, 'chord_blocked_99_count': sum(v is not None and v >= 99 for v in chord)}
        row = {'seq': seq, 'sim': sim_time, 'wall': time.monotonic(),
               'robot_pose': pose, 'subgoal': goal, 'global_plan': path,
               'plan_points': len(path) if path is not None else 0, 'costmap': costmap}
        self._stream.write(json.dumps(row, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x)) + '\n')
