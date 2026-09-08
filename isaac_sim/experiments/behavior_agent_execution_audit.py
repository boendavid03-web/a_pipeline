#!/usr/bin/env python3
"""Opt-in lifecycle experiment; never imported by production launchers.

No Social Force, robot, ROS, root writes or motion recovery. The sole reset is
episode initialization. Existing IRA scheduler suppression is confined to this
process. Always record both BehaviorAgent live state and authored USD state.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

import yaml
from execution_audit_metrics import json_safe, summarize, valid_pose


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--method', choices=['move_to', 'follow_static', 'follow_moving', 'follow_stable'], default='move_to')
    p.add_argument('--ownership', choices=['before_reset', 'after_reset'], default='before_reset')
    p.add_argument('--reset-mode', choices=['playing', 'paused'], default='playing')
    p.add_argument('--duration', type=float, default=15.0)
    p.add_argument('--target-period-sec', type=float, default=0.0,
                   help='follow_moving sample-and-hold period; 0 updates every app frame')
    p.add_argument('--setup-timeout', type=float, default=180.0)
    p.add_argument('--headless', action='store_true')
    args = p.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error('duration must be finite and positive')
    if not math.isfinite(args.target_period_sec) or args.target_period_sec < 0:
        p.error('target period must be finite and nonnegative')
    return args


def main():
    args = arguments()
    args.output.mkdir(parents=True, exist_ok=False)
    document = yaml.safe_load(args.config.read_text())
    config = document['isaacsim.replicator.agent']
    config['environment']['base_stage_asset_path'] = str(args.scene.resolve(strict=True))
    groups = config['character']['groups']
    if any(int(g['num']) != 1 for g in groups.values()) or len(groups) not in (1, 2):
        raise ValueError('audit requires one or two named groups, one person each')
    routes = {name: g['routines'][0]['patrol']['path_points'] for name, g in groups.items()}
    resolved = args.output / 'resolved.yaml'
    resolved.write_text(yaml.safe_dump(document, sort_keys=False))
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'renderer': 'RaytracedLighting',
                         'multi_gpu': False})
    import carb
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdSkel
    timeline = omni.timeline.get_timeline_interface()
    trace = (args.output / 'trace.jsonl').open('w', buffering=1)
    result = dict(schema='behavior_agent_execution_audit/v1', status='INCOMPLETE',
                  method=args.method, ownership=args.ownership, reset_mode=args.reset_mode,
                  target_period_sec=args.target_period_sec,
                  config_source=str(args.config.resolve()), reset_calls_per_person=1,
                  formal_pipeline=False, social_force=False, physical_contact_truth=False,
                  command_calls_per_person=0, people={})

    def emit(kind, **row):
        trace.write(json.dumps(json_safe(dict(type=kind, **row)), allow_nan=False) + '\n')

    def tick():
        if not app.is_running():
            raise RuntimeError('application closed before completion')
        app.update()

    def detach(runtime):
        if runtime.triggers or runtime._active_triggers:
            raise RuntimeError('unexpected triggers confound single task ownership')
        runtime._refresh_runtime_states_coro()
        runtime._active_routine = None
        runtime.routines.clear()
        runtime._routine_p = []

    try:
        enable_extension('isaacsim.replicator.agent.core')
        for _ in range(5):
            tick()
        from isaacsim.replicator.agent.core import api as ira
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager
        import omni.anim.behavior.core as bh
        import omni.anim.navigation.core as navigation
        if not ira.load_config_file(str(resolved.resolve())):
            raise RuntimeError('IRA config rejected')
        task = asyncio.ensure_future(ira.setup_simulation())
        deadline = time.monotonic() + args.setup_timeout
        while not task.done():
            tick()
            if time.monotonic() > deadline:
                task.cancel()
                raise TimeoutError('IRA setup')
        task.result()
        stage = omni.usd.get_context().get_stage()
        if str(UsdGeom.GetStageUpAxis(stage)) != 'Z' or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-9:
            raise RuntimeError('audit contract requires Z-up metres')
        if navigation.acquire_interface().get_navmesh() is None:
            raise RuntimeError('NavMesh unavailable')
        roots = {str(p.GetPath()): p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Characters')) if p.IsA(UsdSkel.Root)}
        if len(roots) != len(groups):
            raise RuntimeError('SkelRoot cardinality mismatch')
        interface = bh.acquire_interface()
        interface.set_random_seed(int(config.get('seed', 7)))
        timeline.play()
        for _ in range(30):
            tick()
        agents = {path: interface.get_agent(path) for path in roots}
        runtimes = {str(r.prim.GetPath()): r for r in AgentsManager.get_instance().get_agents_by_type(IRA_Character)}
        if any(a is None or path not in runtimes or runtimes[path].get_bh_agent() is None for path, a in agents.items()):
            raise RuntimeError('BehaviorAgent or IRA runtime not ready')
        anchors, goals, targets, task_ids = {}, {}, {}, {}

        def state(path):
            agent = agents[path]
            position = list(agent.get_world_translation())
            rotation = list(agent.get_world_rotation())
            authored = UsdGeom.Xformable(roots[path]).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            return dict(path=path, sim_time=float(timeline.get_current_time()),
                        timeline_playing=timeline.is_playing(), position=position,
                        quaternion=rotation, pose_valid=valid_pose(position, rotation),
                        usd_position=list(authored), reported_velocity=list(agent.get_linear_velocity()),
                        enabled=bool(agent.is_enabled()), active_task_id=int(agent.get_action_task_id()),
                        active_task_name=str(agent.get_task_name(agent.get_action_task_id())))

        for path, agent in agents.items():
            name = path.removeprefix('/World/Characters/').split('/')[0]
            anchors[path] = [float(v) for v in routes[name][0]]
            goals[path] = [float(v) for v in routes[name][1]]
            agent.set_auto_avoidance_enabled(False)
            agent.set_obstacle_avoidance_enabled(False)
            emit('before_reset', **state(path), anchor=anchors[path], goal=goals[path])
            if args.ownership == 'before_reset':
                detach(runtimes[path])
        if args.reset_mode == 'paused':
            timeline.pause()  # Explicit negative control: outside supported API lifecycle.
        for path, agent in agents.items():
            ok = agent.reset(target=carb.Float3(*anchors[path]))
            emit('reset_return', path=path, accepted=bool(ok), timeline_playing=timeline.is_playing())
            if not ok:
                raise RuntimeError('reset returned false')
        if args.reset_mode == 'paused':
            timeline.play()
        reset_samples = {p: [] for p in agents}
        for frame in range(15):
            tick()
            for path in agents:
                row = state(path)
                row['anchor_error_m'] = math.dist(row['position'], anchors[path])
                reset_samples[path].append(row)
                emit('reset_settle', frame=frame, **row)
            if frame == 2 and args.ownership == 'after_reset':
                for runtime in runtimes.values():
                    detach(runtime)
        reset_pass = True
        for path in agents:
            rows = reset_samples[path]
            passed = all(r['pose_valid'] and r['anchor_error_m'] <= 0.15 for r in rows[-3:])
            result['people'][path] = dict(anchor=anchors[path], goal=goals[path], reset_pass=passed,
                                          reset_first_error_m=rows[0]['anchor_error_m'],
                                          reset_final_error_m=rows[-1]['anchor_error_m'])
            reset_pass &= passed
        if not reset_pass:
            result['status'] = 'FAIL_RESET'
            return 2
        for index, (path, agent) in enumerate(agents.items()):
            target_path = f'/World/ExecutionAuditTargets/Target{index}'
            api = UsdGeom.XformCommonAPI(UsdGeom.Xform.Define(stage, target_path).GetPrim())
            api.SetTranslate(Gf.Vec3d(*goals[path]))
            targets[path] = (api, target_path, list(goals[path]))
            if args.method == 'move_to':
                task_id = agent.move_to(target=carb.Float3(*goals[path]), auto_brake=True)
            else:
                task_id = agent.follow(target=target_path, distance=0.0)
            if task_id == bh.BEHAVIOR_TASK_ID_INVALID:
                raise RuntimeError('invalid locomotion task id')
            task_ids[path] = int(task_id)
            agent.set_speed(1.0)
        result['command_calls_per_person'] = 1
        samples = {p: [] for p in agents}
        started = float(timeline.get_current_time())
        last_update = {p: started for p in agents}
        updates = {p: 0 for p in agents}
        deadline = time.monotonic() + max(60.0, args.duration * 5.0)
        while float(timeline.get_current_time()) - started < args.duration:
            tick()
            if time.monotonic() > deadline:
                raise TimeoutError('motion observation wall watchdog')
            for path in agents:
                row = state(path)
                row['issued_task_id'] = task_ids[path]
                row['issued_task_status'] = str(agents[path].get_task_status(task_ids[path]))
                row['task_running'] = bool(agents[path].is_task_running(task_ids[path]))
                row['target'] = list(targets[path][2])
                samples[path].append(row)
                emit('motion', **row)
                if not row['pose_valid']:
                    raise RuntimeError('invalid live runtime pose')
                if args.method in ('follow_moving', 'follow_stable'):
                    position = row['position']
                    distance = math.dist(position, goals[path])
                    period = args.target_period_sec if args.method == 'follow_moving' else 2.0
                    if distance > 0.45 and row['sim_time'] - last_update[path] >= period - 1.0e-9:
                        step = min(1.0 if args.method == 'follow_moving' else 2.5, distance)
                        point = [p + (g - p) * step / distance for p, g in zip(position, goals[path])]
                        targets[path][0].SetTranslate(Gf.Vec3d(*point))
                        targets[path] = (*targets[path][:2], point)
                        last_update[path] = row['sim_time']
                        updates[path] += 1
        for path in agents:
            result['people'][path].update(summarize(samples[path], goals[path]))
            result['people'][path]['target_updates'] = updates[path]
        result['status'] = 'PASS' if all(r['pass'] for r in result['people'].values()) else 'FAIL_MOTION'
        return 0 if result['status'] == 'PASS' else 2
    except Exception as exc:
        result['status'] = 'ERROR'
        result['error'] = repr(exc)
        import traceback
        traceback.print_exc()
        return 3
    finally:
        emit('result', **result)
        (args.output / 'metrics.json').write_text(json.dumps(json_safe(result), indent=2, allow_nan=False) + '\n')
        print('EXECUTION_AUDIT_RESULT=' + json.dumps(json_safe(result), allow_nan=False), flush=True)
        trace.close()
        timeline.stop()
        app.close()


if __name__ == '__main__':
    raise SystemExit(main())
