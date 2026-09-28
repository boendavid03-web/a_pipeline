#!/usr/bin/env python3
"""Isolated single-person command tracking; no production imports with side effects."""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import yaml
from execution_audit_metrics import json_safe, valid_pose
from human_motion_execution_metrics import analyze, command_at, integrate, segment_feasibility, validate_schedule
from stop_contract_policy import StopContractPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads(args.spec.read_text())
    schedule = spec['schedule']
    validate_schedule(schedule)
    stop_policy = StopContractPolicy(spec.get('stop_mode', 'continuous'))
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'isaac_sim/scripts'))
    # Exactly the existing pure production mapping, without the runtime launcher.
    from pedestrian_steering import steering_target_from_velocity
    args.output.mkdir(parents=True, exist_ok=False)
    document = yaml.safe_load(Path(spec['config']).read_text())
    config = document['isaacsim.replicator.agent']
    config['environment']['base_stage_asset_path'] = spec['scene']
    groups = config['character']['groups']
    if len(groups) != 1 or next(iter(groups.values()))['num'] != 1:
        raise ValueError('single-person benchmark only')
    resolved = args.output / 'resolved.yaml'
    resolved.write_text(yaml.safe_dump(document, sort_keys=False))
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'multi_gpu': False})
    import carb
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdSkel
    timeline = omni.timeline.get_timeline_interface()
    trace = (args.output / 'trace.jsonl').open('w', buffering=1)
    intervals = []
    result = dict(schema='human_motion_execution/v1', status='INCOMPLETE', spec=spec,
                  social_force_live=False, physical_contact_truth=False,
                  production_pipeline=False, reset_calls=0, follow_calls=0)
    result.update(idle_calls=0, stop_mode=stop_policy.mode, task_events=[])

    def emit(kind, **data):
        trace.write(json.dumps(json_safe(dict(type=kind, **data)), allow_nan=False) + '\n')

    def tick():
        if not app.is_running():
            raise RuntimeError('app closed')
        app.update()

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
        setup = asyncio.ensure_future(ira.setup_simulation())
        deadline = time.monotonic() + 180
        while not setup.done():
            tick()
            if time.monotonic() > deadline:
                setup.cancel()
                raise TimeoutError('IRA setup')
        setup.result()
        stage = omni.usd.get_context().get_stage()
        if str(UsdGeom.GetStageUpAxis(stage)) != 'Z' or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
            raise RuntimeError('requires Z-up metres')
        navmesh = navigation.acquire_interface().get_navmesh()
        if navmesh is None:
            raise RuntimeError('NavMesh unavailable')
        roots = [p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Characters')) if p.IsA(UsdSkel.Root)]
        if len(roots) != 1:
            raise RuntimeError('SkelRoot count')
        prim = roots[0]
        path = str(prim.GetPath())
        interface = bh.acquire_interface()
        interface.set_random_seed(int(config.get('seed', 7)))
        timeline.play()
        for _ in range(30):
            tick()
        agent = interface.get_agent(path)
        runtimes = {str(r.prim.GetPath()): r for r in AgentsManager.get_instance().get_agents_by_type(IRA_Character)}
        if agent is None or path not in runtimes:
            raise RuntimeError('BehaviorAgent not ready')
        runtime = runtimes[path]
        if runtime.triggers or runtime._active_triggers:
            raise RuntimeError('unexpected triggers')
        runtime._refresh_runtime_states_coro()
        runtime._active_routine = None
        runtime.routines.clear()
        runtime._routine_p = []
        agent.set_auto_avoidance_enabled(False)
        agent.set_obstacle_avoidance_enabled(False)

        def state():
            return dict(position=list(agent.get_world_translation()), quaternion=list(agent.get_world_rotation()),
                        facing=list(agent.get_facing_direction()), reported_velocity=list(agent.get_linear_velocity()),
                        reported_speed=float(agent.get_speed()), reported_target=list(agent.get_target_location()),
                        reported_target_object=str(agent.get_target_object()),
                        active_task_id=int(agent.get_action_task_id()), sim_time=float(timeline.get_current_time()),
                        usd_position=list(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()))

        anchor = spec['anchor']
        result['actor_path'] = path
        # Composition layers identify the actual randomly resolved asset and motion library.
        result['usd_layers_sha256'] = {str(l.identifier): hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()
                                     for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()}
        emit('before_reset', **state())
        result['reset_calls'] += 1
        if not agent.reset(target=carb.Float3(*anchor), facing=carb.Float3(1.0, 0.0, 0.0)):
            raise RuntimeError('reset refused')
        settled = []
        for frame in range(15):
            tick()
            s = state()
            s['anchor_error_m'] = math.dist(s['position'], anchor)
            settled.append(s)
            emit('reset_settle', frame=frame, **s)
        result['reset_pass'] = all(valid_pose(s['position'], s['quaternion']) and s['anchor_error_m'] <= 0.15
                                   and s['facing'][0] >= math.cos(math.radians(5)) for s in settled[-3:])
        result['reset_error_m'] = settled[-1]['anchor_error_m']
        result['initial_facing'] = settled[-1]['facing']
        if not result['reset_pass']:
            result['status'] = 'INVALID_RESET'
            return 2

        def nav_diagnostics(position, target):
            # Read-only feasibility gate: never project/correct an input command.
            def project(point):
                closest, island = navmesh.query_closest_point(target=carb.Float3(*point))
                return (list(closest) if closest is not None else None), island
            route = navmesh.query_shortest_path(start_pos=carb.Float3(*position), end_pos=carb.Float3(*target))
            return segment_feasibility(position, target, project, route.length() if route is not None else None)

        # Check ideal integrated path and forward target before issuing motion.
        preflight = []
        for index in range(int(spec['duration'] * 20) + 1):
            t = min(spec['duration'], index / 20)
            d = integrate(schedule, 0.0, t)
            position = [anchor[0] + d[0], anchor[1] + d[1], anchor[2]]
            _, c = command_at(schedule, t)
            mapping = steering_target_from_velocity(position[:2], c['velocity'], spec['lookahead_m'])
            target = [*mapping.target_position_m, anchor[2]]
            preflight.append(nav_diagnostics(position, target)['valid'])
        result['navmesh_preflight'] = dict(checked=len(preflight), invalid=sum(not x for x in preflight))
        if not all(preflight):
            result['status'] = 'INVALID_GEOMETRY'
            return 2
        target_path = '/World/VelocityBenchmarkTarget'
        target_api = UsdGeom.XformCommonAPI(UsdGeom.Xform.Define(stage, target_path).GetPrim())
        initial = state()
        mapping = steering_target_from_velocity(initial['position'][:2], schedule[0]['velocity'], spec['lookahead_m'])
        target = [*mapping.target_position_m, anchor[2]]
        target_api.SetTranslate(Gf.Vec3d(*target))
        task_id = agent.follow(target=target_path, distance=0.0)
        result['follow_calls'] += 1
        if task_id == bh.BEHAVIOR_TASK_ID_INVALID:
            raise RuntimeError('follow refused')
        started = float(timeline.get_current_time())
        previous = state()
        next_control = 0.0
        applied = None
        target_writes = 1
        command_updates = 0
        deadline = time.monotonic() + 90
        while previous['sim_time'] - started < spec['duration'] - 1e-8:
            t0 = previous['sim_time'] - started
            if applied is None or t0 >= next_control - 1e-8:
                command_index, applied = command_at(schedule, t0)
                mapped = steering_target_from_velocity(previous['position'][:2], applied['velocity'], spec['lookahead_m'])
                requested = [*mapped.target_position_m, anchor[2]]
                policy = stop_policy.update(mapped.speed_mps, requested)
                selected_target = policy['target']
                # Match production target write threshold; speed is written at every control tick.
                written = policy['force_target_write'] or math.dist(selected_target, target) >= spec['target_min_shift_m']
                if written:
                    target_api.SetTranslate(Gf.Vec3d(*selected_target))
                    target = selected_target
                    target_writes += 1
                if policy['action']:
                    old_task = int(task_id)
                    task_id = agent.idle() if policy['action'] == 'idle' else agent.follow(target=target_path, distance=0.0)
                    if task_id == bh.BEHAVIOR_TASK_ID_INVALID:
                        raise RuntimeError(f"{policy['action']} refused")
                    result['idle_calls' if policy['action'] == 'idle' else 'follow_calls'] += 1
                    event = dict(t=t0, action=policy['action'], previous_task_id=old_task, issued_task_id=int(task_id))
                    result['task_events'].append(event)
                    emit('task_transition', **event)
                agent.set_speed(mapped.speed_mps)
                command_updates += 1
                emit('command', t=t0, source_t=applied['t'], index=command_index,
                     velocity=applied['velocity'], speed=mapped.speed_mps,
                     requested_target=requested, applied_target=target, target_written=written,
                     reported_speed_after_write=float(agent.get_speed()), action=policy['action'])
                next_control += spec['control_period_sec']
            before_wall = time.monotonic()
            tick()
            current = state()
            t1 = current['sim_time'] - started
            nav_probe = nav_diagnostics(previous['position'], target)
            root_probe = nav_diagnostics(current['position'], current['position'])
            valid_nav = nav_probe['valid'] and root_probe['valid']
            row = dict(t0=t0, t1=t1, position0=previous['position'], position1=current['position'],
                       quaternion=current['quaternion'], facing=current['facing'],
                       reported_velocity=current['reported_velocity'], usd_position=current['usd_position'],
                       reported_speed=current['reported_speed'], reported_target=current['reported_target'],
                       reported_target_object=current['reported_target_object'],
                       command=list(applied['velocity']), command_source_time=applied['t'],
                       active_task_id=current['active_task_id'], issued_task_id=int(task_id),
                       task_running=bool(agent.is_task_running(task_id)), task_status=str(agent.get_task_status(task_id)),
                       target=list(target), navmesh_valid=valid_nav, navmesh_probe=nav_probe,
                       root_navmesh_probe=root_probe, app_wall_sec=time.monotonic() - before_wall)
            intervals.append(row)
            emit('interval', **row)
            if not valid_pose(current['position'], current['quaternion']) or not valid_nav:
                raise RuntimeError('live pose or geometry gate failed; no automatic recovery')
            if time.monotonic() > deadline:
                raise TimeoutError('motion watchdog')
            previous = current
        result.update(analyze(intervals, schedule, spec['duration'], reset_pass=result['reset_pass'], synthetic=spec['synthetic']))
        result.update(command_updates=command_updates, target_writes=target_writes)
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
        print('HUMAN_MOTION_EXECUTION_RESULT=' + json.dumps(json_safe(result), allow_nan=False), flush=True)
        trace.close()
        timeline.stop()
        app.close()


if __name__ == '__main__':
    raise SystemExit(main())
