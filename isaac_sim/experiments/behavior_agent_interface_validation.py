#!/usr/bin/env python3
"""Minimal BehaviorAgent interface validation experiments.

This file is intentionally independent of the formal pedestrian/social
pipeline.  It loads one existing IRA character, resets it once, starts one
follow task, and varies only the target update policy.
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--experiment", choices=("static", "moving", "stable_subgoal"), required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--setup-timeout", type=float, default=300.0)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
if ARGS.duration <= 0.0:
    raise SystemExit("duration must be positive")
sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "renderer": "RaytracedLighting",
    "multi_gpu": False,
    "extra_args": [
        "--/telemetry/enableAnonymousData=false",
        "--/privacy/usage=false",
        "--/privacy/performance=false",
        "--/privacy/personalization=false",
    ],
})

import carb
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Usd, UsdGeom, UsdSkel


def wait_for_task(task: asyncio.Task, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while not task.done() and simulation_app.is_running():
        simulation_app.update()
        if time.monotonic() >= deadline:
            task.cancel()
            raise TimeoutError(f"timed out while {label}")
    if not task.done():
        raise RuntimeError(f"Isaac stopped while {label}")
    return task.result()


def root_prim(stage: Usd.Stage):
    roots = []
    parent = stage.GetPrimAtPath("/World/Characters")
    for prim in Usd.PrimRange(parent):
        if prim.IsA(UsdSkel.Root):
            roots.append(prim)
    if not roots:
        raise RuntimeError("no BehaviorAgent SkelRoot found")
    return roots[0]


def root_matrix(root):
    matrix = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    values = [float(matrix[row][column]) for row in range(4) for column in range(4)]
    if not all(math.isfinite(value) for value in values):
        return None
    translation = matrix.ExtractTranslation()
    point = [float(translation[index]) for index in range(3)]
    if not all(math.isfinite(value) for value in point):
        return None
    return {"matrix": values, "position": point}


def authored_anchor(config: Path):
    document = yaml.safe_load(config.read_text(encoding="utf-8"))
    groups = document["isaacsim.replicator.agent"]["character"]["groups"]
    group = next(iter(groups.values()))
    patrol = group["routines"][0]["patrol"]
    point = patrol["path_points"][0]
    goal = patrol["path_points"][1]
    return [float(value) for value in point], [float(value) for value in goal]


def isolated_config(config: Path, artifact_root: Path) -> Path:
    """Materialize a self-contained artifact config without editing the source."""
    document = yaml.safe_load(config.read_text(encoding="utf-8"))
    root = document["isaacsim.replicator.agent"]
    scene = Path(root["environment"]["base_stage_asset_path"])
    if not scene.is_absolute():
        scene = Path(__file__).resolve().parents[2] / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
    root["environment"]["base_stage_asset_path"] = str(scene.resolve())
    output = artifact_root / "config" / "single_resolved.yaml"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return output


def set_target(api, path: str, target):
    api.SetTranslate(Gf.Vec3d(*[float(value) for value in target]))


def main() -> int:
    config = ARGS.config.resolve()
    trace_path = ARGS.trace.resolve()
    metrics_path = ARGS.metrics.resolve()
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_root = metrics_path.parent.parent
    config = isolated_config(config, artifact_root)
    anchor, authored_goal = authored_anchor(config)
    target_goal = [authored_goal[0], authored_goal[1], anchor[2]]
    trace = trace_path.open("w", encoding="utf-8", buffering=1)
    timeline = None
    try:
        enable_extension("isaacsim.replicator.agent.core")
        for _ in range(5):
            simulation_app.update()
        from isaacsim.replicator.agent.core import api as ira
        import omni.anim.behavior.core as behavior_core
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager

        if not ira.load_config_file(str(config)):
            raise RuntimeError(f"IRA rejected config: {config}")
        wait_for_task(
            asyncio.ensure_future(ira.setup_simulation()),
            ARGS.setup_timeout,
            "setting up IRA",
        )
        stage = omni.usd.get_context().get_stage()
        root = root_prim(stage)
        root_path = str(root.GetPath())
        interface = behavior_core.acquire_interface()
        # IRA registers the BehaviorAgent after one Fabric/timeline tick.  This
        # is discovery initialization only; no reset, follow, or control is
        # issued while priming the runtime.
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        simulation_app.update()
        timeline.pause()
        agent = interface.get_agent(root_path)
        if agent is None:
            raise RuntimeError(f"BehaviorAgent missing for {root_path}")

        # Stop the authored patrol scheduler without changing its source/config.
        # The runtime IRA_Character is not used to discover the BehaviorAgent;
        # it is only used here to prevent the authored patrol from competing
        # with this isolated interface test.
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager
        runtime_agents = {
            str(item.prim.GetPath()): item
            for item in AgentsManager.get_instance().get_agents_by_type(IRA_Character)
        }
        runtime_agent = runtime_agents.get(root_path)
        if runtime_agent is not None:
            runtime_agent._refresh_runtime_states_coro()
            runtime_agent._active_routine = None
            runtime_agent.routines.clear()
            runtime_agent._routine_p = []

        target_path = "/World/BehaviorAgentInterfaceValidationTarget"
        target_prim = UsdGeom.Xform.Define(stage, target_path).GetPrim()
        target_api = UsdGeom.XformCommonAPI(target_prim)
        set_target(target_api, target_path, target_goal)

        reset_result = agent.reset(target=carb.Float3(*anchor))
        if not reset_result:
            raise RuntimeError("BehaviorAgent reset returned false")
        timeline.play()
        for _ in range(15):
            simulation_app.update()
        live_after_reset = root_matrix(root)
        if live_after_reset is None:
            raise RuntimeError("live root is invalid after reset")

        task_id = agent.follow(target=target_path, distance=0.0)
        invalid_id = behavior_core.BEHAVIOR_TASK_ID_INVALID
        if task_id == invalid_id:
            raise RuntimeError("BehaviorAgent.follow returned invalid task id")
        agent.set_speed(1.0)

        header = {
            "schema": "behavior_agent_interface_validation/v1",
            "experiment": ARGS.experiment,
            "config": str(config),
            "root_path": root_path,
            "authored_reset_anchor": anchor,
            "authored_goal": target_goal,
            "task_id": int(task_id),
            "follow_calls": 1,
            "social_force": False,
            "formal_pipeline": False,
        }
        trace.write(json.dumps({"type": "header", **header}, sort_keys=True) + "\n")
        trace.write(json.dumps({"type": "reset_live_root", "value": live_after_reset}, sort_keys=True) + "\n")

        start_sim = float(timeline.get_current_time())
        last_sim = start_sim
        last_position = live_after_reset["position"]
        freeze_sec = 0.0
        max_freeze_sec = 0.0
        distance = 0.0
        target_updates = 0
        task_interruption_count = 0
        invalid_root_count = 0
        target = list(target_goal)
        last_target_update_sim = start_sim
        last_target_position = list(target)
        stable_hold_sec = 2.0
        stable_advance_m = 0.75

        while simulation_app.is_running():
            simulation_app.update()
            sim_time = float(timeline.get_current_time())
            elapsed = sim_time - start_sim
            if elapsed >= ARGS.duration:
                break
            dt = max(1.0e-6, sim_time - last_sim)
            sample = root_matrix(root)
            if sample is None:
                invalid_root_count += 1
            else:
                position = sample["position"]
                step = math.dist(position, last_position)
                distance += step
                if step < 0.01:
                    freeze_sec += dt
                else:
                    max_freeze_sec = max(max_freeze_sec, freeze_sec)
                    freeze_sec = 0.0
                last_position = position
                running = bool(agent.is_task_running(task_id))
                if not running:
                    task_interruption_count += 1

                if ARGS.experiment == "moving":
                    direction = (target_goal[0] - anchor[0], target_goal[1] - anchor[1])
                    norm = math.hypot(*direction)
                    target = [position[0] + direction[0] / norm, position[1] + direction[1] / norm, position[2]]
                    set_target(target_api, target_path, target)
                    target_updates += 1
                elif ARGS.experiment == "stable_subgoal":
                    direction = (target_goal[0] - position[0], target_goal[1] - position[1])
                    norm = math.hypot(*direction)
                    if norm > 0.35 and (sim_time - last_target_update_sim >= stable_hold_sec or math.dist(target, position) < stable_advance_m):
                        step = min(2.5, norm)
                        candidate = [position[0] + direction[0] / norm * step, position[1] + direction[1] / norm * step, position[2]]
                        if math.dist(candidate, last_target_position) >= stable_advance_m or sim_time - last_target_update_sim >= stable_hold_sec:
                            target = candidate
                            set_target(target_api, target_path, target)
                            last_target_position = list(target)
                            last_target_update_sim = sim_time
                            target_updates += 1

                trace.write(json.dumps({
                    "type": "sample",
                    "sim_time": sim_time,
                    "position": position,
                    "target": target,
                    "task_id": int(task_id),
                    "task_running": running,
                    "root_transform_valid": sample is not None,
                }, sort_keys=True) + "\n")
            last_sim = sim_time

        max_freeze_sec = max(max_freeze_sec, freeze_sec)
        final_sample = root_matrix(root)
        final_position = final_sample["position"] if final_sample else None
        metrics = {
            **header,
            "duration_sec": float(timeline.get_current_time()) - start_sim,
            "reset_result": bool(reset_result),
            "live_root_after_reset": live_after_reset,
            "final_position": final_position,
            "distance_m": distance,
            "target_updates": target_updates,
            "follow_restart_count": 0,
            "task_interruption_count": task_interruption_count,
            "motion_error_count": invalid_root_count,
            "freeze_sec": freeze_sec,
            "max_freeze_sec": max_freeze_sec,
            "goal_distance_m": math.dist(final_position, target_goal) if final_position else None,
            "completion": bool(final_position and math.dist(final_position, target_goal) <= 0.45),
        }
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("BEHAVIOR_AGENT_INTERFACE_METRICS=" + json.dumps(metrics, sort_keys=True), flush=True)
        return 0
    except Exception as exc:
        print(f"[BEHAVIOR-AGENT-VALIDATION] ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        trace.close()
        if timeline is not None:
            timeline.stop()
        simulation_app.close()


raise SystemExit(main())
