#!/usr/bin/env python3
"""Read-only IRA/BehaviorAgent discovery audit for one configured person."""

from __future__ import annotations

import argparse
import asyncio
import inspect
import json
import sys
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--setup-timeout", type=float, default=300.0)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
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

import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdSkel


def wait_for_task(task: asyncio.Task, timeout: float):
    import time
    deadline = time.monotonic() + timeout
    while not task.done() and simulation_app.is_running():
        simulation_app.update()
        if time.monotonic() >= deadline:
            task.cancel()
            raise TimeoutError("timed out while setting up IRA")
    if not task.done():
        raise RuntimeError("Isaac stopped during IRA setup")
    return task.result()


def safe_path(value):
    try:
        return str(value)
    except Exception:
        return repr(value)


def public_methods(obj):
    if obj is None:
        return []
    names = []
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception:
            continue
        if callable(value):
            names.append(name)
    return names


def resolve_config(config: Path, output: Path) -> Path:
    document = yaml.safe_load(config.read_text(encoding="utf-8"))
    root = document["isaacsim.replicator.agent"]
    scene = Path(root["environment"]["base_stage_asset_path"])
    if not scene.is_absolute():
        scene = Path(__file__).resolve().parents[2] / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
    root["environment"]["base_stage_asset_path"] = str(scene.resolve())
    resolved = output.parent / "single_resolved.yaml"
    resolved.parent.mkdir(parents=True, exist_ok=True)
    resolved.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return resolved


def main() -> int:
    output = ARGS.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    config = resolve_config(ARGS.config.resolve(), output)
    result = {
        "schema": "behavior_agent_discovery_debug/v1",
        "config": str(config),
        "timeline_played": False,
        "reset_called": False,
        "follow_called": False,
        "rows": [],
    }
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
        wait_for_task(asyncio.ensure_future(ira.setup_simulation()), ARGS.setup_timeout)
        # setup_simulation() can return before Metropolis has completed its
        # runtime-agent collection.  Observe that state without starting the
        # timeline or issuing any agent command.
        for _ in range(30):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        behavior_interface = behavior_core.acquire_interface()
        manager = AgentsManager.get_instance()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        simulation_app.update()
        timeline.pause()
        result["timeline_primed_one_tick"] = True
        runtime = list(manager.get_agents_by_type(IRA_Character))

        stage_entries = []
        characters_root = stage.GetPrimAtPath("/World/Characters")
        for prim in Usd.PrimRange(characters_root):
            if prim.IsA(UsdSkel.Root):
                stage_entries.append({
                    "kind": "UsdSkel.Root",
                    "usd_path": str(prim.GetPath()),
                    "name": prim.GetName(),
                    "type_name": prim.GetTypeName(),
                    "parent": str(prim.GetParent().GetPath()),
                })
        result["stage_skel_roots"] = stage_entries
        result["stage_character_prims"] = [
            {"path": str(prim.GetPath()), "name": prim.GetName(), "type_name": prim.GetTypeName()}
            for prim in Usd.PrimRange(characters_root)
            if prim.IsValid()
        ]
        result["behavior_interface_type"] = type(behavior_interface).__name__
        result["behavior_interface_public_methods"] = public_methods(behavior_interface)
        result["behavior_task_invalid_id"] = safe_path(getattr(behavior_core, "BEHAVIOR_TASK_ID_INVALID", None))
        result["agents_manager_type"] = type(manager).__name__
        result["agents_manager_public_methods"] = public_methods(manager)
        def call_registry(name):
            method = getattr(manager, name, None)
            if method is None:
                return {"error": "missing_method"}
            try:
                value = method()
                if isinstance(value, dict):
                    return {str(key): safe_path(item) for key, item in value.items()}
                if isinstance(value, (list, tuple, set)):
                    return [safe_path(item) for item in value]
                return safe_path(value)
            except Exception as exc:
                return {"error": repr(exc)}
        result["runtime_agents_collect_done"] = call_registry("is_runtime_agents_collect_done")
        result["registered_agent_prims"] = call_registry("get_registered_agent_prims")
        result["runtime_agent_instances_registry"] = call_registry("get_runtime_agent_instances")
        result["registered_agent_classes"] = call_registry("get_all_registered_agent_classes")
        result["registered_behavior_agents"] = []
        for index, obj in enumerate(runtime):
            prim = getattr(obj, "prim", None)
            entry = {
                "index": index,
                "object_type": type(obj).__name__,
                "object_id": id(obj),
                "prim_path": safe_path(prim.GetPath()) if prim is not None else None,
                "prim_name": prim.GetName() if prim is not None else None,
                "object_public_methods": public_methods(obj),
                "object_attributes": sorted(name for name in dir(obj) if not name.startswith("__")),
            }
            result["registered_behavior_agents"].append(entry)

        for entry in stage_entries:
            usd_path = entry["usd_path"]
            try:
                agent = behavior_interface.get_agent(usd_path)
            except Exception as exc:
                agent = None
                entry["get_agent_error"] = repr(exc)
            entry["behavior_agent_object_id"] = id(agent) if agent is not None else None
            entry["behavior_agent_exists"] = agent is not None
            entry["follow_callable"] = bool(agent is not None and callable(getattr(agent, "follow", None)))
            result["rows"].append({
                "object": entry["name"],
                "usd_path": usd_path,
                "runtime_path": None,
                "behavior_agent_exists": entry["behavior_agent_exists"],
                "follow_callable": entry["follow_callable"],
                "agent_object_id": entry["behavior_agent_object_id"],
            })

        for registered in result["registered_behavior_agents"]:
            matching = [row for row in result["rows"] if row["usd_path"] == registered["prim_path"]]
            if matching:
                for row in matching:
                    row["runtime_path"] = registered["prim_path"]
            else:
                result["rows"].append({
                    "object": registered["prim_name"],
                    "usd_path": None,
                    "runtime_path": registered["prim_path"],
                    "behavior_agent_exists": False,
                    "follow_callable": False,
                    "agent_object_id": None,
                    "registered_runtime_object_id": registered["object_id"],
                })

        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(result["rows"], indent=2, sort_keys=True), flush=True)
        print(f"BEHAVIOR_AGENT_DISCOVERY_DEBUG={output}", flush=True)
        return 0
    except Exception as exc:
        result["error"] = repr(exc)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"[BEHAVIOR-AGENT-DISCOVERY] ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        simulation_app.close()


raise SystemExit(main())
