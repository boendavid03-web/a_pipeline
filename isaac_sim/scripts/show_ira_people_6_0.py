#!/usr/bin/env python3
"""Run an offline Isaac Replicator Agent warehouse people visualization."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "ira_people_demo" / "ira_people_demo.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--headless", action="store_true", help="Run without a GUI window.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Wall-clock seconds to run; zero keeps running until the window closes.",
    )
    parser.add_argument("--setup-timeout", type=float, default=300.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


ARGS = parse_args()
CONFIG_PATH = ARGS.config.expanduser().resolve()
# SimulationApp otherwise forwards this script's own flags to Kit as unknown
# application arguments.
sys.argv = [sys.argv[0]]

if not CONFIG_PATH.is_file():
    raise SystemExit(f"ERROR: IRA config does not exist: {CONFIG_PATH}")
if "ISAACSIM_ASSET_ROOT" not in os.environ:
    raise SystemExit("ERROR: ISAACSIM_ASSET_ROOT is not set; use run_isaac_6_0_ira_people.sh")


from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "width": ARGS.width,
        "height": ARGS.height,
        "extra_args": [
            "--/telemetry/enableAnonymousData=false",
            "--/privacy/usage=false",
            "--/privacy/performance=false",
            "--/privacy/personalization=false",
        ],
    }
)


import carb
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdSkel


def wait_for_task(task: asyncio.Task, timeout: float, label: str):
    deadline = time.monotonic() + timeout
    while not task.done() and simulation_app.is_running():
        simulation_app.update()
        if time.monotonic() >= deadline:
            task.cancel()
            raise TimeoutError(f"Timed out after {timeout:.1f}s while {label}")
    if not task.done():
        raise RuntimeError(f"Isaac Sim stopped while {label}")
    return task.result()


def character_roots(stage):
    root = stage.GetPrimAtPath("/World/Characters")
    if not root.IsValid():
        return []
    result = []
    for prim in Usd.PrimRange(root):
        if prim != root and prim.GetParent() == root:
            continue
        if prim.IsA(UsdSkel.Root):
            result.append(prim)
    return result


def character_positions(stage) -> dict[str, tuple[float, float, float]]:
    positions: dict[str, tuple[float, float, float]] = {}
    # IRA moves the BehaviorAgent SkelRoot, not the outer payload prim created
    # for each character.  Track those roots so the stability report reflects
    # actual navigation rather than the stationary payload containers.
    for prim in character_roots(stage):
        point = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        positions[str(prim.GetPath())] = (float(point[0]), float(point[1]), float(point[2]))
    return positions


def set_viewport_camera() -> None:
    if ARGS.headless:
        return
    try:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(
            eye=[16.0, 16.0, 11.0],
            target=[0.0, 0.0, 1.0],
            camera_prim_path="/OmniverseKit_Persp",
        )
    except Exception as exc:
        carb.log_warn(f"Could not set the initial viewport camera: {exc}")


exit_code = 0
timeline = None
try:
    print(f"[IRA-DEMO] Isaac asset root: {os.environ['ISAACSIM_ASSET_ROOT']}", flush=True)
    print(f"[IRA-DEMO] Config: {CONFIG_PATH}", flush=True)

    print("[IRA-DEMO] Enabling isaacsim.replicator.agent.core...", flush=True)
    enable_extension("isaacsim.replicator.agent.core")
    print("[IRA-DEMO] IRA Core enabled; allowing dependencies to initialize...", flush=True)
    for update_index in range(5):
        simulation_app.update()
        print(f"[IRA-DEMO] Extension initialization update {update_index + 1}/5", flush=True)

    from isaacsim.replicator.agent.core import api as ira

    print("[IRA-DEMO] Validating the offline IRA configuration...", flush=True)
    if not ira.load_config_file(str(CONFIG_PATH)):
        raise RuntimeError(f"IRA rejected config: {CONFIG_PATH}")

    print("[IRA-DEMO] Loading warehouse, NavMesh, motion library, and characters...", flush=True)
    setup_task = asyncio.ensure_future(ira.setup_simulation())
    wait_for_task(setup_task, ARGS.setup_timeout, "setting up the IRA scene")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("IRA setup completed without an open USD stage")

    roots = character_roots(stage)
    if len(roots) < 3:
        raise RuntimeError(f"Expected at least 3 skinned characters, found {len(roots)}")

    print(f"[IRA-DEMO] Loaded {len(roots)} skinned character roots:", flush=True)
    for root in roots:
        print(f"[IRA-DEMO]   {root.GetPath()}", flush=True)

    motion_library = stage.GetPrimAtPath("/World/Characters/HumanMotionLibrary")
    if not motion_library.IsValid():
        raise RuntimeError("Local HumanMotionLibrary payload was not created")
    print("[IRA-DEMO] Local HumanMotionLibrary is present.", flush=True)

    set_viewport_camera()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[IRA-DEMO] Patrol loop is running. Close the window or press Ctrl+C to stop.", flush=True)

    started = time.monotonic()
    last_report = started
    start_sim_time = float(timeline.get_current_time())
    frame_count = 0
    start_positions = character_positions(stage)
    while simulation_app.is_running():
        simulation_app.update()
        frame_count += 1
        now = time.monotonic()
        if now - last_report >= 30.0:
            positions = character_positions(stage)
            moved = 0
            for path, current in positions.items():
                initial = start_positions.get(path)
                if initial and math.dist(initial, current) > 0.05:
                    moved += 1
            print(
                f"[IRA-DEMO] Stable for {now - started:.0f}s; "
                f"{moved}/{len(positions)} character skeleton roots changed position.",
                flush=True,
            )
            last_report = now
        if ARGS.duration > 0.0 and now - started >= ARGS.duration:
            break
    wall_elapsed = max(1.0e-9, time.monotonic() - started)
    sim_elapsed = max(0.0, float(timeline.get_current_time()) - start_sim_time)
    print(
        "IRA_PEOPLE_PERFORMANCE="
        f"frames={frame_count} wall_sec={wall_elapsed:.3f} "
        f"sim_sec={sim_elapsed:.3f} fps={frame_count / wall_elapsed:.3f} "
        f"rtf={sim_elapsed / wall_elapsed:.3f}",
        flush=True,
    )
except KeyboardInterrupt:
    print("[IRA-DEMO] Interrupted by user.", flush=True)
except Exception as exc:
    exit_code = 1
    carb.log_error(f"IRA people demo failed: {exc}")
    print(f"[IRA-DEMO] ERROR: {exc}", file=sys.stderr, flush=True)
finally:
    if timeline is not None:
        timeline.stop()
    simulation_app.close()

raise SystemExit(exit_code)
