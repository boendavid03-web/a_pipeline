#!/usr/bin/env python3
"""Load the active Isaac 6 project scene alone in the Isaac Sim 5.1 GUI."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import time
from pathlib import Path

from contract import generated_output_path, pass_or_fail

BACKEND_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SCENE_USD = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
CAMERA_EYE = (38.0, -16.0, 30.0)
CAMERA_TARGET = (16.0, 12.0, 0.5)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--scene-usd", type=Path, default=DEFAULT_SCENE_USD)
    parser.add_argument("--min-wall-seconds", type=float, default=60.0)
    parser.add_argument("--setup-timeout", type=float, default=30.0)
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="Optional GUI viewport PNG; relative paths stay below backend generated/.",
    )
    return parser.parse_args()


ARGS = parse_args()
SCREENSHOT = generated_output_path(BACKEND_ROOT, ARGS.screenshot)

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "fast_shutdown": True,
        "width": 1280,
        "height": 720,
    }
)

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils  # noqa: E402


def wait_for_task(task: asyncio.Task, timeout: float, label: str):
    """Use the same bounded Kit-update wait used by the successful Isaac 6 runner."""

    deadline = time.monotonic() + timeout
    while not task.done() and simulation_app.is_running():
        simulation_app.update()
        if time.monotonic() >= deadline:
            task.cancel()
            raise TimeoutError(f"Timed out after {timeout:.1f}s while {label}")
    if not task.done():
        raise RuntimeError(f"Isaac Sim stopped while {label}")
    return task.result()


def capture_viewport(world: World, output_path: Path) -> None:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    output_path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport is available for scene screenshot")
    capture = capture_viewport_to_file(viewport, file_path=str(output_path))
    task = asyncio.ensure_future(capture.wait_for_result(completion_frames=8))
    for _ in range(300):
        world.step(render=True)
        if task.done():
            break
    if not task.done():
        task.cancel()
        raise RuntimeError(f"Scene screenshot timed out: {output_path}")
    if not task.result():
        raise RuntimeError(f"Scene screenshot capture failed: {output_path}")
    import omni.kit.renderer_capture

    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"Scene screenshot was not written: {output_path}")


def main() -> int:
    scene_path = ARGS.scene_usd.expanduser().resolve()
    if not scene_path.is_file():
        raise FileNotFoundError(f"Active scene USD not found: {scene_path}")
    if ARGS.min_wall_seconds < 0.0 or ARGS.setup_timeout <= 0.0:
        raise ValueError("wall duration must be non-negative and setup timeout positive")
    if ARGS.headless and SCREENSHOT is not None:
        raise ValueError("--screenshot requires a GUI run")

    dependencies, asset_paths, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
    dependency_ids = sorted(str(layer.identifier) for layer in dependencies)
    asset_ids = sorted(str(path) for path in asset_paths)
    unresolved_ids = sorted(str(path) for path in unresolved)

    success, error = wait_for_task(
        asyncio.ensure_future(omni.usd.get_context().open_stage_async(str(scene_path))),
        ARGS.setup_timeout,
        "loading the active project scene",
    )
    if not success:
        raise RuntimeError(f"Could not open active scene: {error}")
    for _ in range(5):
        simulation_app.update()
    simulation_app.reset_render_settings()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Scene open completed without a USD stage")
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    up_axis = str(UsdGeom.GetStageUpAxis(stage)).upper()
    default_prim = stage.GetDefaultPrim()
    prims = list(stage.Traverse())
    collisions = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
    rigid_bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    physics_scenes = [p for p in prims if p.IsA(UsdPhysics.Scene)]
    lights = [p for p in prims if p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase)]
    navmesh_volumes = [p for p in prims if p.GetTypeName() == "NavMeshVolume"]

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    bounds = bbox_cache.ComputeWorldBound(default_prim).ComputeAlignedBox()
    bbox_min = np.asarray(bounds.GetMin(), dtype=float)
    bbox_max = np.asarray(bounds.GetMax(), dtype=float)
    bbox_finite = bool(np.all(np.isfinite(bbox_min)) and np.all(np.isfinite(bbox_max)))

    world = World(
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=meters_per_unit,
    )
    world.reset()
    if not ARGS.headless:
        set_camera_view(
            eye=np.asarray(CAMERA_EYE),
            target=np.asarray(CAMERA_TARGET),
            camera_prim_path="/OmniverseKit_Persp",
        )
    camera_prim = stage.GetPrimAtPath("/OmniverseKit_Persp")

    start_sim = float(world.current_time)
    start_wall = time.monotonic()
    frames = 0
    while time.monotonic() - start_wall < ARGS.min_wall_seconds:
        world.step(render=not ARGS.headless)
        frames += 1
    wall_seconds = time.monotonic() - start_wall
    end_sim = float(world.current_time)

    if SCREENSHOT is not None:
        capture_viewport(world, SCREENSHOT)

    teardown_ok = True
    teardown_error = None
    try:
        world.stop()
    except Exception as exc:
        teardown_ok = False
        teardown_error = repr(exc)

    checks = {
        "dependency_closure": not unresolved_ids,
        "default_prim": bool(default_prim and default_prim.IsValid()),
        "meters_per_unit": math.isclose(meters_per_unit, 1.0, abs_tol=1.0e-9),
        "up_axis": up_axis == "Z",
        "bbox_finite": bbox_finite,
        "collisions_present": len(collisions) >= 1,
        "physics_scene_present": len(physics_scenes) == 1,
        "lights_present": len(lights) >= 1,
        "camera_operable": ARGS.headless or camera_prim.IsValid(),
        "physics_advanced": end_sim > start_sim,
        "wall_duration": ARGS.headless or wall_seconds >= ARGS.min_wall_seconds,
        "teardown_ok": teardown_ok,
    }
    status, failure_reasons = pass_or_fail(
        checks, [teardown_error] if teardown_error else []
    )
    report = {
        "status": status,
        "failure_reasons": failure_reasons,
        "backend": "isaac5",
        "isaac_version": "5.1",
        "scene_usd": str(scene_path),
        "scene_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        "viewport_screenshot": str(SCREENSHOT) if SCREENSHOT is not None else None,
        "default_prim": str(default_prim.GetPath()) if default_prim.IsValid() else None,
        "meters_per_unit": meters_per_unit,
        "up_axis": up_axis,
        "prim_count": len(prims),
        "collision_count": len(collisions),
        "rigid_body_count": len(rigid_bodies),
        "physics_scene_paths": [str(p.GetPath()) for p in physics_scenes],
        "light_paths": [str(p.GetPath()) for p in lights],
        "navmesh_volume_count": len(navmesh_volumes),
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "dependency_layers": dependency_ids,
        "asset_dependencies": asset_ids,
        "unresolved_dependencies": unresolved_ids,
        "camera_eye": list(CAMERA_EYE),
        "camera_target": list(CAMERA_TARGET),
        "physics_frames": frames,
        "simulation_seconds": end_sim - start_sim,
        "wall_seconds": wall_seconds,
    }
    print("SCENE_VALIDATION_RESULT=" + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if status == "PASS" else 2


try:
    print(
        "SCENE_VALIDATION_READY="
        + json.dumps(
            {
                "backend": "isaac5",
                "isaac_version": "5.1",
                "scene_usd": str(ARGS.scene_usd.expanduser().resolve()),
                "robot": False,
                "pedestrians": False,
                "ros": False,
                "lidar": False,
                "navigation": False,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    exit_code = main()
except Exception as exc:
    print(
        "SCENE_VALIDATION_RESULT="
        + json.dumps(
            {
                "status": "FAIL",
                "backend": "isaac5",
                "isaac_version": "5.1",
                "failure_reasons": [repr(exc)],
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    exit_code = 1
finally:
    simulation_app.close()
raise SystemExit(exit_code)
