#!/usr/bin/env python3
"""Show three offline USD-Skel walkers in the local Isaac Sim 6.0 warehouse.

This deliberately does not enable Isaac Replicator Agent or
``omni.anim.behavior.core``.  The latter crashed in this installation's
motion-matching update after roughly six minutes.  Instead, this runner uses
the standard USD Skel binding: it loads the local WalkForward clip once,
bakes a short looping USD SkelAnimation, and binds that animation to three
local skinned-character assets.  Their parent Xforms follow looping routes.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without a GUI window.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Wall-clock seconds to run; zero keeps running until the window closes.",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


ARGS = parse_args()
sys.argv = [sys.argv[0]]  # Do not pass this script's flags to Kit.

if "ISAACSIM_ASSET_ROOT" not in os.environ:
    raise SystemExit("ERROR: ISAACSIM_ASSET_ROOT is not set; use the companion .sh launcher")

ASSET_ROOT = Path(os.environ["ISAACSIM_ASSET_ROOT"]).resolve()
WAREHOUSE = ASSET_ROOT / "Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd"
WALK_FORWARD = ASSET_ROOT / "Isaac/People/MotionLibrary/BuiltinActions/MoveWalk/WalkForward.usd"
CHARACTERS = (
    ASSET_ROOT / "Isaac/People/Characters/F_Business_02/F_Business_02.usd",
    ASSET_ROOT / "Isaac/People/Characters/male_adult_police_04/male_adult_police_04.usd",
    ASSET_ROOT
    / "Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd",
)
for required_asset in (WAREHOUSE, WALK_FORWARD, *CHARACTERS):
    if not required_asset.is_file():
        raise SystemExit(f"ERROR: required local asset is missing: {required_asset}")


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
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdSkel


ROOT = "/World/DynamicSkelWalkers"
FPS = 30.0
LOOP_SECONDS = 12.0


def update_frames(count: int) -> None:
    for _ in range(count):
        simulation_app.update()


def wait_until_loaded(context, timeout: float = 180.0) -> Usd.Stage:
    deadline = time.monotonic() + timeout
    while context.get_stage_loading_status()[2] > 0:
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timed out loading {WAREHOUSE}")
        simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim did not provide a USD stage")
    return stage


def find_first(prim: Usd.Prim, schema_type):
    for candidate in Usd.PrimRange(prim):
        if candidate.IsA(schema_type):
            return candidate
    return Usd.Prim()


def time_range(animation: UsdSkel.Animation) -> tuple[float, float]:
    sample_times: list[float] = []
    for attribute in (
        animation.GetRotationsAttr(),
        animation.GetTranslationsAttr(),
        animation.GetScalesAttr(),
    ):
        sample_times.extend(attribute.GetTimeSamples())
    if len(sample_times) < 2:
        raise RuntimeError(
            f"Local walk clip {animation.GetPath()} has insufficient animation samples: {sample_times}"
        )
    return min(sample_times), max(sample_times)


def bake_looping_walk(stage: Usd.Stage, source: UsdSkel.Animation) -> UsdSkel.Animation:
    """Copy one local walk clip into a stage-local loop usable by all walkers."""
    source_start, source_end = time_range(source)
    target = UsdSkel.Animation.Define(stage, f"{ROOT}/LoopingWalk")
    target.CreateJointsAttr().Set(source.GetJointsAttr().Get())
    target_frames = int(LOOP_SECONDS * FPS)

    source_attributes = (
        (source.GetRotationsAttr(), target.CreateRotationsAttr()),
        (source.GetTranslationsAttr(), target.CreateTranslationsAttr()),
        (source.GetScalesAttr(), target.CreateScalesAttr()),
    )
    for frame in range(target_frames + 1):
        target_time = float(frame)
        # The final frame deliberately equals the first source frame, so the
        # stage timeline can loop without a pose discontinuity.
        fraction = (frame % target_frames) / target_frames
        source_time = source_start + fraction * (source_end - source_start)
        for source_attr, target_attr in source_attributes:
            value = source_attr.Get(Usd.TimeCode(source_time))
            if value is not None:
                target_attr.Set(value, Usd.TimeCode(target_time))
    return target


def set_route(parent: Usd.Prim, points: list[tuple[float, float, float]]) -> None:
    """Author a closed, forward-facing Xform route for one character parent."""
    if points[0] != points[-1]:
        points = [*points, points[0]]
    xform = UsdGeom.Xformable(parent)
    translate = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    orient = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    last_index = len(points) - 1
    for index, point in enumerate(points):
        next_point = points[(index + 1) % last_index]
        heading = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        rotation = Gf.Quatf(
            math.cos(heading / 2.0),
            Gf.Vec3f(0.0, 0.0, math.sin(heading / 2.0)),
        )
        frame = LOOP_SECONDS * FPS * index / last_index
        translate.Set(Gf.Vec3d(*point), Usd.TimeCode(frame))
        orient.Set(rotation, Usd.TimeCode(frame))


def add_character(stage: Usd.Stage, index: int, character_path: Path, walk_animation: UsdSkel.Animation) -> str:
    parent = UsdGeom.Xform.Define(stage, f"{ROOT}/Walker_{index}").GetPrim()
    character = stage.DefinePrim(f"{ROOT}/Walker_{index}/Character")
    character.GetReferences().AddReference(character_path.as_posix())
    update_frames(12)
    skeleton_prim = find_first(character, UsdSkel.Skeleton)
    if not skeleton_prim.IsValid():
        raise RuntimeError(f"No UsdSkel.Skeleton found in {character_path}")
    binding = UsdSkel.BindingAPI.Apply(skeleton_prim)
    binding.CreateAnimationSourceRel().SetTargets([walk_animation.GetPath()])
    return str(skeleton_prim.GetPath())


def set_viewport_camera() -> None:
    if ARGS.headless:
        return
    try:
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(
            eye=[18.0, -18.0, 14.0],
            target=[0.0, 0.0, 1.1],
            camera_prim_path="/OmniverseKit_Persp",
        )
    except Exception as exc:
        carb.log_warn(f"Could not set initial viewport camera: {exc}")


def route_position(stage: Usd.Stage, walker_index: int, time_code: float) -> tuple[float, float, float]:
    prim = stage.GetPrimAtPath(f"{ROOT}/Walker_{walker_index}")
    result = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode(time_code))
    translation = result.ExtractTranslation()
    return float(translation[0]), float(translation[1]), float(translation[2])


exit_code = 0
timeline = None
try:
    print(f"[USD-SKEL-DEMO] Local Isaac asset root: {ASSET_ROOT}", flush=True)
    print(f"[USD-SKEL-DEMO] Warehouse: {WAREHOUSE}", flush=True)
    print(f"[USD-SKEL-DEMO] Local walk clip: {WALK_FORWARD}", flush=True)
    context = omni.usd.get_context()
    context.open_stage(WAREHOUSE.as_posix())
    stage = wait_until_loaded(context)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(LOOP_SECONDS * FPS)

    UsdGeom.Xform.Define(stage, ROOT)
    source_holder = stage.DefinePrim(f"{ROOT}/WalkForwardSource")
    source_holder.GetReferences().AddReference(WALK_FORWARD.as_posix())
    update_frames(12)
    source_prim = find_first(source_holder, UsdSkel.Animation)
    if not source_prim.IsValid():
        raise RuntimeError(f"No UsdSkel.Animation found in local clip {WALK_FORWARD}")
    looping_walk = bake_looping_walk(stage, UsdSkel.Animation(source_prim))

    skeleton_paths = [
        add_character(stage, index, character_path, looping_walk)
        for index, character_path in enumerate(CHARACTERS)
    ]
    routes = (
        [(-6.0, -5.0, 0.0), (-1.5, -5.0, 0.0), (-1.5, 1.0, 0.0), (-6.0, 1.0, 0.0)],
        [(2.0, -5.0, 0.0), (6.5, -5.0, 0.0), (6.5, 1.5, 0.0), (2.0, 1.5, 0.0)],
        [(-5.5, 3.0, 0.0), (0.5, 3.0, 0.0), (0.5, 7.5, 0.0), (-5.5, 7.5, 0.0)],
    )
    for index, route in enumerate(routes):
        set_route(stage.GetPrimAtPath(f"{ROOT}/Walker_{index}"), route)

    for skeleton_path in skeleton_paths:
        targets = UsdSkel.BindingAPI(stage.GetPrimAtPath(skeleton_path)).GetAnimationSourceRel().GetTargets()
        if targets != [looping_walk.GetPath()]:
            raise RuntimeError(f"Walk animation binding failed for {skeleton_path}: {targets}")
        print(f"[USD-SKEL-DEMO] Bound local looping walk to {skeleton_path}", flush=True)

    set_viewport_camera()
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(True)
    timeline.set_start_time(0.0)
    timeline.set_end_time(LOOP_SECONDS)
    timeline.play()
    started = time.monotonic()
    last_report = started
    initial_positions = [route_position(stage, index, 0.0) for index in range(len(CHARACTERS))]
    print(
        "[USD-SKEL-DEMO] Three USD-Skel walkers are running. "
        "No Replicator Agent or omni.anim.behavior.core was enabled.",
        flush=True,
    )
    while simulation_app.is_running():
        simulation_app.update()
        now = time.monotonic()
        if now - last_report >= 30.0:
            timeline_time = timeline.get_current_time() * FPS
            moved = sum(
                math.dist(initial_positions[index], route_position(stage, index, timeline_time)) > 0.05
                for index in range(len(CHARACTERS))
            )
            print(
                f"[USD-SKEL-DEMO] Stable for {now - started:.0f}s; "
                f"{moved}/{len(CHARACTERS)} character routes moved.",
                flush=True,
            )
            last_report = now
        if ARGS.duration > 0.0 and now - started >= ARGS.duration:
            break
except KeyboardInterrupt:
    print("[USD-SKEL-DEMO] Interrupted by user.", flush=True)
except Exception as exc:
    exit_code = 1
    carb.log_error(f"USD Skel walker demo failed: {exc}")
    print(f"[USD-SKEL-DEMO] ERROR: {exc}", file=sys.stderr, flush=True)
finally:
    if timeline is not None:
        timeline.stop()
    simulation_app.close()

raise SystemExit(exit_code)
