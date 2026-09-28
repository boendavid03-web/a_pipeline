#!/usr/bin/env python3
"""Inspect an Isaac Sim 5.1 character USD without modifying the asset."""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from pathlib import Path


DEFAULT_CHARACTER = Path(
    "/home/user/isaacsim/5.1.0/extscache/"
    "omni.metropolis.utils-0.1.20+107.3.3.lx64.cp311/"
    "test_data/characters/test/test.usd"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset", type=Path, default=DEFAULT_CHARACTER)
    parser.add_argument("--setup-timeout", type=float, default=30.0)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "fast_shutdown": True,
    }
)

import omni.usd  # noqa: E402
from pxr import UsdGeom, UsdSkel, UsdUtils  # noqa: E402


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


def main() -> int:
    asset = ARGS.asset.expanduser().resolve()
    report: dict[str, object] = {"status": "FAIL", "asset": str(asset)}
    try:
        if not asset.is_file():
            raise FileNotFoundError(asset)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(asset))
        success, error = wait_for_task(
            asyncio.ensure_future(omni.usd.get_context().open_stage_async(str(asset))),
            ARGS.setup_timeout,
            "opening the character asset",
        )
        if not success:
            raise RuntimeError(f"Could not open character asset: {error}")
        for _ in range(5):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Character open completed without a USD stage")

        skeletons: list[dict[str, object]] = []
        animations: list[dict[str, object]] = []
        meshes: list[str] = []
        bindings: list[dict[str, object]] = []
        sampled_attributes: list[dict[str, object]] = []
        for prim in stage.TraverseAll():
            if prim.IsA(UsdSkel.Skeleton):
                skeleton = UsdSkel.Skeleton(prim)
                joints = [str(value) for value in (skeleton.GetJointsAttr().Get() or [])]
                retarget_attributes = {}
                for attribute in prim.GetAttributes():
                    name = attribute.GetName()
                    if name.startswith("controlRig:") or name.startswith("animation:"):
                        value = attribute.Get()
                        if value is None:
                            retarget_attributes[name] = None
                        elif hasattr(value, "__len__") and not isinstance(value, str):
                            retarget_attributes[name] = {
                                "count": len(value),
                                "preview": [str(item) for item in value[:12]],
                            }
                        else:
                            retarget_attributes[name] = str(value)
                skeletons.append(
                    {
                        "path": str(prim.GetPath()),
                        "joint_count": len(joints),
                        "joints": joints,
                        "applied_schemas": [str(value) for value in prim.GetAppliedSchemas()],
                        "retarget_attributes": retarget_attributes,
                    }
                )
            if prim.IsA(UsdSkel.Animation):
                animation = UsdSkel.Animation(prim)
                joints = [str(value) for value in (animation.GetJointsAttr().Get() or [])]
                animations.append(
                    {
                        "path": str(prim.GetPath()),
                        "joint_count": len(joints),
                        "joints": joints,
                    }
                )
            if prim.IsA(UsdGeom.Mesh):
                meshes.append(str(prim.GetPath()))
            binding = UsdSkel.BindingAPI(prim)
            skeleton_targets = [
                str(path) for path in binding.GetSkeletonRel().GetTargets()
            ]
            animation_targets = [
                str(path) for path in binding.GetAnimationSourceRel().GetTargets()
            ]
            if skeleton_targets or animation_targets:
                bindings.append(
                    {
                        "path": str(prim.GetPath()),
                        "skeleton_targets": skeleton_targets,
                        "animation_targets": animation_targets,
                    }
                )
            for attribute in prim.GetAttributes():
                count = int(attribute.GetNumTimeSamples())
                if count:
                    times = attribute.GetTimeSamples()
                    sampled_attributes.append(
                        {
                            "prim": str(prim.GetPath()),
                            "attribute": attribute.GetName(),
                            "sample_count": count,
                            "first_time": float(times[0]),
                            "last_time": float(times[-1]),
                        }
                    )

        checks = {
            "dependency_closure": not unresolved,
            "skeleton_present": bool(skeletons),
            "skinned_mesh_present": bool(meshes),
            "animation_prim_present": bool(animations),
            "animation_samples_present": any(
                row["attribute"] in {"rotations", "translations", "scales"}
                and row["sample_count"] >= 2
                for row in sampled_attributes
            ),
        }
        report = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "asset": str(asset),
            "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)).upper(),
            "start_time_code": float(stage.GetStartTimeCode()),
            "end_time_code": float(stage.GetEndTimeCode()),
            "time_codes_per_second": float(stage.GetTimeCodesPerSecond()),
            "default_prim": str(stage.GetDefaultPrim().GetPath()),
            "dependency_layers": sorted(str(layer.identifier) for layer in layers),
            "asset_dependencies": sorted(str(path) for path in assets),
            "unresolved_dependencies": sorted(str(path) for path in unresolved),
            "skeletons": skeletons,
            "animations": animations,
            "mesh_count": len(meshes),
            "mesh_paths": meshes,
            "bindings": bindings,
            "sampled_attributes": sampled_attributes,
            "checks": checks,
        }
        return 0 if report["status"] == "PASS" else 2
    except Exception as exc:
        report["error"] = repr(exc)
        return 1
    finally:
        print("ISAAC5_CHARACTER_ASSET_AUDIT=" + json.dumps(report, ensure_ascii=False), flush=True)
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
