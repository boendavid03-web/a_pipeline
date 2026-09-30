#!/usr/bin/env python3
"""Probe source-derived Arena 5 scene colliders in Isaac 5.1 without people."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
WORLDS = ROOT / "isaac_sim/arena_ws/src/arena/simulation-setup/worlds"
ASSETS = ROOT / "isaac_sim/backends/isaac5/generated/arena_worlds"
SHELF = Path("/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_fix/shelf.physics_materials_fixed.usd")
KIT_MDL = Path("/home/user/isaacsim/5.1.0/kit/mdl/core/Base")
EXPERIENCE = ROOT / "isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit"
USD = {
    "factory": ASSETS / "factory/factory_full_scene.usda",
    "hospital": ASSETS / "hospital/hospital_full_scene.usda",
    "ignc": ASSETS / "ignc/ignc_scene.usda",
}


def args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=("factory", "hospital", "ignc", "house17", "map_empty"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world-usd", type=Path, help="Explicit source-derived USD variant for this scene")
    return parser.parse_args()


ARGS = args()
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(
    {
        "headless": True,
        "multi_gpu": False,
        "renderer": "Wireframe",
        "create_new_stage": True,
        "fast_shutdown": False,
        "enable_crashreporter": False,
        "limit_cpu_threads": 8,
        "extra_args": ["--/exts/omni.kit.registry.nucleus/enable=false"],
    },
    experience=str(EXPERIENCE),
)

import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402


def _probe_mesh_point(stage, model_name):
    """Return a point on an upward facing collision triangle, in world units."""
    xforms = UsdGeom.XformCache()
    for prim in stage.Traverse():
        if prim.GetCustomDataByKey("arenaSdfModel") != model_name:
            continue
        for mesh_prim in Usd.PrimRange(prim):
            if not (mesh_prim.IsA(UsdGeom.Mesh) and mesh_prim.HasAPI(UsdPhysics.CollisionAPI)):
                continue
            mesh = UsdGeom.Mesh(mesh_prim)
            points = mesh.GetPointsAttr().Get() or []
            counts = mesh.GetFaceVertexCountsAttr().Get() or []
            indices = mesh.GetFaceVertexIndicesAttr().Get() or []
            matrix = xforms.GetLocalToWorldTransform(mesh_prim)
            offset = 0
            for count in counts:
                face = indices[offset:offset + count]
                offset += count
                if len(face) < 3:
                    continue
                a, b, c = [matrix.Transform(points[int(i)]) for i in face[:3]]
                normal = Gf.Cross(b - a, c - a)
                if normal.GetLength() < 1e-8 or abs(normal[2] / normal.GetLength()) < 0.75:
                    continue
                point = (a + b + c) / 3.0
                if all(math.isfinite(float(value)) and abs(float(value)) < 1000 for value in point):
                    return (float(point[0]), float(point[1]), float(point[2])), str(mesh_prim.GetPath())
    raise RuntimeError(f"No finite horizontal collision triangle for {model_name}")


def _add_wall_map(world, scene):
    walls = yaml.safe_load((WORLDS / scene / "map/walls.yaml").read_text())["walls"]
    for index, (start, end) in enumerate(walls):
        dx, dy = float(end[0] - start[0]), float(end[1] - start[1])
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) / 2.0
        world.scene.add(FixedCuboid(
            prim_path=f"/World/Walls/wall_{index:03d}",
            name=f"arena_wall_{index:03d}",
            position=np.asarray([(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, 1.1]),
            orientation=np.asarray([math.cos(yaw), 0.0, 0.0, math.sin(yaw)]),
            scale=np.asarray([length, 0.05, 2.0]),
        ))
    if scene == "house17":
        static = json.loads((WORLDS / scene / "scenarios/default.json").read_text())["obstacles"]["static"]
    else:
        static = yaml.safe_load((WORLDS / scene / "map/obstacles.yaml").read_text())["static"]
    for index, item in enumerate(static):
        if item["model"].lower() != "shelf":
            raise RuntimeError(f"Unexpected source model: {item['model']}")
        prim = UsdGeom.Xform.Define(world.stage, f"/World/Obstacles/shelf_{index:03d}")
        prim.AddTranslateOp().Set(Gf.Vec3d(*map(float, item.get("pos", item.get("position"))[:3])))
        prim.GetPrim().GetReferences().AddReference(str(SHELF))
    return walls, [item.get("pos", item.get("position"))[:3] for item in static]


def _raycast(origin, direction, distance):
    hit = omni.physx.get_physx_scene_query_interface().raycast_closest(origin, direction, distance)
    if not hit.get("hit"):
        return None
    return {"path": str(hit.get("rigidBody")), "position": [float(x) for x in hit["position"]], "distance": float(hit["distance"])}


def run():
    result = {"scene": ARGS.scene, "source": str(WORLDS / ARGS.scene), "status": "FAIL"}
    try:
        stage_utils.create_new_stage()
        world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        if ARGS.world_usd and ARGS.scene not in USD:
            raise ValueError("--world-usd is only valid for a world with a full USD")
        asset = ARGS.world_usd or USD.get(ARGS.scene, SHELF)
        layers, dependencies, unresolved = UsdUtils.ComputeAllDependencies(str(asset))
        kit_mdl = [str(item) for item in unresolved if str(item) in ("OmniPBR.mdl", "OmniPBR_Opacity.mdl")
                   and (KIT_MDL / str(item)).is_file()]
        missing = [str(item) for item in unresolved if str(item) not in kit_mdl]
        result["dependencies"] = {"asset": str(asset), "layers": len(layers),
                                  "files": len(dependencies), "kit_mdl_modules": kit_mdl,
                                  "missing": missing}
        result["dependency_pass"] = len(missing) == 0
        if ARGS.scene in USD:
            source_usd = ARGS.world_usd or USD[ARGS.scene]
            if not source_usd.is_file():
                raise FileNotFoundError(source_usd)
            stage_utils.add_reference_to_stage(str(source_usd), "/World/Arena")
            result["usd"] = str(source_usd)
            if ARGS.scene == "factory":
                origin, direction, distance = (40.0, 40.0, 2.0), (0.0, 0.0, -1.0), 4.0
            else:
                model = ("aws_robomaker_hospital_floor_01_floor_1" if ARGS.scene == "hospital" else "my_mesh")
                point, collider = _probe_mesh_point(world.stage, model)
                result["sampled_collider"] = collider
                origin, direction, distance = (point[0], point[1], point[2] + 1.0), (0.0, 0.0, -1.0), 2.0
        else:
            walls, shelf_positions = _add_wall_map(world, ARGS.scene)
            result["source_walls"] = len(walls)
            result["source_shelves"] = len(shelf_positions)
            start, end = walls[0]
            midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            direction = (-(end[1] - start[1]), end[0] - start[0])
            norm = math.hypot(*direction)
            direction = (direction[0] / norm, direction[1] / norm, 0.0)
            origin = (midpoint[0] - 0.4 * direction[0], midpoint[1] - 0.4 * direction[1], 0.8)
            distance = 1.0
        world.reset()
        for _ in range(5):
            world.step(render=False, step_sim=True)
        result["ray"] = {"origin": origin, "direction": direction, "distance": distance}
        result["hit"] = _raycast(origin, direction, distance)
        result["raycast_pass"] = bool(result["hit"])
        if ARGS.scene not in USD:
            shelf_hits = []
            offsets = ((0.0, 0.0), (-0.5, 0.0), (0.5, 0.0), (0.0, -0.5), (0.0, 0.5),
                       (-0.5, -0.5), (0.5, 0.5))
            for index, position in enumerate(shelf_positions):
                found = None
                for dx, dy in offsets:
                    hit = _raycast((float(position[0]) + dx, float(position[1]) + dy, 3.0),
                                   (0.0, 0.0, -1.0), 5.0)
                    if hit and f"/Obstacles/shelf_{index:03d}" in hit["path"]:
                        found = hit
                        break
                shelf_hits.append({"index": index, "hit": found})
            result["shelf_hits"] = shelf_hits
            result["shelf_raycast_pass"] = all(item["hit"] is not None for item in shelf_hits)
        if ARGS.scene in USD and result["hit"]:
            hit_z = result["hit"]["position"][2]
            cube = world.scene.add(DynamicCuboid(
                prim_path="/World/ContactProbe",
                name="contact_probe",
                position=np.asarray([origin[0], origin[1], hit_z + 1.0]),
                scale=np.asarray([0.2, 0.2, 0.2]),
                mass=1.0,
            ))
            world.reset()
            samples = []
            for step in range(120):
                world.step(render=False, step_sim=True)
                if step in (0, 29, 59, 119):
                    position, _ = cube.get_world_pose()
                    samples.append({"step": step + 1, "z": float(position[2])})
            result["fall"] = {"surface_z": hit_z, "samples": samples}
            result["dynamic_contact_pass"] = samples[-1]["z"] >= hit_z + 0.07
        elif result["hit"]:
            cube = world.scene.add(DynamicCuboid(
                prim_path="/World/ContactProbe",
                name="contact_probe",
                position=np.asarray(origin),
                scale=np.asarray([0.2, 0.2, 0.2]),
                mass=1.0,
            ))
            world.reset()
            cube.set_linear_velocity(np.asarray([direction[0] * 2.0, direction[1] * 2.0, 0.0]))
            samples = []
            for step in range(20):
                world.step(render=False, step_sim=True)
                position, _ = cube.get_world_pose()
                travel = float((position[0] - origin[0]) * direction[0] + (position[1] - origin[1]) * direction[1])
                samples.append({"step": step + 1, "travel": travel, "z": float(position[2])})
            hit_distance = result["hit"]["distance"]
            result["wall_push"] = {"hit_distance": hit_distance, "samples": [samples[i] for i in (0, 4, 9, 19)],
                                   "max_travel": max(item["travel"] for item in samples)}
            result["dynamic_contact_pass"] = (samples[0]["travel"] > 0.005 and
                                              result["wall_push"]["max_travel"] <= hit_distance - 0.04)
        result["status"] = "PASS" if (result["dependency_pass"] and result["raycast_pass"] and result.get("dynamic_contact_pass") and
                                       (ARGS.scene in USD or result.get("shelf_raycast_pass"))) else "FAIL"
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        ARGS.output.parent.mkdir(parents=True, exist_ok=True)
        ARGS.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print("ARENA_SCENE_PHYSICS_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)
        app.close()
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(run())
