#!/usr/bin/env python3
"""Bounded Jackal wheel target/response probe using the Arena Isaac control nodes.

This is instrumentation only. It bypasses ROS delivery and Nav2, while using
the same imported Jackal URDF, differential nodes, articulation nodes and
wheel parameters as the compatibility adapter. Output belongs in a unique
run directory outside the repository.
"""

import argparse
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--no-floor", action="store_true", help="Diagnostic control without contact")
parser.add_argument("--formal-ground", action="store_true",
                    help="Use the same GroundPlane construction as formal Arena")
parser.add_argument("--wheel-friction-0p2", action="store_true",
                    help="Bind 0.2/0.2 physics material to four wheel colliders")
parser.add_argument("--solver-velocity-16", action="store_true",
                    help="Set Jackal articulation velocity solver iterations to 16")
args = parser.parse_args()
if args.no_floor and args.formal_ground:
    parser.error("--no-floor and --formal-ground are mutually exclusive")
if args.wheel_friction_0p2 and not args.formal_ground:
    parser.error("--wheel-friction-0p2 requires --formal-ground")
if args.solver_velocity_16 and not args.wheel_friction_0p2:
    parser.error("--solver-velocity-16 requires --wheel-friction-0p2")
if args.output.exists():
    raise SystemExit(f"Refusing to overwrite {args.output}")
args.output.mkdir(parents=True)

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "renderer": "Wireframe", "multi_gpu": False,
                     "fast_shutdown": True, "enable_crashreporter": False})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils import extensions
from pxr import UsdPhysics, UsdShade

extensions.enable_extension("isaacsim.asset.importer.urdf")
extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.robot.wheeled_robots")

SOURCE_URDF = Path("[LOCAL_PATH]")
MESH_DIR = SOURCE_URDF.parent
WHEEL_NAMES = ["front_left_wheel_joint", "front_right_wheel_joint",
               "rear_left_wheel_joint", "rear_right_wheel_joint"]
DT = 1 / 60


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def yaw(quat):
    w, x, y, z = [float(a) for a in quat]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def make_urdf():
    tree = ET.parse(SOURCE_URDF)
    for element in tree.getroot().iter():
        name = element.attrib.get("filename")
        if name:
            if name.startswith("package://jackal_description/"):
                name = name.split("package://jackal_description/", 1)[1]
            if not Path(name).is_absolute():
                name = str((MESH_DIR / name).resolve())
            element.attrib["filename"] = name
    path = args.output / "jackal_probe.urdf"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def create_control_graph(pair, index):
    graph = f"/World/jackal/diagnostic_diff_{index}"
    return graph, og.Controller.edit(
        {"graph_path": graph, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("art", "isaacsim.core.nodes.IsaacArticulationController"),
                ("left", "omni.graph.nodes.ConstantToken"),
                ("right", "omni.graph.nodes.ConstantToken"),
                ("names", "omni.graph.nodes.ConstructArray"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("diff.inputs:wheelDistance", 0.36 * 1.5),
                ("diff.inputs:wheelRadius", 0.098),
                ("diff.inputs:maxWheelSpeed", 10.0),
                ("diff.inputs:maxLinearSpeed", -2.0),
                ("diff.inputs:maxAngularSpeed", -4.0),
                ("diff.inputs:linearVelocity", 0.0),
                ("diff.inputs:angularVelocity", 0.0),
                ("art.inputs:targetPrim", "/World/jackal"),
                ("names.inputs:arraySize", 2),
                ("left.inputs:value", pair[0]),
                ("right.inputs:value", pair[1]),
            ],
            og.Controller.Keys.CREATE_ATTRIBUTES: [("names.inputs:input1", "token")],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "diff.inputs:execIn"),
                ("tick.outputs:tick", "art.inputs:execIn"),
                ("diff.outputs:velocityCommand", "art.inputs:velocityCommand"),
                ("left.inputs:value", "names.inputs:input0"),
                ("right.inputs:value", "names.inputs:input1"),
                ("names.outputs:array", "art.inputs:jointNames"),
            ],
        },
    )


def contact_stage_snapshot(stage):
    """Preserve authored contact, solver and material evidence before stepping."""
    snapshot = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        relevant = (path.startswith("/World/groundPlane") or
                    path.startswith("/World/Physics_Materials/") or
                    path.startswith("/colliders/") or
                    (path.startswith("/World/jackal/") and
                     ("wheel_link" in path or
                      prim.HasAPI(UsdPhysics.ArticulationRootAPI))))
        if not relevant:
            continue
        attrs = {}
        for attr in prim.GetAttributes():
            name = str(attr.GetName())
            if any(term in name.lower() for term in
                   ("physics", "physx", "friction", "restitution", "material",
                    "offset", "radius", "height", "width", "solver", "collision")):
                attrs[name] = str(attr.Get())
        snapshot[path] = {
            "type": prim.GetTypeName(),
            "attributes": attrs,
            "relationships": {str(rel.GetName()): [str(p) for p in rel.GetTargets()]
                              for rel in prim.GetRelationships()
                              if "material" in str(rel.GetName()).lower()},
        }
    return snapshot


def wheel_stage_inventory(stage):
    return [{"path": str(prim.GetPath()), "type": prim.GetTypeName(),
             "schemas": list(prim.GetAppliedSchemas()),
             "active": prim.IsActive(), "defined": prim.IsDefined(),
             "loaded": prim.IsLoaded(), "instance": prim.IsInstance(),
             "children": [str(child.GetPath()) for child in prim.GetChildren()],
             "references": str(prim.GetMetadata("references"))}
            for prim in stage.TraverseAll()
            if ("wheel" in str(prim.GetPath()) or
                prim.GetTypeName() == "Cylinder" or
                prim.HasAPI(UsdPhysics.CollisionAPI))]


def bind_wheel_material(stage):
    material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/probe_wheel_0p2",
        static_friction=0.2, dynamic_friction=0.2,
    )
    paths = []
    for name in WHEEL_NAMES:
        link = name.replace("_joint", "_link")
        source_path = f"/colliders/{link}/mesh_0/cylinder"
        composed_path = f"/World/jackal/{link}/collisions/mesh_0/cylinder"
        prim = stage.GetPrimAtPath(source_path)
        composed = stage.GetPrimAtPath(composed_path)
        if not (prim.IsValid() and prim.HasAPI(UsdPhysics.CollisionAPI) and
                composed.IsValid() and composed.HasAPI(UsdPhysics.CollisionAPI)):
            raise RuntimeError(f"Missing source or composed wheel collider: {source_path}, {composed_path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material.material, UsdShade.Tokens.weakerThanDescendants, "physics")
        paths.append(source_path)
    return paths


result = {"probe_type": "instrumentation_only_no_ros_no_nav2", "status": "INCOMPLETE",
          "floor_present": not args.no_floor,
          "floor_type": "formal_ground_plane" if args.formal_ground else
                        ("distant_fixed_cuboid" if args.no_floor else "fixed_cuboid"),
          "wheel_friction_0p2": args.wheel_friction_0p2,
          "solver_velocity_16": args.solver_velocity_16,
          "source_urdf": str(SOURCE_URDF), "source_sha256": sha256(SOURCE_URDF),
          "windows": [{"angular_z": -0.2, "sim_seconds": 3.0},
                      {"angular_z": 0.0, "sim_seconds": 1.0},
                      {"angular_z": -0.5, "sim_seconds": 3.0},
                      {"angular_z": 0.0, "sim_seconds": 1.0}]}

try:
    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    if args.formal_ground:
        world.scene.add_ground_plane(size=100, z_position=0.0)
    else:
        world.scene.add(FixedCuboid(prim_path="/World/floor", name="probe_floor",
                                   position=np.array([0., 0., -100.05 if args.no_floor else -0.05]),
                                   scale=np.array([10., 10., 0.1])))
    urdf = make_urdf()
    result["probe_urdf_sha256"] = sha256(urdf)
    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.set_merge_fixed_joints(False)
    config.set_convex_decomp(False)
    config.set_import_inertia_tensor(False)
    config.set_make_default_prim(False)
    config.set_distance_scale(1.0)
    config.set_fix_base(False)
    config.set_default_drive_type(2)
    config.set_self_collision(False)
    _, imported_path = omni.kit.commands.execute("URDFParseAndImportFile",
                                                  urdf_path=str(urdf), import_config=config,
                                                  dest_path="")
    if not imported_path:
        raise RuntimeError("URDF import returned no prim")
    result["wheel_inventory_before_move"] = wheel_stage_inventory(world.stage)
    if str(imported_path) != "/World/jackal":
        omni.kit.commands.execute("MovePrim", path_from=str(imported_path),
                                  path_to="/World/jackal", keep_world_transform=True)
    result["wheel_inventory_after_move"] = wheel_stage_inventory(world.stage)
    result["composed_wheel_colliders"] = {
        name: world.stage.GetPrimAtPath(
            f"/World/jackal/{name.replace('_joint', '_link')}/collisions/mesh_0/cylinder"
        ).IsValid() for name in WHEEL_NAMES
    }
    robot = world.scene.add(Robot(prim_path="/World/jackal", name="jackal_probe",
                                  position=np.array([0., 0., 0.0635])))
    graph_paths = [create_control_graph(WHEEL_NAMES[:2], 0)[0],
                   create_control_graph(WHEEL_NAMES[2:], 1)[0]]
    if args.wheel_friction_0p2:
        result["wheel_material_bound_paths"] = bind_wheel_material(world.stage)
        result["composed_wheel_material_bindings"] = {}
        for name in WHEEL_NAMES:
            link = name.replace("_joint", "_link")
            path = f"/World/jackal/{link}/collisions/mesh_0/cylinder"
            rel = world.stage.GetPrimAtPath(path).GetRelationship("material:binding:physics")
            result["composed_wheel_material_bindings"][path] = (
                [str(target) for target in rel.GetTargets()] if rel.IsValid() else []
            )
    if args.solver_velocity_16:
        root = world.stage.GetPrimAtPath("/World/jackal/base_link")
        attr = root.GetAttribute("physxArticulation:solverVelocityIterationCount")
        if not attr.IsValid() or attr.Get() != 1:
            raise RuntimeError(f"Expected existing velocity solver iteration count 1, got {attr.Get()}")
        attr.Set(16)
    result["graphs"] = graph_paths
    result["articulation_roots"] = [str(p.GetPath()) for p in world.stage.Traverse()
                                     if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    result["contact_stage_snapshot"] = contact_stage_snapshot(world.stage)
    result["wheel_joint_usd"] = {}
    for prim in world.stage.Traverse():
        if any(str(prim.GetPath()).endswith(name) for name in WHEEL_NAMES):
            result["wheel_joint_usd"][str(prim.GetPath())] = {
                str(attr.GetName()): str(attr.Get()) for attr in prim.GetAttributes()
                if any(term in str(attr.GetName()).lower()
                       for term in ("drive", "damping", "maxforce", "localpos", "axis"))
            }
    world.reset()
    robot.set_world_pose(position=np.array([0., 0., 0.0635]))
    world.play()
    result["dof_names"] = list(robot.dof_names)
    indices = [result["dof_names"].index(n) for n in WHEEL_NAMES]
    result["wheel_indices"] = indices
    for _ in range(60):
        world.step(render=True)

    samples = []
    frame = 0
    for window_index, window in enumerate(result["windows"]):
        command = window["angular_z"]
        for graph in graph_paths:
            og.Controller.attribute(f"{graph}/diff.inputs:angularVelocity").set(command)
        for _ in range(round(window["sim_seconds"] / DT)):
            world.step(render=True)
            frame += 1
            if frame % 6:
                continue
            position, quaternion = robot.get_world_pose()
            targets = [og.Controller.attribute(f"{graph}/diff.outputs:velocityCommand").get()
                       for graph in graph_paths]
            positions = np.asarray(robot.get_joint_positions(), dtype=float)[indices]
            velocities = np.asarray(robot.get_joint_velocities(), dtype=float)[indices]
            samples.append({"window": window_index, "sim_time": frame * DT,
                            "wall_time": time.time(), "input_angular_z": command,
                            "targets_rad_s": [float(v) for pair in targets for v in pair],
                            "actual_position_rad": positions.tolist(),
                            "actual_velocity_rad_s": velocities.tolist(),
                            "chassis_xyz": [float(v) for v in position],
                            "chassis_xy": [float(position[0]), float(position[1])],
                            "chassis_yaw": yaw(quaternion)})
    result["samples"] = samples
    result["status"] = "CAPTURED"
    world.stop()
except BaseException as exc:
    result["error"] = repr(exc)
    raise
finally:
    (args.output / "probe.json").write_text(json.dumps(result, indent=2) + "\n")
    app.close()
