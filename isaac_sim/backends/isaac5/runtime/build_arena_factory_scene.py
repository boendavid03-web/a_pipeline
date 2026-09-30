#!/usr/bin/env python3
"""Build an Isaac USD scene from Arena's workcell or a direct-model SDF world."""

from __future__ import annotations

import argparse
import asyncio
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim import SimulationApp


def _vec(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    values = tuple(float(value) for value in text.split())
    if len(values) < count:
        return default
    return values[:count]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_sdf", type=Path, nargs="?")
    parser.add_argument("visual_usd", type=Path, nargs="?")
    parser.add_argument("output_usd", type=Path, nargs="?")
    parser.add_argument("--world", type=Path, help="Full Arena factory.world SDF")
    parser.add_argument("--models-root", type=Path, help="Arena gazebo_models directory")
    parser.add_argument("--workcell-usd", type=Path, help="Existing textured workcell USD")
    parser.add_argument("--mesh-override", action="append", default=[], metavar="URI=USD",
                        help="Use an already converted USD for an SDF mesh URI")
    parser.add_argument("--include-name", action="append", default=[],
                        help="Select a top-level SDF model by name, for staged world conversion")
    parser.add_argument("--output", type=Path, help="Full world USD destination")
    return parser


def _build(model_sdf: Path, visual_usd: Path, output_usd: Path) -> int:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Keep one default root so a reference at /World/Arena exposes the
    # contents directly below that prim instead of creating /Arena/Arena.
    world = UsdGeom.Xform.Define(stage, "/Factory")
    visual = UsdGeom.Xform.Define(stage, "/Factory/WorkcellVisual")
    visual.GetPrim().GetReferences().AddReference(str(visual_usd))

    root = ET.parse(model_sdf).getroot()
    collisions = root.findall(".//collision")
    box_count = 0
    for index, collision in enumerate(collisions):
        box = collision.find("./geometry/box/size")
        if box is None:
            continue
        size = _vec(box.text, 3, (1.0, 1.0, 1.0))
        pose = _vec(collision.findtext("./pose"), 6, (0.0,) * 6)
        name = collision.get("name") or f"box_{index:03d}"
        safe_name = "".join(char if char.isalnum() or char == "_" else "_" for char in name)
        prim_path = f"/Factory/WorkcellCollision/{safe_name}_{index:03d}"
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
        xform.AddRotateXYZOp().Set(
            Gf.Vec3f(*(math.degrees(angle) for angle in pose[3:6]))
        )
        xform.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        box_count += 1

    stage.SetDefaultPrim(world.GetPrim())
    stage.GetRootLayer().Save()
    print(f"ARENA_FACTORY_SCENE_PASS boxes={box_count} output={output_usd}", flush=True)
    return 0 if box_count else 1


def _safe(name: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return value if value and not value[0].isdigit() else "n_" + value


def _transform(prim, pose_text: str | None, scale=(1.0, 1.0, 1.0)) -> None:
    from pxr import Gf, UsdGeom

    pose = _vec(pose_text, 6, (0.0,) * 6)
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*(math.degrees(a) for a in pose[3:])))
    if scale != (1.0, 1.0, 1.0):
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _mesh_source(uri: str, models_root: Path, world_dir: Path) -> Path:
    if not uri or ".." in Path(uri.removeprefix("model://")).parts:
        raise ValueError(f"Unsupported Arena model URI: {uri}")
    base = models_root if uri.startswith("model://") else world_dir
    relative = uri.removeprefix("model://")
    path = (base / relative).resolve()
    if not path.is_file() or not path.is_relative_to(base.resolve()):
        raise FileNotFoundError(path)
    return path


def _convert_collada_geometry(source: Path, destination: Path) -> bool:
    """Geometry fallback for COLLADA files rejected by Kit's asset converter."""
    from pxr import Gf, Usd, UsdGeom

    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(source).getroot()
    unit = root.find("./c:asset/c:unit", ns)
    meters = float(unit.get("meter", "1")) if unit is not None else 1.0
    if root.findtext("./c:asset/c:up_axis", namespaces=ns) != "Z_UP":
        return False
    stage = Usd.Stage.CreateNew(str(destination))
    prim_root = UsdGeom.Xform.Define(stage, "/Asset")
    count = 0
    for geometry in root.findall("./c:library_geometries/c:geometry", ns):
        mesh = geometry.find("c:mesh", ns)
        if mesh is None:
            continue
        sources = {}
        for item in mesh.findall("c:source", ns):
            array = item.find("c:float_array", ns)
            accessor = item.find("./c:technique_common/c:accessor", ns)
            if array is not None and accessor is not None:
                values = [float(v) for v in array.text.split()]
                stride = int(accessor.get("stride", "1"))
                sources["#" + item.get("id")] = [values[i:i + stride] for i in range(0, len(values), stride)]
        vertices = mesh.find("c:vertices", ns)
        position_ref = None
        if vertices is not None:
            for item in vertices.findall("c:input", ns):
                if item.get("semantic") == "POSITION":
                    position_ref = item.get("source")
        if position_ref not in sources:
            continue
        points = [Gf.Vec3f(*(v[i] * meters for i in range(3))) for v in sources[position_ref]]
        face_indices = []
        for faces in list(mesh):
            if faces.tag.rsplit("}", 1)[-1] not in ("polylist", "triangles"):
                continue
            inputs = faces.findall("c:input", ns)
            vertex_offsets = [int(i.get("offset", "0")) for i in inputs if i.get("semantic") == "VERTEX"]
            if not vertex_offsets:
                continue
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            values = [int(v) for v in faces.findtext("c:p", default="", namespaces=ns).split()]
            indices = values[vertex_offsets[0]::stride]
            sizes = ([int(v) for v in faces.findtext("c:vcount", default="", namespaces=ns).split()]
                     if faces.tag.rsplit("}", 1)[-1] == "polylist" else [3] * int(faces.get("count", "0")))
            offset = 0
            for size in sizes:
                for k in range(1, size - 1):
                    face_indices.extend((indices[offset], indices[offset + k], indices[offset + k + 1]))
                offset += size
        if not face_indices:
            continue
        usd_mesh = UsdGeom.Mesh.Define(stage, f"/Asset/Mesh_{count:03d}")
        usd_mesh.CreatePointsAttr(points)
        usd_mesh.CreateFaceVertexCountsAttr([3] * (len(face_indices) // 3))
        usd_mesh.CreateFaceVertexIndicesAttr(face_indices)
        count += 1
    if not count:
        return False
    stage.SetDefaultPrim(prim_root.GetPrim())
    stage.GetRootLayer().Save()
    return True


async def _convert_missing(meshes: set[str], models_root: Path, world_dir: Path,
                           assets_dir: Path, overrides: dict[str, Path]) -> dict[str, Path]:
    from convert_arena_mesh import _convert

    converted: dict[str, Path] = {}
    for uri in sorted(meshes):
        source = _mesh_source(uri, models_root, world_dir)
        if uri in overrides:
            converted[uri] = overrides[uri]
            continue
        destination = assets_dir / _safe(uri.removeprefix("model://").replace("/", "_"))
        destination = destination.with_suffix(".usd")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.is_file() or destination.stat().st_size == 0:
            if not await _convert(source, destination, False):
                if source.suffix.lower() != ".dae" or not _convert_collada_geometry(source, destination):
                    raise RuntimeError(f"Mesh conversion failed: {source}")
                print(f"ARENA_FACTORY_COLLADA_FALLBACK={source}", flush=True)
        converted[uri] = destination
        print(f"ARENA_FACTORY_MESH_READY uri={uri} usd={destination}", flush=True)
    return converted


def _geometry(stage, path: str, item: ET.Element, converted: dict[str, Path],
              collision: bool) -> bool:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    geometry = item.find("geometry")
    if geometry is None or len(geometry) != 1:
        return False
    kind = geometry[0].tag
    element = geometry[0]
    scale = (1.0, 1.0, 1.0)
    if kind == "box":
        scale = _vec(element.findtext("size"), 3, (1.0,) * 3)
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1.0)
    elif kind == "cylinder":
        radius = float(element.findtext("radius", "0.5"))
        length = float(element.findtext("length", "1.0"))
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(length)
    elif kind == "plane":
        size = _vec(element.findtext("size"), 2, (100.0, 100.0))
        scale = (size[0], size[1], 0.02)
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1.0)
    elif kind == "mesh":
        uri = element.findtext("uri")
        if uri not in converted:
            raise KeyError(uri)
        scale = _vec(element.findtext("scale"), 3, (1.0,) * 3)
        shape = UsdGeom.Xform.Define(stage, path)
        asset = UsdGeom.Xform.Define(stage, path + "/Asset")
        asset.GetPrim().GetReferences().AddReference(str(converted[uri]))
    else:
        raise ValueError(f"Unsupported SDF geometry: {kind}")
    _transform(shape.GetPrim(), item.findtext("pose"), scale)
    if collision:
        if kind == "mesh":
            mesh_prims = [prim for prim in Usd.PrimRange(shape.GetPrim()) if prim.IsA(UsdGeom.Mesh)]
            if not mesh_prims:
                raise RuntimeError(f"Collision asset has no mesh: {path}")
            for prim in mesh_prims:
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")
        else:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdGeom.Imageable(shape.GetPrim()).MakeInvisible()
    else:
        transparency = float(item.findtext("transparency", "0"))
        if transparency >= 1:
            UsdGeom.Imageable(shape.GetPrim()).MakeInvisible()
        diffuse = item.findtext("./material/diffuse") or item.findtext("./material/ambient")
        if diffuse and kind != "mesh":
            rgb = _vec(diffuse, 3, (0.7, 0.7, 0.7))
            material_path = "/Factory/Materials/color_" + "_".join(str(round(v * 255)) for v in rgb)
            material = UsdShade.Material.Define(stage, material_path)
            shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
    return True


def _selected_models(world_xml: ET.Element, models_root: Path,
                     included_names: list[str]) -> list[tuple[ET.Element, ET.Element]]:
    models = world_xml.findall("model")
    if included_names:
        missing = set(included_names) - {model.get("name") for model in models}
        if missing:
            raise ValueError(f"SDF models not found: {sorted(missing)}")
        models = [model for model in models if model.get("name") in included_names]
    selected = []
    for model in models:
        include = model.find("include")
        if include is None or model.findall("link"):
            selected.append((model, model))
            continue
        uri = include.findtext("uri", "")
        if not uri.startswith("model://"):
            raise ValueError(f"Unsupported included SDF model: {uri}")
        model_sdf = models_root / uri[8:] / "model.sdf"
        if not model_sdf.is_file():
            raise FileNotFoundError(model_sdf)
        body = ET.parse(model_sdf).getroot().find("model")
        if body is None:
            raise ValueError(f"No model in {model_sdf}")
        selected.append((model, body))
    return selected


def _build_world(world_sdf: Path, output_usd: Path,
                 converted: dict[str, Path],
                 selected: list[tuple[ET.Element, ET.Element]]) -> int:
    from pxr import Usd, UsdGeom

    world_xml = ET.parse(world_sdf).getroot().find("world")
    if world_xml is None:
        raise ValueError(f"No SDF world in {world_sdf}")
    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Factory")
    model_count = visual_count = collision_count = 0
    expected_visuals = expected_collisions = 0
    for index, (model, body) in enumerate(selected):
        name = model.get("name", f"model_{index}")
        model_path = f"/Factory/Models/{_safe(name)}_{index:03d}"
        model_prim = UsdGeom.Xform.Define(stage, model_path)
        _transform(model_prim.GetPrim(), model.findtext("pose"))
        model_prim.GetPrim().SetCustomDataByKey("arenaSdfModel", name)
        model_count += 1
        body_path = model_path
        if body is not model:
            include_prim = UsdGeom.Xform.Define(stage, model_path + "/Included")
            _transform(include_prim.GetPrim(), model.findtext("./include/pose"))
            body_prim = UsdGeom.Xform.Define(stage, model_path + "/Included/Model")
            _transform(body_prim.GetPrim(), body.findtext("pose"))
            body_path += "/Included/Model"
        for link_index, link in enumerate(body.findall("link")):
            link_path = f"{body_path}/{_safe(link.get('name', 'link'))}_{link_index:02d}"
            link_prim = UsdGeom.Xform.Define(stage, link_path)
            _transform(link_prim.GetPrim(), link.findtext("pose"))
            for kind in ("visual", "collision"):
                for item_index, item in enumerate(link.findall(kind)):
                    if item.find("geometry") is not None:
                        if kind == "visual":
                            expected_visuals += 1
                        else:
                            expected_collisions += 1
                    item_path = f"{link_path}/{kind}_{item_index:03d}"
                    if _geometry(stage, item_path, item, converted, kind == "collision"):
                        if kind == "visual":
                            visual_count += 1
                        else:
                            collision_count += 1
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()
    print(f"ARENA_SDF_WORLD_PASS models={model_count} visuals={visual_count} "
          f"collisions={collision_count} output={output_usd}", flush=True)
    return 0 if model_count and (visual_count, collision_count) == (expected_visuals, expected_collisions) else 1


def main() -> int:
    args = _parser().parse_args()
    if args.world:
        if not all((args.models_root, args.output)):
            raise ValueError("--world requires --models-root and --output")
        world_sdf = args.world.resolve()
        models_root = args.models_root.resolve()
        output_usd = args.output.resolve()
        if not world_sdf.is_file():
            raise FileNotFoundError(world_sdf)
        overrides = {}
        for value in args.mesh_override:
            uri, separator, usd = value.partition("=")
            if not separator or not uri or not usd:
                raise ValueError(f"Expected URI=USD for --mesh-override: {value}")
            overrides[uri] = Path(usd).resolve()
        if args.workcell_usd:
            overrides.setdefault("model://workcell/meshes/mesh.dae", args.workcell_usd.resolve())
        for path in overrides.values():
            if not path.is_file():
                raise FileNotFoundError(path)
        output_usd.parent.mkdir(parents=True, exist_ok=True)
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
        try:
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension("omni.kit.asset_converter")
            app.update()
            world = ET.parse(world_sdf).getroot().find("world")
            if world is None:
                raise ValueError(f"No SDF world in {world_sdf}")
            selected = _selected_models(world, models_root, args.include_name)
            meshes = {mesh.findtext("uri") for _wrapper, body in selected
                      for mesh in body.findall(".//mesh")}
            converted = asyncio.get_event_loop().run_until_complete(
                _convert_missing(meshes, models_root, world_sdf.parent,
                                 output_usd.parent / "meshes", overrides)
            )
            return _build_world(world_sdf, output_usd, converted, selected)
        except Exception:
            import traceback
            traceback.print_exc()
            return 1
        finally:
            app.close()
    if not all((args.model_sdf, args.visual_usd, args.output_usd)):
        raise ValueError("Expected model_sdf visual_usd output_usd or --world options")
    model_sdf = args.model_sdf.resolve()
    visual_usd = args.visual_usd.resolve()
    output_usd = args.output_usd.resolve()
    if not model_sdf.is_file():
        raise FileNotFoundError(model_sdf)
    if not visual_usd.is_file():
        raise FileNotFoundError(visual_usd)
    output_usd.parent.mkdir(parents=True, exist_ok=True)

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        return _build(model_sdf, visual_usd, output_usd)
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
