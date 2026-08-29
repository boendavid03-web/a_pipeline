#!/usr/bin/env python3
"""Convert a planar Gazebo SDF world made of static boxes into a local USDA scene.

The converter is intentionally narrow: it preserves the metric X/Y layout,
height, yaw, and collision geometry used by 2D navigation worlds.  Gazebo
plugins, actors, model:// includes, joints, and meshes are not silently
approximated; they remain separate migration work.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_SOURCE = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)
DEFAULT_OUTPUT = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
# The Gazebo engineering-lobby map is the enclosed 32 m x 24 m footprint.
# Keeping the visual floor / NavMesh inside those walls is important: the
# source Gazebo ground plane is intentionally infinite, but pedestrians must
# never use the exterior apron as a shortcut around the building.
DEFAULT_NAVMESH_BOUNDS = (0.0, 0.0, 32.0, 24.0)
PEDESTRIAN_NAVMESH_CLEARANCE_M = 0.55


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class Box:
    name: str
    pose: PlanarPose
    size: tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scene-name", default="a_pipeline_eng_lobby")
    parser.add_argument(
        "--expected-boxes",
        type=int,
        default=None,
        help="Fail if the active static-box count differs from this value.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Do not write; fail if --output is not the deterministic result.",
    )
    parser.add_argument(
        "--navmesh-bounds",
        nargs=4,
        type=float,
        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"),
        default=DEFAULT_NAVMESH_BOUNDS,
        help=(
            "Finite walkable map bounds in metres.  The generated floor and "
            "NavMesh include volume use these bounds (default: 0 0 32 24)."
        ),
    )
    return parser.parse_args()


def parse_numbers(text: str | None, count: int, description: str) -> tuple[float, ...]:
    if text is None:
        raise ValueError(f"missing {description}")
    values = tuple(float(item) for item in text.split())
    if len(values) != count or not all(math.isfinite(item) for item in values):
        raise ValueError(f"invalid {description}: {text!r}")
    return values


def parse_planar_pose(element: ET.Element | None, description: str) -> PlanarPose:
    if element is None or not (element.text or "").strip():
        return PlanarPose(0.0, 0.0, 0.0, 0.0)
    x, y, z, roll, pitch, yaw = parse_numbers(element.text, 6, description)
    if abs(roll) > 1.0e-9 or abs(pitch) > 1.0e-9:
        raise ValueError(
            f"{description} has roll/pitch; this planar converter only supports yaw"
        )
    return PlanarPose(x, y, z, yaw)


def compose_pose(parent: PlanarPose, child: PlanarPose) -> PlanarPose:
    cosine = math.cos(parent.yaw)
    sine = math.sin(parent.yaw)
    return PlanarPose(
        parent.x + cosine * child.x - sine * child.y,
        parent.y + sine * child.x + cosine * child.y,
        parent.z + child.z,
        parent.yaw + child.yaw,
    )


def usd_identifier(value: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9_]", "_", value.strip())
    if not identifier:
        raise ValueError("empty model/link/collision name")
    if identifier[0].isdigit():
        identifier = "_" + identifier
    return identifier


def load_static_boxes(source: Path) -> tuple[list[Box], int]:
    try:
        root = ET.parse(source).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"cannot read SDF world {source}: {exc}") from exc
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise ValueError(f"{source} does not contain an SDF <world>")

    boxes: list[Box] = []
    names: set[str] = set()
    for model in world.findall("model"):
        model_name = model.get("name", "model")
        if model_name == "ground_plane":
            continue
        static_text = (model.findtext("static") or "true").strip().lower()
        if static_text not in {"1", "true"}:
            raise ValueError(f"dynamic model is not supported: {model_name}")
        model_pose = parse_planar_pose(model.find("pose"), f"{model_name} model pose")
        for link_index, link in enumerate(model.findall("link")):
            link_name = link.get("name", f"link_{link_index}")
            link_pose = compose_pose(
                model_pose,
                parse_planar_pose(link.find("pose"), f"{model_name}/{link_name} pose"),
            )
            for collision_index, collision in enumerate(link.findall("collision")):
                size_element = collision.find("./geometry/box/size")
                if size_element is None:
                    continue
                collision_name = collision.get("name", f"collision_{collision_index}")
                collision_pose = compose_pose(
                    link_pose,
                    parse_planar_pose(
                        collision.find("pose"),
                        f"{model_name}/{link_name}/{collision_name} pose",
                    ),
                )
                size = parse_numbers(
                    size_element.text,
                    3,
                    f"{model_name}/{link_name}/{collision_name} box size",
                )
                if any(axis <= 0.0 for axis in size):
                    raise ValueError(f"non-positive box size in {model_name}: {size}")
                base_name = usd_identifier(model_name)
                if len(link.findall("collision")) > 1 or len(model.findall("link")) > 1:
                    base_name += "__" + usd_identifier(link_name)
                    base_name += "__" + usd_identifier(collision_name)
                if base_name in names:
                    raise ValueError(f"duplicate generated USD prim name: {base_name}")
                names.add(base_name)
                boxes.append(Box(base_name, collision_pose, size))
    return boxes, len(world.findall("include"))


def format_number(value: float) -> str:
    if abs(value) < 5.0e-13:
        value = 0.0
    return format(value, ".12g")


def display_source(source: Path) -> str:
    try:
        return str(source.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(source.resolve())


def cube_usda(box: Box) -> str:
    x, y, z = box.pose.x, box.pose.y, box.pose.z
    sx, sy, sz = box.size
    yaw_degrees = math.degrees(box.pose.yaw)
    semantic_label = "Wall" if sz >= 1.0 else "Pillar"
    material = "WallMaterial" if sz >= 1.0 else "LowObstacleMaterial"
    return f'''        def Cube "{box.name}" (
            # A collision box is an obstacle, not a walkable top surface.
            # IRA uses this navigation-area opinion while sampling its initial
            # patrol pose and when routing between patrol points.
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI", "NavMeshAreaAPI"]
        )
        {{
            custom string aPipeline:semanticLabel = "{semantic_label}"
            string nav:area = "NotWalkable"
            rel material:binding = </World/Looks/{material}>
            double size = 1
            point3f[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
            color3f[] primvars:displayColor = [(0.55, 0.58, 0.62)]
            bool physics:collisionEnabled = true
            double3 xformOp:translate = ({format_number(x)}, {format_number(y)}, {format_number(z)})
            double3 xformOp:rotateXYZ = (0, 0, {format_number(yaw_degrees)})
            double3 xformOp:scale = ({format_number(sx)}, {format_number(sy)}, {format_number(sz)})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
        }}'''


def navmesh_exclude_usda(box: Box) -> str:
    """Create a deterministic no-walk volume around a Gazebo collision box."""
    x, y = box.pose.x, box.pose.y
    yaw_degrees = math.degrees(box.pose.yaw)
    # An explicit exclude volume is required because IRA's initial-position
    # sampler does not consistently treat a static PhysicsCollisionAPI cube as
    # a navigation obstacle.  Inflate by the character radius plus a small
    # visual clearance, matching the patrol route validator.
    size_x = box.size[0] + 2.0 * PEDESTRIAN_NAVMESH_CLEARANCE_M
    size_y = box.size[1] + 2.0 * PEDESTRIAN_NAVMESH_CLEARANCE_M
    return f'''        def NavMeshVolume "NoWalk_{box.name}"
        {{
            float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
            token nav:volume:type = "Exclude"
            double3 xformOp:translate = ({format_number(x)}, {format_number(y)}, 2)
            double3 xformOp:rotateXYZ = (0, 0, {format_number(yaw_degrees)})
            float3 xformOp:scale = ({format_number(size_x)}, {format_number(size_y)}, 4)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
        }}'''


def build_usda(
    source: Path,
    scene_name: str,
    boxes: list[Box],
    includes: int,
    navmesh_bounds: tuple[float, float, float, float],
) -> str:
    source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
    cube_blocks = "\n\n".join(cube_usda(box) for box in boxes)
    exclude_blocks = "\n\n".join(navmesh_exclude_usda(box) for box in boxes)
    x_min, y_min, x_max, y_max = navmesh_bounds
    if not all(math.isfinite(value) for value in navmesh_bounds):
        raise ValueError("navmesh bounds must be finite")
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("navmesh bounds must have positive width and height")
    center_x = 0.5 * (x_min + x_max)
    center_y = 0.5 * (y_min + y_max)
    width = x_max - x_min
    height = y_max - y_min
    return f'''#usda 1.0
(
    defaultPrim = "World"
    doc = "Project-owned Isaac scene generated from the Gazebo V7 engineering lobby."
    endTimeCode = 86400
    framesPerSecond = 60
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 60
    upAxis = "Z"
    customLayerData = {{
        dictionary navmeshSettings = {{
            double agentMaxFloorSlope = 45
            double agentMaxRadius = 0.6
            double agentMaxStepHeight = 0.25
            double agentMinHeight = 1.5
            double agentMinIslandRadius = 0.5
            double agentMinRadius = 0.1
            double agentSamplingDistance = 0.2
            dictionary areas = {{
                dictionary "0" = {{
                    string areaName = "Walkable"
                    float3 color = (0.2, 0.8, 1)
                    double defaultCost = 1
                }}
                dictionary "1" = {{
                    string areaName = "NotWalkable"
                    float3 color = (1, 0, 0)
                    double defaultCost = -1
                }}
            }}
            bool excludeRigidBodies = 1
        }}
        string generatedBy = "isaac_sim/scripts/convert_gazebo_boxes_to_usda.py"
        int migratedStaticBoxes = {len(boxes)}
        int skippedGazeboIncludes = {includes}
        string sourceWorld = "{display_source(source)}"
        string sourceWorldSha256 = "{source_sha256}"
    }}
)

def Xform "World"
{{
    custom string aPipeline:sceneName = "{scene_name}"

    def NavMeshVolume "NavMeshVolume" (
        prepend apiSchemas = ["NavMeshAreaAPI"]
    )
    {{
        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
        token nav:volume:type = "Include"
        double3 xformOp:translate = ({format_number(center_x)}, {format_number(center_y)}, 1.5)
        float3 xformOp:scale = ({format_number(width)}, {format_number(height)}, 4)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}

    def PhysicsScene "PhysicsScene"
    {{
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
    }}

    def Xform "NavMeshObstacles"
    {{
{exclude_blocks}
    }}

    def Scope "Looks"
    {{
        def Material "FloorMaterial"
        {{
            token outputs:surface.connect = </World/Looks/FloorMaterial/PreviewSurface.outputs:surface>
            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0.18, 0.20, 0.23)
                float inputs:metallic = 0
                float inputs:roughness = 0.82
                token outputs:surface
            }}
        }}
        def Material "WallMaterial"
        {{
            token outputs:surface.connect = </World/Looks/WallMaterial/PreviewSurface.outputs:surface>
            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0.56, 0.61, 0.67)
                float inputs:metallic = 0
                float inputs:roughness = 0.68
                token outputs:surface
            }}
        }}
        def Material "LowObstacleMaterial"
        {{
            token outputs:surface.connect = </World/Looks/LowObstacleMaterial/PreviewSurface.outputs:surface>
            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0.82, 0.46, 0.18)
                float inputs:metallic = 0
                float inputs:roughness = 0.72
                token outputs:surface
            }}
        }}
    }}

    def Xform "Environment"
    {{
        def Mesh "Ground" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI", "NavMeshAreaAPI"]
        )
        {{
            custom string aPipeline:semanticLabel = "Floor"
            string nav:area = "Walkable"
            rel material:binding = </World/Looks/FloorMaterial>
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 3, 2, 4, 6, 7, 5, 6, 2, 3, 7, 4, 5, 1, 0, 4, 0, 2, 6, 5, 7, 3, 1]
            point3f[] points = [(-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5)]
            uniform token subdivisionScheme = "none"
            bool physics:collisionEnabled = true
            double3 xformOp:translate = ({format_number(center_x)}, {format_number(center_y)}, -0.05)
            double3 xformOp:scale = ({format_number(width)}, {format_number(height)}, 0.1)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}

        def Xform "GazeboV7Layout"
        {{
{cube_blocks}
        }}
    }}

    def Xform "Lights"
    {{
        def DomeLight "Dome"
        {{
            color3f inputs:color = (0.78, 0.86, 1)
            float inputs:intensity = 650
        }}
        def DistantLight "Sun"
        {{
            float inputs:angle = 1
            color3f inputs:color = (1, 0.94, 0.84)
            float inputs:intensity = 2800
            double3 xformOp:rotateXYZ = (-42, 28, 18)
            uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
        }}
    }}

    def Xform "Markers"
    {{
        def Xform "RobotSpawn"
        {{
            custom string aPipeline:role = "robot_spawn"
            double3 xformOp:translate = (2, 2, 0.3)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}
    }}
}}
'''


def main() -> int:
    args = parse_args()
    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    boxes, includes = load_static_boxes(source)
    if not boxes:
        raise ValueError(f"no active static box collisions found in {source}")
    if args.expected_boxes is not None and len(boxes) != args.expected_boxes:
        raise ValueError(
            f"expected {args.expected_boxes} active boxes, found {len(boxes)}"
        )
    desired = build_usda(
        source,
        args.scene_name,
        boxes,
        includes,
        tuple(args.navmesh_bounds),
    )
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != desired:
            print(f"CUSTOM_SCENE_CHECK=FAIL output is stale: {output}", file=sys.stderr)
            return 1
        print(
            f"CUSTOM_SCENE_CHECK=PASS boxes={len(boxes)} skipped_includes={includes} "
            f"output={output}"
        )
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(desired, encoding="utf-8")
    print(
        f"CUSTOM_SCENE_BUILD=PASS boxes={len(boxes)} skipped_includes={includes} "
        f"output={output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
