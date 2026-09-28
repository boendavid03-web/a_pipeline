#!/usr/bin/env python3
"""Validate the active lobby plus an upright, bounded-motion Mecanum730 in Isaac 5.1."""

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
SCENE_USD = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
ROBOT_USD = (
    PROJECT_ROOT.parent
    / "robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
)
ROBOT_PRIM = "/World/Robot"
SPAWN = (2.0, 2.0, 0.01)
WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
ARM_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
PHYSICS_DT = 1.0 / 60.0
WHEEL_RADIUS_M = 0.077993
ROBOT_CLEARANCE_RADIUS_M = 0.40
ROBOT_COLLISION_PRIM = "/World/RobotCollisionProxy"
ROBOT_COLLISION_MATERIAL_PRIM = "/World/Looks/RobotCollisionProxyMaterial"
ROBOT_COLLISION_PLANAR_PADDING_M = 0.02
ROBOT_PHYSICS_MASS_KG = 116.189
ROBOT_HEADING_HOLD_KP = 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--motion-seconds", type=float, default=2.0)
    parser.add_argument("--stop-seconds", type=float, default=2.0)
    parser.add_argument("--hold-wall-seconds", type=float, default=10.0)
    parser.add_argument("--setup-timeout", type=float, default=30.0)
    parser.add_argument("--screenshot", type=Path, default=None)
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
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402


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


def quaternion_to_rpy(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(value) for value in quat_wxyz]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def tune_articulation() -> None:
    root = stage_utils.get_current_stage().GetPrimAtPath(f"{ROBOT_PRIM}/base_footprint")
    if not root.IsValid() or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(f"Missing articulation root: {root.GetPath()}")
    root.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(16)
    root.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(8)
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)


def lock_navigation_arm(robot: Robot, indices: np.ndarray, targets: np.ndarray) -> None:
    """Copy the proven Isaac 5 navigation lock that emulates Isaac 6's authored pose."""

    zeros = np.zeros(indices.size, dtype=np.float32)
    robot.set_joint_positions(targets, joint_indices=indices)
    robot.set_joint_velocities(zeros, joint_indices=indices)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=targets, joint_velocities=zeros, joint_indices=indices)
    )


def wheel_targets(vx: float) -> np.ndarray:
    return np.asarray([vx, vx, vx, vx], dtype=np.float32) / WHEEL_RADIUS_M


def robot_visual_bounds(stage: Usd.Stage) -> tuple[np.ndarray, np.ndarray]:
    """Copy Isaac 6's authored visual-bounds calculation for the proxy body."""

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    aligned = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(ROBOT_PRIM)).ComputeAlignedBox()
    dimensions = np.asarray(aligned.GetSize(), dtype=float)
    center = 0.5 * (
        np.asarray(aligned.GetMin(), dtype=float) + np.asarray(aligned.GetMax(), dtype=float)
    )
    return dimensions, center


def disable_canonical_robot_collisions(stage: Usd.Stage) -> int:
    """Make the Isaac 6-style proxy the sole robot/environment contact authority."""

    disabled = 0
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(ROBOT_PRIM + "/") and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            disabled += 1
    return disabled


def create_dynamic_collision_proxy(
    stage: Usd.Stage,
    visual_dimensions: np.ndarray,
    visual_center: np.ndarray,
    root_position: np.ndarray,
) -> tuple[SingleRigidPrim, np.ndarray, np.ndarray]:
    """Port Isaac 6's zero-friction dynamic navigation body without changing source USDs."""

    dimensions = np.asarray(visual_dimensions, dtype=float).copy()
    dimensions[:2] += 2.0 * ROBOT_COLLISION_PLANAR_PADDING_M
    # Isaac 6 measures bounds after placing its XFormPrim at the requested
    # spawn.  In Isaac 5, Robot(position=...) is not authored until reset, so
    # these pre-reset bounds are still expressed around the referenced USD's
    # current root transform.  Remove that authored transform first, then
    # apply the requested spawn explicitly.
    root_matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath(ROBOT_PRIM)
    )
    authored_root_position = np.asarray(root_matrix.ExtractTranslation(), dtype=float)
    center_offset = np.asarray(visual_center, dtype=float) - authored_root_position
    center = np.asarray(root_position, dtype=float) + center_offset

    cube = UsdGeom.Cube.Define(stage, ROBOT_COLLISION_PRIM)
    cube.CreateSizeAttr(1.0)
    cube.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    xformable = UsdGeom.Xformable(cube)
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(value) for value in center])
    )
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
    )
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(value) for value in dimensions])
    )
    proxy_prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(proxy_prim).CreateCollisionEnabledAttr(True)

    material = UsdShade.Material.Define(stage, ROBOT_COLLISION_MATERIAL_PRIM)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(0.0)
    physics_material.CreateDynamicFrictionAttr().Set(0.0)
    physics_material.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
    UsdShade.MaterialBindingAPI.Apply(proxy_prim).Bind(material, materialPurpose="physics")

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(proxy_prim)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(proxy_prim).CreateMassAttr(ROBOT_PHYSICS_MASS_KG)
    PhysxSchema.PhysxRigidBodyAPI.Apply(proxy_prim).CreateLockedRotAxisAttr(3)
    proxy_prim.CreateAttribute("primvars:isVolume", Sdf.ValueTypeNames.Bool).Set(True)

    proxy = SingleRigidPrim(
        prim_path=ROBOT_COLLISION_PRIM,
        name="mecanum730_xms5_gate3_physics_body",
        reset_xform_properties=False,
    )
    return proxy, center_offset, dimensions


def proxy_root_pose(proxy: SingleRigidPrim, center_offset: np.ndarray) -> tuple[np.ndarray, float]:
    center, orientation = proxy.get_world_pose()
    yaw = quaternion_to_rpy(np.asarray(orientation, dtype=float))[2]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotated_offset = np.asarray(
        [
            cosine * center_offset[0] - sine * center_offset[1],
            sine * center_offset[0] + cosine * center_offset[1],
            center_offset[2],
        ],
        dtype=float,
    )
    return np.asarray(center, dtype=float) - rotated_offset, yaw


def follow_proxy(robot: Robot, proxy: SingleRigidPrim, center_offset: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    root, yaw = proxy_root_pose(proxy, center_offset)
    orientation = np.asarray([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=float)
    robot.set_world_pose(position=root, orientation=orientation)
    robot.set_linear_velocity(np.zeros(3, dtype=float))
    robot.set_angular_velocity(np.zeros(3, dtype=float))
    return root, orientation


def apply_proxy_command(
    proxy: SingleRigidPrim,
    vx: float,
    target_yaw: float,
) -> None:
    _, orientation = proxy.get_world_pose()
    yaw = quaternion_to_rpy(np.asarray(orientation, dtype=float))[2]
    yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
    current_velocity = np.asarray(proxy.get_linear_velocity(), dtype=float)
    proxy.set_linear_velocity(
        np.asarray([math.cos(yaw) * vx, math.sin(yaw) * vx, current_velocity[2]], dtype=float)
    )
    proxy.set_angular_velocity(
        np.asarray([0.0, 0.0, float(np.clip(ROBOT_HEADING_HOLD_KP * yaw_error, -1.0, 1.0))])
    )


def apply_articulation_locks(
    robot: Robot,
    vx: float,
    wheel_indices: np.ndarray,
    arm_indices: np.ndarray,
    arm_targets: np.ndarray,
) -> None:
    targets = wheel_targets(vx)
    robot.set_joint_velocities(targets, joint_indices=wheel_indices)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_velocities=targets, joint_indices=wheel_indices)
    )
    lock_navigation_arm(robot, arm_indices, arm_targets)


def capture_viewport(
    world: World,
    robot: Robot,
    proxy: SingleRigidPrim,
    center_offset: np.ndarray,
    target_yaw: float,
    wheel_indices: np.ndarray,
    arm_indices: np.ndarray,
    arm_targets: np.ndarray,
    output: Path,
) -> None:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    output.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport for Gate 3 screenshot")
    capture = capture_viewport_to_file(viewport, file_path=str(output))
    task = asyncio.ensure_future(capture.wait_for_result(completion_frames=8))
    for _ in range(300):
        apply_proxy_command(proxy, 0.0, target_yaw)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        world.step(render=True)
        follow_proxy(robot, proxy, center_offset)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        if task.done():
            break
    if not task.done():
        task.cancel()
        raise RuntimeError(f"Gate 3 screenshot timed out: {output}")
    if not task.result():
        raise RuntimeError(f"Gate 3 screenshot capture failed: {output}")
    import omni.kit.renderer_capture

    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not output.is_file() or output.stat().st_size == 0:
        raise RuntimeError(f"Gate 3 screenshot was not written: {output}")


def planar_obstacle_clearance(stage: Usd.Stage, points: list[np.ndarray]) -> float:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    minimum = math.inf
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not path.startswith("/World/Environment/"):
            continue
        if "Ground" in path:
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        lo, hi = box.GetMin(), box.GetMax()
        for point in points:
            dx = max(float(lo[0]) - point[0], 0.0, point[0] - float(hi[0]))
            dy = max(float(lo[1]) - point[1], 0.0, point[1] - float(hi[1]))
            minimum = min(minimum, math.hypot(dx, dy) - ROBOT_CLEARANCE_RADIUS_M)
    return float(minimum)


def main() -> int:
    if not SCENE_USD.is_file() or not ROBOT_USD.is_file():
        raise FileNotFoundError(f"Missing scene or robot: {SCENE_USD}, {ROBOT_USD}")
    if not (0.0 < ARGS.speed <= 0.25):
        raise ValueError("--speed must be in (0, 0.25] m/s")
    if not (0.1 <= ARGS.motion_seconds <= 3.0 and 0.5 <= ARGS.stop_seconds <= 5.0):
        raise ValueError("motion/stop duration is outside the bounded Gate 3 envelope")
    if ARGS.headless and SCREENSHOT is not None:
        raise ValueError("--screenshot requires GUI mode")

    success, error = wait_for_task(
        asyncio.ensure_future(omni.usd.get_context().open_stage_async(str(SCENE_USD))),
        ARGS.setup_timeout,
        "opening the active lobby",
    )
    if not success:
        raise RuntimeError(f"Could not open active lobby: {error}")
    for _ in range(5):
        simulation_app.update()
    simulation_app.reset_render_settings()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Scene loaded without a stage")

    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    stage_utils.add_reference_to_stage(str(ROBOT_USD), ROBOT_PRIM)
    robot = world.scene.add(
        Robot(
            prim_path=ROBOT_PRIM,
            name="mecanum730_xms5_gate3",
            position=np.asarray(SPAWN),
            orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
        )
    )
    tune_articulation()
    for _ in range(5):
        simulation_app.update()
    visual_dimensions, visual_center = robot_visual_bounds(stage)
    disabled_robot_collisions = disable_canonical_robot_collisions(stage)
    if disabled_robot_collisions <= 0:
        raise RuntimeError("Canonical robot has no collision APIs to hand over to the proxy")
    proxy, proxy_center_offset, proxy_dimensions = create_dynamic_collision_proxy(
        stage,
        visual_dimensions,
        visual_center,
        np.asarray(SPAWN, dtype=float),
    )
    world.scene.add(proxy)
    world.reset()

    dof_names = list(robot.dof_names)
    wheel_indices = np.asarray([dof_names.index(name) for name in WHEEL_NAMES], dtype=np.int32)
    arm_indices = np.asarray([dof_names.index(name) for name in ARM_NAMES], dtype=np.int32)
    arm_targets = np.zeros(len(ARM_NAMES), dtype=np.float32)
    _, initial_proxy_orientation = proxy.get_world_pose()
    target_yaw = quaternion_to_rpy(np.asarray(initial_proxy_orientation, dtype=float))[2]

    for _ in range(120):
        apply_proxy_command(proxy, 0.0, target_yaw)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        world.step(render=not ARGS.headless)
        follow_proxy(robot, proxy, proxy_center_offset)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
    initial_position, initial_orientation = follow_proxy(robot, proxy, proxy_center_offset)
    initial_wheel_positions = np.asarray(robot.get_joint_positions(), dtype=float)[wheel_indices]

    if not ARGS.headless:
        set_camera_view(
            # Spawn is inside the southwest room.  Keep the camera inside its
            # y=0 and x=0/8.5 walls; the previous exterior view was occluded.
            eye=np.asarray([5.5, 4.5, 3.2]),
            target=np.asarray([2.2, 2.0, 0.75]),
            camera_prim_path="/OmniverseKit_Persp",
        )

    pose_samples = [initial_position.copy()]
    orientation_samples = [initial_orientation.copy()]
    motion_frames = int(round(ARGS.motion_seconds / PHYSICS_DT))
    stop_frames = int(round(ARGS.stop_seconds / PHYSICS_DT))
    for _ in range(motion_frames):
        apply_proxy_command(proxy, ARGS.speed, target_yaw)
        apply_articulation_locks(robot, ARGS.speed, wheel_indices, arm_indices, arm_targets)
        world.step(render=not ARGS.headless)
        position, orientation = follow_proxy(robot, proxy, proxy_center_offset)
        apply_articulation_locks(robot, ARGS.speed, wheel_indices, arm_indices, arm_targets)
        pose_samples.append(position.copy())
        orientation_samples.append(orientation.copy())

    for _ in range(stop_frames):
        apply_proxy_command(proxy, 0.0, target_yaw)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        world.step(render=not ARGS.headless)
        position, orientation = follow_proxy(robot, proxy, proxy_center_offset)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        pose_samples.append(position.copy())
        orientation_samples.append(orientation.copy())

    hold_start = time.monotonic()
    while not ARGS.headless and time.monotonic() - hold_start < ARGS.hold_wall_seconds:
        apply_proxy_command(proxy, 0.0, target_yaw)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        world.step(render=True)
        follow_proxy(robot, proxy, proxy_center_offset)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)

    if SCREENSHOT is not None:
        capture_viewport(
            world,
            robot,
            proxy,
            proxy_center_offset,
            target_yaw,
            wheel_indices,
            arm_indices,
            arm_targets,
            SCREENSHOT,
        )

    apply_proxy_command(proxy, 0.0, target_yaw)
    apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
    final_position, final_orientation = follow_proxy(robot, proxy, proxy_center_offset)
    final_linear_velocity = np.asarray(proxy.get_linear_velocity(), dtype=float)
    final_angular_velocity = np.asarray(proxy.get_angular_velocity(), dtype=float)
    final_joints = np.asarray(robot.get_joint_positions(), dtype=float)
    final_joint_velocities = np.asarray(robot.get_joint_velocities(), dtype=float)
    wheel_delta = final_joints[wheel_indices] - initial_wheel_positions
    arm_error = float(np.max(np.abs(final_joints[arm_indices] - arm_targets)))
    arm_velocity = float(np.max(np.abs(final_joint_velocities[arm_indices])))
    displacement = final_position - initial_position
    travel_m = float(np.linalg.norm(displacement[:2]))
    max_tilt_deg = max(
        max(abs(math.degrees(quaternion_to_rpy(q)[0])), abs(math.degrees(quaternion_to_rpy(q)[1])))
        for q in orientation_samples
    )
    clearance = planar_obstacle_clearance(stage, pose_samples)

    teardown_ok = True
    teardown_error = None
    try:
        apply_proxy_command(proxy, 0.0, target_yaw)
        apply_articulation_locks(robot, 0.0, wheel_indices, arm_indices, arm_targets)
        world.stop()
    except Exception as exc:
        teardown_ok = False
        teardown_error = repr(exc)

    expected_travel = ARGS.speed * ARGS.motion_seconds
    checks = {
        "spawn_xy_correct": float(np.linalg.norm(initial_position[:2] - np.asarray(SPAWN[:2]))) <= 0.05,
        "bounded_forward_motion": 0.5 * expected_travel <= travel_m <= 1.5 * expected_travel,
        "limited_lateral_drift": abs(float(displacement[1])) <= 0.05,
        "height_stable": abs(float(final_position[2]) - float(initial_position[2])) <= 0.02,
        "no_rollover": max_tilt_deg <= 8.0,
        "obstacle_clearance": clearance >= 0.25,
        "wheel_joints_moved": bool(np.max(np.abs(wheel_delta)) >= 0.5),
        "arm_upright_locked": arm_error <= 1.0e-5 and arm_velocity <= 1.0e-5,
        "zero_speed_exit": float(np.linalg.norm(final_linear_velocity[:2])) <= 0.01
        and float(np.linalg.norm(final_angular_velocity)) <= 0.01
        and float(np.max(np.abs(final_joint_velocities[wheel_indices]))) <= 0.05,
        "teardown_ok": teardown_ok,
    }
    status, failure_reasons = pass_or_fail(checks, [teardown_error] if teardown_error else [])
    report = {
        "status": status,
        "failure_reasons": failure_reasons,
        "backend": "isaac5",
        "isaac_version": "5.1",
        "scene_usd": str(SCENE_USD),
        "scene_sha256": hashlib.sha256(SCENE_USD.read_bytes()).hexdigest(),
        "robot_usd": str(ROBOT_USD),
        "robot_sha256": hashlib.sha256(ROBOT_USD.read_bytes()).hexdigest(),
        "viewport_screenshot": str(SCREENSHOT) if SCREENSHOT is not None else None,
        "navigation_body": "Isaac 6 dynamic zero-friction collision proxy port",
        "proxy_prim": ROBOT_COLLISION_PRIM,
        "proxy_dimensions_m": proxy_dimensions.tolist(),
        "proxy_center_offset_m": proxy_center_offset.tolist(),
        "proxy_mass_kg": ROBOT_PHYSICS_MASS_KG,
        "canonical_robot_collisions_disabled_session_only": disabled_robot_collisions,
        "canonical_robot_role": "full articulation visual and joint authority following physical proxy",
        "spawn_requested": list(SPAWN),
        "initial_position_after_settle": initial_position.tolist(),
        "final_position": final_position.tolist(),
        "displacement": displacement.tolist(),
        "travel_m": travel_m,
        "command_mps": ARGS.speed,
        "motion_seconds": ARGS.motion_seconds,
        "stop_seconds": ARGS.stop_seconds,
        "motion_frames": motion_frames,
        "stop_frames": stop_frames,
        "max_tilt_deg": max_tilt_deg,
        "minimum_geometric_obstacle_clearance_m": clearance,
        "clearance_evidence": "swept robot-circle versus authored collision AABBs; not a contact-force measurement",
        "wheel_joint_names": list(WHEEL_NAMES),
        "wheel_joint_indices": wheel_indices.tolist(),
        "wheel_position_delta_rad": wheel_delta.tolist(),
        "final_wheel_velocity_radps": final_joint_velocities[wheel_indices].tolist(),
        "arm_joint_names": list(ARM_NAMES),
        "arm_joint_targets_rad": arm_targets.tolist(),
        "max_arm_joint_error_rad": arm_error,
        "max_arm_joint_velocity_radps": arm_velocity,
        "final_linear_velocity": final_linear_velocity.tolist(),
        "final_angular_velocity": final_angular_velocity.tolist(),
        "zero_command_sent_before_stop": True,
    }
    print("SCENE_ROBOT_VALIDATION_RESULT=" + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if status == "PASS" else 2


try:
    print(
        "SCENE_ROBOT_VALIDATION_READY="
        + json.dumps(
            {
                "backend": "isaac5",
                "scene": str(SCENE_USD),
                "robot": str(ROBOT_USD),
                "spawn": list(SPAWN),
                "bounded_command": [ARGS.speed, 0.0, 0.0],
                "arm_mode": "authored upright pose hard lock",
                "ros": False,
                "lidar": False,
                "pedestrians": False,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    exit_code = main()
except Exception as exc:
    print(
        "SCENE_ROBOT_VALIDATION_RESULT="
        + json.dumps({"status": "FAIL", "failure_reasons": [repr(exc)]}, ensure_ascii=False),
        flush=True,
    )
    exit_code = 1
finally:
    simulation_app.close()
raise SystemExit(exit_code)
