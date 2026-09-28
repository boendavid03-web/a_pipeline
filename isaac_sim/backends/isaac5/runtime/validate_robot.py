#!/usr/bin/env python3
"""Load the project robot in Isaac Sim 5.1 and run a minimal stability check."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path

from contract import finite_pose, generated_output_path, pass_or_fail

BACKEND_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
GENERATED_ROOT = BACKEND_ROOT / "generated"
DEFAULT_ROBOT_USD = (
    PROJECT_ROOT.parent
    / "robot_related"
    / "robots"
    / "chassis_arm"
    / "motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
)
ROBOT_PRIM_PATH = "/World/Robot"
EXPECTED_WHEEL_JOINTS = (
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
)
SAFE_ARM_JOINTS = {
    # Match the successful Isaac 6 navigation runner's authored visual pose:
    # the arm is a fixed vertical navigation payload, not a task actuator.
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
    "gripper_joint1": 0.0,
    "gripper_joint2": 0.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without a viewport window.")
    parser.add_argument("--steps", type=int, default=360, help="Number of 60 Hz physics steps.")
    parser.add_argument(
        "--min-wall-seconds",
        type=float,
        default=0.0,
        help="For GUI validation, keep rendering until this wall-clock duration has elapsed.",
    )
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT_USD)
    parser.add_argument(
        "--output-scene",
        type=Path,
        default=None,
        help="Optional generated USD output; relative paths are below the backend generated tree.",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="Optional GUI viewport PNG; relative paths are below the backend generated tree.",
    )
    return parser.parse_args()


ARGS = parse_args()
OUTPUT_SCENE = generated_output_path(BACKEND_ROOT, ARGS.output_scene)
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
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402


def quaternion_to_roll_pitch(quat_wxyz: np.ndarray) -> tuple[float, float]:
    w, x, y, z = [float(value) for value in quat_wxyz]
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch


def add_lighting() -> None:
    stage = stage_utils.get_current_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    distant.CreateIntensityAttr(2500.0)
    distant.CreateAngleAttr(1.0)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 35.0, 20.0))


def tune_articulation() -> None:
    stage = stage_utils.get_current_stage()
    root = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/base_footprint")
    if not root.IsValid() or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(f"Missing articulation root: {root.GetPath()}")
    # Author the PhysX attributes explicitly so this remains independent of
    # Isaac Lab while matching the source task's stable solver settings.
    root.CreateAttribute(
        "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int
    ).Set(16)
    root.CreateAttribute(
        "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int
    ).Set(8)
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(
        False
    )


def lock_navigation_arm(
    robot: Robot, arm_indices: np.ndarray, arm_positions: np.ndarray
) -> None:
    """Keep the non-navigation arm at its authored upright pose."""

    if not arm_indices.size:
        return
    zero_velocities = np.zeros(arm_indices.size, dtype=np.float32)
    robot.set_joint_positions(arm_positions, joint_indices=arm_indices)
    robot.set_joint_velocities(zero_velocities, joint_indices=arm_indices)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(
            joint_positions=arm_positions,
            joint_velocities=zero_velocities,
            joint_indices=arm_indices,
        )
    )


def apply_safe_joint_pose(
    robot: Robot,
) -> tuple[dict[str, int], np.ndarray, np.ndarray]:
    dof_names = list(robot.dof_names)
    missing = [name for name in EXPECTED_WHEEL_JOINTS if name not in dof_names]
    if missing:
        raise RuntimeError(f"Missing expected wheel joints: {missing}")

    safe_indices = []
    safe_positions = []
    for name, position in SAFE_ARM_JOINTS.items():
        if name in dof_names:
            safe_indices.append(dof_names.index(name))
            safe_positions.append(position)

    indices = np.asarray(safe_indices, dtype=np.int32)
    positions = np.asarray(safe_positions, dtype=np.float32)
    lock_navigation_arm(robot, indices, positions)

    return (
        {name: dof_names.index(name) for name in EXPECTED_WHEEL_JOINTS},
        indices,
        positions,
    )


def save_stage(scene_path: Path) -> None:
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    if not stage_utils.save_stage(str(scene_path), save_and_reload_in_place=False):
        raise RuntimeError(f"Failed to save stage: {scene_path}")


def get_selected_link_positions(robot: Robot) -> dict[str, list[float]]:
    del robot
    stage = stage_utils.get_current_stage()
    xform_cache = UsdGeom.XformCache()
    selected = {"base_link", "wheel_fl_link", "wheel_fr_link", "wheel_rl_link", "wheel_rr_link"}
    positions = {}
    for name in sorted(selected):
        prim = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/{name}")
        if prim.IsValid():
            translation = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
            positions[name] = [float(translation[i]) for i in range(3)]
    return positions


def capture_viewport(
    world: World,
    output_path: Path,
    robot: Robot,
    arm_indices: np.ndarray,
    arm_positions: np.ndarray,
) -> None:
    """Capture the active RTX viewport and drive Kit until the async write completes."""

    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    output_path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport is available for screenshot capture")
    capture = capture_viewport_to_file(viewport, file_path=str(output_path))
    task = asyncio.ensure_future(capture.wait_for_result(completion_frames=8))
    for _ in range(300):
        lock_navigation_arm(robot, arm_indices, arm_positions)
        world.step(render=True)
        lock_navigation_arm(robot, arm_indices, arm_positions)
        if task.done():
            break
    if not task.done():
        task.cancel()
        raise RuntimeError(f"Viewport screenshot timed out: {output_path}")
    if not task.result():
        raise RuntimeError(f"Viewport screenshot capture failed: {output_path}")
    # Match Isaac 5.1's own viewport capture tests: the asyncio result means
    # rendering completed, while the renderer capture interface is the barrier
    # for the subsequent asynchronous file write.
    import omni.kit.renderer_capture

    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"Viewport screenshot was not written: {output_path}")


def main() -> int:
    robot_path = ARGS.robot_usd.resolve()
    if not robot_path.is_file():
        raise FileNotFoundError(f"Robot USD not found: {robot_path}")
    if ARGS.steps <= 0:
        raise ValueError("--steps must be positive")
    if ARGS.min_wall_seconds < 0.0:
        raise ValueError("--min-wall-seconds must be non-negative")

    stage_utils.create_new_stage()
    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane(
        z_position=0.0,
        static_friction=1.0,
        dynamic_friction=0.8,
        restitution=0.0,
    )
    add_lighting()
    stage_utils.add_reference_to_stage(str(robot_path), ROBOT_PRIM_PATH)
    robot = world.scene.add(
        Robot(
            prim_path=ROBOT_PRIM_PATH,
            name="mecanum730_xms5",
            position=np.asarray([0.0, 0.0, 0.01]),
            orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
        )
    )
    tune_articulation()

    if OUTPUT_SCENE is not None:
        save_stage(OUTPUT_SCENE)

    world.reset()
    wheel_indices, arm_indices, arm_positions = apply_safe_joint_pose(robot)
    for _ in range(30):
        lock_navigation_arm(robot, arm_indices, arm_positions)
        world.step(render=not ARGS.headless)
        lock_navigation_arm(robot, arm_indices, arm_positions)
    if not ARGS.headless:
        set_camera_view(
            eye=np.asarray([3.2, 3.2, 2.2]),
            target=np.asarray([0.0, 0.0, 0.55]),
            camera_prim_path="/OmniverseKit_Persp",
        )

    initial_position, _ = robot.get_world_pose()
    samples = []
    validation_started = time.monotonic()
    frame = 0
    while frame < ARGS.steps or (
        not ARGS.headless and time.monotonic() - validation_started < ARGS.min_wall_seconds
    ):
        lock_navigation_arm(robot, arm_indices, arm_positions)
        world.step(render=not ARGS.headless)
        lock_navigation_arm(robot, arm_indices, arm_positions)
        if frame % 30 == 0 or frame == ARGS.steps - 1:
            position, orientation = robot.get_world_pose()
            linear_velocity = robot.get_linear_velocity()
            angular_velocity = robot.get_angular_velocity()
            samples.append(
                {
                    "frame": frame,
                    "position": np.asarray(position, dtype=float).tolist(),
                    "orientation_wxyz": np.asarray(orientation, dtype=float).tolist(),
                    "linear_velocity": np.asarray(linear_velocity, dtype=float).tolist(),
                    "angular_velocity": np.asarray(angular_velocity, dtype=float).tolist(),
                    "arm_joint_positions": np.asarray(robot.get_joint_positions(), dtype=float)[
                        arm_indices
                    ].tolist(),
                    "arm_joint_velocities": np.asarray(robot.get_joint_velocities(), dtype=float)[
                        arm_indices
                    ].tolist(),
                }
            )
        frame += 1
    validation_wall_seconds = time.monotonic() - validation_started

    if SCREENSHOT is not None:
        if ARGS.headless:
            raise ValueError("--screenshot requires a GUI run (omit --headless)")
        capture_viewport(world, SCREENSHOT, robot, arm_indices, arm_positions)

    final_position, final_orientation = robot.get_world_pose()
    final_position = np.asarray(final_position, dtype=float)
    final_orientation = np.asarray(final_orientation, dtype=float)
    initial_position = np.asarray(initial_position, dtype=float)
    roll, pitch = quaternion_to_roll_pitch(final_orientation)
    xy_drift = float(np.linalg.norm(final_position[:2] - initial_position[:2]))
    z_drift = float(abs(final_position[2] - initial_position[2]))
    link_positions = get_selected_link_positions(robot)
    finite = bool(
        finite_pose(final_position, final_orientation)
        and all(np.all(np.isfinite(sample["linear_velocity"])) for sample in samples)
        and all(np.all(np.isfinite(sample["orientation_wxyz"])) for sample in samples)
    )
    no_rollover = all(
        abs(math.degrees(quaternion_to_roll_pitch(np.asarray(sample["orientation_wxyz"]))[0])) <= 12.0
        and abs(math.degrees(quaternion_to_roll_pitch(np.asarray(sample["orientation_wxyz"]))[1])) <= 12.0
        for sample in samples
    )
    articulation_root = stage_utils.get_current_stage().GetPrimAtPath(
        f"{ROBOT_PRIM_PATH}/base_footprint"
    )
    articulation_root_exists = bool(
        articulation_root.IsValid() and articulation_root.HasAPI(UsdPhysics.ArticulationRootAPI)
    )
    wheel_joint_names = list(robot.dof_names)
    wheel_joints_exist = all(name in wheel_joint_names for name in EXPECTED_WHEEL_JOINTS)
    final_joint_positions = np.asarray(robot.get_joint_positions(), dtype=float)
    final_arm_positions = final_joint_positions[arm_indices]
    max_arm_error = float(np.max(np.abs(final_arm_positions - arm_positions)))
    max_arm_velocity = max(
        float(np.max(np.abs(np.asarray(sample["arm_joint_velocities"], dtype=float))))
        for sample in samples
    )
    teardown_ok = True
    teardown_error = None
    try:
        world.stop()
    except Exception as exc:  # pragma: no cover - requires Kit teardown
        teardown_ok = False
        teardown_error = repr(exc)
    checks = {
        "articulation_root_exists": articulation_root_exists,
        "wheel_joints_exist": wheel_joints_exist,
        "arm_upright_locked": max_arm_error <= 1.0e-5 and max_arm_velocity <= 1.0e-5,
        "finite_pose": finite,
        "no_obvious_rollover": no_rollover,
        "settled_height": z_drift <= 0.02,
        "settled_xy": xy_drift <= 0.08,
        "teardown_ok": teardown_ok,
    }
    status, failure_reasons = pass_or_fail(checks, [teardown_error] if teardown_error else [])

    stage = stage_utils.get_current_stage()
    prim_count = sum(1 for _ in stage.Traverse())
    report = {
        "status": status,
        "failure_reasons": failure_reasons,
        "backend": "isaac5",
        "isaac_version": "5.1",
        "robot_usd": str(robot_path),
        "scene_usd": str(OUTPUT_SCENE) if OUTPUT_SCENE is not None else None,
        "viewport_screenshot": str(SCREENSHOT) if SCREENSHOT is not None else None,
        "robot_prim": ROBOT_PRIM_PATH,
        "articulation_root": f"{ROBOT_PRIM_PATH}/base_footprint",
        "prim_count": prim_count,
        "dof_count": int(robot.num_dof),
        "wheel_joint_names": wheel_joint_names,
        "wheel_joint_indices": wheel_indices,
        "arm_lock": "authored upright pose reset before and after every physics step",
        "arm_joint_names": [wheel_joint_names[index] for index in arm_indices],
        "arm_joint_targets_rad": arm_positions.tolist(),
        "arm_joint_final_positions_rad": final_arm_positions.tolist(),
        "max_arm_joint_error_rad": max_arm_error,
        "max_arm_joint_velocity_radps": max_arm_velocity,
        "initial_position": initial_position.tolist(),
        "final_position": final_position.tolist(),
        "xy_drift_m": xy_drift,
        "z_drift_after_settle_m": z_drift,
        "final_roll_deg": math.degrees(roll),
        "final_pitch_deg": math.degrees(pitch),
        "selected_link_positions": link_positions,
        "physics_steps": frame + 30,
        "validation_wall_seconds": validation_wall_seconds,
        "gui_min_wall_seconds": ARGS.min_wall_seconds if not ARGS.headless else 0.0,
        "samples": samples,
    }
    print("ROBOT_VALIDATION_RESULT=" + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if status == "PASS" else 2


try:
    print(
        "ROBOT_VALIDATION_READY="
        + json.dumps(
            {
                "backend": "isaac5",
                "isaac_version": "5.1",
                "robot_usd": str(ARGS.robot_usd.resolve()),
                "output_scene": str(OUTPUT_SCENE) if OUTPUT_SCENE is not None else None,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    exit_code = main()
except Exception as exc:  # pragma: no cover - requires Kit/runtime failure
    print(
        "ROBOT_VALIDATION_RESULT="
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
    try:
        simulation_app.close()
    except Exception as exc:  # pragma: no cover - requires Kit teardown failure
        print(
            "ROBOT_VALIDATION_TEARDOWN="
            + json.dumps({"status": "FAIL", "app_close": False, "error": repr(exc)}),
            flush=True,
        )
        if exit_code == 0:
            exit_code = 1
    else:
        print(
            "ROBOT_VALIDATION_TEARDOWN="
            + json.dumps({"status": "PASS", "app_close": True}),
            flush=True,
        )
raise SystemExit(exit_code)
