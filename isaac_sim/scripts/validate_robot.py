#!/usr/bin/env python3
"""Load the project robot in Isaac Sim 5.1 and run a minimal stability check."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROBOT_USD = (
    PROJECT_ROOT.parent
    / "robot_related"
    / "robots"
    / "chassis_arm"
    / "motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
)
DEFAULT_SCENE_USD = PROJECT_ROOT / "isaac_sim" / "scenes" / "mecanum_minimal_main.usd"
ROBOT_PRIM_PATH = "/World/Robot"
EXPECTED_WHEEL_JOINTS = (
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
)
SAFE_ARM_JOINTS = {
    "joint1": 0.099483767364,
    "joint2": -0.200712863979,
    "joint3": -0.799360797413,
    "joint4": 0.0,
    "joint5": -1.850049007114,
    "joint6": 0.0,
    "gripper_joint1": 0.0,
    "gripper_joint2": 0.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without a viewport window.")
    parser.add_argument("--steps", type=int, default=360, help="Number of 60 Hz physics steps.")
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT_USD)
    parser.add_argument("--scene-usd", type=Path, default=DEFAULT_SCENE_USD)
    parser.add_argument(
        "--save-scene",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save the minimal reusable USD scene before running physics.",
    )
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }
)

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
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


def apply_safe_joint_pose(robot: Robot) -> dict[str, int]:
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

    if safe_indices:
        indices = np.asarray(safe_indices, dtype=np.int32)
        positions = np.asarray(safe_positions, dtype=np.float32)
        robot.set_joint_positions(positions=positions, joint_indices=indices)
        robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=positions, joint_indices=indices)
        )

    return {name: dof_names.index(name) for name in EXPECTED_WHEEL_JOINTS}


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


def main() -> int:
    robot_path = ARGS.robot_usd.resolve()
    if not robot_path.is_file():
        raise FileNotFoundError(f"Robot USD not found: {robot_path}")
    if ARGS.steps <= 0:
        raise ValueError("--steps must be positive")

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

    if ARGS.save_scene:
        save_stage(ARGS.scene_usd.resolve())

    world.reset()
    wheel_indices = apply_safe_joint_pose(robot)
    for _ in range(30):
        world.step(render=not ARGS.headless)

    initial_position, _ = robot.get_world_pose()
    samples = []
    for frame in range(ARGS.steps):
        world.step(render=not ARGS.headless)
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
                }
            )

    final_position, final_orientation = robot.get_world_pose()
    final_position = np.asarray(final_position, dtype=float)
    final_orientation = np.asarray(final_orientation, dtype=float)
    initial_position = np.asarray(initial_position, dtype=float)
    roll, pitch = quaternion_to_roll_pitch(final_orientation)
    xy_drift = float(np.linalg.norm(final_position[:2] - initial_position[:2]))
    z_drift = float(abs(final_position[2] - initial_position[2]))
    link_positions = get_selected_link_positions(robot)
    finite = bool(
        np.all(np.isfinite(final_position))
        and np.all(np.isfinite(final_orientation))
        and all(np.all(np.isfinite(sample["linear_velocity"])) for sample in samples)
    )
    stable = bool(
        finite
        and z_drift <= 0.02
        and xy_drift <= 0.08
        and abs(math.degrees(roll)) <= 12.0
        and abs(math.degrees(pitch)) <= 12.0
    )

    stage = stage_utils.get_current_stage()
    prim_count = sum(1 for _ in stage.Traverse())
    report = {
        "status": "PASS" if stable else "FAIL",
        "robot_usd": str(robot_path),
        "scene_usd": str(ARGS.scene_usd.resolve()) if ARGS.save_scene else None,
        "robot_prim": ROBOT_PRIM_PATH,
        "articulation_root": f"{ROBOT_PRIM_PATH}/base_footprint",
        "prim_count": prim_count,
        "dof_count": int(robot.num_dof),
        "wheel_joint_indices": wheel_indices,
        "initial_position": initial_position.tolist(),
        "final_position": final_position.tolist(),
        "xy_drift_m": xy_drift,
        "z_drift_after_settle_m": z_drift,
        "final_roll_deg": math.degrees(roll),
        "final_pitch_deg": math.degrees(pitch),
        "selected_link_positions": link_positions,
        "samples": samples,
    }
    print("ROBOT_VALIDATION_RESULT=" + json.dumps(report, ensure_ascii=False), flush=True)
    world.stop()
    return 0 if stable else 2


try:
    raise SystemExit(main())
finally:
    simulation_app.close()
