#!/usr/bin/env python3
"""Gate 6: validate one animated Arena-style pedestrian in the active lobby."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path

from contract import generated_output_path, pass_or_fail


BACKEND_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
ISAAC_ROOT = Path(os.environ.get("ISAAC_SIM_5_ROOT", "/home/user/isaacsim/5.1.0"))
ROS_ROOT = ISAAC_ROOT / "exts/isaacsim.ros2.bridge/humble"
CP311_OVERLAY = Path(
    os.environ.get(
        "ISAAC5_ROS_CP311_OVERLAY_INSTALL",
        str(PROJECT_ROOT / "sim_to_real/robot/comparison_models/ros2_ws_cp311_overlay/install"),
    )
)
SCENE_USD = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
ASSET_ROOT = BACKEND_ROOT / "assets/people"
BIPED_USD = ASSET_ROOT / "retarget/biped_demo_meters.usd"
WALK_USD = ASSET_ROOT / "animations/stand_walk_loop_in_place.skelanim.usd"
CHARACTERS_ROOT = ASSET_ROOT / "characters"
CHARACTER_USDS = (
    CHARACTERS_ROOT / "male_adult_construction_01_new/male_adult_construction_01_new.usd",
    CHARACTERS_ROOT / "female_adult_police_01_new/female_adult_police_01_new.usd",
    CHARACTERS_ROOT / "male_adult_construction_03/male_adult_construction_03.usd",
    CHARACTERS_ROOT / "female_adult_police_02/female_adult_police_02.usd",
    CHARACTERS_ROOT / "male_adult_police_04/male_adult_police_04.usd",
    CHARACTERS_ROOT / "female_adult_police_03_new/female_adult_police_03_new.usd",
    CHARACTERS_ROOT / "F_Medical_01/F_Medical_01.usd",
    CHARACTERS_ROOT / "M_Medical_01/M_Medical_01.usd",
)
CHARACTER_USD = CHARACTER_USDS[0]
PERSON_ROOT = "/World/Pedestrians/person_01"
VISUAL_ROOT = f"{PERSON_ROOT}/Visual"
RETARGET_SOURCE_ROOT = "/World/PedestrianRetargetSource"
RETARGET_SOURCE_SKELETON = f"{RETARGET_SOURCE_ROOT}/Root"
RETARGET_SOURCE_ANIMATION = f"{RETARGET_SOURCE_SKELETON}/Gate6Walk"
COLLIDER_PATH = "/World/PedestrianColliders/person_01"
ROBOT_POSITION = (2.0, 2.0, 0.01)
ROUTE_START = (3.0, 1.5, 0.0)
ROUTE_END = (3.0, 2.5, 0.0)
PEDESTRIAN_SPEED_MPS = 0.4
PHYSICS_DT = 1.0 / 60.0
SCAN_RATE_HZ = 15.0
TRACK_ID = 1
CHARACTER_FORWARD_AXIS = "MINUS_Y"
CHARACTER_FORWARD_YAW_OFFSET_RAD = math.pi / 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument(
        "--suppress-tracks",
        action="store_true",
        help="publish scans/clock/JSON but do not create the ground-truth /pedestrian_tracks publisher",
    )
    parser.add_argument(
        "--mobile-robot",
        action="store_true",
        help="enable the bounded Gate 9 Mecanum physical-proxy control adapter",
    )
    parser.add_argument(
        "--command-topic",
        default="/isaac5/gate9/cmd_vel",
        help="Twist input used only with --mobile-robot",
    )
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument(
        "--gate6-interaction-profile",
        choices=("cross_stop_leave", "narrow_block_wait"),
        default=None,
        help=(
            "opt-in deterministic simulation-time Mecanum command profile used only "
            "for the robot/scared-person Gate 6 acceptance run"
        ),
    )
    parser.add_argument("--mobile-robot-spawn-x", type=float, default=ROBOT_POSITION[0])
    parser.add_argument("--mobile-robot-spawn-y", type=float, default=ROBOT_POSITION[1])
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument(
        "--pedestrian-count", type=int, default=int(os.environ.get("ISAAC5_PEDESTRIAN_COUNT", "20")),
        help="backend-local crowd size (1--30; used by validate_crowd.py)",
    )
    parser.add_argument(
        "--arena-scenario",
        type=Path,
        default=BACKEND_ROOT / "config/arena_eng_lobby_pedestrians.json",
        help="Arena-format scenario supplying crowd starts, identities and waypoint_mode=1 routes",
    )
    parser.add_argument(
        "--pedestrian-seed", type=int, default=int(os.environ.get("ISAAC5_PEDESTRIAN_SEED", "7")),
        help="deterministic crowd route/phase seed",
    )
    parser.add_argument("--pedestrian-speed", type=float, default=None)
    parser.add_argument("--pedestrian-speed-min", type=float, default=float(os.environ.get("ISAAC5_PEDESTRIAN_SPEED_MIN", "0.65")))
    parser.add_argument("--pedestrian-speed-max", type=float, default=float(os.environ.get("ISAAC5_PEDESTRIAN_SPEED_MAX", "0.95")))
    parser.add_argument(
        "--pedestrian-update-rate",
        type=int,
        default=int(os.environ.get("ISAAC5_PEDESTRIAN_UPDATE_RATE", "60")),
        help="social-control and Character/capsule update rate; physics remains 60 Hz",
    )
    parser.add_argument(
        "--gui-render-rate",
        type=int,
        default=int(os.environ.get("ISAAC5_GUI_RENDER_RATE", "30")),
        help="GUI render request rate; physics and LiDAR rates are unchanged",
    )
    parser.add_argument(
        "--social-mode", choices=("off", "social", "hunav"),
        default=os.environ.get("ISAAC5_SOCIAL_MODE", "hunav"),
        help=(
            "hunav uses one external joint HuNav/LightSFM service call; Isaac applies "
            "only hard kinematic, timeout and static-boundary execution limits"
        ),
    )
    parser.add_argument("--hunav-command-timeout", type=float, default=0.35)
    parser.add_argument(
        "--hunav-executor",
        choices=("projected_pose", "native_controller"),
        default=os.environ.get("ISAAC5_HUNAV_EXECUTOR", "projected_pose"),
        help=(
            "projected_pose preserves the compatibility executor; native_controller "
            "uses Isaac 5 INavController as the persistent NavMesh/crowd motion owner"
        ),
    )
    parser.add_argument(
        "--native-controller-probe",
        action="store_true",
        help="inspect the loaded Isaac NavController binding, then exit without opening a stage",
    )
    parser.add_argument(
        "--native-pair-projection",
        choices=(
            "full", "emergency_reserve", "emergency_converged",
            "kinematic_converged", "emergency", "off",
        ),
        default=os.environ.get("ISAAC5_NATIVE_PAIR_PROJECTION", "full"),
        help=(
            "native-controller output pair guard: full targets 0.541 m, emergency "
            "targets 0.501 m, emergency_converged keeps 0.501 m with 24 dense-graph "
            "iterations, kinematic_converged enforces the same reserve in velocity space "
            "with stopping-distance braking, emergency_reserve targets 0.511 m, and off leaves pairs "
            "entirely to INavController (diagnostic only)"
        ),
    )
    parser.add_argument(
        "--native-post-kinematics",
        choices=("adapter_clipped", "controller_direct"),
        default=os.environ.get("ISAAC5_NATIVE_POST_KINEMATICS", "adapter_clipped"),
        help=(
            "adapter_clipped preserves the legacy per-agent speed/acceleration filters; "
            "controller_direct keeps the native joint position solution intact before "
            "the NavMesh and hard pair guards (diagnostic A/B)"
        ),
    )
    parser.add_argument(
        "--native-state-sync",
        choices=("adapter_pose", "controller_persistent"),
        default=os.environ.get("ISAAC5_NATIVE_STATE_SYNC", "adapter_pose"),
        help=(
            "adapter_pose writes the committed adapter pose back into INavController every "
            "frame; controller_persistent updates only speed/goal and preserves the native "
            "controller's internal agent state (diagnostic A/B)"
        ),
    )
    parser.add_argument(
        "--native-stall-recovery",
        choices=("component", "independent_yield"),
        default=os.environ.get("ISAAC5_NATIVE_STALL_RECOVERY", "component"),
        help=(
            "component preserves the old one-recoverer-per-connected-component policy; "
            "independent_yield pauses a non-adjacent yielder set and gives every other "
            "stalled agent bounded route recovery"
        ),
    )
    parser.add_argument(
        "--native-recovery-goal",
        choices=("forward", "lateral"),
        default=os.environ.get("ISAAC5_NATIVE_RECOVERY_GOAL", "forward"),
        help=(
            "forward keeps recovery on the current route segment; lateral evaluates "
            "same-island right/left sidestep goals and selects the lower projection-cost "
            "escape for stalled non-yielders"
        ),
    )
    parser.add_argument(
        "--native-stall-replan-after",
        type=float,
        default=float(os.environ.get("ISAAC5_NATIVE_STALL_REPLAN_AFTER", "0.0")),
        help=(
            "seconds of continuous low executed speed before replacing that agent's "
            "local corners with a validated NavMesh path to its semantic goal; 0 disables"
        ),
    )
    parser.add_argument(
        "--native-intent-mode",
        choices=("hunav_absolute", "route_biased", "route_goal"),
        default=os.environ.get("ISAAC5_NATIVE_INTENT_MODE", "hunav_absolute"),
        help=(
            "hunav_absolute converts the HuNav output directly into a moving goal; "
            "route_biased preserves continuous route progress and applies HuNav as a "
            "bounded steering residual; route_goal gives regular pedestrians a validated "
            "route corner while HuNav controls requested speed"
        ),
    )
    parser.add_argument(
        "--native-agent-padding",
        type=float,
        default=float(os.environ.get("ISAAC5_NATIVE_AGENT_PADDING", "0.50")),
        help="extra INavController dynamic-agent padding in metres",
    )
    parser.add_argument("--hold-wall-seconds", type=float, default=0.0)
    parser.add_argument("--setup-timeout", type=float, default=30.0)
    parser.add_argument("--screenshot", type=Path, default=None)
    return parser.parse_args()


ARGS = parse_args()
if ARGS.width < 320 or ARGS.height < 240:
    raise SystemExit("ERROR: --width/--height must be at least 320x240")
SCREENSHOT = generated_output_path(BACKEND_ROOT, ARGS.screenshot)


def add_ros_python_paths() -> None:
    paths = (
        ROS_ROOT,
        ROS_ROOT / "rclpy",
        CP311_OVERLAY / "semantic_nav_runtime/lib/python3.11/site-packages",
    )
    for path in paths:
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)


if not ARGS.no_ros:
    add_ros_python_paths()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "fast_shutdown": True,
        "width": ARGS.width,
        "height": ARGS.height,
    }
)

import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCapsule  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel, UsdUtils, Vt  # noqa: E402

from lidar_sensor import DualPhysxRaycastLidar  # noqa: E402


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


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.asarray([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


def set_person_pose(root: UsdGeom.Xform, position: np.ndarray, yaw: float) -> None:
    api = UsdGeom.XformCommonAPI(root)
    api.SetTranslate(Gf.Vec3d(*[float(value) for value in position]))
    # Isaac 6 textured Characters author controlRig:forwardAxis="MINUS Y".
    # Route velocity yaw is measured from world +X, so +pi/2 maps local -Y to
    # the velocity direction. Omitting this produces the obvious sideways slide.
    visual_yaw = yaw + CHARACTER_FORWARD_YAW_OFFSET_RAD
    api.SetRotate(
        Gf.Vec3f(0.0, 0.0, math.degrees(visual_yaw)),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def character_facing_error_rad(route_yaw: float) -> float:
    visual_yaw = route_yaw + CHARACTER_FORWARD_YAW_OFFSET_RAD
    # Rotating the model's local -Y forward vector by visual_yaw yields this
    # world-space direction.
    model_forward_world = np.asarray([math.sin(visual_yaw), -math.cos(visual_yaw)])
    velocity_direction = np.asarray([math.cos(route_yaw), math.sin(route_yaw)])
    cosine = float(np.clip(np.dot(model_forward_world, velocity_direction), -1.0, 1.0))
    return math.acos(cosine)


def first_skeleton(stage: Usd.Stage, root_path: str) -> UsdSkel.Skeleton:
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim:
        raise RuntimeError(f"Character root is missing: {root_path}")
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdSkel.Skeleton):
            return UsdSkel.Skeleton(prim)
    raise RuntimeError(f"No UsdSkel.Skeleton below character root: {root_path}")


def ensure_retarget_extension() -> None:
    """Enable the Isaac 5 retargeter used by the Isaac 6 BehaviorAgent path."""

    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled("omni.anim.retarget.core"):
        if not manager.set_extension_enabled_immediate("omni.anim.retarget.core", True):
            raise RuntimeError("Could not enable omni.anim.retarget.core")
        for _ in range(4):
            simulation_app.update()


def create_retarget_source(stage: Usd.Stage) -> UsdSkel.Skeleton:
    """Compose Isaac's tagged 81-joint source rig once, hidden from rendering."""

    source_root = UsdSkel.Root.Define(stage, RETARGET_SOURCE_ROOT)
    source_root.GetPrim().GetReferences().AddReference(str(BIPED_USD))
    UsdGeom.Imageable(source_root.GetPrim()).MakeInvisible()
    for _ in range(6):
        simulation_app.update()
    source_skeleton = first_skeleton(stage, RETARGET_SOURCE_ROOT)
    if str(source_skeleton.GetPrim().GetPath()) != RETARGET_SOURCE_SKELETON:
        raise RuntimeError(
            f"Unexpected retarget source skeleton: {source_skeleton.GetPrim().GetPath()}"
        )
    return source_skeleton


def copy_meter_scaled_walk(
    stage: Usd.Stage,
    skeleton_path: str = RETARGET_SOURCE_SKELETON,
    animation_path: str = RETARGET_SOURCE_ANIMATION,
    source_phase_index: int = 0,
) -> dict[str, object]:
    """Copy the matching 81-joint Isaac walk clip into the 1 m lobby session."""

    source_stage = Usd.Stage.Open(str(WALK_USD))
    if source_stage is None:
        raise RuntimeError(f"Could not open walk animation: {WALK_USD}")
    source = UsdSkel.Animation(source_stage.GetDefaultPrim())
    if not source:
        raise RuntimeError("Walk USD default prim is not a SkelAnimation")
    target = UsdSkel.Animation.Define(stage, animation_path)
    source_joints = source.GetJointsAttr().Get() or []
    target.GetJointsAttr().Set(source_joints)
    source_tps = float(source_stage.GetTimeCodesPerSecond())
    target_tps = float(stage.GetTimeCodesPerSecond())
    time_scale = target_tps / source_tps
    source_times = source.GetRotationsAttr().GetTimeSamples()
    source_scales_attr = source.GetScalesAttr()
    source_first = float(source_times[0])
    clip_span = (float(source_times[-1]) - source_first) * time_scale
    # Retarget exactly one gait cycle.  The previous implementation tiled the
    # source through the full requested run before retargeting.  A 600 second,
    # 20-person preview therefore generated hundreds of thousands of 101-joint
    # samples, taking minutes to start and several GiB of memory.  RuntimeGaitDriver
    # below reuses this single retargeted cycle and advances it from actual
    # travelled distance, just as Isaac 6 couples BehaviorAgent speed to motion.
    authored_times: set[float] = set()
    period_count = max(1, len(source_times) - 1)
    for sample_index, source_time in enumerate(source_times):
        target_time = (float(source_time) - source_first) * time_scale
        phase_index = (sample_index + int(source_phase_index)) % period_count
        phase_source_time = source_times[phase_index]
        rotations = source.GetRotationsAttr().Get(phase_source_time)
        translations = source.GetTranslationsAttr().Get(phase_source_time)
        target.GetRotationsAttr().Set(rotations, target_time)
        # This clip's layer metadata is centimetres, but its local joint
        # translations are authored in the exact numeric scale used by the
        # 1 m Biped skeleton.  Scaling these values collapses every bone.
        target.GetTranslationsAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in translations]),
            target_time,
        )
        scales = source_scales_attr.Get(phase_source_time)
        if scales:
            target.GetScalesAttr().Set(scales, target_time)
        authored_times.add(target_time)
    binding = UsdSkel.BindingAPI.Apply(stage.GetPrimAtPath(skeleton_path))
    binding.CreateAnimationSourceRel().SetTargets([Sdf.Path(animation_path)])
    return {
        "joint_count": len(source_joints),
        "source_sample_count": len(source_times),
        "authored_sample_count": len(authored_times),
        "source_time_codes_per_second": source_tps,
        "target_time_codes_per_second": target_tps,
        "first_target_time": min(authored_times),
        "last_target_time": max(authored_times),
        "loop_span_time_codes": clip_span,
        "source_phase_index": int(source_phase_index) % max(1, len(source_times) - 1),
    }


class RuntimeGaitDriver:
    """Drive one retargeted walk cycle from actual planar distance travelled.

    USD skeleton animation has no per-character playback clock.  Leaving the
    generated time samples attached makes every character follow the global
    timeline at a fixed rate and eventually hold the final sample.  This driver
    caches one retargeted cycle, removes those global-time samples, and writes a
    single default pose selected by cumulative distance.  A stopped pedestrian
    therefore stops stepping; slower motion produces a proportionally slower
    gait, and long runs have constant animation memory.
    """

    def __init__(
        self,
        animation: UsdSkel.Animation,
        *,
        preferred_speed_mps: float,
        target_time_codes_per_second: float,
    ) -> None:
        if preferred_speed_mps <= 0.0 or target_time_codes_per_second <= 0.0:
            raise ValueError("gait speed and time-code rate must be positive")
        rotations = animation.GetRotationsAttr()
        sample_times = [float(value) for value in rotations.GetTimeSamples()]
        if len(sample_times) < 2:
            raise RuntimeError("retargeted gait cycle needs at least two samples")
        # The final sample closes the cycle and duplicates the first pose.
        self._cycle_sample_count = len(sample_times) - 1
        self._animation = animation
        self._templates = tuple(
            Vt.Matrix4dArray(animation.GetTransforms(Usd.TimeCode(value)))
            for value in sample_times[:-1]
        )
        self.cycle_duration_sec = (
            sample_times[-1] - sample_times[0]
        ) / target_time_codes_per_second
        self.cycle_distance_m = preferred_speed_mps * self.cycle_duration_sec
        if self.cycle_duration_sec <= 0.0 or self.cycle_distance_m <= 0.0:
            raise RuntimeError("retargeted gait cycle has an invalid duration")
        self.template_max_matrix_delta = max(
            abs(float(first[row][column]) - float(second[row][column]))
            for first, second in zip(self._templates[0], self._templates[len(self._templates) // 2])
            for row in range(4)
            for column in range(4)
        )
        for attribute in (
            animation.GetRotationsAttr(),
            animation.GetTranslationsAttr(),
            animation.GetScalesAttr(),
        ):
            for sample_time in attribute.GetTimeSamples():
                attribute.ClearAtTime(sample_time)
        self.apply_count = 0
        self.pose_change_count = 0
        self._applied_indices: set[int] = set()
        self._last_index: int | None = None
        self.apply(0.0, force=True)

    def apply(self, cumulative_distance_m: float, *, force: bool = False) -> bool:
        phase = (float(cumulative_distance_m) / self.cycle_distance_m) % 1.0
        index = int(math.floor(phase * self._cycle_sample_count)) % self._cycle_sample_count
        if not force and index == self._last_index:
            return False
        self._animation.SetTransforms(self._templates[index])
        self.apply_count += 1
        if self._last_index is not None and index != self._last_index:
            self.pose_change_count += 1
        self._last_index = index
        self._applied_indices.add(index)
        return True

    def summary(self) -> dict[str, object]:
        timed_samples_remaining = max(
            len(self._animation.GetRotationsAttr().GetTimeSamples()),
            len(self._animation.GetTranslationsAttr().GetTimeSamples()),
            len(self._animation.GetScalesAttr().GetTimeSamples()),
        )
        return {
            "mode": "distance_coupled_runtime_default_pose",
            "cycle_sample_count": self._cycle_sample_count,
            "cycle_duration_sec": self.cycle_duration_sec,
            "cycle_distance_m": self.cycle_distance_m,
            "apply_count": self.apply_count,
            "pose_change_count": self.pose_change_count,
            "unique_pose_indices": len(self._applied_indices),
            "template_max_matrix_delta": self.template_max_matrix_delta,
            "timed_samples_remaining": timed_samples_remaining,
        }


def retarget_walk_to_character(
    stage: Usd.Stage,
    source_skeleton: UsdSkel.Skeleton,
    target_skeleton: UsdSkel.Skeleton,
    source_phase_index: int = 0,
) -> tuple[UsdSkel.Animation, dict[str, object]]:
    """Use NVIDIA's Isaac 5 retarget core with the tags authored in Isaac 6 assets."""

    ensure_retarget_extension()
    source_path = str(source_skeleton.GetPrim().GetPath())
    target_path = str(target_skeleton.GetPrim().GetPath())
    source_info = copy_meter_scaled_walk(
        stage,
        skeleton_path=source_path,
        animation_path=RETARGET_SOURCE_ANIMATION,
        source_phase_index=source_phase_index,
    )
    target_animation_path = f"{target_path}/Gate6Walk"
    if stage.GetPrimAtPath(target_animation_path):
        stage.RemovePrim(target_animation_path)
    from omni.anim.retarget.core.scripts.commands import CreateRetargetAnimationsCommand

    command = CreateRetargetAnimationsCommand(
        source_skeleton_path=source_path,
        target_skeleton_path=target_path,
        source_animation_paths=[RETARGET_SOURCE_ANIMATION],
        target_animation_parent_path=target_path,
        set_root_identity=False,
    )
    command.do()
    target_animation = UsdSkel.Animation(stage.GetPrimAtPath(target_animation_path))
    if not target_animation:
        raise RuntimeError(f"Retargeter did not create {target_animation_path}")
    target_joints = target_skeleton.GetJointsAttr().Get() or []
    animation_joints = target_animation.GetJointsAttr().Get() or []
    if list(target_joints) != list(animation_joints):
        raise RuntimeError("Retargeted animation joint order does not match target character")
    binding = UsdSkel.BindingAPI.Apply(target_skeleton.GetPrim())
    binding.CreateAnimationSourceRel().SetTargets([Sdf.Path(target_animation_path)])
    target_times = target_animation.GetRotationsAttr().GetTimeSamples()
    return target_animation, {
        **source_info,
        "retarget_backend": "omni.anim.retarget.core",
        "source_skeleton_path": source_path,
        "source_joint_count": len(source_skeleton.GetJointsAttr().Get() or []),
        "target_skeleton_path": target_path,
        "target_animation_path": target_animation_path,
        "target_joint_count": len(target_joints),
        "retargeted_sample_count": len(target_times),
    }


def skeleton_pose_evidence(
    skeleton: UsdSkel.Skeleton,
    visual_root_path: str = VISUAL_ROOT,
) -> dict[str, object]:
    """Prove the composed binding evaluates to two different joint poses."""

    binding = UsdSkel.BindingAPI(skeleton.GetPrim())
    targets = [str(path) for path in binding.GetAnimationSourceRel().GetTargets()]
    cache = UsdSkel.Cache()
    cache.Populate(
        UsdSkel.Root.Get(skeleton.GetPrim().GetStage(), visual_root_path),
        Usd.PrimDefaultPredicate,
    )
    query = cache.GetSkelQuery(skeleton)
    anim_query = query.GetAnimQuery()
    sample_a = query.ComputeJointLocalTransforms(Usd.TimeCode(0.0))
    sample_b = query.ComputeJointLocalTransforms(Usd.TimeCode(80.0))
    direct_a = anim_query.ComputeJointLocalTransforms(Usd.TimeCode(0.0)) if anim_query else []
    direct_b = anim_query.ComputeJointLocalTransforms(Usd.TimeCode(80.0)) if anim_query else []
    if not sample_a or not sample_b or len(sample_a) != len(sample_b):
        return {"binding_targets": targets, "joint_count": 0, "max_matrix_delta": 0.0}
    max_delta = max(
        abs(float(matrix_a[row][column]) - float(matrix_b[row][column]))
        for matrix_a, matrix_b in zip(sample_a, sample_b)
        for row in range(4)
        for column in range(4)
    )
    direct_max_delta = max(
        abs(float(matrix_a[row][column]) - float(matrix_b[row][column]))
        for matrix_a, matrix_b in zip(direct_a, direct_b)
        for row in range(4)
        for column in range(4)
    ) if direct_a and direct_b else 0.0
    rest_transforms = skeleton.GetRestTransformsAttr().Get() or []
    rest_translation_sum = sum(float(matrix.ExtractTranslation().GetLength()) for matrix in rest_transforms)
    pose_translation_sum = sum(float(matrix.ExtractTranslation().GetLength()) for matrix in sample_a)
    translation_scale_ratio = (
        pose_translation_sum / rest_translation_sum if rest_translation_sum > 1.0e-9 else 0.0
    )
    return {
        "binding_targets": targets,
        "animation_query_valid": bool(anim_query),
        "animation_query_prim": str(anim_query.GetPrim().GetPath()) if anim_query else None,
        "joint_count": len(sample_a),
        "sample_time_codes": [0.0, 80.0],
        "max_matrix_delta": max_delta,
        "direct_animation_matrix_delta": direct_max_delta,
        "translation_scale_ratio_to_rest": translation_scale_ratio,
    }


def route_state(
    sim_time: float,
    start_position=ROUTE_START,
    end_position=ROUTE_END,
    speed_mps: float = PEDESTRIAN_SPEED_MPS,
) -> tuple[np.ndarray, np.ndarray, float]:
    start = np.asarray(start_position, dtype=float)
    end = np.asarray(end_position, dtype=float)
    delta = end - start
    length = float(np.linalg.norm(delta[:2]))
    unit = delta / length
    cycle = (speed_mps * sim_time) % (2.0 * length)
    if cycle <= length:
        distance = cycle
        direction = 1.0
    else:
        distance = 2.0 * length - cycle
        direction = -1.0
    position = start + unit * distance
    velocity = unit * speed_mps * direction
    yaw = math.atan2(float(velocity[1]), float(velocity[0]))
    return position, velocity, yaw


def route_clearance(
    query,
    start_position=ROUTE_START,
    end_position=ROUTE_END,
) -> tuple[bool, list[dict[str, object]]]:
    rows: list[dict[str, object]] = []
    route_ok = True
    for alpha in np.linspace(0.0, 1.0, 9):
        point = (1.0 - alpha) * np.asarray(start_position) + alpha * np.asarray(end_position)
        ground = query.raycast_closest((float(point[0]), float(point[1]), 2.0), (0.0, 0.0, -1.0), 3.0)
        radial_min = math.inf
        for angle in np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False):
            direction = (math.cos(angle), math.sin(angle), 0.0)
            hit = query.raycast_closest((float(point[0]), float(point[1]), 0.9), direction, 0.35)
            if hit["hit"]:
                radial_min = min(radial_min, float(hit["distance"]))
        point_ok = bool(ground["hit"]) and radial_min >= 0.35
        route_ok = route_ok and point_ok
        rows.append(
            {
                "position": point.tolist(),
                "ground_hit": bool(ground["hit"]),
                "ground_path": str(ground.get("collision", "")),
                "radial_clearance_m": radial_min,
                "clear": point_ok,
            }
        )
    return route_ok, rows


def capture_viewport(world: World, output_path: Path) -> None:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    output_path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport is available")
    capture = capture_viewport_to_file(viewport, file_path=str(output_path))
    task = asyncio.ensure_future(capture.wait_for_result(completion_frames=8))
    for _ in range(300):
        world.step(render=True)
        if task.done():
            break
    if not task.done() or not task.result():
        raise RuntimeError(f"Viewport capture failed: {output_path}")
    import omni.kit.renderer_capture

    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"Viewport capture was not written: {output_path}")


class RosPublisher:
    def __init__(
        self,
        publish_tracks: bool = True,
        mobile_robot: bool = False,
        command_topic: str = "/isaac5/gate9/cmd_vel",
        command_timeout: float = 0.5,
    ):
        import rclpy
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from semantic_nav_runtime.msg import TrackedPedestrianArray
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from tf2_ros.transform_broadcaster import TransformBroadcaster

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("isaac5_gate6_pedestrian")
        self.clock_type = Clock
        self.track_array_type = TrackedPedestrianArray
        self.scan_type = LaserScan
        self.string_type = String
        self.mobile_robot = bool(mobile_robot)
        self.command_timeout = float(command_timeout)
        self.command = (0.0, 0.0)
        self.last_command_wall = -math.inf
        self.command_receive_count = 0
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.track_pub = (
            self.node.create_publisher(TrackedPedestrianArray, "/pedestrian_tracks", 20)
            if publish_tracks
            else None
        )
        self.json_pub = self.node.create_publisher(String, "/isaac_sim/pedestrian_ground_truth_json", 10)
        self.scan_01_pub = self.node.create_publisher(LaserScan, "/scan_01", 10)
        self.scan_02_pub = self.node.create_publisher(LaserScan, "/scan_02", 10)
        self.odom_type = Odometry
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 20)
        self.dynamic_tf = TransformBroadcaster(self.node)
        self.command_subscription = (
            self.node.create_subscription(Twist, command_topic, self.on_command, 20)
            if self.mobile_robot
            else None
        )
        self.static_tf = StaticTransformBroadcaster(self.node)
        transforms = []
        transform_specs = [
            ("base_link", "base_scan_01", (0.20, 0.13, 0.208), 0.0),
            ("base_link", "base_scan_02", (-0.20, -0.13, 0.208), math.pi),
        ]
        if not self.mobile_robot:
            transform_specs.insert(0, ("odom", "base_link", ROBOT_POSITION, 0.0))
        for parent, child, xyz, yaw in transform_specs:
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = float(xyz[0])
            transform.transform.translation.y = float(xyz[1])
            transform.transform.translation.z = float(xyz[2])
            transform.transform.rotation.w = math.cos(0.5 * yaw)
            transform.transform.rotation.z = math.sin(0.5 * yaw)
            transforms.append(transform)
        self.static_tf.sendTransform(transforms)
        self.track_publish_count = 0
        self.track_message_publish_count = 0
        self.track_publish_wall_times: list[float] = []

    def on_command(self, message) -> None:
        linear = float(np.clip(float(message.linear.x), -0.30, 0.30))
        angular = float(np.clip(float(message.angular.z), -1.0, 1.0))
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear, angular = 0.0, 0.0
        self.command = (linear, angular)
        self.last_command_wall = time.monotonic()
        self.command_receive_count += 1

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def current_command(self) -> tuple[float, float]:
        if time.monotonic() - self.last_command_wall > self.command_timeout:
            return 0.0, 0.0
        return self.command

    def publish_robot_state(
        self,
        sim_time: float,
        position: np.ndarray,
        yaw: float,
        command: tuple[float, float],
    ) -> None:
        from geometry_msgs.msg import TransformStamped

        stamp = self.stamp(sim_time)
        odom = self.odom_type()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
        odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
        odom.twist.twist.linear.x = float(command[0])
        odom.twist.twist.angular.z = float(command[1])
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation = odom.pose.pose.orientation
        self.dynamic_tf.sendTransform(transform)

    @staticmethod
    def stamp(seconds: float):
        from builtin_interfaces.msg import Time

        whole = int(max(0.0, seconds))
        return Time(sec=whole, nanosec=int((max(0.0, seconds) - whole) * 1_000_000_000))

    def publish_clock(self, sim_time: float) -> None:
        message = self.clock_type()
        message.clock = self.stamp(sim_time)
        self.clock_pub.publish(message)

    def publish_track(self, sim_time: float, position: np.ndarray, velocity: np.ndarray) -> None:
        self.publish_tracks(sim_time, [(TRACK_ID, "person_01", position, velocity)])

    def publish_tracks(self, sim_time: float, states) -> None:
        from geometry_msgs.msg import Point, Vector3
        from semantic_nav_runtime.msg import TrackedPedestrian

        message = self.track_array_type()
        message.header.stamp = self.stamp(sim_time)
        message.header.frame_id = "odom"
        message.tracks = []
        json_pedestrians = []
        for track_id, stable_id, position, velocity in states:
            track = TrackedPedestrian()
            track.track_id = int(track_id)
            track.position = Point(x=float(position[0]), y=float(position[1]), z=0.0)
            track.velocity = Vector3(x=float(velocity[0]), y=float(velocity[1]), z=0.0)
            track.confidence = 1.0
            track.age = self.track_publish_count + 1
            track.hits = self.track_publish_count + 1
            track.misses = 0
            track.state = "CONFIRMED"
            track.time_since_update = 0.0
            message.tracks.append(track)
            json_pedestrians.append(
                {
                    "id": str(stable_id),
                    "track_id": int(track_id),
                    "position": position.tolist(),
                    "velocity": velocity.tolist(),
                }
            )
        if self.track_pub is not None:
            self.track_pub.publish(message)
            self.track_message_publish_count += 1
        payload = self.string_type()
        payload.data = json.dumps(
            {
                "schema": "a_pipeline_pedestrian_ground_truth/v1",
                "frame_id": "odom",
                "sim_time": sim_time,
                "pedestrians": json_pedestrians,
            },
            separators=(",", ":"),
        )
        self.json_pub.publish(payload)
        self.track_publish_count += 1
        self.track_publish_wall_times.append(time.monotonic())

    def publish_scan(self, scan, publisher) -> None:
        message = self.scan_type()
        message.header.stamp = self.stamp(scan.sim_time)
        message.header.frame_id = scan.frame_id
        message.angle_min = scan.angle_min
        message.angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        message.angle_increment = scan.angle_increment
        message.time_increment = scan.scan_time / len(scan.ranges)
        message.scan_time = scan.scan_time
        message.range_min = scan.range_min
        message.range_max = scan.range_max
        message.ranges = scan.ranges
        publisher.publish(message)

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()


def main() -> int:
    result: dict[str, object] = {"status": "FAIL", "gate": 6}
    world = collider = ros = None
    teardown_errors: list[str] = []
    try:
        if ARGS.duration <= 0.0 or ARGS.hold_wall_seconds < 0.0:
            raise ValueError("durations must be positive/non-negative")
        if ARGS.headless and SCREENSHOT is not None:
            raise ValueError("--screenshot requires a GUI run")
        for path in (SCENE_USD, BIPED_USD, WALK_USD, CHARACTER_USD):
            if not path.is_file():
                raise FileNotFoundError(path)
        dependency_audit = {}
        for label, path in (
            ("scene", SCENE_USD),
            ("retarget_source_rig", BIPED_USD),
            ("walk", WALK_USD),
            ("textured_character", CHARACTER_USD),
        ):
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
            dependency_audit[label] = {
                "layers": sorted(str(layer.identifier) for layer in layers),
                "assets": sorted(str(asset) for asset in assets),
                "unresolved": sorted(str(asset) for asset in unresolved),
            }

        success, error = wait_for_task(
            asyncio.ensure_future(omni.usd.get_context().open_stage_async(str(SCENE_USD))),
            ARGS.setup_timeout,
            "opening the active lobby",
        )
        if not success:
            raise RuntimeError(f"Could not open active lobby: {error}")
        for _ in range(8):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Lobby open completed without a stage")
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetFramesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(max(360.0, ARGS.duration * 60.0 + 60.0))

        pedestrian_root = UsdGeom.Xform.Define(stage, PERSON_ROOT)
        pedestrian_root.GetPrim().CreateAttribute("arena:stableId", Sdf.ValueTypeNames.String).Set("person_01")
        pedestrian_root.GetPrim().CreateAttribute("arena:trackId", Sdf.ValueTypeNames.UInt64).Set(TRACK_ID)
        pedestrian_root.GetPrim().CreateAttribute("arena:walkSpeed", Sdf.ValueTypeNames.Float).Set(PEDESTRIAN_SPEED_MPS)
        # This is one of the exact textured Character payloads used by the
        # Isaac 6 IRA configuration.  Its authored ControlRig tags are consumed
        # by Isaac 5's NVIDIA retarget core below.
        visual_root = UsdSkel.Root.Define(stage, VISUAL_ROOT)
        visual_root.GetPrim().GetReferences().AddReference(str(CHARACTER_USD))
        for _ in range(8):
            simulation_app.update()
        skeleton = first_skeleton(stage, VISUAL_ROOT)
        source_skeleton = create_retarget_source(stage)
        animation, animation_info = retarget_walk_to_character(
            stage, source_skeleton, skeleton
        )
        animation_path = str(animation.GetPrim().GetPath())
        skeleton_joints = [str(value) for value in (skeleton.GetJointsAttr().Get() or [])]
        animation_joints = [str(value) for value in (animation.GetJointsAttr().Get() or [])]
        pose_evidence = skeleton_pose_evidence(skeleton)
        gait_driver = RuntimeGaitDriver(
            animation,
            preferred_speed_mps=PEDESTRIAN_SPEED_MPS,
            target_time_codes_per_second=animation_info["target_time_codes_per_second"],
        )
        # Retargeting has baked a self-contained 101-joint animation. Remove
        # the hidden 81-joint source payload now so its intentionally missing
        # demo animation relationship cannot warn during stage teardown.
        stage.RemovePrim(RETARGET_SOURCE_ROOT)

        world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
        # Populate the existing lobby collision scene before asking PhysX for route clearance.
        world.reset()
        query = omni.physx.get_physx_scene_query_interface()
        route_ok, route_rows = route_clearance(query)
        collider = world.scene.add(
            DynamicCapsule(
                prim_path=COLLIDER_PATH,
                name="gate6_person_01_collider",
                position=np.asarray([ROUTE_START[0], ROUTE_START[1], 0.85]),
                radius=0.25,
                height=1.2,
                color=np.asarray([0.1, 0.1, 0.1]),
                mass=70.0,
            )
        )
        collider_prim = stage.GetPrimAtPath(COLLIDER_PATH)
        UsdGeom.Imageable(collider_prim).MakeInvisible()
        world.reset()
        # DynamicCapsule.post_reset writes its initial velocities. Marking it
        # kinematic before reset makes those otherwise harmless writes produce
        # PhysX errors. Toggle kinematic mode only after object initialization.
        UsdPhysics.RigidBodyAPI(collider_prim).CreateKinematicEnabledAttr().Set(True)
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_looping(True)
        world.play()

        set_person_pose(pedestrian_root, np.asarray(ROUTE_START), math.pi / 2.0)
        collider.set_world_pose(
            np.asarray([ROUTE_START[0], ROUTE_START[1], 0.85]), yaw_quaternion(math.pi / 2.0)
        )
        if not ARGS.headless:
            set_camera_view(
                eye=np.asarray([5.2, 4.5, 3.0]),
                target=np.asarray([2.8, 2.0, 0.9]),
                camera_prim_path="/OmniverseKit_Persp",
            )
        ros = None if ARGS.no_ros else RosPublisher(publish_tracks=not ARGS.suppress_tracks)
        lidar = DualPhysxRaycastLidar(sample_count=2000, rate_hz=SCAN_RATE_HZ, range_min=0.5, range_max=50.0)

        print(
            "GATE6_PEDESTRIAN_READY="
            + json.dumps(
                {
                    "scene": str(SCENE_USD),
                    "person": PERSON_ROOT,
                    "track_id": TRACK_ID,
                    "ros": ros is not None,
                }
            ),
            flush=True,
        )
        positions: list[list[float]] = []
        collider_positions: list[list[float]] = []
        velocity_rows: list[list[float]] = []
        facing_errors: list[float] = []
        lidar_person_hits = [0, 0]
        lidar_samples = 0
        sim_start = float(world.current_time)
        wall_start = time.monotonic()
        total_steps = int(math.ceil(ARGS.duration / PHYSICS_DT))
        for step in range(total_steps):
            sim_time = float(world.current_time - sim_start)
            position, velocity, yaw = route_state(sim_time)
            gait_driver.apply(PEDESTRIAN_SPEED_MPS * sim_time)
            set_person_pose(pedestrian_root, position, yaw)
            collider.set_world_pose(
                np.asarray([position[0], position[1], 0.85]), yaw_quaternion(yaw)
            )
            world.step(render=not ARGS.headless)
            positions.append(position.tolist())
            velocity_rows.append(velocity.tolist())
            facing_errors.append(character_facing_error_rad(yaw))
            collider_position, _ = collider.get_world_pose()
            collider_positions.append(np.asarray(collider_position, dtype=float).tolist())
            current_time = float(world.current_time - sim_start)
            if ros is not None:
                ros.publish_clock(current_time)
            if step % 4 == 0:
                pair = lidar.sample(
                    current_time,
                    np.asarray(ROBOT_POSITION),
                    np.asarray([1.0, 0.0, 0.0, 0.0]),
                )
                lidar_samples += 1
                lidar_person_hits[0] += sum(path == COLLIDER_PATH for path in pair.scan_01.hit_paths)
                lidar_person_hits[1] += sum(path == COLLIDER_PATH for path in pair.scan_02.hit_paths)
                if ros is not None:
                    ros.publish_track(current_time, position, velocity)
                    ros.publish_scan(pair.scan_01, ros.scan_01_pub)
                    ros.publish_scan(pair.scan_02, ros.scan_02_pub)
            if ros is not None:
                target_wall_elapsed = (step + 1) * PHYSICS_DT
                time.sleep(max(0.0, target_wall_elapsed - (time.monotonic() - wall_start)))

        while not ARGS.headless and time.monotonic() - wall_start < ARGS.hold_wall_seconds:
            world.step(render=True)
        if SCREENSHOT is not None:
            capture_viewport(world, SCREENSHOT)

        position_array = np.asarray(positions)
        collider_array = np.asarray(collider_positions)
        visual_travel = float(np.max(np.linalg.norm(position_array[:, :2] - position_array[0, :2], axis=1)))
        collider_tracking_error = float(
            np.max(np.linalg.norm(collider_array[:, :2] - position_array[:, :2], axis=1))
        )
        runtime_gait = gait_driver.summary()
        track_wall_rate_hz = None
        if ros is not None and len(ros.track_publish_wall_times) >= 2:
            track_wall_rate_hz = (len(ros.track_publish_wall_times) - 1) / (
                ros.track_publish_wall_times[-1] - ros.track_publish_wall_times[0]
            )
        checks = {
            "dependency_closure": all(not row["unresolved"] for row in dependency_audit.values()),
            "active_lobby_z_up_meters": str(UsdGeom.GetStageUpAxis(stage)).upper() == "Z"
            and math.isclose(float(UsdGeom.GetStageMetersPerUnit(stage)), 1.0),
            "route_free_space": route_ok,
            "arena_route_contract": TRACK_ID == 1 and PEDESTRIAN_SPEED_MPS > 0.0,
            "humanoid_mesh_present": any(
                prim.IsA(UsdGeom.Mesh) for prim in Usd.PrimRange(visual_root.GetPrim())
            ),
            "skeleton_animation_joint_order_match": skeleton_joints == animation_joints
            and len(skeleton_joints) == 101,
            "distance_coupled_runtime_gait": animation_info["retargeted_sample_count"]
            >= animation_info["source_sample_count"]
            and runtime_gait["timed_samples_remaining"] == 0
            and runtime_gait["template_max_matrix_delta"] > 1.0e-4
            and runtime_gait["pose_change_count"] >= 2
            and runtime_gait["unique_pose_indices"] >= 3
            and pose_evidence["binding_targets"] == [animation_path]
            and pose_evidence["joint_count"] == 101
            and pose_evidence["max_matrix_delta"] > 1.0e-4
            and 0.8 <= pose_evidence["translation_scale_ratio_to_rest"] <= 1.2,
            "physical_collider": collider_prim.HasAPI(UsdPhysics.CollisionAPI)
            and collider_prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and bool(UsdPhysics.RigidBodyAPI(collider_prim).GetKinematicEnabledAttr().Get()),
            "stable_motion": visual_travel >= 0.75
            and collider_tracking_error <= 0.02
            and bool(np.isfinite(collider_array).all()),
            "character_faces_velocity": max(facing_errors) <= math.radians(1.0),
            "dual_lidar_contract": lidar_samples >= int(ARGS.duration * 14.0)
            and all(count > 0 for count in lidar_person_hits),
            "ros_tracks_published": ros is None
            or ARGS.suppress_tracks
            or (
                ros.track_message_publish_count == lidar_samples
                and track_wall_rate_hz is not None
                and 14.0 <= track_wall_rate_hz <= 16.0
            ),
            "screenshot_written": SCREENSHOT is None or (SCREENSHOT.is_file() and SCREENSHOT.stat().st_size > 0),
        }
        status, reasons = pass_or_fail(checks)
        result = {
            "status": status,
            "failure_reasons": reasons,
            "gate": 6,
            "backend": "isaac5_owned_textured_character_retarget_adapter",
            "isaac_version": "5.1.0",
            "scene_usd": str(SCENE_USD),
            "character_usd": str(CHARACTER_USD),
            "retarget_source_biped_usd": str(BIPED_USD),
            "walk_usd": str(WALK_USD),
            "dependency_audit": dependency_audit,
            "person_root": PERSON_ROOT,
            "physical_collider_path": COLLIDER_PATH,
            "physical_truth_scope": "kinematic capsule collision and PhysX ray intersection only; no mesh contact truth claim",
            "route": {"start": list(ROUTE_START), "end": list(ROUTE_END), "speed_mps": PEDESTRIAN_SPEED_MPS},
            "route_clearance": route_rows,
            "animation": animation_info,
            "runtime_gait": runtime_gait,
            "evaluated_skeleton_pose": pose_evidence,
            "visual_travel_m": visual_travel,
            "character_forward_axis": CHARACTER_FORWARD_AXIS,
            "maximum_facing_velocity_error_deg": math.degrees(max(facing_errors)),
            "collider_tracking_error_m": collider_tracking_error,
            "lidar_samples": lidar_samples,
            "lidar_person_hit_beams": lidar_person_hits,
            "track_id": TRACK_ID,
            "track_publish_count": 0 if ros is None else ros.track_publish_count,
            "track_message_publish_count": 0 if ros is None else ros.track_message_publish_count,
            "tracks_suppressed": bool(ARGS.suppress_tracks),
            "track_wall_rate_hz": track_wall_rate_hz,
            "viewport_screenshot": str(SCREENSHOT) if SCREENSHOT is not None else None,
            "wall_time_s": time.monotonic() - wall_start,
            "checks": checks,
            "teardown": None,
        }
        return 0 if status == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "gate": 6, "error": repr(exc), "teardown": None}
        return 1
    finally:
        if ros is not None:
            try:
                ros.close()
            except Exception as exc:
                teardown_errors.append(f"ros_close: {exc!r}")
        if world is not None:
            try:
                world.stop()
            except Exception as exc:
                teardown_errors.append(f"world_stop: {exc!r}")
        teardown = {"status": "PASS" if not teardown_errors else "FAIL", "errors": teardown_errors}
        result["teardown"] = teardown
        if teardown_errors and result.get("status") == "PASS":
            result["status"] = "FAIL"
        print("GATE6_PEDESTRIAN_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    exit_code = main()
    simulation_app.close()
    raise SystemExit(exit_code)
