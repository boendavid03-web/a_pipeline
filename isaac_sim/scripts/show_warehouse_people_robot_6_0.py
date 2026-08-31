#!/usr/bin/env python3
"""Offline Isaac Sim 6.0.1 scene selector + optional IRA people + Mecanum ROS 2 capture.

This is intentionally independent from the 5.1 navigation runner and from
arena_ws.  It opens only local USD assets, loads the existing offline IRA
configuration, and exchanges commands/telemetry with system ROS 2 over
localhost UDP.  The external bridge provides Gazebo-compatible bag topics.
The arm is not a navigation actuator; its authored visual pose is preserved
while the chassis root owns deterministic planar navigation.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path

from pedestrian_free_space_guard import SustainedIntrusionTracker
from pedestrian_social import (
    PedestrianMotionState,
    PedestrianSocialForceController,
    RobotMotionState,
    SocialForceParameters,
    SocialQualityTracker,
    SocialYieldPlanner,
)
from pedestrian_steering import PatrolPolylineCursor, steering_target_from_velocity
from rtx_lidar_scan import project_rtx_returns
from udp_telemetry import COMPRESSED_MAGIC, TelemetryEncoder
from isaac_actuation_contract import (
    COMMAND_PROTOCOL_VERSION,
    finite_or_none,
    fixed_tick_pose_twist,
    world_to_ros_body_twist,
)


SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
LAUNCHER_SHA256 = os.environ.get("ISAAC_LAUNCHER_SHA256", "unknown")
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_CUSTOM_SCENE_USD = PROJECT_ROOT / "isaac_sim/scenes/a_pipeline_eng_lobby.usda"
CUSTOM_SCENE_USD = Path(
    os.environ.get("ISAAC_CUSTOM_SCENE_USD", str(DEFAULT_CUSTOM_SCENE_USD))
).expanduser().resolve()


def custom_spawn_coordinate(name: str, default: float) -> float:
    raw_value = os.environ.get(name, str(default)).strip()
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise SystemExit(f"ERROR: {name} must be a finite number in metres") from exc
    if not math.isfinite(value):
        raise SystemExit(f"ERROR: {name} must be a finite number in metres")
    return value


ROBOT_USD = (
    PROJECT_ROOT.parent
    / "robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
)
ROBOT_VISUAL_USD = ROBOT_USD.parent / "configuration/mecanum730_xms5_default_base.usd"
WAREHOUSE_IRA_CONFIG = SCRIPT_DIR / "ira_people_demo/ira_people_demo.yaml"
CUSTOM_IRA_CONFIG = Path(
    os.environ.get(
        "ISAAC_CUSTOM_IRA_CONFIG",
        str(SCRIPT_DIR / "ira_people_demo/custom_eng_lobby_people.yaml"),
    )
).expanduser().resolve()
SCENE_SPECS = {
    # The IRA configuration is authored specifically for this centimetre/Y-up
    # sample.  It is intentionally not injected into the other scenes.
    "warehouse": {
        "relative_path": Path("Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd"),
        "spawn": (850.0, 0.0, 600.0),
        "camera_eye": (1800.0, 1500.0, -2000.0),
        "camera_target": (500.0, 0.0, -300.0),
        "ira_people_supported": True,
    },
    "simple_room": {
        "relative_path": Path("Isaac/Environments/Simple_Room/simple_room.usd"),
        "spawn_ros": (0.0, 0.0, 0.30),
        "camera_eye": (4.0, -4.0, 3.0),
        "camera_target": (0.0, 0.0, 0.30),
        "ira_people_supported": False,
    },
    "hospital": {
        "relative_path": Path("Isaac/Environments/Hospital/hospital.usd"),
        "spawn_ros": (0.0, 0.0, 0.30),
        "camera_eye": (8.0, -8.0, 5.0),
        "camera_target": (0.0, 0.0, 0.30),
        "ira_people_supported": False,
    },
    "digital_twin_warehouse": {
        "relative_path": Path("Isaac/Environments/Digital_Twin_Warehouse/small_warehouse_digital_twin.usd"),
        "spawn_ros": (0.0, 0.0, 0.30),
        "camera_eye": (8.0, -8.0, 5.0),
        "camera_target": (0.0, 0.0, 0.30),
        "ira_people_supported": False,
    },
    # Project-owned Z-up/metre scene.  The default asset is generated from the
    # Gazebo V7 engineering-lobby geometry, while ISAAC_CUSTOM_SCENE_USD lets a
    # caller replace it with another local USD without editing this runner.
    "custom": {
        "path": CUSTOM_SCENE_USD,
        "spawn_ros": (
            custom_spawn_coordinate("ISAAC_CUSTOM_SPAWN_X_M", 2.0),
            custom_spawn_coordinate("ISAAC_CUSTOM_SPAWN_Y_M", 2.0),
            custom_spawn_coordinate("ISAAC_CUSTOM_SPAWN_Z_M", 0.01),
        ),
        "camera_eye": (38.0, -16.0, 30.0),
        "camera_target": (16.0, 12.0, 0.5),
        "auto_frame": False,
        "ira_people_supported": CUSTOM_SCENE_USD == DEFAULT_CUSTOM_SCENE_USD.resolve(),
        "ira_config": CUSTOM_IRA_CONFIG,
    },
}
MOTION_LIBRARY_RELATIVE = Path("Isaac/People/MotionLibrary/HumanMotionLibrary.usd")
WALK_RELATIVE = Path("Isaac/People/MotionLibrary/BuiltinActions/MoveWalk/WalkForward.usd")

ROBOT_PRIM = "/World/Robot"
ROBOT_COLLISION_PRIM = "/World/RobotCollisionProxy"
ROBOT_COLLISION_MATERIAL_PRIM = "/World/RobotCollisionProxyPhysicsMaterial"
COLLISION_TEST_OBSTACLE_PRIM = "/World/CollisionValidationObstacle"
RTX_SENSOR_ROOT = "/World/RtxSensors"
PHYSICS_DT = 1.0 / 60.0
TIMELINE_FPS = 30.0
TIMELINE_DURATION_SEC = 86400.0
ROOT_HEIGHT_STAGE_UNITS = 0.0
# The warehouse is centimetre-authored and Y-up.  This is an 8.5 m, 6.0 m
# X-Z apron location, separated from the three Patrol_Point X-Z routes.
# Test-only pose across the nearly north/south Patrol_Point_A/B/C route.
PEDESTRIAN_AVOIDANCE_TEST_SPAWN = (498.0, ROOT_HEIGHT_STAGE_UNITS, 300.0)
CUSTOM_PEDESTRIAN_AVOIDANCE_TEST_SPAWN = (6.0, 9.75, 0.01)
# The robot USD is Z-up.  Rotate it -90 degrees around X so its local +Z is
# the IRA warehouse's +Y, while preserving local +X as the forward direction.
SQRT_HALF = math.sqrt(0.5)
ROBOT_UPRIGHT_ORIENTATION = (SQRT_HALF, -SQRT_HALF, 0.0, 0.0)
WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
# Keep the runtime safety clamp well above normal teleoperation and policy
# speeds.  The previous 0.6 m/s ceiling silently truncated faster commands
# (for example a 0.984 m/s teleop command) even though /cmd_vel recorded the
# requested value.
MAX_LINEAR_MPS = 10.0
MAX_ANGULAR_RADPS = 1.5
COMMAND_TIMEOUT_SEC = 0.5
CMD_VEL_UDP_HOST = "127.0.0.1"
CMD_VEL_UDP_PORT = int(os.environ.get("ISAAC_CMD_VEL_UDP_PORT", "15973"))
CMD_VEL_PACKET = struct.Struct("!IQdddd")
TELEMETRY_UDP_PORT = int(os.environ.get("ISAAC_TELEMETRY_UDP_PORT", "15974"))
RESET_UDP_PORT = int(os.environ.get("ISAAC_RESET_UDP_PORT", "15975"))
TELEMETRY_SCHEMA = "isaac_6_warehouse_telemetry/v1"
TELEMETRY_PUBLISH_PERIOD_SEC = 1.0 / 30.0
PEDESTRIAN_PUBLISH_PERIOD_SEC = 1.0 / 15.0
PEDESTRIAN_MIN_VISUAL_CLEARANCE_M = 0.15
PEDESTRIAN_FREE_SPACE_GUARD_PERIOD_SEC = 1.0 / 60.0
PEDESTRIAN_FREE_SPACE_SUSTAINED_INTRUSION_SAMPLES = 3
CUSTOM_FREE_SPACE_MAP_YAML = os.environ.get(
    "ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML", ""
).strip()
# This legacy variable remains the route-generation/validation clearance.
CUSTOM_FREE_SPACE_CLEARANCE_M = float(
    os.environ.get("ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M", "0.55")
)
CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M = float(
    os.environ.get("ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M", "0.20")
)
EXPECTED_PEDESTRIAN_COUNT = int(
    os.environ.get("ISAAC_EXPECTED_PEDESTRIAN_COUNT", "-1")
)
PEDESTRIAN_SEED = int(os.environ.get("ISAAC_PEDESTRIAN_SEED", "7"))
PEDESTRIAN_BASE_SPEED_MPS = float(
    os.environ.get("ISAAC_PEDESTRIAN_SPEED", "1.0")
)
if EXPECTED_PEDESTRIAN_COUNT < -1:
    raise SystemExit("ERROR: ISAAC_EXPECTED_PEDESTRIAN_COUNT must be -1 or non-negative")
if not 0 <= PEDESTRIAN_SEED <= 4_294_967_295:
    raise SystemExit("ERROR: ISAAC_PEDESTRIAN_SEED must be between 0 and 4294967295")
if not math.isfinite(PEDESTRIAN_BASE_SPEED_MPS) or PEDESTRIAN_BASE_SPEED_MPS <= 0.0:
    raise SystemExit("ERROR: ISAAC_PEDESTRIAN_SPEED must be positive")
DEFAULT_LIDAR_RATE_HZ = 15
MIN_LIDAR_RATE_HZ = 1
MAX_LIDAR_RATE_HZ = 30
DEFAULT_LIDAR_SAMPLE_COUNT = 2000
MIN_LIDAR_SAMPLE_COUNT = 90
MAX_LIDAR_SAMPLE_COUNT = 4096
ROBOT_POSE_APPLY_PERIOD_SEC = 1.0 / 20.0
COLLISION_PLANAR_PADDING_M = 0.02
COLLISION_VERTICAL_CLEARANCE_M = 0.02
COLLISION_STOP_MARGIN_M = 0.01
ROBOT_PHYSICS_MASS_KG = 116.189
ROBOT_HEADING_HOLD_KP = 8.0
# Start outside the footprint of this tall arm-equipped robot.  PhysX scene
# queries otherwise report the robot itself instead of the warehouse/people.
LIDAR_RANGE_MIN_M = 0.5
LIDAR_RANGE_MAX_M = 50.0
RTX_SENSOR_ATTACH_SYNC_FRAMES = 5
RTX_WARMUP_PROGRESS_PERIOD_SEC = 10.0
RTX_FRAME_QUEUE_SIZE = 64
STAGE_METERS_PER_UNIT = 1.0
STAGE_UP_AXIS = "Y"


def environment_flag(name: str, default: bool) -> bool:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    normalized = raw_value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise SystemExit(
        f"ERROR: {name} must be one of 1/0, true/false, yes/no, or on/off"
    )


def environment_choice(name: str, default: str, choices: set[str]) -> str:
    value = os.environ.get(name, default).strip().lower()
    if value not in choices:
        raise SystemExit(
            f"ERROR: {name} must be one of: {', '.join(sorted(choices))}"
        )
    return value


SCENE_NAME = environment_choice("ISAAC_SCENE", "warehouse", set(SCENE_SPECS))
SCENE_SPEC = SCENE_SPECS[SCENE_NAME]
IRA_CONFIG = Path(SCENE_SPEC.get("ira_config", WAREHOUSE_IRA_CONFIG))


def environment_integer(
    name: str,
    default: int,
    minimum: int,
    maximum: int,
    *,
    unit: str = "",
) -> int:
    raw_value = os.environ.get(name, str(default)).strip()
    try:
        value = int(raw_value, 10)
    except ValueError as exc:
        raise SystemExit(f"ERROR: {name} must be an integer") from exc
    if not minimum <= value <= maximum:
        suffix = f" {unit}" if unit else ""
        raise SystemExit(
            f"ERROR: {name} must be between {minimum} and {maximum}{suffix}"
        )
    return value


def environment_float(
    name: str,
    default: float,
    minimum: float,
    maximum: float,
    *,
    unit: str = "",
) -> float:
    raw_value = os.environ.get(name, str(default)).strip()
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise SystemExit(f"ERROR: {name} must be a number") from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        suffix = f" {unit}" if unit else ""
        raise SystemExit(
            f"ERROR: {name} must be between {minimum} and {maximum}{suffix}"
        )
    return value


# The supported RTX profiles author scanRateBaseHz as a USD uint, so accept
# explicit integral rates only. Keeping this as the single source of truth prevents
# the sensor tick, telemetry period, LaserScan metadata, and bag validator
# from silently describing different frequencies.
LIDAR_RATE_HZ = environment_integer(
    "ISAAC_LIDAR_RATE_HZ",
    DEFAULT_LIDAR_RATE_HZ,
    MIN_LIDAR_RATE_HZ,
    MAX_LIDAR_RATE_HZ,
    unit="Hz",
)
LIDAR_SAMPLE_COUNT = environment_integer(
    "ISAAC_LIDAR_SAMPLE_COUNT",
    DEFAULT_LIDAR_SAMPLE_COUNT,
    MIN_LIDAR_SAMPLE_COUNT,
    MAX_LIDAR_SAMPLE_COUNT,
    unit="samples",
)
MIN_SIMULATION_FRAME_RATE_HZ = environment_integer(
    "ISAAC_MIN_SIMULATION_FRAME_RATE_HZ",
    1,
    1,
    int(round(1.0 / PHYSICS_DT)),
    unit="Hz",
)
LIDAR_PUBLISH_PERIOD_SEC = 1.0 / float(LIDAR_RATE_HZ)
PEDESTRIAN_SOCIAL_MODE = environment_choice(
    "ISAAC_PEDESTRIAN_SOCIAL_MODE",
    "legacy",
    {"gazebo_social", "legacy"},
)
PEDESTRIAN_SOCIAL_MASS_KG = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MASS_KG", 20.0, 0.1, 1000.0, unit="kg"
)
PEDESTRIAN_PERSONAL_SPACE_M = environment_float(
    "ISAAC_PEDESTRIAN_PERSONAL_SPACE_M", 1.0, 0.1, 10.0, unit="m"
)
PEDESTRIAN_VISUAL_OVERLAP_M = environment_float(
    "ISAAC_PEDESTRIAN_VISUAL_OVERLAP_M", 0.45, 0.05, 5.0, unit="m"
)
PEDESTRIAN_MAX_PERSONAL_SPACE_VIOLATION_RATIO = environment_float(
    "ISAAC_PEDESTRIAN_MAX_PERSONAL_SPACE_VIOLATION_RATIO", 0.05, 0.0, 1.0
)
PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M", 1.25, 0.45, 2.0, unit="m"
)
PEDESTRIAN_SOCIAL_YIELD_RESUME_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_YIELD_RESUME_M", 1.50, 0.46, 3.0, unit="m"
)
PEDESTRIAN_SOCIAL_NEIGHBOR_RANGE_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_NEIGHBOR_RANGE_M", 10.0, 0.1, 30.0, unit="m"
)
PEDESTRIAN_SOCIAL_FORCE_WEIGHT = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_FORCE_WEIGHT", 5.1, 0.0, 30.0
)
PEDESTRIAN_ROBOT_SOCIAL_FORCE_WEIGHT = environment_float(
    "ISAAC_PEDESTRIAN_ROBOT_SOCIAL_FORCE_WEIGHT", 5.1, 0.0, 30.0
)
PEDESTRIAN_ROBOT_PERSONAL_SPACE_FORCE_WEIGHT = environment_float(
    "ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_FORCE_WEIGHT", 6.0, 0.0, 50.0
)
PEDESTRIAN_SOCIAL_RELAXATION_TIME_SEC = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_RELAXATION_TIME_SEC", 0.5, 0.05, 5.0, unit="s"
)
PEDESTRIAN_SOCIAL_SMOOTHING_TIME_SEC = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_SMOOTHING_TIME_SEC", 0.35, 0.02, 5.0, unit="s"
)
PEDESTRIAN_SOCIAL_MAX_ACCEL_MPS2 = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MAX_ACCEL_MPS2", 4.0, 0.1, 30.0, unit="m/s^2"
)
PEDESTRIAN_SOCIAL_MAX_STEERING_CORRECTION_MPS = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_CORRECTION_MPS",
    0.65,
    0.05,
    3.0,
    unit="m/s",
)
PEDESTRIAN_SOCIAL_MAX_LATERAL_STEERING_MPS = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MAX_LATERAL_STEERING_MPS",
    0.45,
    0.01,
    2.0,
    unit="m/s",
)
PEDESTRIAN_SOCIAL_MAX_STEERING_ANGLE_DEG = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_ANGLE_DEG",
    35.0,
    1.0,
    80.0,
    unit="deg",
)
PEDESTRIAN_SOCIAL_MIN_SPEED_MPS = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_MIN_SPEED_MPS", 0.15, 0.0, 1.5, unit="m/s"
)
PEDESTRIAN_AGENT_RADIUS_M = environment_float(
    "ISAAC_PEDESTRIAN_AGENT_RADIUS_M", 0.35, 0.05, 1.0, unit="m"
)
PEDESTRIAN_ROBOT_RADIUS_M = environment_float(
    "ISAAC_PEDESTRIAN_ROBOT_RADIUS_M", 0.47, 0.05, 2.0, unit="m"
)
PEDESTRIAN_ROBOT_CLEARANCE_M = environment_float(
    "ISAAC_PEDESTRIAN_ROBOT_CLEARANCE_M", 1.0, 0.1, 5.0, unit="m"
)
PEDESTRIAN_ROBOT_PERSONAL_SPACE_SIGMA_M = environment_float(
    "ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_SIGMA_M",
    0.2,
    0.01,
    2.0,
    unit="m",
)
PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M",
    0.50,
    0.30,
    1.5,
    unit="m",
)
PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M",
    0.80,
    0.31,
    2.0,
    unit="m",
)
PEDESTRIAN_SOCIAL_EMERGENCY_DODGE_CLEARANCE_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_DODGE_CLEARANCE_M",
    0.20,
    0.05,
    1.0,
    unit="m",
)
PEDESTRIAN_SOCIAL_DEBUG_PERIOD_SEC = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_DEBUG_PERIOD_SEC", 5.0, 0.5, 60.0, unit="s"
)
PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M", 1.0, 0.4, 2.0, unit="m"
)
PEDESTRIAN_SOCIAL_ROUTE_LOOKAHEAD_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_ROUTE_LOOKAHEAD_M", 1.5, 0.5, 4.0, unit="m"
)
PEDESTRIAN_SOCIAL_WAYPOINT_REACH_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_WAYPOINT_REACH_M", 0.35, 0.1, 1.0, unit="m"
)
PEDESTRIAN_SOCIAL_TARGET_MIN_SHIFT_M = environment_float(
    "ISAAC_PEDESTRIAN_SOCIAL_TARGET_MIN_SHIFT_M", 0.02, 0.0, 0.25, unit="m"
)
PEDESTRIAN_SOCIAL_TRACE_PATH = os.environ.get(
    "ISAAC_PEDESTRIAN_SOCIAL_TRACE_PATH", ""
).strip()
if PEDESTRIAN_VISUAL_OVERLAP_M >= PEDESTRIAN_PERSONAL_SPACE_M:
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_VISUAL_OVERLAP_M must be smaller than "
        "ISAAC_PEDESTRIAN_PERSONAL_SPACE_M"
    )
if PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M <= PEDESTRIAN_VISUAL_OVERLAP_M:
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M must be greater than "
        "ISAAC_PEDESTRIAN_VISUAL_OVERLAP_M"
    )
if PEDESTRIAN_SOCIAL_YIELD_RESUME_M <= PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M:
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_SOCIAL_YIELD_RESUME_M must be greater than "
        "ISAAC_PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M"
    )
if (
    PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M
    <= PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M
):
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M must be "
        "greater than ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M"
    )
if (
    PEDESTRIAN_ROBOT_CLEARANCE_M
    < PEDESTRIAN_ROBOT_RADIUS_M + PEDESTRIAN_AGENT_RADIUS_M
):
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_ROBOT_CLEARANCE_M must be at least "
        "ISAAC_PEDESTRIAN_ROBOT_RADIUS_M + ISAAC_PEDESTRIAN_AGENT_RADIUS_M"
    )


@dataclass(frozen=True)
class PedestrianDodgeProfile:
    """Deterministic emergency-dodge parameters layered over native avoidance."""

    trigger_clearance_m: float
    motion_scale: float
    approach_hold_sec: float
    patrol_resume_delay_sec: float
    cooldown_sec: float
    rearm_clearance_m: float


PEDESTRIAN_DODGE_PROFILES = {
    # Natural mode: give BehaviorAgent's continuous object avoidance room to
    # work and reserve a slower dodge for a sustained close encounter.
    "gentle": PedestrianDodgeProfile(
        trigger_clearance_m=0.65,
        motion_scale=0.75,
        approach_hold_sec=0.25,
        patrol_resume_delay_sec=0.15,
        cooldown_sec=0.75,
        rearm_clearance_m=1.30,
    ),
    # Exact historical behavior retained for reproducible A/B comparisons.
    "legacy_dodge": PedestrianDodgeProfile(
        trigger_clearance_m=1.20,
        motion_scale=2.00,
        approach_hold_sec=0.0,
        patrol_resume_delay_sec=0.50,
        cooldown_sec=1.00,
        rearm_clearance_m=1.70,
    ),
}


def pedestrian_avoidance_mode() -> str:
    """Resolve the four-state mode while preserving the historical boolean."""
    explicit = os.environ.get("ISAAC_PEDESTRIAN_AVOIDANCE_MODE")
    choices = {"off", "native", "gentle", "legacy_dodge"}
    if explicit is not None:
        mode = explicit.strip().lower()
        if mode not in choices:
            raise SystemExit(
                "ERROR: ISAAC_PEDESTRIAN_AVOIDANCE_MODE must be one of: "
                + ", ".join(sorted(choices))
            )
        return mode
    if environment_flag("ISAAC_PEDESTRIAN_DODGE", False):
        return "legacy_dodge"
    return "gentle" if SCENE_NAME == "custom" and PEOPLE_ENABLED else (
        "native" if PEOPLE_ENABLED else "off"
    )


ROBOT_COLLISION_PROTECTION_ENABLED = environment_flag(
    "ISAAC_ROBOT_COLLISION_PROTECTION", True
)
ROBOT_PHYSICS_ENABLED = environment_flag(
    "ISAAC_ROBOT_PHYSICS", SCENE_NAME == "custom"
)
PEOPLE_ENABLED = environment_flag(
    "ISAAC_ENABLE_PEOPLE", bool(SCENE_SPEC["ira_people_supported"])
)
if PEOPLE_ENABLED and not SCENE_SPEC["ira_people_supported"]:
    raise SystemExit(
        "ERROR: ISAAC_ENABLE_PEOPLE=1 requires warehouse or the default project "
        f"engineering-lobby USD; {SCENE_NAME} has no compatible IRA patrol configuration"
    )
PEDESTRIAN_AVOIDANCE_MODE = pedestrian_avoidance_mode()
PEDESTRIAN_ROBOT_OBJECT_AVOIDANCE_ENABLED = (
    PEOPLE_ENABLED and PEDESTRIAN_AVOIDANCE_MODE != "off"
)
PEDESTRIAN_ROBOT_DODGE_ENABLED = (
    PEOPLE_ENABLED and PEDESTRIAN_AVOIDANCE_MODE in PEDESTRIAN_DODGE_PROFILES
)
PEDESTRIAN_DODGE_PROFILE = (
    PEDESTRIAN_DODGE_PROFILES.get(PEDESTRIAN_AVOIDANCE_MODE)
    if PEOPLE_ENABLED
    else None
)
if PEDESTRIAN_SOCIAL_MODE == "gazebo_social" and PEDESTRIAN_DODGE_PROFILE is not None:
    # Continuous footprint-aware avoidance owns normal encounters.  Preserve
    # the validated dodge task only for a genuinely dangerous residual state.
    PEDESTRIAN_DODGE_PROFILE = replace(
        PEDESTRIAN_DODGE_PROFILE,
        trigger_clearance_m=PEDESTRIAN_SOCIAL_EMERGENCY_DODGE_CLEARANCE_M,
    )
if not PEOPLE_ENABLED and PEDESTRIAN_AVOIDANCE_MODE != "off":
    raise SystemExit(
        "ERROR: ISAAC_PEDESTRIAN_AVOIDANCE_MODE must be off when "
        "ISAAC_ENABLE_PEOPLE=0"
    )
LIDAR_MODE = environment_choice("ISAAC_LIDAR_MODE", "rtx", {"physx", "rtx"})
LIDAR_BACKEND = (
    "physx_scene_query" if LIDAR_MODE == "physx" else "isaac_rtx_lidar"
)
PHYSX_CAPTURE_BACKEND = (
    "omni.physx_scene_query" if LIDAR_MODE == "physx" else None
)
PHYSX_ANALYTIC_LEGS_ENABLED = (
    LIDAR_MODE == "physx"
    and PEOPLE_ENABLED
    and environment_flag("ISAAC_PHYSX_ANALYTIC_LEGS", True)
)
PHYSX_ANALYTIC_LEG_RADIUS_M = environment_float(
    "ISAAC_PHYSX_ANALYTIC_LEG_RADIUS_M", 0.065, 0.03, 0.12, unit="m"
)
PHYSX_ANALYTIC_LEGS_DEBUG = environment_flag(
    "ISAAC_PHYSX_ANALYTIC_LEGS_DEBUG", False
)
RTX_LIDAR_PROFILE = environment_choice(
    "ISAAC_RTX_LIDAR_PROFILE",
    "rplidar_s2e",
    {"example_dense", "navigation_2d_32k", "rplidar_s2e"},
)
LIDAR_PAIRING_TIMESTAMP_DOMAIN = (
    "isaac_rtx_gmo_timestamp_ns"
    if LIDAR_MODE == "rtx"
    else "isaac_telemetry_sim_time"
)
# GMO timestamps are native sensor counters and are safe for exact front/rear
# pairing, but they do not necessarily advance at the same rate as the USD
# timeline when a GUI/render workload drops RTX captures. Every ROS header
# therefore uses the telemetry timeline shared by /clock, odom, and TF.
LIDAR_TIMESTAMP_DOMAIN = "isaac_telemetry_sim_time"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac GUI.")
    parser.add_argument("--duration", type=float, default=0.0, help="Simulation seconds; 0 means run until closed.")
    parser.add_argument("--fast", action="store_true", help="Do not real-time pace a headless run.")
    parser.add_argument(
        "--app-update-rate-limit-hz",
        type=float,
        default=0.0,
        help=(
            "Test-only wall-rate limit for application updates; 0 disables it. "
            "Use 13--15 Hz to exercise variable-step PhysX catch-up."
        ),
    )
    parser.add_argument(
        "--deterministic",
        action="store_true",
        help=(
            "Use Kit manual timing so sensor rates follow simulation time even "
            "when rendering is slower than wall time."
        ),
    )
    parser.add_argument("--no-ros", action="store_true", help="Do not create the ROS 2 /cmd_vel subscriber.")
    parser.add_argument(
        "--test-command", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
        help="Constant body-frame command for deterministic headless verification.",
    )
    parser.add_argument(
        "--test-collision-obstacle",
        action="store_true",
        help="Add a temporary wall in front of the spawn for collision verification.",
    )
    parser.add_argument(
        "--test-pedestrian-avoidance",
        action="store_true",
        help="Place the robot across the IRA patrol route and report pedestrian clearance.",
    )
    parser.add_argument(
        "--test-pedestrian-social",
        action="store_true",
        help=(
            "Validate person/person spacing, movement, and free-space stability "
            "for the selected scene."
        ),
    )
    parser.add_argument("--setup-timeout", type=float, default=300.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


ARGS = parse_args()
if ARGS.duration < 0.0:
    raise SystemExit("ERROR: --duration must be non-negative")
if (
    not math.isfinite(ARGS.app_update_rate_limit_hz)
    or ARGS.app_update_rate_limit_hz < 0.0
    or ARGS.app_update_rate_limit_hz > 60.0
):
    raise SystemExit(
        "ERROR: --app-update-rate-limit-hz must be 0 or a value up to 60 Hz"
    )
if ARGS.app_update_rate_limit_hz > 0.0 and (ARGS.fast or ARGS.deterministic):
    raise SystemExit(
        "ERROR: --app-update-rate-limit-hz cannot be combined with --fast or "
        "--deterministic"
    )
if ARGS.test_pedestrian_avoidance and not PEOPLE_ENABLED:
    raise SystemExit(
        "ERROR: --test-pedestrian-avoidance requires ISAAC_ENABLE_PEOPLE=1"
    )
if ARGS.test_pedestrian_social and not PEOPLE_ENABLED:
    raise SystemExit(
        "ERROR: --test-pedestrian-social requires ISAAC_ENABLE_PEOPLE=1"
    )
if ARGS.test_pedestrian_social and ARGS.duration < 5.0:
    raise SystemExit(
        "ERROR: --test-pedestrian-social requires --duration of at least 5 seconds"
    )
sys.argv = [sys.argv[0]]  # Do not leak application flags into Kit.

if "ISAACSIM_ASSET_ROOT" not in os.environ:
    raise SystemExit("ERROR: ISAACSIM_ASSET_ROOT is not set; use the companion launcher")
ASSET_ROOT = Path(os.environ["ISAACSIM_ASSET_ROOT"]).expanduser().resolve()
SCENE_USD = Path(SCENE_SPEC["path"]) if "path" in SCENE_SPEC else (
    ASSET_ROOT / SCENE_SPEC["relative_path"]
)
MOTION_LIBRARY_USD = ASSET_ROOT / MOTION_LIBRARY_RELATIVE
WALK_USD = ASSET_ROOT / WALK_RELATIVE
RTX_LIDAR_PROFILES = {
    "example_dense": {
        "relative_path": "Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda",
        "model": "Isaac RTX Example_Rotary_2D (128-channel dense diagnostic)",
    },
    "rplidar_s2e": {
        "relative_path": (
            "Isaac/Sensors/Slamtec/RPLIDAR_S2E/RPLIDAR_S2E.usda"
        ),
        "model": "Isaac RTX Slamtec RPLIDAR S2E (single-channel 2D)",
    },
    "navigation_2d_32k": {
        "path": Path(__file__).resolve().parents[1]
        / "config"
        / "rtx_lidar"
        / "navigation_2d_32k.usda",
        "model": (
            "A-Pipeline RTX navigation 2D (single-channel, 32 kHz firing, "
            "30 Hz authored ceiling)"
        ),
    },
}
RTX_LIDAR_PROFILE_SPEC = RTX_LIDAR_PROFILES[RTX_LIDAR_PROFILE]
RTX_LIDAR_USD = (
    Path(RTX_LIDAR_PROFILE_SPEC["path"])
    if "path" in RTX_LIDAR_PROFILE_SPEC
    else ASSET_ROOT / RTX_LIDAR_PROFILE_SPEC["relative_path"]
)
required_files = [
    ROBOT_USD,
    ROBOT_VISUAL_USD,
    SCENE_USD,
    *([RTX_LIDAR_USD] if LIDAR_MODE == "rtx" else []),
]
if PEOPLE_ENABLED:
    required_files.extend([IRA_CONFIG, MOTION_LIBRARY_USD, WALK_USD])
for required in required_files:
    if not required.is_file():
        raise SystemExit(f"ERROR: required local file is missing: {required}")
RTX_LIDAR_ASSET_SHA256 = (
    hashlib.sha256(RTX_LIDAR_USD.read_bytes()).hexdigest()
    if LIDAR_MODE == "rtx"
    else None
)


from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "enable_motion_bvh": LIDAR_MODE == "rtx",
        "multi_gpu": False,
        "width": ARGS.width,
        "height": ARGS.height,
        "extra_args": [
            # Isaac Sim 6 multi-tick RTX scheduling reads each sensor's
            # omni:sensor:tickRate against /ExternalSimulationTime.  These
            # settings must be present at app startup (runtime toggles are too
            # late to rebuild the Hydra scheduling pipeline).
            "--/rtx/hydra/supportMultiTickRate=true",
            "--/rtx/rendering/perSensorTickTlas=true",
            # Normal GUI playback follows wall time and catches up physics
            # below.  Deterministic evaluation advances a fixed simulation
            # interval per explicit application update.
            "--/app/player/useFixedTimeStepping="
            + ("true" if ARGS.deterministic else "false"),
            "--/app/runLoops/main/manualModeEnabled="
            + ("true" if (ARGS.fast or ARGS.deterministic) else "false"),
            "--/telemetry/enableAnonymousData=false",
            "--/privacy/usage=false",
            "--/privacy/performance=false",
            "--/privacy/personalization=false",
        ],
    }
)


import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim, XFormPrim  # noqa: E402
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdSkel  # noqa: E402
from physx_lidar_people import (  # noqa: E402
    is_ignored_person_query_collider,
    nearest_ray_capsule_intersections,
    scene_query_hit_value,
)


def clamp_twist(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    planar = math.hypot(vx, vy)
    if planar > MAX_LINEAR_MPS:
        vx *= MAX_LINEAR_MPS / planar
        vy *= MAX_LINEAR_MPS / planar
    return vx, vy, max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, wz))


def advance_periodic_origin(previous: float, current: float, period: float) -> float:
    """Advance a fixed simulation-time schedule without frame-quantization drift."""
    if not math.isfinite(previous):
        return current
    elapsed = max(0.0, current - previous)
    periods = max(1, int(math.floor((elapsed + 1.0e-9) / period)))
    return previous + periods * period


def yaw_from_quaternion(quaternion: np.ndarray) -> float:
    """Return ROS yaw from the configured Y-up or Z-up stage orientation."""
    w, x, y, z = [float(item) for item in quaternion]
    if STAGE_UP_AXIS == "Z":
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_z = 2.0 * (x * z - w * y)
    return math.atan2(-forward_z, forward_x)


def robot_orientation(yaw: float) -> np.ndarray:
    """Compose ROS-positive yaw with the target stage's up-axis conversion."""
    cosine = math.cos(0.5 * yaw)
    sine = math.sin(0.5 * yaw)
    if STAGE_UP_AXIS == "Z":
        return np.asarray([cosine, 0.0, 0.0, sine])
    return np.asarray(
        [
            SQRT_HALF * cosine,
            -SQRT_HALF * cosine,
            SQRT_HALF * sine,
            SQRT_HALF * sine,
        ]
    )


def stage_from_ros_offset(forward_m: float, left_m: float, up_m: float) -> np.ndarray:
    """Convert a ROS-frame metric vector into stage units for either up axis."""
    scale = STAGE_METERS_PER_UNIT
    if STAGE_UP_AXIS == "Z":
        return np.asarray([forward_m / scale, left_m / scale, up_m / scale])
    return np.asarray([forward_m / scale, up_m / scale, -left_m / scale])


def stage_to_ros_vector(vector_stage: np.ndarray) -> np.ndarray:
    """Convert a stage vector/position to ROS x/y/z metres."""
    vector = np.asarray(vector_stage, dtype=float) * STAGE_METERS_PER_UNIT
    if STAGE_UP_AXIS == "Z":
        return np.asarray([vector[0], vector[1], vector[2]], dtype=float)
    return np.asarray([vector[0], -vector[2], vector[1]], dtype=float)


def rotate_stage_planar(vector_stage: np.ndarray, yaw: float) -> np.ndarray:
    """Rotate a stage vector by ROS-positive yaw around the stage up axis."""
    vector = np.asarray(vector_stage, dtype=float)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    if STAGE_UP_AXIS == "Z":
        return np.asarray(
            [
                cosine * vector[0] - sine * vector[1],
                sine * vector[0] + cosine * vector[1],
                vector[2],
            ],
            dtype=float,
        )
    return np.asarray(
        [
            cosine * vector[0] + sine * vector[2],
            vector[1],
            -sine * vector[0] + cosine * vector[2],
        ],
        dtype=float,
    )


def scene_default_spawn() -> np.ndarray:
    """Return the profile spawn in the loaded stage's coordinate convention."""
    stage_spawn = SCENE_SPEC.get("spawn")
    if stage_spawn is not None:
        return np.asarray(stage_spawn, dtype=float)
    forward_m, left_m, up_m = SCENE_SPEC["spawn_ros"]
    return stage_from_ros_offset(forward_m, left_m, up_m)


def pedestrian_avoidance_test_spawn() -> np.ndarray:
    """Return the scene-specific fixed pose used by avoidance validation."""
    if SCENE_NAME == "custom":
        return np.asarray(CUSTOM_PEDESTRIAN_AVOIDANCE_TEST_SPAWN, dtype=float)
    return np.asarray(PEDESTRIAN_AVOIDANCE_TEST_SPAWN, dtype=float)


def character_roots(stage: Usd.Stage) -> list[Usd.Prim]:
    root = stage.GetPrimAtPath("/World/Characters")
    if not root.IsValid():
        return []
    # Keep this identical to the known-good standalone IRA runner.  IRA moves
    # the character SkelRoot; the nested UsdSkel.Skeleton only contains joint
    # animation and is not the navigation transform.
    result = []
    for prim in Usd.PrimRange(root):
        if prim != root and prim.GetParent() == root:
            continue
        if prim.IsA(UsdSkel.Root):
            result.append(prim)
    return result


def character_positions(
    stage: Usd.Stage, *, require_runtime: bool = False
) -> dict[str, np.ndarray]:
    """Read live character positions, including Fabric-only agent motion.

    Isaac's BehaviorAgent advances its transform in the runtime/Fabric scene.
    Reading only the composed USD SkelRoot therefore reports the authored
    spawn pose forever even while the rendered person walks.  Prefer the
    BehaviorAgent interface and retain USD only as a setup-time fallback.
    """
    try:
        import omni.anim.behavior.core as behavior_core

        behavior_interface = behavior_core.acquire_interface()
    except Exception:
        behavior_interface = None
    positions: dict[str, np.ndarray] = {}
    missing_runtime: list[str] = []
    for prim in character_roots(stage):
        path = str(prim.GetPath())
        agent = behavior_interface.get_agent(path) if behavior_interface else None
        if agent is not None:
            point = agent.get_world_translation()
            positions[path] = np.asarray(
                [float(point.x), float(point.y), float(point.z)], dtype=float
            )
            continue
        missing_runtime.append(path)
        positions[path] = np.asarray(
            UsdGeom.Xformable(prim)
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            .ExtractTranslation(),
            dtype=float,
        )
    if require_runtime and missing_runtime:
        raise RuntimeError(
            "BehaviorAgent runtime did not initialize for: "
            + ", ".join(missing_runtime)
        )
    return positions


@dataclass(frozen=True)
class AnalyticLegSnapshot:
    """One scan-time snapshot of all animated lower-leg capsule axes."""

    sim_time: float
    segment_starts: np.ndarray
    segment_ends: np.ndarray
    radii: np.ndarray
    labels: tuple[str, ...]
    joint_debug: dict[str, dict[str, list[float]]]


@dataclass(frozen=True)
class BehaviorAgentLegBinding:
    path: str
    agent: object
    joint_indices: tuple[int, int, int, int]


class PhysxAnalyticPeopleLidar:
    """Cache BehaviorAgent joints and collect analytic-LiDAR diagnostics."""

    JOINT_TAGS = ("LeftFoot", "LeftShin", "RightFoot", "RightShin")

    def __init__(self, stage: Usd.Stage, radius_m: float, *, debug: bool = False):
        import omni.anim.behavior.core as behavior_core

        behavior_interface = behavior_core.acquire_interface()
        bindings: list[BehaviorAgentLegBinding] = []
        for prim in character_roots(stage):
            path = str(prim.GetPath())
            agent = behavior_interface.get_agent(path)
            if agent is None:
                raise RuntimeError(
                    f"BehaviorAgent runtime missing while binding analytic legs: {path}"
                )
            indices = tuple(int(agent.get_joint_index(tag)) for tag in self.JOINT_TAGS)
            missing = [tag for tag, index in zip(self.JOINT_TAGS, indices) if index < 0]
            if missing:
                raise RuntimeError(
                    f"BehaviorAgent joints missing for analytic legs: {path}: {missing}"
                )
            bindings.append(
                BehaviorAgentLegBinding(
                    path=path,
                    agent=agent,
                    joint_indices=indices,
                )
            )
        if not bindings:
            raise RuntimeError("analytic pedestrian legs enabled but no characters were found")

        self.bindings = tuple(bindings)
        self.radius_m = float(radius_m)
        self.radius_stage = self.radius_m / STAGE_METERS_PER_UNIT
        self.debug = bool(debug)
        self.scan_pairs = 0
        self.total_beams = 0
        self.fallback_beams = 0
        self.ignored_closest_hits = 0
        self.ignored_all_hits = 0
        self.analytic_accepted_beams = 0
        self.physx_accepted_beams = 0
        self.no_return_beams = 0
        self.unknown_character_hit_paths: set[str] = set()
        self.pair_compute_ms: deque[float] = deque(maxlen=4096)
        self.latest_diagnostics: dict[str, object] = {}

    @staticmethod
    def _joint_world_position(agent, index: int, path: str, tag: str) -> np.ndarray:
        translation = carb.Float3(0.0, 0.0, 0.0)
        rotation = carb.Float4(0.0, 0.0, 0.0, 1.0)
        if not agent.get_joint_world_transform(index, translation, rotation):
            raise RuntimeError(
                f"BehaviorAgent joint transform failed for analytic legs: {path}: {tag}"
            )
        point = np.asarray(
            [float(translation.x), float(translation.y), float(translation.z)],
            dtype=float,
        )
        if not np.all(np.isfinite(point)):
            raise RuntimeError(
                f"BehaviorAgent joint transform is non-finite: {path}: {tag}: {point}"
            )
        return point

    def snapshot(self, sim_time: float) -> AnalyticLegSnapshot:
        starts: list[np.ndarray] = []
        ends: list[np.ndarray] = []
        labels: list[str] = []
        joint_debug: dict[str, dict[str, list[float]]] = {}
        for binding in self.bindings:
            points = {
                tag: self._joint_world_position(
                    binding.agent,
                    index,
                    binding.path,
                    tag,
                )
                for tag, index in zip(self.JOINT_TAGS, binding.joint_indices)
            }
            for side in ("Left", "Right"):
                foot = points[f"{side}Foot"]
                shin = points[f"{side}Shin"]
                length_m = float(np.linalg.norm(shin - foot) * STAGE_METERS_PER_UNIT)
                if not 0.25 <= length_m <= 0.65:
                    raise RuntimeError(
                        "implausible animated lower-leg length for analytic LiDAR: "
                        f"{binding.path}: {side}: {length_m:.6f} m"
                    )
                starts.append(foot)
                ends.append(shin)
                labels.append(f"{binding.path}:{side.lower()}")
            if self.debug:
                joint_debug[binding.path] = {
                    tag: (point * STAGE_METERS_PER_UNIT).round(6).tolist()
                    for tag, point in points.items()
                }

        return AnalyticLegSnapshot(
            sim_time=float(sim_time),
            segment_starts=np.asarray(starts, dtype=float),
            segment_ends=np.asarray(ends, dtype=float),
            radii=np.full(len(starts), self.radius_stage, dtype=float),
            labels=tuple(labels),
            joint_debug=joint_debug,
        )

    def record_pair(
        self,
        snapshot: AnalyticLegSnapshot,
        scan_stats: dict[str, dict[str, object]],
        compute_ms: float,
    ) -> None:
        self.scan_pairs += 1
        self.pair_compute_ms.append(float(compute_ms))
        for stats in scan_stats.values():
            self.total_beams += int(stats["total_beams"])
            self.fallback_beams += int(stats["fallback_beams"])
            self.ignored_closest_hits += int(stats["ignored_closest_hits"])
            self.ignored_all_hits += int(stats["ignored_all_hits"])
            self.analytic_accepted_beams += int(stats["analytic_accepted_beams"])
            self.physx_accepted_beams += int(stats["physx_accepted_beams"])
            self.no_return_beams += int(stats["no_return_beams"])
            self.unknown_character_hit_paths.update(
                str(path) for path in stats["unknown_character_hit_paths"]
            )
        self.latest_diagnostics = {
            "schema": "physx_analytic_people_lidar/v1",
            "sim_time": snapshot.sim_time,
            "radius_m": self.radius_m,
            "people": len(self.bindings),
            "legs": len(snapshot.labels),
            "pair_compute_ms": float(compute_ms),
            "scans": scan_stats,
            "joint_world_xyz_m": snapshot.joint_debug,
        }

    def summary(self) -> dict[str, object]:
        durations = np.asarray(self.pair_compute_ms, dtype=float)
        return {
            "schema": "physx_analytic_people_lidar_summary/v1",
            "enabled": True,
            "radius_m": self.radius_m,
            "people": len(self.bindings),
            "legs": len(self.bindings) * 2,
            "scan_pairs": self.scan_pairs,
            "total_beams": self.total_beams,
            "fallback_beams": self.fallback_beams,
            "fallback_ratio": (
                self.fallback_beams / self.total_beams if self.total_beams else 0.0
            ),
            "ignored_closest_hits": self.ignored_closest_hits,
            "ignored_all_hits": self.ignored_all_hits,
            "accepted_body_collider_hits": 0,
            "analytic_accepted_beams": self.analytic_accepted_beams,
            "physx_accepted_beams": self.physx_accepted_beams,
            "no_return_beams": self.no_return_beams,
            "unknown_character_hit_paths": sorted(self.unknown_character_hit_paths),
            "pair_compute_ms_median": (
                float(np.median(durations)) if durations.size else None
            ),
            "pair_compute_ms_p95": (
                float(np.percentile(durations, 95.0)) if durations.size else None
            ),
            "pair_compute_ms_max": float(np.max(durations)) if durations.size else None,
        }


def custom_patrol_start_anchors() -> dict[str, tuple[float, float, float]]:
    """Read each generated one-person group's deterministic spawn anchor."""
    import yaml

    config = yaml.safe_load(CUSTOM_IRA_CONFIG.read_text(encoding="utf-8"))
    groups = config["isaacsim.replicator.agent"]["character"]["groups"]
    anchors: dict[str, tuple[float, float, float]] = {}
    for group_name, group in groups.items():
        if int(group.get("num", 0)) != 1:
            raise RuntimeError(
                f"Custom generated group {group_name!r} must contain exactly one person"
            )
        for routine in group.get("routines", []):
            patrol = routine.get("patrol")
            if patrol is None:
                continue
            points = patrol.get("path_points", [])
            if not points or not isinstance(points[0], list) or len(points[0]) != 3:
                raise RuntimeError(f"Invalid custom patrol start for {group_name!r}")
            anchors[group_name] = tuple(float(value) for value in points[0])
            break
    if not anchors:
        raise RuntimeError(f"No patrol anchors found in {CUSTOM_IRA_CONFIG}")
    return anchors


def custom_free_space_guard():
    """Load a non-mutating intrusion boundary inside the planned route margin."""
    if SCENE_NAME != "custom" or not CUSTOM_FREE_SPACE_MAP_YAML:
        return None
    if not math.isfinite(CUSTOM_FREE_SPACE_CLEARANCE_M) or CUSTOM_FREE_SPACE_CLEARANCE_M <= 0.0:
        raise RuntimeError("ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M must be positive")
    if (
        not math.isfinite(CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M)
        or CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M <= 0.0
        or CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M > CUSTOM_FREE_SPACE_CLEARANCE_M
    ):
        raise RuntimeError(
            "ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M must be positive "
            "and no greater than ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M"
        )
    from convert_gazebo_boxes_to_usda import load_static_boxes
    from generate_free_space_people_config import FreeSpaceMap

    static_world = (
        PROJECT_ROOT
        / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
    )
    static_boxes, _ = load_static_boxes(static_world)
    return FreeSpaceMap(
        Path(CUSTOM_FREE_SPACE_MAP_YAML),
        CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M,
        static_boxes,
    )


def reset_custom_people_to_safe_anchors(stage: Usd.Stage) -> dict[str, list[float]]:
    """Prevent IRA's random initial placement from starting outside the SLAM free area."""
    if SCENE_NAME != "custom":
        return {}
    import carb
    import omni.anim.behavior.core as behavior_core

    behavior_interface = behavior_core.acquire_interface()
    patrol_anchors = custom_patrol_start_anchors()
    anchors: dict[str, list[float]] = {}
    for prim in character_roots(stage):
        path = str(prim.GetPath())
        agent = behavior_interface.get_agent(path)
        if agent is None:
            raise RuntimeError(f"BehaviorAgent runtime missing for safe reset: {path}")
        relative = path.removeprefix("/World/Characters/")
        group_name = relative.split("/", 1)[0]
        anchor = patrol_anchors.get(group_name)
        if anchor is None:
            raise RuntimeError(
                f"No generated patrol start matches character group {group_name!r}"
            )
        if not agent.reset(target=carb.Float3(*anchor)):
            raise RuntimeError(f"BehaviorAgent refused safe reset for {path}")
        anchors[path] = list(anchor)
    return anchors


def observe_custom_people_free_space(
    stage: Usd.Stage,
    free_space,
    intrusion_tracker: SustainedIntrusionTracker | None,
):
    """Observe real live positions without mutating BehaviorAgent poses."""
    if free_space is None or intrusion_tracker is None:
        return None, {}
    current_positions = character_positions(stage, require_runtime=True)
    snapshot = intrusion_tracker.update(
        {
            path: bool(free_space.contains_world(position[0], position[1]))
            for path, position in current_positions.items()
        }
    )
    return snapshot, current_positions


def configure_pedestrian_robot_avoidance(
    stage: Usd.Stage, mode: str, social_mode: str
) -> dict[str, dict[str, object]]:
    """Enable continuous avoidance and select whether the robot participates."""
    import omni.anim.behavior.core as behavior_core

    behavior_interface = behavior_core.acquire_interface()
    configured: dict[str, dict[str, object]] = {}
    missing: list[str] = []
    for prim in character_roots(stage):
        path = str(prim.GetPath())
        agent = behavior_interface.get_agent(path)
        if agent is None:
            missing.append(path)
            continue
        # Obstacle avoidance is the continuous NavMesh/velocity layer and
        # remains enabled in both social modes.  BehaviorAgent auto avoidance
        # may launch a discrete task of its own, so gazebo_social disables it;
        # the explicit close-clearance dodge below is then the sole emergency
        # task replacement.  Legacy behavior remains selectable.
        agent.set_obstacle_avoidance_enabled(True)
        agent.set_auto_avoidance_enabled(social_mode == "legacy")
        agent.set_auto_avoidance_mass(PEDESTRIAN_SOCIAL_MASS_KG)
        object_avoidance_enabled = mode != "off"
        if object_avoidance_enabled:
            for robot_path in (ROBOT_COLLISION_PRIM, ROBOT_PRIM):
                agent.remove_obstacle_avoidance_ignored_object(robot_path)
                agent.remove_auto_avoidance_ignored_object(robot_path)
        else:
            # Preserve the IRA agents' normal person/person avoidance while
            # excluding only this robot from object avoidance.  This gives a
            # clean collision-only A/B mode without changing crowd behavior.
            for robot_path in (ROBOT_COLLISION_PRIM, ROBOT_PRIM):
                agent.add_obstacle_avoidance_ignored_object(robot_path)
                agent.add_auto_avoidance_ignored_object(robot_path)
        configured[path] = {
            "mode": mode,
            "social_mode": social_mode,
            "person_person_avoidance_enabled": True,
            "auto_avoidance_mass_kg": float(agent.get_auto_avoidance_mass()),
            "robot_object_avoidance_enabled": object_avoidance_enabled,
            "robot_dodge_enabled": mode in PEDESTRIAN_DODGE_PROFILES,
            "robot_dodge_role": (
                "emergency_fallback"
                if social_mode == "gazebo_social"
                and mode in PEDESTRIAN_DODGE_PROFILES
                else "legacy"
            ),
            "obstacle_avoidance": bool(agent.is_obstacle_avoidance_enabled()),
            "auto_avoidance": bool(agent.is_auto_avoidance_enabled()),
            "robot_obstacle_avoidance_ignored": bool(
                agent.is_obstacle_avoidance_ignored_object(ROBOT_COLLISION_PRIM)
            ),
            "robot_collision_proxy_ignored": bool(
                agent.is_auto_avoidance_ignored_object(ROBOT_COLLISION_PRIM)
            ),
        }
    if missing:
        raise RuntimeError(
            "BehaviorAgent runtime missing while configuring avoidance: "
            + ", ".join(missing)
        )
    return configured


def signed_planar_box_clearance_m(
    point_stage: np.ndarray,
    root_position_stage: np.ndarray,
    yaw: float,
    collision_dimensions_m: np.ndarray,
) -> float:
    """Signed distance from a pedestrian center to the robot's X-Z box."""
    relative_m = stage_to_ros_vector(
        np.asarray(point_stage, dtype=float)
        - np.asarray(root_position_stage, dtype=float)
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * relative_m[0] + sine * relative_m[1]
    local_y = -sine * relative_m[0] + cosine * relative_m[1]
    half_x = 0.5 * float(collision_dimensions_m[0])
    half_y = 0.5 * float(
        collision_dimensions_m[1 if STAGE_UP_AXIS == "Z" else 2]
    )
    delta_x = abs(local_x) - half_x
    delta_y = abs(local_y) - half_y
    outside = math.hypot(max(delta_x, 0.0), max(delta_y, 0.0))
    inside = min(max(delta_x, delta_y), 0.0)
    return outside + inside


class PedestrianSocialYielding:
    """Temporarily govern one agent's speed so an approaching peer can pass."""

    def __init__(
        self,
        initial_positions: dict[str, np.ndarray],
        *,
        trigger_distance_m: float,
        resume_distance_m: float,
        role: str,
    ) -> None:
        import omni.anim.behavior.core as behavior_core
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager

        behavior_interface = behavior_core.acquire_interface()
        self.agents = {
            path: behavior_interface.get_agent(path) for path in initial_positions
        }
        missing = [
            path
            for path in initial_positions
            if self.agents.get(path) is None
        ]
        if missing:
            raise RuntimeError(
                "IRA runtime missing for pedestrian social yielding: "
                + ", ".join(missing)
            )
        runtime_agents = {
            str(runtime_agent.prim.GetPath()): runtime_agent
            for runtime_agent in AgentsManager.get_instance().get_agents_by_type(
                IRA_Character
            )
        }
        self.patrol_speeds_stage_units: dict[str, float] = {}
        for path in initial_positions:
            runtime_agent = runtime_agents.get(path)
            selectors = [
                routine.walk_speed_selector
                for routine in (runtime_agent.routines if runtime_agent else [])
                if getattr(routine, "walk_speed_selector", None) is not None
            ]
            if not selectors:
                raise RuntimeError(
                    f"IRA patrol speed missing for pedestrian social yielding: {path}"
                )
            selector = selectors[0]
            speed_mps = 0.5 * (float(selector.min) + float(selector.max))
            self.patrol_speeds_stage_units[path] = (
                speed_mps / STAGE_METERS_PER_UNIT
            )
        self.planner = SocialYieldPlanner(
            trigger_distance_m=trigger_distance_m,
            resume_distance_m=resume_distance_m,
        )
        self.trigger_distance_m = float(trigger_distance_m)
        self.resume_distance_m = float(resume_distance_m)
        self.role = str(role)
        self.yielded_restore_speeds: dict[str, float] = {}
        self.yield_count = 0
        self.yield_count_by_person = {path: 0 for path in initial_positions}
        self.max_active_yielders = 0

    def update(
        self,
        positions: dict[str, np.ndarray],
        inhibited_paths=(),
    ) -> None:
        inhibited = set(inhibited_paths)
        for path in sorted(inhibited & set(self.yielded_restore_speeds)):
            agent = self.agents[path]
            restore_speed = self.yielded_restore_speeds.pop(path)
            agent.set_speed(restore_speed)
            print(
                "[WAREHOUSE-ROBOT] Pedestrian social yield preempted by "
                "robot avoidance: "
                f"person={path.rsplit('/', 1)[-1]}",
                flush=True,
            )
        decision = self.planner.update(
            {
                path: stage_to_ros_vector(position)[:2]
                for path, position in positions.items()
                if path not in inhibited
            }
        )
        for path in decision.end_yielding:
            agent = self.agents[path]
            restore_speed = self.yielded_restore_speeds.pop(path)
            agent.set_speed(restore_speed)
            print(
                "[WAREHOUSE-ROBOT] Pedestrian social yield ended: "
                f"person={path.rsplit('/', 1)[-1]} "
                f"restored_speed_stage_units={restore_speed:.3f}",
                flush=True,
            )
        for path in decision.begin_yielding:
            agent = self.agents[path]
            restore_speed = self.patrol_speeds_stage_units[path]
            self.yielded_restore_speeds[path] = restore_speed
            # Preserve the currently running moveTo task and patrol coroutine.
            # Replacing it with idle/dodge makes IRA advance to the next route
            # waypoint on resume, which can produce a wall-crossing shortcut.
            agent.set_speed(0.0)
            self.yield_count += 1
            self.yield_count_by_person[path] += 1
            print(
                "[WAREHOUSE-ROBOT] Pedestrian social yield started: "
                f"person={path.rsplit('/', 1)[-1]} "
                f"role={self.role} trigger_m={self.trigger_distance_m:.2f}",
                flush=True,
            )
        self.max_active_yielders = max(
            self.max_active_yielders, len(decision.active_yielders)
        )

    def summary(self) -> dict[str, object]:
        return {
            "role": self.role,
            "trigger_distance_m": self.trigger_distance_m,
            "resume_distance_m": self.resume_distance_m,
            "yield_count": self.yield_count,
            "yield_count_by_person": self.yield_count_by_person,
            "max_active_yielders": self.max_active_yielders,
            "patrol_speeds_stage_units": self.patrol_speeds_stage_units,
            "active_yielders": sorted(self.yielded_restore_speeds),
        }


def pedestrian_social_force_parameters() -> SocialForceParameters:
    """Build the pure controller contract from validated environment values."""

    return SocialForceParameters(
        neighbor_range_m=PEDESTRIAN_SOCIAL_NEIGHBOR_RANGE_M,
        relaxation_time_sec=PEDESTRIAN_SOCIAL_RELAXATION_TIME_SEC,
        human_social_force_weight=PEDESTRIAN_SOCIAL_FORCE_WEIGHT,
        robot_social_force_weight=PEDESTRIAN_ROBOT_SOCIAL_FORCE_WEIGHT,
        robot_personal_space_force_weight=(
            PEDESTRIAN_ROBOT_PERSONAL_SPACE_FORCE_WEIGHT
        ),
        agent_radius_m=PEDESTRIAN_AGENT_RADIUS_M,
        robot_radius_m=PEDESTRIAN_ROBOT_RADIUS_M,
        robot_clearance_m=PEDESTRIAN_ROBOT_CLEARANCE_M,
        robot_personal_space_sigma_m=(
            PEDESTRIAN_ROBOT_PERSONAL_SPACE_SIGMA_M
        ),
        smoothing_time_sec=PEDESTRIAN_SOCIAL_SMOOTHING_TIME_SEC,
        max_total_social_accel_mps2=PEDESTRIAN_SOCIAL_MAX_ACCEL_MPS2,
        max_speed_correction_mps=(
            PEDESTRIAN_SOCIAL_MAX_STEERING_CORRECTION_MPS
        ),
        max_lateral_speed_mps=PEDESTRIAN_SOCIAL_MAX_LATERAL_STEERING_MPS,
        max_steering_angle_rad=math.radians(
            PEDESTRIAN_SOCIAL_MAX_STEERING_ANGLE_DEG
        ),
        minimum_command_speed_mps=PEDESTRIAN_SOCIAL_MIN_SPEED_MPS,
    )


class BehaviorAgentSocialMotion:
    """Execute complete social velocity through one persistent follow task.

    Isaac 6.0.1 has no public desired-velocity setter on ``IBehaviorAgent``.
    Its writable locomotion contract is a scalar speed plus a navigation target.
    In the opt-in social mode this adapter suspends IRA's per-waypoint routine,
    starts one non-terminating ``follow`` task, and moves an invisible target in
    the complete social ``(vx, vy)`` direction.  BehaviorAgent still owns the
    NavMesh controller, heading, motion matching, and walking animation.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        initial_positions: dict[str, np.ndarray],
        free_space_guard=None,
    ) -> None:
        import omni.anim.behavior.core as behavior_core
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager

        self.behavior_core = behavior_core
        self.free_space_guard = free_space_guard
        behavior_interface = behavior_core.acquire_interface()
        self.agents = {
            path: behavior_interface.get_agent(path) for path in initial_positions
        }
        missing = [path for path, agent in self.agents.items() if agent is None]
        if missing:
            raise RuntimeError(
                "BehaviorAgent runtime missing for Gazebo social motion: "
                + ", ".join(missing)
            )
        self.runtime_agents = {
            str(runtime_agent.prim.GetPath()): runtime_agent
            for runtime_agent in AgentsManager.get_instance().get_agents_by_type(
                IRA_Character
            )
        }
        missing_runtime = [
            path for path in initial_positions if path not in self.runtime_agents
        ]
        if missing_runtime:
            raise RuntimeError(
                "IRA patrol runtime missing for Gazebo social steering: "
                + ", ".join(missing_runtime)
            )
        self.preferred_speeds_mps: dict[str, float] = {}
        self.patrol_cursors: dict[str, PatrolPolylineCursor] = {}
        self.follow_task_ids: dict[str, int] = {}
        self.follow_restart_count = 0
        self.target_write_count = 0
        self.last_target_positions_m: dict[str, tuple[float, float]] = {}
        self.previous_positions_m: dict[str, tuple[float, float]] = {}
        self.previous_actual_lateral_mps: dict[str, float] = {}
        self.maximum_sample_displacement_m = 0.0
        self.maximum_actual_lateral_delta_mps = 0.0
        self.maximum_commanded_lateral_mps = 0.0
        self.maximum_actual_lateral_mps = 0.0
        self.lateral_command_sample_count = 0
        self.actual_lateral_motion_sample_count = 0
        self.free_space_constrained_target_count = 0
        self.current_freeze_sec = {path: 0.0 for path in initial_positions}
        self.maximum_freeze_sec = {path: 0.0 for path in initial_positions}
        self.trace_file = None
        if PEDESTRIAN_SOCIAL_TRACE_PATH:
            trace_path = Path(PEDESTRIAN_SOCIAL_TRACE_PATH).expanduser().resolve()
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.trace_file = trace_path.open("w", encoding="utf-8", buffering=1)
            self.trace_file.write(
                json.dumps(
                    {
                        "schema": "isaac_pedestrian_social_steering/v1",
                        "type": "header",
                        "adapter": "behavior_agent_persistent_follow_target_2d",
                    },
                    sort_keys=True,
                )
                + "\n"
            )
        UsdGeom.Xform.Define(
            stage, "/World/PedestrianSocialSteeringTargets"
        )
        self.target_xforms: dict[str, UsdGeom.XformCommonAPI] = {}
        self.target_paths: dict[str, str] = {}
        for path in initial_positions:
            runtime_agent = self.runtime_agents[path]
            selectors = [
                routine.walk_speed_selector
                for routine in runtime_agent.routines
                if getattr(routine, "walk_speed_selector", None) is not None
            ]
            if not selectors:
                raise RuntimeError(
                    f"IRA patrol speed missing for Gazebo social motion: {path}"
                )
            selector = selectors[0]
            authored_midpoint_mps = 0.5 * (
                float(selector.min) + float(selector.max)
            )
            # The generated custom config authors a deterministic one-value
            # speed range.  BehaviorAgent may still report its 1.5 m/s default
            # during the setup transition, so the authored value is the only
            # stable patrol-speed contract here.
            self.preferred_speeds_mps[path] = authored_midpoint_mps
            patrol_routines = [
                routine
                for routine in runtime_agent.routines
                if getattr(routine, "path_points", None)
            ]
            if not patrol_routines:
                raise RuntimeError(
                    f"IRA patrol path missing for Gazebo social steering: {path}"
                )
            route_points_m = []
            for point in patrol_routines[0].path_points:
                point_stage = np.asarray(
                    [float(point[0]), float(point[1]), float(point[2])], dtype=float
                )
                point_m = stage_to_ros_vector(point_stage)[:2]
                route_points_m.append((float(point_m[0]), float(point_m[1])))
            initial_m_array = stage_to_ros_vector(initial_positions[path])[:2]
            initial_m = float(initial_m_array[0]), float(initial_m_array[1])
            self.patrol_cursors[path] = PatrolPolylineCursor(
                route_points_m,
                initial_m,
                waypoint_reach_m=PEDESTRIAN_SOCIAL_WAYPOINT_REACH_M,
                route_lookahead_m=PEDESTRIAN_SOCIAL_ROUTE_LOOKAHEAD_M,
            )
            target_path = (
                "/World/PedestrianSocialSteeringTargets/Target_"
                + hashlib.sha256(path.encode("utf-8")).hexdigest()[:12]
            )
            target_prim = UsdGeom.Xform.Define(stage, target_path).GetPrim()
            self.target_xforms[path] = UsdGeom.XformCommonAPI(target_prim)
            self.target_paths[path] = target_path
            base_direction = self.patrol_cursors[path].desired_direction(initial_m)
            initial_target_m = (
                initial_m[0]
                + base_direction[0] * PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M,
                initial_m[1]
                + base_direction[1] * PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M,
            )
            initial_target_m, _ = self._free_space_safe_target(
                initial_m, initial_target_m, base_direction
            )
            self._write_target(path, initial_target_m, force=True)

        # PatrolRuntimeState would otherwise replace follow() with the next
        # dense, auto-braking move_to().  These are private 6.0.1 scheduler
        # fields, deliberately isolated to this opt-in adapter.
        for path in initial_positions:
            runtime_agent = self.runtime_agents[path]
            runtime_agent._refresh_runtime_states_coro()
            runtime_agent._active_routine = None
            runtime_agent.routines.clear()
            runtime_agent._routine_p = []
            self._launch_follow(path, initial=True)
        self.controller = PedestrianSocialForceController(
            pedestrian_social_force_parameters()
        )
        self.last_sim_time: float | None = None
        self.speed_update_count = 0
        self.inhibited_update_count = 0
        self.latest_debug: dict[str, dict[str, object]] = {}
        self.next_debug_sim_time = -math.inf

    def _write_target(
        self,
        path: str,
        target_position_m: tuple[float, float],
        *,
        force: bool = False,
    ) -> bool:
        previous = self.last_target_positions_m.get(path)
        if (
            not force
            and previous is not None
            and math.dist(previous, target_position_m)
            < PEDESTRIAN_SOCIAL_TARGET_MIN_SHIFT_M
        ):
            return False
        target_stage = stage_from_ros_offset(
            target_position_m[0], target_position_m[1], 0.0
        )
        self.target_xforms[path].SetTranslate(
            Gf.Vec3d(*[float(value) for value in target_stage])
        )
        self.last_target_positions_m[path] = target_position_m
        self.target_write_count += 1
        return True

    def _launch_follow(self, path: str, *, initial: bool = False) -> None:
        task_id = self.agents[path].follow(
            target=self.target_paths[path], distance=0.0
        )
        if task_id == self.behavior_core.BEHAVIOR_TASK_ID_INVALID:
            raise RuntimeError(
                f"BehaviorAgent refused continuous steering follow task: {path}"
            )
        self.follow_task_ids[path] = task_id
        if not initial:
            self.follow_restart_count += 1

    def _free_space_safe_target(
        self,
        position_m: tuple[float, float],
        requested_target_m: tuple[float, float],
        base_direction: tuple[float, float],
    ) -> tuple[tuple[float, float], bool]:
        guard = self.free_space_guard
        # A navigation/motion-matching step can carry the root slightly beyond
        # the runtime intrusion guard.  Every segment beginning at that current
        # point is then (correctly) rejected by ``segment_world_free``.  Steer
        # back toward confirmed free cells instead of freezing in place; this
        # remains a continuous BehaviorAgent command and never teleports the
        # character.
        if guard is not None and not guard.contains_world(
            position_m[0], position_m[1]
        ):
            recovery_world = guard.world(
                guard.nearest(position_m[0], position_m[1])
            )
            recovery_position = (
                float(recovery_world[0]),
                float(recovery_world[1]),
            )
            recovery_directions = (
                base_direction,
                (-base_direction[1], base_direction[0]),
                (base_direction[1], -base_direction[0]),
                (-base_direction[0], -base_direction[1]),
            )
            # A target at only the nearest cell centre may fall inside the
            # BehaviorAgent arrival/navigation radius, leaving the animated
            # root pinned just outside the raster guard.  Extend the recovery
            # along the first continuously safe tangent so locomotion has a
            # meaningful target distance and walks back into free space.
            for direction in recovery_directions:
                for distance_m in (1.0, 0.8, 0.6, 0.4):
                    candidate = (
                        recovery_position[0] + direction[0] * distance_m,
                        recovery_position[1] + direction[1] * distance_m,
                    )
                    if guard.segment_world_free(
                        recovery_position[0],
                        recovery_position[1],
                        candidate[0],
                        candidate[1],
                    ):
                        self.free_space_constrained_target_count += 1
                        return candidate, True
            self.free_space_constrained_target_count += 1
            return recovery_position, True
        if guard is None or guard.segment_world_free(
            position_m[0],
            position_m[1],
            requested_target_m[0],
            requested_target_m[1],
        ):
            return requested_target_m, False
        offset = (
            requested_target_m[0] - position_m[0],
            requested_target_m[1] - position_m[1],
        )
        for fraction in (0.8, 0.6, 0.4, 0.2):
            candidate = (
                position_m[0] + offset[0] * fraction,
                position_m[1] + offset[1] * fraction,
            )
            if guard.segment_world_free(
                position_m[0], position_m[1], candidate[0], candidate[1]
            ):
                self.free_space_constrained_target_count += 1
                return candidate, True
        for fraction in (1.0, 0.8, 0.6, 0.4, 0.2):
            candidate = (
                position_m[0]
                + base_direction[0]
                * PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M
                * fraction,
                position_m[1]
                + base_direction[1]
                * PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M
                * fraction,
            )
            if guard.segment_world_free(
                position_m[0], position_m[1], candidate[0], candidate[1]
            ):
                self.free_space_constrained_target_count += 1
                return candidate, True
        self.free_space_constrained_target_count += 1
        return position_m, True

    def _ensure_follow(self, path: str) -> None:
        task_id = self.follow_task_ids.get(path)
        if task_id is not None and self.agents[path].is_task_running(task_id):
            return
        self._launch_follow(path)

    def update(
        self,
        positions: dict[str, np.ndarray],
        robot_collision_center_stage: np.ndarray,
        robot_yaw: float,
        robot_collision_dimensions_m: np.ndarray,
        robot_world_velocity_mps: tuple[float, float],
        sim_time: float,
        inhibited_paths=(),
    ) -> None:
        dt = (
            PEDESTRIAN_PUBLISH_PERIOD_SEC
            if self.last_sim_time is None
            else max(1.0e-6, sim_time - self.last_sim_time)
        )
        self.last_sim_time = sim_time
        states: dict[str, PedestrianMotionState] = {}
        positions_m: dict[str, tuple[float, float]] = {}
        actual_velocities_mps: dict[str, tuple[float, float]] = {}
        navigation_reported_velocities_mps: dict[str, tuple[float, float]] = {}
        base_directions: dict[str, tuple[float, float]] = {}
        for path, position_stage in positions.items():
            agent = self.agents[path]
            # The previous-frame navigation-agent velocity is the locomotion
            # state; the default current-frame value comes from motion matching
            # and can contain animation-level variation unsuitable for forces.
            velocity = agent.get_linear_velocity(True)
            velocity_stage = np.asarray(
                [float(velocity.x), float(velocity.y), float(velocity.z)],
                dtype=float,
            )
            position_array = stage_to_ros_vector(position_stage)[:2]
            position_m = float(position_array[0]), float(position_array[1])
            velocity_array = stage_to_ros_vector(velocity_stage)[:2]
            navigation_reported_velocity_mps = (
                float(velocity_array[0]),
                float(velocity_array[1]),
            )
            previous_position_m = self.previous_positions_m.get(path)
            actual_velocity_mps = (
                (
                    (position_m[0] - previous_position_m[0]) / dt,
                    (position_m[1] - previous_position_m[1]) / dt,
                )
                if previous_position_m is not None
                else navigation_reported_velocity_mps
            )
            base_direction = self.patrol_cursors[path].desired_direction(position_m)
            positions_m[path] = position_m
            actual_velocities_mps[path] = actual_velocity_mps
            navigation_reported_velocities_mps[path] = (
                navigation_reported_velocity_mps
            )
            base_directions[path] = base_direction
            states[path] = PedestrianMotionState(
                position_m=position_m,
                velocity_mps=actual_velocity_mps,
                desired_direction=base_direction,
                preferred_speed_mps=self.preferred_speeds_mps[path],
            )
        planar_dimension_index = 1 if STAGE_UP_AXIS == "Z" else 2
        robot_state = RobotMotionState(
            position_m=tuple(
                float(value)
                for value in stage_to_ros_vector(robot_collision_center_stage)[:2]
            ),
            velocity_mps=robot_world_velocity_mps,
            yaw_rad=float(robot_yaw),
            half_extents_m=(
                0.5 * float(robot_collision_dimensions_m[0]),
                0.5
                * float(robot_collision_dimensions_m[planar_dimension_index]),
            ),
        )
        outputs = self.controller.update(states, robot_state, dt)
        inhibited = set(inhibited_paths)
        debug: dict[str, dict[str, object]] = {}
        for path, output in outputs.items():
            is_inhibited = path in inhibited
            target_written = False
            if is_inhibited:
                self.inhibited_update_count += 1
            else:
                self._ensure_follow(path)
                steering_command = steering_target_from_velocity(
                    positions_m[path],
                    output.final_desired_velocity_mps,
                    PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M,
                )
                applied_target_m, free_space_constrained = (
                    self._free_space_safe_target(
                        positions_m[path],
                        steering_command.target_position_m,
                        base_directions[path],
                    )
                )
                target_written = self._write_target(
                    path, applied_target_m
                )
                self.agents[path].set_speed(
                    steering_command.speed_mps / STAGE_METERS_PER_UNIT
                )
                self.speed_update_count += 1
            if is_inhibited:
                steering_command = None
                free_space_constrained = False
            base_direction = base_directions[path]
            left_direction = -base_direction[1], base_direction[0]
            desired_forward = sum(
                output.final_desired_velocity_mps[index]
                * base_direction[index]
                for index in range(2)
            )
            desired_lateral = sum(
                output.final_desired_velocity_mps[index]
                * left_direction[index]
                for index in range(2)
            )
            actual_velocity = actual_velocities_mps[path]
            actual_forward = sum(
                actual_velocity[index] * base_direction[index]
                for index in range(2)
            )
            actual_lateral = sum(
                actual_velocity[index] * left_direction[index]
                for index in range(2)
            )
            self.maximum_commanded_lateral_mps = max(
                self.maximum_commanded_lateral_mps, abs(desired_lateral)
            )
            self.maximum_actual_lateral_mps = max(
                self.maximum_actual_lateral_mps, abs(actual_lateral)
            )
            if abs(desired_lateral) >= 0.02:
                self.lateral_command_sample_count += 1
            if abs(actual_lateral) >= 0.02:
                self.actual_lateral_motion_sample_count += 1
            previous_lateral = self.previous_actual_lateral_mps.get(path)
            if previous_lateral is not None:
                self.maximum_actual_lateral_delta_mps = max(
                    self.maximum_actual_lateral_delta_mps,
                    abs(actual_lateral - previous_lateral),
                )
            self.previous_actual_lateral_mps[path] = actual_lateral
            previous_position = self.previous_positions_m.get(path)
            if previous_position is not None:
                self.maximum_sample_displacement_m = max(
                    self.maximum_sample_displacement_m,
                    math.dist(previous_position, positions_m[path]),
                )
            self.previous_positions_m[path] = positions_m[path]
            locomotion_speed_command_mps = math.hypot(
                *output.final_desired_velocity_mps
            )
            if (
                not is_inhibited
                and locomotion_speed_command_mps >= 0.2
                and math.hypot(*actual_velocity) < 0.05
            ):
                self.current_freeze_sec[path] += dt
                self.maximum_freeze_sec[path] = max(
                    self.maximum_freeze_sec[path], self.current_freeze_sec[path]
                )
            else:
                self.current_freeze_sec[path] = 0.0
            reported_target = self.agents[path].get_target_location()
            reported_target_stage = np.asarray(
                [
                    float(reported_target.x),
                    float(reported_target.y),
                    float(reported_target.z),
                ],
                dtype=float,
            )
            facing = self.agents[path].get_facing_direction()
            facing_stage = np.asarray(
                [float(facing.x), float(facing.y), float(facing.z)], dtype=float
            )
            debug[path] = {
                "position_m": list(positions_m[path]),
                "actual_navigation_velocity_mps": list(actual_velocity),
                "actual_navigation_velocity_source": "pose_derived_position_delta",
                "behavior_agent_reported_navigation_velocity_mps": list(
                    navigation_reported_velocities_mps[path]
                ),
                "actual_navigation_forward_mps": actual_forward,
                "actual_navigation_lateral_mps": actual_lateral,
                "base_patrol_direction": list(base_direction),
                "heading_direction": list(stage_to_ros_vector(facing_stage)[:2]),
                "desired_component_mps": list(output.desired_component_mps),
                "human_social_component_mps2": list(
                    output.human_social_component_mps2
                ),
                "robot_social_component_mps2": list(
                    output.robot_social_component_mps2
                ),
                "robot_personal_space_component_mps2": list(
                    output.robot_personal_space_component_mps2
                ),
                "applied_social_accel_mps2": list(
                    output.applied_social_accel_mps2
                ),
                "final_desired_velocity_mps": list(
                    output.final_desired_velocity_mps
                ),
                "desired_forward_component_mps": desired_forward,
                "desired_lateral_component_mps": desired_lateral,
                "social_controller_speed_command_mps": output.speed_command_mps,
                "locomotion_speed_command_mps": (
                    locomotion_speed_command_mps if not is_inhibited else 0.0
                ),
                "locomotion_steering_velocity_mps": list(
                    output.final_desired_velocity_mps
                ),
                "locomotion_requested_target_position_m": (
                    list(steering_command.target_position_m)
                    if steering_command is not None
                    else None
                ),
                "locomotion_target_position_m": list(
                    self.last_target_positions_m[path]
                ),
                "locomotion_target_free_space_constrained": (
                    free_space_constrained
                ),
                "behavior_agent_reported_target_m": list(
                    stage_to_ros_vector(reported_target_stage)[:2]
                ),
                "follow_task_id": self.follow_task_ids[path],
                "follow_task_running": self.agents[path].is_task_running(
                    self.follow_task_ids[path]
                ),
                "target_written_this_update": target_written,
                "robot_footprint_clearance_m": (
                    output.robot_footprint_clearance_m
                ),
                "robot_personal_space_violation": (
                    output.robot_personal_space_violation
                ),
                "inhibited_by_emergency": is_inhibited,
            }
        self.latest_debug = debug
        if self.trace_file is not None:
            self.trace_file.write(
                json.dumps(
                    {
                        "schema": "isaac_pedestrian_social_steering/v1",
                        "type": "sample",
                        "sim_time": sim_time,
                        "people": debug,
                    },
                    sort_keys=True,
                )
                + "\n"
            )
        if sim_time >= self.next_debug_sim_time:
            print(
                "PEDESTRIAN_GAZEBO_SOCIAL_DEBUG="
                + json.dumps(
                    {
                        "sim_time": sim_time,
                        "people": debug,
                        "summary": self.controller.summary(),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            self.next_debug_sim_time = sim_time + PEDESTRIAN_SOCIAL_DEBUG_PERIOD_SEC

    def summary(self) -> dict[str, object]:
        return {
            "mode": "gazebo_social",
            "adapter": "behavior_agent_persistent_follow_target_2d",
            "lateral_vector_applied_directly": True,
            "directional_owner": "social_2d_target_plus_isaac_navmesh",
            "patrol_task_replacement": True,
            "patrol_execution": "persistent_follow_moving_target",
            "stock_per_waypoint_autobrake": False,
            "isaac_public_direct_velocity_setter_available": False,
            "isaac_public_control_entry": (
                "IBehaviorAgent.follow(target_prim, distance=0)"
            ),
            "ira_private_scheduler_suspended": True,
            "parameters": asdict(self.controller.parameters),
            "preferred_speeds_mps": self.preferred_speeds_mps,
            "speed_update_count": self.speed_update_count,
            "inhibited_update_count": self.inhibited_update_count,
            "target_write_count": self.target_write_count,
            "follow_restart_count": self.follow_restart_count,
            "follow_task_ids": self.follow_task_ids,
            "trace_path": PEDESTRIAN_SOCIAL_TRACE_PATH or None,
            "maximum_sample_displacement_m": self.maximum_sample_displacement_m,
            "maximum_actual_lateral_delta_mps": (
                self.maximum_actual_lateral_delta_mps
            ),
            "maximum_commanded_lateral_mps": self.maximum_commanded_lateral_mps,
            "maximum_actual_lateral_mps": self.maximum_actual_lateral_mps,
            "lateral_command_sample_count": self.lateral_command_sample_count,
            "actual_lateral_motion_sample_count": (
                self.actual_lateral_motion_sample_count
            ),
            "free_space_constrained_target_count": (
                self.free_space_constrained_target_count
            ),
            "maximum_freeze_sec_by_person": self.maximum_freeze_sec,
            "patrol_cursors": {
                path: cursor.summary()
                for path, cursor in self.patrol_cursors.items()
            },
            "latest_people": self.latest_debug,
            **self.controller.summary(),
        }

    def close(self) -> None:
        if self.trace_file is not None:
            self.trace_file.close()
            self.trace_file = None


class PedestrianRobotAvoidance:
    """Run an uninterrupted BehaviorAgent dodge around the exact robot body."""

    def __init__(
        self,
        initial_positions: dict[str, np.ndarray],
        mode: str,
        profile: PedestrianDodgeProfile,
    ) -> None:
        import omni.anim.behavior.core as behavior_core
        from isaacsim.replicator.agent.core.character import IRA_Character
        from omni.metropolis.pipeline.agent import AgentsManager

        self.behavior_core = behavior_core
        self.mode = mode
        self.profile = profile
        self.behavior_interface = behavior_core.acquire_interface()
        self.agents = {
            path: self.behavior_interface.get_agent(path)
            for path in initial_positions
        }
        missing = [path for path, agent in self.agents.items() if agent is None]
        if missing:
            raise RuntimeError(
                "BehaviorAgent runtime missing for robot avoidance: "
                + ", ".join(missing)
            )
        self.previous_positions = {
            path: np.asarray(position, dtype=float).copy()
            for path, position in initial_positions.items()
        }
        self.previous_clearances: dict[str, float] = {}
        self.approaching_since = {path: None for path in initial_positions}
        self.cooldown_until = {path: -math.inf for path in initial_positions}
        self.encounter_armed = {path: True for path in initial_positions}
        self.active_dodge_task_ids: dict[str, int] = {}
        self.patrol_resume_times: dict[str, float] = {}
        self.dodge_count = 0
        self.dodge_count_by_person = {path: 0 for path in initial_positions}

        manager = AgentsManager.get_instance()
        self.runtime_agents = {
            str(runtime_agent.prim.GetPath()): runtime_agent
            for runtime_agent in manager.get_agents_by_type(IRA_Character)
        }
        missing_runtime_agents = [
            path for path in initial_positions if path not in self.runtime_agents
        ]
        if missing_runtime_agents:
            raise RuntimeError(
                "IRA native patrol runtime missing for robot avoidance: "
                + ", ".join(missing_runtime_agents)
            )

    def _set_native_patrol_paused(self, path: str, paused: bool) -> None:
        # IRA's native routine owns the BehaviorAgent move task.  Suspending
        # its update loop prevents it from immediately replacing the dodge;
        # the routine generator resumes normally after that task completes.
        self.runtime_agents[path]._usd_parsing_succeeds = not paused

    @staticmethod
    def _dodge_direction(
        current: np.ndarray,
        previous: np.ndarray,
        robot_position: np.ndarray,
    ) -> np.ndarray:
        # Work in ROS x/y so the same left/right rule is valid in both the
        # warehouse's Y-up centimetre stage and the custom Z-up metre stage.
        travel = stage_to_ros_vector(
            np.asarray(current) - np.asarray(previous)
        )[:2]
        relative = stage_to_ros_vector(
            np.asarray(current) - np.asarray(robot_position)
        )[:2]
        if float(np.linalg.norm(travel)) > 1.0e-5:
            direction = np.asarray([travel[1], -travel[0]], dtype=float)
            if float(np.dot(direction, relative)) < 0.0:
                direction *= -1.0
        elif float(np.linalg.norm(relative)) > 1.0e-5:
            direction = relative.astype(float)
        else:
            direction = np.asarray([1.0, 0.0], dtype=float)
        direction /= max(1.0e-9, float(np.linalg.norm(direction)))
        direction_stage = stage_from_ros_offset(
            float(direction[0]), float(direction[1]), 0.0
        )
        return direction_stage / max(
            1.0e-9, float(np.linalg.norm(direction_stage))
        )

    def update(
        self,
        positions: dict[str, np.ndarray],
        robot_position: np.ndarray,
        robot_yaw: float,
        collision_dimensions_m: np.ndarray,
        sim_time: float,
    ) -> None:
        for path, current in positions.items():
            previous = self.previous_positions.get(path, current)
            clearance = signed_planar_box_clearance_m(
                current,
                robot_position,
                robot_yaw,
                collision_dimensions_m,
            )
            previous_clearance = self.previous_clearances.get(path, math.inf)
            approaching = clearance < previous_clearance - 0.002
            if approaching:
                if self.approaching_since.get(path) is None:
                    self.approaching_since[path] = sim_time
            else:
                self.approaching_since[path] = None
            approaching_since = self.approaching_since.get(path)
            sustained_approach = bool(
                approaching
                and approaching_since is not None
                and sim_time - approaching_since
                >= self.profile.approach_hold_sec - 1.0e-9
            )
            agent = self.agents.get(path)

            patrol_resume_time = self.patrol_resume_times.get(path)
            if patrol_resume_time is not None:
                if sim_time >= patrol_resume_time:
                    self._set_native_patrol_paused(path, False)
                    del self.patrol_resume_times[path]
                    print(
                        "[WAREHOUSE-ROBOT] Pedestrian patrol resumed: "
                        f"person={path.rsplit('/', 1)[-1]}",
                        flush=True,
                    )
                else:
                    self.previous_positions[path] = np.asarray(
                        current, dtype=float
                    ).copy()
                    self.previous_clearances[path] = clearance
                    continue

            active_task_id = self.active_dodge_task_ids.get(path)
            if agent is not None and active_task_id is not None:
                task_status = agent.get_task_status(active_task_id)
                if task_status == self.behavior_core.BehaviorTaskStatus.RUNNING:
                    self.previous_positions[path] = np.asarray(
                        current, dtype=float
                    ).copy()
                    self.previous_clearances[path] = clearance
                    continue
                del self.active_dodge_task_ids[path]
                self.patrol_resume_times[path] = (
                    sim_time + self.profile.patrol_resume_delay_sec
                )
                self.encounter_armed[path] = False
                self.cooldown_until[path] = sim_time + self.profile.cooldown_sec
                self.approaching_since[path] = None
                print(
                    "[WAREHOUSE-ROBOT] Pedestrian robot-avoidance complete: "
                    f"person={path.rsplit('/', 1)[-1]} "
                    f"status={task_status.name} clearance_m={clearance:.3f}",
                    flush=True,
                )
                self.previous_positions[path] = np.asarray(
                    current, dtype=float
                ).copy()
                self.previous_clearances[path] = clearance
                continue

            if (
                not self.encounter_armed.get(path, True)
                and clearance >= self.profile.rearm_clearance_m
            ):
                self.encounter_armed[path] = True

            if (
                agent is not None
                and self.encounter_armed.get(path, True)
                and sustained_approach
                and clearance <= self.profile.trigger_clearance_m
                and sim_time >= self.cooldown_until.get(path, -math.inf)
            ):
                direction = self._dodge_direction(current, previous, robot_position)
                self._set_native_patrol_paused(path, True)
                dodge_task_id = agent.dodge(
                    direction=carb.Float3(*[float(value) for value in direction]),
                    motion_scale=self.profile.motion_scale,
                )
                if dodge_task_id == self.behavior_core.BEHAVIOR_TASK_ID_INVALID:
                    self._set_native_patrol_paused(path, False)
                    self.cooldown_until[path] = (
                        sim_time + self.profile.cooldown_sec
                    )
                    self.approaching_since[path] = None
                else:
                    self.active_dodge_task_ids[path] = dodge_task_id
                    self.encounter_armed[path] = False
                    self.dodge_count += 1
                    self.dodge_count_by_person[path] += 1
                    self.approaching_since[path] = None
                    print(
                        "[WAREHOUSE-ROBOT] Pedestrian robot-avoidance dodge: "
                        f"person={path.rsplit('/', 1)[-1]} "
                        f"mode={self.mode} "
                        f"clearance_m={clearance:.3f} "
                        f"motion_scale={self.profile.motion_scale:.2f} "
                        f"direction={np.asarray(direction).round(3).tolist()}",
                        flush=True,
                    )
            self.previous_positions[path] = np.asarray(current, dtype=float).copy()
            self.previous_clearances[path] = clearance


def deinstance_robot_visuals(stage: Usd.Stage) -> int:
    """Disable source-authored visual instances before Hydra sees the robot.

    The 6.0.1 Fabric scene delegate loses the generated prototype paths for
    this robot's 41 instanceable visual scopes when the metre/Z-up asset is
    referenced into the centimetre/Y-up IRA stage.  Besides making meshes
    disappear, that invalid prototype state later crashes RTX Hydra in USD
    metadata lookup.  Authoring non-instanceable overrides on the composed
    prims keeps the source robot USD untouched.
    """
    root = stage.GetPrimAtPath(ROBOT_PRIM)
    if not root.IsValid():
        raise RuntimeError(f"Robot reference did not compose at {ROBOT_PRIM}")
    paths = [prim.GetPath() for prim in Usd.PrimRange(root) if prim.IsInstanceable()]
    with Sdf.ChangeBlock():
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid() and prim.IsInstanceable():
                prim.SetInstanceable(False)
    remaining = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(root)
        if prim.IsInstanceable()
    ]
    if remaining:
        raise RuntimeError(
            "Robot still contains instanceable prims after Hydra safety override: "
            f"{remaining}"
        )
    return len(paths)


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


def integrate_navigation_pose(
    command: tuple[float, float, float],
    position: np.ndarray,
    yaw: float,
    dt: float,
) -> tuple[np.ndarray, float]:
    """Apply deterministic planar navigation in a Y-up or Z-up stage."""
    vx, vy, wz = command
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    position = np.asarray(position, dtype=float).copy()
    forward = (cos_yaw * vx - sin_yaw * vy) * dt
    left = (sin_yaw * vx + cos_yaw * vy) * dt
    position += stage_from_ros_offset(forward, left, 0.0)
    yaw += wz * dt
    return position, yaw


def robot_visual_bounds(stage: Usd.Stage) -> tuple[np.ndarray, np.ndarray]:
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    aligned = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(ROBOT_PRIM)).ComputeAlignedBox()
    dimensions_m = np.asarray(aligned.GetSize(), dtype=float) * STAGE_METERS_PER_UNIT
    center_stage = 0.5 * (
        np.asarray(aligned.GetMin(), dtype=float)
        + np.asarray(aligned.GetMax(), dtype=float)
    )
    return dimensions_m, center_stage


def stage_box_orientation(yaw: float) -> tuple[float, float, float, float]:
    """Return an x/y/z/w quaternion around the loaded stage's up axis."""
    sine = math.sin(0.5 * yaw)
    cosine = math.cos(0.5 * yaw)
    return (0.0, 0.0, sine, cosine) if STAGE_UP_AXIS == "Z" else (0.0, sine, 0.0, cosine)


class RobotCollisionProxy:
    """Robot-sized PhysX body used for contact and conservative scene queries.

    In dynamic mode this body is the navigation base: gravity and PhysX
    contacts determine its vertical pose while commanded planar velocities
    drive it.  The detailed visual robot follows this body.  Kinematic mode is
    retained for the centimetre/Y-up sample warehouse where the project robot
    is only a visual compatibility layer.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        visual_dimensions_m: np.ndarray,
        visual_center_stage: np.ndarray,
        root_position_stage: np.ndarray,
        dynamic: bool = False,
    ) -> None:
        self.dynamic = dynamic
        self.body: SingleRigidPrim | None = None
        self._dynamic_command = (0.0, 0.0, 0.0)
        self._dynamic_target_yaw: float | None = None
        self._physics_callback_id: int | None = None
        self.physics_control_steps = 0
        dimensions_m = np.asarray(visual_dimensions_m, dtype=float).copy()
        planar_indices = [0, 1] if STAGE_UP_AXIS == "Z" else [0, 2]
        vertical_index = 2 if STAGE_UP_AXIS == "Z" else 1
        dimensions_m[planar_indices] += 2.0 * COLLISION_PLANAR_PADDING_M
        if not dynamic:
            dimensions_m[vertical_index] = max(
                0.1,
                dimensions_m[vertical_index] - 2.0 * COLLISION_VERTICAL_CLEARANCE_M,
            )
        self.dimensions_m = dimensions_m
        self.half_extent_stage = 0.5 * dimensions_m / STAGE_METERS_PER_UNIT
        self.center_offset_stage = (
            np.asarray(visual_center_stage, dtype=float)
            - np.asarray(root_position_stage, dtype=float)
        )
        self.query = omni.physx.get_physx_scene_query_interface()

        cube = UsdGeom.Cube.Define(stage, ROBOT_COLLISION_PRIM)
        cube.CreateSizeAttr(1.0)
        cube.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        xformable = UsdGeom.Xformable(cube)
        self.translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        scale_stage = dimensions_m / STAGE_METERS_PER_UNIT
        xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*[float(value) for value in scale_stage])
        )
        proxy_prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(proxy_prim).CreateCollisionEnabledAttr(True)
        if dynamic:
            # This proxy is already an ideal planar velocity actuator, not a
            # free-sliding block or a wheel-contact model.  Coulomb friction
            # on its full bottom face subtracts roughly mu*g*dt from every
            # commanded step and previously created a frame-rate-dependent
            # velocity bias.  Keep normal contact/gravity and obstacle
            # response, but remove that unmodelled tangential floor force.
            material = UsdShade.Material.Define(
                stage, ROBOT_COLLISION_MATERIAL_PRIM
            )
            physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            physics_material.CreateStaticFrictionAttr().Set(0.0)
            physics_material.CreateDynamicFrictionAttr().Set(0.0)
            physics_material.CreateRestitutionAttr().Set(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(
                material.GetPrim()
            ).CreateFrictionCombineModeAttr("min")
            UsdShade.MaterialBindingAPI.Apply(proxy_prim).Bind(
                material,
                materialPurpose="physics",
            )
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(proxy_prim)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(not dynamic)
        # BehaviorAgent's object-avoidance trigger filters for runtime rigid
        # volumes.  Explicit mass also keeps the robot above the agent's
        # default 20 kg auto-avoidance threshold instead of relying on PhysX's
        # inferred density for an invisible proxy.
        UsdPhysics.MassAPI.Apply(proxy_prim).CreateMassAttr(
            ROBOT_PHYSICS_MASS_KG if dynamic else 1000.0
        )
        if dynamic:
            # A planar navigation base may yaw but must not tip over when its
            # conservative full-height collision box touches a wall.  Keep Z
            # translation free so gravity and floor contact remain physical.
            locked_rotation_axes = 3 if STAGE_UP_AXIS == "Z" else 5
            physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(proxy_prim)
            physx_body.CreateLockedRotAxisAttr(locked_rotation_axes)
        proxy_prim.CreateAttribute("primvars:isVolume", Sdf.ValueTypeNames.Bool).Set(True)
        self.set_pose(root_position_stage, 0.0)

    def initialize_dynamic(self) -> None:
        """Attach the runtime tensor view after PhysX begins simulation."""
        if not self.dynamic:
            return
        self.body = SingleRigidPrim(
            prim_path=ROBOT_COLLISION_PRIM,
            name="mecanum730_xms5_physics_body",
        )
        self.body.initialize()
        _, orientation = self.body.get_world_pose()
        self._dynamic_target_yaw = yaw_from_quaternion(
            np.asarray(orientation, dtype=float)
        )
        self._physics_callback_id = SimulationManager.register_callback(
            self._on_physics_pre_step,
            event=IsaacEvents.PRE_PHYSICS_STEP,
        )

    def set_dynamic_command(self, command: tuple[float, float, float]) -> None:
        """Latch a command that the controller reapplies before every PhysX step."""
        if self.body is None or self._physics_callback_id is None:
            raise RuntimeError("dynamic robot controller has not been initialized")
        self._dynamic_command = clamp_twist(*command)

    def _on_physics_pre_step(self, step_dt: float, _context: object) -> None:
        """Apply velocity at physics rate, independent of Kit/render frame rate."""
        if (
            self.body is None
            or self._dynamic_target_yaw is None
            or not math.isfinite(step_dt)
            or step_dt <= 0.0
        ):
            return
        command = self._dynamic_command
        self.apply_dynamic_command(command, self._dynamic_target_yaw)
        self._dynamic_target_yaw = math.atan2(
            math.sin(self._dynamic_target_yaw + command[2] * step_dt),
            math.cos(self._dynamic_target_yaw + command[2] * step_dt),
        )
        self.physics_control_steps += 1

    def close_dynamic_controller(self) -> None:
        """Remove the physics callback before stopping/closing the simulation."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    def apply_dynamic_command(
        self,
        command: tuple[float, float, float],
        target_yaw: float | None = None,
    ) -> None:
        """Set planar velocity and hold the commanded heading through contacts."""
        if self.body is None:
            raise RuntimeError("dynamic robot body has not been initialized")
        center, orientation = self.body.get_world_pose()
        yaw = yaw_from_quaternion(np.asarray(orientation, dtype=float))
        vx, vy, wz = command
        if target_yaw is not None:
            yaw_error = math.atan2(
                math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)
            )
            wz = max(
                -MAX_ANGULAR_RADPS,
                min(MAX_ANGULAR_RADPS, wz + ROBOT_HEADING_HOLD_KP * yaw_error),
            )
        cosine, sine = math.cos(yaw), math.sin(yaw)
        forward = (cosine * vx - sine * vy) / STAGE_METERS_PER_UNIT
        left = (sine * vx + cosine * vy) / STAGE_METERS_PER_UNIT
        current_velocity = np.asarray(self.body.get_linear_velocity(), dtype=float)
        if STAGE_UP_AXIS == "Z":
            linear_velocity = np.asarray([forward, left, current_velocity[2]])
            angular_velocity = np.asarray([0.0, 0.0, wz])
        else:
            linear_velocity = np.asarray([forward, current_velocity[1], -left])
            angular_velocity = np.asarray([0.0, wz, 0.0])
        self.body.set_linear_velocity(linear_velocity)
        self.body.set_angular_velocity(angular_velocity)

    def dynamic_root_pose(self) -> tuple[np.ndarray, float]:
        """Return the visible robot root pose implied by the dynamic body."""
        if self.body is None:
            raise RuntimeError("dynamic robot body has not been initialized")
        center, orientation = self.body.get_world_pose()
        yaw = yaw_from_quaternion(np.asarray(orientation, dtype=float))
        root = np.asarray(center, dtype=float) - rotate_stage_planar(
            self.center_offset_stage, yaw
        )
        return root, yaw

    def reset_dynamic_pose(self, root_position_stage: np.ndarray, yaw: float) -> None:
        """Teleport the initialized PhysX proxy and clear all rigid-body motion."""
        if not self.dynamic or self.body is None:
            raise RuntimeError("dynamic robot body is unavailable for reset")
        center = self.center(root_position_stage, yaw)
        box_orientation = stage_box_orientation(yaw)
        self.body.set_world_pose(
            position=np.asarray(center, dtype=float),
            orientation=np.asarray(
                [box_orientation[3], *box_orientation[:3]], dtype=float
            ),
        )
        self.body.set_linear_velocity(np.zeros(3, dtype=float))
        self.body.set_angular_velocity(np.zeros(3, dtype=float))
        self._dynamic_command = (0.0, 0.0, 0.0)
        self._dynamic_target_yaw = yaw

    def center(self, root_position_stage: np.ndarray, yaw: float) -> np.ndarray:
        rotated_offset = rotate_stage_planar(self.center_offset_stage, yaw)
        return np.asarray(root_position_stage, dtype=float) + rotated_offset

    def set_pose(self, root_position_stage: np.ndarray, yaw: float) -> None:
        if self.dynamic and self.body is not None:
            raise RuntimeError("cannot teleport an initialized dynamic robot body")
        center = self.center(root_position_stage, yaw)
        self.translate_op.Set(Gf.Vec3d(*[float(value) for value in center]))
        orientation = stage_box_orientation(yaw)
        self.orient_op.Set(Gf.Quatd(orientation[3], Gf.Vec3d(*orientation[:3])))

    @staticmethod
    def _path_from_hit(hit, snake_name: str, camel_name: str) -> str:
        if isinstance(hit, dict):
            return str(hit.get(camel_name, hit.get(snake_name, "")))
        return str(getattr(hit, snake_name, ""))

    @classmethod
    def _is_robot_hit(cls, hit) -> bool:
        collision = cls._path_from_hit(hit, "collision", "collision")
        rigid_body = cls._path_from_hit(hit, "rigid_body", "rigidBody")
        return any(
            path.startswith(ROBOT_PRIM) or path.startswith(ROBOT_COLLISION_PRIM)
            for path in (collision, rigid_body)
        )

    @classmethod
    def _hit_path(cls, hit) -> str:
        return cls._path_from_hit(hit, "collision", "collision") or cls._path_from_hit(
            hit, "rigid_body", "rigidBody"
        )

    def blocking_path(
        self,
        current_position_stage: np.ndarray,
        current_yaw: float,
        candidate_position_stage: np.ndarray,
        candidate_yaw: float,
    ) -> str | None:
        """Return the first obstacle that a swept/rotated collision box hits."""
        if np.allclose(
            current_position_stage,
            candidate_position_stage,
            rtol=0.0,
            atol=1.0e-9,
        ) and math.isclose(current_yaw, candidate_yaw, rel_tol=0.0, abs_tol=1.0e-9):
            return None
        half_extent = carb.Float3(
            *[float(value) for value in self.half_extent_stage]
        )
        current_center = self.center(current_position_stage, current_yaw)
        translation_target = self.center(candidate_position_stage, current_yaw)
        delta = translation_target - current_center
        distance = float(np.linalg.norm(delta))
        blocked_paths: list[str] = []

        def report_sweep(hit) -> bool:
            if not self._is_robot_hit(hit):
                hit_distance = float(getattr(hit, "distance", math.inf))
                if hit_distance <= distance + COLLISION_STOP_MARGIN_M / STAGE_METERS_PER_UNIT:
                    blocked_paths.append(self._hit_path(hit))
            return True

        if distance > 1.0e-9:
            direction = delta / distance
            self.query.sweep_box_all(
                half_extent,
                carb.Float3(*[float(value) for value in current_center]),
                carb.Float4(*stage_box_orientation(current_yaw)),
                carb.Float3(*[float(value) for value in direction]),
                distance + COLLISION_STOP_MARGIN_M / STAGE_METERS_PER_UNIT,
                report_sweep,
                False,
            )
            if blocked_paths:
                return blocked_paths[0]

        candidate_center = self.center(candidate_position_stage, candidate_yaw)

        def report_overlap(hit) -> bool:
            if not self._is_robot_hit(hit):
                blocked_paths.append(self._hit_path(hit))
            return True

        self.query.overlap_box(
            half_extent,
            carb.Float3(*[float(value) for value in candidate_center]),
            carb.Float4(*stage_box_orientation(candidate_yaw)),
            report_overlap,
            False,
        )
        return blocked_paths[0] if blocked_paths else None


def create_collision_validation_obstacle(
    stage: Usd.Stage, collision_center_stage: np.ndarray
) -> None:
    """Create a temporary wall used only by --test-collision-obstacle."""
    dimensions_m = np.asarray([0.15, 2.0, 2.0], dtype=float)
    center = np.asarray(collision_center_stage, dtype=float).copy()
    center[0] += 0.8 / STAGE_METERS_PER_UNIT
    cube = UsdGeom.Cube.Define(stage, COLLISION_TEST_OBSTACLE_PRIM)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.1, 0.1)])
    xformable = UsdGeom.Xformable(cube)
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(value) for value in center])
    )
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(
            *[
                float(value) / STAGE_METERS_PER_UNIT
                for value in dimensions_m
            ]
        )
    )
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


class RosCmdVel:
    """Exchange commands/telemetry with the system-Humble ROS UDP bridge."""

    def __init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((CMD_VEL_UDP_HOST, CMD_VEL_UDP_PORT))
        self.socket.setblocking(False)
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self.telemetry_encoder = TelemetryEncoder()
        self.reset_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.reset_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.reset_socket.bind((CMD_VEL_UDP_HOST, RESET_UDP_PORT))
        self.reset_socket.setblocking(False)
        self._large_telemetry_reported = False
        self._command = (0.0, 0.0, 0.0)
        self._last_received_sim_time = -math.inf
        self._command_sequence_id = 0
        self._bridge_receive_sim_time = -math.inf
        self.received_count = 0
        self._pending_reset: dict[str, object] | None = None

    def spin_once(self) -> None:
        while True:
            try:
                packet, _ = self.socket.recvfrom(CMD_VEL_PACKET.size)
            except BlockingIOError:
                break
            if len(packet) != CMD_VEL_PACKET.size:
                continue
            version, sequence_id, bridge_sim_time, vx, vy, wz = CMD_VEL_PACKET.unpack(packet)
            if version != COMMAND_PROTOCOL_VERSION or not all(
                math.isfinite(value) for value in (bridge_sim_time, vx, vy, wz)
            ):
                continue
            self._command = clamp_twist(vx, vy, wz)
            self._last_received_sim_time = bridge_sim_time
            self._bridge_receive_sim_time = bridge_sim_time
            self._command_sequence_id = int(sequence_id)
            self.received_count += 1
            if self.received_count == 1:
                print(
                    "[WAREHOUSE-ROBOT] First relayed /cmd_vel received: "
                    f"vx={self._command[0]:.3f}, vy={self._command[1]:.3f}, "
                    f"wz={self._command[2]:.3f}",
                    flush=True,
                )
        while True:
            try:
                packet, _ = self.reset_socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                request = json.loads(packet.decode("utf-8"))
                pose = request.get("pose")
                if (
                    request.get("schema") != "isaac_reset_request/v1"
                    or request.get("frame_id") not in {"map", "odom"}
                    or not isinstance(pose, list)
                    or len(pose) != 3
                    or not all(math.isfinite(float(value)) for value in pose)
                ):
                    raise ValueError("invalid reset contract")
                self._pending_reset = {
                    "sequence_id": int(request["sequence_id"]),
                    "pose": tuple(float(value) for value in pose),
                }
            except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue

    def pop_reset_request(self) -> dict[str, object] | None:
        request, self._pending_reset = self._pending_reset, None
        return request

    def command(self, sim_time: float) -> tuple[tuple[float, float, float], bool, float]:
        """Return applied command plus watchdog state in the simulation clock."""
        age = sim_time - self._last_received_sim_time
        watchdog = not math.isfinite(age) or age < 0.0 or age > COMMAND_TIMEOUT_SEC
        return (self._command if not watchdog else (0.0, 0.0, 0.0), watchdog, age)

    def actuation_metadata(self) -> dict[str, object]:
        return {
            "command_received": self.received_count > 0,
            "command_sequence_id": self._command_sequence_id,
            "bridge_receive_sim_time": finite_or_none(
                self._bridge_receive_sim_time
            ),
            "received_command": list(self._command),
        }

    def send_telemetry(self, payload: dict[str, object]) -> None:
        packets = self.telemetry_encoder.encode(payload)
        if not self._large_telemetry_reported and (
            len(packets) > 1 or packets[0].startswith(COMPRESSED_MAGIC)
        ):
            print(
                "[WAREHOUSE-ROBOT] Large telemetry framing active: "
                f"datagrams={len(packets)}, bytes={[len(packet) for packet in packets]}",
                flush=True,
            )
            self._large_telemetry_reported = True
        for packet in packets:
            self.telemetry_socket.sendto(
                packet, (CMD_VEL_UDP_HOST, TELEMETRY_UDP_PORT)
            )

    def send_shutdown(self, sim_time: float) -> None:
        self.send_telemetry(
            {
                "schema": TELEMETRY_SCHEMA,
                "sim_time": max(0.0, sim_time),
                "event": "shutdown",
            }
        )

    def close(self) -> None:
        self.socket.close()
        self.telemetry_socket.close()
        self.reset_socket.close()


def make_laser_ranges(
    robot_position: np.ndarray,
    robot_yaw: float,
    mount_translation: tuple[float, float, float],
    mount_yaw: float,
    leg_snapshot: AnalyticLegSnapshot | None = None,
) -> tuple[list[float | None], dict[str, object]]:
    """Raycast one planar scan in the loaded stage's navigation plane."""
    scale = STAGE_METERS_PER_UNIT
    mount_x, mount_y, mount_z = mount_translation
    sensor_position = np.asarray(robot_position, dtype=float) + rotate_stage_planar(
        stage_from_ros_offset(mount_x, mount_y, mount_z), robot_yaw
    )
    sensor_yaw = robot_yaw + mount_yaw
    minimum_stage = LIDAR_RANGE_MIN_M / scale
    maximum_query_stage = (LIDAR_RANGE_MAX_M - LIDAR_RANGE_MIN_M) / scale
    angle_increment = 2.0 * math.pi / LIDAR_SAMPLE_COUNT
    query = omni.physx.get_physx_scene_query_interface()
    world_angles = (
        sensor_yaw
        - math.pi
        + np.arange(LIDAR_SAMPLE_COUNT, dtype=float) * angle_increment
    )
    cosine = np.cos(world_angles)
    sine = np.sin(world_angles)
    if STAGE_UP_AXIS == "Z":
        directions = np.column_stack(
            (cosine, sine, np.zeros(LIDAR_SAMPLE_COUNT, dtype=float))
        )
    else:
        directions = np.column_stack(
            (cosine, np.zeros(LIDAR_SAMPLE_COUNT, dtype=float), -sine)
        )
    origins = sensor_position[None, :] + directions * minimum_stage
    if leg_snapshot is None:
        analytic_distances = np.full(LIDAR_SAMPLE_COUNT, np.inf, dtype=float)
        analytic_leg_indices = np.full(LIDAR_SAMPLE_COUNT, -1, dtype=int)
    else:
        analytic_distances, analytic_leg_indices = nearest_ray_capsule_intersections(
            origins,
            directions,
            leg_snapshot.segment_starts,
            leg_snapshot.segment_ends,
            leg_snapshot.radii,
        )

    stats: dict[str, object] = {
        "total_beams": LIDAR_SAMPLE_COUNT,
        "fallback_beams": 0,
        "fallback_ratio": 0.0,
        "ignored_closest_hits": 0,
        "ignored_all_hits": 0,
        "analytic_accepted_beams": 0,
        "physx_accepted_beams": 0,
        "no_return_beams": 0,
        "analytic_hits_by_leg": {},
        "unknown_character_hit_paths": [],
    }
    unknown_character_hit_paths: set[str] = set()
    analytic_hits_by_leg: dict[str, list[tuple[int, float]]] = {}
    ranges: list[float | None] = []
    for index in range(LIDAR_SAMPLE_COUNT):
        direction = directions[index]
        origin = origins[index]
        hit = query.raycast_closest(tuple(origin), tuple(direction), maximum_query_stage)
        physx_distance_stage = math.inf
        if hit["hit"]:
            collision_path = str(hit.get("collision", ""))
            if leg_snapshot is not None and is_ignored_person_query_collider(
                collision_path
            ):
                stats["fallback_beams"] = int(stats["fallback_beams"]) + 1
                stats["ignored_closest_hits"] = (
                    int(stats["ignored_closest_hits"]) + 1
                )

                def report_all(candidate: object) -> bool:
                    nonlocal physx_distance_stage
                    candidate_path = str(
                        scene_query_hit_value(candidate, "collision", "")
                    )
                    if is_ignored_person_query_collider(candidate_path):
                        stats["ignored_all_hits"] = int(stats["ignored_all_hits"]) + 1
                        return True
                    if "/World/Characters/" in candidate_path:
                        unknown_character_hit_paths.add(candidate_path)
                    candidate_distance = float(
                        scene_query_hit_value(candidate, "distance", math.inf)
                    )
                    if candidate_distance < physx_distance_stage:
                        physx_distance_stage = candidate_distance
                    return True

                query.raycast_all(
                    tuple(origin),
                    tuple(direction),
                    maximum_query_stage,
                    report_all,
                )
            else:
                physx_distance_stage = float(hit["distance"])
                if "/World/Characters/" in collision_path:
                    unknown_character_hit_paths.add(collision_path)

        analytic_distance_stage = float(analytic_distances[index])
        if analytic_distance_stage < physx_distance_stage:
            selected_distance_stage = analytic_distance_stage
            stats["analytic_accepted_beams"] = (
                int(stats["analytic_accepted_beams"]) + 1
            )
            leg_index = int(analytic_leg_indices[index])
            if leg_snapshot is not None and leg_index >= 0:
                label = leg_snapshot.labels[leg_index]
                analytic_hits_by_leg.setdefault(label, []).append(
                    (
                        index,
                        LIDAR_RANGE_MIN_M + analytic_distance_stage * scale,
                    )
                )
        else:
            selected_distance_stage = physx_distance_stage
            if math.isfinite(selected_distance_stage):
                stats["physx_accepted_beams"] = int(stats["physx_accepted_beams"]) + 1

        if math.isfinite(selected_distance_stage):
            distance_m = LIDAR_RANGE_MIN_M + selected_distance_stage * scale
            ranges.append(min(LIDAR_RANGE_MAX_M, distance_m))
        else:
            # JSON has no portable Infinity value; the ROS bridge maps null to
            # LaserScan's conventional +inf no-return representation.
            ranges.append(None)
            stats["no_return_beams"] = int(stats["no_return_beams"]) + 1
    stats["fallback_ratio"] = int(stats["fallback_beams"]) / LIDAR_SAMPLE_COUNT
    stats["analytic_hits_by_leg"] = {
        label: {
            "count": len(hits),
            "mean_angle_rad": math.atan2(
                sum(math.sin(-math.pi + index * angle_increment) for index, _ in hits),
                sum(math.cos(-math.pi + index * angle_increment) for index, _ in hits),
            ),
            "range_m_min": min(distance for _, distance in hits),
            "range_m_max": max(distance for _, distance in hits),
            "range_m_mean": sum(distance for _, distance in hits) / len(hits),
        }
        for label, hits in analytic_hits_by_leg.items()
    }
    stats["unknown_character_hit_paths"] = sorted(unknown_character_hit_paths)
    return ranges, stats


def make_dual_scan_payload(
    robot_position: np.ndarray,
    robot_yaw: float,
    people_lidar: PhysxAnalyticPeopleLidar | None = None,
    sim_time: float = 0.0,
) -> dict[str, object]:
    pair_started = time.perf_counter()
    leg_snapshot = people_lidar.snapshot(sim_time) if people_lidar is not None else None
    angle_increment = 2.0 * math.pi / LIDAR_SAMPLE_COUNT
    metadata = {
        "angle_min": -math.pi,
        "angle_increment": angle_increment,
        "range_min": LIDAR_RANGE_MIN_M,
        "range_max": LIDAR_RANGE_MAX_M,
        "scan_time": LIDAR_PUBLISH_PERIOD_SEC,
    }
    front_ranges, front_stats = make_laser_ranges(
        robot_position,
        robot_yaw,
        (0.2, 0.13, 0.208),
        0.0,
        leg_snapshot,
    )
    rear_ranges, rear_stats = make_laser_ranges(
        robot_position,
        robot_yaw,
        (-0.2, -0.13, 0.208),
        math.pi,
        leg_snapshot,
    )
    payload = {
        "scan_01": {
            **metadata,
            "ranges": front_ranges,
        },
        "scan_02": {
            **metadata,
            "ranges": rear_ranges,
        },
    }
    if people_lidar is not None and leg_snapshot is not None:
        people_lidar.record_pair(
            leg_snapshot,
            {"scan_01": front_stats, "scan_02": rear_stats},
            (time.perf_counter() - pair_started) * 1000.0,
        )
    return payload


class RtxDualLidar:
    """Read two native RTX lidars and project their GMO returns to 360 slots.

    Isaac Sim 6.0.1 creates a Hydra render product for every ``LidarSensor``.
    Creating or destroying several of those products while the timeline is
    already advancing can race Hydra/FIF graph reconfiguration.  The caller
    therefore constructs this object while the timeline is stopped.  Each
    sensor is attached separately with several stopped-timeline application
    updates between attachments, then the caller starts playback and waits for
    one valid frame from both sensors.
    """

    _specs = (
        ("scan_01", "RtxLidarFront", (0.2, 0.13, 0.208), 0.0),
        ("scan_02", "RtxLidarRear", (-0.2, -0.13, 0.208), math.pi),
    )

    def __init__(
        self,
        stage_meters_per_unit: float,
        robot_position_stage: np.ndarray,
        robot_yaw: float,
    ) -> None:
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import (
            Lidar,
            LidarSensor,
            parse_generic_model_output_data,
        )
        import isaacsim.sensors.experimental.rtx.generic_model_output as gmo_utils
        from omni.replicator.core import Writer

        if stage_meters_per_unit <= 0.0:
            raise ValueError("stage_meters_per_unit must be positive")
        self._parse_gmo = parse_generic_model_output_data
        self._expected_magic = int(gmo_utils.getMagicNumberGMO())
        self._stage_meters_per_unit = stage_meters_per_unit
        self._lidars: dict[str, object] = {}
        self._sensors: dict[str, object] = {}
        self._writers: dict[str, object] = {}
        self._last_writer_references: dict[str, str] = {}
        self._frame_lock = threading.Lock()
        self._frame_queues: dict[
            str, deque[tuple[tuple[int, int], dict[str, object]]]
        ] = {
            name: deque(maxlen=RTX_FRAME_QUEUE_SIZE)
            for name, _prim, _translation, _yaw in self._specs
        }
        self._last_sensor_timestamp_ns: dict[str, int] = {}
        self._seen_keys: dict[str, set[tuple[int, int]]] = {
            name: set() for name in self._frame_queues
        }
        self._seen_key_order: dict[str, deque[tuple[int, int]]] = {
            name: deque() for name in self._frame_queues
        }
        self._dropped_unpaired: dict[str, int] = {
            name: 0 for name in self._frame_queues
        }
        self._paired_timestamps_ns: deque[int] = deque(maxlen=256)
        self._paired_wall_times: deque[float] = deque(maxlen=256)
        self._latest_stats: dict[str, dict[str, float | int]] = {}
        self._poll_counts: dict[str, int] = {}
        self._buffer_counts: dict[str, int] = {}
        self._no_data_counts: dict[str, int] = {}
        self._stale_data_counts: dict[str, int] = {}
        self._callback_counts: dict[str, int] = {}
        self._zero_element_counts: dict[str, int] = {}
        self._invalid_magic_counts: dict[str, int] = {}
        self._parse_error_counts: dict[str, int] = {}
        self._last_gmo_headers: dict[str, tuple[int, int, int, int]] = {}
        self._published_pairs = 0
        self._first_reported = False
        # Writer callbacks can still be delivered while Replicator/Hydra is
        # tearing down render products.  Ignore those late callbacks before
        # their GMO buffers become invalid.
        self._closing = False
        capture_setting = carb.settings.get_settings().get(
            "/omni/replicator/captureOnPlay"
        )
        self._capture_on_play_before = bool(capture_setting)
        self._set_capture_on_play = rep.orchestrator.set_capture_on_play
        self._replicator_stop = rep.orchestrator.stop
        self._replicator_get_status = rep.orchestrator.get_status
        self._capture_on_play_armed = False

        class APipelineRtxSensorDriver(Writer):
            """Receive native RTX GMO frames from the OnFrame Writer graph."""

            def __init__(writer_self) -> None:
                writer_self.data_structure = "renderProduct"
                writer_self.annotators = [
                    rep.annotators.get("GenericModelOutput")
                ]
                writer_self._receiver = None
                writer_self._sensor_name = ""

            def initialize(
                writer_self, receiver: object, sensor_name: str
            ) -> None:
                writer_self._receiver = receiver
                writer_self._sensor_name = sensor_name

            def write(writer_self, data: dict[str, object]) -> None:
                receiver = writer_self._receiver
                if receiver is not None and not receiver._closing:
                    name = writer_self._sensor_name
                    receiver._callback_counts[name] = (
                        receiver._callback_counts.get(name, 0) + 1
                    )
                    receiver._consume_writer_payload(name, data)

        rep.WriterRegistry.register(APipelineRtxSensorDriver)
        # Match Isaac Sim's own RTX sensor tests: arm capture while stopped,
        # attach all render products, and let the PLAY timeline event start the
        # Orchestrator.  Starting it explicitly before PLAY leaves OnFrame
        # writer triggers without the normal playback transition.
        self._set_capture_on_play(True)
        self._capture_on_play_armed = True

        common_attributes = {
            "omni:sensor:Core:nearRangeM": LIDAR_RANGE_MIN_M,
            "omni:sensor:Core:farRangeM": LIDAR_RANGE_MAX_M,
            "omni:sensor:Core:scanRateBaseHz": LIDAR_RATE_HZ,
            "omni:sensor:Core:outputFrameOfReference": "SENSOR",
        }
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Cannot create RTX lidars without an open USD stage")
        stage.DefinePrim(RTX_SENSOR_ROOT, "Xform")
        for topic_name, prim_name, mount_translation, mount_yaw in self._specs:
            sensor_path = f"{RTX_SENSOR_ROOT}/{prim_name}"
            if stage.GetPrimAtPath(sensor_path).IsValid():
                raise RuntimeError(f"RTX lidar prim already exists: {sensor_path}")

            # Do not use Lidar.create(usd_path=...) here.  That path invokes
            # Isaac's MetricAssembler.  When a metre-scale sensor is nested
            # below the already-converted robot in this centimetre stage, the
            # assembler scales the first sensor by 100 and then the second by
            # 10,000.  Apart from putting all returns kilometres away, that
            # corrupts the shared Fabric/Hydra hierarchy.  A direct USD
            # reference composes the unit-agnostic OmniLidar definition
            # without modifying this stage's transform hierarchy.
            sensor_prim = stage.DefinePrim(sensor_path, "OmniLidar")
            if not sensor_prim.GetReferences().AddReference(str(RTX_LIDAR_USD)):
                raise RuntimeError(
                    f"Could not add raw RTX lidar USD reference: {RTX_LIDAR_USD}"
                )
            lidar = Lidar(
                path=sensor_path,
                accumulate_outputs=True,
                aux_output_level="FULL",
                tick_rate=float(LIDAR_RATE_HZ),
                attributes=common_attributes,
            )
            composed_prim = stage.GetPrimAtPath(sensor_path)
            authored_timing = {}
            for attribute_name in (
                "omni:sensor:tickRate",
                "omni:sensor:Core:scanRateBaseHz",
                "omni:sensor:Core:patternFiringRateHz",
                "omni:sensor:Core:reportRateBaseHz",
                "omni:sensor:Core:numberOfChannels",
                "omni:sensor:Core:numberOfEmitters",
            ):
                attribute = composed_prim.GetAttribute(attribute_name)
                authored_timing[attribute_name] = (
                    attribute.Get() if attribute.IsValid() else None
                )
            effective_tick_rate = authored_timing["omni:sensor:tickRate"]
            effective_scan_rate = authored_timing[
                "omni:sensor:Core:scanRateBaseHz"
            ]
            if (
                effective_tick_rate is None
                or effective_scan_rate is None
                or not math.isclose(
                    float(effective_tick_rate), float(LIDAR_RATE_HZ), abs_tol=1e-6
                )
                or not math.isclose(
                    float(effective_scan_rate), float(LIDAR_RATE_HZ), abs_tol=1e-6
                )
            ):
                raise RuntimeError(
                    "RTX lidar timing attributes did not accept the requested rate: "
                    f"requested={LIDAR_RATE_HZ}, readback={authored_timing}"
                )
            print(
                "[WAREHOUSE-ROBOT] RTX lidar timing readback: "
                f"topic={topic_name}, values={authored_timing}",
                flush=True,
            )
            self._lidars[topic_name] = lidar
            # Keep an explicitly attached GMO annotator as well as the Writer,
            # exactly as the upstream LidarSensor tests do.  The Writer is the
            # primary path; the annotator provides an independent fallback and
            # makes warmup failures observable instead of silently timing out.
            sensor = LidarSensor(lidar, annotators=["generic-model-output"])
            writer = sensor.attach_writer(
                "APipelineRtxSensorDriver",
                receiver=self,
                sensor_name=topic_name,
            )
            self._sensors[topic_name] = sensor
            self._writers[topic_name] = writer
            self._poll_counts[topic_name] = 0
            self._buffer_counts[topic_name] = 0
            self._no_data_counts[topic_name] = 0
            self._stale_data_counts[topic_name] = 0
            self._callback_counts[topic_name] = 0
            self._zero_element_counts[topic_name] = 0
            self._invalid_magic_counts[topic_name] = 0
            self._parse_error_counts[topic_name] = 0
            print(
                "[WAREHOUSE-ROBOT] Native RTX lidar attached before playback: "
                f"topic={topic_name}, prim={sensor_path}",
                flush=True,
            )
            # Flush the first render-product/Writer graph before authoring the
            # second.  The timeline is deliberately stopped here; these app
            # updates synchronize Hydra without advancing IRA or robot motion.
            for _ in range(RTX_SENSOR_ATTACH_SYNC_FRAMES):
                simulation_app.update()
        self.set_robot_pose(robot_position_stage, robot_yaw, validate=True)
        print(
            "[WAREHOUSE-ROBOT] Native RTX dual lidar prepared before playback: "
            f"profile={RTX_LIDAR_PROFILE}, config={RTX_LIDAR_USD.name}, "
            f"asset_sha256={RTX_LIDAR_ASSET_SHA256}, "
            f"samples={LIDAR_SAMPLE_COUNT}, "
            f"rate_hz={LIDAR_RATE_HZ}, "
            "aux_output=FULL, intensity=GMO.scalar->ROS 0..255",
            flush=True,
        )

    def set_robot_pose(
        self,
        robot_position_stage: np.ndarray,
        robot_yaw: float,
        *,
        validate: bool = False,
    ) -> None:
        """Keep world-root sensors aligned with the robot and ROS static TF."""
        robot_position_stage = np.asarray(robot_position_stage, dtype=float)
        for topic_name, _prim_name, mount_translation, mount_yaw in self._specs:
            mount_x, mount_y, mount_z = mount_translation
            position = robot_position_stage + rotate_stage_planar(
                stage_from_ros_offset(mount_x, mount_y, mount_z), robot_yaw
            )
            self._lidars[topic_name].set_world_poses(
                positions=np.asarray([position], dtype=float),
                orientations=np.asarray(
                    [robot_orientation(robot_yaw + mount_yaw)], dtype=float
                ),
            )
            if not validate:
                continue
            lidar_matrix = np.asarray(
                UsdGeom.Xformable(
                    self._lidars[topic_name].prims[0]
                ).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
                dtype=float,
            )
            actual_position = lidar_matrix[3, :3]
            basis_scale = np.asarray(
                [
                    np.linalg.norm(lidar_matrix[index, :3])
                    for index in range(3)
                ],
                dtype=float,
            )
            if not np.allclose(actual_position, position, rtol=0.0, atol=1.0e-3):
                raise RuntimeError(
                    "RTX lidar world position does not match its robot mount: "
                    f"topic={topic_name}, expected={position.tolist()}, "
                    f"actual={actual_position.tolist()}"
                )
            if not np.allclose(basis_scale, 1.0, rtol=0.0, atol=1.0e-5):
                raise RuntimeError(
                    "RTX lidar inherited an unsafe metric scale: "
                    f"topic={topic_name}, basis_scale={basis_scale.tolist()}"
                )
            print(
                "[WAREHOUSE-ROBOT] Native RTX lidar transform validated: "
                f"topic={topic_name}, translation_stage="
                f"{actual_position.round(4).tolist()}, basis_scale="
                f"{basis_scale.round(6).tolist()}",
                flush=True,
            )

    @property
    def latest_stats(self) -> dict[str, dict[str, float | int]]:
        return self._latest_stats

    @property
    def published_pairs(self) -> int:
        return self._published_pairs

    @property
    def measured_pair_rate_hz(self) -> float | None:
        if len(self._paired_timestamps_ns) < 2:
            return None
        span_sec = (
            self._paired_timestamps_ns[-1] - self._paired_timestamps_ns[0]
        ) / 1.0e9
        if span_sec <= 0.0:
            return None
        return (len(self._paired_timestamps_ns) - 1) / span_sec

    @property
    def measured_pair_wall_rate_hz(self) -> float | None:
        if len(self._paired_wall_times) < 2:
            return None
        span_sec = self._paired_wall_times[-1] - self._paired_wall_times[0]
        if span_sec <= 0.0:
            return None
        return (len(self._paired_wall_times) - 1) / span_sec

    @property
    def pairing_diagnostics(self) -> dict[str, object]:
        timestamp_deltas_ms = [
            round((current - previous) / 1.0e6, 6)
            for previous, current in zip(
                self._paired_timestamps_ns,
                list(self._paired_timestamps_ns)[1:],
            )
        ]
        return {
            "requested_rate_hz": LIDAR_RATE_HZ,
            "measured_pair_rate_hz": self.measured_pair_rate_hz,
            "measured_pair_wall_rate_hz": self.measured_pair_wall_rate_hz,
            "published_pairs": self._published_pairs,
            "recent_native_timestamp_deltas_ms": timestamp_deltas_ms[-12:],
            "dropped_unpaired": dict(self._dropped_unpaired),
            "queued_frames": {
                name: len(queue) for name, queue in self._frame_queues.items()
            },
        }

    @property
    def warmup_diagnostics(self) -> dict[str, dict[str, object]]:
        return {
            name: {
                "polls": self._poll_counts.get(name, 0),
                "buffers": self._buffer_counts.get(name, 0),
                "no_data": self._no_data_counts.get(name, 0),
                "stale_data": self._stale_data_counts.get(name, 0),
                "callbacks": self._callback_counts.get(name, 0),
                "zero_elements": self._zero_element_counts.get(name, 0),
                "invalid_magic": self._invalid_magic_counts.get(name, 0),
                "parse_errors": self._parse_error_counts.get(name, 0),
                "capture_on_play": int(self._capture_on_play_armed),
                "orchestrator_status": str(self._replicator_get_status()),
                "last_gmo_header": self._last_gmo_headers.get(name),
                "valid_frames": int(name in self._latest_stats),
                "queued_frames": len(self._frame_queues.get(name, ())),
                "dropped_unpaired": self._dropped_unpaired.get(name, 0),
            }
            for name in self._sensors
        }

    @staticmethod
    def _gmo_key(gmo: object) -> tuple[int, int]:
        return (
            int(getattr(gmo, "frameId", 0)),
            int(getattr(gmo, "timestampNs", 0)),
        )

    def _accept_gmo(self, name: str, gmo: object) -> None:
        count = int(gmo.numElements)
        if count <= 0:
            return
        key = self._gmo_key(gmo)
        with self._frame_lock:
            seen = self._seen_keys[name]
            if key in seen:
                return
            seen.add(key)
            key_order = self._seen_key_order[name]
            key_order.append(key)
            if len(key_order) > RTX_FRAME_QUEUE_SIZE * 2:
                seen.discard(key_order.popleft())
        coords_name = getattr(gmo.elementsCoordsType, "name", "")
        ranges, intensities, stats = project_rtx_returns(
            gmo.x[:count],
            gmo.y[:count],
            gmo.z[:count],
            gmo.scalar[:count],
            cartesian=coords_name == "CARTESIAN",
            sample_count=LIDAR_SAMPLE_COUNT,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / LIDAR_SAMPLE_COUNT,
            range_min=LIDAR_RANGE_MIN_M,
            range_max=LIDAR_RANGE_MAX_M,
        )
        stats["gmo_frame_id"] = key[0]
        stats["gmo_timestamp_ns"] = key[1]
        stats["scan_complete"] = int(getattr(gmo, "scanComplete", 0))
        self._latest_stats[name] = stats
        with self._frame_lock:
            previous_timestamp_ns = self._last_sensor_timestamp_ns.get(name)
            actual_scan_time = LIDAR_PUBLISH_PERIOD_SEC
            if (
                previous_timestamp_ns is not None
                and key[1] > previous_timestamp_ns
            ):
                actual_scan_time = (key[1] - previous_timestamp_ns) / 1.0e9
            if (
                key[1] >= 0
                and (
                    previous_timestamp_ns is None
                    or key[1] > previous_timestamp_ns
                )
            ):
                self._last_sensor_timestamp_ns[name] = key[1]
        payload = {
            "angle_min": -math.pi,
            "angle_increment": 2.0 * math.pi / LIDAR_SAMPLE_COUNT,
            "range_min": LIDAR_RANGE_MIN_M,
            "range_max": LIDAR_RANGE_MAX_M,
            "scan_time": actual_scan_time,
            "sensor_timestamp_ns": key[1],
            "sensor_frame_id": key[0],
            "ranges": ranges,
            "intensities": intensities,
            "intensity_source": "isaac_rtx_gmo_scalar",
        }
        with self._frame_lock:
            queue = self._frame_queues[name]
            if len(queue) == queue.maxlen:
                self._dropped_unpaired[name] += 1
            queue.append((key, payload))
            if len(queue) >= 2:
                previous_key = queue[-2][0]
                previous_order = (
                    previous_key[1] if previous_key[1] > 0 else previous_key[0]
                )
                current_order = key[1] if key[1] > 0 else key[0]
                if current_order < previous_order:
                    ordered = sorted(
                        queue,
                        key=lambda item: (
                            item[0][1] if item[0][1] > 0 else item[0][0]
                        ),
                    )
                    queue.clear()
                    queue.extend(ordered)

    def _consume_gmo_raw(self, name: str, raw_data: object) -> None:
        """Validate and consume one raw GenericModelOutput buffer."""
        if raw_data is None:
            self._no_data_counts[name] = self._no_data_counts.get(name, 0) + 1
            return
        self._buffer_counts[name] = self._buffer_counts.get(name, 0) + 1
        try:
            gmo = self._parse_gmo(raw_data)
        except Exception:
            self._parse_error_counts[name] = (
                self._parse_error_counts.get(name, 0) + 1
            )
            return
        # Check the common header before touching lidar-only auxiliary fields.
        # An annotator can briefly expose an uninitialized buffer while its
        # render product starts; reading scanComplete from that default GMO
        # makes the native binding emit misleading modality warnings.
        if int(getattr(gmo, "magicNumber", 0)) != self._expected_magic:
            self._invalid_magic_counts[name] = (
                self._invalid_magic_counts.get(name, 0) + 1
            )
            return
        self._last_gmo_headers[name] = (
            int(getattr(gmo, "frameId", 0)),
            int(getattr(gmo, "timestampNs", 0)),
            int(getattr(gmo, "scanComplete", 0)),
            int(getattr(gmo, "numElements", 0)),
        )
        if int(gmo.numElements) <= 0:
            self._zero_element_counts[name] = (
                self._zero_element_counts.get(name, 0) + 1
            )
            return
        self._accept_gmo(name, gmo)

    def _consume_writer_payload(self, name: str, data: object) -> None:
        """Extract GMO data from one renderProduct-structured Writer payload."""
        if not isinstance(data, dict):
            self._no_data_counts[name] = self._no_data_counts.get(name, 0) + 1
            return
        reference = repr(data.get("reference_time"))
        if self._last_writer_references.get(name) == reference:
            self._stale_data_counts[name] = (
                self._stale_data_counts.get(name, 0) + 1
            )
            return
        self._last_writer_references[name] = reference
        raw_data = None
        render_products = data.get("renderProducts")
        if isinstance(render_products, dict):
            for render_product in render_products.values():
                if not isinstance(render_product, dict):
                    continue
                raw_data = render_product.get("GenericModelOutput")
                if isinstance(raw_data, dict):
                    raw_data = raw_data.get("data")
                if raw_data is not None:
                    break
        self._consume_gmo_raw(name, raw_data)

    def _read_sensors(self) -> None:
        """Read direct GMO annotators; Writer callbacks arrive independently."""
        for name, sensor in self._sensors.items():
            self._poll_counts[name] = self._poll_counts.get(name, 0) + 1
            try:
                raw_data, _info = sensor.get_data("generic-model-output")
            except Exception:
                self._parse_error_counts[name] = (
                    self._parse_error_counts.get(name, 0) + 1
                )
                continue
            self._consume_gmo_raw(name, raw_data)

    def poll_payload(self) -> dict[str, object] | None:
        self._read_sensors()
        front_name, rear_name = self._specs[0][0], self._specs[1][0]
        payload = None
        with self._frame_lock:
            front_queue = self._frame_queues[front_name]
            rear_queue = self._frame_queues[rear_name]
            while front_queue and rear_queue:
                front_key, front_payload = front_queue[0]
                rear_key, rear_payload = rear_queue[0]
                same_capture = front_key == rear_key
                if same_capture:
                    front_queue.popleft()
                    rear_queue.popleft()
                    payload = {
                        front_name: front_payload,
                        rear_name: rear_payload,
                    }
                    paired_timestamp_ns = max(front_key[1], rear_key[1])
                    if (
                        paired_timestamp_ns >= 0
                        and (
                            not self._paired_timestamps_ns
                            or paired_timestamp_ns
                            > self._paired_timestamps_ns[-1]
                        )
                    ):
                        self._paired_timestamps_ns.append(paired_timestamp_ns)
                    break
                front_order = front_key[1] if front_key[1] > 0 else front_key[0]
                rear_order = rear_key[1] if rear_key[1] > 0 else rear_key[0]
                if front_order < rear_order:
                    front_queue.popleft()
                    self._dropped_unpaired[front_name] += 1
                else:
                    rear_queue.popleft()
                    self._dropped_unpaired[rear_name] += 1
        if payload is None:
            return None
        self._published_pairs += 1
        self._paired_wall_times.append(time.monotonic())
        if not self._first_reported:
            self._first_reported = True
            print(
                "[WAREHOUSE-ROBOT] First native RTX dual scan ready: "
                + json.dumps(self._latest_stats, sort_keys=True),
                flush=True,
            )
        return payload

    def begin_close(self) -> None:
        """Stop accepting Writer callbacks before the timeline is stopped."""
        self._closing = True

    def close(self) -> None:
        self.begin_close()
        try:
            self._replicator_stop()
            simulation_app.update()
        except Exception:
            pass
        for sensor in self._sensors.values():
            try:
                sensor._invalidate_sensor()
            except Exception:
                pass
            # Match the lifecycle used by Isaac's own RTX examples: destroy
            # one render product at a time and let Hydra consume the graph
            # change before destroying the next one.
            try:
                simulation_app.update()
            except Exception:
                pass
        self._sensors.clear()
        self._writers.clear()
        self._lidars.clear()
        with self._frame_lock:
            self._frame_queues.clear()
        try:
            self._set_capture_on_play(self._capture_on_play_before)
        except Exception:
            pass


def pedestrian_payload(
    positions: dict[str, np.ndarray],
    previous_positions: dict[str, np.ndarray],
    elapsed: float,
) -> list[dict[str, object]]:
    result = []
    for path, position in sorted(positions.items()):
        previous = previous_positions.get(path)
        if previous is None or elapsed <= 1.0e-9:
            velocity = np.zeros(3, dtype=float)
        else:
            velocity = (position - previous) / elapsed
        ros_velocity = stage_to_ros_vector(velocity)
        speed = float(np.linalg.norm(ros_velocity[:2]))
        yaw = math.atan2(ros_velocity[1], ros_velocity[0]) if speed > 1.0e-5 else 0.0
        relative_parts = path.removeprefix("/World/Characters/").split("/")
        # IRA often reuses the same character asset in several groups.  The
        # SkelRoot leaf name is therefore not a stable unique track id.  The
        # authored group instance (for example bookshop_to_coffee_2) is.
        pedestrian_id = (
            relative_parts[1]
            if len(relative_parts) >= 2
            else path.rsplit("/", 1)[-1]
        )
        result.append(
            {
                "id": pedestrian_id,
                "position": stage_to_ros_vector(position).tolist(),
                "velocity": ros_velocity.tolist(),
                "yaw": yaw,
            }
        )
    return result


def set_viewport_camera(stage: Usd.Stage) -> None:
    """Frame the selected environment instead of guessing its world origin."""
    if ARGS.headless:
        return
    try:
        from isaacsim.core.utils.viewports import set_camera_view

        eye = list(SCENE_SPEC["camera_eye"])
        target = list(SCENE_SPEC["camera_target"])
        if SCENE_SPEC.get("auto_frame", SCENE_NAME != "warehouse"):
            default_prim = stage.GetDefaultPrim()
            if default_prim.IsValid():
                bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                )
                bounds = bbox_cache.ComputeWorldBound(default_prim).ComputeAlignedBox()
                minimum = np.asarray(bounds.GetMin(), dtype=float)
                maximum = np.asarray(bounds.GetMax(), dtype=float)
                dimensions = maximum - minimum
                if (
                    np.all(np.isfinite(minimum))
                    and np.all(np.isfinite(maximum))
                    and float(np.max(dimensions)) > 1.0e-3
                ):
                    center = 0.5 * (minimum + maximum)
                    radius = max(1.0, float(np.max(dimensions)))
                    if STAGE_UP_AXIS == "Z":
                        eye = (center + np.asarray([1.2, -1.2, 0.9]) * radius).tolist()
                    else:
                        eye = (center + np.asarray([1.2, 0.9, -1.2]) * radius).tolist()
                    target = center.tolist()
                    print(
                        "[WAREHOUSE-ROBOT] Viewport framed selected scene: "
                        f"center={np.round(center, 3).tolist()}, "
                        f"dimensions={np.round(dimensions, 3).tolist()}",
                        flush=True,
                    )
        set_camera_view(
            eye=eye,
            target=target,
            camera_prim_path="/OmniverseKit_Persp",
        )
    except Exception as exc:
        carb.log_warn(f"Could not set initial viewport camera: {exc}")


def main() -> int:
    timeline = None
    ros = None
    rtx_lidar = None
    robot = None
    collision_proxy = None
    pedestrian_social_motion = None
    exit_reason = "window_closed"
    free_space_guard = None
    steering_free_space_guard = None
    free_space_intrusion_tracker = None
    # Runtime pose recovery is deliberately disabled.  Keep this explicit
    # counter in the result contract so tests can detect any future regression.
    pedestrian_free_space_recovery_count = 0
    try:
        global STAGE_METERS_PER_UNIT, STAGE_UP_AXIS
        print("[WAREHOUSE-ROBOT] Isaac Sim version: 6.0.1", flush=True)
        print(f"[WAREHOUSE-ROBOT] Local asset root: {ASSET_ROOT}", flush=True)
        print(f"[WAREHOUSE-ROBOT] Scene: {SCENE_NAME} ({SCENE_USD})", flush=True)
        print(f"[WAREHOUSE-ROBOT] People enabled: {PEOPLE_ENABLED}", flush=True)
        if PEOPLE_ENABLED:
            print(f"[WAREHOUSE-ROBOT] HumanMotionLibrary USD: {MOTION_LIBRARY_USD}", flush=True)
            print(f"[WAREHOUSE-ROBOT] Local WalkForward USD: {WALK_USD}", flush=True)
        print(f"[WAREHOUSE-ROBOT] Robot source USD: {ROBOT_USD}", flush=True)
        print(f"[WAREHOUSE-ROBOT] Robot visual layer: {ROBOT_VISUAL_USD}", flush=True)
        # ROS stays in a separate system-Humble process.  Do not load rclpy or
        # the ROS bridge into Kit: Isaac 6.0.1 and Ubuntu Humble use different
        # Python runtimes, while localhost UDP is a deterministic ABI boundary.
        if PEOPLE_ENABLED:
            print("[WAREHOUSE-ROBOT] Enabling offline IRA people pipeline...", flush=True)
            enable_extension("isaacsim.replicator.agent.core")
            for _ in range(5):
                simulation_app.update()

            from isaacsim.replicator.agent.core import api as ira

            if SCENE_NAME == "custom":
                import NavSchema

                probe_success, probe_error = wait_for_task(
                    asyncio.ensure_future(
                        omni.usd.get_context().open_stage_async(str(SCENE_USD))
                    ),
                    ARGS.setup_timeout,
                    "probing custom NavMesh stage",
                )
                if not probe_success:
                    raise RuntimeError(f"Could not probe custom stage: {probe_error}")
                for _ in range(5):
                    simulation_app.update()
                probe_stage = omni.usd.get_context().get_stage()
                volume_prim = probe_stage.GetPrimAtPath("/World/NavMeshVolume")
                ground_prim = probe_stage.GetPrimAtPath("/World/Environment/Ground")
                print(
                    "[WAREHOUSE-ROBOT] Custom NavMesh input: "
                    f"volume_valid={volume_prim.IsValid()}, "
                    f"volume_type={volume_prim.GetTypeName()}, "
                    f"volume_schema={volume_prim.IsA(NavSchema.NavMeshVolume)}, "
                    f"ground_valid={ground_prim.IsValid()}, "
                    f"ground_type={ground_prim.GetTypeName()}",
                    flush=True,
                )

            if not ira.load_config_file(str(IRA_CONFIG)):
                raise RuntimeError(f"IRA rejected offline config: {IRA_CONFIG}")
            wait_for_task(
                asyncio.ensure_future(ira.setup_simulation()),
                ARGS.setup_timeout,
                "loading offline IRA warehouse",
            )
        else:
            print(
                "[WAREHOUSE-ROBOT] Opening selected scene directly; IRA people "
                "pipeline is disabled.",
                flush=True,
            )
            success, error = wait_for_task(
                asyncio.ensure_future(
                    omni.usd.get_context().open_stage_async(str(SCENE_USD))
                ),
                ARGS.setup_timeout,
                "loading selected no-people scene",
            )
            if not success:
                raise RuntimeError(f"Could not open warehouse stage: {error}")
            for _ in range(5):
                simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Scene setup completed without a USD stage")
        # Multi-tick RTX sensors are scheduled from the Fabric
        # /ExternalSimulationTime prim maintained by SimulationManager's
        # physics callback.  Merely advancing omni.timeline leaves that clock
        # undefined/stale in a standalone app and can make a correctly-authored
        # 15 Hz lidar behave like the historical 10 Hz default.  This is the
        # initialization sequence used by Isaac Sim 6's own standalone RTX
        # lidar examples.  Preserve the existing 60 Hz physics contract.
        SimulationManager.setup_simulation(dt=PHYSICS_DT, device="cpu")
        # In variable-step GUI playback Kit advances the timeline from wall
        # time, but PhysX limits catch-up work to roughly
        # physics_hz/minFrameRate steps per application update.  The historical
        # implicit 30 Hz value discarded physics time whenever this rendered
        # scene fell to 13--15 app updates/s.  A low explicit floor preserves
        # physical time (at the cost of slowing the app if physics itself cannot
        # keep up) instead of silently shortening displacement.
        timing_settings = carb.settings.get_settings()
        timing_settings.set(
            "/persistent/simulation/minFrameRate",
            MIN_SIMULATION_FRAME_RATE_HZ,
        )
        stage_meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
        STAGE_METERS_PER_UNIT = stage_meters_per_unit
        STAGE_UP_AXIS = str(UsdGeom.GetStageUpAxis(stage)).upper()
        if STAGE_UP_AXIS not in {"Y", "Z"}:
            raise RuntimeError(f"Unsupported scene up axis: {STAGE_UP_AXIS}")
        if SCENE_NAME == "warehouse" and STAGE_UP_AXIS != "Y":
            raise RuntimeError(f"Expected Y-up warehouse stage, got {STAGE_UP_AXIS}")
        if ROBOT_PHYSICS_ENABLED and SCENE_NAME != "custom":
            raise RuntimeError(
                "dynamic robot physics is currently supported only by the "
                "project Z-up/metre custom scene"
            )
        robot_spawn = np.asarray(
            pedestrian_avoidance_test_spawn()
            if ARGS.test_pedestrian_avoidance
            else scene_default_spawn(),
            dtype=float,
        )
        if (
            ARGS.test_pedestrian_avoidance
            and PEDESTRIAN_AVOIDANCE_MODE == "off"
        ):
            raise RuntimeError(
                "--test-pedestrian-avoidance requires "
                "ISAAC_PEDESTRIAN_AVOIDANCE_MODE=native, gentle, or legacy_dodge"
            )
        people = character_roots(stage)
        if PEOPLE_ENABLED:
            if EXPECTED_PEDESTRIAN_COUNT >= 0 and len(people) != EXPECTED_PEDESTRIAN_COUNT:
                raise RuntimeError(
                    "IRA character count does not match the generated Gazebo population: "
                    f"expected={EXPECTED_PEDESTRIAN_COUNT} found={len(people)}"
                )
            if EXPECTED_PEDESTRIAN_COUNT < 0 and len(people) < 3:
                raise RuntimeError(f"Expected at least 3 IRA character roots, found {len(people)}")
            if not stage.GetPrimAtPath("/World/Characters/HumanMotionLibrary").IsValid():
                raise RuntimeError("IRA did not create the local HumanMotionLibrary payload")
        elif people:
            raise RuntimeError(
                "No-people scene unexpectedly contains character roots: "
                + ", ".join(str(person.GetPath()) for person in people)
            )
        for point_name in ("Patrol_Point_A", "Patrol_Point_B", "Patrol_Point_C"):
            point = stage.GetPrimAtPath(f"/World/{point_name}")
            if point.IsValid():
                translation = (
                    UsdGeom.Xformable(point)
                    .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    .ExtractTranslation()
                )
                print(f"[WAREHOUSE-ROBOT] {point_name} stage_xyz={[float(axis) for axis in translation]}", flush=True)
        print(f"[WAREHOUSE-ROBOT] IRA characters ready: {len(people)}", flush=True)
        for person in people:
            print(f"[WAREHOUSE-ROBOT]   character root: {person.GetPath()}", flush=True)

        # The top-level robot USD pulls in its full multi-rigid-body physics
        # layer.  In this centimetre/Y-up IRA stage that articulation holds the
        # whole process near 6 FPS even when hidden, while navigation already
        # uses a deterministic root pose rather than wheel-contact dynamics.
        # Reference the same asset package's complete visual/base layer and
        # drive its root kinematically.  This preserves the user's robot model
        # and dimensions without the incompatible physics composition.
        stage_utils.add_reference_to_stage(str(ROBOT_VISUAL_USD), ROBOT_PRIM)
        deinstanced_visuals = deinstance_robot_visuals(stage)
        print(
            "[WAREHOUSE-ROBOT] Hydra safety: disabled instanceable overrides "
            f"on {deinstanced_visuals} robot visual scopes",
            flush=True,
        )
        for _ in range(12):
            simulation_app.update()
        robot = XFormPrim(ROBOT_PRIM, name="mecanum730_xms5_visual")
        robot.set_world_poses(
            positions=np.asarray([robot_spawn], dtype=float),
            orientations=np.asarray([robot_orientation(0.0)], dtype=float),
        )
        timeline = omni.timeline.get_timeline_interface()
        # IRA normally reapplies these values after opening the warehouse,
        # because the USD's authored timeCodesPerSecond otherwise makes live
        # time advance extremely slowly.  Apply the same contract in both the
        # people and no-people paths so ROS telemetry remains continuous.
        timeline.set_time_codes_per_second(TIMELINE_FPS)
        if timeline.get_end_time() < TIMELINE_DURATION_SEC:
            timeline.set_end_time(TIMELINE_DURATION_SEC)
        timeline.commit_silently()
        # Keep playback stopped until every stage edit, viewport change and
        # RTX render-product/Writer attachment has completed.  Isaac 6.0.1's
        # own multi-sensor tests use this pre-play lifecycle to avoid a
        # Hydra/FIF graph race.
        timeline.stop()
        for _ in range(3):
            simulation_app.update()
        dimensions_m, visual_center_stage = robot_visual_bounds(stage)
        if float(np.max(dimensions_m)) < 0.3 or float(np.max(dimensions_m)) > 5.0:
            raise RuntimeError(
                "Robot metric composition produced an implausible size: "
                f"dimensions_m={dimensions_m.tolist()}"
            )
        vertical_index = 2 if STAGE_UP_AXIS == "Z" else 1
        if dimensions_m[vertical_index] < 0.8:
            raise RuntimeError(
                f"Robot is not upright in the {STAGE_UP_AXIS}-up scene: "
                f"dimensions_m={dimensions_m.tolist()}"
            )
        if str(UsdGeom.GetStageUpAxis(stage)).upper() != STAGE_UP_AXIS:
            raise RuntimeError("Robot initialization changed the scene up axis")
        if not math.isclose(float(UsdGeom.GetStageMetersPerUnit(stage)), stage_meters_per_unit):
            raise RuntimeError("Robot initialization changed the IRA stage units")
        collision_proxy = RobotCollisionProxy(
            stage,
            dimensions_m,
            visual_center_stage,
            robot_spawn,
            dynamic=ROBOT_PHYSICS_ENABLED,
        )
        if ARGS.test_collision_obstacle:
            create_collision_validation_obstacle(
                stage,
                collision_proxy.center(
                    robot_spawn, 0.0
                ),
            )
        for _ in range(2):
            simulation_app.update()
        if ROBOT_PHYSICS_ENABLED:
            print(
                "[WAREHOUSE-ROBOT] Robot control: dynamic PhysX rigid body; "
                f"mass={ROBOT_PHYSICS_MASS_KG:.3f} kg, gravity/contact enabled, "
                "roll/pitch locked, visual model follows physical body",
                flush=True,
            )
        else:
            print(
                "[WAREHOUSE-ROBOT] Robot control: collision-aware kinematic "
                "Mecanum root; full visual/base layer preserved, articulation "
                "physics omitted",
                flush=True,
            )
        print(
            "[WAREHOUSE-ROBOT] A/B controls: "
            f"robot_collision_protection={ROBOT_COLLISION_PROTECTION_ENABLED}, "
            f"pedestrian_social_mode={PEDESTRIAN_SOCIAL_MODE}, "
            f"pedestrian_avoidance_mode={PEDESTRIAN_AVOIDANCE_MODE}, "
            f"pedestrian_robot_dodge={PEDESTRIAN_ROBOT_DODGE_ENABLED}",
            flush=True,
        )
        print(
            f"[WAREHOUSE-ROBOT] Robot composed dimensions (m): {dimensions_m.round(3).tolist()}",
            flush=True,
        )
        print(
            "[WAREHOUSE-ROBOT] Robot collision proxy dimensions (m): "
            f"{collision_proxy.dimensions_m.round(3).tolist()}",
            flush=True,
        )
        print(
            f"[WAREHOUSE-ROBOT] Scene stage preserved: upAxis={STAGE_UP_AXIS}, metersPerUnit={stage_meters_per_unit}",
            flush=True,
        )
        manual_mode = bool(timing_settings.get("/app/runLoops/main/manualModeEnabled"))
        fixed_time = bool(timing_settings.get("/app/player/useFixedTimeStepping"))
        min_frame_rate = int(
            timing_settings.get("/persistent/simulation/minFrameRate")
        )
        multi_tick = bool(timing_settings.get("/rtx/hydra/supportMultiTickRate"))
        per_sensor_tlas = bool(
            timing_settings.get("/rtx/rendering/perSensorTickTlas")
        )
        print(
            "[WAREHOUSE-ROBOT] Robot visual root update: "
            f"{1.0 / ROBOT_POSE_APPLY_PERIOD_SEC:.0f} Hz",
            flush=True,
        )
        print(
            f"[WAREHOUSE-ROBOT] Timing: manualModeEnabled={manual_mode}, "
            f"useFixedTimeStepping={fixed_time}, "
            f"timeline_fps={timeline.get_time_codes_per_second():.1f}, "
            f"minFrameRate={min_frame_rate}, "
            f"multi_tick={multi_tick}, per_sensor_tlas={per_sensor_tlas}, "
            f"physics_dt={SimulationManager.get_physics_dt()}",
            flush=True,
        )

        set_viewport_camera(stage)
        for _ in range(3):
            simulation_app.update()

        if LIDAR_MODE == "rtx" and not ARGS.no_ros:
            rtx_lidar = RtxDualLidar(
                stage_meters_per_unit,
                robot_spawn,
                0.0,
            )

        timeline.play()
        # Give the already-authored render products a few normal playback
        # ticks before requiring sensor data.  Do not create another RTX graph
        # after this point.
        for _ in range(RTX_SENSOR_ATTACH_SYNC_FRAMES):
            simulation_app.update()

        if ROBOT_PHYSICS_ENABLED:
            collision_proxy.initialize_dynamic()
            # Let gravity establish floor contact before publishing the first
            # robot pose.  Keep the visual and separately authored RTX sensors
            # synchronized with the dynamic body throughout settling.
            for _ in range(30):
                collision_proxy.set_dynamic_command((0.0, 0.0, 0.0))
                simulation_app.update()
                settled_position, settled_yaw = collision_proxy.dynamic_root_pose()
                robot.set_world_poses(
                    positions=np.asarray([settled_position], dtype=float),
                    orientations=np.asarray(
                        [robot_orientation(settled_yaw)], dtype=float
                    ),
                )
                if rtx_lidar is not None:
                    rtx_lidar.set_robot_pose(settled_position, settled_yaw)
            print(
                "[WAREHOUSE-ROBOT] Dynamic robot settled on PhysX floor: "
                f"root_xyz={settled_position.round(4).tolist()} "
                f"yaw={settled_yaw:.4f}",
                flush=True,
            )

        if rtx_lidar is not None:
            warmup_deadline = time.monotonic() + ARGS.setup_timeout
            next_warmup_report = time.monotonic() + RTX_WARMUP_PROGRESS_PERIOD_SEC
            while rtx_lidar.poll_payload() is None:
                if time.monotonic() >= warmup_deadline:
                    raise RuntimeError(
                        "native RTX dual lidar produced no complete scan before "
                        f"the {ARGS.setup_timeout:.1f}s setup timeout; diagnostics="
                        + json.dumps(rtx_lidar.warmup_diagnostics, sort_keys=True)
                    )
                simulation_app.update()
                if time.monotonic() >= next_warmup_report:
                    print(
                        "[WAREHOUSE-ROBOT] Native RTX lidar warmup progress: "
                        + json.dumps(rtx_lidar.warmup_diagnostics, sort_keys=True),
                        flush=True,
                    )
                    next_warmup_report += RTX_WARMUP_PROGRESS_PERIOD_SEC
            print(
                "[WAREHOUSE-ROBOT] Native RTX lidar warmup passed; "
                f"both sensors produced ranges and {LIDAR_SAMPLE_COUNT} "
                "aligned intensity slots.",
                flush=True,
            )

        ros = None if ARGS.no_ros else RosCmdVel()
        if ros is not None:
            print(
                "[WAREHOUSE-ROBOT] External ROS 2 UDP bridge ready: "
                f"commands={CMD_VEL_UDP_PORT}, telemetry={TELEMETRY_UDP_PORT}.",
                flush=True,
            )
        else:
            print("[WAREHOUSE-ROBOT] ROS disabled by --no-ros; test-command mode only.", flush=True)
        initial_position = np.asarray(robot.get_world_poses()[0][0], dtype=float)
        navigation_position = initial_position.copy()
        navigation_yaw = yaw_from_quaternion(
            np.asarray(robot.get_world_poses()[1][0], dtype=float)
        )
        # PhysX reports a wrapped quaternion yaw.  Keep a separate continuous
        # heading for total-turn diagnostics; a positive turn beyond pi would
        # otherwise appear as a negative final-minus-initial yaw change.
        navigation_yaw_unwrapped = navigation_yaw
        initial_navigation_yaw_unwrapped = navigation_yaw_unwrapped
        if PEOPLE_ENABLED:
            free_space_guard = custom_free_space_guard()
            if PEDESTRIAN_SOCIAL_MODE == "gazebo_social":
                # Patrols are authored with the stricter route-generation
                # margin (normally 0.55 m).  Steering may use the established
                # runtime intrusion boundary (normally 0.20 m); constraining
                # every local target to the route margin can pin an animated
                # root at tight turns even though it remains safely in free
                # space.
                steering_free_space_guard = free_space_guard
            runtime_deadline = time.monotonic() + 15.0
            while True:
                try:
                    character_positions(stage, require_runtime=True)
                    break
                except RuntimeError:
                    if time.monotonic() >= runtime_deadline:
                        raise
                    simulation_app.update()
            safe_start_anchors = reset_custom_people_to_safe_anchors(stage)
            if safe_start_anchors:
                if free_space_guard is not None:
                    for path, anchor in safe_start_anchors.items():
                        if not free_space_guard.contains_world(anchor[0], anchor[1]):
                            raise RuntimeError(
                                f"Generated SLAM patrol anchor is not free: {path}={anchor}"
                            )
                # Let the reset propagate through Fabric before publishing the
                # first pedestrian state or rendering a frame for the demo.
                for _ in range(3):
                    simulation_app.update()
                print(
                    "[WAREHOUSE-ROBOT] Reset custom IRA people to safe patrol anchors: "
                    + json.dumps(safe_start_anchors, sort_keys=True),
                    flush=True,
                )
            initial_people_positions = character_positions(stage, require_runtime=True)
            if free_space_guard is not None:
                free_space_intrusion_tracker = SustainedIntrusionTracker(
                    sustained_samples=(
                        PEDESTRIAN_FREE_SPACE_SUSTAINED_INTRUSION_SAMPLES
                    )
                )
                free_space_intrusion_tracker.update(
                    {
                        path: bool(
                            free_space_guard.contains_world(position[0], position[1])
                        )
                        for path, position in initial_people_positions.items()
                    }
                )
            print(
                "[WAREHOUSE-ROBOT] Live BehaviorAgent positions ready: "
                + json.dumps(
                    {
                        path: position.round(3).tolist()
                        for path, position in initial_people_positions.items()
                    }
                ),
                flush=True,
            )
            avoidance_configuration = configure_pedestrian_robot_avoidance(
                stage, PEDESTRIAN_AVOIDANCE_MODE, PEDESTRIAN_SOCIAL_MODE
            )
        else:
            initial_people_positions = {}
            avoidance_configuration = {}
        people_lidar = (
            PhysxAnalyticPeopleLidar(
                stage,
                PHYSX_ANALYTIC_LEG_RADIUS_M,
                debug=PHYSX_ANALYTIC_LEGS_DEBUG,
            )
            if PHYSX_ANALYTIC_LEGS_ENABLED
            else None
        )
        if people_lidar is not None:
            initial_leg_snapshot = people_lidar.snapshot(
                float(timeline.get_current_time())
            )
            print(
                "[WAREHOUSE-ROBOT] PhysX + analytic dynamic legs ready: "
                f"people={len(people_lidar.bindings)} "
                f"legs={len(initial_leg_snapshot.labels)} "
                f"radius_m={people_lidar.radius_m:.3f} "
                "joints=LeftFoot/LeftShin/RightFoot/RightShin",
                flush=True,
            )
        print(
            "[WAREHOUSE-ROBOT] Pedestrian/robot avoidance mode: "
            + json.dumps(
                {
                    path: settings
                    for path, settings in avoidance_configuration.items()
                },
                sort_keys=True,
            ),
            flush=True,
        )
        pedestrian_robot_avoidance = (
            PedestrianRobotAvoidance(
                initial_people_positions,
                PEDESTRIAN_AVOIDANCE_MODE,
                PEDESTRIAN_DODGE_PROFILE,
            )
            if PEDESTRIAN_DODGE_PROFILE is not None
            else None
        )
        pedestrian_social_tracker = SocialQualityTracker(
            personal_space_m=PEDESTRIAN_PERSONAL_SPACE_M,
            visual_overlap_m=PEDESTRIAN_VISUAL_OVERLAP_M,
        )
        if PEDESTRIAN_SOCIAL_MODE == "gazebo_social":
            yield_trigger_m = PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M
            yield_resume_m = PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M
            yield_role = "emergency_fallback"
        else:
            yield_trigger_m = PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M
            yield_resume_m = PEDESTRIAN_SOCIAL_YIELD_RESUME_M
            yield_role = "legacy_primary"
        pedestrian_social_yielding = (
            PedestrianSocialYielding(
                initial_people_positions,
                trigger_distance_m=yield_trigger_m,
                resume_distance_m=yield_resume_m,
                role=yield_role,
            )
            if PEOPLE_ENABLED
            else None
        )
        pedestrian_social_motion = (
            BehaviorAgentSocialMotion(
                stage, initial_people_positions, steering_free_space_guard
            )
            if PEOPLE_ENABLED and PEDESTRIAN_SOCIAL_MODE == "gazebo_social"
            else None
        )
        max_people_displacements_m = {path: 0.0 for path in initial_people_positions}
        previous_people_positions = {
            path: position.copy() for path, position in initial_people_positions.items()
        }
        last_people_sim_time = float(timeline.get_current_time())
        last_telemetry_sim_time = -math.inf
        last_lidar_sim_time = -math.inf
        last_analytic_leg_debug_sim_time = -math.inf
        # Native RTX acquisition and ROS publication are separate clocks.
        # Polling the render product only when the equally-rated ROS gate opens
        # aliases any Writer callback that becomes visible one app update late;
        # at 15 Hz that produced a repeatable 66.67/133.33 ms pattern (10 Hz
        # average).  Drain new native pairs after every app update, then let the
        # 30 Hz telemetry path publish each pair once at the requested cadence.
        pending_rtx_scans: deque[dict[str, object]] = deque(
            maxlen=RTX_FRAME_QUEUE_SIZE
        )
        last_people_publish_sim_time = -math.inf
        last_robot_pose_apply_sim_time = -math.inf
        started = time.monotonic()
        started_sim_time = float(timeline.get_current_time())
        started_physics_steps = collision_proxy.physics_control_steps
        last_report = started
        last_report_sim_time = started_sim_time
        last_report_frame = 0
        last_report_physics_steps = started_physics_steps
        last_command: np.ndarray | None = None
        executed_command = (0.0, 0.0, 0.0)
        actual_velocity: tuple[float, float, float] | None = None
        actual_velocity_source = "unavailable"
        pose_derived_velocity: tuple[float, float, float] | None = None
        pose_derived_velocity_source = "pose_derived_velocity_invalid:first_sample"
        test_physx_linear_samples: list[float] = []
        test_physx_angular_samples: list[float] = []
        test_pose_linear_samples: list[float] = []
        test_pose_angular_samples: list[float] = []
        previous_pose_sample: tuple[float, float, float, float] | None = None
        pending_reset_event: dict[str, object] | None = None
        command_watchdog_active = True
        command_age_sec = float("inf")
        collision_blocked_count = 0
        last_collision_path = ""
        last_collision_report = -math.inf
        pedestrian_min_robot_clearance_m = math.inf
        pedestrian_min_robot_clearance_by_person_m = {
            path: math.inf for path in initial_people_positions
        }
        pedestrian_near_robot_frames = 0
        pedestrian_inside_robot_frames = 0
        last_pedestrian_avoidance_sample_sim_time = -math.inf
        last_pedestrian_social_warning_sim_time = -math.inf
        last_free_space_guard_sim_time = -math.inf
        frame = 0
        print(
            "WAREHOUSE_PEOPLE_ROBOT_READY="
            + json.dumps(
                {
                    "isaac_sim": "6.0.1",
                    "robot": str(ROBOT_USD),
                    "robot_visual_layer": str(ROBOT_VISUAL_USD),
                    "robot_spawn": robot_spawn.tolist(),
                    "stage_meters_per_unit": stage_meters_per_unit,
                    "scene": SCENE_NAME,
                    "scene_usd": str(SCENE_USD),
                    "stage_up_axis": STAGE_UP_AXIS,
                    "wheel_joints": list(WHEEL_NAMES),
                    "wheel_velocity_control": False,
                    "wheel_visual_rotation": False,
                    "robot_physics_enabled": ROBOT_PHYSICS_ENABLED,
                    "robot_physics_mode": (
                        "dynamic_rigid_body" if ROBOT_PHYSICS_ENABLED else "kinematic"
                    ),
                    "robot_physics_mass_kg": (
                        ROBOT_PHYSICS_MASS_KG if ROBOT_PHYSICS_ENABLED else None
                    ),
                    "robot_gravity_enabled": ROBOT_PHYSICS_ENABLED,
                    "robot_floor_contact_enabled": ROBOT_PHYSICS_ENABLED,
                    "robot_collision_proxy_static_friction": (
                        0.0 if ROBOT_PHYSICS_ENABLED else None
                    ),
                    "robot_collision_proxy_dynamic_friction": (
                        0.0 if ROBOT_PHYSICS_ENABLED else None
                    ),
                    "robot_collision_proxy_friction_combine_mode": (
                        "min" if ROBOT_PHYSICS_ENABLED else None
                    ),
                    "robot_dimensions_m": dimensions_m.tolist(),
                    "collision_proxy_dimensions_m": collision_proxy.dimensions_m.tolist(),
                    "collision_proxy_prim": ROBOT_COLLISION_PRIM,
                    "collision_aware_motion": ROBOT_COLLISION_PROTECTION_ENABLED,
                    "robot_collision_protection": ROBOT_COLLISION_PROTECTION_ENABLED,
                    "people_enabled": PEOPLE_ENABLED,
                    "pedestrian_social_mode": PEDESTRIAN_SOCIAL_MODE,
                    "pedestrian_social_adapter": (
                        "behavior_agent_persistent_follow_target_2d"
                        if pedestrian_social_motion is not None
                        else (
                            "legacy_discrete_yield" if PEOPLE_ENABLED else "disabled"
                        )
                    ),
                    "pedestrian_avoidance_mode": PEDESTRIAN_AVOIDANCE_MODE,
                    "pedestrian_robot_object_avoidance": (
                        PEDESTRIAN_ROBOT_OBJECT_AVOIDANCE_ENABLED
                    ),
                    "pedestrian_robot_dodge": PEDESTRIAN_ROBOT_DODGE_ENABLED,
                    "pedestrian_robot_dodge_clearance_m": (
                        PEDESTRIAN_DODGE_PROFILE.trigger_clearance_m
                        if PEDESTRIAN_DODGE_PROFILE is not None
                        else None
                    ),
                    "pedestrian_robot_dodge_profile": (
                        asdict(PEDESTRIAN_DODGE_PROFILE)
                        if PEDESTRIAN_DODGE_PROFILE is not None
                        else None
                    ),
                    "pedestrian_person_person_avoidance": PEOPLE_ENABLED,
                    "pedestrian_social_mass_kg": (
                        PEDESTRIAN_SOCIAL_MASS_KG if PEOPLE_ENABLED else None
                    ),
                    "pedestrian_personal_space_m": (
                        PEDESTRIAN_PERSONAL_SPACE_M if PEOPLE_ENABLED else None
                    ),
                    "pedestrian_visual_overlap_m": (
                        PEDESTRIAN_VISUAL_OVERLAP_M if PEOPLE_ENABLED else None
                    ),
                    "pedestrian_social_yield_trigger_m": (
                        yield_trigger_m
                        if PEOPLE_ENABLED
                        else None
                    ),
                    "pedestrian_social_yield_resume_m": (
                        yield_resume_m
                        if PEOPLE_ENABLED
                        else None
                    ),
                    "pedestrian_route_clearance_m": (
                        CUSTOM_FREE_SPACE_CLEARANCE_M
                        if free_space_guard is not None
                        else None
                    ),
                    "pedestrian_intrusion_guard_clearance_m": (
                        CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M
                        if free_space_guard is not None
                        else None
                    ),
                    "robot_avoidance_mass_kg": (
                        ROBOT_PHYSICS_MASS_KG if ROBOT_PHYSICS_ENABLED else 1000.0
                    ),
                    "ros_stage_planar_mapping": (
                        "ros_x=stage_x,ros_y=stage_y"
                        if STAGE_UP_AXIS == "Z"
                        else "ros_x=stage_x,ros_y=-stage_z"
                    ),
                    "navigation_control": (
                        "physx_pre_step_rigid_body_velocity"
                        if ROBOT_PHYSICS_ENABLED
                        else (
                            "collision_stop_only_kinematic_mecanum_visual_proxy"
                            if ROBOT_COLLISION_PROTECTION_ENABLED
                            else "unchecked_kinematic_mecanum_visual_proxy"
                        )
                    ),
                    "robot_pose_apply_rate_hz": 1.0 / ROBOT_POSE_APPLY_PERIOD_SEC,
                    "robot_physics_control_rate_hz": (
                        1.0 / PHYSICS_DT if ROBOT_PHYSICS_ENABLED else None
                    ),
                    "robot_visual_instances_removed": deinstanced_visuals,
                    "manual_timing": manual_mode,
                    "fixed_time_stepping": fixed_time,
                    "min_simulation_frame_rate_hz": min_frame_rate,
                    "app_update_rate_limit_hz": (
                        ARGS.app_update_rate_limit_hz or None
                    ),
                    "lidar_mode": LIDAR_MODE,
                    "lidar_backend": LIDAR_BACKEND,
                    "physx_capture_backend": PHYSX_CAPTURE_BACKEND,
                    "lidar_profile": (
                        RTX_LIDAR_PROFILE if LIDAR_MODE == "rtx" else "physx_raycast"
                    ),
                    "lidar_model": (
                        RTX_LIDAR_PROFILE_SPEC["model"]
                        if LIDAR_MODE == "rtx"
                        else (
                            "PhysX environment raycast + analytic dynamic pedestrian legs"
                            if people_lidar is not None
                            else "PhysX raycast"
                        )
                    ),
                    "physx_analytic_legs_enabled": people_lidar is not None,
                    "physx_analytic_leg_radius_m": (
                        people_lidar.radius_m if people_lidar is not None else None
                    ),
                    "lidar_profile_asset": (
                        str(RTX_LIDAR_USD) if LIDAR_MODE == "rtx" else None
                    ),
                    "lidar_profile_asset_sha256": RTX_LIDAR_ASSET_SHA256,
                    "lidar_intensity": (
                        "native_rtx_gmo_scalar_0_255"
                        if LIDAR_MODE == "rtx"
                        else "unavailable"
                    ),
                    "lidar_samples": LIDAR_SAMPLE_COUNT,
                    "lidar_rate_hz": LIDAR_RATE_HZ,
                    "lidar_rate_basis": "simulation_time",
                    "lidar_timestamp_domain": LIDAR_TIMESTAMP_DOMAIN,
                    "lidar_pairing_timestamp_domain": (
                        LIDAR_PAIRING_TIMESTAMP_DOMAIN
                    ),
                    "producer_source_sha256": SOURCE_SHA256,
                    "launcher_sha256": LAUNCHER_SHA256,
                    "people": len(people),
                    "ros_cmd_vel": ros is not None,
                    "ros_telemetry": ros is not None,
                    "ros_topics": [
                        "/scan",
                        "/scan_01",
                        "/scan_02",
                        "/scan_merged",
                        "/odom",
                        "/tf",
                        "/tf_static",
                        "/clock",
                        "/cmd_vel",
                        "/cmd_vel_stamped",
                        "/drl_vo/actuation_decision",
                        "/isaac/actuation_state",
                        "/isaac/reset_pose",
                        "/isaac/reset_event",
                        "/pedestrian_ground_truth",
                        "/data_collection/episode_event",
                        "/data_collection/sensor_config",
                    ]
                    if ros is not None
                    else [],
                    "arm_lock": "authored_visual_pose",
                    "assets_are_local": True,
                }
            ),
            flush=True,
        )

        while simulation_app.is_running():
            loop_started = time.monotonic()
            if ARGS.test_command is not None:
                command = clamp_twist(*ARGS.test_command)
                command_watchdog_active = False
                command_age_sec = 0.0
            elif ros is not None:
                ros.spin_once()
                command, command_watchdog_active, command_age_sec = ros.command(
                    float(timeline.get_current_time())
                )
            else:
                command = (0.0, 0.0, 0.0)
                command_watchdog_active = True
                command_age_sec = float("inf")
            reset_request = ros.pop_reset_request() if ros is not None else None
            if reset_request is not None:
                sequence_id = int(reset_request["sequence_id"])
                reset_x, reset_y, reset_yaw = reset_request["pose"]
                rejection_reason = None
                if any(abs(value) > 1.0e-6 for value in command):
                    rejection_reason = "command_not_stopped"
                current_ros = stage_to_ros_vector(navigation_position)
                target_position = stage_from_ros_offset(
                    reset_x, reset_y, float(current_ros[2])
                )
                if rejection_reason is None:
                    blocking_path = collision_proxy.blocking_path(
                        target_position, reset_yaw, target_position, reset_yaw
                    )
                    if blocking_path is not None:
                        rejection_reason = f"target_collision:{blocking_path}"
                if rejection_reason is None:
                    if ROBOT_PHYSICS_ENABLED:
                        collision_proxy.reset_dynamic_pose(target_position, reset_yaw)
                    else:
                        collision_proxy.set_pose(target_position, reset_yaw)
                    navigation_position = target_position
                    navigation_yaw = reset_yaw
                    navigation_yaw_unwrapped = reset_yaw
                    executed_command = (0.0, 0.0, 0.0)
                    actual_velocity = None
                    actual_velocity_source = "fixed_tick_pose_difference_invalid:reset"
                    pose_derived_velocity = None
                    pose_derived_velocity_source = (
                        "pose_derived_velocity_invalid:reset_or_teleport"
                    )
                    previous_pose_sample = None
                    robot.set_world_poses(
                        positions=np.asarray([navigation_position], dtype=float),
                        orientations=np.asarray(
                            [robot_orientation(navigation_yaw)], dtype=float
                        ),
                    )
                    if rtx_lidar is not None:
                        rtx_lidar.set_robot_pose(navigation_position, navigation_yaw)
                pending_reset_event = {
                    "schema": "isaac_reset_event/v1",
                    "sequence_id": sequence_id,
                    "simulation_time_sec": float(timeline.get_current_time()),
                    "accepted": rejection_reason is None,
                    "reason": rejection_reason or "reset_applied",
                    "pose": [reset_x, reset_y, reset_yaw],
                }
            previous_sim_time = float(timeline.get_current_time())
            command_array = np.asarray(command, dtype=float)
            command_changed = last_command is None or not np.allclose(
                command_array, last_command, rtol=0.0, atol=1.0e-6
            )
            if command_changed:
                last_command = command_array.copy()
            if ROBOT_PHYSICS_ENABLED:
                collision_proxy.set_dynamic_command(command)
            # This also advances IRA behavior trees and Skel animation when
            # the optional people pipeline is enabled.
            simulation_app.update()
            sim_time = float(timeline.get_current_time())
            if (
                free_space_guard is not None
                and sim_time
                >= last_free_space_guard_sim_time
                + PEDESTRIAN_FREE_SPACE_GUARD_PERIOD_SEC
                - 1.0e-9
            ):
                (
                    intrusion_snapshot,
                    intrusion_sample_positions,
                ) = observe_custom_people_free_space(
                    stage,
                    free_space_guard,
                    free_space_intrusion_tracker,
                )
                if intrusion_snapshot.sustained_intrusions:
                    print(
                        "[WAREHOUSE-ROBOT] WARNING: sustained pedestrian "
                        "free-space intrusion observed (pose unchanged): "
                        + json.dumps(
                            {
                                path: np.asarray(
                                    intrusion_sample_positions[path]
                                ).round(3).tolist()
                                for path in (
                                    intrusion_snapshot.sustained_intrusions
                                )
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                last_free_space_guard_sim_time = advance_periodic_origin(
                    last_free_space_guard_sim_time,
                    sim_time,
                    PEDESTRIAN_FREE_SPACE_GUARD_PERIOD_SEC,
                )
            if rtx_lidar is not None:
                while True:
                    acquired_scans = rtx_lidar.poll_payload()
                    if acquired_scans is None:
                        break
                    if len(pending_rtx_scans) >= RTX_FRAME_QUEUE_SIZE:
                        raise RuntimeError(
                            "RTX-to-telemetry scan queue overflow; refusing to "
                            "silently drop or repeat a native lidar frame"
                        )
                    pending_rtx_scans.append(acquired_scans)
            step_dt = max(0.0, min(0.25, sim_time - previous_sim_time))
            executed_command = command
            if ROBOT_PHYSICS_ENABLED:
                previous_wrapped_yaw = navigation_yaw
                navigation_position, navigation_yaw = collision_proxy.dynamic_root_pose()
                navigation_yaw_unwrapped += math.atan2(
                    math.sin(navigation_yaw - previous_wrapped_yaw),
                    math.cos(navigation_yaw - previous_wrapped_yaw),
                )
                # This is the independent ground-truth measurement: read the
                # resolved PhysX rigid-body state after the simulation step.
                # It must never be populated from ``command``.
                if collision_proxy.body is None:
                    raise RuntimeError("dynamic body unavailable for velocity telemetry")
                actual_velocity = world_to_ros_body_twist(
                    np.asarray(collision_proxy.body.get_linear_velocity(), dtype=float),
                    np.asarray(collision_proxy.body.get_angular_velocity(), dtype=float),
                    navigation_yaw,
                    STAGE_UP_AXIS,
                    STAGE_METERS_PER_UNIT,
                )
                actual_velocity_source = "physx_rigid_body_api"
                if ARGS.test_command is not None:
                    test_physx_linear_samples.append(actual_velocity[0])
                    test_physx_angular_samples.append(actual_velocity[2])
                # The visual hierarchy and RTX sensors are not children of
                # the physical proxy, so synchronize them from the resolved
                # PhysX pose after every step.
                robot.set_world_poses(
                    positions=np.asarray([navigation_position], dtype=float),
                    orientations=np.asarray(
                        [robot_orientation(navigation_yaw)], dtype=float
                    ),
                )
                if rtx_lidar is not None:
                    rtx_lidar.set_robot_pose(
                        navigation_position,
                        navigation_yaw,
                    )
            elif step_dt > 0.0:
                candidate_position, candidate_yaw = integrate_navigation_pose(
                    command,
                    navigation_position,
                    navigation_yaw,
                    step_dt,
                )
                blocking_path = (
                    collision_proxy.blocking_path(
                        navigation_position,
                        navigation_yaw,
                        candidate_position,
                        candidate_yaw,
                    )
                    if ROBOT_COLLISION_PROTECTION_ENABLED
                    else None
                )
                if blocking_path is None:
                    navigation_position = candidate_position
                    navigation_yaw = candidate_yaw
                    navigation_yaw_unwrapped = candidate_yaw
                else:
                    executed_command = (0.0, 0.0, 0.0)
                    collision_blocked_count += 1
                    last_collision_path = blocking_path
                    now = time.monotonic()
                    if now - last_collision_report >= 1.0:
                        print(
                            "[WAREHOUSE-ROBOT] Collision blocked requested "
                            f"motion against {blocking_path}",
                            flush=True,
                        )
                        last_collision_report = now
            pose_sample = (
                sim_time,
                *stage_to_ros_vector(navigation_position)[:2].tolist(),
                navigation_yaw,
            )
            pose_derived_velocity, pose_invalid_reason = fixed_tick_pose_twist(
                previous_pose_sample,
                pose_sample,
                max_dt_sec=2.0,
            )
            previous_pose_sample = pose_sample
            pose_derived_velocity_source = (
                "pose_derived_velocity"
                if pose_derived_velocity is not None
                else f"pose_derived_velocity_invalid:{pose_invalid_reason}"
            )
            if ARGS.test_command is not None and pose_derived_velocity is not None:
                test_pose_linear_samples.append(pose_derived_velocity[0])
                test_pose_angular_samples.append(pose_derived_velocity[2])
            if not ROBOT_PHYSICS_ENABLED:
                actual_velocity = pose_derived_velocity
                invalid_reason = pose_invalid_reason
                actual_velocity_source = (
                    "fixed_tick_pose_difference"
                    if actual_velocity is not None
                    else f"fixed_tick_pose_difference_invalid:{invalid_reason}"
                )
            # Keep mathematical integration per-frame and update the complete
            # visual robot root at 20 Hz.  Command transitions apply
            # immediately so stop/turn input remains crisp.
            robot_pose_due = not ROBOT_PHYSICS_ENABLED and (
                sim_time
                >= last_robot_pose_apply_sim_time
                + ROBOT_POSE_APPLY_PERIOD_SEC
                - 1.0e-9
            )
            if not ROBOT_PHYSICS_ENABLED and (command_changed or robot_pose_due):
                robot.set_world_poses(
                    positions=np.asarray([navigation_position], dtype=float),
                    orientations=np.asarray(
                        [robot_orientation(navigation_yaw)], dtype=float
                    ),
                )
                collision_proxy.set_pose(navigation_position, navigation_yaw)
                if rtx_lidar is not None:
                    rtx_lidar.set_robot_pose(
                        navigation_position,
                        navigation_yaw,
                    )
                if command_changed:
                    last_robot_pose_apply_sim_time = sim_time
                else:
                    last_robot_pose_apply_sim_time = advance_periodic_origin(
                        last_robot_pose_apply_sim_time,
                        sim_time,
                        ROBOT_POSE_APPLY_PERIOD_SEC,
                    )
            if (
                ros is not None
                and sim_time
                >= last_telemetry_sim_time + TELEMETRY_PUBLISH_PERIOD_SEC - 1.0e-9
            ):
                telemetry: dict[str, object] = {
                    "schema": TELEMETRY_SCHEMA,
                    "sim_time": sim_time,
                    "sensor_config": {
                        "schema": "isaac_sensor_config/v1",
                        "lidar_mode": LIDAR_MODE,
                        "lidar_backend": LIDAR_BACKEND,
                        "physx_capture_backend": PHYSX_CAPTURE_BACKEND,
                        "lidar_profile": (
                            RTX_LIDAR_PROFILE
                            if LIDAR_MODE == "rtx"
                            else "physx_raycast"
                        ),
                        "lidar_profile_asset": (
                            str(RTX_LIDAR_USD) if LIDAR_MODE == "rtx" else None
                        ),
                        "lidar_profile_asset_sha256": RTX_LIDAR_ASSET_SHA256,
                        "lidar_rate_hz": LIDAR_RATE_HZ,
                        "lidar_rate_basis": "simulation_time",
                        "lidar_timestamp_domain": LIDAR_TIMESTAMP_DOMAIN,
                        "lidar_pairing_timestamp_domain": (
                            LIDAR_PAIRING_TIMESTAMP_DOMAIN
                        ),
                        "lidar_samples": LIDAR_SAMPLE_COUNT,
                        "telemetry_encoding": "json-or-zlib-fragmented/v1",
                        "lidar_range_min_m": LIDAR_RANGE_MIN_M,
                        "lidar_range_max_m": LIDAR_RANGE_MAX_M,
                        "physx_analytic_legs_enabled": people_lidar is not None,
                        "physx_analytic_leg_radius_m": (
                            people_lidar.radius_m if people_lidar is not None else None
                        ),
                        "manual_timing": manual_mode,
                        "fixed_time_stepping": fixed_time,
                        "min_simulation_frame_rate_hz": min_frame_rate,
                        "physics_rate_hz": 1.0 / PHYSICS_DT,
                        "app_update_rate_limit_hz": (
                            ARGS.app_update_rate_limit_hz or None
                        ),
                        "producer_source_sha256": SOURCE_SHA256,
                        "launcher_sha256": LAUNCHER_SHA256,
                        "odom_twist_source": "simulator_actuation.actual_velocity",
                        "odom_twist_semantics": (
                            "physx_reported_velocity_not_pose_derived_truth"
                            if ROBOT_PHYSICS_ENABLED
                            else "fixed_tick_pose_difference"
                        ),
                        "evaluation_primary_velocity_source": (
                            "pose_derived_velocity_from_world_pose_and_sim_time"
                        ),
                        "actual_velocity_source": (
                            "physx_rigid_body_api"
                            if ROBOT_PHYSICS_ENABLED
                            else "fixed_tick_pose_difference"
                        ),
                        "actuation_state_topic": "/isaac/actuation_state",
                        "actuation_state_schema": "navigation_evaluation_msgs/SimulatorActuationState",
                        "pedestrian_count": len(people),
                        "pedestrian_seed": PEDESTRIAN_SEED,
                        "pedestrian_base_speed_mps": PEDESTRIAN_BASE_SPEED_MPS,
                        "pedestrian_social_mode": PEDESTRIAN_SOCIAL_MODE,
                    },
                    "robot_pose": [
                        *stage_to_ros_vector(navigation_position)[:2].tolist(),
                        navigation_yaw,
                    ],
                    "command": list(executed_command),
                    "actuation": {
                        **(ros.actuation_metadata() if ros is not None else {
                            "command_received": False,
                            "command_sequence_id": 0,
                            "bridge_receive_sim_time": None,
                            "received_command": [0.0, 0.0, 0.0],
                        }),
                        "applied_command": list(executed_command),
                        "actual_velocity": list(actual_velocity) if actual_velocity is not None else None,
                        "actual_velocity_source": actual_velocity_source,
                        "physx_reported_velocity": (
                            list(actual_velocity)
                            if ROBOT_PHYSICS_ENABLED and actual_velocity is not None
                            else None
                        ),
                        "physx_reported_velocity_source": (
                            actual_velocity_source if ROBOT_PHYSICS_ENABLED else None
                        ),
                        "pose_derived_velocity": (
                            list(pose_derived_velocity)
                            if pose_derived_velocity is not None
                            else None
                        ),
                        "pose_derived_velocity_source": pose_derived_velocity_source,
                        "command_age_sec": command_age_sec if math.isfinite(command_age_sec) else None,
                        "watchdog_active": command_watchdog_active,
                        "collision_protection_active": bool(
                            ROBOT_COLLISION_PROTECTION_ENABLED and last_collision_path
                        ),
                        "control_reasons": (["watchdog"] if command_watchdog_active else [])
                        + (["collision_protection"] if executed_command != command else []),
                    },
                }
                if pending_reset_event is not None:
                    telemetry["reset_event"] = pending_reset_event
                    pending_reset_event = None
                if (
                    sim_time
                    >= last_people_publish_sim_time
                    + PEDESTRIAN_PUBLISH_PERIOD_SEC
                    - 1.0e-9
                ):
                    current_people = character_positions(stage) if PEOPLE_ENABLED else {}
                    elapsed = max(0.0, sim_time - last_people_sim_time)
                    telemetry["pedestrians"] = pedestrian_payload(
                        current_people, previous_people_positions, elapsed
                    )
                    previous_people_positions = {
                        path: position.copy()
                        for path, position in current_people.items()
                    }
                    last_people_sim_time = sim_time
                    last_people_publish_sim_time = advance_periodic_origin(
                        last_people_publish_sim_time,
                        sim_time,
                        PEDESTRIAN_PUBLISH_PERIOD_SEC,
                    )
                if (
                    sim_time
                    >= last_lidar_sim_time + LIDAR_PUBLISH_PERIOD_SEC - 1.0e-9
                ):
                    if rtx_lidar is not None:
                        scans = (
                            pending_rtx_scans.popleft()
                            if pending_rtx_scans
                            else None
                        )
                    else:
                        scans = make_dual_scan_payload(
                            navigation_position,
                            navigation_yaw,
                            people_lidar,
                            sim_time,
                        )
                    if scans is not None:
                        telemetry["scans"] = scans
                        if people_lidar is not None and people_lidar.debug:
                            telemetry["analytic_people_lidar"] = (
                                people_lidar.latest_diagnostics
                            )
                            if (
                                sim_time
                                >= last_analytic_leg_debug_sim_time + 0.5 - 1.0e-9
                            ):
                                print(
                                    "ANALYTIC_PEOPLE_LIDAR_DEBUG="
                                    + json.dumps(
                                        people_lidar.latest_diagnostics,
                                        sort_keys=True,
                                    ),
                                    flush=True,
                                )
                                last_analytic_leg_debug_sim_time = sim_time
                        last_lidar_sim_time = advance_periodic_origin(
                            last_lidar_sim_time,
                            sim_time,
                            LIDAR_PUBLISH_PERIOD_SEC,
                        )
                ros.send_telemetry(telemetry)
                last_telemetry_sim_time = advance_periodic_origin(
                    last_telemetry_sim_time,
                    sim_time,
                    TELEMETRY_PUBLISH_PERIOD_SEC,
                )
            if PEOPLE_ENABLED and frame % 120 == 0:
                current_people_positions = character_positions(stage)
                for path, current in current_people_positions.items():
                    initial = initial_people_positions.get(path)
                    if initial is None:
                        continue
                    distance_m = float(
                        np.linalg.norm(stage_to_ros_vector(current - initial)[:2])
                    )
                    max_people_displacements_m[path] = max(
                        max_people_displacements_m.get(path, 0.0), distance_m
                    )
            if (
                PEOPLE_ENABLED
                and sim_time
                >= last_pedestrian_avoidance_sample_sim_time
                + PEDESTRIAN_PUBLISH_PERIOD_SEC
                - 1.0e-9
            ):
                sampled_people_positions = character_positions(stage)
                if pedestrian_robot_avoidance is not None:
                    pedestrian_robot_avoidance.update(
                        sampled_people_positions,
                        navigation_position,
                        navigation_yaw,
                        collision_proxy.dimensions_m,
                        sim_time,
                    )
                if pedestrian_social_yielding is not None:
                    inhibited_social_yield_paths = (
                        set(pedestrian_robot_avoidance.active_dodge_task_ids)
                        | set(pedestrian_robot_avoidance.patrol_resume_times)
                        if pedestrian_robot_avoidance is not None
                        else set()
                    )
                    pedestrian_social_yielding.update(
                        sampled_people_positions,
                        inhibited_social_yield_paths,
                    )
                if pedestrian_social_motion is not None:
                    inhibited_social_motion_paths = set(
                        pedestrian_social_yielding.yielded_restore_speeds
                    )
                    if pedestrian_robot_avoidance is not None:
                        inhibited_social_motion_paths.update(
                            pedestrian_robot_avoidance.active_dodge_task_ids
                        )
                        inhibited_social_motion_paths.update(
                            pedestrian_robot_avoidance.patrol_resume_times
                        )
                    if actual_velocity is None:
                        robot_world_velocity_mps = (0.0, 0.0)
                    else:
                        cosine, sine = math.cos(navigation_yaw), math.sin(
                            navigation_yaw
                        )
                        robot_world_velocity_mps = (
                            cosine * actual_velocity[0]
                            - sine * actual_velocity[1],
                            sine * actual_velocity[0]
                            + cosine * actual_velocity[1],
                        )
                    pedestrian_social_motion.update(
                        sampled_people_positions,
                        collision_proxy.center(
                            navigation_position, navigation_yaw
                        ),
                        navigation_yaw,
                        collision_proxy.dimensions_m,
                        robot_world_velocity_mps,
                        sim_time,
                        inhibited_social_motion_paths,
                    )
                social_snapshot = pedestrian_social_tracker.update(
                    {
                        path: stage_to_ros_vector(position)[:2]
                        for path, position in sampled_people_positions.items()
                    }
                )
                if (
                    social_snapshot.has_visual_overlap
                    and sim_time >= last_pedestrian_social_warning_sim_time + 1.0
                ):
                    print(
                        "[WAREHOUSE-ROBOT] WARNING: pedestrian visual overlap "
                        f"pairs={social_snapshot.visual_overlap_pairs} "
                        f"minimum_m={social_snapshot.min_center_distance_m:.3f} "
                        f"closest_pair={social_snapshot.closest_pair}",
                        flush=True,
                    )
                    last_pedestrian_social_warning_sim_time = sim_time
                # Always sample person/robot clearance.  Previously normal
                # custom demos emitted no evidence that object avoidance had
                # actually maintained separation.
                frame_clearances = {
                    path: signed_planar_box_clearance_m(
                        current,
                        navigation_position,
                        navigation_yaw,
                        collision_proxy.dimensions_m,
                    )
                    for path, current in sampled_people_positions.items()
                }
                if frame_clearances:
                    for path, clearance in frame_clearances.items():
                        pedestrian_min_robot_clearance_by_person_m[path] = min(
                            pedestrian_min_robot_clearance_by_person_m.get(
                                path, math.inf
                            ),
                            clearance,
                        )
                    frame_minimum = min(frame_clearances.values())
                    pedestrian_min_robot_clearance_m = min(
                        pedestrian_min_robot_clearance_m, frame_minimum
                    )
                    if frame_minimum < 2.0:
                        pedestrian_near_robot_frames += 1
                    if frame_minimum < 0.0:
                        pedestrian_inside_robot_frames += 1
                last_pedestrian_avoidance_sample_sim_time = advance_periodic_origin(
                    last_pedestrian_avoidance_sample_sim_time,
                    sim_time,
                    PEDESTRIAN_PUBLISH_PERIOD_SEC,
                )
            if time.monotonic() - last_report >= 30.0:
                now = time.monotonic()
                positions, orientations = robot.get_world_poses()
                position = positions[0]
                orientation = orientations[0]
                moving_people = sum(value > 0.05 for value in max_people_displacements_m.values())
                wall_delta = max(1.0e-9, now - last_report)
                timeline_delta = max(0.0, sim_time - last_report_sim_time)
                realtime_factor = timeline_delta / wall_delta
                app_update_delta = frame - last_report_frame
                physics_step_delta = (
                    collision_proxy.physics_control_steps - last_report_physics_steps
                    if ROBOT_PHYSICS_ENABLED
                    else 0
                )
                physics_timeline_ratio = (
                    physics_step_delta / (timeline_delta / PHYSICS_DT)
                    if ROBOT_PHYSICS_ENABLED and timeline_delta > 0.0
                    else None
                )
                physics_ratio_text = (
                    f"{physics_timeline_ratio:.4f}"
                    if physics_timeline_ratio is not None
                    else "unavailable"
                )
                lidar_rate = (
                    rtx_lidar.measured_pair_rate_hz
                    if rtx_lidar is not None
                    else LIDAR_RATE_HZ
                )
                lidar_wall_rate = (
                    rtx_lidar.measured_pair_wall_rate_hz
                    if rtx_lidar is not None
                    else None
                )
                lidar_rate_text = (
                    f"{lidar_rate:.2f}" if lidar_rate is not None else "warming"
                )
                lidar_wall_rate_text = (
                    f"{lidar_wall_rate:.2f}"
                    if lidar_wall_rate is not None
                    else "warming"
                )
                print(
                    f"[WAREHOUSE-ROBOT] t={sim_time:.1f}s robot_xyz={np.asarray(position).round(3).tolist()} "
                    f"yaw={yaw_from_quaternion(np.asarray(orientation)):.3f} "
                    f"rtf={realtime_factor:.2f} "
                    f"timeline_delta_sec={timeline_delta:.3f} "
                    f"wall_delta_sec={wall_delta:.3f} "
                    f"app_updates={app_update_delta} "
                    f"physics_steps={physics_step_delta} "
                    f"physics_timeline_ratio={physics_ratio_text} "
                    f"lidar_sim_hz={lidar_rate_text}/{LIDAR_RATE_HZ} "
                    f"lidar_wall_hz={lidar_wall_rate_text} "
                    f"people_moving={moving_people}/{len(people)}",
                    flush=True,
                )
                if people_lidar is not None:
                    print(
                        "[WAREHOUSE-ROBOT] Analytic leg LiDAR: "
                        + json.dumps(people_lidar.summary(), sort_keys=True),
                        flush=True,
                    )
                last_report = now
                last_report_sim_time = sim_time
                last_report_frame = frame
                last_report_physics_steps = collision_proxy.physics_control_steps
            frame += 1
            if (
                ARGS.duration > 0.0
                and sim_time - started_sim_time >= ARGS.duration
            ):
                exit_reason = "duration_reached"
                break
            target_app_period = (
                1.0 / ARGS.app_update_rate_limit_hz
                if ARGS.app_update_rate_limit_hz > 0.0
                else PHYSICS_DT if ARGS.headless and not ARGS.fast else 0.0
            )
            if target_app_period > 0.0:
                remaining = target_app_period - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)

        final_positions, final_orientations = robot.get_world_poses()
        final_position = np.asarray(final_positions[0], dtype=float)
        final_yaw = yaw_from_quaternion(np.asarray(final_orientations[0], dtype=float))
        planar_indices = [0, 1] if STAGE_UP_AXIS == "Z" else [0, 2]
        displacement = float(
            np.linalg.norm((final_position - initial_position)[planar_indices])
        )
        wall_elapsed = max(1.0e-9, time.monotonic() - started)
        timeline_elapsed = max(
            0.0, float(timeline.get_current_time()) - started_sim_time
        )
        main_physics_steps = (
            collision_proxy.physics_control_steps - started_physics_steps
            if ROBOT_PHYSICS_ENABLED
            else 0
        )
        physics_timeline_ratio = (
            main_physics_steps / (timeline_elapsed / PHYSICS_DT)
            if ROBOT_PHYSICS_ENABLED and timeline_elapsed > 0.0
            else None
        )
        physics_ratio_text = (
            f"{physics_timeline_ratio:.6f}"
            if physics_timeline_ratio is not None
            else "unavailable"
        )
        print(
            "[WAREHOUSE-ROBOT] Performance: "
            f"frames={frame} wall_sec={wall_elapsed:.3f} "
            f"fps={frame / wall_elapsed:.3f} "
            f"timeline_sec={timeline_elapsed:.3f} "
            f"physics_steps={main_physics_steps} "
            f"physics_timeline_ratio={physics_ratio_text}",
            flush=True,
        )
        test_velocity_tracking = None
        if ARGS.test_command is not None and test_pose_linear_samples:
            command_linear = float(clamp_twist(*ARGS.test_command)[0])
            actual_linear = np.asarray(test_pose_linear_samples, dtype=float)
            errors = actual_linear - command_linear
            absolute_errors = np.abs(errors)
            checks = {
                "absolute_bias": abs(float(np.mean(errors))) <= 0.05,
                "mae": float(np.mean(absolute_errors)) <= 0.08,
                "p95_absolute_error": float(np.percentile(absolute_errors, 95)) <= 0.15,
            }
            command_angular = float(clamp_twist(*ARGS.test_command)[2])
            angular_tracking = None
            if abs(command_angular) > 1.0e-9 and test_pose_angular_samples:
                pose_angular = np.asarray(test_pose_angular_samples, dtype=float)
                angular_errors = pose_angular - command_angular
                angular_absolute_errors = np.abs(angular_errors)
                angular_checks = {
                    "absolute_bias": abs(float(np.mean(angular_errors))) <= 0.05,
                    "mae": float(np.mean(angular_absolute_errors)) <= 0.08,
                    "p95_absolute_error": (
                        float(np.percentile(angular_absolute_errors, 95)) <= 0.15
                    ),
                }
                angular_tracking = {
                    "sample_count": int(pose_angular.size),
                    "command_angular_z_radps": command_angular,
                    "mean_pose_derived_angular_z_radps": float(np.mean(pose_angular)),
                    "bias_radps": float(np.mean(angular_errors)),
                    "mae_radps": float(np.mean(angular_absolute_errors)),
                    "p95_absolute_error_radps": float(
                        np.percentile(angular_absolute_errors, 95)
                    ),
                    "checks": angular_checks,
                    "passed": all(angular_checks.values()),
                }
            test_velocity_tracking = {
                "source": "pose_derived_velocity",
                "sample_count": int(actual_linear.size),
                "command_linear_x_mps": command_linear,
                "mean_actual_linear_x_mps": float(np.mean(actual_linear)),
                "bias_mps": float(np.mean(errors)),
                "mae_mps": float(np.mean(absolute_errors)),
                "p95_absolute_error_mps": float(np.percentile(absolute_errors, 95)),
                "mean_actual_command_ratio": (
                    float(np.mean(actual_linear) / command_linear)
                    if abs(command_linear) > 1.0e-9
                    else None
                ),
                "criteria": {
                    "absolute_bias_max_mps": 0.05,
                    "mae_max_mps": 0.08,
                    "p95_absolute_error_max_mps": 0.15,
                },
                "checks": checks,
                "angular_tracking": angular_tracking,
                "passed": all(checks.values()) and (
                    angular_tracking is None or angular_tracking["passed"]
                ),
                "physx_reported_velocity_diagnostic": {
                    "source": "physx_rigid_body_api",
                    "sample_count": len(test_physx_linear_samples),
                    "mean_linear_x_mps": (
                        float(np.mean(test_physx_linear_samples))
                        if test_physx_linear_samples
                        else None
                    ),
                    "mean_reported_command_ratio": (
                        float(np.mean(test_physx_linear_samples) / command_linear)
                        if test_physx_linear_samples and abs(command_linear) > 1.0e-9
                        else None
                    ),
                    "mean_angular_z_radps": (
                        float(np.mean(test_physx_angular_samples))
                        if test_physx_angular_samples
                        else None
                    ),
                },
            }
        result = {
            "status": "PASS",
            "exit_reason": exit_reason,
            "sim_time": float(timeline.get_current_time()),
            "frames": frame,
            "wall_elapsed_sec": wall_elapsed,
            "average_fps": frame / wall_elapsed,
            "timeline_elapsed_sec": timeline_elapsed,
            "physics_steps_main_loop": main_physics_steps,
            "physics_steps_per_expected_timeline_step": physics_timeline_ratio,
            "min_simulation_frame_rate_hz": min_frame_rate,
            "manual_timing": manual_mode,
            "fixed_time_stepping": fixed_time,
            "app_update_rate_limit_hz": ARGS.app_update_rate_limit_hz or None,
            "robot_final_position": final_position.tolist(),
            "robot_final_yaw_ros_rad": final_yaw,
            "robot_final_yaw_unwrapped_ros_rad": navigation_yaw_unwrapped,
            "robot_yaw_change_ros_rad": (
                navigation_yaw_unwrapped - initial_navigation_yaw_unwrapped
            ),
            "robot_planar_displacement_stage_units": displacement,
            "robot_planar_displacement_m": displacement * stage_meters_per_unit,
            "collision_proxy_dimensions_m": collision_proxy.dimensions_m.tolist(),
            "robot_physics_enabled": ROBOT_PHYSICS_ENABLED,
            "robot_physics_mode": (
                "dynamic_rigid_body" if ROBOT_PHYSICS_ENABLED else "kinematic"
            ),
            "robot_physics_mass_kg": (
                ROBOT_PHYSICS_MASS_KG if ROBOT_PHYSICS_ENABLED else None
            ),
            "robot_gravity_enabled": ROBOT_PHYSICS_ENABLED,
            "robot_floor_contact_enabled": ROBOT_PHYSICS_ENABLED,
            "robot_collision_proxy_static_friction": (
                0.0 if ROBOT_PHYSICS_ENABLED else None
            ),
            "robot_collision_proxy_dynamic_friction": (
                0.0 if ROBOT_PHYSICS_ENABLED else None
            ),
            "robot_collision_proxy_friction_combine_mode": (
                "min" if ROBOT_PHYSICS_ENABLED else None
            ),
            "robot_physics_control_rate_hz": (
                1.0 / PHYSICS_DT if ROBOT_PHYSICS_ENABLED else None
            ),
            "robot_physics_control_steps": (
                collision_proxy.physics_control_steps
                if ROBOT_PHYSICS_ENABLED
                else 0
            ),
            "test_command_velocity_tracking": test_velocity_tracking,
            "collision_blocked_count": collision_blocked_count,
            "last_collision_path": last_collision_path,
            "robot_collision_protection": ROBOT_COLLISION_PROTECTION_ENABLED,
            "people_enabled": PEOPLE_ENABLED,
            "pedestrian_free_space_map_yaml": (
                CUSTOM_FREE_SPACE_MAP_YAML if free_space_guard is not None else None
            ),
            "pedestrian_free_space_clearance_m": (
                CUSTOM_FREE_SPACE_CLEARANCE_M if free_space_guard is not None else None
            ),
            "pedestrian_route_clearance_m": (
                CUSTOM_FREE_SPACE_CLEARANCE_M if free_space_guard is not None else None
            ),
            "pedestrian_intrusion_guard_clearance_m": (
                CUSTOM_FREE_SPACE_GUARD_CLEARANCE_M
                if free_space_guard is not None
                else None
            ),
            "pedestrian_free_space_recovery_count": pedestrian_free_space_recovery_count,
            "pedestrian_free_space_reset_count": pedestrian_free_space_recovery_count,
            "pedestrian_runtime_reset_count": pedestrian_free_space_recovery_count,
            "pedestrian_free_space_guard_rate_hz": (
                1.0 / PEDESTRIAN_FREE_SPACE_GUARD_PERIOD_SEC
                if free_space_guard is not None
                else None
            ),
            "pedestrian_free_space_sustained_intrusion_samples": (
                PEDESTRIAN_FREE_SPACE_SUSTAINED_INTRUSION_SAMPLES
                if free_space_guard is not None
                else None
            ),
            "pedestrian_free_space_intrusions": (
                free_space_intrusion_tracker.summary()
                if free_space_intrusion_tracker is not None
                else None
            ),
            "pedestrian_social_mode": PEDESTRIAN_SOCIAL_MODE,
            "pedestrian_social_motion": (
                pedestrian_social_motion.summary()
                if pedestrian_social_motion is not None
                else {
                    "mode": PEDESTRIAN_SOCIAL_MODE,
                    "adapter": (
                        "legacy_discrete_yield" if PEOPLE_ENABLED else "disabled"
                    ),
                    "patrol_task_replacement": False,
                }
            ),
            "pedestrian_avoidance_mode": PEDESTRIAN_AVOIDANCE_MODE,
            "pedestrian_robot_object_avoidance": (
                PEDESTRIAN_ROBOT_OBJECT_AVOIDANCE_ENABLED
            ),
            "pedestrian_robot_dodge": PEDESTRIAN_ROBOT_DODGE_ENABLED,
            "pedestrian_robot_dodge_profile": (
                asdict(PEDESTRIAN_DODGE_PROFILE)
                if PEDESTRIAN_DODGE_PROFILE is not None
                else None
            ),
            "pedestrian_avoidance_configuration": avoidance_configuration,
            "pedestrian_min_robot_clearance_m": (
                pedestrian_min_robot_clearance_m
                if math.isfinite(pedestrian_min_robot_clearance_m)
                else None
            ),
            "pedestrian_min_robot_clearance_by_person_m": {
                path: clearance if math.isfinite(clearance) else None
                for path, clearance in (
                    pedestrian_min_robot_clearance_by_person_m.items()
                )
            },
            "pedestrian_near_robot_frames": pedestrian_near_robot_frames,
            "pedestrian_inside_robot_frames": pedestrian_inside_robot_frames,
            "pedestrian_social_quality": pedestrian_social_tracker.summary(),
            "pedestrian_social_yielding": (
                pedestrian_social_yielding.summary()
                if pedestrian_social_yielding is not None
                else None
            ),
            "pedestrian_robot_dodge_count": (
                pedestrian_robot_avoidance.dodge_count
                if pedestrian_robot_avoidance is not None
                else 0
            ),
            "pedestrian_robot_dodge_count_by_person": (
                pedestrian_robot_avoidance.dodge_count_by_person
                if pedestrian_robot_avoidance is not None
                else {path: 0 for path in initial_people_positions}
            ),
            "arm_dynamics_enabled": False,
            "arm_pose_mode": (
                "authored_visual_pose_on_dynamic_base"
                if ROBOT_PHYSICS_ENABLED
                else "authored_visual_pose"
            ),
            "people": len(people),
            "people_moving": sum(value > 0.05 for value in max_people_displacements_m.values()),
            "people_max_displacement_m": max_people_displacements_m,
            "ros_cmd_vel_messages_received": ros.received_count if ros is not None else 0,
            "lidar_mode": LIDAR_MODE,
            "lidar_backend": LIDAR_BACKEND,
            "physx_capture_backend": PHYSX_CAPTURE_BACKEND,
            "lidar_profile": (
                RTX_LIDAR_PROFILE if LIDAR_MODE == "rtx" else "physx_raycast"
            ),
            "lidar_profile_asset": (
                str(RTX_LIDAR_USD) if LIDAR_MODE == "rtx" else None
            ),
            "lidar_profile_asset_sha256": RTX_LIDAR_ASSET_SHA256,
            "lidar_intensity": LIDAR_MODE == "rtx",
            "physx_analytic_people_lidar": (
                people_lidar.summary() if people_lidar is not None else None
            ),
            "lidar_requested_rate_hz": LIDAR_RATE_HZ,
            "producer_source_sha256": SOURCE_SHA256,
            "launcher_sha256": LAUNCHER_SHA256,
            "lidar_measured_pair_rate_hz": (
                rtx_lidar.measured_pair_rate_hz
                if rtx_lidar is not None
                else LIDAR_RATE_HZ
            ),
            "lidar_measured_pair_wall_rate_hz": (
                rtx_lidar.measured_pair_wall_rate_hz
                if rtx_lidar is not None
                else None
            ),
            "rtx_lidar_scan_pairs": (
                rtx_lidar.published_pairs if rtx_lidar is not None else 0
            ),
            "rtx_lidar_latest_stats": (
                rtx_lidar.latest_stats if rtx_lidar is not None else {}
            ),
            "rtx_lidar_pairing_diagnostics": (
                rtx_lidar.pairing_diagnostics if rtx_lidar is not None else {}
            ),
        }
        # Preserve the measured evidence even when a strict assertion below
        # rejects the run; the final RESULT marker remains PASS-only.
        print(
            "WAREHOUSE_PEOPLE_ROBOT_METRICS=" + json.dumps(result),
            flush=True,
        )
        if ARGS.test_command is not None and not test_pose_linear_samples:
            raise RuntimeError(
                "Robot test command produced no valid pose-derived velocity samples"
            )
        if (
            ARGS.test_command is not None
            and math.hypot(ARGS.test_command[0], ARGS.test_command[1]) > 0.0
            and displacement * stage_meters_per_unit < 0.05
        ):
            if not ARGS.test_collision_obstacle:
                raise RuntimeError(
                    "Robot test command did not move the base enough: "
                    f"{displacement * stage_meters_per_unit:.3f} m"
                )
        if (
            test_velocity_tracking is not None
            and abs(test_velocity_tracking["command_linear_x_mps"]) > 0.05
            and not ARGS.test_collision_obstacle
            and not test_velocity_tracking["passed"]
        ):
            raise RuntimeError(
                "Robot test command failed pose-derived velocity tracking criteria: "
                + json.dumps(test_velocity_tracking, sort_keys=True)
            )
        if ARGS.test_command is not None and abs(ARGS.test_command[2]) > 1.0e-6:
            yaw_change = (
                navigation_yaw_unwrapped - initial_navigation_yaw_unwrapped
            )
            if yaw_change * ARGS.test_command[2] <= 0.0:
                raise RuntimeError(
                    "Robot angular command and ROS yaw changed in opposite directions: "
                    f"wz={ARGS.test_command[2]:.3f}, yaw_change={yaw_change:.3f}"
                )
        if ARGS.test_collision_obstacle:
            if collision_blocked_count < 1:
                raise RuntimeError("Collision validation obstacle did not block the robot")
            if displacement * stage_meters_per_unit > 0.65:
                raise RuntimeError(
                    "Robot penetrated too far into the collision validation obstacle: "
                    f"displacement={displacement * stage_meters_per_unit:.3f} m"
                )
        if ARGS.test_pedestrian_avoidance:
            if pedestrian_near_robot_frames < 1:
                raise RuntimeError(
                    "Pedestrian avoidance validation had no robot/pedestrian encounter"
                )
            if SCENE_NAME == "custom":
                if (
                    PEDESTRIAN_SOCIAL_MODE == "gazebo_social"
                    and result["pedestrian_robot_dodge_count"] != 0
                ):
                    raise RuntimeError(
                        "Gazebo-social pedestrian avoidance unexpectedly used "
                        "an emergency dodge: "
                        f"count={result['pedestrian_robot_dodge_count']}"
                    )
                if (
                    PEDESTRIAN_SOCIAL_MODE == "legacy"
                    and result["pedestrian_robot_dodge_count"] < 1
                ):
                    raise RuntimeError(
                        "Legacy custom pedestrian avoidance validation produced "
                        "no dodge"
                    )
                if pedestrian_inside_robot_frames != 0:
                    raise RuntimeError(
                        "Custom pedestrian avoidance validation detected robot "
                        f"overlap: inside_frames={pedestrian_inside_robot_frames}"
                    )
            if (
                pedestrian_min_robot_clearance_m
                < PEDESTRIAN_MIN_VISUAL_CLEARANCE_M
            ):
                raise RuntimeError(
                    "Pedestrian entered the robot visual safety clearance: "
                    f"minimum={pedestrian_min_robot_clearance_m:.3f} m, "
                    f"required={PEDESTRIAN_MIN_VISUAL_CLEARANCE_M:.3f} m, "
                    "by_person="
                    + json.dumps(
                        {
                            path.rsplit("/", 1)[-1]: round(clearance, 3)
                            for path, clearance in (
                                pedestrian_min_robot_clearance_by_person_m.items()
                            )
                        },
                        sort_keys=True,
                    )
                    )
        if ARGS.test_pedestrian_social:
            social_quality = result["pedestrian_social_quality"]
            social_motion = result["pedestrian_social_motion"]
            if PEDESTRIAN_SOCIAL_MODE == "gazebo_social":
                if not social_motion["lateral_vector_applied_directly"]:
                    raise RuntimeError(
                        "Gazebo-social adapter did not apply the complete 2D vector"
                    )
                if social_motion["target_write_count"] <= result["people"]:
                    raise RuntimeError(
                        "Gazebo-social adapter did not continuously update steering "
                        "targets: "
                        f"writes={social_motion['target_write_count']}, "
                        f"people={result['people']}"
                    )
                if result["pedestrian_robot_dodge_count"] != 0:
                    raise RuntimeError(
                        "Gazebo-social normal validation used emergency dodge: "
                        f"count={result['pedestrian_robot_dodge_count']}"
                    )
            if social_quality["sample_frames"] < 1:
                raise RuntimeError(
                    "Pedestrian social validation sampled no crowd frames"
                )
            free_space_intrusions = result["pedestrian_free_space_intrusions"]
            if (
                free_space_intrusions is not None
                and free_space_intrusions["sustained_intrusion_count"] != 0
            ):
                raise RuntimeError(
                    "Pedestrian social validation found sustained free-space "
                    "intrusions: "
                    + json.dumps(free_space_intrusions, sort_keys=True)
                )
            if social_quality["visual_overlap_pair_samples"] != 0:
                raise RuntimeError(
                    "Pedestrian social validation found visual overlap: "
                    + json.dumps(social_quality, sort_keys=True)
                )
            if (
                social_quality["personal_space_violation_pair_ratio"]
                > PEDESTRIAN_MAX_PERSONAL_SPACE_VIOLATION_RATIO
            ):
                raise RuntimeError(
                    "Pedestrian personal-space violation ratio is too high: "
                    f"actual={social_quality['personal_space_violation_pair_ratio']:.6f}, "
                    "allowed="
                    f"{PEDESTRIAN_MAX_PERSONAL_SPACE_VIOLATION_RATIO:.6f}"
                )
            if pedestrian_inside_robot_frames != 0:
                raise RuntimeError(
                    "Pedestrian social validation found a person inside the robot "
                    f"for {pedestrian_inside_robot_frames} sampled frames"
                )
            if result["people_moving"] != result["people"]:
                raise RuntimeError(
                    "Pedestrian social validation requires every character to move: "
                    f"moving={result['people_moving']}/{result['people']}"
                )
        if PEOPLE_ENABLED and ARGS.duration > 0.0 and result["people_moving"] < 1:
            raise RuntimeError(
                "Dynamic-pedestrian validation failed: no character root moved "
                "more than 0.05 m"
            )
        if rtx_lidar is not None and ARGS.duration >= 2.0:
            measured_rate_hz = rtx_lidar.measured_pair_rate_hz
            allowed_error_hz = max(0.25, LIDAR_RATE_HZ * 0.15)
            if (
                measured_rate_hz is None
                or abs(measured_rate_hz - LIDAR_RATE_HZ) > allowed_error_hz
            ):
                raise RuntimeError(
                    "RTX dual-lidar rate validation failed: "
                    f"requested={LIDAR_RATE_HZ} Hz, "
                    f"measured={measured_rate_hz}, "
                    f"diagnostics={rtx_lidar.pairing_diagnostics}"
                )
        print("WAREHOUSE_PEOPLE_ROBOT_RESULT=" + json.dumps(result), flush=True)
        return 0
    except KeyboardInterrupt:
        print("[WAREHOUSE-ROBOT] Interrupted by user.", flush=True)
        return 0
    except Exception as exc:
        carb.log_error(f"Warehouse people robot integration failed: {exc}")
        print(f"[WAREHOUSE-ROBOT] ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if pedestrian_social_motion is not None:
            pedestrian_social_motion.close()
        if collision_proxy is not None:
            collision_proxy.close_dynamic_controller()
        if ros is not None:
            try:
                final_sim_time = (
                    float(timeline.get_current_time()) if timeline is not None else 0.0
                )
                ros.send_shutdown(final_sim_time)
            except Exception:
                pass
            ros.close()
        if rtx_lidar is not None:
            # Writer callbacks may fire during the stopped-timeline update;
            # reject them before their GMO render-product buffers are torn down.
            rtx_lidar.begin_close()
        if timeline is not None:
            timeline.stop()
            try:
                simulation_app.update()
            except Exception:
                pass
        if rtx_lidar is not None:
            rtx_lidar.close()
        simulation_app.close()


raise SystemExit(main())
