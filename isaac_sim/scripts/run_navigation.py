#!/usr/bin/env python3
"""Run the project Mecanum robot, ROS 2 topics, dual scans, and test pedestrians."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_USD = (
    PROJECT_ROOT.parent
    / "robot_related"
    / "robots"
    / "chassis_arm"
    / "motion_wheel_arm_simple_sphere_usd"
    / "mecanum730_xms5_default.usd"
)
SCENE_USD = PROJECT_ROOT / "isaac_sim" / "scenes" / "mecanum_lidar_main.usd"
ROBOT_PRIM = "/World/Robot"
ARTICULATION_ROOT = f"{ROBOT_PRIM}/base_footprint"
PHYSICS_DT = 1.0 / 60.0
WHEEL_RADIUS_M = 0.077993
WHEEL_BASE_SUM_M = 0.1575 + 0.1725
# Match the Isaac 6 warehouse runner: retain only an extreme-value guard and
# do not silently cap ordinary teleoperation commands.
MAX_LINEAR_MPS = 10.0
MAX_ANGULAR_RADPS = 1.5
ROOT_HEIGHT_M = 0.0
COMMAND_TIMEOUT_SEC = 0.5
WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
LIDAR_SAMPLE_COUNT = 360
LIDAR_RANGE_MIN_M = 1.0
LIDAR_RANGE_MAX_M = 50.0
LIDAR_PUBLISH_EVERY_FRAMES = 6
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
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--renderer",
        default="RaytracedLighting",
        help=(
            "Isaac Sim viewport renderer (for example RaytracedLighting or "
            "Wireframe). Wireframe is useful when the RTX Hydra viewport is "
            "unstable on a newer GPU/driver combination."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Simulation seconds before exit; 0 runs until the window closes or Ctrl-C.",
    )
    parser.add_argument("--pedestrians", type=int, default=4)
    parser.add_argument("--pedestrian-seed", type=int, default=7)
    parser.add_argument("--pedestrian-speed", type=float, default=0.8)
    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="Disable the PhysX raycast LaserScan publishers.",
    )
    parser.add_argument(
        "--rtx-lidar-prims",
        action="store_true",
        help=(
            "Also create the two Example_Rotary_2D RTX Lidar prims. Disabled by "
            "default because moving RTX sensors intermittently crash Isaac Sim 5.1 "
            "on this RTX 5090; ROS LaserScan still uses PhysX raycasts."
        ),
    )
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument(
        "--test-command",
        nargs=3,
        type=float,
        metavar=("VX", "VY", "WZ"),
        help="Apply a constant body-frame command for repeatable smoke tests.",
    )
    parser.add_argument("--fast", action="store_true", help="Do not pace headless simulation in real time.")
    parser.add_argument("--no-save-scene", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
if ARGS.duration < 0.0:
    raise ValueError("--duration must be non-negative")
if ARGS.pedestrians < 0:
    raise ValueError("--pedestrians must be non-negative")

RTX_LIDAR_PRIMS_ENABLED = bool(ARGS.rtx_lidar_prims and not ARGS.no_lidar)

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": ARGS.renderer,
        # PhysX raycasts provide the ROS scans and do not require motion BVH.
        # Only enable it for the opt-in RTX diagnostic prims: moving RTX
        # sensors intermittently crash Isaac Sim 5.1 on this RTX 5090.
        "multi_gpu": False,
        "enable_motion_bvh": RTX_LIDAR_PRIMS_ENABLED,
        "width": 1280,
        "height": 720,
    }
)

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.physx  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCapsule, FixedCuboid  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Gf, Sdf, UsdLux, UsdPhysics, UsdSemantics  # noqa: E402


if not ARGS.no_ros:
    enable_extension("isaacsim.ros2.bridge")
if RTX_LIDAR_PRIMS_ENABLED:
    enable_extension("isaacsim.sensors.rtx")
simulation_app.update()

if not ARGS.no_ros:
    import rclpy  # noqa: E402
    from builtin_interfaces.msg import Time as RosTime  # noqa: E402
    from geometry_msgs.msg import TransformStamped, Twist, TwistStamped  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402
    from rosgraph_msgs.msg import Clock  # noqa: E402
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
    from sensor_msgs.msg import LaserScan  # noqa: E402
    from std_msgs.msg import String  # noqa: E402
    from tf2_msgs.msg import TFMessage  # noqa: E402


def yaw_from_quaternion(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = [float(value) for value in quat_wxyz]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.asarray([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=float)


def clamp_twist(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    planar_norm = math.hypot(vx, vy)
    if planar_norm > MAX_LINEAR_MPS:
        scale = MAX_LINEAR_MPS / planar_norm
        vx *= scale
        vy *= scale
    return vx, vy, max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, wz))


def mecanum_wheel_velocities(vx: float, vy: float, wz: float) -> np.ndarray:
    return np.asarray(
        [
            (vx - vy - WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx + vy + WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx + vy - WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
            (vx - vy + WHEEL_BASE_SUM_M * wz) / WHEEL_RADIUS_M,
        ],
        dtype=np.float32,
    )


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
    root = stage_utils.get_current_stage().GetPrimAtPath(ARTICULATION_ROOT)
    if not root.IsValid() or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(f"Missing articulation root: {ARTICULATION_ROOT}")
    root.CreateAttribute(
        "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int
    ).Set(16)
    root.CreateAttribute(
        "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int
    ).Set(4)
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(
        False
    )


def add_test_obstacles(world: World) -> None:
    wall_specs = (
        ("north", (0.0, 3.0, 0.75), (6.0, 0.12, 1.5)),
        ("south", (0.0, -3.0, 0.75), (6.0, 0.12, 1.5)),
        ("east", (3.0, 0.0, 0.75), (0.12, 6.0, 1.5)),
        ("west", (-3.0, 0.0, 0.75), (0.12, 6.0, 1.5)),
        ("box_a", (1.5, 1.2, 0.5), (0.6, 0.6, 1.0)),
        ("box_b", (-1.6, -1.0, 0.35), (0.8, 0.5, 0.7)),
    )
    for name, position, scale in wall_specs:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/TestEnvironment/{name}",
                name=f"obstacle_{name}",
                position=np.asarray(position),
                scale=np.asarray(scale),
                color=np.asarray([0.55, 0.58, 0.62]),
            )
        )


def lock_navigation_arm(
    robot: Robot, arm_indices: np.ndarray, arm_positions: np.ndarray
) -> None:
    """Hard-lock the non-navigation arm joints for base-navigation runs.

    Position targets alone are not enough for this imported robot: its arm
    joints have gravity and comparatively soft USD drive settings, so they can
    visibly sag or oscillate while the base is teleoperated.  Resetting the
    joint state before and after every physics step keeps the arm fixed without
    changing the four wheel DOFs used by ``/cmd_vel``.
    """
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


def set_safe_joint_pose(robot: Robot) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dof_names = list(robot.dof_names)
    missing = [name for name in WHEEL_NAMES if name not in dof_names]
    if missing:
        raise RuntimeError(f"Missing expected wheel joints: {missing}")
    wheel_indices = np.asarray([dof_names.index(name) for name in WHEEL_NAMES], dtype=np.int32)
    arm_indices = []
    arm_positions = []
    for name, value in SAFE_ARM_JOINTS.items():
        if name in dof_names:
            arm_indices.append(dof_names.index(name))
            arm_positions.append(value)
    arm_indices_np = np.asarray(arm_indices, dtype=np.int32)
    arm_positions_np = np.asarray(arm_positions, dtype=np.float32)
    lock_navigation_arm(robot, arm_indices_np, arm_positions_np)
    return wheel_indices, arm_indices_np, arm_positions_np


@dataclass
class Pedestrian:
    identifier: str
    body: DynamicCapsule
    start: np.ndarray
    end: np.ndarray
    speed: float
    phase: float
    position: np.ndarray
    velocity: np.ndarray


class PedestrianManager:
    def __init__(self, world: World, count: int, seed: int, speed: float):
        self.pedestrians: list[Pedestrian] = []
        rng = np.random.default_rng(seed)
        routes = (
            ((-2.4, -2.1), (2.4, -2.1)),
            ((-2.3, 2.0), (2.3, 2.0)),
            ((-2.1, -2.4), (-2.1, 2.4)),
            ((2.1, -2.4), (2.1, 2.4)),
            ((-2.2, -1.5), (2.2, 1.5)),
            ((-2.2, 1.5), (2.2, -1.5)),
        )
        colors = (
            np.asarray([0.18, 0.55, 0.95]),
            np.asarray([0.95, 0.38, 0.22]),
            np.asarray([0.30, 0.78, 0.42]),
            np.asarray([0.78, 0.32, 0.82]),
        )
        for index in range(count):
            route_start, route_end = routes[index % len(routes)]
            start = np.asarray([*route_start, 0.85], dtype=float)
            end = np.asarray([*route_end, 0.85], dtype=float)
            phase = float(rng.random())
            initial = start + phase * (end - start)
            body = world.scene.add(
                DynamicCapsule(
                    prim_path=f"/World/Pedestrians/person_{index:02d}",
                    name=f"person_{index:02d}",
                    position=initial,
                    radius=0.24,
                    height=1.2,
                    color=colors[index % len(colors)],
                    mass=70.0,
                )
            )
            prim = stage_utils.get_current_stage().GetPrimAtPath(body.prim_path)
            semantic_api = UsdSemantics.LabelsAPI.Apply(prim, "class")
            semantic_api.CreateLabelsAttr().Set(["Person"])
            self.pedestrians.append(
                Pedestrian(
                    identifier=f"person_{index:02d}",
                    body=body,
                    start=start,
                    end=end,
                    speed=max(0.05, speed * float(rng.uniform(0.85, 1.15))),
                    phase=phase,
                    position=initial.copy(),
                    velocity=np.zeros(3, dtype=float),
                )
            )

    def initialize_kinematic_bodies(self) -> None:
        """Switch bodies after Scene reset to avoid resetting velocities on kinematic actors."""
        stage = stage_utils.get_current_stage()
        for pedestrian in self.pedestrians:
            prim = stage.GetPrimAtPath(pedestrian.body.prim_path)
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            if not rigid_body_api:
                rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_body_api.CreateKinematicEnabledAttr().Set(True)

    def update(self, sim_time: float) -> None:
        for pedestrian in self.pedestrians:
            delta = pedestrian.end - pedestrian.start
            route_length = float(np.linalg.norm(delta[:2]))
            if route_length <= 1.0e-6:
                continue
            unit = delta / route_length
            cycle_distance = (pedestrian.phase * 2.0 * route_length + pedestrian.speed * sim_time) % (
                2.0 * route_length
            )
            if cycle_distance <= route_length:
                distance = cycle_distance
                direction = 1.0
            else:
                distance = 2.0 * route_length - cycle_distance
                direction = -1.0
            pedestrian.position = pedestrian.start + unit * distance
            pedestrian.velocity = unit * pedestrian.speed * direction
            yaw = math.atan2(pedestrian.velocity[1], pedestrian.velocity[0])
            pedestrian.body.set_world_pose(pedestrian.position, yaw_quaternion(yaw))

    def json_payload(self, sim_time: float) -> str:
        return json.dumps(
            {
                "schema": "a_pipeline_pedestrian_ground_truth/v1",
                "frame_id": "odom",
                "sim_time": sim_time,
                "pedestrians": [
                    {
                        "id": pedestrian.identifier,
                        "position": pedestrian.position.tolist(),
                        "velocity": pedestrian.velocity.tolist(),
                        "yaw": math.atan2(pedestrian.velocity[1], pedestrian.velocity[0]),
                    }
                    for pedestrian in self.pedestrians
                ],
            },
            separators=(",", ":"),
        )


class RosInterface:
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node("a_pipeline_isaac_sim")
        self.lock = threading.Lock()
        self.command = (0.0, 0.0, 0.0)
        self.last_command_wall_time = -math.inf
        self.sim_time = 0.0
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        static_tf_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.node.create_publisher(TFMessage, "/tf_static", static_tf_qos)
        self.cmd_stamped_pub = self.node.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.scan_01_pub = self.node.create_publisher(
            LaserScan, "/scan_01", qos_profile_sensor_data
        )
        self.scan_02_pub = self.node.create_publisher(
            LaserScan, "/scan_02", qos_profile_sensor_data
        )
        self.pedestrian_json_pub = self.node.create_publisher(
            String, "/isaac_sim/pedestrian_ground_truth_json", 10
        )
        self.episode_pub = self.node.create_publisher(String, "/data_collection/episode_event", 10)
        self.subscription = self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.publish_static_tf()

    @staticmethod
    def stamp(seconds: float) -> RosTime:
        seconds = max(0.0, seconds)
        whole = int(seconds)
        return RosTime(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))

    def _on_cmd_vel(self, message: Twist) -> None:
        command = clamp_twist(message.linear.x, message.linear.y, message.angular.z)
        with self.lock:
            self.command = command
            self.last_command_wall_time = time.monotonic()
        stamped = TwistStamped()
        stamped.header.stamp = self.stamp(self.sim_time)
        stamped.header.frame_id = "base_link"
        stamped.twist = message
        self.cmd_stamped_pub.publish(stamped)

    def spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def current_command(self) -> tuple[float, float, float]:
        with self.lock:
            if time.monotonic() - self.last_command_wall_time > COMMAND_TIMEOUT_SEC:
                return 0.0, 0.0, 0.0
            return self.command

    def publish_clock(self, sim_time: float) -> None:
        self.sim_time = sim_time
        message = Clock()
        message.clock = self.stamp(sim_time)
        self.clock_pub.publish(message)

    def publish_static_tf(self) -> None:
        transforms = []
        specs = (
            ("base_scan", (0.2, 0.13, 0.208), 0.0),
            ("base_scan_01", (0.2, 0.13, 0.208), 0.0),
            ("base_scan_02", (-0.2, -0.13, 0.208), math.pi),
        )
        for child, translation, yaw in specs:
            transform = TransformStamped()
            transform.header.stamp = self.stamp(0.0)
            transform.header.frame_id = "base_link"
            transform.child_frame_id = child
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            quat = yaw_quaternion(yaw)
            transform.transform.rotation.w = float(quat[0])
            transform.transform.rotation.x = float(quat[1])
            transform.transform.rotation.y = float(quat[2])
            transform.transform.rotation.z = float(quat[3])
            transforms.append(transform)
        self.tf_static_pub.publish(TFMessage(transforms=transforms))

    def publish_odometry(
        self,
        sim_time: float,
        position: np.ndarray,
        orientation: np.ndarray,
        body_twist: tuple[float, float, float],
    ) -> None:
        stamp = self.stamp(sim_time)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = float(orientation[0])
        odom.pose.pose.orientation.x = float(orientation[1])
        odom.pose.pose.orientation.y = float(orientation[2])
        odom.pose.pose.orientation.z = float(orientation[3])
        odom.twist.twist.linear.x = float(body_twist[0])
        odom.twist.twist.linear.y = float(body_twist[1])
        odom.twist.twist.angular.z = float(body_twist[2])
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[transform]))

    def publish_pedestrians(self, sim_time: float, manager: PedestrianManager) -> None:
        message = String()
        message.data = manager.json_payload(sim_time)
        self.pedestrian_json_pub.publish(message)

    def make_laser_scan(
        self,
        sim_time: float,
        robot_position: np.ndarray,
        robot_orientation: np.ndarray,
        mount_translation: tuple[float, float, float],
        mount_yaw: float,
        frame_id: str,
    ) -> LaserScan:
        robot_yaw = yaw_from_quaternion(robot_orientation)
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        sensor_x = (
            float(robot_position[0])
            + cos_yaw * mount_translation[0]
            - sin_yaw * mount_translation[1]
        )
        sensor_y = (
            float(robot_position[1])
            + sin_yaw * mount_translation[0]
            + cos_yaw * mount_translation[1]
        )
        sensor_z = float(robot_position[2]) + mount_translation[2]
        sensor_yaw = robot_yaw + mount_yaw
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / LIDAR_SAMPLE_COUNT
        query = omni.physx.get_physx_scene_query_interface()
        ranges = []
        for index in range(LIDAR_SAMPLE_COUNT):
            local_angle = angle_min + index * angle_increment
            world_angle = sensor_yaw + local_angle
            direction = (math.cos(world_angle), math.sin(world_angle), 0.0)
            # Example_Rotary_2D has a 1 m near range. Starting the PhysX ray
            # there also excludes the robot's own chassis and arm geometry.
            origin = (
                sensor_x + direction[0] * LIDAR_RANGE_MIN_M,
                sensor_y + direction[1] * LIDAR_RANGE_MIN_M,
                sensor_z,
            )
            hit = query.raycast_closest(
                origin, direction, LIDAR_RANGE_MAX_M - LIDAR_RANGE_MIN_M
            )
            if hit["hit"]:
                ranges.append(LIDAR_RANGE_MIN_M + float(hit["distance"]))
            else:
                ranges.append(float("inf"))

        scan = LaserScan()
        scan.header.stamp = self.stamp(sim_time)
        scan.header.frame_id = frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (LIDAR_SAMPLE_COUNT - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.time_increment = 0.1 / LIDAR_SAMPLE_COUNT
        scan.scan_time = 0.1
        scan.range_min = LIDAR_RANGE_MIN_M
        scan.range_max = LIDAR_RANGE_MAX_M
        scan.ranges = ranges
        scan.intensities = []
        return scan

    def publish_lasers(
        self,
        sim_time: float,
        robot_position: np.ndarray,
        robot_orientation: np.ndarray,
    ) -> None:
        front = self.make_laser_scan(
            sim_time,
            robot_position,
            robot_orientation,
            (0.2, 0.13, 0.208),
            0.0,
            "base_scan_01",
        )
        rear = self.make_laser_scan(
            sim_time,
            robot_position,
            robot_orientation,
            (-0.2, -0.13, 0.208),
            math.pi,
            "base_scan_02",
        )
        self.scan_01_pub.publish(front)
        self.scan_02_pub.publish(rear)
        front.header.frame_id = "base_scan"
        self.scan_pub.publish(front)

    def publish_episode_event(self, event: str, sim_time: float) -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema": "semantic_nav_episode_event/v1",
                "event": event,
                "episode_id": "isaac_sim_runtime",
                "sim_time": sim_time,
            },
            separators=(",", ":"),
        )
        self.episode_pub.publish(message)

    def close(self) -> None:
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def create_lidar(parent: str, name: str, translation: tuple[float, float, float], yaw: float):
    orientation = Gf.Quatd(math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=f"/{name}",
        parent=parent,
        config="Example_Rotary_2D",
        translation=translation,
        orientation=orientation,
    )
    if sensor is None or not sensor.IsValid():
        raise RuntimeError(f"Failed to create RTX lidar: {name}")
    return sensor


def save_scene() -> None:
    SCENE_USD.parent.mkdir(parents=True, exist_ok=True)
    if not stage_utils.save_stage(str(SCENE_USD), save_and_reload_in_place=False):
        raise RuntimeError(f"Failed to save scene: {SCENE_USD}")


def apply_planar_root_velocity(robot: Robot, command: tuple[float, float, float]) -> None:
    position, orientation = robot.get_world_pose()
    position = np.asarray(position, dtype=float)
    orientation = np.asarray(orientation, dtype=float)
    yaw = yaw_from_quaternion(orientation)
    vx, vy, wz = command
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_vx = cos_yaw * vx - sin_yaw * vy
    world_vy = sin_yaw * vx + cos_yaw * vy
    vertical = float(np.clip(8.0 * (ROOT_HEIGHT_M - position[2]), -0.35, 0.35))

    w, x, y, z = orientation
    body_up = np.asarray(
        [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]
    )
    upright_axis = np.cross(body_up, np.asarray([0.0, 0.0, 1.0]))
    robot.set_linear_velocity(np.asarray([world_vx, world_vy, vertical]))
    robot.set_angular_velocity(
        np.asarray(
            [
                float(np.clip(10.0 * upright_axis[0], -1.5, 1.5)),
                float(np.clip(10.0 * upright_axis[1], -1.5, 1.5)),
                wz,
            ]
        )
    )


def main() -> int:
    if not ROBOT_USD.is_file():
        raise FileNotFoundError(f"Robot USD not found: {ROBOT_USD}")

    stage_utils.create_new_stage()
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane(
        z_position=0.0,
        static_friction=1.0,
        dynamic_friction=0.8,
        restitution=0.0,
    )
    add_lighting()
    add_test_obstacles(world)
    stage_utils.add_reference_to_stage(str(ROBOT_USD), ROBOT_PRIM)
    robot = world.scene.add(
        Robot(
            prim_path=ROBOT_PRIM,
            name="mecanum730_xms5",
            position=np.asarray([0.0, 0.0, ROOT_HEIGHT_M]),
            orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
        )
    )
    tune_articulation()
    pedestrian_manager = PedestrianManager(
        world, ARGS.pedestrians, ARGS.pedestrian_seed, ARGS.pedestrian_speed
    )

    if RTX_LIDAR_PRIMS_ENABLED:
        create_lidar(ARTICULATION_ROOT, "base_scan_01", (0.2, 0.13, 0.208), 0.0)
        create_lidar(ARTICULATION_ROOT, "base_scan_02", (-0.2, -0.13, 0.208), math.pi)
        simulation_app.update()

    if not ARGS.no_save_scene:
        save_scene()

    world.reset()
    pedestrian_manager.initialize_kinematic_bodies()
    robot.set_world_pose(
        position=np.asarray([0.0, 0.0, ROOT_HEIGHT_M]),
        orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
    )
    wheel_indices, arm_indices, arm_positions = set_safe_joint_pose(robot)
    ros = RosInterface() if not ARGS.no_ros else None

    simulation_app.update()
    start_wall = time.monotonic()
    frame = 0
    final_position = np.asarray([0.0, 0.0, ROOT_HEIGHT_M], dtype=float)
    final_orientation = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=float)
    exit_reason = "window_closed"
    if ros:
        ros.publish_episode_event("start", 0.0)
    print(
        "NAVIGATION_RUNTIME_READY="
        + json.dumps(
            {
                "robot": str(ROBOT_USD),
                "scene": str(SCENE_USD),
                "wheel_indices": wheel_indices.tolist(),
                "arm_lock": "reset joint position and velocity every physics step",
                "pedestrians": ARGS.pedestrians,
                "ros": not ARGS.no_ros,
                "lidar": not ARGS.no_lidar,
                "laser_scan_backend": "physx_raycast" if not ARGS.no_lidar else None,
                "rtx_lidar_prims": RTX_LIDAR_PRIMS_ENABLED,
                "topics": [
                    "/scan",
                    "/scan_01",
                    "/scan_02",
                    "/odom",
                    "/tf",
                    "/tf_static",
                    "/clock",
                    "/cmd_vel",
                    "/cmd_vel_stamped",
                    "/isaac_sim/pedestrian_ground_truth_json",
                    "/data_collection/episode_event",
                ]
                if ros
                else [],
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    try:
        while simulation_app.is_running():
            loop_start = time.monotonic()
            if ros:
                ros.spin_once()
                command = ros.current_command()
            elif ARGS.test_command:
                command = clamp_twist(*ARGS.test_command)
            else:
                command = (0.0, 0.0, 0.0)

            apply_planar_root_velocity(robot, command)
            robot.get_articulation_controller().apply_action(
                ArticulationAction(
                    joint_velocities=mecanum_wheel_velocities(*command),
                    joint_indices=wheel_indices,
                )
            )
            lock_navigation_arm(robot, arm_indices, arm_positions)

            sim_time = float(world.current_time)
            pedestrian_manager.update(sim_time)
            world.step(render=not ARGS.headless)
            # Reapply after PhysX so gravity cannot leave a visible arm offset
            # between the current frame and the next teleop command.
            lock_navigation_arm(robot, arm_indices, arm_positions)
            sim_time = float(world.current_time)

            if ros:
                ros.publish_clock(sim_time)
                if frame % 2 == 0:
                    position, orientation = robot.get_world_pose()
                    ros.publish_odometry(
                        sim_time,
                        np.asarray(position, dtype=float),
                        np.asarray(orientation, dtype=float),
                        command,
                    )
                if frame % 4 == 0:
                    ros.publish_pedestrians(sim_time, pedestrian_manager)
                if not ARGS.no_lidar and frame % LIDAR_PUBLISH_EVERY_FRAMES == 0:
                    position, orientation = robot.get_world_pose()
                    ros.publish_lasers(
                        sim_time,
                        np.asarray(position, dtype=float),
                        np.asarray(orientation, dtype=float),
                    )

            frame += 1
            if ARGS.duration > 0.0 and sim_time >= ARGS.duration:
                exit_reason = "duration_reached"
                break
            if ARGS.headless and not ARGS.fast:
                remaining = PHYSICS_DT - (time.monotonic() - loop_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
    finally:
        final_sim_time = float(world.current_time)
        final_position, final_orientation = robot.get_world_pose()
        final_position = np.asarray(final_position, dtype=float)
        final_orientation = np.asarray(final_orientation, dtype=float)
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        if ros:
            ros.publish_episode_event("end", final_sim_time)
            for _ in range(3):
                ros.spin_once()
                time.sleep(0.01)
            ros.close()
        world.stop()

    print(
        "NAVIGATION_RUNTIME_RESULT="
        + json.dumps(
            {
                "status": "PASS",
                "exit_reason": exit_reason,
                "sim_time": final_sim_time,
                "wall_time": time.monotonic() - start_wall,
                "frames": frame,
                "final_position": final_position.tolist(),
                "final_orientation_wxyz": final_orientation.tolist(),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


try:
    raise SystemExit(main())
finally:
    simulation_app.close()
