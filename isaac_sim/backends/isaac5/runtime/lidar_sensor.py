"""Non-RTX 2D laser based on the PhysX scene-query interface."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def ray_start_offsets_outside_box(
    sensor_xy: np.ndarray,
    ray_directions_xy: np.ndarray,
    box_half_extents_xy: np.ndarray,
    minimum_offset: float,
    epsilon: float = 0.01,
) -> np.ndarray:
    """Port Isaac 6's per-ray conservative robot self-filter start offsets."""

    sensor = np.asarray(sensor_xy, dtype=float).reshape(2)
    directions = np.asarray(ray_directions_xy, dtype=float)
    half_extents = np.asarray(box_half_extents_xy, dtype=float).reshape(2)
    directions = directions / np.linalg.norm(directions, axis=1)[:, None]
    if np.any(np.abs(sensor) >= half_extents):
        raise ValueError("sensor mount must be strictly inside robot XY bounds")
    boundaries = np.where(directions >= 0.0, half_extents, -half_extents)
    with np.errstate(divide="ignore", invalid="ignore"):
        axis_exit = (boundaries - sensor) / directions
    axis_exit[np.abs(directions) <= 1.0e-12] = np.inf
    exit_offsets = np.min(axis_exit, axis=1)
    if np.any(~np.isfinite(exit_offsets)) or np.any(exit_offsets < 0.0):
        raise ValueError("could not compute finite forward robot-box exits")
    return np.maximum(float(minimum_offset), exit_offsets + float(epsilon))


@dataclass
class LaserScanData:
    sim_time: float
    frame_id: str
    ranges: list[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    scan_time: float
    hit_paths: list[str]


class PhysxRaycastLidar:
    """A renderer-independent planar ray sampler.

    Rays begin at ``range_min`` from the sensor origin.  This avoids treating
    the robot's own chassis as the nearest obstacle and matches the historical
    Isaac5 scene-query backend.
    """

    def __init__(
        self,
        sample_count: int = 360,
        rate_hz: float = 10.0,
        range_min: float = 0.10,
        range_max: float = 20.0,
        frame_id: str = "base_scan",
        mount_translation: tuple[float, float, float] = (0.20, 0.13, 0.208),
        mount_yaw: float = 0.0,
        angle_min: float = -math.pi,
        angle_span: float = 2.0 * math.pi,
        robot_half_extents_xy: tuple[float, float] = (0.3123700064, 0.2435245481),
    ):
        self.sample_count = int(sample_count)
        self.rate_hz = float(rate_hz)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.frame_id = frame_id
        self.mount_translation = tuple(float(v) for v in mount_translation)
        self.mount_yaw = float(mount_yaw)
        if (
            self.sample_count < 4
            or self.rate_hz <= 0.0
            or self.range_max <= self.range_min
            or angle_span <= 0.0
        ):
            raise ValueError("invalid laser configuration")
        import omni.physx

        self.query = omni.physx.get_physx_scene_query_interface()
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_span) / self.sample_count
        robot_angles = (
            self.angle_min
            + np.arange(self.sample_count, dtype=float) * self.angle_increment
            + self.mount_yaw
        )
        robot_directions = np.column_stack((np.cos(robot_angles), np.sin(robot_angles)))
        self.ray_start_offsets = ray_start_offsets_outside_box(
            np.asarray(self.mount_translation[:2], dtype=float),
            robot_directions,
            np.asarray(robot_half_extents_xy, dtype=float),
            self.range_min,
        )
        self.sample_count_total = 0

    @staticmethod
    def _yaw(quat_wxyz: np.ndarray) -> float:
        w, x, y, z = [float(value) for value in quat_wxyz]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def sample(
        self, sim_time: float, robot_position: np.ndarray, robot_orientation: np.ndarray
    ) -> LaserScanData:
        robot_position = np.asarray(robot_position, dtype=float)
        robot_yaw = self._yaw(np.asarray(robot_orientation, dtype=float))
        cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
        tx, ty, tz = self.mount_translation
        sensor_x = robot_position[0] + cos_yaw * tx - sin_yaw * ty
        sensor_y = robot_position[1] + sin_yaw * tx + cos_yaw * ty
        sensor_z = robot_position[2] + tz
        sensor_yaw = robot_yaw + self.mount_yaw
        ranges: list[float] = []
        hit_paths: list[str] = []
        for index in range(self.sample_count):
            local_angle = self.angle_min + index * self.angle_increment
            world_angle = sensor_yaw + local_angle
            direction = (math.cos(world_angle), math.sin(world_angle), 0.0)
            start_offset = float(self.ray_start_offsets[index])
            origin = (
                sensor_x + direction[0] * start_offset,
                sensor_y + direction[1] * start_offset,
                sensor_z,
            )
            hit = self.query.raycast_closest(origin, direction, self.range_max - start_offset)
            if hit["hit"]:
                ranges.append(min(self.range_max, start_offset + float(hit["distance"])))
                hit_paths.append(str(hit.get("collision", "")))
            else:
                ranges.append(float("inf"))
                hit_paths.append("")
        self.sample_count_total += 1
        return LaserScanData(
            sim_time=float(sim_time),
            frame_id=self.frame_id,
            ranges=ranges,
            angle_min=self.angle_min,
            angle_increment=self.angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            scan_time=1.0 / self.rate_hz,
            hit_paths=hit_paths,
        )


@dataclass(frozen=True)
class DualLaserScanData:
    """Two scene-query scans captured at one simulation timestamp."""

    scan_01: LaserScanData
    scan_02: LaserScanData


class DualPhysxRaycastLidar:
    """Independent non-RTX front/rear raycasters with a shared sample time.

    The two sensors intentionally keep separate frames and transforms.  Both
    scans use the same fixed beam layout so a downstream synchronizer can
    reject layout drift without hiding the physical sensor offset.
    """

    def __init__(
        self,
        sample_count: int = 2000,
        rate_hz: float = 15.0,
        range_min: float = 0.5,
        range_max: float = 50.0,
        angle_min: float = -math.pi,
        angle_span: float = 2.0 * math.pi,
    ):
        common = {
            "sample_count": sample_count,
            "rate_hz": rate_hz,
            "range_min": range_min,
            "range_max": range_max,
            "angle_min": angle_min,
            "angle_span": angle_span,
        }
        self.scan_01 = PhysxRaycastLidar(
            frame_id="base_scan_01",
            mount_translation=(0.20, 0.13, 0.208),
            mount_yaw=0.0,
            **common,
        )
        self.scan_02 = PhysxRaycastLidar(
            frame_id="base_scan_02",
            mount_translation=(-0.20, -0.13, 0.208),
            mount_yaw=math.pi,
            **common,
        )
        self.rate_hz = float(rate_hz)
        self.sample_count = int(sample_count)

    def sample(
        self,
        sim_time: float,
        robot_position: np.ndarray,
        robot_orientation: np.ndarray,
    ) -> DualLaserScanData:
        return DualLaserScanData(
            scan_01=self.scan_01.sample(sim_time, robot_position, robot_orientation),
            scan_02=self.scan_02.sample(sim_time, robot_position, robot_orientation),
        )
