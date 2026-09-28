#!/usr/bin/env python3
"""Isaac 6-style physical-proxy Mecanum adapter for the Gate 9 smoke."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from isaacsim.core.prims import SingleRigidPrim, XFormPrim
from isaacsim.core.utils import stage as stage_utils
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


ROBOT_PRIM = "/World/Robot"
PROXY_PRIM = "/World/RobotCollisionProxy"
MATERIAL_PRIM = "/World/Looks/Gate9RobotCollisionMaterial"
WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
ARM_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
ARM_LOCK_MODE = "authored_visual_pose_on_dynamic_base"


def yaw_from_quaternion(quaternion) -> float:
    w, x, y, z = [float(value) for value in quaternion]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.asarray([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


class Gate9MobileMecanum:
    """Port Isaac 6's fixed-pose visual follower plus dynamic box body."""

    def __init__(self, stage: Usd.Stage, world, robot_usd: Path, spawn=(2.0, 2.0, 0.01)):
        self.stage = stage
        self.world = world
        self.spawn = np.asarray(spawn, dtype=float)
        self.robot_visual_usd = robot_usd.parent / "configuration/mecanum730_xms5_default_base.usd"
        if not self.robot_visual_usd.is_file():
            raise FileNotFoundError(self.robot_visual_usd)
        # Match the active Isaac 6 runner: the full articulation is validated
        # separately, while navigation uses its collision-free visual/base
        # layer. The authored arm pose is therefore fixed and cannot fall or
        # be re-solved while the dynamic proxy moves underneath it.
        stage_utils.add_reference_to_stage(str(self.robot_visual_usd), ROBOT_PRIM)
        robot_root = stage.GetPrimAtPath(ROBOT_PRIM)
        if not robot_root.IsValid():
            raise RuntimeError("Isaac 6 robot visual layer did not compose")
        self.deinstanced_visual_count = 0
        for prim in Usd.PrimRange(robot_root):
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                self.deinstanced_visual_count += 1
        self.robot = XFormPrim(
            prim_paths_expr=ROBOT_PRIM,
            name="mecanum730_xms5_gate9_visual",
            positions=np.asarray([self.spawn], dtype=float),
            orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=float),
        )
        self.robot.set_world_poses(
            positions=np.asarray([self.spawn], dtype=float),
            orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=float),
        )

        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        ).ComputeWorldBound(stage.GetPrimAtPath(ROBOT_PRIM)).ComputeAlignedBox()
        dimensions = np.asarray(bbox.GetSize(), dtype=float)
        bbox_min = np.asarray(bbox.GetMin(), dtype=float)
        center = 0.5 * (bbox_min + np.asarray(bbox.GetMax(), dtype=float))
        self.center_offset = center - self.spawn
        self.visual_bottom_offset_z = float(bbox_min[2] - self.spawn[2])
        if dimensions[2] < 0.8:
            raise RuntimeError(f"robot visual layer is not upright: dimensions={dimensions.tolist()}")
        self.proxy_dimensions = dimensions.copy()
        self.proxy_dimensions[:2] += 0.04
        proxy_center = self.spawn + self.center_offset

        self.disabled_canonical_collisions = 0

        cube = UsdGeom.Cube.Define(stage, PROXY_PRIM)
        cube.CreateSizeAttr(1.0)
        cube.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        xformable = UsdGeom.Xformable(cube)
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*proxy_center))
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
        )
        xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*self.proxy_dimensions))
        proxy_prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(proxy_prim).CreateCollisionEnabledAttr(True)
        material = UsdShade.Material.Define(stage, MATERIAL_PRIM)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(0.0)
        physics_material.CreateDynamicFrictionAttr().Set(0.0)
        physics_material.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
        UsdShade.MaterialBindingAPI.Apply(proxy_prim).Bind(material, materialPurpose="physics")
        body = UsdPhysics.RigidBodyAPI.Apply(proxy_prim)
        body.CreateRigidBodyEnabledAttr(True)
        body.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(proxy_prim).CreateMassAttr(116.189)
        PhysxSchema.PhysxRigidBodyAPI.Apply(proxy_prim).CreateLockedRotAxisAttr(3)
        self.proxy = SingleRigidPrim(PROXY_PRIM, name="gate9_robot_physics_body", reset_xform_properties=False)
        self.arm_lock_mode = ARM_LOCK_MODE
        self.minimum_visual_bottom_z = math.inf
        self.maximum_visual_bottom_z = -math.inf
        self.last_command = (0.0, 0.0)

    def initialize(self) -> None:
        self.follow_visual()

    def pose(self) -> tuple[np.ndarray, float]:
        center, orientation = self.proxy.get_world_pose()
        yaw = yaw_from_quaternion(orientation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        offset = np.asarray([
            cosine * self.center_offset[0] - sine * self.center_offset[1],
            sine * self.center_offset[0] + cosine * self.center_offset[1],
            self.center_offset[2],
        ])
        return np.asarray(center, dtype=float) - offset, yaw

    def apply_command(self, linear: float, angular: float) -> None:
        linear = float(np.clip(linear, -0.30, 0.30))
        angular = float(np.clip(angular, -1.0, 1.0))
        _position, yaw = self.pose()
        velocity = np.asarray(self.proxy.get_linear_velocity(), dtype=float)
        self.proxy.set_linear_velocity(np.asarray([math.cos(yaw) * linear, math.sin(yaw) * linear, velocity[2]]))
        self.proxy.set_angular_velocity(np.asarray([0.0, 0.0, angular]))
        self.last_command = (linear, angular)

    def follow_visual(self) -> None:
        position, yaw = self.pose()
        self.robot.set_world_poses(
            positions=np.asarray([position], dtype=float),
            orientations=np.asarray([yaw_quaternion(yaw)], dtype=float),
        )
        bottom_z = float(position[2] + self.visual_bottom_offset_z)
        self.minimum_visual_bottom_z = min(self.minimum_visual_bottom_z, bottom_z)
        self.maximum_visual_bottom_z = max(self.maximum_visual_bottom_z, bottom_z)

    def stop(self) -> None:
        self.apply_command(0.0, 0.0)
