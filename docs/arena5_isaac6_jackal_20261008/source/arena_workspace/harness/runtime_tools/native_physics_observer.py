"""Read actual USD/PhysX robot state on the owned Isaac simulation thread.

No contact API, drive parameter, collider, or robot state is modified.
"""
import hashlib
import json
import os
import pathlib
import time
import traceback

import omni.usd
from omni.physx import get_physx_interface
from pxr import Usd, UsdGeom, UsdPhysics
from arena_isaac import run_after_tick_queue


def install(world):
    output = pathlib.Path(__file__).parent
    stream = (output / 'physics.jsonl').open('a', buffering=1)
    last_snapshot = 0.0
    errors = 0

    def emit(kind, **data):
        stream.write(json.dumps({'kind': kind, 'wall': time.monotonic(),
                                 'sim': world.current_time, **data}) + '\n')

    emit('provenance', pid=os.getpid(), domain=os.environ.get('ROS_DOMAIN_ID'),
         file=__file__, sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
         mode='read_only; no contact reporting API applied')

    def tick():
        nonlocal last_snapshot, errors
        try:
            if time.monotonic() - last_snapshot < .1:
                return
            last_snapshot = time.monotonic()
            phase_file = output / 'physics_phase.json'
            phase = json.loads(phase_file.read_text()).get('phase') if phase_file.exists() else 'startup'
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath('/World/env_0/Robots/jackal') if stage else None
            if not root or not root.IsValid():
                return
            cache = UsdGeom.XformCache()
            bodies, drives = [], []
            for prim in Usd.PrimRange(root):
                path = str(prim.GetPath())
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    transform = cache.GetLocalToWorldTransform(prim)
                    quat = transform.ExtractRotationQuat()
                    velocity = prim.GetAttribute('physics:velocity').Get()
                    angular = prim.GetAttribute('physics:angularVelocity').Get()
                    physx = get_physx_interface().get_rigidbody_transformation(path)
                    bodies.append({'path': path,
                                   'usd_position': list(transform.ExtractTranslation()),
                                   'usd_rotation_xyzw': list(quat.GetImaginary()) + [quat.GetReal()],
                                   'usd_velocity_attr': list(velocity) if velocity is not None else None,
                                   'usd_angular_velocity_attr': list(angular) if angular is not None else None,
                                   'physx_transform': {k: list(v) if k in ('position', 'rotation') else v
                                                       for k, v in physx.items()}})
                if prim.IsA(UsdPhysics.Joint) and 'wheel' in prim.GetName():
                    api = UsdPhysics.DriveAPI(prim, 'angular')
                    if api:
                        drives.append({'path': path,
                                       'target_velocity_deg_s': api.GetTargetVelocityAttr().Get(),
                                       'damping': api.GetDampingAttr().Get(),
                                       'stiffness': api.GetStiffnessAttr().Get(),
                                       'max_force': api.GetMaxForceAttr().Get()})
            emit('robot_physics', phase=phase, bodies=bodies, drives=drives)
        except Exception:
            errors += 1
            if errors <= 10:
                emit('error', error=traceback.format_exc(), error_count=errors)
        finally:
            run_after_tick_queue.put(tick)

    run_after_tick_queue.put(tick)
