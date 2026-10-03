#!/usr/bin/env python3
"""Isolated Jackal contact/drive evidence capture; never edits installed assets."""
import argparse
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--model', choices=('current', 'official'), required=True)
parser.add_argument('--no-ground', action='store_true', help='distant diagnostic floor for target-sign control')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error('output exists; refusing overwrite')
args.output.mkdir(parents=True)

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'Wireframe', 'multi_gpu': False,
                     'fast_shutdown': True, 'enable_crashreporter': False})
import numpy as np
import omni.graph.core as og
import omni.kit.commands
from omni.physx import get_physx_simulation_interface
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils import extensions
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import PhysxSchema, PhysicsSchemaTools, UsdGeom, UsdPhysics

DT = 1 / 60
NAMES = ['front_left_wheel_joint', 'front_right_wheel_joint',
         'rear_left_wheel_joint', 'rear_right_wheel_joint']
URDF = Path('/home/user/arena_isaac5_host_overlay_ws/install/arena_simulation_setup/share/arena_simulation_setup/entities/robots/jackal/urdf/jackal.urdf')
OFFICIAL = Path('/home/user/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/Clearpath/Jackal/jackal.usd')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def yaw(q):
    w, x, y, z = map(float, q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def vec(v):
    return [float(v[0]), float(v[1]), float(v[2])]

def make_urdf():
    tree = ET.parse(URDF)
    for el in tree.getroot().iter():
        name = el.attrib.get('filename')
        if name:
            if name.startswith('package://jackal_description/'):
                name = name.split('package://jackal_description/', 1)[1]
            el.set('filename', str((URDF.parent / name).resolve()) if not Path(name).is_absolute() else name)
    path = args.output / 'jackal_probe.urdf'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return path

def graph(pair, index):
    path = f'/World/jackal/forensic_diff_{index}'
    og.Controller.edit({'graph_path': path, 'evaluator_name': 'execution'}, {
        og.Controller.Keys.CREATE_NODES: [('tick','omni.graph.action.OnPlaybackTick'),
            ('diff','isaacsim.robot.wheeled_robots.DifferentialController'),
            ('art','isaacsim.core.nodes.IsaacArticulationController'),
            ('left','omni.graph.nodes.ConstantToken'), ('right','omni.graph.nodes.ConstantToken'),
            ('names','omni.graph.nodes.ConstructArray')],
        og.Controller.Keys.SET_VALUES: [('diff.inputs:wheelDistance',0.54),
            ('diff.inputs:wheelRadius',0.098),('diff.inputs:maxWheelSpeed',10.0),
            ('diff.inputs:maxLinearSpeed',-2.0),('diff.inputs:maxAngularSpeed',-4.0),
            ('diff.inputs:linearVelocity',0.0),('diff.inputs:angularVelocity',0.0),
            ('art.inputs:targetPrim','/World/jackal'),('names.inputs:arraySize',2),
            ('left.inputs:value',pair[0]),('right.inputs:value',pair[1])],
        og.Controller.Keys.CREATE_ATTRIBUTES: [('names.inputs:input1','token')],
        og.Controller.Keys.CONNECT: [('tick.outputs:tick','diff.inputs:execIn'),
            ('tick.outputs:tick','art.inputs:execIn'),
            ('diff.outputs:velocityCommand','art.inputs:velocityCommand'),
            ('left.inputs:value','names.inputs:input0'),('right.inputs:value','names.inputs:input1'),
            ('names.outputs:array','art.inputs:jointNames')]})
    return path

def attributes(prim):
    return {str(a.GetName()): str(a.Get()) for a in prim.GetAttributes()
            if any(s in str(a.GetName()).lower() for s in
                   ('drive','mass','inertia','solver','friction','contactoffset','restoffset','axis','localrot','localpos','radius','height'))}

def read_forces(robot, indices, force_indices, capability):
    out = {}
    for key, method in [('measured_joint_effort','get_measured_joint_efforts'),
                        ('measured_joint_force','get_measured_joint_forces'),
                        ('applied_joint_effort','get_applied_joint_efforts')]:
        try:
            value = getattr(robot, method)()
            rows = force_indices if key == 'measured_joint_force' else indices
            out[key] = np.asarray(value, dtype=float)[rows].tolist()
            capability[key] = 'OBSERVED'
        except Exception as exc:
            out[key] = None
            capability[key] = 'NOT_OBSERVABLE_WITH_CURRENT_API: ' + repr(exc)
    return out

def report_contacts(physx, frame, stream, errors):
    try:
        headers, contacts, anchors = physx.get_full_contact_report()
        for h in headers:
            c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            if 'wheel' not in (c0+c1):
                continue
            data = {'frame':frame, 'event':str(h.type), 'actor0':str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                    'actor1':str(PhysicsSchemaTools.intToSdfPath(h.actor1)), 'collider0':c0,
                    'collider1':c1, 'points':[], 'friction_anchors':[]}
            for j in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data):
                c=contacts[j]
                data['points'].append({'position':vec(c.position),'normal':vec(c.normal),
                    'impulse':vec(c.impulse),'separation':float(c.separation)})
            for j in range(h.friction_anchors_offset, h.friction_anchors_offset+h.num_friction_anchors_data):
                a=anchors[j]
                data['friction_anchors'].append({'position':vec(a.position),'impulse':vec(a.impulse)})
            stream.write(json.dumps(data)+'\n')
    except Exception as exc:
        errors.setdefault('contact_report',repr(exc))

manifest={'model':args.model,'status':'INCOMPLETE','physics_dt':DT,
          'windows':[[-0.2,3],[0,1],[-0.5,3],[0,1]],
          'source_path':str(URDF if args.model=='current' else OFFICIAL),
          'source_sha256':sha(URDF if args.model=='current' else OFFICIAL),
          'floor_type':'distant_fixed_cuboid' if args.no_ground else 'formal_ground_plane',
          'effort_semantics':'measured_joint_effort is solver projected incoming joint force; applied_joint_effort is explicit API effort, not drive output',
          'drive_output':'NOT_OBSERVABLE_WITH_CURRENT_API'}
try:
    if args.model=='current':
        extensions.enable_extension('isaacsim.asset.importer.urdf')
        extensions.enable_extension('isaacsim.core.nodes')
        extensions.enable_extension('isaacsim.robot.wheeled_robots')
    world=World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    if args.no_ground:
        world.scene.add(FixedCuboid(prim_path='/World/floor',name='distant_floor',
                                    position=np.array([0.,0.,-100.05]),scale=np.array([10.,10.,0.1])))
    else:
        world.scene.add_ground_plane(size=100,z_position=0.0)
    if args.model=='current':
        temp=make_urdf()
        manifest['probe_urdf_sha256']=sha(temp)
        _,cfg=omni.kit.commands.execute('URDFCreateImportConfig')
        cfg.set_merge_fixed_joints(False); cfg.set_convex_decomp(False)
        cfg.set_import_inertia_tensor(False); cfg.set_make_default_prim(False)
        cfg.set_distance_scale(1.0); cfg.set_fix_base(False)
        cfg.set_default_drive_type(2); cfg.set_self_collision(False)
        _,path=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(temp),import_config=cfg,dest_path='')
        if not path: raise RuntimeError('URDF import returned no path')
        if str(path)!='/World/jackal':
            omni.kit.commands.execute('MovePrim',path_from=str(path),path_to='/World/jackal',keep_world_transform=True)
        robot=world.scene.add(Robot(prim_path='/World/jackal',name='jackal_forensic',position=np.array([0.,0.,0.0635])))
        graphs=[graph(NAMES[:2],0),graph(NAMES[2:],1)]
    else:
        add_reference_to_stage(usd_path=str(OFFICIAL),prim_path='/World/jackal')
        robot=world.scene.add(Robot(prim_path='/World/jackal',name='jackal_official',position=np.array([0.,0.,0.0635])))
        graphs=[]
    stage=world.stage
    prims=[p for p in stage.Traverse() if 'wheel' in str(p.GetPath()) or p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    manifest['physics_snapshot']={str(p.GetPath()):{'type':p.GetTypeName(),'schemas':list(p.GetAppliedSchemas()),
        'attributes':attributes(p),'material_relationships':{str(r.GetName()):[str(x) for x in r.GetTargets()] for r in p.GetRelationships() if 'material' in str(r.GetName()).lower()}} for p in prims}
    manifest['articulation_roots']=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    wheel_bodies=[]
    for p in stage.Traverse():
        if 'wheel_link' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI):
            wheel_bodies.append(str(p.GetPath()))
            api=PhysxSchema.PhysxContactReportAPI.Apply(p)
            api.CreateThresholdAttr().Set(0.0)
    manifest['wheel_contact_report_bodies']=wheel_bodies
    wheel_views={p.split('/')[-1].replace('_wheel_link',''): world.scene.add(SingleRigidPrim(
        prim_path=p,name='forensic_'+p.split('/')[-1],reset_xform_properties=False)) for p in wheel_bodies}
    if len(wheel_bodies)!=4: raise RuntimeError(f'expected four wheel rigid bodies; got {wheel_bodies}')
    world.reset()
    robot.set_world_pose(position=np.array([0.,0.,0.0635]))
    world.play()
    manifest['dof_names']=list(robot.dof_names)
    indices=[manifest['dof_names'].index(n) for n in NAMES]
    manifest['wheel_indices']=indices
    joint_map=dict(robot._articulation_view._metadata.joint_indices)
    manifest['articulation_joint_indices']=joint_map
    force_indices=[joint_map[n]+1 for n in NAMES]
    manifest['wheel_incoming_force_rows']=force_indices
    manifest['wheel_joint_prims']={str(p.GetPath()):attributes(p) for p in stage.Traverse()
        if any(str(p.GetPath()).endswith(n) for n in NAMES)}
    for _ in range(60): world.step(render=True)
    physics=get_physx_simulation_interface()
    capability={}; errors={}
    with (args.output/'samples.jsonl').open('w') as samples, (args.output/'contacts.jsonl').open('w') as contact_file:
        frame=0
        for wi,(command,seconds) in enumerate(manifest['windows']):
            if args.model=='current':
                for path in graphs: og.Controller.attribute(f'{path}/diff.inputs:angularVelocity').set(command)
            target_mag=abs(command)*0.54/(2*0.098)
            nominal=[target_mag,-target_mag,target_mag,-target_mag]
            for _ in range(round(seconds/DT)):
                if args.model=='official':
                    robot.apply_action(ArticulationAction(joint_velocities=np.array(nominal),joint_indices=np.array(indices)))
                world.step(render=True)
                frame+=1
                report_contacts(physics,frame,contact_file,errors)
                pos,q=robot.get_world_pose()
                if args.model=='current':
                    target=[float(v) for path in graphs for v in og.Controller.attribute(f'{path}/diff.outputs:velocityCommand').get()]
                else: target=nominal
                item={'frame':frame,'window':wi,'sim_time':frame*DT,'wall_monotonic':time.monotonic(),
                    'command_angular_z':command,'targets_rad_s':target,'actual_position_rad':np.asarray(robot.get_joint_positions(),dtype=float)[indices].tolist(),
                    'actual_velocity_rad_s':np.asarray(robot.get_joint_velocities(),dtype=float)[indices].tolist(),
                    'chassis_xyz':vec(pos),'chassis_yaw':yaw(q)}
                try: item['chassis_linear_velocity']=vec(robot.get_linear_velocity())
                except Exception as exc: capability['chassis_linear_velocity']='NOT_OBSERVABLE_WITH_CURRENT_API: '+repr(exc)
                try: item['chassis_angular_velocity']=vec(robot.get_angular_velocity())
                except Exception as exc: capability['chassis_angular_velocity']='NOT_OBSERVABLE_WITH_CURRENT_API: '+repr(exc)
                item['wheel_body_state']={}
                for wheel,view in wheel_views.items():
                    try:
                        body_pos,_=view.get_world_pose()
                        item['wheel_body_state'][wheel]={'origin_xyz':vec(body_pos),
                            'linear_velocity':vec(view.get_linear_velocity()),
                            'angular_velocity':vec(view.get_angular_velocity())}
                    except Exception as exc:
                        capability['wheel_body_state']='NOT_OBSERVABLE_WITH_CURRENT_API: '+repr(exc)
                item.update(read_forces(robot,indices,force_indices,capability))
                samples.write(json.dumps(item)+'\n')
    manifest['capability']=capability; manifest['errors']=errors
    manifest['status']='CAPTURED'
    world.stop()
except BaseException as exc:
    manifest['error']=repr(exc)
    raise
finally:
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    app.close()
