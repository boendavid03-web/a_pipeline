#!/usr/bin/env python3
"""Bounded velocity telemetry through the A-fixed CrowdSim action/drive path."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, random, sys, time
from pathlib import Path
sys.dont_write_bytecode = True
SOURCE = Path('/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_tilt_motion_20261008T173940Z/source').resolve()
ROOT = Path('/home/user/workspace/NavIsaaclab2.0')
OUT = Path(__file__).resolve().parent
HIST = ROOT/'Assets/repro_logs/navisaaclab_motion_range_20261009T013250Z/REPLAY_ACTIONS.json'
HIST_XY = [8.274999618530273, 13.52500057220459]
HIST_YAW = -2.903446912765503

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()
def cpu(x): return x.detach().cpu().numpy() if hasattr(x,'detach') else x
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def quat_rpy(q):
    w,x,y,z=[float(v) for v in q]
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return [roll,pitch,yaw]
def vec_pose(robot):
    p=cpu(robot.data.root_pos_w[0]).tolist(); q=cpu(robot.data.root_quat_w[0]).tolist()
    r,pitch,y=quat_rpy(q)
    return {'x':float(p[0]),'y':float(p[1]),'z':float(p[2]),'roll':r,'pitch':pitch,'yaw':y,'quat_wxyz':q}
def save(p,obj): p.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--headless',action='store_true')
    ap.add_argument('--seed',type=int,default=703)
    a=ap.parse_args()
    if Path.cwd().resolve()!=SOURCE: raise RuntimeError('A_FIXED_SOURCE_ROOT_MISMATCH')
    random.seed(a.seed)
    from CrowdSim.crowd_sim import cfg_path,load_config
    from CrowdSim.world.builder import build_env
    from CrowdSim.protomotions_runtime import shutdown_runtime
    import numpy as np, torch
    random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed); torch.cuda.manual_seed_all(a.seed)
    cfg=load_config(cfg_path('CrowdSim/config/env.yaml'))
    if cfg['scene']!='warehouse': raise RuntimeError('SCENE_NOT_WAREHOUSE')
    cfg['car']['usd']='/home/user/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd'
    cfg['humanoid']['checkpoint']='/home/user/workspace/NavIsaaclab2.0/NavIsaaclab2.0-main/data/pretrained_models/masked_mimic/smpl_57f98a9/last.ckpt'
    cfg['humanoid']['motion_file']='/home/user/workspace/NavIsaaclab2.0/Assets/motion/amass_smpl_validation.pt'
    sc=load_config(cfg_path('CrowdSim/config/scenes/warehouse.yaml'))
    sc['scene_usd']='/home/user/isaacsim_assets/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd'
    sc['scene_map']=str(SOURCE/'CrowdSim/maps/newmap.yaml'); cfg['scene']=sc
    nav=cfg['navigation']; nav['num_humanoids']=0; nav['num_robots']=1; nav['path']['seed']=a.seed
    nav['rl']['goal_curriculum']['enabled']=False; nav['recording']['enabled']=False; nav['recording']['output_dir']=str(OUT/'navigation')
    cfg['humanoid']['human_mesh']=False; cfg['humanoid']['state_recording']['auto_record']=False
    cfg['sensors']['camera']['auto_record']=False
    save(OUT/'CONFIG.json',cfg)
    manifest={'source':str(SOURCE),'source_files':{p.relative_to(SOURCE).as_posix():sha(p) for p in sorted(SOURCE.rglob('*')) if p.is_file() and '.git' not in p.parts},
      'robot_usd':{'path':cfg['car']['usd'],'sha256':sha(cfg['car']['usd'])},'warehouse_usd':{'path':sc['scene_usd'],'sha256':sha(sc['scene_usd'])},
      'map_yaml':{'path':sc['scene_map'],'sha256':sha(sc['scene_map'])},'history_actions':{'path':str(HIST),'sha256':sha(HIST)},'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'headless':a.headless}
    save(OUT/'IDENTITY_START.json',manifest)
    h=json.loads(HIST.read_text()); actions=h['replayed_commands']
    if len(actions)!=42: raise RuntimeError('HISTORICAL_ACTION_COUNT_MISMATCH')
    requested=[]
    fixed=[('01_stop',(0,0)),('02_slow_straight',(0.2,0)),('03_medium_straight',(0.5,0)),('04_fast_straight',(1,0)),
      ('05_left_turn',(0,0.5)),('06_right_turn',(0,-0.5)),('07_forward_left',(0.5,0.5)),('08_forward_right',(0.5,-0.5))]
    for name,cmd in fixed: requested.extend([{'segment':name,'v':cmd[0],'omega':cmd[1],'reset_marker':False} for _ in range(15)])
    variable=[(0.1,0),(0.3,0.2),(0.6,0.4),(1,0.5),(0.7,0),(0.4,-0.3),(0.8,-0.5),(0.2,-0.5),(0.5,0),(0.9,0.3),(0.3,0.5),(0.6,-0.2),(0.1,-0.4),(0.5,0.5),(0,0)]
    requested.extend([{'segment':'09_continuous_variable','v':v,'omega':w,'reset_marker':False} for v,w in variable])
    requested.extend([{'segment':'10_final_stop','v':0.,'omega':0.,'reset_marker':False} for _ in range(15)])
    requested.extend([{'segment':'11_history_robot7_replay','v':float(v),'omega':float(w),'history_step':i+1,'reset_marker':i==0} for i,(v,w) in enumerate(actions)])
    save(OUT/'PRESET_SCRIPT.json',{'source':'user-specified sequence plus REPLAY_ACTIONS.json','control_period_count':len(requested),'control_period_dt_expected_s':0.04,'stages':fixed+[('09_continuous_variable','15 fixed varying commands'),('10_final_stop','15 periods'),('11_history_robot7_replay','42 exact stored actions')],'commands':requested})
    runtime=manager=None
    try:
      result=build_env(cfg,num_envs=1,headless=a.headless)
      if result is None: raise RuntimeError('BUILD_ENV_NONE')
      env,agent,manager,runtime=result
      if manager.config.num_humanoids!=0 or manager.config.num_robots!=1: raise RuntimeError('RUNTIME_COUNTS_MISMATCH')
      agent.eval(); app=env.simulator._simulation_app
      # Save the legal free spawn generated by the original warehouse/map sampler.
      base_xy=manager.starts_xy[0].copy(); base_yaw=float(manager._robot_spawn_yaws[0])
      # Put robot 7's historical spawn in place before its first action block reset.
      manager.starts_xy[0]=np.asarray(HIST_XY,dtype=np.float32); manager.task.starts_xy[0]=manager.starts_xy[0]
      manager._robot_spawn_yaws[0]=HIST_YAW
      env._crowdsim_robot_spawn_poses[0,:2]=torch.as_tensor(HIST_XY,dtype=torch.float32,device=manager.config.device)
      env._crowdsim_robot_spawn_poses[0,3:7]=manager._yaw_to_quat_tensor(torch.tensor([HIST_YAW],device=manager.config.device))[0]
      obs,_=env.reset()
      dt=float(manager._env_dt)
      if not (0.02<=dt<=0.1): raise RuntimeError(f'CONTROL_DT_UNEXPECTED:{dt}')
      robot=manager.robot; drive=manager.drive; wheel_ids=list(drive._wheel_joint_ids or [])
      if not wheel_ids: raise RuntimeError('WHEEL_JOINT_IDS_UNAVAILABLE')
      # Nonintrusive trace around the existing action-path pose write.
      context={'active':False,'before':None,'written':None}
      orig_write=robot.write_root_pose_to_sim
      def traced_write(poses,*args,**kwargs):
        if context['active']:
          context['before']=vec_pose(robot)
          p=cpu(poses[0]).tolist(); rr,pp,yy=quat_rpy(p[3:7])
          context['written']={'x':float(p[0]),'y':float(p[1]),'z':float(p[2]),'roll':rr,'pitch':pp,'yaw':yy,'quat_wxyz':[float(q) for q in p[3:7]]}
        return orig_write(poses,*args,**kwargs)
      robot.write_root_pose_to_sim=traced_write
      # The reset path uses CrowdSim's existing robot reset API and clears wheel state.
      def reset_history():
        manager.starts_xy[0]=np.asarray(HIST_XY,dtype=np.float32); manager.task.starts_xy[0]=manager.starts_xy[0]
        manager._robot_spawn_yaws[0]=HIST_YAW
        manager.reset_robot_rl_episodes(np.asarray([True]),repeat_mask=np.asarray([True]))
      # We begin at the target already; switch to base legal start for calibrated stages.
      manager.starts_xy[0]=base_xy; manager.task.starts_xy[0]=base_xy; manager._robot_spawn_yaws[0]=base_yaw
      env._crowdsim_robot_spawn_poses[0,:2]=torch.as_tensor(base_xy,dtype=torch.float32,device=manager.config.device)
      env._crowdsim_robot_spawn_poses[0,3:7]=manager._yaw_to_quat_tensor(torch.tensor([base_yaw],device=manager.config.device))[0]
      # Re-run the existing reset to establish the regular legal free spawn.
      obs,_=env.reset(torch.tensor([0],device=manager.config.device))
      # Force base location through reset API and then measure prescribed fixed sequence.
      manager.starts_xy[0]=base_xy; manager.task.starts_xy[0]=base_xy; manager._robot_spawn_yaws[0]=base_yaw
      manager.reset_robot_rl_episodes(np.asarray([True]),repeat_mask=np.asarray([True]))
      physx=robot.root_physx_view
      get_jv=getattr(physx,'get_dof_velocities',None); get_jt=getattr(physx,'get_dof_velocity_targets',None)
      get_rpy=lambda:vec_pose(robot)
      rows=[]; step=0; reset_next=False
      # Fixed commands first, from a map-sampled legal free area.
      for phase in ('fixed','history'):
        if phase=='history':
          reset_history(); reset_next=True
        sequence=requested if phase=='fixed' else requested[150:]
        if phase=='fixed': sequence=requested[:150]
        for entry in sequence:
          pre=vec_pose(robot); command=np.asarray([[entry['v']/manager.config.rl_max_linear_velocity,entry['omega']/manager.config.rl_max_angular_velocity]],dtype=np.float32)
          manager.set_robot_rl_actions(command)
          obs=agent.add_agent_info_to_obs(obs)
          with torch.no_grad():
            hres=agent.model(agent.obs_dict_to_tensordict(obs)); human_action=hres.get('mean_action',hres['action'])
          context.update(active=True,before=None,written=None)
          _,_,human_done,_,_=env.step(human_action)
          context['active']=False
          _,_,robot_done,info,*_=manager.get_robot_rl_feedback()
          post=vec_pose(robot); written=context['written']; write_before=context['before']
          if written is None: raise RuntimeError('DRIVE_ROOT_WRITE_NOT_CAPTURED')
          effective=drive._effective_commands[0].astype(float).tolist(); actual_vxy=(np.asarray([post['x']-pre['x'],post['y']-pre['y']],dtype=float)/dt)
          yawmid=pre['yaw']+0.5*effective[1]*dt; expected_vxy=np.asarray([effective[0]*math.cos(yawmid),effective[0]*math.sin(yawmid)])
          actual_forward=float(actual_vxy[0]*math.cos(pre['yaw'])+actual_vxy[1]*math.sin(pre['yaw']))
          actual_lateral=float(-actual_vxy[0]*math.sin(pre['yaw'])+actual_vxy[1]*math.cos(pre['yaw']))
          actual_w=wrap(post['yaw']-pre['yaw'])/dt
          jv=cpu(get_jv())[0,wheel_ids].astype(float).tolist() if get_jv is not None else None
          jt=cpu(get_jt())[0,wheel_ids].astype(float).tolist() if get_jt is not None else None
          collision=bool(info['collision'][0]) if 'collision' in info else None
          record={'control_step':step,'sim_time_s':(step+1)*dt,'dt_s':dt,'segment':entry['segment'],'segment_step':entry.get('history_step',1+(step%15)),
            'history_step':entry.get('history_step'),'reset_marker':bool(reset_next or entry.get('reset_marker',False)),
            'raw_command_v_mps':float(entry['v']),'raw_command_omega_radps':float(entry['omega']),
            'effective_command_v_mps':float(effective[0]),'effective_command_omega_radps':float(effective[1]),
            'wall_blocked':bool(drive.wall_blocked[0]),'collision_flag':collision,
            'pre_pose':pre,'pre_root_write_pose':write_before,'root_write_pose':written,'post_physics_pose':post,
            'actual_world_vx_mps':float(actual_vxy[0]),'actual_world_vy_mps':float(actual_vxy[1]),
            'actual_forward_v_mps':actual_forward,'actual_lateral_v_mps':actual_lateral,'actual_omega_radps':actual_w,
            'expected_world_vx_mps':float(expected_vxy[0]),'expected_world_vy_mps':float(expected_vxy[1]),
            'linear_vector_error_mps':float(np.linalg.norm(actual_vxy-expected_vxy)),
            'angular_error_radps':float(abs(actual_w-effective[1])),
            'wheel_joint_names':[robot.joint_names[i] for i in wheel_ids],'wheel_target_radps':jt,'wheel_actual_radps':jv,
            'wall_contact_proven':False,'contact_sensor_available':False}
          rows.append(record); reset_next=False; step+=1
          obs,_=env.reset(human_done.nonzero(as_tuple=False).squeeze(-1))
          if step%25==0: print('TELEMETRY_PROGRESS',step,flush=True)
      robot.write_root_pose_to_sim=orig_write
      with (OUT/'telemetry.jsonl').open('x') as f:
        for r in rows: f.write(json.dumps(r,allow_nan=False,separators=(',',':'))+'\n')
      fields=['control_step','sim_time_s','dt_s','segment','segment_step','history_step','reset_marker','raw_command_v_mps','raw_command_omega_radps','effective_command_v_mps','effective_command_omega_radps','wall_blocked','collision_flag','pre_x','pre_y','pre_yaw','write_x','write_y','write_yaw','post_x','post_y','post_yaw','roll_before','pitch_before','roll_after','pitch_after','actual_world_vx_mps','actual_world_vy_mps','actual_forward_v_mps','actual_lateral_v_mps','actual_omega_radps','expected_world_vx_mps','expected_world_vy_mps','linear_vector_error_mps','angular_error_radps','wheel_joint_names','wheel_target_radps','wheel_actual_radps']
      with (OUT/'telemetry.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in rows:
          b=r['pre_pose'];q=r['root_write_pose'];p=r['post_physics_pose']
          w.writerow({'control_step':r['control_step'],'sim_time_s':r['sim_time_s'],'dt_s':r['dt_s'],'segment':r['segment'],'segment_step':r['segment_step'],'history_step':r['history_step'],'reset_marker':r['reset_marker'],'raw_command_v_mps':r['raw_command_v_mps'],'raw_command_omega_radps':r['raw_command_omega_radps'],'effective_command_v_mps':r['effective_command_v_mps'],'effective_command_omega_radps':r['effective_command_omega_radps'],'wall_blocked':r['wall_blocked'],'collision_flag':r['collision_flag'],'pre_x':b['x'],'pre_y':b['y'],'pre_yaw':b['yaw'],'write_x':q['x'],'write_y':q['y'],'write_yaw':q['yaw'],'post_x':p['x'],'post_y':p['y'],'post_yaw':p['yaw'],'roll_before':b['roll'],'pitch_before':b['pitch'],'roll_after':p['roll'],'pitch_after':p['pitch'],'actual_world_vx_mps':r['actual_world_vx_mps'],'actual_world_vy_mps':r['actual_world_vy_mps'],'actual_forward_v_mps':r['actual_forward_v_mps'],'actual_lateral_v_mps':r['actual_lateral_v_mps'],'actual_omega_radps':r['actual_omega_radps'],'expected_world_vx_mps':r['expected_world_vx_mps'],'expected_world_vy_mps':r['expected_world_vy_mps'],'linear_vector_error_mps':r['linear_vector_error_mps'],'angular_error_radps':r['angular_error_radps'],'wheel_joint_names':json.dumps(r['wheel_joint_names']),'wheel_target_radps':json.dumps(r['wheel_target_radps']),'wheel_actual_radps':json.dumps(r['wheel_actual_radps'])})
      print('TELEMETRY_COMPLETE',step,'dt',dt,flush=True)
    finally:
      if runtime is not None: shutdown_runtime(runtime,manager)

if __name__=='__main__': main()
