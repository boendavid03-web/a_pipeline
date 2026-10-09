import json, math, hashlib, time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
B=Path('/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_teleop_velocity_20261009T042935Z')
def load(path): return [json.loads(s) for s in Path(path).read_text().splitlines()]
def sha(path):
 h=hashlib.sha256()
 with open(path,'rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def wrap(v): return math.atan2(math.sin(v),math.cos(v))
def stats(rows):
 le=np.array([r['linear_vector_error_mps'] for r in rows]); ae=np.array([r['angular_error_radps'] for r in rows])
 ll=np.array([max(.01,.05*abs(r['effective_command_v_mps'])) for r in rows]); al=np.array([max(.02,.05*abs(r['effective_command_omega_radps'])) for r in rows])
 return {'steps':len(rows),'linear_error_mean_mps':float(np.mean(le)),'linear_error_p95_mps':float(np.percentile(le,95)),'linear_error_max_mps':float(np.max(le)),'linear_over_limit_count':int(np.sum(le>ll)),'linear_over_limit_control_steps':[int(r['control_step']) for r,x,y in zip(rows,le,ll) if x>y],
 'angular_error_mean_radps':float(np.mean(ae)),'angular_error_p95_radps':float(np.percentile(ae,95)),'angular_error_max_radps':float(np.max(ae)),'angular_over_limit_count':int(np.sum(ae>al)),'angular_over_limit_control_steps':[int(r['control_step']) for r,x,y in zip(rows,ae,al) if x>y],
 'unique_any_over_limit_control_steps':sorted({int(r['control_step']) for r,x,y,z,q in zip(rows,le,ll,ae,al) if x>y or z>q}),
 'linear_all_step_pass':bool(np.all(le<=ll)),'angular_all_step_pass':bool(np.all(ae<=al))}
def byseg(rows):
 out={}
 for name in dict.fromkeys(r['segment'] for r in rows):
  rr=[r for r in rows if r['segment']==name]; ss=stats(rr)
  ss.update({'actual_forward_mean_mps':float(np.mean([r['actual_forward_v_mps'] for r in rr])),'actual_forward_p95_mps':float(np.percentile([r['actual_forward_v_mps'] for r in rr],95)),
             'actual_omega_mean_radps':float(np.mean([r['actual_omega_radps'] for r in rr])),'actual_omega_p95_radps':float(np.percentile([r['actual_omega_radps'] for r in rr],95)),
             'effective_v_mean_mps':float(np.mean([r['effective_command_v_mps'] for r in rr])),'effective_omega_mean_radps':float(np.mean([r['effective_command_omega_radps'] for r in rr])),
             'wall_blocked_steps':int(sum(bool(r['wall_blocked']) for r in rr)),'collision_flag_steps':int(sum(r['collision_flag'] is True for r in rr))})
  out[name]=ss
 return out
def cumulative(rows):
 use=[r for r in rows if not r['reset_marker']]
 dv=np.array([[r['actual_world_vx_mps']-r['expected_world_vx_mps'],r['actual_world_vy_mps']-r['expected_world_vy_mps']] for r in use])*np.array([r['dt_s'] for r in use])[:,None]
 da=np.array([wrap((r['post_physics_pose']['yaw']-r['pre_pose']['yaw'])-r['effective_command_omega_radps']*r['dt_s']) for r in use])
 # Drift across each commanded zero segment, excluding wrap/reset boundaries.
 zero=[]
 for name in ['01_stop','10_final_stop']:
  rr=[r for r in rows if r['segment']==name]
  p0=rr[0]['pre_pose']; p1=rr[-1]['post_physics_pose']
  zero.append({'segment':name,'net_xy_drift_m':math.hypot(p1['x']-p0['x'],p1['y']-p0['y']),'net_yaw_drift_rad':wrap(p1['yaw']-p0['yaw']),'max_step_linear_speed_mps':max(math.hypot(r['actual_world_vx_mps'],r['actual_world_vy_mps']) for r in rr)})
 # Endpoint of 150-step preset measured before the step-150 reset.
 fixed=[r for r in rows if r['control_step']<150]
 p0=fixed[0]['pre_pose']; p1=fixed[-1]['post_physics_pose']
 return {'nonreset_steps':len(use),'cumulative_world_displacement_error_vector_m':dv.sum(axis=0).tolist(),'cumulative_world_displacement_error_norm_m':float(np.linalg.norm(dv.sum(axis=0))),
         'cumulative_signed_heading_error_rad':float(da.sum()),'cumulative_absolute_heading_error_rad':float(np.abs(da).sum()),'zero_command_segments':zero,
         'fixed_150_step_actual_endpoint_drift_m':math.hypot(p1['x']-p0['x'],p1['y']-p0['y']),'fixed_150_step_actual_heading_change_rad':wrap(p1['yaw']-p0['yaw'])}
base=load(B/'telemetry.jsonl'); cand=load(D/'telemetry.jsonl')
assert len(base)==len(cand)==192
assert [r['control_step'] for r in base]==[r['control_step'] for r in cand]
assert all(abs(b['raw_command_v_mps']-c['raw_command_v_mps'])<1e-9 and abs(b['raw_command_omega_radps']-c['raw_command_omega_radps'])<1e-9 for b,c in zip(base,cand))
assert all(abs(b['effective_command_v_mps']-c['effective_command_v_mps'])<1e-7 and abs(b['effective_command_omega_radps']-c['effective_command_omega_radps'])<1e-7 for b,c in zip(base,cand))
assert [r['reset_marker'] for r in base]==[r['reset_marker'] for r in cand]
seg_base=byseg(base); seg_cand=byseg(cand)
comparison={'result':'BLOCKED','candidate_decision':'REJECTED; retain A-fixed baseline','criterion':'MOTION_REPLAY_PASS only if every one of 192 control steps meets both specified gates and retained subchecks pass. This candidate regressed and has remaining violations.',
 'protocol_match':{'steps_baseline':len(base),'steps_candidate':len(cand),'raw_action_sequence_exact_match':True,'effective_action_sequence_exact_match':True,'dt_s_unique_candidate':sorted(set(round(r['dt_s'],9) for r in cand)),'reset_markers_candidate':[int(r['control_step']) for r in cand if r['reset_marker']]},
 'baseline':{'directory':str(B),'telemetry_jsonl_sha256':sha(B/'telemetry.jsonl'),'overall':stats(base),'segments':seg_base,'cumulative_and_drift':cumulative(base)},
 'candidate':{'directory':str(D),'telemetry_jsonl_sha256':sha(D/'telemetry.jsonl'),'overall':stats(cand),'segments':seg_cand,'cumulative_and_drift':cumulative(cand)},
 'delta_candidate_minus_baseline':{'linear_over_limit_steps':stats(cand)['linear_over_limit_count']-stats(base)['linear_over_limit_count'],'angular_over_limit_steps':stats(cand)['angular_over_limit_count']-stats(base)['angular_over_limit_count'],'unique_any_over_limit_steps':len(stats(cand)['unique_any_over_limit_control_steps'])-len(stats(base)['unique_any_over_limit_control_steps']),
  'by_segment':{k:{'linear_over_limit_delta':seg_cand[k]['linear_over_limit_count']-seg_base[k]['linear_over_limit_count'],'angular_over_limit_delta':seg_cand[k]['angular_over_limit_count']-seg_base[k]['angular_over_limit_count'],'actual_forward_mean_delta_mps':seg_cand[k]['actual_forward_mean_mps']-seg_base[k]['actual_forward_mean_mps'],'actual_omega_mean_delta_radps':seg_cand[k]['actual_omega_mean_radps']-seg_base[k]['actual_omega_mean_radps']} for k in seg_base}},
 'key_history_steps':{str(n):{'control_step':n+150,'baseline':next(r for r in base if r['control_step']==n+150),'candidate':next(r for r in cand if r['control_step']==n+150)} for n in [13,15,30,34,41]},
 'state_identity':{'candidate_initial_state':json.loads((D/'INITIAL_STATE.json').read_text()),'candidate_reset_state':json.loads((D/'RESET_STATE.json').read_text()),'baseline_initial_pre_pose':base[0]['pre_pose'],'candidate_initial_pre_pose':cand[0]['pre_pose'],'baseline_pre_reset_endpoint':base[149]['post_physics_pose'],'candidate_pre_reset_endpoint':cand[149]['post_physics_pose'],'baseline_reset_pre_control_pose':base[150]['pre_pose'],'candidate_reset_pre_control_pose':cand[150]['pre_pose']},
 'evidence_limitations':['Contact objects, contact positions, forces and impulses were UNAVAILABLE because no contact sensor/force collection exists and none was enabled.','wall_blocked and collision_flag remain software flags, not proof of no physical contact.','Runtime PhysX read-only snapshots have 0.01 s physics dt; positions/velocities/targets in SI/radians as named; forces in N m; impulses UNAVAILABLE. Captures after each existing substep with no added steps.'],
 'mechanism_evidence':'The active yaw-only pose overwrite was the candidate mechanism. Runtime records confirm left/right wheel stiffness and damping are zero, caster_base stiffness/damping 5729.578/572.958, wheel joints have zero target during the watched samples, but this does not prove these values caused the mismatch. Preserving full orientation increased rather than reduced gate failures in this replay; causal attribution remains unproven.'}
(D/'COMPARISON.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
# before/after overlays
x=np.arange(192); seg=[r['segment'] for r in base]
for fname,title,key_cmd,key_act,ylabel in [('velocity_comparison.png','Forward speed: baseline vs candidate','effective_command_v_mps','actual_forward_v_mps','m/s'),('angular_velocity_comparison.png','Angular speed: baseline vs candidate','effective_command_omega_radps','actual_omega_radps','rad/s')]:
 fig,ax=plt.subplots(figsize=(14,4.5))
 ax.plot(x,[r[key_cmd] for r in base],color='black',lw=1.2,label='effective command')
 ax.plot(x,[r[key_act] for r in base],color='#2a6fbb',lw=1,label='A-fixed baseline measured')
 ax.plot(x,[r[key_act] for r in cand],color='#e07a1f',lw=1,label='candidate measured')
 ax.set(title=title,xlabel='Control step',ylabel=ylabel);ax.grid(alpha=.3);ax.legend(ncol=3)
 for i in range(1,192):
  if seg[i]!=seg[i-1]: ax.axvline(i-.5,color='gray',lw=.5,alpha=.45)
 fig.tight_layout();fig.savefig(D/fname,dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(14,4.5))
for rs,color,label in [(base,'#2a6fbb','A-fixed baseline'),(cand,'#e07a1f','candidate')]:
 err=np.array([r['linear_vector_error_mps'] for r in rs]); lim=np.array([max(.01,.05*abs(r['effective_command_v_mps'])) for r in rs]); bad=err>lim
 ax.plot(x,err,color=color,lw=1,label=label+' error'); ax.scatter(x[bad],err[bad],color=color,marker='x',s=28,label=label+' over limit')
ax.plot(x,[max(.01,.05*abs(r['effective_command_v_mps'])) for r in cand],color='black',lw=1,ls='--',label='per-step limit')
ax.set(title='World linear velocity vector error per step',xlabel='Control step',ylabel='m/s');ax.grid(alpha=.3);ax.legend(ncol=2)
for i in range(1,192):
 if seg[i]!=seg[i-1]:ax.axvline(i-.5,color='gray',lw=.5,alpha=.45)
fig.tight_layout();fig.savefig(D/'velocity_error.png',dpi=160);plt.close(fig)
# end source identity and hash verification
base_src=Path('/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_tilt_motion_20261008T173940Z/source')
src=D/'source'; changed=[]
for p in sorted(src.rglob('*')):
 if p.is_file() and '.git' not in p.parts:
  rel=p.relative_to(src)
  bp=base_src/rel
  if not bp.exists() or sha(p)!=sha(bp): changed.append({'path':str(rel),'candidate_sha256':sha(p),'baseline_sha256':sha(bp) if bp.exists() else None})
end={'source':str(src.resolve()),'source_file_count':sum(1 for p in src.rglob('*') if p.is_file() and '.git' not in p.parts),'source_changes_vs_A_fixed':changed,'runtime_exit_code':int((D/'EXIT_CODE.txt').read_text()),'ended_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'loaded_source_path_from_run':json.loads((D/'IDENTITY_START.json').read_text())['source'],'runtime_identity_start_sha256':sha(D/'IDENTITY_START.json'),'robot_usd':json.loads((D/'IDENTITY_START.json').read_text())['robot_usd'],'warehouse_usd':json.loads((D/'IDENTITY_START.json').read_text())['warehouse_usd'],'map_yaml':json.loads((D/'IDENTITY_START.json').read_text())['map_yaml'],'history_actions':json.loads((D/'IDENTITY_START.json').read_text())['history_actions']}
(D/'IDENTITY_END.json').write_text(json.dumps(end,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'baseline':comparison['baseline']['overall'],'candidate':comparison['candidate']['overall'],'delta':comparison['delta_candidate_minus_baseline'],'identity_changes':changed,'cumulative':comparison['candidate']['cumulative_and_drift']},ensure_ascii=False,indent=2))
