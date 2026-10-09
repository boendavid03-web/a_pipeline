import csv, json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D=Path(__file__).resolve().parent
rows=[json.loads(s) for s in (D/'telemetry.jsonl').read_text().splitlines()]
def p95(a): return float(np.percentile(np.asarray(a,dtype=float),95)) if len(a) else None
segments=[]
for name in dict.fromkeys(r['segment'] for r in rows):
    rr=[r for r in rows if r['segment']==name]
    le=np.array([r['linear_vector_error_mps'] for r in rr]); ae=np.array([r['angular_error_radps'] for r in rr])
    llim=np.array([max(.01,.05*abs(r['effective_command_v_mps'])) for r in rr])
    alim=np.array([max(.02,.05*abs(r['effective_command_omega_radps'])) for r in rr])
    lpass=le<=llim; apass=ae<=alim
    segments.append({'segment':name,'steps':len(rr),'command_v_mean_mps':float(np.mean([r['effective_command_v_mps'] for r in rr])),
      'command_omega_mean_radps':float(np.mean([r['effective_command_omega_radps'] for r in rr])),
      'actual_forward_mean_mps':float(np.mean([r['actual_forward_v_mps'] for r in rr])),
      'actual_forward_p95_mps':p95([r['actual_forward_v_mps'] for r in rr]),'actual_omega_mean_radps':float(np.mean([r['actual_omega_radps'] for r in rr])),
      'actual_omega_p95_radps':p95([r['actual_omega_radps'] for r in rr]),'linear_error_mean_mps':float(le.mean()),'linear_error_p95_mps':p95(le),
      'linear_error_max_mps':float(le.max()),'linear_limit_mean_mps':float(llim.mean()),'linear_over_limit_steps':int((~lpass).sum()),
      'linear_step_pass':bool(lpass.all()),'linear_segment_mean_pass':bool(le.mean()<=llim.mean()),
      'angular_error_mean_radps':float(ae.mean()),'angular_error_p95_radps':p95(ae),'angular_error_max_radps':float(ae.max()),
      'angular_limit_mean_radps':float(alim.mean()),'angular_over_limit_steps':int((~apass).sum()),'angular_step_pass':bool(apass.all()),
      'angular_segment_mean_pass':bool(ae.mean()<=alim.mean()),'wall_blocked_steps':int(sum(r['wall_blocked'] for r in rr)),
      'collision_flag_steps':int(sum(r['collision_flag'] is True for r in rr)),
      'steps_over_any_limit':[r['history_step'] if name.startswith('11_') else r['control_step'] for r,l,a in zip(rr,lpass,apass) if not(l and a)]})

def phase_components(r):
    pre=r['pre_pose']; wr=r['root_write_pose']; post=r['post_physics_pose']; dt=r['dt_s']; v=r['effective_command_v_mps']; w=r['effective_command_omega_radps']
    yawmid=pre['yaw']+.5*w*dt; exp=np.array([v*math.cos(yawmid),v*math.sin(yawmid)])
    k=(np.array([wr['x']-pre['x'],wr['y']-pre['y']])/dt)-exp
    ph=np.array([post['x']-wr['x'],post['y']-wr['y']])/dt
    kw=math.atan2(math.sin(wr['yaw']-pre['yaw']),math.cos(wr['yaw']-pre['yaw']))/dt-w
    pw=math.atan2(math.sin(post['yaw']-wr['yaw']),math.cos(post['yaw']-wr['yaw']))/dt
    return {'prewrite_linear_error_mps':float(np.linalg.norm(k)),'physics_displacement_speed_mps':float(np.linalg.norm(ph)),
      'prewrite_angular_error_radps':abs(kw),'physics_angular_increment_radps':abs(pw)}
hist=[r for r in rows if r['segment']=='11_history_robot7_replay']
history_steps=[]
for n in (13,16,30,31):
    # Prior RESULT.json labels action rows with zero-based control_step values.
    r=next(x for x in hist if x['history_step']==n+1); c=phase_components(r)
    history_steps.append({'history_step_zero_based':n,'history_step_one_based':n+1,'effective_command':[r['effective_command_v_mps'],r['effective_command_omega_radps']],
      'actual_forward_mps':r['actual_forward_v_mps'],'actual_omega_radps':r['actual_omega_radps'],
      'linear_error_mps':r['linear_vector_error_mps'],'linear_limit_mps':max(.01,.05*abs(r['effective_command_v_mps'])),
      'angular_error_radps':r['angular_error_radps'],'angular_limit_radps':max(.02,.05*abs(r['effective_command_omega_radps'])),
      'linear_pass':r['linear_vector_error_mps']<=max(.01,.05*abs(r['effective_command_v_mps'])),'angular_pass':r['angular_error_radps']<=max(.02,.05*abs(r['effective_command_omega_radps'])),
      'wall_blocked':r['wall_blocked'],'collision_flag':r['collision_flag'],'reset_marker':r['reset_marker'],**c})
zero=[r for r in rows if abs(r['effective_command_v_mps'])<1e-8 and abs(r['effective_command_omega_radps'])<1e-8]
zero_move=[{'control_step':r['control_step'],'segment':r['segment'],'actual_linear_speed_mps':math.hypot(r['actual_world_vx_mps'],r['actual_world_vy_mps']),'actual_omega_radps':r['actual_omega_radps']} for r in zero if math.hypot(r['actual_world_vx_mps'],r['actual_world_vy_mps'])>.01 or abs(r['actual_omega_radps'])>.02]
outliers=[{'control_step':r['control_step'],'segment':r['segment'],'history_step':r['history_step'],'linear_error_mps':r['linear_vector_error_mps'],'linear_limit_mps':max(.01,.05*abs(r['effective_command_v_mps'])),'angular_error_radps':r['angular_error_radps'],'angular_limit_radps':max(.02,.05*abs(r['effective_command_omega_radps'])),**phase_components(r)} for r in rows if r['linear_vector_error_mps']>max(.01,.05*abs(r['effective_command_v_mps'])) or r['angular_error_radps']>max(.02,.05*abs(r['effective_command_omega_radps']))]
summary={'protocol':'fixed script; one Isaac process; zero people, one robot; dt 0.04 s','steps':len(rows),'resets_marked':[r['control_step'] for r in rows if r['reset_marker']],
 'step_gate':'linear <= max(0.01 m/s, 5% of |effective v|); angular <= max(0.02 rad/s, 5% of |effective omega|)',
 'all_step_linear_pass':not any(o['linear_error_mps']>o['linear_limit_mps'] for o in outliers),'all_step_angular_pass':not any(o['angular_error_radps']>o['angular_limit_radps'] for o in outliers),
 'zero_command_steps':len(zero),'zero_command_motion_over_threshold':zero_move,'segments':segments,'step_outliers':outliers,'historical_robot7_steps_13_16_30_31':history_steps,
 'history_overall':{'steps':len(hist),'linear_over_limit_steps':sum(r['linear_vector_error_mps']>max(.01,.05*abs(r['effective_command_v_mps'])) for r in hist),'angular_over_limit_steps':sum(r['angular_error_radps']>max(.02,.05*abs(r['effective_command_omega_radps'])) for r in hist),'max_linear_error_mps':max(r['linear_vector_error_mps'] for r in hist),'max_angular_error_radps':max(r['angular_error_radps'] for r in hist)},
 'history_action_sequence_exact_match':all(abs(float(hist[i]['effective_command_v_mps'])-float(json.loads((D.parent/'navisaaclab_motion_range_20261009T013250Z/REPLAY_ACTIONS.json').read_text())['replayed_commands'][i][0]))<1e-7 and abs(float(hist[i]['effective_command_omega_radps'])-float(json.loads((D.parent/'navisaaclab_motion_range_20261009T013250Z/REPLAY_ACTIONS.json').read_text())['replayed_commands'][i][1]))<1e-7 for i in range(42)),
 'contact_evidence':'No contact sensor/impulse data recorded; wall_blocked and navigation collision flags are only their named software signals.'}
(D/'SUMMARY.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

x=np.arange(len(rows)); cmdv=np.array([r['effective_command_v_mps'] for r in rows]); actv=np.array([r['actual_forward_v_mps'] for r in rows]); cmdw=np.array([r['effective_command_omega_radps'] for r in rows]); actw=np.array([r['actual_omega_radps'] for r in rows]); seg=np.array([r['segment'] for r in rows])
for fn,title,c,a,ylabel in [('velocity_comparison.png','Commanded and measured forward velocity',cmdv,actv,'m/s'),('angular_velocity_comparison.png','Commanded and measured angular velocity',cmdw,actw,'rad/s')]:
 fig,ax=plt.subplots(figsize=(13,4)); ax.plot(x,c,label='effective command',lw=1.5); ax.plot(x,a,label='measured',lw=1.1); ax.set(title=title,xlabel='Control step',ylabel=ylabel); ax.grid(alpha=.3); ax.legend();
 for i in range(1,len(seg)):
  if seg[i]!=seg[i-1]: ax.axvline(i-.5,color='gray',lw=.5,alpha=.6)
 fig.tight_layout();fig.savefig(D/fn,dpi=150);plt.close(fig)
linerr=np.array([r['linear_vector_error_mps'] for r in rows]); lim=np.array([max(.01,.05*abs(r['effective_command_v_mps'])) for r in rows]); bad=linerr>lim
fig,ax=plt.subplots(figsize=(13,4));ax.plot(x,linerr,label='world-vector linear error',lw=1.1);ax.plot(x,lim,label='per-step limit',lw=1,ls='--');ax.scatter(x[bad],linerr[bad],color='red',label='over limit',s=20,zorder=3);ax.set(title='Per-control-step linear velocity vector error',xlabel='Control step',ylabel='m/s');ax.grid(alpha=.3);ax.legend();
for i in range(1,len(seg)):
 if seg[i]!=seg[i-1]:ax.axvline(i-.5,color='gray',lw=.5,alpha=.6)
fig.tight_layout();fig.savefig(D/'velocity_error.png',dpi=150);plt.close(fig)
