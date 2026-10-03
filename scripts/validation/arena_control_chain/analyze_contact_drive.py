#!/usr/bin/env python3
"""Summarize raw per-step Jackal contact/drive forensic captures."""
import argparse
import collections
import json
import math
import statistics
from pathlib import Path

p=argparse.ArgumentParser(); p.add_argument('raw',type=Path); p.add_argument('--output',type=Path,required=True); args=p.parse_args()
samples=[json.loads(line) for line in (args.raw/'samples.jsonl').open()]
contacts=[json.loads(line) for line in (args.raw/'contacts.jsonl').open()]
manifest=json.loads((args.raw/'manifest.json').read_text())
sample_by_frame={x['frame']:x for x in samples}
if manifest['status']!='CAPTURED': raise SystemExit('capture incomplete')
wheels=['front_left','front_right','rear_left','rear_right']; signs=[1,-1,1,-1]
byframe=collections.defaultdict(lambda:{w:{'reports':0,'points':0,'anchors':0,'normal_impulse':0.,'friction_impulse_xy':0.,'friction_x':0.,'friction_y':0.,'separation_min':None,'relative_contact_xy_speed':[],'relative_contact_lateral_y':[]} for w in wheels})
for item in contacts:
    path=item['collider0']+item['collider1']
    match=[w for w in wheels if w+'_wheel' in path]
    if len(match)!=1: continue
    w=match[0]; out=byframe[item['frame']][w]; out['reports']+=1
    out['points']+=len(item['points']);out['anchors']+=len(item['friction_anchors'])
    for point in item['points']:
        out['normal_impulse']+=sum(a*b for a,b in zip(point['impulse'],point['normal']))
        sep=point['separation'];out['separation_min']=sep if out['separation_min'] is None else min(sep,out['separation_min'])
        state=sample_by_frame.get(item['frame'],{}).get('wheel_body_state',{}).get(w)
        if state:
            px,py,pz=point['position']; ox,oy,oz=state['origin_xyz']
            vx,vy,vz=state['linear_velocity']; wx,wy,wz=state['angular_velocity']
            cx=vx+wy*(pz-oz)-wz*(py-oy)
            cy=vy+wz*(px-ox)-wx*(pz-oz)
            out['relative_contact_xy_speed'].append(math.hypot(cx,cy))
            out['relative_contact_lateral_y'].append(cy)
    for anchor in item['friction_anchors']:
        x,y,_=anchor['impulse'];out['friction_x']+=x;out['friction_y']+=y
        out['friction_impulse_xy']+=math.hypot(x,y)
anchor_yaw_by_frame=collections.defaultdict(lambda:{'lateral':0.,'longitudinal':0.})
for item in contacts:
    frame=item['frame']; sample=sample_by_frame.get(frame)
    if not sample: continue
    cx,cy=sample['chassis_xyz'][:2]
    for anchor in item['friction_anchors']:
        px,py,_=anchor['position'];jx,jy,_=anchor['impulse']
        anchor_yaw_by_frame[frame]['lateral']+=(px-cx)*jy
        anchor_yaw_by_frame[frame]['longitudinal']-=(py-cy)*jx
result={'model':manifest['model'],'sample_count':len(samples),'contact_report_count':len(contacts),'windows':{},'spikes':[]}
for wi in [0,2]:
    z=[s for s in samples if s['window']==wi]; start,end=z[0],z[-1]
    target=abs(start['targets_rad_s'][0]);dur=end['sim_time']-start['sim_time']
    displacement=[end['actual_position_rad'][j]-start['actual_position_rad'][j] for j in range(4)]
    wheel_yaw=.098/.54*(displacement[1]+displacement[3]-displacement[0]-displacement[2])/2
    chassis_yaw=end['chassis_yaw']-start['chassis_yaw']
    d={'command':start['command_angular_z'],'duration':dur,'target_rad_s':start['targets_rad_s'],
       'wheel_displacement_rad':displacement,'mean_wheel_velocity_rad_s':[v/dur for v in displacement],
       'chassis_yaw_rad':chassis_yaw,'wheel_model_yaw_rad':wheel_yaw,
       'chassis_to_wheel_yaw_ratio':chassis_yaw/wheel_yaw if wheel_yaw else None,
       'median_relative_wheel_velocity_error':statistics.median(abs(s['actual_velocity_rad_s'][j]-s['targets_rad_s'][j])/target for s in z for j in range(4)),
       'reverse_velocity_fraction':sum(s['actual_velocity_rad_s'][j]*signs[j]<0 for s in z for j in range(4))/(len(z)*4),
       'peak_speed_over_target':max(abs(v) for s in z for v in s['actual_velocity_rad_s'])/target,
       'wheel_contacts':{},'friction_anchor_yaw_impulse_mean_Nms':{
           'lateral':statistics.mean(anchor_yaw_by_frame[s['frame']]['lateral'] for s in z),
           'longitudinal':statistics.mean(anchor_yaw_by_frame[s['frame']]['longitudinal'] for s in z),
           'net':statistics.mean(sum(anchor_yaw_by_frame[s['frame']].values()) for s in z)},
       'joint_effort_abs_median':statistics.median(abs(v) for s in z for v in s['measured_joint_effort']),
       'joint_effort_abs_max':max(abs(v) for s in z for v in s['measured_joint_effort'])}
    for j,w in enumerate(wheels):
        c=[byframe[s['frame']][w] for s in z]
        d['wheel_contacts'][w]={'frames_with_points':sum(t['points']>0 for t in c),
            'frames_without_points':sum(t['points']==0 for t in c),
            'median_points':statistics.median(t['points'] for t in c),
            'median_normal_impulse_Ns':statistics.median(t['normal_impulse'] for t in c),
            'median_friction_anchor_impulse_xy_Ns':statistics.median(t['friction_impulse_xy'] for t in c),
            'mean_friction_anchor_y_Ns':statistics.mean(t['friction_y'] for t in c),
            'median_abs_joint_effort_Nm':statistics.median(abs(s['measured_joint_effort'][j]) for s in z),
            'peak_abs_wheel_velocity_rad_s':max(abs(s['actual_velocity_rad_s'][j]) for s in z),
            'median_abs_post_step_contact_xy_speed_m_s':statistics.median(abs(v) for t in c for v in t['relative_contact_xy_speed']) if any(t['relative_contact_xy_speed'] for t in c) else None,
            'mean_post_step_lateral_contact_velocity_y_m_s':statistics.mean(v for t in c for v in t['relative_contact_lateral_y']) if any(t['relative_contact_lateral_y'] for t in c) else None}
    result['windows'][str(wi)]=d
    events=[]
    for s in z:
        for j,w in enumerate(wheels):
            v=s['actual_velocity_rad_s'][j]
            if abs(v)>3*target:
                c=byframe[s['frame']][w]
                prev=byframe[s['frame']-1][w]
                events.append({'frame':s['frame'],'sim_time':s['sim_time'],'wheel':w,'velocity':v,
                    'target':s['targets_rad_s'][j],'joint_effort':s['measured_joint_effort'][j],
                    'contact_points':c['points'],'prev_contact_points':prev['points'],
                    'normal_impulse_Ns':c['normal_impulse'],'friction_anchor_impulse_xy_Ns':c['friction_impulse_xy'],
                    'chassis_yaw_rate':s.get('chassis_angular_velocity',[None,None,None])[2],
                    'median_abs_post_step_contact_xy_speed_m_s':statistics.median(c['relative_contact_xy_speed']) if c['relative_contact_xy_speed'] else None})
    d['spike_event_count']=len(events)
    d['spikes_with_contact']=sum(e['contact_points']>0 for e in events)
    d['spikes_with_prev_contact']=sum(e['prev_contact_points']>0 for e in events)
    d['spikes_no_contact']=sum(e['contact_points']==0 for e in events)
    result['spikes']+=sorted(events,key=lambda e:abs(e['velocity']),reverse=True)[:10]
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'model':result['model'],'windows':result['windows'],'top_spikes':result['spikes'][:5]},indent=2))
