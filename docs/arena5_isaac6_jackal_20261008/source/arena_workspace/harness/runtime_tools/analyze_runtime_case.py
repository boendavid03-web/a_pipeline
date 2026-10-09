"""Offline summary of preserved runtime observations. Does not launch processes."""
import collections,json,math,pathlib,statistics,sys

case=pathlib.Path(sys.argv[1]);run=case.parent.parent
log=case/'probe/stdout.log'
data=json.loads(log.read_text().split('A_LIFECYCLE_JSON ',1)[1]) if log.exists() and 'A_LIFECYCLE_JSON ' in log.read_text() else {}
if data:(case/'PROBE_RESULT.json').write_text(json.dumps(data))
trials=data.get('trials',[]);rows=data.get('rows',{})
result={'case':case.name,'trials':trials,'observer_error':data.get('error'),'row_counts':{k:len(v) for k,v in rows.items()},'safety':'NOT_EVALUATED','physx_contact':'UNVERIFIED'}
holds=[x.get('hold',{}).get('status','NOT_OBSERVED') for x in trials]
result['lifecycle']='PASS' if len(trials)==3 and holds==['PASS']*3 and not data.get('error') else ('STOP_FAIL' if 'FAIL' in holds else ('STOP_INCONCLUSIVE' if 'INCONCLUSIVE' in holds else 'INCOMPLETE'))
if any('moving_cancel' not in r['phase'] and (r.get('terminal') or {}).get('status')!=4 for r in trials):result['lifecycle']='TASK_FAIL'
result['moving_cancel']=[]
for item in data.get('moving_cancel_checks',[]):
 window=item.get('tf_motion_window',[])
 xy=lambda p:[p['msg']['transform']['translation'][k] for k in ['x','y']]
 result['moving_cancel'].append({'moving':item['moving'],'observed_cmd':item.get('cmd'),'tf_samples':len(window),'observed_displacement_m':math.dist(xy(window[0]),xy(window[-1])) if window else None})
events=[e for r in rows.get('collisions',[]) for e in r['msg'].get('events',[])]
result['footprint_2d']={'event_records':len(events),'kinds':dict(collections.Counter(str(e.get('type',e.get('kind','UNKNOWN'))) for e in events)),'meaning':'2D footprint events; not PhysX contact counts'}
trace=pathlib.Path('[LOCAL_PATH]')/case.name
result['trace_directory']=str(trace)
sdk=[]
for p in trace.glob('sdk_*.jsonl'):
 records=[json.loads(x) for x in p.read_text().splitlines()];obs=[x for x in records if x['kind']=='obs'];steps=[x for x in records if x['kind']=='step'];deltas=[b['sim']-a['sim'] for a,b in zip(obs,obs[1:]) if b['sim']>a['sim']]
 observations={r['seq']:r for r in obs};frame_errors=[];headings=[]
 for state in [r for r in records if r['kind']=='actual_policy_state']:
  f=observations[state['seq']]['features'];pose=f['robot_pose'];body=f['robot_state'];theta=pose[2];expected=[math.cos(theta)*body[2]-math.sin(theta)*body[3],math.sin(theta)*body[2]+math.cos(theta)*body[3]];robot=state['robot'];frame_errors.append(math.dist(expected,[robot['vx'],robot['vy']]));headings.append(theta)
 sdk.append({'file':str(p),'identity':records[0],'counts':dict(collections.Counter(x['kind'] for x in records)),'models_loaded':[x for x in records if x['kind']=='actual_model_load'],'obs_interval_median_sim_s':statistics.median(deltas) if deltas else None,'obs_rate_from_median_hz':1/statistics.median(deltas) if deltas else None,'obs_rate_over_observed_sim_span_hz':(len(obs)-1)/(obs[-1]['sim']-obs[0]['sim']) if len(obs)>1 and obs[-1]['sim']>obs[0]['sim'] else None,'neural_forwards':dict(sum((collections.Counter(x.get('neural_forwards',{})) for x in steps),collections.Counter())),'max_actual_humans':max((len(x['features'].get('pedestrians') or []) for x in obs),default=None),'actual_policy_frame':{'samples':len(frame_errors),'max_body_to_world_velocity_error_mps':max(frame_errors) if frame_errors else None,'max_abs_actual_heading_rad':max(map(abs,headings)) if headings else None}})
result['native_sdk']=sdk
edge=trace/'edge_publish.jsonl'
if edge.exists():
 records=[json.loads(r) for r in edge.read_text().splitlines()];errors=[];scale_violations=0;headings=[]
 for r in records:
  if r['action_type']!='omnidirectional':continue
  vx,vy=r['action'][:2];omega=r['action'][2] if len(r['action'])==3 else 0.;theta=r['robot_pose'][2];v=math.hypot(vx,vy);w=((math.atan2(vy,vx)-theta+math.pi)%(2*math.pi)-math.pi)/r['step_dt_s']+omega if v>=1e-6 else omega
  scales=[actual/expected for actual,expected in [(r['cmd']['linear']['x'],v),(r['cmd']['angular']['z'],w)] if abs(expected)>1e-8]
  if len(scales)==2:errors.append(abs(scales[0]-scales[1]))
  scale_violations+=int(any(s<-.000000001 or s>1.000000001 for s in scales));headings.append(theta)
 result['actual_edge_projection']={'samples':len(records),'max_scale_consistency_error':max(errors) if errors else None,'scale_outside_0_1_records':scale_violations,'max_abs_heading_rad':max(map(abs,headings)) if headings else None,'meaning':'Actual final cmd retains the single world-action projection curvature and uniform limit scale; no cached cmd samples'}
p=trace/'consumer.jsonl'
if p.exists():
 records=[json.loads(x) for x in p.read_text().splitlines()];consumed=[x for x in records if x['kind']=='consume'];perception=[x for x in records if x['kind']=='perception'];errors=[];distances=[];positions=[];velocities=[];headings=[];stamp=set()
 for r in consumed:
  stamp.add((r['header_stamp']['sec'],r['header_stamp']['nanosec']))
  for a in r['incoming']:
   external=r['external'].get(str(a['id']))
   if external and external['internal_id'] in r['pool']['ids']:
    i=r['pool']['ids'].index(external['internal_id']);positions.append(a['pose'][:2]);velocities.append(a['velocity']);headings.append(a['pose'][2]);errors.append({'position':math.dist(a['pose'][:2],r['pool']['positions'][i]),'velocity':math.dist(a['velocity'],r['pool']['velocities'][i]),'heading':abs(math.atan2(math.sin(a['pose'][2]-r['pool']['headings'][i]),math.cos(a['pose'][2]-r['pool']['headings'][i])))})
 for r in perception:
  robots=[i for i,k in enumerate(r['pool']['kinds']) if k==1];humans=[i for i,k in enumerate(r['pool']['kinds']) if k==0]
  distances.extend(math.dist(r['pool']['positions'][i],r['pool']['positions'][j]) for i in humans for j in robots)
 result['human_consumer']={'identity':records[0],'counts':dict(collections.Counter(x['kind'] for x in records)),'compared_robot_updates':len(errors),'distinct_source_stamps':len(stamp),'max_position_error_m':max((e['position'] for e in errors),default=None),'max_velocity_error_mps':max((e['velocity'] for e in errors),default=None),'max_heading_error_rad':max((e['heading'] for e in errors),default=None),'robot_position_span_m':max((math.dist(p,positions[0]) for p in positions),default=None),'max_robot_speed_mps':max((math.hypot(*v) for v in velocities),default=None),'heading_span_rad':max(headings)-min(headings) if headings else None,'robot_neighbor_rows':sum(any(bool(v) for v in r['neighbors'].values()) for r in perception),'min_robot_human_distance_m':min(distances) if distances else None,'planners':sorted(set(r['planner'] for r in records if r['kind']=='planner'))}
usd=collections.defaultdict(list);discrepancies=[]
for r in rows.get('usd',[]):
 incoming={('/World/'+p['name'].lstrip('/')):p for p in r['requested_input']['pedestrians']}
 for prim in r['usd']['prims']:
  pos=prim['pose']['position'];point=[pos['x'],pos['y']];usd[(r['phase'],prim['name'])].append(point)
  if prim['name'] in incoming:
   a=incoming[prim['name']]['pose']['position'];discrepancies.append(math.dist(point,[a['x'],a['y']]))
result['isaac_human_usd']={'responses':len(rows.get('usd',[])),'max_async_pose_difference_m':max(discrepancies) if discrepancies else None,'phase_motion':[{'phase':key[0],'prim':key[1],'samples':len(points),'max_displacement_m':max(math.dist(p,points[0]) for p in points)} for key,points in usd.items()]}
result['actual_endpoints']=data.get('actual_endpoints',{})
if (case/'driver_error.txt').exists():result['driver_error']=(case/'driver_error.txt').read_text()
if (case/'CONFIG_ONLY_restored.json').exists():result['restored_source_hashes']=json.loads((case/'CONFIG_ONLY_restored.json').read_text())
(case/'SUMMARY.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in {'actual_endpoints','trials','native_sdk','moving_cancel'}},indent=2))
for trial in trials:print('TRIAL',trial['phase'],trial.get('terminal') or trial.get('after_cancel'),trial.get('hold'))
for item in sdk:print('SDK',item['identity']['executable'],item['counts'],item['obs_rate_from_median_hz'],item['neural_forwards'],item['models_loaded'])
