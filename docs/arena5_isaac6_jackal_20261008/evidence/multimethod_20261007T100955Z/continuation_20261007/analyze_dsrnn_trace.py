import glob,json,math,pathlib
run='multimethod_dsrnn_headless_20261007T181954Z_2376297'
base=pathlib.Path('[LOCAL_PATH]')/run
path=[json.loads(x) for x in open(base/'path_subgoal_costmap.jsonl')]
edge=[json.loads(x) for x in open(base/'edge_publish.jsonl')]
sdk=[json.loads(x) for x in open(next(base.glob('sdk*.jsonl')))]
stop=json.load(open(base/'autonomous_nav_stop_trace.json'))
by_seq={x['seq']:x for x in path}
first_collision=stop['collisions'][0]
near=min(path,key=lambda r:abs(r['wall']-first_collision[0]))
neural=[x for x in sdk if x['kind']=='actual_neural_input']
actions=[x for x in sdk if x['kind']=='step']
models=[x for x in sdk if x['kind']=='actual_model_load']
key=[]
for seq in [46,47,50,52,54,55,56,57,60,66,69,72,76]:
 p=by_seq.get(seq);e=next((x for x in edge if x['seq']==seq),None)
 if p and p['robot_pose']:
  key.append({'seq':seq,'sim':p['sim'],'robot_pose':p['robot_pose'],'subgoal':p['subgoal'],'plan_points':p['plan_points'],'plan_y_minmax':[min(v[1] for v in p['global_plan']),max(v[1] for v in p['global_plan'])] if p['global_plan'] else None,'robot_cost':p['costmap']['robot_cost'] if p['costmap'] else None,'subgoal_cost':p['costmap']['subgoal_cost'] if p['costmap'] else None,'chord_blocked_count':p['costmap']['chord_blocked_99_count'] if p['costmap'] else None,'action':e.get('action') if e else None,'cmd':e.get('cmd') if e else None})
finite=lambda obj:all(math.isfinite(float(v)) for v in flatten(obj))
def flatten(x):
 if isinstance(x,dict):
  for v in x.values():yield from flatten(v)
 elif isinstance(x,list):
  for v in x:yield from flatten(v)
 elif isinstance(x,(int,float)):yield x
summary={'case':run,'result':json.load(open(pathlib.Path('[LOCAL_PATH]')/run/'result.json')),'trace_dir':str(base),'path_rows':len(path),'path_rows_with_plan':sum(x['plan_points']>0 for x in path),'neural_inputs':len(neural),'neural_inputs_all_finite':all(finite(x['inputs']) and finite(x['rnn_hxs']) for x in neural),'masks_first_last':[neural[0]['masks'],neural[-1]['masks']] if neural else None,'model_loads':[{'path':x['path'],'sha256':x['sha256']} for x in models],'policy_step_rows':len(actions),'edge_action_rows':len(edge),'collision_first_wall':first_collision[0],'collision_first_seq':near['seq'],'collision_first_pose_nearest':near['robot_pose'],'key_rows':key}
out=pathlib.Path(__file__).with_name('DSRNN_PATH_INPUT_ACTION_ANALYSIS.json');out.write_text(json.dumps(summary,indent=2));print(json.dumps({k:summary[k] for k in ('path_rows','path_rows_with_plan','neural_inputs','neural_inputs_all_finite','masks_first_last','policy_step_rows','edge_action_rows','collision_first_seq','collision_first_pose_nearest')}))
