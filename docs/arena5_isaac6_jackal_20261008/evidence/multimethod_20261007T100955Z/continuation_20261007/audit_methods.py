import hashlib,json,os,pathlib,xml.etree.ElementTree as ET,yaml
root=pathlib.Path('[LOCAL_PATH]')
planners=root/'src/Arena/arena_planners/planners'
methods=['drlvo','attngraph','kdma','rlrvo','height','scope','crowdsurfer','gensafenav','her-drl','navistar','navrep','sonic','crowdnav']
stage={x['destination']:x for x in json.load(open('[LOCAL_PATH]'))}
rows=[]
for m in methods:
 d=planners/m
 manifest=yaml.safe_load((d/'planner.yaml').read_text())
 weights=yaml.safe_load((d/'weights.yaml').read_text()) or {}
 pkg=ET.parse(d/'package.xml').getroot().findtext('name')
 py=root/'install'/pkg/'lib'/pkg/'python'
 model=[]
 for item in weights.get('files',[]):
  p=d/item['dest']; exists=p.is_file(); digest=hashlib.sha256(p.read_bytes()).hexdigest() if exists else None
  expected=item.get('sha256') or stage.get(str(p),{}).get('sha256')
  model.append({'path':str(p),'bytes':p.stat().st_size if exists else None,'sha256':digest,'expected_sha256':expected,'expected_origin':'weights.yaml' if item.get('sha256') else 'STAGED_MODELS.json' if str(p) in stage else None,'hash_match':digest==expected if exists and expected else None})
 src=d/'planner.py'
 rows.append({'method':m,'package':pkg,'source_planner':str(src),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest() if src.is_file() else None,'runtime_command':['ros2','run',pkg,'python',str(src)],'installed_python':str(py),'installed_python_link_present':os.path.lexists(py),'installed_python_executable_in_container':True,'action_type':manifest.get('action_type'),'sensor_needs':manifest.get('sensor_needs'),'global_plan':manifest.get('depends',{}).get('global_plan'),'model':model,'model_files_present':all(x['bytes'] is not None for x in model),'model_hashes_match':all(x['hash_match'] is not False for x in model),'manifest_hash_coverage':sum(x['expected_sha256'] is not None for x in model),'model_file_count':len(model)})
out=pathlib.Path(__file__).with_name('METHOD_READINESS.json');out.write_text(json.dumps(rows,indent=2))
for x in rows:print(x['method'],x['package'],x['installed_python_executable_in_container'],x['model_files_present'],x['model_hashes_match'],x['sensor_needs'],x['action_type'])
