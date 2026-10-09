"""Guarded native A runtime. Usage: configured_a_session.py METHOD HUMANS CASE.

Optional diagnostics and timing metadata alignment are explicitly CONFIG_ONLY.
"""
import hashlib,json,pathlib,subprocess,sys
from gpu_preflight import require_safe
BASE=pathlib.Path('[LOCAL_PATH]');HERE=pathlib.Path(__file__).resolve().parent;RUN=HERE.parents[1]
if len(sys.argv)!=4:raise SystemExit('Usage: METHOD(nav2/sicnav/dsrnn/cd-sarl/cadrl) HUMANS(0/1) UNIQUE_CASE')
method,count,case=sys.argv[1:];assert method in {'nav2','sicnav','dsrnn','cd-sarl','cadrl','drlvo','attngraph','kdma','rlrvo','height','scope','crowdsurfer','gensafenav','her-drl','navistar','navrep','sonic'} and count in {'0','1'} and '/' not in case
require_safe()
# Exclusive A lock, after resource gate; no stale-lock deletion.
lock=RUN/'A_RUNTIME.lock';handle=lock.open('x');handle.write(json.dumps({'argv':sys.argv,'pid':__import__('os').getpid()}));handle.close();__import__('os').environ['A3_OWNED_RUNTIME_LOCK']=str(lock)
entry=BASE/'arena_ws/src/Arena/arena_isaac/arena_isaac/arena_isaac/run_isaacsim.py';launch=BASE/'arena_ws/src/Arena/humansim/arena_humansim/launch/arena_humansim.launch.py';original={};modified={}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
try:
 final=json.loads((RUN/'evidence/FINAL_SOURCE_MANIFEST.json').read_text())
 for item in final['sources']:assert sha(pathlib.Path(item['path']))==item['sha256'],'Source changed: '+item['path']
 for item in json.loads((RUN/'evidence/STAGED_MODELS.json').read_text()):assert sha(pathlib.Path(item['destination']))==item['sha256'],'Staged model changed'
 sensor=BASE/'arena_ws/data/continuation_run_20261003T204052-0700/isaac6_sensor_assets'
 for item in json.loads((sensor/'provenance.json').read_text()):assert sha(pathlib.Path(item['path']))==item['sha256'],'Official sensor byte changed'
 original={entry:entry.read_bytes(),launch:launch.read_bytes()};marker='_carb_settings.set("/exts/isaacsim.ros2.bridge/publish_without_verification", True)';text=original[entry].decode();assert marker in text and 'CONFIG_ONLY: this owned domain' not in text
 addition='\n# CONFIG_ONLY: this owned domain; A2 official RTX root, restored after this case.\nif os.environ.get("ROS_DOMAIN_ID") == "189":\n    _carb_settings.set("/persistent/isaac/asset_root/default", "/opt/arena_ws/data/continuation_run_20261003T204052-0700/isaac6_sensor_assets")\n'
 modified[entry]=text.replace(marker,marker+addition).encode();text=original[launch].decode();old='cmd=["pkill", "-INT", "-f", "rviz2.*arena_humansim.rviz"]';assert old in text
 modified[launch]=text.replace(old,'cmd=(["true"] if __import__("os").environ.get("ROS_DOMAIN_ID") == "189" else ["pkill", "-INT", "-f", "rviz2.*arena_humansim.rviz"])').encode()
 # CONFIG_ONLY observer, scoped to the owned domain. It logs actual native
 # callback inputs/results and model loads, without changing their values.
 trace_host=BASE/'arena_ws/data/parallel_round3/A_integration/run_20261004T172624Z/runtime_traces'/case
 trace_host.mkdir(parents=True,exist_ok=False)
 trace_container='/opt/arena_ws/data/parallel_round3/A_integration/run_20261004T172624Z/runtime_traces/'+case
 __import__('os').environ['A_RUNTIME_TRACE_DIR']=trace_container
 if __import__('os').environ.get('A_RUNTIME_PHYSICS_OBSERVER')=='1':
  helper=trace_host/'native_physics_observer.py';helper.write_bytes((HERE/'native_physics_observer.py').read_bytes())
  text=modified[entry].decode();marker='    pedestrian_runtime.initialize(world)';assert text.count(marker)==1
  addition='\n    if os.environ.get("ROS_DOMAIN_ID") == "189":\n        import importlib.util\n        _a_followup_spec = importlib.util.spec_from_file_location("a_native_physics_observer", '+repr(trace_container+'/native_physics_observer.py')+')\n        _a_followup_observer = importlib.util.module_from_spec(_a_followup_spec)\n        _a_followup_spec.loader.exec_module(_a_followup_observer)\n        _a_followup_observer.install(world)\n'
  modified[entry]=text.replace(marker,marker+addition).encode()
 if method!='nav2':
  helper=trace_host/'native_sdk_observer.py';helper.write_bytes((HERE/'native_sdk_observer.py').read_bytes())
  sdk=BASE/'arena_ws/src/Arena/arena_planners/arena_planners/arena_planners/sdk.py';original[sdk]=sdk.read_bytes();text=original[sdk].decode()
  marker='    PlannerSDK(manifest=manifest, capabilities=capabilities).run(step_fn, on_reset=on_reset, on_cancel=on_cancel)';assert text.count(marker)==1
  addition='    if __import__("os").environ.get("ROS_DOMAIN_ID") == "189":\n        import importlib.util\n        _spec = importlib.util.spec_from_file_location("a3_native_sdk_observer", '+repr(trace_container+'/native_sdk_observer.py')+')\n        _observer = importlib.util.module_from_spec(_spec)\n        _spec.loader.exec_module(_observer)\n        _observer.install(PlannerSDK, step_fn, manifest, '+repr(trace_container)+')\n'
  modified[sdk]=text.replace(marker,addition+marker).encode()
  edge=BASE/'arena_ws/src/Arena/arena_planners/arena_planners/arena_planners/bridge/edge_node.py';original[edge]=edge.read_bytes();text=original[edge].decode()
  marker='        self._clamp_to_limits(msg)\n        self._cmd_vel_pub.publish(msg)';assert text.count(marker)==1
  observation='        self._clamp_to_limits(msg)\n        if __import__("os").environ.get("ROS_DOMAIN_ID") == "189":\n            import json as _a3_json, time as _a3_time\n            with open('+repr(trace_container+'/edge_publish.jsonl')+', "a") as _a3_log:\n                _a3_log.write(_a3_json.dumps({"pid": __import__("os").getpid(), "file": __file__, "wall": _a3_time.monotonic(), "seq": action.seq, "action_type": action.action_type, "action": action.action, "robot_pose": features.get("robot_pose"), "robot_state": features.get("robot_state"), "step_dt_s": 1.0 / float(self._rate.value), "cmd": {part: {axis: getattr(getattr(msg, part), axis) for axis in ("x", "y", "z")} for part in ("linear", "angular")}}, default=lambda value: value.tolist() if hasattr(value, "tolist") else str(value)) + "\\n")\n        self._cmd_vel_pub.publish(msg)'
  modified[edge]=text.replace(marker,observation).encode()
  if method=='dsrnn' and __import__('os').environ.get('A_RUNTIME_DSRNN_PATH_OBSERVER')=='1':
   helper=trace_host/'dsrnn_path_observer.py';helper.write_bytes((HERE/'dsrnn_path_observer.py').read_bytes())
   text=modified[edge].decode()
   setup_marker='        assert self._data_push is not None'
   assert text.count(setup_marker)==1
   setup_addition='\n        import importlib.util as _a_path_importlib\n        _a_path_spec = _a_path_importlib.spec_from_file_location("a_dsrnn_path_observer", '+repr(trace_container+'/dsrnn_path_observer.py')+')\n        _a_path_module = _a_path_importlib.module_from_spec(_a_path_spec)\n        _a_path_spec.loader.exec_module(_a_path_module)\n        _a_path_recorder = _a_path_module.Recorder('+repr(trace_container+'/path_subgoal_costmap.jsonl')+')\n'
   text=text.replace(setup_marker,setup_marker+setup_addition)
   tick_marker='                        features = self._filter_wire_features(self._obs_manager.collect())'
   assert text.count(tick_marker)==1
   tick_addition='                        _a_raw_features = self._obs_manager.collect()\n                        _a_path_recorder.record(self._seq + 1, t.sec + t.nanosec * 1e-9, _a_raw_features)\n                        features = self._filter_wire_features(_a_raw_features)'
   modified[edge]=text.replace(tick_marker,tick_addition).encode()
 if count=='1':
  helper=trace_host/'consumer_observer.py';helper.write_bytes((BASE/'reports/continuation/tools/consumer_observer.py').read_bytes())
  if __import__('os').environ.get('A_RUNTIME_NEIGHBOR_OBSERVER')=='1':
   observer=helper.read_text();marker="'policies':pool.policy_idx[:n].tolist()";assert observer.count(marker)==1
   addition="'vision_range':pool.vision_range[:n].tolist(),'vision_fov':pool.vision_fov[:n].tolist(),'proximity_sense':pool.proximity_sense[:n].tolist(),'neighbor_indptr':pool.neighbor_indptr.tolist(),'neighbor_indices':pool.neighbor_indices.tolist(),'neighbor_csr_complete':len(pool.neighbor_indptr)==n+1,"
   helper.write_text(observer.replace(marker,addition+marker))
  text=modified[launch].decode();marker='        executable="arena_humansim_node",';assert text.count(marker)==1
  addition='\n        prefix=('+repr('/opt/venv/bin/python '+trace_container+'/consumer_observer.py '+trace_container)+' if os.environ.get("ROS_DOMAIN_ID") == "189" else None),'
  modified[launch]=text.replace(marker,marker+addition).encode()
 (trace_host/'CONFIG_ONLY_IDENTITY.json').write_text(json.dumps({'original':{str(p):hashlib.sha256(v).hexdigest() for p,v in original.items()},'observed':{str(p):hashlib.sha256(v).hexdigest() for p,v in modified.items()}},indent=2))
 for path,data in modified.items():path.write_bytes(data)
 require_safe()
 nav2_smoke=__import__('os').environ.get('A_RUNTIME_NAV2_SMOKE')=='1'
 if nav2_smoke:assert method=='nav2' and count=='0','Headless autonomous navigation smoke is Jackal+Nav2 without pedestrians'
 autonomous_probe=__import__('os').environ.get('A_RUNTIME_AUTONAV_PROBE')=='1'
 if autonomous_probe:assert count=='0','Original quicktest autonomous probe is zero-pedestrian only'
 probe=HERE/('autonomous_nav_probe.py' if nav2_smoke or autonomous_probe else 'lifecycle_probe.py')
 rc=subprocess.call([sys.executable,str(HERE/'native_session.py'),method,count,case,str(probe)])
finally:
 restored={}
 for path,data in modified.items():
  assert path.read_bytes()==data,'Concurrent source change; refusing restore overwrite: '+str(path)
  path.write_bytes(original[path]);restored[str(path)]=sha(path)
 if restored:
  out=RUN/'runtime_cases'/case
  if out.is_dir():(out/'CONFIG_ONLY_restored.json').write_text(json.dumps(restored,indent=2))
 lock.unlink()
sys.exit(rc)
