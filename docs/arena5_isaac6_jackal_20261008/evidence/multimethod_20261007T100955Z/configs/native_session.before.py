import json,os,pathlib,shlex,subprocess,sys,time
from gpu_preflight import require_safe
require_safe()
B=pathlib.Path('[LOCAL_PATH]');R=pathlib.Path(__file__).resolve().parents[2]/'runtime_cases';R.mkdir(exist_ok=True);C='arena-arena_ws-arena-1';D='189';method=sys.argv[1];count=int(sys.argv[2]);case=sys.argv[3];probe=pathlib.Path(sys.argv[4]);assert method in {'nav2','sicnav','dsrnn','cd-sarl'} and count in {0,1};headless=os.environ.get('A_RUNTIME_HEADLESS','0');assert headless in {'0','1'};O=R/case;O.mkdir(exist_ok=False);children={};owned={};(O/'CASE.json').write_text(json.dumps({'method':method,'humans':count,'domain':D,'headless':headless=='1','probe_sim_limit_s':120,'probe_wall_limit_s':210,'moving_cancel_wall_s':60,'stop_hold_wall_s':8,'state':'DRAFT_NOT_EXECUTED'},indent=2));(O/'driver_source.py').write_bytes(pathlib.Path(__file__).read_bytes())
def processes():
 code="import pathlib,json,os;out={}\nfor p in pathlib.Path('/proc').glob('[0-9]*'):\n try:\n  pid=int(p.name);cmd=(p/'cmdline').read_bytes().replace(b'\\0',b' ').decode(errors='replace').strip();st=(p/'stat').read_text().rsplit(')',1)[1].split();out[pid]={'pgid':os.getpgid(pid),'args':cmd,'start_ticks':st[19]}\n except (OSError,ProcessLookupError):pass\nprint(json.dumps(out))"
 return {int(k):v for k,v in json.loads(subprocess.check_output(['docker','exec',C,'/opt/venv/bin/python','-c',code],text=True)).items()}
baseline=processes();(O/'baseline_processes.json').write_text(json.dumps(baseline,indent=2))
def launch(label,cmd,stdin=None):
 out=O/label;out.mkdir();(out/'command.txt').write_text(shlex.join(cmd)+(' < '+str(stdin) if stdin else '')+'\n');(out/'environment.txt').write_text('ROS_DOMAIN_ID=189; frozen Jazzy/Isaac6; original world; current disclosed source patches\n')
 so=(out/'stdout.log').open('w');se=(out/'stderr.log').open('w');inp=stdin.open() if stdin else None;p=subprocess.Popen(cmd,stdin=inp,stdout=so,stderr=se);children[label]=(p,so,se,inp);return p

def finish(label,timeout=60):
 p,so,se,inp=children[label];rc=p.wait(timeout=timeout);so.close();se.close()
 if inp:inp.close()
 (O/label/'exit_code.txt').write_text(str(rc));return rc

def ready(label,marker,timeout):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  if marker in (O/label/'stdout.log').read_text(errors='replace'):return
  if children[label][0].poll() is not None:raise RuntimeError(label+' exited before '+marker)
  time.sleep(.3)
 raise TimeoutError(label+' '+marker)

def claim(label,marker):
 deadline=time.monotonic()+15
 while time.monotonic()<deadline:
  found={pid:v for pid,v in processes().items() if pid not in baseline and marker in v['args']}
  if len(found)==1:
   pid,v=next(iter(found.items()));owned[label]={'pid':pid,**v};(O/'ownership.json').write_text(json.dumps(owned,indent=2));return
  time.sleep(.2)
 raise RuntimeError('ownership '+label+' '+repr(found))

def native(kind,uid,args):
 return ['docker','exec','-u',str(uid),'-e','HOME=/home/arena','-e','ROS_DOMAIN_ID='+D,'-e','PYTHONUNBUFFERED=1','-e','TORCH_FORCE_WEIGHTS_ONLY_LOAD=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','PYTHONDONTWRITEBYTECODE=1',C,'bash','-c','source /opt/arena_ws/source >/dev/null; export ARENA_CONTAINER=1 ARENA_GPU=1; exec setsid --wait /opt/venv/bin/python -m arena_cli '+kind+' "$@"','--',*args]
try:
 launch('runtime',native('runtime',0,['sim:=isaac','headless:='+('true' if headless=='1' else 'false'),'world:=map_empty','log_level:=info','sim.isaac.viewport.resolution:=960x640']));claim('runtime','/opt/ros/jazzy/bin/ros2 launch arena_bringup arena_runtime.launch.py');ready('runtime','node initialized (sim=isaac)',330);print('RUNTIME_READY',flush=True)
 if __import__('os').environ.get('A_RUNTIME_ALIGN_PHYSICS_METADATA')=='1':
  assert method=='dsrnn','Metadata candidate is limited to the declared DS-RNN followup cases'
  setup=pathlib.Path(__file__).parent/'align_isaac_physics_metadata.py';(O/'metadata_source.py').write_bytes(setup.read_bytes())
  launch('metadata',['docker','exec','-i','-e','ROS_DOMAIN_ID='+D,C,'bash','-c','source /opt/arena_ws/source >/dev/null; exec /opt/venv/bin/python -'],setup);assert finish('metadata',40)==0
 args=['world:=map_empty','task.robots:=scenario','task.scenario:=quicktest','task.auto_reset:=false','env.id:=0','log_level:=info','robot:=jackal[1]','robot.mobile:='+('nav2' if method=='nav2' else 'drl'),*(['robot.mobile.planner:='+method] if method!='nav2' else []),*(['robot.mobile.deadline:=30.0'] if method!='nav2' else []),'human:='+('arena' if count else 'dummy'),'task.obstacles:=random','task.random.static.n:=[0,0]','task.random.interactive.n:=[0,0]','task.random.dynamic.n:=['+str(count)+','+str(count)+']','task.random.dynamic.models:=[arenian]','humansim.local_planner:=sfm','humansim.global_planner:=navmesh','humansim.seed:=42','humansim.force_local_planner:=true','debug.map_server:=true']
 # CONFIG_ONLY explicit native scenario placement. The default launch remains
 # the original random fixture, and the same guarded driver owns all cleanup.
 obstacle_mode=__import__('os').environ.get('A_RUNTIME_OBSTACLES_MODE','random');assert obstacle_mode in {'random','scenario'}
 scenario_override=__import__('os').environ.get('A_RUNTIME_SCENARIO_FILE','')
 if obstacle_mode=='scenario':
  assert count==1 and scenario_override,'Explicit one-person scenario requires a fixture path'
  args=[('task.obstacles:=scenario' if value=='task.obstacles:=random' else 'task.scenario:='+scenario_override if value=='task.scenario:=quicktest' else value) for value in args]
 launch('env',native('env',1000,args));claim('env','/opt/ros/jazzy/bin/ros2 launch task_generator task_generator.launch.py');ready('env','Successfully switched controllers!',180);print('ENV_READY',flush=True)
 ns='/arena/env_0/task_generator_node/jackal'
 framecmds=[f'ros2 param set {ns}/jackal_velocity_controller tf_frame_prefix_enable false']+[f'ros2 control set_controller_state jackal_velocity_controller {s} -c {ns}/controller_manager' for s in ['inactive','unconfigured','inactive','active']]
 for i,cmd in enumerate(framecmds):
  label='frame_'+str(i)
  if i==0:
   setup=pathlib.Path(__file__).parent/'set_controller_frame_prefix.py';launch(label,['docker','exec','-i','-e','ROS_DOMAIN_ID='+D,C,'bash','-c','source /opt/arena_ws/source >/dev/null; exec /opt/venv/bin/python -'],setup)
  else:launch(label,['docker','exec','-e','ROS_DOMAIN_ID='+D,C,'bash','-c','source /opt/arena_ws/source >/dev/null; exec '+cmd])
  assert finish(label,70 if i==0 else 25)==0
 (O/'probe_source.py').write_bytes(probe.read_bytes());launch('probe',['docker','exec','-i','-e','ROS_DOMAIN_ID='+D,'-e','A_METHOD='+method,'-e','A_HUMANS='+str(count),'-e','A_TRACE_DIR='+__import__('os').environ.get('A_RUNTIME_TRACE_DIR',''),'-e','A_SCENARIO_FILE='+__import__('os').environ.get('A_RUNTIME_SCENARIO_FILE',''),C,'bash','-c','source /opt/arena_ws/source >/dev/null; exec timeout --signal=INT --kill-after=5 780 /opt/venv/bin/python -'],probe);assert finish('probe',800)==0
 (O/'completed.txt').write_text('native probe completed');print('PROBE_COMPLETED',flush=True)
except Exception as exc:
 (O/'driver_error.txt').write_text(repr(exc));print('DRIVER_ERROR',repr(exc),flush=True);raise
finally:
 for label in ['env','runtime']:
  if label in owned:
   v=owned[label];now=processes()
   if v['pid'] in now and now[v['pid']]=={k:q for k,q in v.items() if k!='pid'}:
    assert not any(p in baseline and q['pgid']==v['pgid'] for p,q in now.items()),'baseline process in group'
    cmd=['docker','exec','-u','0',C,'kill','-INT','--','-'+str(v['pgid'])];subprocess.run(cmd,capture_output=True)
  if label in children:
   try:finish(label,40)
   except subprocess.TimeoutExpired:
    if label in owned:
     v=owned[label];now=processes()
     members={p:q for p,q in now.items() if q['pgid']==v['pgid']}
     assert not any(p in baseline for p in members),'protected baseline in owned group'
     assert v['pid'] in now and now[v['pid']]=={k:q for k,q in v.items() if k!='pid'},'root identity changed; refusing group TERM'
     subprocess.run(['docker','exec','-u','0',C,'kill','-TERM','--','-'+str(v['pgid'])],capture_output=True)
    finish(label,20)
 # ros2 control can create a detached CLI daemon. It belongs to this case only
 # if absent from the baseline and its exact argv names our reserved domain.
 auxiliary=[]
 for pid,v in processes().items():
  if pid not in baseline and 'ros2cli.daemon.daemonize' in v['args'] and '--ros-domain-id 189' in v['args']:
   now=processes()
   assert now.get(pid)==v,'CLI daemon identity changed; refusing signal'
   cmd=['docker','exec','-u','0',C,'kill','-INT',str(pid)]
   result=subprocess.run(cmd,capture_output=True,text=True)
   auxiliary.append({'pid':pid,**v,'command':cmd,'exit_code':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
 (O/'auxiliary_cleanup.json').write_text(json.dumps(auxiliary,indent=2))
 (O/'final_processes.json').write_text(json.dumps(processes(),indent=2));(O/'container_final.json').write_text(subprocess.check_output(['docker','inspect','arena-arena_ws-isaac-1'],text=True))
 # Lossless archive of this owned container's Kit log, before the next case
 # recreates it. This does not restart or modify the container.
 import gzip,hashlib
 archive_cmd=['docker','cp','arena-arena_ws-isaac-1:/isaac-sim/kit/logs/Kit/Isaac-Sim Python/6.0','-']
 archive_process=subprocess.Popen(archive_cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 digest=hashlib.sha256();uncompressed=0
 with gzip.open(O/'isaac_kit_logs.tar.gz','xb',compresslevel=6) as stream:
  while chunk:=archive_process.stdout.read(2**20):stream.write(chunk);digest.update(chunk);uncompressed+=len(chunk)
 archive_error=archive_process.stderr.read().decode(errors='replace');archive_rc=archive_process.wait(timeout=30)
 (O/'KIT_LOG_ARCHIVE.json').write_text(json.dumps({'command':archive_cmd,'exit_code':archive_rc,'stderr':archive_error,'uncompressed_tar_bytes':uncompressed,'uncompressed_tar_sha256':digest.hexdigest(),'archive_bytes':(O/'isaac_kit_logs.tar.gz').stat().st_size},indent=2))
