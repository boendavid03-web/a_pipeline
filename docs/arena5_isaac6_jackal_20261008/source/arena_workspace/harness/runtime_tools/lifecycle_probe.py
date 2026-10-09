import json,math,pathlib,time,os
method=os.environ["A_METHOD"];human_count=int(os.environ["A_HUMANS"])
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict as md
from rosidl_runtime_py.utilities import get_message
from task_generator_msgs.action import RunEpisode
from task_generator_msgs.srv import ResetEpisode
from arena_robots_msgs.action import GotoPose
from arena_robots_msgs.msg import CollisionEvents
from isaacsim_msgs.msg import Prim
from isaacsim_msgs.srv import EditPrims,GetPrims
from arena_people_msgs.msg import Pedestrians
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist,TwistStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
from rcl_interfaces.srv import GetParameters
rclpy.init();n=rclpy.create_node('a2_lifecycle_observer',parameter_overrides=[Parameter('use_sim_time',value=True)]);b=Buffer();listener=TransformListener(b,n);ns='/arena/env_0/task_generator_node/jackal';phase='startup';rows={'poses':[],'cmd':[],'cmd_channels':[],'clock':[],'odom':[],'paths':[],'collisions':[],'scans':[],'maps':[],'states':[],'world':[],'agents':[],'observations':[],'joints':[],'controller_cmd':[]};out={'trials':[]};latest={};last={};q=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT);latched=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def sample(k,m,topic=None):
 now=time.monotonic()
 # A single terminal zero can arrive within 80ms of the preceding drive cmd.
 # Preserve every actual cmd callback; never synthesize repeated cached zeros.
 if now-last.get((k,topic),0)<(0.0 if k in ('cmd','cmd_channels','controller_cmd') else (1.0 if k in ('maps','scans') else .08)):return
 last[k,topic]=now;data={'phase':phase,'wall':now,'sim':n.get_clock().now().nanoseconds*1e-9,'msg':md(m),'topic':topic};
 if k in ('maps','scans'):
  values=data['msg'].pop('data' if k=='maps' else 'ranges',[])
  finite=[v for v in values if math.isfinite(v)];data['summary']={'samples':len(values),'finite':len(finite),'min':min(finite) if finite else None,'max':max(finite) if finite else None}
 rows[k].append(data)
 if topic:latest[topic]=data
for typ,topic,k in [(Twist,ns+'/cmd_vel','cmd'),(Odometry,ns+'/odom','odom'),(LaserScan,ns+'/lidar','scans'),(Path,ns+'/plan','paths'),(Path,ns+'/local_plan','paths'),(CollisionEvents,ns+'/collision_events','collisions')]:n.create_subscription(typ,topic,lambda m,k=k,t=topic:sample(k,m,t),q)
n.create_subscription(Clock,'/clock',lambda m:sample('clock',m,'/clock'),q)
for topic in [ns+'/cmd_vel_nav',ns+'/cmd_vel_smoothed']:
 n.create_subscription(Twist,topic,lambda m,t=topic:sample('cmd_channels',m,t),q)
for topic in [ns+'/joint_states',ns+'/isaac/joint_states',ns+'/isaac/joint_commands_velocity']:
 n.create_subscription(JointState,topic,lambda m,t=topic:sample('joints',m,t),q)
n.create_subscription(TwistStamped,ns+'/jackal_velocity_controller/cmd_vel',lambda m:sample('controller_cmd',m,ns+'/jackal_velocity_controller/cmd_vel'),q)
rows['usd']=[]
usd_client=n.create_client(GetPrims,'/isaac/GetPrims') if human_count else None
usd_pending=None;usd_before=None;usd_next=0
n.create_subscription(Pedestrians,'/isaac/arena_peds',lambda m:sample('observations',m,'/isaac/arena_peds'),q)
for topic in ['/arena/env_0/task_generator_node/map','/arena/env_0/door_mask',ns+'/global_costmap/costmap',ns+'/local_costmap/costmap']:n.create_subscription(OccupancyGrid,topic,lambda m,t=topic:sample('maps',m,t),latched)
def checkpoint(label):
 print('PROBE_PHASE '+label,flush=True)
 if os.environ.get('A_TRACE_DIR'):
  (pathlib.Path(os.environ['A_TRACE_DIR'])/'physics_phase.json').write_text(json.dumps({'phase':phase,'label':label,'wall':time.monotonic(),'sim':n.get_clock().now().nanoseconds*1e-9}))
 p=pathlib.Path('/opt/arena_ws/data/parallel_round3/A_integration/run_20261004T172624Z/runtime_checkpoints')/str(__import__('os').getpid())
 p.mkdir(exist_ok=True,parents=True)
 (p/(label+'.json')).write_text(json.dumps({**out,'row_counts':{k:len(v) for k,v in rows.items()}}))
def step():
 global usd_pending,usd_before,usd_next
 rclpy.spin_once(n,timeout_sec=.02)
 if usd_pending is not None and usd_pending.done():
  try:rows['usd'].append({'phase':phase,'wall':time.monotonic(),'sim':n.get_clock().now().nanoseconds*1e-9,'requested_input':usd_before,'usd':md(usd_pending.result())})
  except Exception as ex:out.setdefault('usd_errors',[]).append(repr(ex))
  usd_pending=None
 if usd_client is not None and usd_pending is None and time.monotonic()>=usd_next and usd_client.service_is_ready():
  message=latest.get('/isaac/arena_peds',{}).get('msg',{})
  if message.get('pedestrians'):
   usd_before=message;names=['/World/'+p['name'].lstrip('/') for p in message['pedestrians']];usd_pending=usd_client.call_async(GetPrims.Request(names=names));usd_next=time.monotonic()+2
 if time.monotonic()-last.get(('poses',None),0)>.1:
  try:sample('poses',b.lookup_transform('map','env_0/jackal/base_link',rclpy.time.Time()))
  except Exception:pass

def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:step()
def wait(f,timeout):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:step()
 return f.done()
def call(cls,topic,request,timeout=40):
 c=n.create_client(cls,topic);assert c.wait_for_service(timeout_sec=15),topic;f=c.call_async(request);assert wait(f,timeout),topic;return f.result()

from rcl_interfaces.srv import SetParametersAtomically
from arena_humansim_msgs.msg import AgentStates
for topic,key in [('/arena/env_0/task_generator_node/world_state','world'),('/arena/env_0/task_generator_node/agent_states','agents')]:
 n.create_subscription(AgentStates,topic,lambda m,key=key,t=topic:sample(key,m,t),q)
spin(3)
for topic in ['/arena/env_0/arena_peds','/arena/env_0/human/stream','/arena/env_0/task_generator_node/jackal/people','/arena/env_0/task_generator_node/jackal/pedestrians','/isaac/pedestrians']:
 types=dict(n.get_topic_names_and_types()).get(topic,[])
 if types:
  try:n.create_subscription(get_message(types[0]),topic,lambda m,t=topic:sample('observations',m,t),q)
  except Exception as ex:out.setdefault('subscription_errors',[]).append([topic,repr(ex)])
for topic in ['/arena/state/envs','/arena/env_0/task_generator_node/state/episode','/arena/env_0/task_generator_node/semantics/state']:
 types=dict(n.get_topic_names_and_types()).get(topic,[])
 if types:n.create_subscription(get_message(types[0]),topic,lambda m,t=topic:sample('states',m,t),latched)
# Record native human-observation publications actually present in this graph.
for topic,types in n.get_topic_names_and_types():
 if '/jackal/' in topic and any('Pedestrian' in typ or 'Human' in typ or 'Agent' in typ for typ in types):
  try:n.create_subscription(get_message(types[0]),topic,lambda m,t=topic:sample('observations',m,t),q)
  except Exception as ex:out.setdefault('subscription_errors',[]).append([topic,repr(ex)])
spin(3);out['registry']=latest['/arena/state/envs']['msg'];out['topics']=n.get_topic_names_and_types()
out['actual_endpoints']={topic:{'publishers':[{'name':v.node_name,'namespace':v.node_namespace,'type':v.topic_type} for v in n.get_publishers_info_by_topic(topic)],'subscribers':[{'name':v.node_name,'namespace':v.node_namespace,'type':v.topic_type} for v in n.get_subscriptions_info_by_topic(topic)]} for topic in [ns+'/lidar','/arena/env_0/arena_peds','/arena/env_0/task_generator_node/world_state','/isaac/arena_peds',ns+'/cmd_vel',ns+'/cmd_vel_nav',ns+'/cmd_vel_smoothed',ns+'/jackal_velocity_controller/cmd_vel',ns+'/isaac/joint_commands_velocity',ns+'/isaac/joint_states']}
out['actual_parameters']={}
for node,names in [('/arena',['physics_dt']), (ns+'/jackal_velocity_controller',['cmd_vel_timeout','publish_cmd','publish_rate','use_sim_time']), *([(ns+'/collision_monitor',['stop_pub_timeout','cmd_vel_in_topic','cmd_vel_out_topic','use_sim_time']),(ns+'/velocity_smoother',['velocity_timeout','smoothing_frequency','feedback','use_sim_time'])] if method=='nav2' else [])]:
 try:
  out['actual_parameters'][node]= {'names':names,'response':md(call(GetParameters,node+'/get_parameters',GetParameters.Request(names=names)))}
 except Exception as ex:out['actual_parameters'][node]={'names':names,'error':repr(ex)}
client=ActionClient(n,RunEpisode,'/arena/env_0/task_generator_node/lifecycle/run_episode');assert client.wait_for_server(timeout_sec=20)
try:
 for scenario_name,case_phase,expected_goal in [(os.environ.get('A_SCENARIO_FILE') or "/opt/arena_ws/data/continuation_run_20261003T204052-0700/fixtures/sdk_free_goal",p,[13.,20.]) for p in ["native_free_goal_1","native_moving_cancel","native_free_goal_2"]]:
  count=human_count
  changed=call(SetParametersAtomically,"/arena/env_0/task_generator_node/set_parameters_atomically",SetParametersAtomically.Request(parameters=[Parameter("task.scenario.file",value=scenario_name).to_parameter_msg()]))
  assert changed.result.successful
  response=call(SetParametersAtomically,'/arena/env_0/task_generator_node/set_parameters_atomically',SetParametersAtomically.Request(parameters=[Parameter('task.random.dynamic.n',value=[count,count]).to_parameter_msg()]))
  out.setdefault('parameter_changes',[]).append({'count':count,'response':md(response)});assert response.result.successful
  phase=case_phase;checkpoint(phase+'_started');f=client.send_goal_async(RunEpisode.Goal(world='map_empty',seed=42));assert wait(f,45);h=f.result();assert h.accepted;r=h.get_result_async()
  if phase == 'native_moving_cancel':
   deadline=time.monotonic()+60;moving=False;pp=[];cc=[];window=[]
   while time.monotonic()<deadline and not r.done():
    step();pp=[v for v in rows['poses'] if v['phase']==phase];cc=[v for v in rows['cmd'] if v['phase']==phase]
    if pp and cc:
     pos=pp[-1]['msg']['transform']['translation'];moving=8.3<pos['x']<11.5 and abs(pos['y']-20)<1 and cc[-1]['msg']['linear']['x']>.2
     # A map pose alone can come from reset teleportation. Require fresh TF
     # displacement inside the driving region, as well as the actual command.
     window=[v for v in pp if pp[-1]['wall']-.8<=v['wall']<=pp[-1]['wall'] and 8.0<=v['msg']['transform']['translation']['x']<11.5 and abs(v['msg']['transform']['translation']['y']-20)<1]
     if moving and len(window)>=3:
      xy=lambda v:[v['msg']['transform']['translation'][key] for key in ['x','y']]
      moving=math.dist(xy(window[0]),xy(window[-1]))>=.02
     else:moving=False
     if moving:break
   out.setdefault('moving_cancel_checks',[]).append({'moving':moving,'before_cancel_wall':time.monotonic(),'pose':pp[-1] if pp else None,'cmd':cc[-1] if cc else None,'tf_motion_window':window if pp and cc else []});assert moving
   done=False
  else:
   wall_end=time.monotonic()+210;sim_start=n.get_clock().now().nanoseconds*1e-9
   while not r.done() and time.monotonic()<wall_end and n.get_clock().now().nanoseconds*1e-9-sim_start<120:step()
   done=r.done()
  row={'terminal_received_wall':time.monotonic(),'terminal_received_sim':n.get_clock().now().nanoseconds*1e-9,'phase':phase,'count':count,'expected_goal_map':expected_goal,'scenario':scenario_name,'accepted':h.accepted,'terminal':md(r.result()) if done else None}
  if not done:
   row['cancel_requested_wall']=time.monotonic();c=h.cancel_goal_async();wait(c,20);row['cancel_ack']=md(c.result()) if c.done() else None;wait(r,40);row['cancel_terminal_wall']=time.monotonic();row['cancel_terminal_sim']=n.get_clock().now().nanoseconds*1e-9;row['after_cancel']=md(r.result()) if r.done() else None
  out['trials'].append(row)
  spin(.5);hold_start=time.monotonic();spin(8)
  cmds=[v for v in rows['cmd'] if v['wall']>=hold_start and v['topic']==ns+'/cmd_vel'];poses=[v for v in rows['poses'] if v['wall']>=hold_start]
  xy=lambda v:[v['msg']['transform']['translation'][key] for key in ['x','y']]
  drift=max((math.dist(xy(v),xy(poses[0])) for v in poses),default=None)
  cmd_max=max((abs(v['msg'][part][axis]) for v in cmds for part in ['linear','angular'] for axis in ['x','y','z']),default=None)
  status='INCONCLUSIVE' if len(cmds)<10 or len(poses)<10 else ('PASS' if cmd_max<1e-6 and drift<=.02 else 'FAIL')
  row['hold']={'status':status,'cmd_count':len(cmds),'pose_count':len(poses),'drift_m':drift,'max_abs_cmd_component':cmd_max,'wall_s':8,'wall_start':hold_start,'wall_end':time.monotonic(),'sim_first':poses[0]['sim'] if poses else None,'sim_last':poses[-1]['sim'] if poses else None}
  # Missing observations remain INCONCLUSIVE. Continue collecting the other
  # lifecycle stages only when the actual TF hold and last received zero cmd
  # establish that it is reasonable to submit the next diagnostic task.
  assert len(poses)>=10 and drift<=.02,'Stopped displacement not established'
  assert cmd_max is None or cmd_max<1e-6,'Nonzero final drive command during stopped hold'
  if not cmds:
   last_cmd=latest.get(ns+'/cmd_vel')
   assert last_cmd and all(abs(last_cmd['msg'][part][axis])<1e-6 for part in ['linear','angular'] for axis in ['x','y','z']),'No received zero command before hold'
  checkpoint(phase+'_finished')
  if 'moving_cancel' not in phase:
   assert done,'Task watchdog reached; record TASK_FAIL without changing method/model'
   terminal=row['terminal'];assert terminal['status']==4 and terminal['result']['state']==RunEpisode.Result.SUCCESS,'Expected actual task SUCCESS'
  else:
   assert r.done(),'Cancellation terminal missing'
   cancelled=md(r.result());assert cancelled['status']==5 and cancelled['result']['state']==RunEpisode.Result.SKIPPED,'Expected canceled action and SKIPPED episode'
 if any(row.get('hold',{}).get('status')!='PASS' for row in out['trials']):raise RuntimeError('Lifecycle task stages executed; stop acceptance remains INCONCLUSIVE or FAIL')
except Exception as ex:
 out['error']=repr(ex);raise
finally:
 out['rows']=rows;print('A_LIFECYCLE_JSON '+json.dumps(out),flush=True);n.destroy_node();rclpy.shutdown()
