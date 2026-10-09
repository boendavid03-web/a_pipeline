"""Read-only observation of the native child; callbacks retain original results."""
import hashlib,inspect,json,marshal,os,pathlib,sys,time

def install(cls, step_fn, manifest, output):
    directory=pathlib.Path(output);directory.mkdir(parents=True,exist_ok=True)
    log=(directory/('sdk_'+str(os.getpid())+'.jsonl')).open('x',buffering=1)
    def emit(kind,**data):
        log.write(json.dumps({'kind':kind,'wall':time.monotonic(),**data},default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
    source=pathlib.Path(inspect.getfile(step_fn));sdk=pathlib.Path(inspect.getfile(cls))
    loaded={}
    for name,module in list(sys.modules.items()):
        p=pathlib.Path(getattr(module,'__file__','') or '.')
        if p.parent==source.parent and p.is_file():loaded[name]={'file':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
    emit('provenance',pid=os.getpid(),executable=sys.executable,argv=sys.argv,domain=os.environ.get('ROS_DOMAIN_ID'),planner_file=str(source),planner_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),step_code_sha256=hashlib.sha256(marshal.dumps(step_fn.__code__)).hexdigest(),loaded_planner_modules=loaded,sdk_file=str(sdk),sdk_sha256=hashlib.sha256(sdk.read_bytes()).hexdigest(),manifest=manifest,observer_file=__file__,observer_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest())
    torch=sys.modules.get('torch')
    if torch is not None:
        original_load=torch.load
        def load(f,*args,**kwargs):
            result=original_load(f,*args,**kwargs)
            p=pathlib.Path(f) if isinstance(f,(str,pathlib.Path)) else None
            emit('actual_model_load',path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest() if p else None,kwargs=kwargs,force_weights_only=os.environ.get('TORCH_FORCE_WEIGHTS_ONLY_LOAD'))
            return result
        torch.load=load
    original_obs=cls._handle_obs;original_send=cls._send_data;original_control=cls._handle_control
    def observe(self,frame,callback):
        f=frame.features
        features={k:v for k,v in f.items() if k in {'robot_pose','robot_state','goal_pose','pedestrians'}}
        forwards={};previous=sys.getprofile()
        def profile(call,event,arg):
            if event=='call' and call.f_code.co_name=='forward':
                obj=call.f_locals.get('self');name=type(obj).__name__
                if name in {'SRNN','_SARLValueNetwork'}:forwards[name]=forwards.get(name,0)+1
            if event=='call' and call.f_code.co_name=='act' and type(call.f_locals.get('self')).__name__=='Policy' and '/dsrnn/' in call.f_code.co_filename:
                emit('actual_neural_input',seq=frame.seq,sim=frame.t_sec+frame.t_nanosec*1e-9,file=call.f_code.co_filename,inputs={k:v.detach().cpu().tolist() for k,v in call.f_locals['inputs'].items()},rnn_hxs={k:v.detach().cpu().tolist() for k,v in call.f_locals.get('rnn_hxs',{}).items()},masks=call.f_locals['masks'].detach().cpu().tolist() if 'masks' in call.f_locals else None)
            if event=='call' and call.f_code.co_name=='predict' and type(call.f_locals.get('self')).__name__=='SARLPolicy':
                state=call.f_locals.get('state')
                if state is not None:emit('actual_policy_state',seq=frame.seq,sim=frame.t_sec+frame.t_nanosec*1e-9,file=call.f_code.co_filename,robot=vars(state.self_state),humans=[vars(h) for h in state.human_states])
        emit('obs',seq=frame.seq,sim=frame.t_sec+frame.t_nanosec*1e-9,active=self._active,features=features)
        def step(features):
            started=time.monotonic();result=callback(features)
            emit('step',seq=frame.seq,sim=frame.t_sec+frame.t_nanosec*1e-9,compute_wall_s=time.monotonic()-started,action=result,neural_forwards=forwards)
            return result
        sys.setprofile(profile)
        try:return original_obs(self,frame,step)
        finally:sys.setprofile(previous)
    def send(self,frame):
        result=original_send(self,frame)
        if hasattr(frame,'action'):emit('action_sent',seq=frame.seq,sim=frame.t_sec+frame.t_nanosec*1e-9,action_type=frame.action_type,action=frame.action)
        return result
    def control(self,frame,on_reset,on_cancel):
        result=original_control(self,frame,on_reset,on_cancel);emit('control',op=type(frame).__name__,episode_id=getattr(frame,'episode_id',None),active=self._active)
        return result
    cls._handle_obs=observe;cls._send_data=send;cls._handle_control=control
