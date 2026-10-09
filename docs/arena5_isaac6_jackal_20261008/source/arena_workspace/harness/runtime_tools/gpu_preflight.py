"""Read-only gate. Never stops training or starts a container."""
import json,os,pathlib,shutil,subprocess,sys
BASE=pathlib.Path('[LOCAL_PATH]');C='arena-arena_ws-arena-1'
def check():
 reasons=[];record={}
 # A3 records current host memory and project locks before any runtime launch.
 record['host_memory_bytes']={line.split(':',1)[0]:int(line.split()[1])*1024 for line in pathlib.Path('/proc/meminfo').read_text().splitlines() if line.split(':',1)[0] in {'MemTotal','MemAvailable','SwapTotal','SwapFree'}}
 record['project_runtime_locks']=[str(p) for p in (BASE/'reports').rglob('A_RUNTIME.lock')]
 if any(p!=os.environ.get('A3_OWNED_RUNTIME_LOCK') for p in record['project_runtime_locks']):reasons.append('An A runtime ownership lock exists')
 gpu=subprocess.check_output(['nvidia-smi','--query-gpu=memory.total,memory.free,utilization.gpu','--format=csv,noheader,nounits'],text=True).strip();record['gpu']=gpu
 processes=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader,nounits'],text=True).strip();record['gpu_compute_apps']=processes
 if processes:reasons.append('GPU compute owners remain; no co-existence acceptance')
 if any(int(line.split(',')[1])<24576 for line in gpu.splitlines()):reasons.append('GPU free memory below conservative 24 GiB gate')
 disk=shutil.disk_usage(BASE);required=max(50*2**30,int(disk.total*.15));record['disk']={'free_bytes':disk.free,'required_bytes':required}
 if disk.free<required:reasons.append('Filesystem reserve below required max(50GiB,15%)')
 state=json.loads(subprocess.check_output(['docker','inspect','arena-arena_ws-isaac-1'],text=True))[0]['State'];record['isaac']=state
 if state['Running']:reasons.append('Existing Isaac is running; ownership not available')
 code="import pathlib,json;rows=[]\nfor p in pathlib.Path('/proc').glob('[0-9]*'):\n try:\n  env=(p/'environ').read_bytes().split(b'\\0')\n  if b'ROS_DOMAIN_ID=189' in env:rows.append(int(p.name))\n except (OSError,PermissionError):pass\nprint(json.dumps(rows))"
 owners=json.loads(subprocess.check_output(['docker','exec',C,'/opt/venv/bin/python','-c',code],text=True));record['domain189_owners']=owners
 if owners:reasons.append('Reserved domain189 has processes')
 ports=[]
 for path in [pathlib.Path('/proc/net/udp'),pathlib.Path('/proc/net/udp6')]:
  for line in path.read_text().splitlines()[1:]:
   port=int(line.split()[1].split(':')[-1],16)
   if 54650<=port<54900:ports.append(port)
 record['host_reserved_udp_ports']=sorted(set(ports))
 portcode="import pathlib,json;ports=[]\nfor p in [pathlib.Path('/proc/net/udp'),pathlib.Path('/proc/net/udp6')]:\n for line in p.read_text().splitlines()[1:]:\n  port=int(line.split()[1].split(':')[-1],16)\n  if 54650<=port<54900:ports.append(port)\nprint(json.dumps(sorted(set(ports))))"
 record['container_reserved_udp_ports']=json.loads(subprocess.check_output(['docker','exec',C,'/opt/venv/bin/python','-c',portcode],text=True))
 if ports or record['container_reserved_udp_ports']:reasons.append('Reserved domain189 UDP range already bound')
 record['reasons']=reasons;record['status']='PASS' if not reasons else 'WAITING_FOR_RESOURCE_SLOT';return record
def require_safe():
 result=check();print(json.dumps(result),flush=True)
 if result['reasons']:raise SystemExit(75)
if __name__=='__main__':require_safe()
