#!/usr/bin/python3
"""Read-only summary of the single bounded diagnostic bag and Nav2 trace."""
import bisect
import collections
import json
import math
import re
import sqlite3
import statistics
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import VoxelGrid
from rosgraph_msgs.msg import Clock

ROOT=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/user/arena_local_costmap_diag_20261001/run_20261001')
DB=ROOT/'raw_rosbag/raw_rosbag_0.db3'
TYPES={'/tf':TFMessage,'/tf_static':TFMessage,'/clock':Clock,
       '/task_generator_node/jackal/lidar':LaserScan,
       '/task_generator_node/jackal/local_costmap/costmap':OccupancyGrid,
       '/task_generator_node/jackal/local_costmap/voxel_grid':VoxelGrid}
def voxel_counts(words, size_z):
    mask = (1 << size_z) - 1
    marked = sum(((int(x) >> 16) & mask).bit_count() for x in words)
    unknown = sum(((int(x) & mask) & ~((int(x) >> 16) & mask)).bit_count() for x in words)
    free = len(words) * size_z - marked - unknown
    return marked, unknown, free

# nav2_voxel_grid: 11 marked, 01 unknown, 00 free.
assert voxel_counts([1], 1) == (0, 1, 0)
assert voxel_counts([0x10001], 1) == (1, 0, 0)
assert voxel_counts([0], 1) == (0, 0, 1)

def stamp(x): return x.sec+x.nanosec*1e-9
conn=sqlite3.connect(f'file:{DB}?mode=ro',uri=True)
names={i:n for i,n in conn.execute('select id,name from topics')}
tf=collections.defaultdict(list);scans=[];clocks=[];grids=[];voxels=[]; tf_bad=collections.Counter()
for tid,ts,raw in conn.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name=names[tid]
    if name not in TYPES: continue
    msg=deserialize_message(raw,TYPES[name]); wall=ts*1e-9
    if name in ('/tf','/tf_static'):
        for item in msg.transforms:
            tf[(name,item.header.frame_id,item.child_frame_id)].append((stamp(item.header.stamp),wall))
            if not item.header.frame_id: tf_bad[item.child_frame_id]+=1
    elif name=='/clock': clocks.append((wall,stamp(msg.clock)))
    elif name.endswith('/lidar'):
        near=sum(msg.range_min<=r<=min(msg.range_max,2.5) for r in msg.ranges)
        finite=sum(math.isfinite(r) and msg.range_min<=r<=msg.range_max for r in msg.ranges)
        scans.append({'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                      'sim':stamp(msg.header.stamp),'wall':wall,'frame':msg.header.frame_id,
                      'beam_count':len(msg.ranges),'finite':finite,'near':near,
                      'ranges':list(msg.ranges),'angle_min':msg.angle_min,'angle_increment':msg.angle_increment})
    elif name.endswith('/costmap'):
        data=list(msg.data); grids.append({'sim':stamp(msg.header.stamp),'wall':wall,'occupied':sum(x==100 for x in data),
            'max':max(data),'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
            'size':[msg.info.width,msg.info.height],'resolution':msg.info.resolution,
            'data':data})
    elif name.endswith('/voxel_grid'):
        data=list(msg.data)
        marked, unknown, free = voxel_counts(data, msg.size_z)
        voxels.append({'sim':stamp(msg.header.stamp),'wall':wall,'marked':marked,'unknown':unknown,
                       'free':free})
conn.close()
lines=(ROOT/'launch.log').read_text(errors='replace').splitlines()
markers=collections.defaultdict(list)
for line in lines:
    if 'ARENA_DIAG_' not in line: continue
    side='local' if 'local_costmap.local_costmap' in line else 'global' if 'global_costmap.global_costmap' in line else 'other'
    kind=re.search('ARENA_DIAG_([A-Z_]+)',line).group(1)
    kv=dict(re.findall(r'([a-z_]+)=([^ ]+)',line))
    markers[(side,kind)].append(kv)
local={kind:arr for (side,kind),arr in markers.items() if side=='local'}
callbacks={int(x['stamp_ns']):x for x in local.get('SCAN_CALLBACK',[])}
drops={int(x['stamp_ns']):x for x in local.get('TF_DROP',[])}
accepts={int(x['stamp_ns']):x for x in local.get('TF_ACCEPT',[])}
clocks_w=[x[0] for x in clocks]
def sim_at(w):
    i=bisect.bisect_right(clocks_w,w)-1
    return clocks[i][1] if i>=0 else None
wait=[]
for s,x in drops.items():
    if s in callbacks:
        c=callbacks[s]
        wait.append({'stamp_ns':s,'wall_s':(int(x['wall_ns'])-int(c['wall_ns']))*1e-9,
                     'sim_s':(int(x['sim_ns'])-int(c['sim_ns']))*1e-9})
edge=tf[('/tf','jackal/chassis_link','jackal/lidar_link')]
edge_w=[x[1] for x in edge]
edge_lag=[{'tf_stamp':t,'receipt_wall':w,'sim_at_receipt':sim_at(w),'lag_sim':sim_at(w)-t if sim_at(w) else None} for t,w in edge]
scanrows=[]
for s in scans:
    t=s['sim']; req=t+.05
    future=next(((x,y) for x,y in edge if x>=req),None)
    scanrows.append({k:v for k,v in s.items() if k!='ranges'} | {'callback':s['stamp_ns'] in callbacks,
       'tf_drop':s['stamp_ns'] in drops,'tf_accept':s['stamp_ns'] in accepts,
       'future_lidar_tf':future[0] if future else None,
       'future_lidar_tf_receipt_wall_delay':round(future[1]-s['wall'],3) if future else None})
summary={
 'bag':str(DB),'scans':len(scans),'scan_near_frames':sum(s['near']>0 for s in scans),
 'scan_near_returns':sum(s['near'] for s in scans),'scan_finite_returns':sum(s['finite'] for s in scans),
 'scan_callback_received':len(callbacks),'tf_accepted':len(accepts),'tf_dropped':len(drops),
 'tf_unresolved':len(set(callbacks)-set(drops)-set(accepts)),
 'tf_drop_reasons':dict(collections.Counter(x.get('reason_code') for x in drops.values())),
 'drop_wait_wall_s':{'min':min(x['wall_s'] for x in wait),'median':statistics.median(x['wall_s'] for x in wait),
                     'max':max(x['wall_s'] for x in wait)} if wait else None,
 'drop_wait_sim_s':{'min':min(x['sim_s'] for x in wait),'median':statistics.median(x['sim_s'] for x in wait),
                     'max':max(x['sim_s'] for x in wait)} if wait else None,
 'marker_counts':{kind:len(arr) for kind,arr in local.items()},
 'voxel_cycles':len(local.get('VOXEL_CYCLE',[])),
 'voxel_cycle_max':{k:max(int(x.get(k,0)) for x in local.get('VOXEL_CYCLE',[])) for k in ['marking_obs','marking_input','range_pass','map_pass','mark_voxel_calls','mark_cell_calls','clearing_obs','clearing_input','voxels_after_mark','lethal_after_mark']},
 'combine_max':{k:max(int(x.get(k,0)) for x in local.get('COMBINE',[])) for k in ['layer_lethal','master_after']},
 'local_grids':len(grids),'nonzero_grids':sum(g['occupied']>0 for g in grids),'max_occupied_cells':max(g['occupied'] for g in grids),
 'voxel_grids':len(voxels),'marked_voxel_grids':sum(v['marked']>0 for v in voxels),'max_marked_voxels':max(v['marked'] for v in voxels),
 'max_unknown_voxels':max(v['unknown'] for v in voxels),'max_free_voxels':max(v['free'] for v in voxels),
 'tf_edge_lidar':edge_lag,'empty_parent_tf_counts':dict(tf_bad),
 'scan_trace':scanrows,
}
# Track the first near wall scan through the next available output grids.
s=next(s for s in scans if s['near']>0)
nextgrid=next((g for g in grids if g['wall']>=s['wall']),None)
nextvox=next((v for v in voxels if v['wall']>=s['wall']),None)
summary['example']={'scan':next(row for row in scanrows if row['stamp_ns']==s['stamp_ns']),
                    'tf_drop':drops.get(s['stamp_ns']),
                    'next_grid':{k:v for k,v in nextgrid.items() if k!='data'} if nextgrid else None,
                    'next_voxel_grid':nextvox,
                    'first_near_beams':[(i,round(s['angle_min']+i*s['angle_increment'],4),round(r,4))
                       for i,r in enumerate(s['ranges']) if math.isfinite(r) and .08<=r<=2.5][:12]}
out=ROOT/'trace_analysis.json';out.write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
for k,v in summary.items():
    if k not in ('scan_trace','tf_edge_lidar','empty_parent_tf_counts'):
        print(k,v)
print('tf_edge_lidar',edge_lag)
print('empty_parent_tf_counts',dict(tf_bad))
