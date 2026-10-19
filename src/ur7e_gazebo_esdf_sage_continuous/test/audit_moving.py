"""Independent whole-arm audit against recorded moving-object ground truth."""
import argparse,json
from pathlib import Path
from dataclasses import replace
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.partial_map import load_snapshot
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.trajectory import edge_clear
from design_scan import Fixture
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
events=json.loads((out/'moving_scenario.json').read_text());truth=[json.loads(x) for x in (out/'moving_truth.jsonl').read_text().splitlines()]
rows=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8');q=np.stack([rows[n] for n in rows.dtype.names[4:10]],-1);times=rows['sim_time']
static=np.inf;selfgap=np.inf
for i in range(0,len(q),256):
 centers=g.spheres_batch(q[i:i+256]);static=min(static,float((Fixture().query(centers)-g.radii).min()));selfgap=min(selfgap,float(g.self_clearance_batch(centers).min()))
minima={};speed={};samples={}
for trial in ['crossing','blocked']:
 tr=[t for t in truth if t['trial']==trial]
 if not tr:continue
 stamps=np.array([t['stamp'] for t in tr]);pos=np.array([t['position'] for t in tr]);chosen=(times>=stamps[0])&(times<=stamps[-1]);tt=times[chosen];qq=q[chosen]
 pp=np.stack([np.interp(tt,stamps,pos[:,k]) for k in range(3)],-1);minimum=np.inf
 for i in range(0,len(qq),256):
  cs=g.spheres_batch(qq[i:i+256]);delta=abs(cs-pp[i:i+256,None,:])-.04;dist=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)-g.radii
  minimum=min(minimum,float(dist.min()))
 minima[trial]=minimum;samples[trial]=len(qq);speed[trial]=float(np.linalg.norm(np.diff(pos,axis=0)/np.diff(stamps)[:,None],axis=-1).max())
updates=[json.loads(x) for x in (out/'online/updates.jsonl').read_text().splitlines()];t=np.array([u['commit_sim_time'] for u in updates]);lat=np.array([u['latency'] for u in updates]);gaps=np.diff(t)
feedback=json.loads((out/'sage_feedback.json').read_text());path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path);cache={0:base};checked=0;violations=[]
for record in feedback:
 version=record['execute_map_version']
 if version not in cache:
  with np.load(out/'online'/f'version_{version:06d}.npz') as d:occupied=d['occupied']
  dynamic=distance_transform_edt(~occupied)*base.resolution-np.sqrt(3)*base.resolution/2-.02 if occupied.any() else np.full(occupied.shape,np.inf)
  cache[version]=replace(base,distance=np.minimum(base.distance,dynamic).astype(np.float32))
 if not edge_clear(g,cache[version],record['q'],record['next'],c['collision_margin'],c['self_margin']):violations.append(record['iteration'])
 checked+=1
r=dict(scenario_complete=events[-1]['kind']=='complete',min_static_gap_m=static,min_self_gap_m=selfgap,min_moving_gap_m=minima,moving_samples=samples,max_obstacle_speed_m_s=speed,online_frames=len(updates),online_versions=max(u['version'] for u in updates),online_commit_gap_max_s=float(max(gaps)),online_commit_gap_p95_s=float(np.quantile(gaps,.95)),online_latency_p95_s=float(np.quantile(lat,.95)),online_latency_max_s=float(max(lat)),executed_steps_checked=checked,recorded_map_edge_violations=violations)
(out/'moving_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
assert r['scenario_complete'];assert static>=.03 and selfgap>=.015 and all(x>=.03 for x in minima.values());assert not violations
assert max(gaps)<1.,'Online update continuity failed'
assert all(v<.016 for v in speed.values()),'Object exceeded low speed bound'
