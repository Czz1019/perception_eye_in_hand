"""Independent audit of every submitted short horizon and continuity metrics."""
import json,argparse
from pathlib import Path
from dataclasses import replace
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.partial_map import load_snapshot
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.trajectory import edge_clear
p=argparse.ArgumentParser();p.add_argument('output');out=Path(p.parse_args().output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path)
records=[json.loads(x) for x in (out/'sage_feedback.jsonl').read_text().splitlines()];events=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
cache={0:base};violations=[];edges=0
for r in records:
 version=r['execute_map_version']
 if version not in cache:
  with np.load(out/'online'/f'version_{version:06d}.npz') as d:occ=d['occupied']
  dynamic=distance_transform_edt(~occ)*base.resolution-np.sqrt(3)*base.resolution/2-.02 if occ.any() else np.full(occ.shape,np.inf)
  cache[version]=replace(base,distance=np.minimum(base.distance,dynamic).astype(np.float32))
 for a,b in zip(r['prefix'][:-1],r['prefix'][1:]):
  edges+=1
  if not edge_clear(g,cache[version],a,b,c['collision_margin'],c['self_margin']):violations.append(r['iteration'])
lat=np.array([r['latency'] for r in records]);summary={}
execution=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
commands=np.stack([execution[k] for k in execution.dtype.names[10:16]],-1)
times=execution['sim_time']
for target in [e for e in events if e['kind']=='target']:
 rev=target['revision'];rr=[r for r in records if r['revision']==rev];goal=next((e for e in events if e['kind']=='goal_reached' and e['revision']==rev),None)
 first=int(np.searchsorted(times,target['sim_time']));baseline=commands[max(0,first-1)];after=times>=target['sim_time'];moved=np.flatnonzero(after&(np.max(abs(commands-baseline),axis=1)>.001));first_motion=float(times[moved[0]]-target['sim_time']) if moved.size else None
 summary[rev]=dict(first_motion_over_1mrad_after_target_sim_s=first_motion,first_submission_after_target_sim_s=rr[0]['stamp']-target['sim_time'] if rr else None,reached=goal is not None,error_m=goal['error_m'] if goal else None,accepted=len(rr),accepted_while_background_search=sum(r['background_pending'] for r in rr))
result=dict(short_horizon_edges_checked=edges,violations=violations,nonzero_shift_submissions=sum(r['shift_steps']>0 for r in records),accepted_while_background_search=sum(r['background_pending'] for r in records),accepted_plan_latency_p50_s=float(np.median(lat)),accepted_plan_latency_p95_s=float(np.quantile(lat,.95)),accepted_plan_latency_max_s=float(lat.max()),guard_p95_s=float(np.quantile([r.get('guard_seconds',0) for r in records],.95)),goals=summary,stale_optimization_rejections=sum(e['kind']=='stale_plan' for e in events))
(out/'continuous_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert not violations
assert result['nonzero_shift_submissions']>0
assert all(x['reached'] for x in summary.values())
