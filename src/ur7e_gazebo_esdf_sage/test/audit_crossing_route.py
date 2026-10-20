"""Check whether obstacle sweep intersects a sensor-only static baseline route."""
import argparse,json
from pathlib import Path
import numpy as np
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from ur7e_gazebo_esdf_sage.partial_map import load_snapshot
from ur7e_gazebo_esdf_sage.planning import route
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output);c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres']);path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path)
e=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()];guides=[x for x in e if x['kind']=='guide' and x['revision']==1];first=guides[0];start=np.array(first['joints'][0]);goal=np.array(first['goal']);baseline=route(g,base,start,goal,f.metadata['views'],c)
qs=np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.max(abs(b-a))/.01))+1)) for a,b in zip(baseline,baseline[1:])]);cs=g.spheres_batch(qs)
truth=[json.loads(x) for x in (out/'moving_truth.jsonl').read_text().splitlines()];positions=np.array([x['position'] for x in truth if x['trial']=='crossing'])[::10]
minimum=np.inf
for position in positions:
 d=abs(cs-position)-.04;minimum=min(minimum,float((np.linalg.norm(np.maximum(d,0),axis=-1)+np.minimum(d.max(-1),0)-g.radii).min()))
r=dict(baseline_waypoints=len(baseline),online_guide_waypoint_counts=[len(x['joints']) for x in guides],baseline_vs_obstacle_swept_volume_min_gap_m=minimum,interpretation='Spatial crossing only; this counterfactual does not prove time-coincident collision.')
(out/'crossing_route_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));assert minimum<.03,'Scenario did not intersect baseline route safety envelope'
