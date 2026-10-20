"""Offline conservative subdivision experiment; never writes a planning map.

Unknown boxes are discarded only if wholly within a verified URDF solid.
Mixed boxes are subdivided; all unresolved exterior fragments remain blocking.
Distance to retained boxes is lower-bounded by distance to their circumspheres.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from ur7e_gazebo_esdf_sage.ownership import RobotOwnership
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry

p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--levels',type=int,default=4);a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/robot_ownership.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c))
with np.load(path) as d:m=json.loads(str(d['metadata']))
g=Geometry(c['robot_urdf'],c['collision_spheres']);q=np.array(m['freeze_q']);model=RobotOwnership(m['urdf'],g);poses=g.fk(q)
solids=[]
for link,v,n,fc in model.solids:
 t=poses[link];v=v@t[:3,:3].T+t[:3,3];n=n@t[:3,:3].T;fc=fc@t[:3,:3].T+t[:3,3]
 # Coplanar faces need one equivalent half-space only.
 planes=np.unique(np.round(np.c_[n,np.sum(n*fc,axis=1)],12),axis=0)
 solids.append((v.min(0),v.max(0),planes[:,:3],planes[:,3]))
def classify(points,res):
 inside=np.zeros(len(points),bool);possible=np.zeros(len(points),bool)
 for low,high,n,b in solids:
  ids=np.flatnonzero(np.all((points+res/2>=low)&(points-res/2<=high),axis=1)&~inside)
  support=res/2*abs(n).sum(axis=1)
  for start in range(0,len(ids),256):
   ix=ids[start:start+256];signed=points[ix]@n.T-b
   inside[ix]|=np.all(signed+support<=-1e-8,axis=1)
   possible[ix]|=~np.any(signed-support>1e-8,axis=1)
 return inside,possible&~inside
z,y,x=np.indices(f.snapshot.distance.shape);points=f.snapshot.origin+f.snapshot.resolution*np.stack([x,y,z],-1).reshape(-1,3)
with np.load(path.parent/'native_esdf.npz') as d:
 from ur7e_nvblox.depth import known_distances
 occupied=known_distances(d['distance'],float(d['unknown']))&(d['distance']<=.02)
unknown=points[(f.source==0)&~occupied];centers=g.spheres(q);low=f.snapshot.origin-f.snapshot.resolution/2;high=f.snapshot.origin+(np.array(f.snapshot.distance.shape[::-1])-.5)*f.snapshot.resolution;distance=np.minimum(centers-low,high-centers).min(axis=1);counts=[]
# Measured occupied cells cannot be removed by body classification.
if occupied.any():distance=np.minimum(distance,cKDTree(points[occupied]).query(centers)[0]-np.sqrt(3)*f.snapshot.resolution/2)
offsets=np.array(np.meshgrid(*[[-1.,1.]]*3)).reshape(3,-1).T
for level in range(a.levels+1):
 res=f.snapshot.resolution/2**level;inside,mixed=classify(unknown,res)
 keep=~inside if level==a.levels else ~inside&~mixed
 if keep.any():distance=np.minimum(distance,cKDTree(unknown[keep]).query(centers)[0]-np.sqrt(3)*res/2)
 counts.append(dict(level=level,resolution_m=res,discarded_interior=int(inside.sum()),retained_unknown=int(keep.sum()),mixed=int(mixed.sum())))
 print(counts[-1],flush=True)
 if level<a.levels:unknown=(unknown[mixed,None]+offsets*res/4).reshape(-1,3)
gaps=distance-g.radii
r=dict(mode='offline distance to represented unknown and occupied boxes only; native positive-distance cap not applied; no map mutation or controller use',levels=counts,per_ball_gap_bound_m=gaps.tolist(),blocked_ball_indices=np.flatnonzero(gaps<c['collision_margin']).tolist(),min_gap_bound_m=float(gaps.min()))
(out/'subvoxel_diagnostic.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
