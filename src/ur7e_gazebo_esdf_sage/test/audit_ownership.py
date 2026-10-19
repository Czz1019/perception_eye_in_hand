"""Post-run audit of body interiors, with an independent hull/corner check."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from ur7e_gazebo_esdf_sage.ownership import RobotOwnership,load_snapshot
from design_scan import Fixture

p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/robot_ownership.npz'));session=path.parent
f=FrozenMap.load(session/'frozen_map.npz',identity(c));snap=load_snapshot(f,path)
with np.load(path,allow_pickle=False) as d: owned=d['owned'].copy();metadata=json.loads(str(d['metadata']))
g=Geometry(c['robot_urdf'],c['collision_spheres']);model=RobotOwnership(metadata['urdf'],g)
q=np.array(metadata['freeze_q']);poses=g.fk(q)
z,y,x=np.indices(snap.distance.shape);points=snap.origin+snap.resolution*np.stack((x,y,z),-1).reshape(-1,3)
selected=points[owned];corners=np.array(np.meshgrid(*[[-snap.resolution/2,snap.resolution/2]]*3)).reshape(3,-1).T
verified=np.zeros(len(selected),bool)
for link,vertices,_,_ in model.solids:
 t=poses[link];world=vertices@t[:3,:3].T+t[:3,3]
 hull=ConvexHull(world);planes=hull.equations
 for start in range(0,len(selected),128):
  pts=selected[start:start+128,None]+corners
  verified[start:start+128]|=np.all(pts@planes[:,:3].T+planes[:,3]<=1e-9,axis=(1,2))
external_unknown=(f.source==0)&~owned
with np.load(session/'native_esdf.npz') as d:
 from ur7e_nvblox.depth import known_distances
 occupied=known_distances(d['distance'],float(d['unknown']))&(d['distance']<=.02)
gaps=snap.query(g.spheres(q))-g.radii
external=[]
# Distinguish real exterior gaps from conservative voxel-boundary rejection.
centres=g.spheres(q)
near=np.zeros(len(points),bool)
for centre,radius in zip(centres,g.radii):
 near|=np.linalg.norm(points-centre,axis=1)<=radius+c['collision_margin']
ids=np.flatnonzero(near & (f.source==0) & ~occupied)
inside_centres,_=model.mask(points[ids],0.,q)
exterior_points=points[ids[~inside_centres]]
for i,(centre,radius) in enumerate(zip(centres,g.radii)):
 distances=np.linalg.norm(exterior_points-centre,axis=1)
 selected_ids=np.flatnonzero(distances<=radius+c['collision_margin'])
 if len(selected_ids):
  j=selected_ids[np.argmin(distances[selected_ids])]
  external.append(dict(index=i,count=len(selected_ids),nearest=exterior_points[j].tolist(),distance_to_ball_center_m=float(distances[j])))
r=dict(environment_map_uid=f.uid,collision_uid=metadata['collision_uid'],owned_cells=int(owned.sum()),
 owned_cells_outside_verified_solid=int((~verified).sum()),external_unknown_cells_with_positive_distance=int(np.count_nonzero(snap.distance.reshape(-1)[external_unknown]>0)),
 owned_cells_overriding_measured_obstacles=int(np.count_nonzero(owned&occupied)),owned_cells_inside_fixture=int(np.count_nonzero(Fixture().query(selected)<=0)),
 start_min_gap_m=float(gaps.min()),blocked_ball_indices=np.flatnonzero(gaps<c['collision_margin']).tolist(),per_ball_gap_m=gaps.tolist(),external_unknown_gaps=external)
(out/'ownership_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
assert r['owned_cells_outside_verified_solid']==0
assert r['external_unknown_cells_with_positive_distance']==0
assert r['owned_cells_overriding_measured_obstacles']==0
assert r['owned_cells_inside_fixture']==0
