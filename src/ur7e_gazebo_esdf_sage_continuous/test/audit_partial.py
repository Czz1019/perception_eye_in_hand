"""Independent policy and actual trajectory audit; no fixture geometry input."""
import argparse,json
from pathlib import Path
from dataclasses import replace
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.partial_map import load_snapshot
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry
from ur7e_nvblox.depth import known_distances
p=argparse.ArgumentParser();p.add_argument('output');args=p.parse_args();out=Path(args.output);c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/partial_policy.npz'));session=path.parent
f=FrozenMap.load(session/'frozen_map.npz',identity(c));snapshot=load_snapshot(f,path)
with np.load(path) as d:metadata=json.loads(str(d['metadata']));admitted=d['admitted'];obstacle=d['obstacle'];uncertainty=d['uncertainty']
with np.load(session/'robot_ownership.npz') as d:owned=d['owned']
with np.load(session/'native_esdf.npz') as d:raw=d['distance'];known=known_distances(raw,float(d['unknown']))
occupied=known&(raw<=.02);observed=(f.source>0)&~occupied
nearest=distance_transform_edt(~observed.reshape(obstacle.shape))*snapshot.resolution
explored=admitted&~owned&~observed
assert np.all(nearest.reshape(-1)[explored]<=metadata['unknown_band_m']+1e-9)
assert not np.any(admitted&occupied)
assert np.all(snapshot.distance.reshape(-1)[occupied]<=0)
assert np.all(snapshot.distance<=obstacle)
assert np.all(f.source[explored]==0)
rows=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8');planning=np.isin(rows['phase'],['REACHING','GOAL_REACHED','SOLVING_GUIDE']);rows=rows[planning]
g=Geometry(c['robot_urdf'],c['collision_spheres']);q=np.stack([rows[n] for n in rows.dtype.names[4:10]],axis=-1)
obstacle_snapshot=replace(snapshot,distance=obstacle);min_obstacle=np.inf;min_policy=np.inf;max_unknown=0.;unknown_count=0;point_count=0
# Audit center + six extrema; these samples quantify exposure, not replace the
# complete-sphere hard feasibility/continuous-segment checks in execution.
offsets=np.r_[np.zeros((1,3)),np.eye(3),-np.eye(3)]
for start in range(0,len(q),256):
 centers=g.spheres_batch(q[start:start+256]);min_obstacle=min(min_obstacle,float((obstacle_snapshot.query(centers)-g.radii).min()));min_policy=min(min_policy,float((snapshot.query(centers)-g.radii).min()))
 pts=(centers[...,None,:]+g.radii[None,:,None,None]*offsets).reshape(-1,3)
 ix=np.rint((pts-snapshot.origin)/snapshot.resolution).astype(int)
 assert np.all((ix>=0)&(ix<snapshot.dimensions))
 ids=np.ravel_multi_index(ix[:,::-1].T,obstacle.shape);unknown=(f.source[ids]==0)&~owned[ids]
 unknown_count+=int(unknown.sum());point_count+=len(ids)
 if unknown.any():max_unknown=max(max_unknown,float(nearest.reshape(-1)[ids[unknown]].max()))
r=dict(policy=metadata,planning_samples=len(q),environment_uid=f.uid,source_labels_preserved=True,
 admitted_unknown_cells=int(explored.sum()),max_admitted_unknown_grid_distance_m=float(nearest.reshape(-1)[explored].max()),
 min_planning_known_obstacle_gap_m=float(min_obstacle),min_planning_feasibility_gap_m=float(min_policy),
 uncertainty_sample_definition='sphere center and six axis extrema, nearest grid lookup; not volume fraction',
 unknown_sample_fraction=unknown_count/max(point_count,1),max_sampled_unknown_grid_distance_m=max_unknown)
(out/'partial_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
assert len(q)>0 and min_obstacle>=.029 and min_policy>=.029
assert max_unknown<=metadata['unknown_band_m']+snapshot.resolution*np.sqrt(3)+1e-8
