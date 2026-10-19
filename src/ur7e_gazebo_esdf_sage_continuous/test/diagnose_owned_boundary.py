"""Read-only geometric witness: interior points rejected by whole-cell ownership.

This does not relax classification or edit a map. A sphere centered at the
shoulder joint origin is invariant under shoulder pan; its inscribed radius
is proven directly by the verified convex mesh half-spaces.
"""
from pathlib import Path
import argparse,json,numpy as np
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.ownership import RobotOwnership,load_snapshot
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
parser=argparse.ArgumentParser();parser.add_argument('output');args=parser.parse_args();out=Path(args.output);c=load_mapping(out/'mapping.yaml');p=next((out/'maps').glob('*/robot_ownership.npz'));f=FrozenMap.load(p.parent/'frozen_map.npz',identity(c));d=np.load(p);m=json.loads(str(d['metadata']));g=Geometry(c['robot_urdf'],c['collision_spheres']);q=np.array(m['freeze_q']);model=RobotOwnership(m['urdf'],g)
link,vertices,normals,faces=next(x for x in model.solids if x[0]=='shoulder_link');radius=float(np.min(np.sum(normals*faces,axis=1)));center=g.fk(q)[link][:3,3]
z,y,x=np.indices(f.snapshot.distance.shape);points=f.snapshot.origin+f.snapshot.resolution*np.stack([x,y,z],-1).reshape(-1,3)
core=np.linalg.norm(points-center,axis=1)<radius-1e-8
ids=np.flatnonzero(core & ~d['owned'] & (f.source==0));ball=g.spheres(q)[0];assert len(ids), 'No such boundary witness in this snapshot';j=ids[np.argmin(np.linalg.norm(points[ids]-ball,axis=1))]
r=dict(mode='offline structural diagnostic; never a planning map',core_center=center.tolist(),core_radius_m=radius,unowned_unknown_centers_inside_invariant_core=len(ids),nearest_witness=points[j].tolist(),witness_distance_to_ball_center_m=float(np.linalg.norm(points[j]-ball)),shoulder_ball_radius_m=float(g.radii[0]),required_margin_m=c['collision_margin'],points=points[ids].tolist())
print(json.dumps(r,indent=2));(out/'partial_voxel_diagnostic.json').write_text(json.dumps(r,indent=2))
