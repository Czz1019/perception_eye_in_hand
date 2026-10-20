"""Audit new semantics using recorded depth views and saved native ESDF.

Does not load the old planning distances/sources. This is not a fresh Gazebo run.
"""
import argparse,json
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.field import FieldBuilder,PLANNING_POLICY
from ur7e_gazebo_esdf_sage_continuous.self_filter import mask
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_nvblox.depth import intrinsics_from_info,known_distances
p=argparse.ArgumentParser();p.add_argument('session');p.add_argument('output');a=p.parse_args();session=Path(a.session);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1];c=load_mapping(root/'config/mapping.yaml');c['free_space_backend']='cpu'
g=Geometry(c['robot_urdf'],c['collision_spheres']);f=FieldBuilder(c);extra=yaml.safe_load((root/'config/scan.yaml').read_text())['depth_self_mask']
with np.load(session/'native_esdf.npz') as data:
 raw=data['distance'].copy();known=known_distances(raw,float(data['unknown']));np.testing.assert_allclose(f.points,data['points'],atol=1e-6)
records=[]
for folder in sorted((session/'observations').iterdir()):
 record=json.loads((folder/'pose.json').read_text());info=record['camera_info'];intr=intrinsics_from_info(SimpleNamespace(**info),c['depth_stride'])
 with np.load(folder/'depth.npz') as data:depth=data['meters'][::c['depth_stride'],::c['depth_stride']].copy()
 depth[~np.isfinite(depth)|(depth<c['min_depth'])|(depth>c['max_depth'])]=0.
 ray_depth=depth.copy() if c['tabletop'].get('ray_evidence')=='raw_depth_before_returns' else None
 q=np.array(record['q_rad']);pose=np.array(record['T_base_depth']);centers=g.spheres(q)
 depth[mask(depth,pose,intr,centers,g.radii,g.link_ids,extra)]=0.
 d,s=f.update(raw,known,depth,pose,intr,ray_depth=ray_depth);records.append(record)
frozen=FrozenMap.create(d,s,f.origin,f.dims,c['grid_resolution'],dict(identity=identity(c),planning_policy=PLANNING_POLICY,stamp=records[-1]['stamp'],epoch=1,sequence=len(records),source='recorded_view_replay',observations=records))
frozen.save(out/'frozen_map.npz');np.savez_compressed(out/'tabletop_evidence.npz',**f.evidence())
centers=g.spheres(records[-1]['q_rad']);gap=frozen.snapshot.query(centers)-g.radii
result=dict(mode='16 saved depth views plus final native ESDF; old planning field never read',planning_policy=PLANNING_POLICY,observations=len(records),observed_cells=int((s==1).sum()),tabletop_cells=int((s==2).sum()),scan_prior_cells=0,reliable_columns=int(f.reliable.sum()),start_min_gap_m=float(gap.min()),blocked_spheres=[dict(index=int(i),link=g.links[g.link_ids[i]],center=centers[i].tolist(),gap_m=float(gap[i])) for i in np.flatnonzero(gap<c['collision_margin'])])
(out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
