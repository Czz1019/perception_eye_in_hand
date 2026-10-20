"""Offline test-case selection from a frozen map; never used by the controller."""
import argparse,json
from pathlib import Path
import numpy as np
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from design_scan import Fixture

p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
f=FrozenMap.load(next((out/'maps').rglob('frozen_map.npz')),identity(c));rng=np.random.default_rng(20261014)
anchors=np.array(f.metadata['views']);safe=[]
for batch in range(8):
    qs=anchors[rng.integers(len(anchors),size=2000)]+rng.normal(size=(2000,6))*[.25,.25,.45,.35,.35,.25]
    qs=np.clip(qs,g.lower,g.upper);centers=g.spheres_batch(qs)
    valid=(f.snapshot.query(centers)-g.radii).min(-1)>.04
    valid &= g.self_clearance_batch(centers)>.02
    for q in qs[valid]:
        xyz=g.fk(q)['tool0'][:3,3]
        if xyz[0]>.3 and xyz[2]<.9:safe.append(dict(q=q.tolist(),xyz=xyz.tolist()))
start=np.array(f.metadata['observations'][-1]['q_rad']);truth=Fixture()
safe.sort(key=lambda p:p['xyz'][2])
for candidate in safe[:80]:
    centers=g.spheres_batch(np.linspace(start,candidate['q'],120))
    candidate['direct_fixture_gap']=float((truth.query(centers)-g.radii).min())
(out/'candidate_targets.json').write_text(json.dumps(safe,indent=2))
print('valid candidates',len(safe));print(json.dumps(safe[:12],indent=2))
