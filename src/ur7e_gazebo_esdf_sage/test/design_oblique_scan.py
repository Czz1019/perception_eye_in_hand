"""Offline fixture certification of oblique scan poses; no planning-map input."""
from pathlib import Path
import json,numpy as np,yaml
from design_scan import Fixture
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage.field import scan_samples
root=Path(__file__).resolve().parents[1]
g=Geometry(root/'config/real_calibrated.urdf',root/'config/collision_spheres.yaml');truth=Fixture()
c=yaml.safe_load((root/'config/scan_coverage.yaml').read_text());seed=np.array(c['views'][0]);views=[np.array(q) for q in c['views']]
profiles=[[-1.8431118015077466,-.8478192603852042,1.2548725922918542,-3.889736459759497,.29779465582345094,-.766472363876342],[-1.8495540241531503,-.8640260739134707,.7517805489542492,-3.8469688473261936,.2753385766396387,-.6167448162260105]]
def gaps(a,b,step=.002):
 n=max(2,int(np.ceil(np.max(abs(b-a))/step))+1);qs=np.linspace(a,b,n);cs=g.spheres_batch(qs)
 return float((truth.query(cs)-g.radii).min()),float(g.self_clearance_batch(cs).min())
selected=[]
for profile in profiles:
 candidates=[]
 for base in np.arange(-5.84,.161,.5):
  q=np.array(profile);q[0]=base
  env,sg=gaps(q,q)
  if env>.085 and sg>.02 and np.all((q>=g.lower)&(q<=g.upper)):candidates.append(q)
 current=seed
 while candidates and len(views)<62:
  candidates.sort(key=lambda q:np.linalg.norm(q-current))
  found=None
  for q in candidates:
   env,sg=gaps(current,q)
   back_env,back_self=gaps(q,seed)
   if min(env,back_env)>.085 and min(sg,back_self)>.02:found=q;break
  if found is None:break
  candidates.pop(next(i for i,x in enumerate(candidates) if x is found))
  views.append(found);selected.append(dict(q=found.tolist(),camera=g.fk(found)[g.camera][:3,3].tolist()));current=found
 if not np.array_equal(current,seed):views.append(seed.copy())
qs=scan_samples(np.array(views),step=.00025);envs=[];selves=[]
for start in range(0,len(qs),512):
 cs=g.spheres_batch(qs[start:start+512]);envs.append(float((truth.query(cs)-g.radii).min()));selves.append(float(g.self_clearance_batch(cs).min()))
padding=float(g.motion_bound*np.max(np.sum(abs(np.diff(qs,axis=0)),axis=1))/2)
assert min(envs)-padding>=c['prior_clearance']
assert min(selves)-2*padding>=.015
c['views']=[q.tolist() for q in views];c['commissioning']=dict(c['commissioning'],min_environment_gap=min(envs),min_self_gap=min(selves),sampling_padding=padding,samples=len(qs))
(root/'config/scan_oblique.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
r=dict(views=len(views),samples=len(qs),environment_gap=min(envs),self_gap=min(selves),padding=padding,added=selected)
(root/'validation/oblique_scan_design.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
