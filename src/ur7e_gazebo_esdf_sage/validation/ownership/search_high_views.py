import numpy as np,yaml,json,sys,time
from scipy.optimize import least_squares
sys.path.insert(0,'src/ur7e_gazebo_esdf_sage/test')
from design_scan import Fixture
from ur7e_gazebo_esdf_sage.config import load_mapping
from ur7e_real_static.model import Geometry
c=load_mapping('src/ur7e_gazebo_esdf_sage/config/mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres']);v=np.array(yaml.safe_load(open('src/ur7e_gazebo_esdf_sage/config/scan_coverage.yaml'))['views']);truth=Fixture();results=[]
for z in [.95,1.05]:
 target=np.array([0.,-.33,z]);axis=np.array([0.,0.,-1.])
 def fun(q):
  p=g.fk(q)[g.camera];cs=g.spheres(q)
  return np.r_[10*(p[:3,3]-target),1*(p[:3,2]-axis),.005*(q-v[0]),20*np.maximum(0,.09-(truth.query(cs)-g.radii)),100*max(0,.022-g.self_clearance(cs))]
 for seed in [*v[::4],*[v[0]+np.random.default_rng(i).normal(size=6)*[.3,.5,1.,1.,1.,1.] for i in range(30)]]:
  sol=least_squares(fun,np.clip(seed,g.lower+.01,g.upper-.01),bounds=(g.lower,g.upper),max_nfev=100)
  q=sol.x;p=g.fk(q)[g.camera];cs=g.spheres(q);env=(truth.query(cs)-g.radii).min();sg=g.self_clearance(cs)
  print('candidate',z,round(float(np.linalg.norm(p[:3,3]-target)),3),round(float(np.linalg.norm(p[:3,2]-axis)),3),round(float(env),3),round(float(sg),3),flush=True)
  if np.linalg.norm(p[:3,3]-target)>.05 or np.linalg.norm(p[:3,2]-axis)>.35 or env<.08 or sg<.019:continue
  for i,a in enumerate(v):
   n=max(2,int(np.max(abs(a-q))/.002)+1);cs=g.spheres_batch(np.linspace(a,q,n));em=(truth.query(cs)-g.radii).min();sm=g.self_clearance_batch(cs).min()
   if em>.08 and sm>.02:
    r=dict(q=q.tolist(),anchor=i,camera=p[:3,3].tolist(),axis=p[:3,2].tolist(),path_env=float(em),path_self=float(sm));results.append(r);print(json.dumps(r),flush=True);break
open('/home/wqj/nvblox_output/gazebo_esdf_sage/ownership_offline_20261016/high_view_search.json','w').write(json.dumps(results,indent=2))
print('solutions',len(results),flush=True)
