import sys,json,numpy as np,xacro,yaml
from pathlib import Path
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
root=Path('/home/czz/perception_eye_in_hand/nvblox_in_hand');sys.path.insert(0,str(root/'src/ur7e_d435_gazebo/scripts'))
from orbit_geometry import Model,Fixture,Guard,JOINTS
pkg=root/'src/ur7e_d435_gazebo';model=Model(xacro.process_file(str(pkg/'urdf/ur7e_d435.urdf.xacro')).toxml());view=np.array([0,-1.57,1.1,-.90879335,-3.21580312,-.69520875]);old=model.camera_pose(view);R=old[:3,:3];guard=Guard(model,Fixture(pkg/'config/collision_world_gazebo_tall.yml'));initial=yaml.safe_load((pkg/'config/initial_positions.yaml').read_text());start=np.array([initial[j] for j in JOINTS]);attempts=[]
def solve(pos,seed):
 def fun(q):
  t=model.camera_pose(q);return np.r_[t[:3,3]-pos,(t[:3,2]-R[:,2])*.3]
 res=least_squares(fun,seed,bounds=(model.lower,model.upper),xtol=1e-11,gtol=1e-11,ftol=1e-11,max_nfev=500)
 if np.linalg.norm(fun(res.x))>1e-5:raise RuntimeError('IK error '+str(np.linalg.norm(fun(res.x))))
 return res.x
try:
 rng=np.random.default_rng(127);found=False
 print('Original gaps',guard.gaps([view]),'initial',guard.gaps([start]),flush=True)
 for height,x,amp in [(.82,.4,.08),(.8,.4,.08),(.78,.42,.06)]:
  position=np.array([x,old[1,3],height])
  for k in range(30):
   try:
    seed=view if k==0 else np.clip(view+rng.normal(0,.7 if k<15 else 1.5,6),model.lower+.01,model.upper-.01)
    center=solve(position,seed)
    if not guard.valid(center):continue
    ys=np.linspace(-amp,amp,41);qs=np.zeros((41,6));qs[20]=center
    for idx in range(21,41):qs[idx]=solve(position+[0,ys[idx],0],qs[idx-1])
    for idx in range(19,-1,-1):qs[idx]=solve(position+[0,ys[idx],0],qs[idx+1])
    if np.max(np.abs(np.diff(qs,axis=0)))>.15:raise RuntimeError('IK branch jump')
    if not all(guard.valid(q) for q in qs):raise RuntimeError('Invalid pose')
    if not all(guard.edge(a,b) for a,b in zip(qs,qs[1:])):raise RuntimeError('Invalid edge')
    route=guard.connect(start,center,rng)
    if route is None:raise RuntimeError('No collision-checked approach')
    plan={'old_pose':old.tolist(),'old_view_joints':view.tolist(),'center_joints':center.tolist(),'center_camera_pose':model.camera_pose(center).tolist(),'y_offsets_m':ys.tolist(),'joint_samples':qs.tolist(),'all_poses_valid':True,'all_edges_valid':True,'approach_route':[q.tolist() for q in route],'initial_joints':start.tolist(),'max_camera_error_m':max(np.linalg.norm(model.camera_pose(q)[:3,3]-(position+[0,y,0])) for y,q in zip(ys,qs)),'amplitude_m':amp,'attempts':attempts}
    (Path(__file__).parent/'motion_plan.json').write_text(json.dumps(plan,indent=2)+'\n');print('PLANNED',position,amp,'route',len(route),'error',plan['max_camera_error_m'],flush=True);found=True;break
   except RuntimeError as e:attempts.append({'height':height,'x':x,'amplitude':amp,'seed':k,'error':str(e)})
  if found:break
  print('No checked solution for height',height,flush=True)
 if not found:raise RuntimeError('No valid higher left-right trajectory')
finally:guard.oracle.close()
