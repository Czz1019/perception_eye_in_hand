"""Offline observation-pose design only. No ROS node, RTDE connection or controller."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.ndimage import minimum_filter
from ur7e_real_static.model import Geometry

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
SRC=ROOT/'log/real_reconstruction_20260928/real/scan_20260928_130419_942f86'
M=json.loads((SRC/'metadata.json').read_text());g=Geometry(M['config']['robot_urdf'],M['config']['collision_spheres'])
oldq=np.array([v['q_rad'] for v in M['views']]);oldp=np.array([v['T_base_depth'] for v in M['views']])
with np.load(SRC/'surface.npz',allow_pickle=False) as f:V=f['vertices'].copy();F=f['triangles'].copy()
TREE=cKDTree(V);EDGE=np.linalg.norm(V[F]-V[np.roll(F,1,axis=1)],axis=2).max()
# A uniform-area surface sample avoids rewarding dense meshing of one object.
rng=np.random.default_rng(9);tri=V[F];area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/2
ids=rng.choice(len(F),30000,p=area/area.sum());u=rng.random(30000);v=rng.random(30000);swap=u+v>1;u[swap]=1-u[swap];v[swap]=1-v[swap]
P=tri[ids,0]+u[:,None]*(tri[ids,1]-tri[ids,0])+v[:,None]*(tri[ids,2]-tri[ids,0])
ci=M['views'][0]['color_info'];fx,fy,cx,cy=np.array(ci['k'])[[0,4,2,5]]/2;W,H=ci['width']//2,ci['height']//2

def visibility(T):
    pc=(P-T[:3,3])@T[:3,:3];z=pc[:,2]
    ix=np.rint(fx*pc[:,0]/np.maximum(z,1e-6)+cx).astype(int);iy=np.rint(fy*pc[:,1]/np.maximum(z,1e-6)+cy).astype(int)
    good=(z>=.2)&(z<=3)&(ix>=1)&(ix<W-1)&(iy>=1)&(iy<H-1)
    zbuf=np.full((H,W),np.inf);np.minimum.at(zbuf,(iy[good],ix[good]),z[good])
    zbuf=minimum_filter(zbuf,size=3)
    vis=np.zeros(len(P),bool);vis[good]=z[good]<=zbuf[iy[good],ix[good]]+.025
    return vis

def clearance(q):
    c=g.spheres(q);d=TREE.query(c)[0]-g.radii-EDGE
    return float(d.min()),g.self_clearance(c)

def jacobian_sigma(q):
    a=g.fk(q)[g.camera];J=np.zeros((6,6));eps=1e-5
    for j in range(6):
        b=q.copy();b[j]+=eps;b=g.fk(b)[g.camera]
        J[:3,j]=(b[:3,3]-a[:3,3])/eps
        J[3:,j]=.3*Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).as_rotvec()/eps
    return float(np.linalg.svd(J,compute_uv=False)[-1])

candidates=[];masks=[]
# Keep one elbow-positive joint branch and retain the observed wrist branch. Camera/wrist model alarms are recorded,
# not cleared or used to claim execution readiness. Optical roll is free during IK.
lower=np.maximum(g.lower,np.deg2rad([-150,-160,25,-285,-150,-175]))
upper=np.minimum(g.upper,np.deg2rad([20,-35,145,-80,-20,-50]))
for x in [.22,.38,.54]:
 for y in [-.45,-.2,.05,.25]:
  for z in [.63,.73]:
   for dy in [-.12,0,.12]:
    p=np.array([x,y,z]);target=p+np.array([.16,dy,.12-z]);axis=target-p;axis/=np.linalg.norm(axis)
    def task_residual(q):
        T=g.fk(q)[g.camera]
        return np.r_[T[:3,3]-p,.4*(T[:3,2]-axis)]
    order=np.argsort(np.linalg.norm(oldp[:,:3,3]-p,axis=1))
    best=None
    for seed in [oldq[order[0]],oldq[0],oldq[13]]:
        seed=np.clip(seed,lower+1e-4,upper-1e-4)
        sol=least_squares(lambda q:np.r_[task_residual(q),.0002*(q-seed)],seed,bounds=(lower,upper),max_nfev=110,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        q=sol.x
        if np.linalg.norm(task_residual(q))<1e-4:
            env,selfgap=clearance(q);sig=jacobian_sigma(q)
            centers=g.spheres(q);arm=(g.link_ids[g.pair_a]<6)&(g.link_ids[g.pair_b]<6)
            armgap=np.min(np.linalg.norm(centers[g.pair_a[arm]]-centers[g.pair_b[arm]],axis=1)-g.radii[g.pair_a[arm]]-g.radii[g.pair_b[arm]])
            if env>.045 and armgap>.02 and sig>.025:
                cost=np.linalg.norm(q-oldq[order[0]])
                if best is None or cost<best[0]:best=(cost,q,env,selfgap,sig)
    if best is None:continue
    _,q,env,selfgap,sig=best;poses=g.fk(q);dc=poses[g.camera];cc=poses['camera_color_optical_frame'];vis=visibility(cc)
    if vis.mean()<.06:continue
    rec=dict(id=f'C{len(candidates)+1:02d}',q_rad=q.tolist(),T_base_depth=dc.tolist(),T_base_color=cc.tolist(),T_base_tool0=poses['tool0'].tolist(),
             env_surface_lower_bound_m=env,self_model_gap_m=selfgap,jacobian_sigma_scaled=sig,visible_fraction=float(vis.mean()),
             aim_point_base=target.tolist(),nearest_recorded_view=int(order[0]+1))
    candidates.append(rec);masks.append(vis)
    print(rec['id'],np.round(p,2),dy,'cover',round(vis.mean(),3),'env',round(env,3),flush=True)
np.savez_compressed(OUT/'candidate_arrays.npz',points=P,visibility=np.asarray(masks),reference_visibility=np.asarray([visibility(g.fk(q)['camera_color_optical_frame']) for q in oldq]))
(OUT/'candidates.json').write_text(json.dumps(dict(candidates=candidates,mesh_max_triangle_edge_m=float(EDGE),surface_area_m2=float(area.sum()),sample_count=len(P)),indent=2)+'\n')
print('DONE',len(candidates),flush=True)
