"""Offline selection and sampled path audit; does not enable or implement execution."""
from pathlib import Path
import json,itertools,hashlib
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.ndimage import minimum_filter
import yaml
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS

D=Path(__file__).resolve().parent;ROOT=D.parents[2]
SRC=ROOT/'log/real_reconstruction_20260928/real/scan_20260928_130419_942f86'
meta=json.loads((SRC/'metadata.json').read_text());g=Geometry(meta['config']['robot_urdf'],meta['config']['collision_spheres'])
source=json.loads((D/'candidates.json').read_text());C=source['candidates'];arr=np.load(D/'candidate_arrays.npz');P=arr['points'];vis=arr['visibility'];reference=arr['reference_visibility']
with np.load(SRC/'surface.npz',allow_pickle=False) as f:V=f['vertices'];F=f['triangles']
tree=cKDTree(V);maxedge=source['mesh_max_triangle_edge_m']
arm=(g.link_ids[g.pair_a]<6)&(g.link_ids[g.pair_b]<6)

# Reference is the last SAVED scan pose, never assumed to be the current robot state.
q0=np.array(meta['views'][-1]['q_rad']);T0=g.fk(q0)[g.camera]
entryq=[q0.copy()];liftposes=[]
for height in np.linspace(T0[2,3],.70,19)[1:]:
 target=T0.copy();target[2,3]=height
 def fun(q):
  actual=g.fk(q)[g.camera]
  return np.r_[actual[:3,3]-target[:3,3],.3*Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).as_rotvec()]
 sol=least_squares(fun,entryq[-1],bounds=(g.lower+.01,g.upper-.01),max_nfev=100,gtol=1e-10,ftol=1e-10,xtol=1e-10)
 assert np.linalg.norm(fun(sol.x))<1e-5,'Entry lift IK failed'
 entryq.append(sol.x);liftposes.append(g.fk(sol.x)[g.camera])
q_entry=entryq[-1]

def visibility(T):
 ci=meta['views'][0]['color_info'];fx,fy,cx,cy=np.array(ci['k'])[[0,4,2,5]]/2;W,H=ci['width']//2,ci['height']//2
 pc=(P-T[:3,3])@T[:3,:3];z=pc[:,2]
 ix=np.rint(fx*pc[:,0]/np.maximum(z,1e-6)+cx).astype(int);iy=np.rint(fy*pc[:,1]/np.maximum(z,1e-6)+cy).astype(int)
 good=(z>=.2)&(z<=3)&(ix>=1)&(ix<W-1)&(iy>=1)&(iy<H-1)
 zbuf=np.full((H,W),np.inf);np.minimum.at(zbuf,(iy[good],ix[good]),z[good]);zbuf=minimum_filter(zbuf,size=3)
 mask=np.zeros(len(P),bool);mask[good]=z[good]<=zbuf[iy[good],ix[good]]+.025
 return mask

entryvis=visibility(g.fk(q_entry)['camera_color_optical_frame'])
cover=entryvis.copy();chosen=[]
# Prefer z >= 0.70 m transit/observation heights for the first automatic-scan design.
eligible=np.array([np.array(c['T_base_depth'])[2,3]>=.699 for c in C])
for _ in range(7):
 gains=(vis & ~cover).sum(1).astype(float);gains[~eligible]=-1;gains[chosen]=-1
 idx=int(gains.argmax());assert gains[idx]>0
 chosen.append(idx);cover|=vis[idx]
qs=np.array([q_entry]+[C[i]['q_rad'] for i in chosen]);n=len(qs)

def audit(a,b,dense=False):
 # The model motion bound limits every sphere's centre displacement between samples.
 intervals=max(1,int(np.ceil(g.motion_bound*np.sum(abs(b-a))/.004))) if dense else max(2,int(np.ceil(np.max(abs(b-a))/.025)))
 samples=a+np.linspace(0,1,intervals+1)[:,None]*(b-a)
 env=armgap=selfgap=np.inf;camz=np.inf
 for start in range(0,len(samples),256):
  c=g.spheres_batch(samples[start:start+256]);e=tree.query(c.reshape(-1,3))[0].reshape(c.shape[:2])-g.radii-maxedge
  env=min(env,float(e.min()));allself=np.linalg.norm(c[:,g.pair_a]-c[:,g.pair_b],axis=2)-g.radii[g.pair_a]-g.radii[g.pair_b]
  armgap=min(armgap,float(allself[:,arm].min()));selfgap=min(selfgap,float(allself.min()))
  camz=min(camz,float(g.poses_batch(samples[start:start+256])[g.camera][:,2,3].min()))
 pad=g.motion_bound*np.sum(abs(b-a))/(2*intervals) if dense else 0.
 return dict(samples=len(samples),environment_surface_bound_m=env-pad,arm_only_model_gap_m=armgap-2*pad,
             complete_self_model_gap_m=selfgap-2*pad,min_camera_height_m=camz,
             max_joint_delta_deg=float(np.rad2deg(np.max(abs(b-a)))),joint_l1_rad=float(np.sum(abs(b-a))),
             sample_motion_padding_m=pad)

cost=np.full((n,n),np.inf)
for i in range(n):
 for j in range(i+1,n):
  result=audit(qs[i],qs[j]);ok=result['environment_surface_bound_m']>.036 and result['arm_only_model_gap_m']>.020 and result['min_camera_height_m']>.61
  if ok:cost[i,j]=cost[j,i]=np.linalg.norm(qs[i]-qs[j])
best=None
for order in itertools.permutations(range(1,n)):
 order=(0,)+order;length=sum(cost[a,b] for a,b in zip(order,order[1:]))
 if best is None or length<best[0]:best=(length,order)
assert np.isfinite(best[0]),'No preliminary connected route; add transit candidates'
order=best[1];waypoints=[];ordered_masks=[]
for number,i in enumerate(order,1):
 q=qs[i];poses=g.fk(q);record=dict(name=f'S{number:02d}',candidate_id='ENTRY' if i==0 else C[chosen[i-1]]['id'],
   source='new offline IK pose',q_rad=q.tolist(),q_deg=np.rad2deg(q).tolist())
 for frame in [g.camera,'camera_color_optical_frame','tool0']:
  T=poses[frame];record[frame]=dict(position_m=T[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(T[:3,:3]).as_quat().tolist(),T_base_frame=T.tolist())
 record['optical_axis_base']=poses[g.camera][:3,2].tolist()
 record['capture_after_stationary']=True;record['suggested_hold_sec']=2.0
 waypoints.append(record);ordered_masks.append(entryvis if i==0 else vis[chosen[i-1]])
edges=[]
for a,b in zip(waypoints,waypoints[1:]):
 result=audit(np.array(a['q_rad']),np.array(b['q_rad']),True);result.update(start=a['name'],end=b['name'])
 result['known_surface_and_arm_screen_pass']=result['environment_surface_bound_m']>.03 and result['arm_only_model_gap_m']>.015
 result['complete_self_margin_15mm_pass']=result['complete_self_model_gap_m']>=.015
 edges.append(result)
 assert result['known_surface_and_arm_screen_pass'],result
liftchecks=[audit(a,b,True) for a,b in zip(entryq,entryq[1:])]
assert min(c['environment_surface_bound_m'] for c in liftchecks)>.03
print('Entry arm minimum',min(c['arm_only_model_gap_m'] for c in liftchecks),flush=True)
result=dict(schema_version=1,execution_ready=False,frame_id='base_link',joint_names=JOINTS,
 selection_scope='Offline candidate poses and nominal paths only; not executable approval',
 source_scan=str(SRC),source_map_sha256=hashlib.sha256((SRC/'surface.npz').read_bytes()).hexdigest(),
 model_sha256=meta['model_sha256'],
 unresolved=['camera/wrist sphere model fails the existing 15 mm complete self-clearance margin',
             'partial reconstructed surface does not prove unknown space or unmodelled payload is clear',
             'actual entry joint state and continuous whole-robot paths need validation before any execution'],
 reference_entry=dict(source='saved observation 35, not a live robot measurement',q_rad=q0.tolist(),
    lift_q_rad=[q.tolist() for q in entryq],lift_camera_T=[g.fk(q)[g.camera].tolist() for q in entryq]),
 waypoints=waypoints,
 route_edges=edges,entry_lift_checks=liftchecks,
 coverage=dict(method='uniform-area sampled final surface; RGB FOV and approximate point z-buffer occlusion',
   sample_count=len(P),selected_fraction=float(np.logical_or.reduce(ordered_masks).mean()),
   candidate_union_fraction=float(vis.any(0).mean()),recorded35_predicted_fraction=float(reference.any(0).mean()),
   actual_scene_completeness_measured=False),
 path_screen=dict(mesh_max_triangle_edge_m=maxedge,sphere_motion_step_bound_m=.004,environment_margin_m=.03,
                  arm_only_margin_m=.015,complete_self_margin_m=.015))
result=json.loads(json.dumps(result,default=lambda x:x.item()))
(D/'selected_observation_poses.yaml').write_text(yaml.safe_dump(result,allow_unicode=True,sort_keys=False))
(D/'route_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
np.savez_compressed(D/'selected_visibility.npz',points=P,masks=np.array(ordered_masks))
print('ROUTE',[(w['name'],w['candidate_id'],np.round(w[g.camera]['position_m'],3).tolist()) for w in waypoints])
print('Coverage',result['coverage'])
print('Edges',edges)
print('Entry min env',min(a['environment_surface_bound_m'] for a in liftchecks))
