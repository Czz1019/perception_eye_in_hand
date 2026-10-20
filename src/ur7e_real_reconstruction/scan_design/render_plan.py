"""Finalize offline route diagnostics and figures; never contacts hardware."""
from pathlib import Path
import json
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ur7e_real_static.model import Geometry
D=Path(__file__).resolve().parent;data=json.loads((D/'route_analysis.json').read_text());src=Path(data['source_scan'])
m=json.loads((src/'metadata.json').read_text());g=Geometry(m['config']['robot_urdf'],m['config']['collision_spheres'])
w=data['waypoints'];Q=np.array([x['q_rad'] for x in w]);qpaths=[]
def sigma(q):
 poses=g.fk(q);p=poses[g.camera][:3,3];cols=[]
 for name,parent,child,origin,axis,kind in g.joints:
  if kind=='fixed':continue
  joint=poses[parent]@origin;a=joint[:3,:3]@axis;cols.append(np.r_[np.cross(a,p-joint[:3,3]),.3*a])
 J=np.column_stack(cols)
 return float(np.linalg.svd(J,compute_uv=False)[-1])
for i,(a,b) in enumerate(zip(Q,Q[1:])):
 qs=a+np.linspace(0,1,max(3,int(np.ceil(np.max(abs(a-b))/np.deg2rad(2)))+1))[:,None]*(b-a)
 data['route_edges'][i]['sampled_min_scaled_jacobian_sigma']=min(sigma(q) for q in qs)
 qpaths.append(g.poses_batch(qs)[g.camera][:,:3,3])
entry=data['entry_lift_checks']
data['reference_entry']['known_surface_and_arm_screen_pass']=min(x['environment_surface_bound_m'] for x in entry)>.03 and min(x['arm_only_model_gap_m'] for x in entry)>.015
data['reference_entry']['complete_self_margin_pass']=min(x['complete_self_model_gap_m'] for x in entry)>=.015
if not data['reference_entry']['known_surface_and_arm_screen_pass']:
 data['reference_entry']['status']='rejected_reference_lift_not_for_execution'
 data['unresolved'].append('the reference vertical entry lift fails the 15 mm arm-only model gap; it is rejected, not an executable entry route')
data['jacobian_diagnostic']='camera Jacobian: translation in m/rad, rotation multiplied by 0.3 m; sampled every <=2 deg maximum joint change; not a continuous singularity proof'
(D/'route_analysis.json').write_text(json.dumps(data,indent=2)+'\n')
(D/'selected_observation_poses.yaml').write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
arr=np.load(D/'selected_visibility.npz');P=arr['points'];seen=arr['masks'].any(0)
with np.load(src/'surface.npz',allow_pickle=False) as mesh:v=mesh['vertices']
fig=plt.figure(figsize=(14,7),layout='constrained');ax=fig.add_subplot(121,projection='3d')
ax.scatter(*v[::4].T,s=.7,c='#b7bdc6',alpha=.2)
colors=plt.cm.viridis(np.linspace(.08,.9,len(w)))
for path in qpaths:ax.plot(*path.T,c='#2166ac',lw=1.4)
for i,wp in enumerate(w):
 T=np.array(wp['camera_depth_optical_frame']['T_base_frame']);p=T[:3,3]
 ax.scatter(*p,s=38,c=[colors[i]]);ax.text(*p,' '+wp['name'],fontsize=9)
 ax.quiver(*p,*T[:3,2],length=.12,color='#e58224',linewidth=1.)
# Entry lift is specifically rejected. Show the saved starting point only.
ref=np.array(data['reference_entry']['lift_camera_T'])[0,:3,3]
ax.scatter(*ref,c='#c0392b',marker='x',s=55);ax.text(*ref,' saved #35\n entry unresolved',fontsize=8,color='#c0392b')
ax.set(xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='Proposed camera poses and joint-interpolated paths')
ax.view_init(elev=29,azim=-65);ax.set_box_aspect([1.1,1.8,.9])
ax=fig.add_subplot(122)
ax.scatter(P[~seen,0],P[~seen,1],s=1,c='#b7bdc6',label='Not predicted visible')
ax.scatter(P[seen,0],P[seen,1],s=1,c='#3a9b89',label='Predicted visible surface')
for path in qpaths:ax.plot(path[:,0],path[:,1],c='#2166ac',lw=1.2)
for i,wp in enumerate(w):
 p=np.array(wp['camera_depth_optical_frame']['position_m']);ax.scatter(p[0],p[1],s=40,c=[colors[i]],edgecolor='white');ax.annotate(wp['name'],p[:2],xytext=(6,5),textcoords='offset points',fontsize=9)
ax.set(xlabel='X [m]',ylabel='Y [m]',title=f"Approximate coverage of saved mesh: {data['coverage']['selected_fraction']*100:.1f}%")
ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left',markerscale=3);ax.grid(alpha=.15)
fig.suptitle('OFFLINE CANDIDATES ONLY | 8 scan poses | complete collision / entry validation unresolved',fontsize=14)
fig.savefig(D/'assets/selected_route.png',dpi=150);plt.close(fig)
print('route diagnostics',json.dumps(data['route_edges'],indent=2))
print('entry minima',min(x['environment_surface_bound_m'] for x in entry),min(x['arm_only_model_gap_m'] for x in entry))
