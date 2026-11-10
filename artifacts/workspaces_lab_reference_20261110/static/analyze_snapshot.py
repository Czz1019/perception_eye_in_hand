import os,json,sys
from pathlib import Path
os.environ['MPLCONFIGDIR']='/tmp/ur7e_lab_accuracy_mpl'
import mpl_toolkits
mpl_toolkits.__path__=['/home/czz/.local/lib/python3.10/site-packages/mpl_toolkits']
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np,trimesh,open3d as o3d
from PIL import Image
p=Path('/home/czz/perception_eye_in_hand/artifacts/workspaces_lab_reference_20261110/static');ws=Path('/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand');pkg=ws/'src/ur7e_d435_gazebo';sys.path.insert(0,str(pkg/'scripts'))
import yaml
from lab_scene import build_lab_scene
cfg=yaml.safe_load((pkg/'config/lab_scene.yaml').read_text());_,manifest=build_lab_scene(cfg,pkg/'meshes/lab');fixtures=manifest['fixtures']
meta=json.loads((p/'capture.json').read_text());K=np.array(meta['K']);T=np.array(meta['T_base_depth']);z=np.load(p/'depth_m.npy');H,W=z.shape;yy,xx=np.indices(z.shape);dirs=np.stack(((xx-K[0,2])/K[0,0],(yy-K[1,2])/K[1,1],np.ones(z.shape)),-1)@T[:3,:3].T;rays=np.concatenate((np.broadcast_to(T[:3,3],dirs.shape),dirs),-1).astype(np.float32)
scene=o3d.t.geometry.RaycastingScene();meshes=[];names={}
for i,f in enumerate(fixtures):
 kind=f['kind'];d=f['dimensions']
 if kind=='box':m=trimesh.creation.box(d)
 elif kind=='cylinder':m=trimesh.creation.cylinder(radius=d[0],height=d[1],sections=64)
 elif kind=='sphere':m=trimesh.creation.icosphere(subdivisions=3,radius=d[0])
 elif kind=='mesh':m=trimesh.load(pkg/'meshes/lab'/('blue_rack_ring.stl' if f['name'].startswith('blue') else 'white_rack_ring.stl'))
 m.apply_translation(f['position']);m.visual.vertex_colors=np.uint8(np.array(f['color'])*255)
 om=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(m.vertices),o3d.utility.Vector3iVector(m.faces));gid=scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(om));names[gid]=f['name'];meshes.append(m)
trimesh.util.concatenate(meshes).export(p/'ground_truth_lab.ply')
gtcast=scene.cast_rays(o3d.core.Tensor(rays));gt=gtcast['t_hit'].numpy();gid=gtcast['geometry_ids'].numpy();np.save(p/'ground_truth_depth.npy',gt)
m=o3d.io.read_triangle_mesh(str(p/'mesh.ply'));ms=o3d.t.geometry.RaycastingScene();ms.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m));hit=ms.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy();np.save(p/'mesh_depth.npy',hit)
gpts=T[:3,3]+dirs*gt[:,:,None]
roi=np.isfinite(gt)&(gpts[:,:,0]>=.17)&(gpts[:,:,0]<=1.01)&(abs(gpts[:,:,1])<=.50)&(gpts[:,:,2]>=-.001)&(gpts[:,:,2]<=.40)
rawvalid=np.isfinite(z)&(z>=.1)&(z<=3)
# Ground-truth contains lab fixtures, not the UR mesh: remove foreground arm occlusion.
occluded=roi&rawvalid&(z<gt-.06)
ref=roi&rawvalid&~occluded;mv=np.isfinite(hit)&(hit>=.1)&(hit<=3);both=ref&mv

def stats(a):
 a=np.asarray(a);a=a[np.isfinite(a)];return {'n':len(a),'mae_mm':float(abs(a).mean()*1000),'rmse_mm':float(np.sqrt((a*a).mean())*1000),'median_abs_mm':float(np.median(abs(a))*1000),'p95_abs_mm':float(np.percentile(abs(a),95)*1000),'signed_mean_mm':float(a.mean()*1000)} if len(a) else {'n':0}
res={'configuration':{'voxel_size_m':.02,'truncation_vox':4,'truncation_m':.08,'input':[W,H],'motion':'static initial lab observation pose','reference':'procedural Gazebo lab geometry; cylinder 64 sides/sphere subdiv3; robot excluded'},'mesh':{'vertices':len(m.vertices),'triangles':len(m.triangles),'reference_pixels':int(ref.sum()),'excluded_foreground_pixels':int(occluded.sum()),'coverage':float(both.sum()/ref.sum()),'within_20mm_fraction_of_reference':float((both&(np.abs(np.subtract(hit,gt,out=np.full_like(hit,np.nan),where=np.isfinite(hit)&np.isfinite(gt)))<=.02)).sum()/ref.sum()),'vs_ground_truth':stats(hit[both]-gt[both]),'raw_vs_ground_truth':stats(z[ref]-gt[ref]),'vs_raw_depth':stats(hit[both]-z[both])}}

def category(name):
 if name.startswith(('blue_rack','white_rack')):return 'racks_and_tubes'
 if name.startswith('bench'):return 'table'
 if name.startswith('reagent'):return 'reagent_bottles'
 if name.startswith(('tray','vial')):return 'tray_and_vials'
 if name.startswith('flask'):return 'flask'
 if name.startswith('block'):return 'scale_blocks'
 return 'other'
labels=np.full(z.shape,'',dtype='U25')
for ident,name in names.items():labels[gid==ident]=category(name)
res['mesh']['by_object_group']={}
for label in np.unique(labels[ref]):
 mask=ref&(labels==label);overlap=mask&mv;res['mesh']['by_object_group'][label]={'reference_pixels':int(mask.sum()),'coverage':float(overlap.sum()/mask.sum()),**stats(hit[overlap]-gt[overlap])}
# Complement axial depth errors with nearest-surface Euclidean distances.
objects=ref&(labels!='table');objects_hit=objects&mv
mesh_xyz=(T[:3,3]+dirs*hit[:,:,None])[both].astype(np.float32)
gt_xyz=gpts[ref].astype(np.float32)
mesh_to_gt=scene.compute_distance(o3d.core.Tensor(mesh_xyz)).numpy()
gt_to_mesh=ms.compute_distance(o3d.core.Tensor(gt_xyz)).numpy()
res['mesh']['visible_mesh_to_gt_euclidean']=stats(mesh_to_gt)
res['mesh']['visible_gt_to_mesh_euclidean']=stats(gt_to_mesh)
res['mesh']['visible_gt_within_20mm_of_mesh_fraction']=float((gt_to_mesh<=.02).mean())
res['mesh']['objects_excluding_table']={'reference_pixels':int(objects.sum()),'coverage':float(objects_hit.sum()/objects.sum()),'vs_gt_axial':stats(hit[objects_hit]-gt[objects_hit])}
# Analytic signed distance to union of the exact box/cylinder/sphere/annular solids.
def truth_distance(q):
 best=np.full(len(q),np.inf);source=np.full(len(q),-1)
 for index,f in enumerate(fixtures):
  v=q-np.array(f['position']);d=f['dimensions'];kind=f['kind']
  if kind=='box':
   a=abs(v)-np.array(d)/2;sd=np.linalg.norm(np.maximum(a,0),axis=1)+np.minimum(a.max(1),0)
  elif kind=='sphere':sd=np.linalg.norm(v,axis=1)-d[0]
  else:
   radial=np.linalg.norm(v[:,:2],axis=1)
   if kind=='cylinder':a=radial-d[0];h=d[1]/2
   else:
    r=cfg['racks'][0 if f['name'].startswith('blue') else 1];a=np.maximum(radial-r['ring_radius'],r['hole_radius']-radial);h=.003
   b=abs(v[:,2])-h;sd=np.hypot(np.maximum(a,0),np.maximum(b,0))+np.minimum(np.maximum(a,b),0)
  source[sd<best]=index;best=np.minimum(best,sd)
 return best,source
qdata=np.load(p/'esdf_roi_query.npz');q=qdata['xyz'];d=qdata['distance'];obs=qdata['observed'];truth,source=truth_distance(q);outside=truth>=0;free=outside&obs;negative=free&(d<0)
res['esdf']={'roi_query_cells':len(q),'observed_cells':int(obs.sum()),'unknown_fraction':float((~obs).mean()),'ground_truth_free_cells':int(outside.sum()),'observed_free_cells':int(free.sum()),'free_space_observed_fraction':float(free.sum()/outside.sum()),'observed_free_distance_error':stats(d[free]-truth[free]),'free_cells_marked_inside':int(negative.sum()),'free_cells_gt_distance_ge_20mm_marked_inside':int((negative&(truth>=.02)).sum()),'distance_bands':{}}
for lo,hi in [(0,.02),(.02,.05),(.05,.10),(.10,.20),(.20,2)]:
 k=free&(truth>=lo)&(truth<hi);res['esdf']['distance_bands'][f'{lo:.2f}-{hi:.2f}m']=stats(d[k]-truth[k])
a=np.load(p/'esdf_roi_observed.npy');res['esdf']['roi_observed_points_from_service']=len(a)
res['esdf']['by_nearest_geometry']={}
for category_name in sorted(set(category(fixtures[i]['name']) for i in source)):
 k=free&np.array([category(fixtures[i]['name'])==category_name for i in source]);res['esdf']['by_nearest_geometry'][category_name]=stats(d[k]-truth[k])
site=a[:,6]>0
site_roi=site&(a[:,0]>=.17)&(a[:,0]<=1.01)&(abs(a[:,1])<=.5)
unique,count=np.unique(np.round(a[site_roi,2],3),return_counts=True);order=np.argsort(count)[::-1][:8]
res['esdf']['most_common_site_z_m']=[{'z':float(unique[i]),'count':int(count[i])} for i in order]

np.savez_compressed(p/'accuracy_fields.npz',xyz=q,estimated=d,observed=obs,ground_truth=truth)
# Native observed ESDF cloud with distance scalar, usable in CloudCompare.
record=np.empty(len(a),dtype=[('xyz','<f4',(3,)),('distance','<f4')]);record['xyz']=a[:,:3];record['distance']=a[:,3]
header=f'ply\nformat binary_little_endian 1.0\nelement vertex {len(a)}\nproperty float x\nproperty float y\nproperty float z\nproperty float distance\nend_header\n';(p/'esdf_observed.ply').write_bytes(header.encode()+record.tobytes())
# Render same-view mesh colors using triangle barycentrics, independent of RViz.
mc=ms.cast_rays(o3d.core.Tensor(rays));prim=mc['primitive_ids'].numpy();uv=mc['primitive_uvs'].numpy();rgb=np.ones((H,W,3))*.12;valid=mv;faces=np.asarray(m.triangles);colors=np.asarray(m.vertex_colors)
if len(colors):
 tri=faces[prim[valid]];u=uv[valid,0,None];v=uv[valid,1,None];rgb[valid]=(1-u-v)*colors[tri[:,0]]+u*colors[tri[:,1]]+v*colors[tri[:,2]]
Image.fromarray(np.uint8(np.clip(rgb,0,1)*255)).save(p/'mesh_rgb_render.png')
fig,ax=plt.subplots(2,3,figsize=(15,9));cm=plt.get_cmap('viridis').copy();cm.set_bad('#222222')
ax[0,0].imshow(Image.open(p/'rgb.png'));ax[0,0].set_title('Actual simulated RGB')
ax[0,1].imshow(rgb);ax[0,1].set_title('Exported colored Mesh / same depth viewpoint')
ax[0,2].imshow(np.ma.masked_where(~ref,gt),cmap=cm,vmin=.3,vmax=.9);ax[0,2].set_title('Ground truth / evaluated tabletop ROI')
ax[1,0].imshow(np.ma.masked_where(~rawvalid,z),cmap=cm,vmin=.3,vmax=.9);ax[1,0].set_title('Raw depth (m)')
ax[1,1].imshow(np.ma.masked_where(~mv,hit),cmap=cm,vmin=.3,vmax=.9);ax[1,1].set_title(f'Mesh raycast / coverage {res["mesh"]["coverage"]:.1%}')
e=np.full(z.shape,np.nan);e[both]=abs(hit[both]-gt[both])*1000
im=ax[1,2].imshow(e,cmap='magma',vmin=0,vmax=50);fig.colorbar(im,ax=ax[1,2],label='mm');ax[1,2].set_title(f'Mesh vs GT / P95 {res["mesh"]["vs_ground_truth"]["p95_abs_mm"]:.1f} mm')
for x in ax.flat:x.axis('off')
fig.suptitle('workspaces official ROS NvbloxNode | lab bench | 20 mm voxels | static view');fig.tight_layout();fig.savefig(p/'mesh_accuracy.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(1,3,figsize=(16,5));section=(abs(q[:,2]-.11)<.002)
for i,(values,title,cmap,vmin,vmax) in enumerate([(np.where(obs,d,np.nan),'Estimated ESDF at z=0.11 m','turbo',0,.2),(truth,'Ground truth distance','turbo',0,.2),(np.where(free,(d-truth)*1000,np.nan),'Free-space ESDF error (mm)','coolwarm',-50,50)]):
 k=section&np.isfinite(values);im=ax[i].scatter(q[k,0],q[k,1],c=values[k],s=12,marker='s',cmap=cmap,vmin=vmin,vmax=vmax);ax[i].set_aspect('equal');ax[i].set_title(title);ax[i].set_xlabel('base X (m)');ax[i].set_ylabel('base Y (m)');ax[i].set_facecolor('#222222');fig.colorbar(im,ax=ax[i],shrink=.7)
fig.suptitle('3D ESDF field section (not a 2D ESDF computation); blank = unknown / excluded');fig.tight_layout();fig.savefig(p/'esdf_accuracy.png',dpi=150);plt.close(fig)
# Three-dimensional observed free-space distance field, no fabricated unknown points.
fig=plt.figure(figsize=(10,8));ax=fig.add_subplot(projection='3d');k=free&(truth<.16);ids=np.flatnonzero(k)[::3];im=ax.scatter(q[ids,0],q[ids,1],q[ids,2],c=d[ids],s=2,cmap='turbo',vmin=0,vmax=.16);ax.view_init(50,-60);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)');ax.set_box_aspect((.84,1,.4));fig.colorbar(im,ax=ax,shrink=.6,label='ESDF distance (m)');ax.set_title('Observed 3D ESDF / tabletop ROI');fig.savefig(p/'esdf_3d.png',dpi=150);plt.close(fig)
(p/'accuracy_summary.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
