"""Offline analysis of wall time, header freshness and saved mesh; no ROS actions."""
from pathlib import Path
import json,re,csv,numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent
r=json.loads((p/'results.json').read_text());assert r['status']=='completed'
s=json.loads((p/'samples.json').read_text());cloud=json.loads((p/'cloud_records.json').read_text());trajectory=json.loads((p/'camera_trajectory.json').read_text())
def quant(v):return {k:float(np.percentile(v,q)) for k,q in [('p50',50),('p95',95),('p99',99)]} if len(v) else None
def timers(file):return {tag:{'n':int(n),'s':float(t)} for tag,n,t in re.findall(r'^\s*(\S+)\s+(\d+)\s+([\d.]+)\s+\(',file.read_text(),re.M)}
T=r['world_from_base_link'];rot=Rotation.from_quat(T['quaternion_xyzw']).as_matrix();trans=np.array(T['translation'])
def mesh_stats(file):
 with file.open() as f:
  header=[]
  while True:
   line=f.readline();header.append(line)
   if line.strip()=='end_header':break
   if not line:raise RuntimeError('Incomplete PLY')
  text=''.join(header);nv=int(re.search(r'element vertex (\d+)',text)[1]);nf=int(re.search(r'element face (\d+)',text)[1]);a=np.array([list(map(float,f.readline().split())) for _ in range(nv)])
 xyz=a[:,:3]@rot.T+trans;rgb=a[:,6:9]
 d={'file':file.name,'vertices':nv,'triangles':nf,'world_aabb_min':xyz.min(0).tolist(),'world_aabb_max':xyz.max(0).tolist(),'unique_vertex_colors':len(np.unique(rgb,axis=0)),'non_gray_vertex_fraction':float((np.ptp(rgb,axis=1)>10).mean()),'wall_top_roi':[]}
 for name,cy in [('wall1',.2),('wall2',-.3)]:
  # Conditional error only: fixed interior of known box top, 6cm vertical search band.
  mask=(xyz[:,0]>.28)&(xyz[:,0]<.52)&(abs(xyz[:,1]-cy)<.025)&(abs(xyz[:,2]-.5)<.06)
  e=abs(xyz[mask,2]-.5)*1000
  d['wall_top_roi'].append({'wall':name,'vertices':len(e),'abs_height_error_mm':quant(e),'rmse_mm':float(np.sqrt(np.mean(e**2))) if len(e) else None})
 return d
out={'date':'2026-11-09','status':r['status'],'gpu_completion_measured':False,'rows':[],'meshes':[]}
for mode,back in r['backends'].items():
 ta=timers(p/(mode+'_static_timings.txt'));tb=timers(p/(mode+'_measure_timings.txt'))
 sa=r['snapshots'][mode+'_static_timings.txt'];sb=r['snapshots'][mode+'_measure_timings.txt'];dt=sb['midpoint']-sa['midpoint']
 official={key:(tb[tag]['n']-ta[tag]['n'])/dt for key,tag in [('depth_fusion_hz','ros/depth/integrate'),('color_fusion_hz','ros/color/integrate'),('esdf_call_hz','ros/esdf')]}
 for stage in ['static','measure']:
  ph=r['phases'][mode+'_'+stage];start,end=ph['start_monotonic'],ph['end_monotonic'];ev=[x for x in cloud if start<=x['receive_wall']<=end]
  ts=np.array([x['receive_wall'] for x in ev]);hs=np.array([x['header_ns'] for x in ev],dtype=np.int64);diff=np.diff(hs);span=ts[-1]-ts[0]
  row={'mode':mode,'voxel_size_m':back['voxel_size_m'],'phase':stage,'duration_s':end-start,'depth_input_hz':ph['depth_input_wall_hz'],'rgb_input_hz':ph['rgb_input_wall_hz'],'cloud_hz':(len(ev)-1)/span,'associated_new_input_hz':float((diff>0).sum()/span),'repeated_header_fraction':float((diff==0).mean()),'backward_header_count':int((diff<0).sum()),'age_ms':quant([x['age_ms'] for x in ev if x['age_ms'] is not None]),'interval_ms':quant(np.diff(ts)*1000),'points_min':min(x['points'] for x in ev),'points_max':max(x['points'] for x in ev),'mesh_hz':ph['mesh_wall_hz'],'simulation_rtf':ph['simulation_rtf'],'messages':len(ev)}
  if stage=='measure':row.update(official);row['fusion_count_method']='Official timer count delta between static-end and motion-end service midpoints';row['fusion_count_window_s']=dt
  else:
   m=[x for x in s['metrics'] if start+3<=x[0]<=end]
   elapsed=m[-1][0]-m[0][0]
   row.update({key:(m[-1][1]['totals'][tag]-m[0][1]['totals'][tag])/elapsed for key,tag in [('depth_fusion_hz','depth_integrated'),('color_fusion_hz','color_integrated'),('esdf_call_hz','esdf_updated')]})
   row['fusion_count_method']='Diagnostic cumulative counter delta after first 3s callback drain; not mean of reported rolling Hz';row['fusion_count_window_s']=elapsed
  tr=[x for x in trajectory if start<=x['wall_time']<=end];poses=np.array([x['position'] for x in tr]);row['camera_world_position_min']=poses.min(0).tolist();row['camera_world_position_max']=poses.max(0).tolist()
  out['rows'].append(row)
  file=p/('mesh_'+mode+('_static' if stage=='static' else '')+'.ply');d=mesh_stats(file);d.update(mode=mode,phase=stage,voxel_size_m=back['voxel_size_m']);out['meshes'].append(d)
(p/'analysis.json').write_text(json.dumps(out,indent=2)+'\n')
with (p/'test_results.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.writer(f);w.writerow(['voxel_size_m','phase','duration_s','depth_fusion_hz','color_fusion_hz','esdf_calls_hz','esdf_points_hz','associated_new_input_hz','mesh_hz','points_min','points_max','age_p50_ms','age_p95_ms'])
 for x in out['rows']:w.writerow([x[k] for k in ['voxel_size_m','phase','duration_s','depth_fusion_hz','color_fusion_hz','esdf_call_hz','cloud_hz','associated_new_input_hz','mesh_hz','points_min','points_max']]+[x['age_ms']['p50'],x['age_ms']['p95']])
print(json.dumps(out,indent=2))
