from pathlib import Path
import json,re,numpy as np
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());out={'status':r['status'],'measurement':'existing official timer sample differences/static-end to motion-end wall time; pointcloud received intervals','modes':{}}
def timers(name):
 text=(p/(name+'_timings.txt')).read_text()
 return {tag:int(n) for tag,n in re.findall(r'^\s*(\S+)\s+(\d+)\s+[\d.]+\s+\(',text,re.M)}
for mode in ['wide','dense']:
 phase=r['phases'][mode+'_wrist_motion'];a=timers(mode+'_static');b=timers(mode+'_wrist_motion');dt=phase['end_monotonic']-r['phases'][mode+'_static']['end_monotonic']
 d={key:(b[tag]-a[tag])/dt for tag,key in [('ros/depth/integrate','depth_hz'),('ros/color/integrate','color_hz'),('ros/esdf','esdf_call_hz')]}
 d.update({k:phase[k] for k in ['esdf_cloud_wall_hz','mesh_wall_hz','esdf_cloud_points','cloud_age_latest_clock_sec','simulation_rtf']});d['cloud_snapshot']=r['backends'][mode]['cloud_snapshot'];d['publishers']=r['backends'][mode]['custom_cloud_publishers']
 dist=np.load(p/(mode+'_esdf.npy'))['distance'];d['distance_percentiles_m']={str(q):float(np.percentile(dist,q)) for q in [0,5,25,50,75,90,95,100]};d['palette_comparison_same_cloud']={}
 for key,lo,hi in [('wide',0.,1.),('dense',-.09,.24)]:
  normalized=np.clip((dist-lo)/(hi-lo),0,1);bins=np.histogram(normalized,bins=np.linspace(0,1,13))[0]
  d['palette_comparison_same_cloud'][key]={'min_m':lo,'max_m':hi,'equal_width_12_bins_point_counts':bins.tolist(),'occupied_bins':int((bins>0).sum()),'largest_bin_fraction':float(bins.max()/len(dist)),'below_range':int((dist<lo).sum()),'above_range':int((dist>hi).sum())}
 out['modes'][mode]=d
(p/'analysis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
