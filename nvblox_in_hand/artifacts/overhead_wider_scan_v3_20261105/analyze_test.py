from pathlib import Path
import json,re,numpy as np
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());traj=json.loads((p/'camera_trajectory.json').read_text());s=json.loads((p/'samples.json').read_text());out={'status':r['status'],'phases':{},'measurement':'official counter differences and wall timestamps; no GPU completion measurement'}
def timers(name):
 return {tag:int(n) for tag,n in re.findall(r'^\s*(\S+)\s+(\d+)\s+[\d.]+\s+\(',(p/(name+'_timings.txt')).read_text(),re.M)}
a=timers('overhead_static');b=timers('overhead_wrist_motion');dt=r['phases']['overhead_wrist_motion']['end_monotonic']-r['phases']['overhead_static']['end_monotonic']
out['motion_interval_wall_seconds']=dt;out['official_rates_hz']={key:(b[tag]-a[tag])/dt for tag,key in [('ros/depth/integrate','depth'),('ros/color/integrate','color'),('ros/esdf','esdf_calls')]}
for name,phase in r['phases'].items():
 poses=[x for x in traj if phase['start_monotonic']<=x['wall_time']<=phase['end_monotonic']];v=np.array([x['position'] for x in poses]);f=np.array([x['optical_forward'] for x in poses]);f0=f[0];angles=np.rad2deg(np.arccos(np.clip(f@f0,-1,1)))
 out['phases'][name]={k:phase[k] for k in ['wall_seconds','esdf_cloud_wall_hz','mesh_wall_hz','esdf_cloud_points','rgb_input_wall_hz','depth_input_wall_hz','cloud_age_latest_clock_sec','simulation_rtf']};out['phases'][name]['camera']={'samples':len(poses),'world_position_min':v.min(axis=0).tolist(),'world_position_max':v.max(axis=0).tolist(),'optical_forward_world_z_range':[float(f[:,2].min()),float(f[:,2].max())],'forward_angle_from_phase_start_max_deg':float(angles.max())}
 motion=phase if 'wrist_motion' in name else None
 if motion:
  e=[x for x in s['cloud'] if phase['start_monotonic']<=x[0]<=phase['end_monotonic']];iv=np.diff([x[0] for x in e])*1000;out['motion_receive_interval_ms']={'p50':float(np.percentile(iv,50)),'p95':float(np.percentile(iv,95))}
out['before_cloud']=r['before_scan_cloud'];out['after_cloud']=r['backends']['overhead']['cloud_snapshot'];out['native_esdf_query']=r['backends']['overhead']['native_esdf_query'];out['observation_pose']=r['camera_observation_pose']
(p/'analysis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
