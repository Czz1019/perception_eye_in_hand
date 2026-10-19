"""Generate report figures from the 2026-10-13 real scan; no robot connection."""
from pathlib import Path
import json,shutil,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
R=Path(__file__).resolve().parent;A=R/'assets';E=R/'evidence'
WS=R.parents[2];L=WS/'log/auto_esdf_real_20261013'
S=Path('/home/wqj/nvblox_output/ur7e_auto_esdf/scan_20261013_200952_ba2346')
RUN=S.parent/'plans/run_20261013_200952_9f3cf2';FAILED=S.parent/'plans/run_20261013_200458_c7aafc'
A.mkdir(exist_ok=True);E.mkdir(exist_ok=True)
for name in ['initial_state.json','final_state.json','map_validation.json','events.jsonl','robot_messages.jsonl','script_channel_check.json','rtde_startup_result.json','rtde_startup_feedback.json','rtde_startup_diagnostic.log','start_response.log','start2_response.log']:
 shutil.copy2(L/name,E/name)
for name in ['result.json','plan.json','servo_trace.npz']:shutil.copy2(RUN/name,E/name)
shutil.copy2(FAILED/'result.json',E/'first_attempt_result.json')
for name in ['summary.json','observations.json','mapping.yaml']:shutil.copy2(S/name,E/name)
for name in ['control.yaml','waypoints.yaml']:shutil.copy2(WS/'src/ur7e_auto_esdf/config'/name,E/name)
P=S.parent/'plans/preview_20261013_200907_58ec2b'
for name in ['route.png','arm_segments.png']:shutil.copy2(P/name,A/name)
obs=json.loads((S/'observations.json').read_text());result=json.loads((RUN/'result.json').read_text());plan=json.loads((RUN/'plan.json').read_text())
events=[json.loads(l) for l in (L/'events.jsonl').read_text().splitlines()]
ctrl=[e for e in events if e.get('source')=='controller' and e.get('run_directory')==str(RUN)]
first={}
for e in ctrl:first.setdefault(e['phase'],e['time'])
frames={}
for e in events:
 if e.get('source')=='mapper' and e.get('session')==str(S) and e.get('frames',0)>0 and e.get('metrics',{}).get('input_stamp'):
  frames[e['metrics']['input_stamp']]=e
z=np.load(RUN/'servo_trace.npz');tr=z['samples'];seg=z['segment_index'];dt=np.diff(tr[:,0]);same=seg[1:]==seg[:-1]
metrics={'run':str(RUN),'scan':str(S),'frames':64,'views':8,'duration_start_to_done_sec':first['DONE']-first['STARTING'],
 'warmup_phase_sec':first['ACQUIRING_CONTROL']-first['WARMING_UP'],'from_first_control_to_done_sec':first['DONE']-first['ACQUIRING_CONTROL'],
 'trace_samples':len(tr),'max_actual_speed_rad_s':float(np.max(abs(tr[:,15:21]))),'max_tracking_error_rad':float(np.max(abs(tr[:,3:9]-tr[:,9:15]))),
 'within_segment_period_ms_p50_p95_p99_max':(np.percentile(dt[same]*1000,[50,95,99,100])).tolist(),
 'max_control_diagnostic_interval_ms':1000*max(s['diagnostics']['max_interval_sec'] for s in result['motion_segments']),
 'metric_samples':len(frames),'perception_ms':{},'observations':[],'phase_first_times':first}
for key in ['native_esdf_ms','free_field_ms','receive_to_result_ms']:
 vals=[e['metrics'][key] for e in frames.values()];metrics['perception_ms'][key]=dict(zip(['p50','p95','max'],map(float,np.percentile(vals,[50,95,100]))))
fig,axs=plt.subplots(2,4,figsize=(16,7.2))
for i,(o,ax) in enumerate(zip(obs,axs.flat)):
 name=f'S{i+1:02d}';photo=json.loads((L/'photos'/f'{name}_rgb.json').read_text())
 shutil.copy2(L/'photos'/f'{name}_rgb.png',A/f'{name}_rgb.png');shutil.copy2(L/'photos'/f'{name}_rgb.json',E/f'{name}_rgb.json')
 rgb=Image.open(A/f'{name}_rgb.png');ax.imshow(rgb);ax.axis('off');ax.set_title(name+' | 8 ESDF frames')
 depth=np.load(S/'observations'/f'{i+1:03d}'/'depth.npz')['meters'];pose=np.array(o['T_base_depth']);q=np.array(o['q_rad'])
 row={'name':name,'xyz_m':pose[:3,3].tolist(),'joint_target_error_rad':float(np.max(abs(q-plan['segments'][i]['q_end']))),
      'rgb_stamp':photo['image_stamp'],'depth_stamp':o['stamp'],'rgb_minus_saved_depth_sec':photo['image_stamp']-o['stamp'],'depth_valid_fraction':float(np.mean(depth>0))}
 metrics['observations'].append(row)
 f,aa=plt.subplots(1,2,figsize=(12,4.5));aa[0].imshow(rgb);aa[0].set_title(f'{name}: RGB (separate preview stream)');aa[0].axis('off')
 im=aa[1].imshow(np.ma.masked_where(depth<=0,depth),cmap='viridis',vmin=.2,vmax=1.5);aa[1].set_title('Saved depth (m); invalid = white');aa[1].axis('off');f.colorbar(im,ax=aa[1],shrink=.85,label='Optical depth (m)')
 f.suptitle(f'{name} | camera in base_link: '+', '.join(f'{x:.3f}' for x in pose[:3,3])+' m')
 f.tight_layout();f.savefig(A/f'{name}_observation.png',dpi=140);plt.close(f)
fig.suptitle('Real D435i observations | 2026-10-13 | separate RGB preview, not ESDF colors');fig.tight_layout();fig.savefig(A/'observations_overview.png',dpi=140);plt.close(fig)
n=np.load(S/'native_esdf.npz');points=n['points'];d=n['distance'];unknown=float(n['unknown']);valid=np.isfinite(d)&(~np.isclose(abs(d),abs(unknown),atol=.001));band=valid&(abs(d)<=.05)
metrics['native_known_cells']=int(valid.sum());metrics['native_surface_band_cells']=int(band.sum())
f=plt.figure(figsize=(14,7),layout='constrained');p=points[band]
ax=f.add_subplot(121,projection='3d');sc=ax.scatter(*p.T,c=d[band],cmap='coolwarm',vmin=-.05,vmax=.05,s=3)
ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title='Native ESDF: perspective');ax.view_init(35,-65);ax.set_box_aspect(np.maximum(np.ptp(p,axis=0),.1))
ax=f.add_subplot(122);order=np.argsort(p[:,2]);sc=ax.scatter(p[order,0],p[order,1],c=d[band][order],cmap='coolwarm',vmin=-.05,vmax=.05,s=5)
ax.set(xlabel='X (m)',ylabel='Y (m)',title='Native ESDF: top projection');ax.set_aspect('equal');ax.grid(alpha=.2)
f.colorbar(sc,ax=f.axes,shrink=.65,label='Signed native distance (m)');f.suptitle('Saved native ESDF in base_link | 25 mm grid | abs(distance) <= 0.05 m');f.savefig(A/'native_esdf.png',dpi=150);plt.close(f)
n=np.load(S/'frozen_map.npz');dist=n['distance'];source=n['source'].reshape(dist.shape);meta=json.loads(str(n['metadata']));origin=np.array(meta['origin']);res=meta['resolution']
f,aa=plt.subplots(1,3,figsize=(16,5),layout='constrained')
for ax,height in zip(aa,[.1,.3,.5]):
 zi=int(round((height-origin[2])/res));sl=np.ma.masked_where(source[zi]==0,dist[zi]);im=ax.imshow(sl,origin='lower',extent=[origin[0],origin[0]+(dist.shape[2]-1)*res,origin[1],origin[1]+(dist.shape[1]-1)*res],vmin=0,vmax=.25,cmap='viridis');ax.set(title=f'Z = {origin[2]+zi*res:.2f} m',xlabel='base_link X (m)',ylabel='Y (m)');ax.set_facecolor('#cccccc')
f.colorbar(im,ax=aa,shrink=.7,label='Conservative distance (m)');f.suptitle('Frozen planning field | gray = unknown/blocked | no commissioned free-space priors');f.savefig(A/'conservative_slices.png',dpi=150,bbox_inches='tight');plt.close(f)
f,aa=plt.subplots(3,1,figsize=(13,9),sharex=True);t0=tr[0,0]
for idx in np.unique(seg):
 a=tr[seg==idx];t=a[:,0]-t0
 aa[0].plot(t,np.max(abs(a[:,15:21]),axis=1),lw=.7)
 aa[1].plot(t,np.max(abs(a[:,3:9]-a[:,9:15]),axis=1),lw=.7)
 aa[2].plot(t[1:],np.diff(a[:,0])*1000,lw=.6)
 aa[0].text(t.mean(),.145,f'S{idx+1:02d}',ha='center',fontsize=9)
aa[0].axhline(.12,c='gray',ls='--',label='planned limit 0.12');aa[0].set(ylabel='max actual speed (rad/s)',ylim=(0,.16));aa[0].legend()
aa[1].set(ylabel='max |q_cmd - q| (rad)');aa[2].axhline(40,c='red',ls='--',label='deadline 40 ms');aa[2].set(ylabel='recorded interval (ms)',xlabel='Seconds since first servo sample');aa[2].legend()
f.suptitle('Second run: real RTDE feedback | gaps = stopped capture, excluded from period statistics');f.tight_layout();f.savefig(A/'motion_metrics.png',dpi=150);plt.close(f)
(R/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
manifest={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(A.glob('*.png'))};(R/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:v for k,v in metrics.items() if k not in ['observations','phase_first_times']},indent=2))
