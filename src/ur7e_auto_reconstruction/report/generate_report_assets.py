"""Offline figures and metrics from recorded real runs; no robot connection."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,shutil,hashlib
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image,ImageDraw
R=Path(__file__).resolve().parent
W=R.parents[2]
L=W/'log/auto_real_20260929'
O=Path('/home/wqj/nvblox_output/ur7e_auto_reconstruction')
S=O/'scan_20260929_131301_9fb3a9'
A=R/'assets';A.mkdir(exist_ok=True)
E=R/'evidence';E.mkdir(exist_ok=True)
route=yaml.safe_load((R.parent/'config/waypoints.yaml').read_text())['waypoints']
views=json.loads((S/'observations.json').read_text())
metrics={'runs':{},'observations':[],'full_scan_passed':False,'operator_observation':'无明显抖动','collision_confirmation':None}
for label,name in [('first','run_20260929_130313_72d380'),('second','run_20260929_131301_6bccec')]:
 p=O/'plans'/name;a=np.load(p/'servo_trace.npz')['samples'];dt=np.diff(a[:,0])*1000
 m={'directory':str(p),'result':json.loads((p/'result.json').read_text()),'samples':len(a),'trace_duration_sec':float(a[-1,0]-a[0,0]),'recorded_interval_ms_p50_p95_p99_max':np.percentile(dt,[50,95,99,100]).tolist(),'max_abs_dq_rad_s':abs(a[:,15:21]).max(0).tolist(),'max_abs_tracking_rad':abs(a[:,3:9]-a[:,9:15]).max(0).tolist()}
 metrics['runs'][label]=m
 shutil.copy2(p/'result.json',E/(label+'_result_original.json'))
 if label=='second':
  fig,axes=plt.subplots(3,1,figsize=(12,8),layout='constrained',sharex=True);t=a[:,0]-a[0,0]
  for j in range(6):
   axes[0].plot(t[::5],a[::5,15+j],lw=.7,label=f'J{j+1}')
   axes[1].plot(t[::5],(a[::5,3+j]-a[::5,9+j])*1000,lw=.7)
  axes[0].set(ylabel='Actual velocity [rad/s]',title='Second real run | S01 to S08 | terminated by scheduling guard');axes[0].legend(ncol=6)
  axes[0].axhline(.12,color='gray',ls='--');axes[0].axhline(-.12,color='gray',ls='--')
  axes[1].set(ylabel='Command - actual [mrad]')
  axes[2].plot(t[1:],dt,lw=.5);axes[2].axhline(40,color='red',ls='--');axes[2].set(xlabel='Time since first recorded command [s]',ylabel='Recorded interval [ms]',title='Fatal interval is not in trace: guard ran before appending a sample')
  for ax in axes:ax.grid(alpha=.2)
  fig.savefig(A/'second_run_motion.png',dpi=150);plt.close(fig)
for i,(v,w) in enumerate(zip(views,route)):
 shutil.copy2(S/v['path']/'rgb.png',A/f'S{i+1:02d}_rgb.png')
 metrics['observations'].append({'name':w['name'],'max_joint_error_rad':float(max(abs(np.array(v['q_rad'])-w['q_rad']))),'camera_depth_xyz_m':np.array(v['T_base_depth'])[:3,3].tolist(),'captured_at':datetime.fromtimestamp(v['depth_stamp'],ZoneInfo('Asia/Shanghai')).isoformat(),'completed_capture':i<7})
canvas=Image.new('RGB',(1280,540),'#eeeeee');draw=ImageDraw.Draw(canvas)
for i in range(8):
 x=(i%4)*320;y=(i//4)*270;canvas.paste(Image.open(A/f'S{i+1:02d}_rgb.png').resize((320,240)),(x,y+30));draw.text((x+8,y+8),f'S{i+1:02d}'+(' (partial: 1 frame)' if i==7 else ''),fill='black')
canvas.save(A/'observations_overview.png')
a=np.load(S/'surface.npz');v,t,c=a['vertices'],a['triangles'],a['colors'];tri=v[t];fc=c[t].mean(1)/255;lo,hi=v.min(0),v.max(0)
metrics['map']={'session':str(S),'frames':json.loads((S/'metadata.json').read_text())['frames'],'views':len(views),'vertices':len(v),'triangles':len(t),'bounds_min_m':lo.tolist(),'bounds_max_m':hi.tolist(),'colored_vertex_fraction':float(np.mean(c.max(1)-c.min(1)>20)),'npz_sha256':hashlib.sha256((S/'surface.npz').read_bytes()).hexdigest()}
fig=plt.figure(figsize=(14,7),layout='constrained');ax=fig.add_subplot(121,projection='3d')
ax.add_collection3d(Poly3DCollection(tri,facecolors=fc,edgecolors='none',antialiaseds=False))
ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='Saved colored mesh | perspective');ax.set_zticks(np.linspace(lo[2],hi[2],4));ax.set_box_aspect(hi-lo);ax.view_init(elev=48,azim=-63)
ax=fig.add_subplot(122);order=np.argsort(tri[:,:,2].mean(1));ax.add_collection(PolyCollection(tri[order,:,:2],facecolors=fc[order],edgecolors='none',antialiaseds=False));ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='X [m]',ylabel='Y [m]',title='Saved colored mesh | top view');ax.set_aspect('equal');ax.set_facecolor('#eef1f4')
fig.suptitle(f'UR7e + D435i | partial scan: 58 frames, 8 recorded views\n{len(v):,} vertices / {len(t):,} triangles | base_link | 15 mm voxels');fig.savefig(A/'saved_mesh.png',dpi=155);plt.close(fig)
for n in ['final_readonly_state.json','operator_observation.json','events.jsonl','control_regression.log','warmup_test.log','retry_launch.log']:
 shutil.copy2(L/n,E/n)
base=json.loads((R.parent/'validation/source_before.json').read_text());changed=[n for n,h in base.items() if not (W/n).is_file() or hashlib.sha256((W/n).read_bytes()).hexdigest()!=h]
metrics['source_integrity']={'checked':len(base),'changed':changed,'passed':not changed}
(R/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
(R/'figure_manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in A.glob('*.png')},indent=2)+'\n')
print(json.dumps({'map':metrics['map'],'observations':metrics['observations'],'source_integrity':metrics['source_integrity']},indent=2))
