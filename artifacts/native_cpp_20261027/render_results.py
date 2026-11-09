"""Render recorded measurements; never synthesize geometry or timing samples.
Run: PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/esdf_mpl /usr/bin/python3 -B render_results.py
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from PIL import Image
import io
P=Path(__file__).resolve().parent
D={k:json.loads((P/(k+'_003.json')).read_text()) for k in ['python','native']}
plt.rcParams.update({'font.size':10,'figure.facecolor':'white'})
norm=Normalize(-.15,.15)
fig=plt.figure(figsize=(13,8))
for j,(phase,title) in enumerate([('stationary','Stationary'),('moving','Moving wrist')]):
 for i,(key,label) in enumerate([('python','Python / Torch'),('native','Native C++ / official ROS')]):
  z=np.load(P/(key+'_003_'+phase+'.npz'));a=fig.add_subplot(2,2,j*2+i+1,projection='3d');x=z['xyz']
  sc=a.scatter(x[:,0],x[:,1],x[:,2],c=z['distance'],cmap='coolwarm_r',norm=norm,s=1.5)
  a.view_init(23,-58);a.set(xlim=(-1,1),ylim=(-1,1),zlim=(-.25,1.25),xlabel='base_link X [m]',ylabel='Y [m]',zlabel='Z [m]')
  a.set_title(f'{label} | {title} | {len(x):,} observed-band voxels')
fig.suptitle('Recorded Gazebo ESDF | 3 cm voxels | 640 x 360 depth | 2 x 2 x 1.5 m export ROI',fontsize=13)
fig.subplots_adjust(left=.02,right=.90,bottom=.02,top=.91,wspace=.05,hspace=.15)
cax=fig.add_axes([.93,.2,.014,.6]);fig.colorbar(sc,cax=cax,label='Signed ESDF [m] (unknown excluded)')
fig.savefig(P/'esdf_comparison.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(1,2,figsize=(12,4.2));x=np.arange(2);w=.35
for i,(key,label,color) in enumerate([('python','Python / Torch','#5b7cab'),('native','Native C++','#149d8e')]):
 rates=[D[key][p]['received_cloud_hz'] for p in ['stationary','moving']]
 lat=[D[key][p]['observed_input_to_cloud_ms'] for p in ['stationary','moving']]
 pos=x+(i-.5)*w
 bars=ax[0].bar(pos,rates,w,label=label,color=color);ax[0].bar_label(bars,fmt='%.2f',padding=3)
 bars=ax[1].bar(pos,[v['p50'] for v in lat],w,label=label,color=color);ax[1].bar_label(bars,fmt='%.2f',padding=3)
 ax[1].errorbar(pos,[v['p50'] for v in lat],yerr=[np.zeros(2),[v['p95']-v['p50'] for v in lat]],fmt='none',color='#333333',capsize=5)
for a in ax:a.set_xticks(x,['Stationary (60 s)','Moving (60 s)']);a.spines[['top','right']].set_visible(False);a.grid(axis='y',alpha=.2);a.set_axisbelow(True)
ax[0].set(ylabel='Unique-stamp cloud messages / wall second',ylim=(0,25),title='Actual output: input limited; no old-map replay')
ax[1].set(ylabel='Common-input arrival to cloud receipt [ms]',ylim=(0,24),title='Median latency; whisker = P95')
ax[0].legend(loc='upper right');fig.tight_layout();fig.savefig(P/'performance.png',dpi=170);plt.close(fig)
f=np.load(P/'native_003_frames.npz');frames=[];n=len([k for k in f.files if k.startswith('xyz_')]);t0=int(f['stamp_0'])
for i in range(n):
 fig=plt.figure(figsize=(7.2,5.4));a=fig.add_subplot(111,projection='3d');xyz=f[f'xyz_{i}'];dist=f[f'distance_{i}']
 a.scatter(xyz[:,0],xyz[:,1],xyz[:,2],c=dist,cmap='coolwarm_r',norm=norm,s=2)
 a.view_init(23,-58);a.set(xlim=(-1,1),ylim=(-1,1),zlim=(-.25,1.25),xlabel='base_link X [m]',ylabel='Y [m]',zlabel='Z [m]')
 t=(int(f[f'stamp_{i}'])-t0)*1e-9
 a.set_title(f'Native C++ | recorded source stamp +{t:.2f} s | {len(xyz):,} voxels\n3 cm / 640 x 360 | 2 FPS sampled preview, mapper ~18 Hz',fontsize=11)
 fig.tight_layout();buf=io.BytesIO();fig.savefig(buf,format='png',dpi=100);plt.close(fig);buf.seek(0);frames.append(Image.open(buf).convert('RGB'))
frames[0].save(P/'native_live.gif',save_all=True,append_images=frames[1:],duration=500,loop=0)
a=np.load(P/'native_003_stationary.npz');b=np.load(P/'python_003_stationary.npz')
def keyed(z):return {tuple(k):float(d) for k,d in zip(np.rint(z['xyz']/.03-.5).astype(int),z['distance'])}
A=keyed(a);B=keyed(b);keys=A.keys()&B.keys();err=np.array([abs(A[k]-B[k]) for k in keys]);summary={'native_points':len(A),'python_points':len(B),'common_voxel_centres':len(keys),'distance_abs_error_m':{'p50':float(np.percentile(err,50)),'p95':float(np.percentile(err,95)),'max':float(err.max())},'note':'Sequential live runs; geometry agreement, not ground-truth error.'}
(P/'geometry_comparison.json').write_text(json.dumps(summary,indent=2));print(summary)
