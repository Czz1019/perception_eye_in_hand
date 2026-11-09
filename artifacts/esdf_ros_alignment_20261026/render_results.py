"""Render actual saved ROS samples; does not generate or alter ESDF values."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

root=Path(__file__).resolve().parent
fig=plt.figure(figsize=(13,5.5),layout='constrained')
for index,(name,label) in enumerate([('final_native_005','0.05 m voxels'),('final_native_010','0.10 m voxels')],1):
 data=np.load(root/(name+'.npz'))
 xyz,d=data['xyz'],data['distance']
 ax=fig.add_subplot(1,2,index,projection='3d')
 plot=ax.scatter(*xyz.T,c=d,cmap='coolwarm',norm=Normalize(-.15,.15),s=5,alpha=.85,rasterized=True)
 ax.set(xlim=(-1,1),ylim=(-1,1),zlim=(-.25,1.25),xlabel='base_link x (m)',ylabel='y (m)',zlabel='z (m)',
        title=f'{label} | {len(xyz):,} observed samples')
 ax.set_box_aspect((2,2,1.5));ax.view_init(elev=24,azim=-55)
fig.colorbar(plot,ax=fig.axes,shrink=.65,pad=.04,label='Signed ESDF distance (m)')
fig.suptitle('Gazebo / UR7e eye-in-hand: actual observed ESDF output\nDifferent capture times; visual samples, not a reconstruction accuracy comparison',fontsize=13)
fig.savefig(root/'observed_esdf_comparison.png',dpi=150)
plt.close(fig)

fig,ax=plt.subplots(figsize=(8.5,4.3),layout='constrained')
labels=['Old dense / 5 cm','New native / 5 cm','New native / 10 cm']
files=['cyclone_baseline_dense_005','final_native_005','final_native_010']
x=np.arange(len(labels));width=.25
for offset,phase,title in [(-width,'stationary','Stationary output'),(0,'moving','Moving output')]:
 rates=[json.loads((root/(p+'.json')).read_text())[phase]['received_cloud_hz'] for p in files]
 rects=ax.bar(x+offset,rates,width,label=title)
 ax.bar_label(rects,fmt='%.1f',padding=3)
inputs=[json.loads((root/(p+'.json')).read_text())['stationary']['rates_hz']['depth_received'] for p in files]
rects=ax.bar(x+width,inputs,width,label='Stationary depth input');ax.bar_label(rects,fmt='%.1f',padding=3)
ax.axhline(30,color='#777777',ls='--',lw=1,label='Configured target (not measured)')
ax.set(xticks=x,xticklabels=labels,ylabel='Received wall-clock Hz',ylim=(0,35),title='Same Cyclone DDS / Gazebo scene, sequential runs')
ax.legend(fontsize=9,loc='upper left');ax.spines[['top','right']].set_visible(False)
fig.savefig(root/'rates_comparison.png',dpi=150)
plt.close(fig)
