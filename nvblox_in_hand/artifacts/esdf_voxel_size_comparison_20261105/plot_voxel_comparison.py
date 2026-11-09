from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;a=json.loads((p/'analysis.json').read_text());gates=['0.02','0.03','0.05'];x=np.arange(3);fig,axes=plt.subplots(1,2,figsize=(13,4.8),layout='constrained');w=.24
for j,(k,label,c) in enumerate([('esdf_call_hz','ESDF calls','#888888'),('cloud_hz','esdf_points messages','#2679b5'),('associated_input_advance_hz','Associated new depth stamps','#43885b')]):
 ys=[a['summary'][g][k]['mean'] for g in gates];sd=[a['summary'][g][k]['sample_std'] for g in gates];xs=x+(j-1)*w;axes[0].bar(xs,ys,w,yerr=sd,capsize=3,label=label,color=c)
 for xx,y in zip(xs,ys):axes[0].text(xx,y+.5,f'{y:.2f}',ha='center',fontsize=9)
axes[0].axhline(30,color='gray',ls='--');axes[0].set_ylim(0,max(37,max(a['summary'][g]['esdf_call_hz']['max'] for g in gates)+3));axes[0].set_ylabel('Hz, wall time');axes[0].legend(fontsize=8,loc='lower right');axes[0].set_title('Fusion gate40; ESDF/view gates33.4; three repeats')
counts=[a['summary'][g]['points_max'] for g in gates];axes[1].bar(x,counts,color=['#b8862b','#a67a3b','#966e49'])
for xx,y in zip(x,counts):axes[1].text(xx,y+max(counts)*.015,f'{y:,}',ha='center')
axes[1].set_ylim(0,max(counts)*1.15);axes[1].set_ylabel('Observed ESDF points per message');axes[1].set_title('Same 640x480 input, high stationary camera, 3m range')
for ax in axes:ax.set_xticks(x,['0.02 m','0.03 m','0.05 m']);ax.set_xlabel('voxel_size');ax.grid(axis='y',alpha=.15)
fig.savefig(p/'voxel_size_esdf_frequency.png',dpi=180);plt.close(fig)
