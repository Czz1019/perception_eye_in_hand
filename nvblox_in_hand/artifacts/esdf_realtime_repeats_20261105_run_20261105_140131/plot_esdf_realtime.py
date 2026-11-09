from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;a=json.loads((p/'analysis.json').read_text());gates=['30.0','33.4','40.0'];x=np.arange(3);fig,axes=plt.subplots(1,2,figsize=(14,4.8),layout='constrained');w=.24
for j,(key,label,color) in enumerate([('esdf_call_hz','ESDF calls','#888888'),('cloud_hz','3D cloud messages','#2278b5'),('associated_input_advance_hz','Associated depth stamp advances','#429062')]):
 means=[a['summary'][g][key]['mean'] for g in gates];std=[a['summary'][g][key]['sample_std'] for g in gates];axes[0].bar(x+(j-1)*w,means,w,yerr=std,capsize=3,label=label,color=color)
 for xi,y in zip(x+(j-1)*w,means):axes[0].text(xi,y+.5,f'{y:.2f}',ha='center',fontsize=9)
axes[0].axhline(30,color='gray',ls='--',lw=1);axes[0].set_ylim(0,37);axes[0].set_ylabel('Events / wall second');axes[0].set_xticks(x,['Fusion 30','Fusion 33.4','Fusion 40']);axes[0].legend(fontsize=8,loc='lower right');axes[0].set_title('Same ESDF/view gates: 33.4 Hz; three repeats per group')
for j,(key,label,color) in enumerate([('p50','P50 input age','#be8226'),('p95','P95 input age','#b74e47')]):
 vals=[a['summary'][g]['pooled_age_ms'][key] for g in gates];xs=x+(j-.5)*.32;axes[1].bar(xs,vals,.32,label=label,color=color)
 for xi,y in zip(xs,vals):axes[1].text(xi,y+.6,f'{y:.0f}',ha='center',fontsize=9)
axes[1].set_ylabel('Latest received /clock minus cloud header (ms)');axes[1].set_xticks(x,['Fusion 30','Fusion 33.4','Fusion 40']);axes[1].legend(fontsize=9);axes[1].set_title('Receiver-side input-age proxy; excludes screen rendering')
for ax in axes:ax.grid(axis='y',alpha=.15)
fig.savefig(p/'esdf_realtime_comparison.png',dpi=180);plt.close(fig)
