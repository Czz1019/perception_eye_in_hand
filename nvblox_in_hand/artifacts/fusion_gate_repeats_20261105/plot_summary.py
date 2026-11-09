from pathlib import Path
import json,csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;a=json.loads((p/'analysis.json').read_text());gates=[30.,33.4,40.];x=np.arange(3);w=.23
fig,ax=plt.subplots(figsize=(10,5.3),layout='constrained')
for j,(key,title,color) in enumerate([('depth_fusion_hz','Depth integrations','#2676b7'),('color_fusion_hz','Color integrations','#d58924'),('esdf_call_hz','ESDF update calls','#46834d')]):
 means=[a['summary'][str(g)][key]['mean'] for g in gates];std=[a['summary'][str(g)][key]['sample_std'] for g in gates];xs=x+(j-1)*w
 ax.bar(xs,means,w,label=title,color=color,alpha=.8,yerr=std,capsize=4)
 for i,g in enumerate(gates):
  vals=[r[key] for r in a['rounds'] if r['fusion_gate_hz']==g];ax.scatter(np.repeat(xs[i],len(vals))+np.linspace(-.025,.025,len(vals)),vals,s=20,c='black',zorder=4)
 for xi,m in zip(xs,means):ax.text(xi,m+.65,f'{m:.2f}',ha='center',fontsize=9)
ax.axhline(30,ls='--',c='gray',label='30 Hz camera target');ax.set_xticks(x,['Fusion gate 30 Hz','Fusion gate 33.4 Hz','Fusion gate 40 Hz']);ax.set_ylim(0,37);ax.set_ylabel('Count / wall-clock second');ax.set_title('Nine fresh-map runs: 3 repetitions per fusion gate\n640x480, 3cm voxels, 3m range; ESDF gate fixed at 33.4 Hz');ax.legend(loc='lower right',fontsize=9);ax.grid(axis='y',alpha=.18);fig.savefig(p/'fusion_repeats_summary.png',dpi=180);plt.close(fig)
with (p/'round_results.csv').open('w',encoding='utf-8-sig',newline='') as f:
 writer=csv.writer(f);writer.writerow(['顺序','融合门限Hz','重复轮','深度输入Hz','RGB输入Hz','深度融合Hz','颜色融合Hz','ESDF调用Hz','Mesh发布Hz'])
 for i,r in enumerate(a['rounds'],1):writer.writerow([i,r['fusion_gate_hz'],r['repeat']]+[f'{r[k]:.3f}' for k in ['depth_input_wall_hz','rgb_input_wall_hz','depth_fusion_hz','color_fusion_hz','esdf_call_hz','mesh_wall_hz']])
print(p/'fusion_repeats_summary.png')
