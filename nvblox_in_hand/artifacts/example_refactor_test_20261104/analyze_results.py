from pathlib import Path
import json,re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
r=json.loads((p/'results.json').read_text());s=json.loads((p/'samples.json').read_text())
def timers(name):
 text=(p/(name+'_timings.txt')).read_text();out={}
 for tag,count,total,mean in re.findall(r'^\s*(\S+)\s+(\d+)\s+([\d.]+)\s+\(([\d.]+)',text,re.M):
  out[tag]={'samples':int(count),'total_sec':float(total),'mean_sec':float(mean)}
 return out
analysis={'status':r['status'],'counter_interval_method':'Difference of existing official timer sample counts between static-end and motion-end snapshots; wall endpoints approximate service execution by the phase end, including the intervening short stats-save gap. No production metrics were added.','modes':{}}
for mode in ('official3d','official2d','diagnostic3d'):
 a=timers(mode+'_static');b=timers(mode+'_wrist_motion')
 dt=r['phases'][mode+'_wrist_motion']['end_monotonic']-r['phases'][mode+'_static']['end_monotonic']
 d={'counter_interval_wall_seconds':dt,'from_snapshot':mode+'_static_timings.txt','to_snapshot':mode+'_wrist_motion_timings.txt'}
 for tag,label in [('ros/depth/integrate','depth_integrated'),('ros/color/integrate','color_integrated'),('ros/esdf','esdf_updated')]:
  n=b[tag]['samples']-a[tag]['samples'];total=b[tag]['total_sec']-a[tag]['total_sec']
  d[label]={'sample_difference':n,'wall_hz':n/dt,'host_timer_mean_ms':total/n*1000}
 log=(p/('mapping_'+mode+'.log')).read_text()
 vals=[float(t)*1000 for n,t in re.findall(r'ros/esdf_integration\s+(\d+)\s+([-\d.]+)',log) if int(n)>=100]
 d['official_rolling_mean_input_age_ms']={'min':min(vals),'max':max(vals),'median_of_windows':float(np.median(vals)),'windows':len(vals),'not_screen_latency':True}
 phase=r['phases'][mode+'_wrist_motion']
 for key in ('mesh_wall_hz','esdf_cloud_wall_hz','esdf_slice_wall_hz','esdf_cloud_points','esdf_slice_points','cloud_age_latest_clock_sec','rgb_input_wall_hz','depth_input_wall_hz','simulation_rtf'):
  if key in phase:d[key]=phase[key]
 # Emulate only the upstream image timestamp gate to see how often the same
 # 30Hz target rejects jittered intervals; this is explanatory, not fusion truth.
 begin=r['phases'][mode+'_static']['end_monotonic'];end=phase['end_monotonic']
 rows=[(t,stamp) for t,stamp in s['depth'] if begin<=t<=end]
 accepted=[];last=float('-inf')
 for t,stamp in rows:
  if stamp-last>1/30:accepted.append(t);last=stamp
 if len(rows)>1:
  stamps=np.array([v[1] for v in rows]);delta=np.diff(stamps)
  d['depth_timestamp_gate_diagnostic']={'received':len(rows),'accepted_by_gate':len(accepted),'gate_only_wall_hz':(len(accepted)-1)/(accepted[-1]-accepted[0]),'positive_intervals_p50_ms':float(np.median(delta[delta>0])*1000),'interval_fraction_le_1_over_30':float(np.mean(delta<=1/30)),'not_actual_fusion_count':True}
 analysis['modes'][mode]=d
(p/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2)+'\n')
labels=['Official 3D','Official 2D','Diagnostic 3D']
fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
x=np.arange(3);w=.25
axes[0].bar(x-w,[analysis['modes'][m]['depth_integrated']['wall_hz'] for m in analysis['modes']],w,label='Depth integrated')
axes[0].bar(x,[analysis['modes'][m]['esdf_updated']['wall_hz'] for m in analysis['modes']],w,label='ESDF update calls')
axes[0].bar(x+w,[analysis['modes'][m]['mesh_wall_hz'] for m in analysis['modes']],w,label='Mesh messages')
axes[0].axhline(30,color='gray',linestyle='--',label='Depth/ESDF target')
axes[0].set_xticks(x,labels);axes[0].set_ylabel('Hz (wall time)');axes[0].set_ylim(0,35);axes[0].legend(fontsize=8)
d=r['backends']['diagnostic3d']['cloud_snapshot'];axes[1].bar(['Outside','Inside'],[d['outside'],d['inside']],color=['#2d9f75','#c8653e'])
for i,v in enumerate([d['outside'],d['inside']]):axes[1].text(i,v+100,str(v),ha='center')
axes[1].set_ylabel('Observed ESDF voxels');axes[1].set_title('Diagnostic snapshot: 14,791 voxels')
fig.savefig(p/'test_summary.png',dpi=160);plt.close(fig)
print(json.dumps(analysis,ensure_ascii=False,indent=2))
