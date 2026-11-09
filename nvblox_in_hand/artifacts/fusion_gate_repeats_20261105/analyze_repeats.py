"""Analyze existing official snapshots; no live ROS or motion commands."""
from pathlib import Path
import json,re,numpy as np,bisect
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());s=json.loads((p/'samples.json').read_text())
def timers(name):
 return {tag:{'n':int(n),'total_s':float(total)} for tag,n,total in re.findall(r'^\s*(\S+)\s+(\d+)\s+([\d.]+)\s+\(', (p/name).read_text(),re.M)}
out={'status':r['status'],'method':'Existing official timer counts differenced between before/after snapshots. Interval is the difference between service request/response wall midpoint estimates; duration bracket saved. Not CUDA completion FPS.','rounds':[],'summary':{}}
for mode,back in r.get('backends',{}).items():
 first=mode+'_static_timings.txt';last=mode+'_measure_timings.txt'
 if not (p/last).exists():continue
 ta=timers(first);tb=timers(last);sa=r['snapshots'][first];sb=r['snapshots'][last];begin=sa['midpoint'];end=sb['midpoint'];dt=end-begin;ph=r['phases'][mode+'_measure'];gate=back['fusion_gate_hz']
 d={'mode':mode,'fusion_gate_hz':gate,'repeat':back['repeat'],'wall_seconds':dt,'wall_duration_bracket_seconds':[sb['wall_begin']-sa['wall_end'],sb['wall_end']-sa['wall_begin']],'source_before':first,'source_after':last}
 for sensor in ['depth','color']:
  tags=[f'ros/{sensor}_image_callback',f'ros/{sensor}',f'ros/{sensor}/integrate'];ns=[tb[x]['n']-ta[x]['n'] for x in tags]
  d[sensor+'_pipeline']={'callbacks':ns[0],'process_entries':ns[1],'integrations':ns[2],'integration_fraction_of_callbacks':ns[2]/max(1,ns[0]),'host_integrate_mean_ms':(tb[tags[2]]['total_s']-ta[tags[2]]['total_s'])*1000/max(1,ns[2])}
  d[sensor+'_fusion_hz']=ns[2]/dt
 d['esdf_call_count']=tb['ros/esdf']['n']-ta['ros/esdf']['n'];d['esdf_call_hz']=d['esdf_call_count']/dt
 for key in ['depth_input_wall_hz','rgb_input_wall_hz','mesh_wall_hz','simulation_rtf']:d[key]=ph.get(key)
 d['input_gate_model']={}
 for sensor,key in [('depth','depth'),('color','rgb')]:
  # Prime a gate model with prior input; this is explanatory and has its own
  # state, not a reconstruction of the mapper's entire per-frame history.
  rows=[x for x in s[key] if begin-5<=x[0]<=end];accepted=[];last_stamp=None
  for t,stamp in rows:
   if last_stamp is None or stamp-last_stamp>1/gate:
    last_stamp=stamp
    if begin<=t<=end:accepted.append(t)
  measured=[x for x in rows if begin<=x[0]<=end];deltas=np.diff([x[1] for x in measured]);positive=deltas[deltas>0]
  d['input_gate_model'][sensor]={'input_messages':len(measured),'model_accepted':len(accepted),'model_accepted_hz':len(accepted)/dt,'positive_stamp_interval_p50_ms':float(np.percentile(positive,50)*1000) if len(positive) else None,'adjacent_interval_at_or_below_gate_fraction':float(np.mean(deltas<=1/gate)) if len(deltas) else None,'not_actual_fusion_count':True}
 out['rounds'].append(d)
for gate in [30.,33.4,40.]:
 rows=[x for x in out['rounds'] if x['fusion_gate_hz']==gate];summary={'repetitions':len(rows)}
 for k in ['depth_fusion_hz','color_fusion_hz','esdf_call_hz','depth_input_wall_hz','rgb_input_wall_hz','mesh_wall_hz','simulation_rtf']:
  values=[x[k] for x in rows if x[k] is not None]
  if values:summary[k]={'mean':float(np.mean(values)),'sample_std':float(np.std(values,ddof=1)) if len(values)>1 else None,'min':min(values),'max':max(values)}
 for sensor in ['depth','color']:
  count={k:sum(x[sensor+'_pipeline'][k] for x in rows) for k in ['callbacks','process_entries','integrations']};count['integration_fraction_of_callbacks']=count['integrations']/max(1,count['callbacks']);summary[sensor+'_pipeline_totals']=count
 out['summary'][str(gate)]=summary
(p/'analysis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
