"""Analyze directly recorded ESDF headers and receipt times from completed runs."""
from pathlib import Path
import json,numpy as np,csv
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());f=json.loads((p/'fusion_counts.json').read_text());records=json.loads((p/'cloud_records.json').read_text());depth=json.loads((p/'depth_receipts.json').read_text());receipts={}
for t,ns in depth:receipts.setdefault(ns,t)
def percentiles(v):
 return {k:float(np.percentile(v,q)) for k,q in [('p50',50),('p95',95),('p99',99)]} if len(v) else None
out={'status':r['status'],'scope':'New nine-run voxel size 0.02/0.03/0.05m observed 3D ESDF output comparison; all use diagnostic backend and identical RViz/export load. Header advance is associated newest depth stamp, not geometric-change or GPU kernel completion FPS.','rounds':[],'summary':{}}
for fc in f['rounds']:
 mode=fc['mode'];phase=r['phases'][mode+'_measure'];begin=phase['start_monotonic'];end=phase['end_monotonic'];events=[x for x in records if begin<=x['receive_wall']<=end]
 assert len(events)>10 and all(x['header_ns']>0 and x['frame']=='base_link' for x in events)
 ts=np.array([x['receive_wall'] for x in events]);hs=np.array([x['header_ns'] for x in events],dtype=np.int64);iv=np.diff(ts)*1000;diff=np.diff(hs);span=float(ts[-1]-ts[0]);changes=int((diff>0).sum());ages=np.array([x['age_ms'] for x in events if x['age_ms'] is not None]);lag=np.array([(x['receive_wall']-receipts[x['header_ns']])*1000 for x in events if x['header_ns'] in receipts]);points=[x['points'] for x in events]
 streak=1;maxstreak=1
 for delta in diff:
  streak=streak+1 if delta==0 else 1;maxstreak=max(streak,maxstreak)
 d={'mode':mode,'fusion_gate_hz':fc['fusion_gate_hz'],'voxel_size_m':fc['voxel_size_m'],'repeat':fc['repeat'],'window_begin':begin,'window_end':end,'messages':len(events),'esdf_call_hz':fc['esdf_call_hz'],'depth_fusion_hz':fc['depth_fusion_hz'],'color_fusion_hz':fc['color_fusion_hz'],'input_depth_hz':fc['depth_input_wall_hz'],'input_rgb_hz':fc['rgb_input_wall_hz'],'cloud_hz':(len(events)-1)/span,'associated_input_advance_hz':changes/span,'distinct_associated_input_stamps':len(np.unique(hs)),'repeated_adjacent_header_fraction':float((diff==0).mean()),'backward_header_count':int((diff<0).sum()),'max_identical_header_streak':maxstreak,'cloud_age_ms':percentiles(ages),'negative_age_count':int((ages<0).sum()),'receive_interval_ms':percentiles(iv),'observer_input_to_cloud_ms':percentiles(lag),'matched_input_receipts':len(lag),'negative_observer_lag_count':int((lag<0).sum()),'points':{'min':min(points),'max':max(points)},'mesh_hz':fc['mesh_wall_hz'],'simulation_rtf':fc['simulation_rtf']}
 out['rounds'].append(d)
for gate in [.02,.03,.05]:
 rows=[x for x in out['rounds'] if x['voxel_size_m']==gate];group={'repetitions':len(rows)}
 for key in ['esdf_call_hz','cloud_hz','associated_input_advance_hz','repeated_adjacent_header_fraction','depth_fusion_hz','color_fusion_hz','mesh_hz']:
  vals=[x[key] for x in rows];group[key]={'mean':float(np.mean(vals)),'sample_std':float(np.std(vals,ddof=1)) if len(vals)>1 else None,'min':min(vals),'max':max(vals)}
 ages=[];intervals=[];observer=[];negative=0;count=0
 for row in rows:
  ev=[x for x in records if row['window_begin']<=x['receive_wall']<=row['window_end']];ages.extend(x['age_ms'] for x in ev if x['age_ms'] is not None);intervals.extend(np.diff([x['receive_wall'] for x in ev])*1000);observer.extend((x['receive_wall']-receipts[x['header_ns']])*1000 for x in ev if x['header_ns'] in receipts);count+=len(ev)
 group['pooled_age_ms']=percentiles(ages);group['pooled_receive_interval_ms']=percentiles(intervals);group['pooled_observer_input_to_cloud_ms']=percentiles(observer);group['messages']=count;group['negative_age_count']=sum(x['negative_age_count'] for x in rows);group['backward_header_count']=sum(x['backward_header_count'] for x in rows);group['points_min']=min(x['points']['min'] for x in rows);group['points_max']=max(x['points']['max'] for x in rows);out['summary'][str(gate)]=group
(p/'analysis.json').write_text(json.dumps(out,indent=2)+'\n')
with (p/'esdf_realtime_comparison.csv').open('w',encoding='utf-8-sig',newline='') as file:
 writer=csv.writer(file);writer.writerow(['体素大小m','重复次数','ESDF调用均值Hz','3D点云发布均值Hz','关联新深度输出均值Hz','相邻重复时间戳均值%','输入年龄P50ms','输入年龄P95ms','接收间隔P95ms'])
 for gate,d in out['summary'].items():writer.writerow([gate,d['repetitions']]+[round(d[k]['mean'],3) for k in ['esdf_call_hz','cloud_hz','associated_input_advance_hz']]+[round(d['repeated_adjacent_header_fraction']['mean']*100,2),round(d['pooled_age_ms']['p50'],2),round(d['pooled_age_ms']['p95'],2),round(d['pooled_receive_interval_ms']['p95'],2)])
print(json.dumps(out['summary'],indent=2))
