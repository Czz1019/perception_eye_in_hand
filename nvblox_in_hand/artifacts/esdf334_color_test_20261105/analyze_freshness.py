import json,bisect,pathlib,numpy as np
p=pathlib.Path(__file__).resolve().parent
s=json.loads((p/'samples.json').read_text());r=json.loads((p/'results.json').read_text())
ct=[x[0] for x in s['clocks']];out={'method':'Reconstruct associated depth header stamp from recorded latest /clock minus recorded cloud age; round to microseconds. This identifies associated input stamps, not geometric changes or GPU completion.', 'motion_phases':{}}
for mode in ('baseline3d','diagnostic3d','diagnostic40'):
 phase=r['phases'][mode+'_wrist_motion'];rows=[x for x in s['cloud'] if phase['start_monotonic']<=x[0]<=phase['end_monotonic']]
 stamps=[]
 for t,n,age in rows:
  i=bisect.bisect_right(ct,t)-1
  if i>=0 and age is not None:stamps.append(round((s['clocks'][i][1]-age)*1e6))
 dur=rows[-1][0]-rows[0][0];iv=np.diff([x[0] for x in rows])*1000
 out['motion_phases'][mode]={'messages':len(rows),'distinct_associated_depth_stamps':len(set(stamps)), 'associated_stamp_changes_wall_hz':sum(a!=b for a,b in zip(stamps,stamps[1:]))/dur,'consecutive_repeated_stamp_fraction':sum(a==b for a,b in zip(stamps,stamps[1:]))/max(1,len(stamps)-1),'receive_interval_ms':{'p50':float(np.percentile(iv,50)),'p95':float(np.percentile(iv,95))}}
(p/'display_freshness.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
