"""Report measured command arrivals and planning latency without equating rates."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('output');out=Path(p.parse_args().output)
f=[json.loads(x) for x in (out/'sage_feedback.jsonl').read_text().splitlines()]
e=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
r=np.genfromtxt(out/'command_arrivals.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
mask=(r['phase'][1:]=='REACHING')&(r['phase'][:-1]=='REACHING');dw=np.diff(r['wall_time'])[mask];ds=np.diff(r['sim_time'])[mask]
execution=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
q=np.stack([execution[k] for k in execution.dtype.names[10:16]],-1)
mask=(execution['phase'][1:]=='REACHING')&(execution['phase'][:-1]=='REACHING')
dt=np.diff(execution['sim_time'])[mask]
speed=np.abs(np.diff(q,axis=0)[mask]/np.maximum(dt[:,None],1e-9));speed=speed[dt>0]

result=dict(command_arrival_samples_reaching=len(dw),command_arrival_wall_gap_p50_s=float(np.median(dw)),command_arrival_wall_gap_p95_s=float(np.quantile(dw,.95)),command_arrival_wall_gap_max_s=float(dw.max()),max_reaching_command_speed_rad_s=float(speed.max()),optimizer_p95_s=float(np.quantile([x['optimize_seconds'] for x in f],.95)),submission_latency_including_guard_p95_s=float(np.quantile([x['latency']+x['guard_seconds'] for x in f],.95)),background_requests=sum(x['kind']=='background_requested' for x in e),background_failures=sum(x['kind']=='background_failed' for x in e),interpretation='Command receipt intervals include scheduling/transport and are not a publisher frequency guarantee. Accepted-plan latency excludes rejected/overdue optimizer results; see events.jsonl.')
(out/'performance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert result['max_reaching_command_speed_rad_s']<.17
