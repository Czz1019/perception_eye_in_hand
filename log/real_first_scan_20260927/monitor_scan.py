import json,time
from pathlib import Path
p=Path('log/real_first_scan_20260927/output/status.json');samples=[];start=time.monotonic()
while time.monotonic()-start<30:
    try:samples.append(dict(wall=time.monotonic(),status=json.loads(p.read_text())))
    except (FileNotFoundError,json.JSONDecodeError):pass
    time.sleep(.5)
a,b=samples[0],samples[-1];sa,sb=a['status'],b['status'];duration=b['wall']-a['wall'];ta=sa['perception']['totals'];tb=sb['perception']['totals']
result=dict(duration_s=duration,depth_received_hz=(tb['depth_received']-ta['depth_received'])/duration,completed_field_hz=(sb['frames']-sa['frames'])/duration,first_frames=sa['frames'],last_frames=sb['frames'],last_views=sb['views'],capture_errors=sorted({s['status']['capture_error'] for s in samples if s['status']['capture_error']}),last_status=sb)
Path('log/real_first_scan_20260927/scan_monitor.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
