import socket,time,struct,json
from pathlib import Path
root=Path(__file__).parent;b=bytearray()
with socket.create_connection(('192.168.56.100',30001),timeout=3) as s,open(root/'scan_primary.bin','wb') as f,open(root/'robot_messages.jsonl','w',buffering=1) as msgs:
 s.settimeout(1);end=time.monotonic()+600
 while time.monotonic()<end:
  try:a=s.recv(65536)
  except socket.timeout:continue
  if not a:break
  f.write(a);f.flush();b.extend(a)
  while len(b)>=5:
   n,typ=struct.unpack_from('>IB',b)
   if n<5:raise RuntimeError('bad packet')
   if len(b)<n:break
   if typ==20:msgs.write(json.dumps({'time':time.time(),'payload_hex':bytes(b[5:n]).hex(),'text':bytes(b[5:n]).decode(errors='replace')})+'\n')
   del b[:n]
