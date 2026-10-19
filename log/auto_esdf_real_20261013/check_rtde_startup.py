import socket,time,json,threading,struct
from pathlib import Path
import rtde_control,rtde_receive
root=Path(__file__).parent;raw=bytearray();done=threading.Event();states=[]
def primary():
 with socket.create_connection(('192.168.56.100',30001),timeout=3) as s:
  s.settimeout(.2)
  while not done.is_set():
   try:raw.extend(s.recv(65536))
   except socket.timeout:pass
r=rtde_receive.RTDEReceiveInterface('192.168.56.100',125.)
def feedback():
 while not done.wait(.01):states.append(dict(time=time.time(),runtime=r.getRuntimeState(),q=r.getActualQ(),dq=r.getActualQd(),safety=r.getSafetyMode()))
t=threading.Thread(target=primary);f=threading.Thread(target=feedback);t.start();f.start();c=None;result={}
try:
 c=rtde_control.RTDEControlInterface('192.168.56.100',500.,5)
 result['started']=True;result['program_running']=c.isProgramRunning()
except Exception as e:result['error']=str(e)
finally:
 if c is not None:
  result['stop_script']=c.stopScript();c.disconnect()
 time.sleep(.5);done.set();t.join();f.join();r.disconnect()
 (root/'rtde_startup_primary.bin').write_bytes(raw)
 (root/'rtde_startup_feedback.json').write_text(json.dumps(states))
 (root/'rtde_startup_result.json').write_text(json.dumps(result,indent=2))
 print(result)
 i=0
 while i+5<=len(raw):
  n,typ=struct.unpack_from('>IB',raw,i)
  if n<5 or i+n>len(raw):break
  if typ==20:print('robot_message',repr(bytes(raw[i+5:i+n])))
  i+=n
