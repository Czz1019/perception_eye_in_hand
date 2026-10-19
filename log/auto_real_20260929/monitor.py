import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
root=Path('/home/wqj/realtime_nvblox_ws/log/auto_real_20260929')
rclpy.init();n=Node('automatic_scan_test_monitor')
data={};changed={};f=open(root/'events.jsonl','a',buffering=1)
def callback(name,msg):
 value=json.loads(msg.data);data[name]=value
 key=(value.get('phase'),value.get('waypoint'),value.get('error'),value.get('epoch'),value.get('gate_frames'))
 if key!=changed.get(name):
  event={'time':time.time(),'source':name,**value};f.write(json.dumps(event)+'\n');print(json.dumps(event),flush=True);changed[name]=key
 tmp=root/'status.tmp.json';tmp.write_text(json.dumps({'received':time.time(),**data},indent=2));tmp.replace(root/'status.json')
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/ur7e_auto_scan/status',lambda m:callback('controller',m),qos)
n.create_subscription(String,'/ur7e_auto_mapper/capture_status',lambda m:callback('mapper',m),10)
def joint(m):data['joints']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'q':list(m.position),'dq':list(m.velocity)}
n.create_subscription(JointState,'/ur7e_auto/joint_states',joint,5)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
 f.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
