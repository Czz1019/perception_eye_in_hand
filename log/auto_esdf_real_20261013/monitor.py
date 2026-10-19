import json,time
from pathlib import Path
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import JointState,Image
root=Path('/home/wqj/realtime_nvblox_ws/log/auto_esdf_real_20261013')
(root/'photos').mkdir(exist_ok=True)
rclpy.init();n=Node('auto_esdf_real_test_recorder')
data={};changed={};photos=set();latest=None;f=open(root/'events.jsonl','a',buffering=1)
def photo(m):
 global latest
 if m.encoding not in ('rgb8','bgr8'):return
 a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
 if m.encoding=='bgr8':a=a[:,:,::-1]
 latest=(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,a)
def callback(name,msg):
 value=json.loads(msg.data);data[name]=value
 key=(value.get('phase'),value.get('waypoint'),value.get('error'),value.get('epoch'),value.get('gate_frames'))
 if key!=changed.get(name):
  event={'time':time.time(),'source':name,**value};f.write(json.dumps(event)+'\n');changed[name]=key
 c=data.get('controller',{});m=data.get('mapper',{});wp=c.get('waypoint')
 if c.get('phase')=='CAPTURING' and m.get('gate_frames',0)>=2 and wp and wp not in photos and latest:
  stamp,a=latest;PILImage.fromarray(a).save(root/'photos'/f'{wp}_rgb.png')
  (root/'photos'/f'{wp}_rgb.json').write_text(json.dumps({'image_stamp':stamp,'recorded':time.time(),'controller':c,'mapper':m,'joints':data.get('joints')},indent=2));photos.add(wp)
 tmp=root/'status.tmp.json';tmp.write_text(json.dumps({'received':time.time(),**data},indent=2));tmp.replace(root/'status.json')
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/ur7e_auto_esdf_scan/status',lambda m:callback('controller',m),qos)
n.create_subscription(String,'/ur7e_auto_esdf_mapper/capture_status',lambda m:callback('mapper',m),10)
def joint(m):data['joints']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'q':list(m.position),'dq':list(m.velocity)}
n.create_subscription(JointState,'/ur7e_auto_esdf/joint_states',joint,5)
n.create_subscription(Image,'/camera/camera/color/image_raw',photo,qos_profile_sensor_data)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
 f.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
