"""Reproduce recorded interface check against a running native node.
Sources official overlay and ROS domain as in user guide. Writes live_map.nvblx
and service_validation.json beside this file; rerunning replaces those artifacts.
"""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nvblox_msgs.srv import EsdfAndGradients, FilePath
rclpy.init();n=Node('native_service_validation');result={}
def call(t,name,req):
 c=n.create_client(t,name)
 if not c.wait_for_service(timeout_sec=10):raise RuntimeError(name+' unavailable')
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=30)
 if not f.done():raise RuntimeError(name+' timed out')
 return f.result()
r=EsdfAndGradients.Request();r.use_aabb=True;r.frame_id='base_link';r.aabb_min_m.x=r.aabb_min_m.y=r.aabb_min_m.z=10.;r.aabb_size_m.x=r.aabb_size_m.y=r.aabb_size_m.z=.09
v=call(EsdfAndGradients,'/ur7e_nvblox/get_esdf_and_gradient',r)
result['esdf_service']={'success':v.success,'voxel_size':v.voxel_size_m,'dimensions':[{'label':d.label,'size':d.size,'stride':d.stride} for d in v.esdf_and_gradients.layout.dim],'values':sorted(set(v.esdf_and_gradients.data)),'length':len(v.esdf_and_gradients.data)}
assert v.success and abs(v.voxel_size_m-.03)<1e-7
r=FilePath.Request();r.file_path=str(Path(__file__).resolve().parent/'live_map.nvblx');v=call(FilePath,'/ur7e_nvblox/save_map',r)
result['save_map']={'success':v.success,'path':r.file_path,'bytes':Path(r.file_path).stat().st_size};assert v.success
state={}
def cloud(m):
 import numpy as np
 a=np.frombuffer(m.data,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('distance','<f4')]);d=a['distance']
 state.update(points=m.width*m.height,frame=m.header.frame_id,unknown=int((d==1e6).sum()),observed=int((d!=1e6).sum()),fields=[f.name for f in m.fields])
s=n.create_subscription(PointCloud2,'/ur7e_nvblox/esdf_3d',cloud,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE));deadline=time.monotonic()+15
while not state and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
result['full_cloud']=state;assert state['points']==217800 and state['unknown']>0 and state['frame']=='base_link'
print(json.dumps(result,indent=2));(Path(__file__).resolve().parent/'service_validation.json').write_text(json.dumps(result,indent=2))
n.destroy_node();rclpy.shutdown()
