"""Read-only checks of live calibrated TF, rendered depth and ros2_control."""
import argparse,json,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer,TransformListener
from controller_manager_msgs.srv import ListControllers
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,transform_matrix
from ur7e_real_static.model import Geometry
from ur7e_real_static.camera_collision import fixed_transform
from ur7e_gazebo_esdf_sage.self_filter import mask as robot_depth_mask
from design_scan import Fixture
import xml.etree.ElementTree as ET

p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--expect-hold',action='store_true');a=p.parse_args();out=Path(a.output)
rclpy.init();node=Node('esdf_sage_readonly_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
buffer=Buffer(node=node);listener=TransformListener(buffer,node);received={};counts=dict(depth=0,joints=0,commands=0);commands=[];joint_positions=[]
def receive(name,msg):
    received[name]=msg
    if name in counts:counts[name]+=1
    if name=='commands':commands.append(list(msg.data))
    if name=='joints':joint_positions.append(list(msg.position))
subs=[node.create_subscription(kind,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data) for name,kind,topic in [('depth',Image,'/camera/depth/image_rect_raw'),('info',CameraInfo,'/camera/depth/camera_info'),('joints',JointState,'/joint_states'),('commands',Float64MultiArray,'/forward_position_controller/commands')]]
client=node.create_client(ListControllers,'/controller_manager/list_controllers')
until=time.monotonic()+6
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
assert all(n in received for n in ['depth','info','joints'])
depthmsg=received['depth'];info=received['info'];stamp=Time.from_msg(depthmsg.header.stamp)
# Let dynamic TF catch up to the selected image.
until=time.monotonic()+.3
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
real=ET.parse(Path(__file__).resolve().parents[1]/'config/real_calibrated.urdf').getroot()
expected=fixed_transform(real,'tool0','camera_depth_optical_frame')
actual=transform_matrix(buffer.lookup_transform('tool0','camera_depth_optical_frame',Time()).transform)
assert np.max(abs(actual-expected))<2e-6
pose=transform_matrix(buffer.lookup_transform('base_link','camera_depth_optical_frame',stamp).transform)
g=Geometry(out/'robot.urdf',Path(__file__).resolve().parents[1]/'config/collision_spheres.yaml')
centers=np.empty_like(g.local)
for i,link in enumerate(g.links):
    t=transform_matrix(buffer.lookup_transform('base_link',link,stamp).transform);ids=g.link_ids==i
    centers[ids]=g.local[ids]@t[:3,:3].T+t[:3,3]
depth=depth_meters(depthmsg,2,.2,3.);intr=intrinsics_from_info(info,2)
depth[robot_depth_mask(depth,pose,intr,centers,g.radii,g.link_ids,yaml.safe_load((out/'scan.yaml').read_text())['depth_self_mask'])]=0
v,u=np.indices(depth.shape);valid=depth>0
camera=np.stack([(u-intr.cx)*depth/intr.fx,(v-intr.cy)*depth/intr.fy,depth],-1)[valid]
points=camera@pose[:3,:3].T+pose[:3,3]
distance=np.abs(Fixture().query(points))
assert len(distance)>100
result=dict(tool_to_depth_max_matrix_error=float(abs(actual-expected).max()),rendered_surface_median_error_m=float(np.median(distance)),rendered_surface_p95_error_m=float(np.quantile(distance,.95)),valid_depth_points=len(points),received_6s=counts,frame_id=depthmsg.header.frame_id,depth_size=[depthmsg.width,depthmsg.height])
assert client.wait_for_service(timeout_sec=5)
f=client.call_async(ListControllers.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=5)
result['controllers']={x.name:x.state for x in f.result().controller}
result['command_range_rad']=float(np.ptp(commands,axis=0).max())
result['actual_joint_range_rad']=float(np.ptp(joint_positions,axis=0).max())
if a.expect_hold:assert result['command_range_rad']<1e-8 and result['actual_joint_range_rad']<.001
assert result['controllers']['joint_state_broadcaster']=='active' and result['controllers']['forward_position_controller']=='active'
(out/'runtime_probe.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert result['rendered_surface_p95_error_m']<.02,result
node.destroy_node();rclpy.shutdown()
