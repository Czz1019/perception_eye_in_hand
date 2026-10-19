"""Installed launcher, namespaced calibrated TF, and frozen-map visualization; domain 85."""
import json,os,signal,subprocess,threading,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState,PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from ur7e_auto_reconstruction.trajectory import load_inputs
from ur7e_nvblox.depth import transform_matrix
assert os.environ.get('ROS_DOMAIN_ID')=='85'
root=Path(__file__).resolve().parents[1];ws=root.parents[1]
_,_,route,g=load_inputs(*(root/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))
q=route['waypoints'][0]['q_rad'];surface=sorted((root/'validation/integration/maps').glob('scan_*/surface.npz'))[-1]
log=open(root/'validation/launch_smoke.log','w')
p=subprocess.Popen([str(ws/'run_ur7e_auto_reconstruction.sh'),'run_controller:=false','run_rviz:=false','load:='+str(surface)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();print('DOMAIN',rclpy.get_default_context().get_domain_id(),flush=True);node=Node('smoke_observer',cli_args=['--ros-args','-r','/tf:=/ur7e_auto/tf','-r','/tf_static:=/ur7e_auto/tf_static'])
buffer=Buffer();listener=TransformListener(buffer,node)
pub=node.create_publisher(JointState,'/ur7e_auto/joint_states',10)
status=[];cloud=[]
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/ur7e_auto_mapper/status',lambda m:status.append(json.loads(m.data)),qos)
node.create_subscription(PointCloud2,'/ur7e_auto_mapper/surface_points',cloud.append,qos)
def state():
 m=JointState();m.header.stamp=node.get_clock().now().to_msg();m.name=route['joint_names'];m.position=q;m.velocity=[0.]*6;pub.publish(m)
node.create_timer(.02,state)
ex=SingleThreadedExecutor();ex.add_node(node);t=threading.Thread(target=ex.spin,daemon=True);t.start();result={}
try:
 end=time.monotonic()+45
 while not status or not cloud or not buffer.can_transform('base_link','camera_depth_optical_frame',Time()):
  if p.poll() is not None or time.monotonic()>end:raise AssertionError(f'Launch/TF/frozen surface timeout: status={status[-1:]}, clouds={len(cloud)}, frames={buffer.all_frames_as_yaml()}, nodes={node.get_node_names_and_namespaces()}')
  time.sleep(.1)
 pose=transform_matrix(buffer.lookup_transform('base_link','camera_depth_optical_frame',Time()).transform)
 assert np.allclose(pose,g.fk(q)['camera_depth_optical_frame'],atol=1e-8)
 assert status[-1]['phase']=='FROZEN' and cloud[-1].width>1000
 result.update(installed_launch=True,calibrated_tf_max_error=float(abs(pose-g.fk(q)['camera_depth_optical_frame']).max()),
               late_surface_vertices=cloud[-1].width,phase=status[-1]['phase'],no_control_node=not any(n=='ur7e_auto_scan' for n,ns in node.get_node_names_and_namespaces()))
 assert result['no_control_node'];result['passed']=True
finally:
 ex.shutdown();t.join();node.destroy_node();rclpy.shutdown()
 if p.poll() is None:p.send_signal(signal.SIGINT)
 try:p.wait(12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();result['clean_shutdown']=False
 else:result['clean_shutdown']=p.returncode==0
 log.close();(root/'validation/launch_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
