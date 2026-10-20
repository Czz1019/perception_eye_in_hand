"""Installed launch + frozen-map replay + synthetic TF, no RTDE node."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='87'
from pathlib import Path
import json,time,subprocess,signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,JointState
from tf2_ros import Buffer,TransformListener
from ur7e_nvblox.depth import transform_matrix
from ur7e_auto_esdf.trajectory import load_inputs
from ur7e_perception_planning_core.geometry import JOINTS
root=Path(__file__).resolve().parents[1];out=root/'validation'
saved=json.loads((out/'integration/results.json').read_text())['final_map']
_,_,route,g=load_inputs(*(root/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))
stream=(out/'launch.log').open('w')
p=subprocess.Popen(['/opt/ros/humble/bin/ros2','launch','ur7e_auto_esdf','bringup.launch.py','run_controller:=false','run_rviz:=false','load:='+saved],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('offline_launch_observer',cli_args=['--ros-args','-r','/tf:=/ur7e_auto_esdf/tf','-r','/tf_static:=/ur7e_auto_esdf/tf_static'])
received={};buffer=Buffer(node=node);listener=TransformListener(buffer,node)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for name in ['esdf','surface_esdf']:
 node.create_subscription(PointCloud2,'/ur7e_auto_esdf_mapper/'+name,lambda m,n=name:received.update({n:m.width*m.height}),qos)
pub=node.create_publisher(JointState,'/ur7e_auto_esdf/joint_states',10)
q=np.array(route['waypoints'][0]['q_rad']);result={};end=time.monotonic()+40
try:
 while time.monotonic()<end:
  msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=JOINTS;msg.position=q.tolist();pub.publish(msg)
  rclpy.spin_once(node,timeout_sec=.05)
  try:
   t=transform_matrix(buffer.lookup_transform('base_link','camera_depth_optical_frame',Time()).transform)
   error=float(abs(t-g.fk(q)['camera_depth_optical_frame']).max())
  except Exception:continue
  if all(received.get(n,0)>100 for n in ['esdf','surface_esdf']) and error<1e-6:break
 else:raise AssertionError(received)
 assert 'ur7e_auto_esdf_scan' not in [n for n,_ in node.get_node_names_and_namespaces()]
 assert not any(n.endswith('/execute') for n,_ in node.get_service_names_and_types())
 result=dict(passed=True,late_subscriber_points=received,tf_matrix_max_error=error,rtde_node_started=False)
finally:
 node.destroy_node();rclpy.shutdown()
 if p.poll() is None:p.send_signal(signal.SIGINT)
 try:p.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();result['forced_kill']=True
 stream.close();result['launch_exit_code']=p.returncode
 (out/'launch_results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 assert result.get('passed') and p.returncode==0
