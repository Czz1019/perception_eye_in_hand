"""Read-only RViz overlay. Test-object truth is visualization-only, never planner input."""
import argparse,json,time
from pathlib import Path
from zipfile import BadZipFile
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output).resolve()
c=load_mapping(out/'mapping.yaml');origin=np.array(c['grid_center'])-np.array(c['grid_size'])/2;res=c['grid_resolution']
rclpy.init();node=Node('stage2_readonly_visualization',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=node.create_publisher(MarkerArray,'/stage2_visualization',10);status={};position=None;last_odom=0.;version=-1;points=[]
def on_status(msg):
 global status
 value=json.loads(msg.data)
 if value.get('run_id')==str(out):status=value
def on_odom(msg):
 global position,last_odom
 position=msg.pose.pose.position;last_odom=time.monotonic()
node.create_subscription(String,'/sim_esdf_sage/status',on_status,10)
node.create_subscription(Odometry,'/obstacle_'+out.name+'/odom',on_odom,qos_profile_sensor_data)
def marker(i,kind,color):
 m=Marker();m.header.frame_id='base_link';m.ns='stage2_view_only';m.id=i;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b,m.color.a=color;return m
def tick():
 global version,points
 new=(status.get('online') or {}).get('version',0)
 if new!=version and new>0:
  try:
   with np.load(out/'online'/f'version_{new:06d}.npz') as d:z,y,x=np.nonzero(d['occupied'])
   xyz=origin+res*np.stack([x,y,z],-1);points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in xyz];version=new
  except (OSError,ValueError,EOFError,BadZipFile):pass
 occupied=marker(0,Marker.CUBE_LIST,(1.,.25,.05,.35));occupied.scale.x=occupied.scale.y=occupied.scale.z=res;occupied.points=points
 cube=marker(1,Marker.CUBE,(.1,1.,.15,1.));cube.scale.x=cube.scale.y=cube.scale.z=.08
 if position is not None and time.monotonic()-last_odom<.5:cube.pose.position=position
 else:cube.action=Marker.DELETE
 label=marker(2,Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.));label.pose.position.x=.25;label.pose.position.y=.25;label.pose.position.z=1.35;label.scale.z=.035
 label.text=status.get('phase','waiting')+'\nSAGE:'+str(status.get('sage_iterations',0))+'  Map:'+str(version)
 pub.publish(MarkerArray(markers=[occupied,cube,label]))
node.create_timer(.1,tick)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
