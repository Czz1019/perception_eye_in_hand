"""Independent Gazebo moving-object experiment; truth never enters the planner."""
import argparse,json,time
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,String
from ur7e_gazebo_esdf_sage.config import load_mapping
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
deadline=time.monotonic()+60
while not all((out/f).exists() for f in ['mapping.yaml','scan.yaml','robot.urdf','status.json']):
 if time.monotonic()>deadline:raise TimeoutError('Simulation output was not initialized')
 time.sleep(.05)
rclpy.init();node=Node('moving_obstacle_tester',parameter_overrides=[Parameter('use_sim_time',value=True)])
results=[];truth=(out/'moving_truth.jsonl').open('w');commands=deque(maxlen=3000);q=None;position=None;present=False;trial='waiting';name='obstacle_'+out.name;namespace='/'+name;live_run=None
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
def now():return node.get_clock().now().nanoseconds*1e-9
def state(m):
 global q
 if all(n in m.name for n in JOINTS):q=np.array([m.position[m.name.index(n)] for n in JOINTS])
def odom(m):
 global position
 v=m.pose.pose.position;position=np.array([v.x,v.y,v.z]);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 truth.write(json.dumps(dict(stamp=t,position=position.tolist(),velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],trial=trial))+'\n');truth.flush()
def live_status(msg):
 global live_run
 live_run=json.loads(msg.data).get('run_id')
node.create_subscription(String,'/sim_esdf_sage/status',live_status,10)
def same_run():return live_run==str(out.resolve())
node.create_subscription(JointState,'/joint_states',state,qos_profile_sensor_data)
node.create_subscription(Odometry,namespace+'/odom',odom,qos_profile_sensor_data)
node.create_subscription(Float64MultiArray,'/forward_position_controller/commands',lambda m:commands.append((now(),m.data)),qos_profile_sensor_data)
goals=node.create_publisher(PoseStamped,'/target_pose',10);vel=node.create_publisher(Twist,namespace+'/cmd_vel',10)
def status():
 try:return json.loads((out/'status.json').read_text())
 except (ValueError,FileNotFoundError):return {}
def record(kind,**data):
 results.append(dict(kind=kind,trial=trial,sim_time=now(),**data));(out/'moving_scenario.json').write_text(json.dumps(results,indent=2));print(results[-1],flush=True)
def wait(fn,timeout=120):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.02)
  if live_run is not None and not same_run():raise RuntimeError('Different simulation run; no control action permitted')
  s=status()
  if s.get('stopped'):raise RuntimeError(str(s))
  if fn(s):return s
 raise TimeoutError(str(status()))
def pause(t):
 start=now();return wait(lambda s:now()-start>=t,t*4+20)
def call(kind,topic,req):
 if not same_run():raise RuntimeError('Run binding missing')
 cli=node.create_client(kind,topic);assert cli.wait_for_service(timeout_sec=10)
 fut=cli.call_async(req);rclpy.spin_until_future_complete(node,fut,timeout_sec=15);r=fut.result();node.destroy_client(cli)
 assert r is not None and r.success,str(r)
def speed(y):
 m=Twist();m.linear.y=y;vel.publish(m)
def goal(xyz):
 m=PoseStamped();m.header.frame_id='base_link';m.pose.position.x=float(xyz[0]);m.pose.position.y=float(xyz[1]);m.pose.position.z=float(xyz[2]);m.pose.orientation.w=1.;goals.publish(m);record('target',xyz=list(xyz))
def visible_cells(center):
 version=(status().get('online') or {}).get('version',0);file=out/'online'/f'version_{version:06d}.npz'
 try:
  with np.load(file) as d:occupied=d['occupied']
 except (OSError,ValueError,EOFError):return 0
 origin=np.array(c['grid_center'])-np.array(c['grid_size'])/2;res=c['grid_resolution'];low=np.floor((center-.08-origin)/res).astype(int);high=np.ceil((center+.08-origin)/res).astype(int)
 return int(occupied[low[2]:high[2]+1,low[1]:high[1]+1,low[0]:high[0]+1].sum())
def spawn(center):
 global present,position
 assert q is not None
 d=abs(g.spheres(q)-center)-.04;gap=float((np.linalg.norm(np.maximum(d,0),axis=-1)+np.minimum(d.max(-1),0)-g.radii).min());assert gap>.08,gap
 position=None;req=SpawnEntity.Request();req.name=name
 req.xml=f'''<sdf version="1.6"><model name="{name}"><link name="body"><gravity>false</gravity><inertial><mass>1</mass><inertia><ixx>0.0011</ixx><iyy>0.0011</iyy><izz>0.0011</izz></inertia></inertial><collision name="collision"><geometry><box><size>0.08 0.08 0.08</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.08 0.08 0.08</size></box></geometry><material><ambient>0.1 0.9 0.1 1</ambient></material></visual></link><plugin name="moving_test" filename="libgazebo_ros_planar_move.so"><ros><namespace>{namespace}</namespace></ros><update_rate>100</update_rate><publish_rate>50</publish_rate><publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf><odometry_frame>world</odometry_frame></plugin></model></sdf>'''
 req.initial_pose.position.x=float(center[0]);req.initial_pose.position.y=float(center[1]);req.initial_pose.position.z=float(center[2]);req.initial_pose.orientation.w=1.
 record('spawn_requested',center=list(center),size=[.08]*3,initial_gap_m=gap);call(SpawnEntity,'/spawn_entity',req);present=True;wait(lambda s:position is not None,10);record('spawned');wait(lambda s:visible_cells(center)>10,15);record('sensor_detected',cells=visible_cells(center))
def remove():
 global present
 speed(0.);req=DeleteEntity.Request();req.name=name;call(DeleteEntity,'/delete_entity',req);present=False;record('deleted')
try:
 s=wait(lambda s:same_run() and s.get('phase')=='ONLINE_READY' and (s.get('online') or {}).get('frames',0)>=20,1800);uid=s['map_uid'];seed_tool=g.fk(q)['tool0'][:3,3].copy();record('ready',map_uid=uid,seed_tool=seed_tool.tolist())
 trial='crossing';spawn(np.array([.25,-.375,.675]));pause(2);record('crossing_begin');speed(-.01);goal([.4,-.12,.5])
 wait(lambda s:position[1]<=-.52,100);speed(0.);record('crossing_end',position=position.tolist(),status=status())
 s=wait(lambda s:s.get('phase')=='GOAL_REACHED',240);record('crossing_reached',status=s);remove();pause(3)
 trial='return';goal(seed_tool);pause(1);wait(lambda s:s.get('phase')=='GOAL_REACHED',240);record('returned')
 trial='blocked';spawn(np.array([.4,-.12,.5]));pause(3);goal([.4,-.12,.5]);wait(lambda s:s.get('phase')=='WAIT_DYNAMIC',90);start=now();pause(3)
 values=np.array([v for t,v in commands if t>=start]);assert len(values)>30;spread=float(np.ptp(values,axis=0).max());assert spread<1e-8;record('blocked_hold',range_rad=spread)
 record('unblocking_begin');speed(.01);wait(lambda s:position[1]>=.16,100);speed(0.);record('unblocking_end',position=position.tolist())
 s=wait(lambda s:s.get('phase')=='GOAL_REACHED',240);assert s['map_uid']==uid and s['frames']==315;record('resumed_reached',status=s);remove();record('complete',passed=True)
except Exception as e:
 record('failed',error=str(e))
 if same_run():speed(0.)
 try:call(Trigger,'/sim_esdf_sage/stop',Trigger.Request())
 except Exception:pass
 raise
finally:
 if present and same_run():
  try:remove()
  except Exception:pass
 truth.close();node.destroy_node();rclpy.shutdown()
