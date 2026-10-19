"""Gazebo-only stage-one experiment. Fixture truth stays in this test process."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
from ur7e_gazebo_esdf_sage.config import load_mapping
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
rclpy.init();node=Node('online_stage_one_tester',parameter_overrides=[Parameter('use_sim_time',value=True)])
commands=[];joints=[];results=[];object_present=False;name='online_test_block';center=np.array([.40,-.12,.50]);size=np.array([.08]*3)
node.create_subscription(Float64MultiArray,'/forward_position_controller/commands',lambda m:commands.append((time.monotonic(),list(m.data))),qos_profile_sensor_data)
def joint(m):
 if all(n in m.name for n in JOINTS):joints.append((time.monotonic(),[m.position[m.name.index(n)] for n in JOINTS]))
node.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
publisher=node.create_publisher(PoseStamped,'/target_pose',10)
def status():
 try:return json.loads((out/'status.json').read_text())
 except (ValueError,FileNotFoundError):return {}
def record(kind,**kw):
 r=dict(kind=kind,sim_time=node.get_clock().now().nanoseconds*1e-9,wall_time=time.monotonic(),**kw);results.append(r);(out/'online_scenario.json').write_text(json.dumps(results,indent=2));print(r,flush=True)
def wait(predicate,seconds):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05);s=status()
  if predicate(s):return s
  if s.get('stopped'):raise RuntimeError(str(s))
 raise TimeoutError(str(status()))
def pause(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
def call(kind,service,request):
 client=node.create_client(kind,service)
 if not client.wait_for_service(timeout_sec=10):raise RuntimeError(service+' unavailable')
 future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=20);result=future.result();node.destroy_client(client)
 if result is None or not result.success:raise RuntimeError(str(result))
 return result

def region_cells():
 s=status();version=(s.get('online') or {}).get('version',0);file=out/'online'/f'version_{version:06d}.npz'
 if not file.exists():return 0
 with np.load(file) as d:occupied=d['occupied']
 c=load_mapping(out/'mapping.yaml');origin=np.array(c['grid_center'])-np.array(c['grid_size'])/2;res=c['grid_resolution']
 low=np.floor((center-size/2-.04-origin)/res).astype(int);high=np.ceil((center+size/2+.04-origin)/res).astype(int)
 return int(occupied[low[2]:high[2]+1,low[1]:high[1]+1,low[0]:high[0]+1].sum())
try:
 s=wait(lambda s:s.get('phase')=='ONLINE_READY' and (s.get('online') or {}).get('frames',0)>=10,1200)
 uid=s['map_uid'];frames=s['frames'];pause(2)
 c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres']);q=np.array(joints[-1][1]);delta=abs(g.spheres(q)-center)-size/2
 gap=float((np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)-g.radii).min());assert gap>.08,gap
 baseline=region_cells();record('baseline_ready',status=s,region_cells=baseline,initial_box_gap_m=gap)
 req=SpawnEntity.Request();req.name=name;req.xml=f'<sdf version="1.6"><model name="{name}"><static>true</static><link name="body"><collision name="collision"><geometry><box><size>0.08 0.08 0.08</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.08 0.08 0.08</size></box></geometry><material><ambient>0.1 0.9 0.1 1</ambient></material></visual></link></model></sdf>';req.initial_pose.position.x=float(center[0]);req.initial_pose.position.y=float(center[1]);req.initial_pose.position.z=float(center[2]);req.initial_pose.orientation.w=1.
 record('spawn_requested',center=center.tolist(),size=size.tolist());call(SpawnEntity,'/spawn_entity',req);object_present=True;record('spawn_completed')
 wait(lambda s:region_cells()>baseline+10,15);record('detected',region_cells=region_cells(),online=status()['online'])
 msg=PoseStamped();msg.header.frame_id='base_link';msg.pose.position.x=float(center[0]);msg.pose.position.y=float(center[1]);msg.pose.position.z=float(center[2]);msg.pose.orientation.w=1.;publisher.publish(msg);record('target_requested',goal=center.tolist())
 wait(lambda s:s.get('phase')=='WAIT_DYNAMIC',60);start=time.monotonic();pause(3)
 samples=np.array([q for t,q in commands if t>=start]);assert len(samples)>30
 command_range=float(np.ptp(samples,axis=0).max());assert command_range<1e-8,command_range
 record('blocked_hold_passed',command_range_rad=command_range,status=status())
 req=DeleteEntity.Request();req.name=name;record('delete_requested');call(DeleteEntity,'/delete_entity',req);object_present=False;record('delete_completed')
 wait(lambda s:region_cells()<=baseline+2,30);record('ray_cleared',region_cells=region_cells(),online=status()['online'])
 s=wait(lambda s:s.get('phase')=='GOAL_REACHED',240)
 events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()];goal=[e for e in events if e['kind']=='goal_reached'][-1]
 assert s['map_uid']==uid and s['frames']==frames and goal['error_m']<.02
 record('resumed_reached',passed=True,goal_event=goal,status=s)
 pause(2);record('complete',passed=True)
except Exception as exc:
 record('failed',passed=False,error=str(exc))
 try:call(Trigger,'/sim_esdf_sage/stop',Trigger.Request())
 except Exception:pass
 raise
finally:
 if object_present:
  try:
   req=DeleteEntity.Request();req.name=name;call(DeleteEntity,'/delete_entity',req);record('cleanup_deleted')
  except Exception:pass
 node.destroy_node();rclpy.shutdown()
