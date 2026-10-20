"""Simulation-only local outage injection; verifies actual command hold and recovery."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Float64MultiArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
p=argparse.ArgumentParser();p.add_argument('output');out=Path(p.parse_args().output).resolve()
rclpy.init();node=Node('continuous_fault_tester',parameter_overrides=[Parameter('use_sim_time',value=True)])
status={};commands=[];results={}
def receive(msg):
 global status
 value=json.loads(msg.data)
 if value.get('run_id')==str(out):status=value
node.create_subscription(String,'/sim_esdf_sage/status',receive,10)
node.create_subscription(Float64MultiArray,'/forward_position_controller/commands',lambda msg:commands.append((time.monotonic(),list(msg.data))),qos_profile_sensor_data)
pub=node.create_publisher(PoseStamped,'/target_pose',10)
def wait(pred,timeout):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.01)
  if pred():return
 raise TimeoutError(str(status))
def call(service):
 assert status.get('run_id')==str(out)
 client=node.create_client(Trigger,'/sim_esdf_sage/'+service);assert client.wait_for_service(timeout_sec=5)
 future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
 assert future.result().success;node.destroy_client(client)
try:
 wait(lambda:status.get('run_id')==str(out) and not status.get('stopped'),15)
 assert json.loads((out/'moving_scenario.json').read_text())[-1]['kind']=='complete'
 scenario=json.loads((out/'moving_scenario.json').read_text());goal=scenario[0]['seed_tool']
 msg=PoseStamped();msg.header.frame_id='base_link';msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=goal;msg.pose.orientation.w=1.;pub.publish(msg)
 wait(lambda:status.get('phase')=='REACHING',30)
 start=time.monotonic();wait(lambda:time.monotonic()-start>1.,3)
 call('test_map_stale');injected=time.monotonic()
 wait(lambda:str(status.get('error','')).startswith('ONLINE_MAP_STALE'),2)
 wait(lambda:time.monotonic()-injected>1.5,3)
 held=np.array([q for t,q in commands if injected+.3<t<injected+1.5]);assert len(held)>50
 results['map_outage_hold_range_rad']=float(np.ptp(held,axis=0).max());assert results['map_outage_hold_range_rad']<1e-8
 wait(lambda:status.get('phase')=='GOAL_REACHED',180);results['map_recovery_reached']=True
 call('test_feedback_loss');wait(lambda:status.get('stopped'),5)
 start=time.monotonic();wait(lambda:time.monotonic()-start>1.,3)
 held=np.array([q for t,q in commands if t>start+.2]);results['feedback_loss_hold_range_rad']=float(np.ptp(held,axis=0).max());assert results['feedback_loss_hold_range_rad']<1e-8
 results['passed']=True
finally:
 (out/'faults_continuous.json').write_text(json.dumps(results,indent=2));print(results)
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
