import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger, Empty

rclpy.init()
node = rclpy.create_node('exercise_sim_esdf', parameter_overrides=[Parameter('use_sim_time', value=True)])
state = {'joints': None, 'poses': [], 'stats': None}
names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
def joints(m): state['joints'] = dict(zip(m.name, m.position))
def pose(m): state['poses'].append([m.pose.position.x,m.pose.position.y,m.pose.position.z])
node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
node.create_subscription(PoseStamped,'/ur7e_nvblox/camera_pose',pose,qos_profile_sensor_data)
node.create_subscription(String,'/ur7e_nvblox/stats',lambda m: state.update(stats=json.loads(m.data)),2)
pub=node.create_publisher(Float64MultiArray,'/forward_position_controller/commands',1)
reset=node.create_client(Trigger,'/ur7e_nvblox/reset_map')
reset_sim=node.create_client(Empty,'/reset_simulation')
assert reset_sim.wait_for_service(timeout_sec=5), 'This exercise requires Gazebo simulation'
def spin_for(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.05)
def wait_for(predicate,timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
        if predicate(): return
    raise AssertionError('Timed out waiting for condition')
wait_for(lambda: state['joints'] is not None and len(state['poses'])>2 and state['stats'] is not None)
initial=np.array([state['joints'][j] for j in names]); p0=np.array(state['poses'][-1])
report={'initial_joints':initial.tolist(), 'initial_camera_position':p0.tolist()}
def ramp(a,b,seconds=4):
    start=time.monotonic()
    while True:
        alpha=min(1., (time.monotonic()-start)/seconds)
        pub.publish(Float64MultiArray(data=((1-alpha)*a+alpha*b).tolist()))
        spin_for(.05)
        if alpha>=1: break
try:
    target=initial.copy();target[0]+=.08
    ramp(initial,target);spin_for(3)
    actual=np.array([state['joints'][j] for j in names]); p1=np.array(state['poses'][-1])
    assert abs(actual[0]-target[0])<.01,(actual,target)
    assert np.linalg.norm(p1-p0)>.02,(p0,p1)
    report.update(moved_joints=actual.tolist(), moved_camera_position=p1.tolist(),camera_translation_m=float(np.linalg.norm(p1-p0)))
finally:
    current=np.array([state['joints'][j] for j in names])
    ramp(current,initial);spin_for(3)
restored=np.array([state['joints'][j] for j in names])
assert np.max(np.abs(restored-initial))<.01,restored
report['restored_joint_max_error_rad']=float(np.max(np.abs(restored-initial)))
before=state['stats']['latest']['map_epoch']
future=reset.call_async(Trigger.Request());wait_for(future.done)
assert future.result().success
wait_for(lambda:state['stats']['latest'].get('map_epoch',-1)>before and state['stats']['latest'].get('known_cells',0)>100)
report['reset_service_epoch']=state['stats']['latest']['map_epoch']
# Full Gazebo reset also affects upstream ros2_control; test mapper reset alone.
report['final_stats']=state['stats']
report['result']='PASS'
Path('/tmp/ur7e_esdf_validation/motion_reset_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
