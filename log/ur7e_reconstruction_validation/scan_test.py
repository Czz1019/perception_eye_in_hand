import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Empty, Trigger

rclpy.init(); node = rclpy.create_node('reconstruction_scan_test')
state={'joints':None,'stats':None,'cloud':None}
names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
node.create_subscription(JointState,'/joint_states',lambda m:state.update(joints=dict(zip(m.name,m.position))),qos_profile_sensor_data)
node.create_subscription(String,'/ur7e_reconstruction/stats',lambda m:state.update(stats=json.loads(m.data)),2)
node.create_subscription(PointCloud2,'/ur7e_reconstruction/surface_points',lambda m:state.update(cloud=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
gazebo=node.create_client(Empty,'/pause_physics')
assert gazebo.wait_for_service(timeout_sec=5),'This test requires Gazebo'
pub=node.create_publisher(Float64MultiArray,'/forward_position_controller/commands',1)
save=node.create_client(Trigger,'/ur7e_reconstruction/save_mesh')
def wait_for(predicate,timeout=30):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate():return
    raise RuntimeError('Condition timed out')
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
def ramp(a,b):
    start=time.monotonic()
    while True:
        u=min(1.,(time.monotonic()-start)/8.)
        # Smooth position interpolation; no sudden jumps in simulation commands.
        u=u*u*(3-2*u)
        pub.publish(Float64MultiArray(data=((1-u)*a+u*b).tolist()))
        spin(.05)
        if u>=1:break
def metrics():
    m=state['cloud']; p=np.frombuffer(m.data,dtype='<f4').reshape(-1,4)[:,:3]
    return {'vertices':len(p),'above_ground_vertices':int(np.count_nonzero(p[:,2]>.15)),
            'z_max':float(p[:,2].max()),'triangles':state['stats']['latest']['triangles']}
wait_for(lambda:state['joints'] and state['stats'] and state['cloud'] is not None and state['cloud'].width>100)
initial=np.array([state['joints'][j] for j in names]);before=metrics()
target=initial.copy();target[3]+=.6;target[4]-=.6
try:
    ramp(initial,target);spin(10)
    actual=np.array([state['joints'][j] for j in names])
    assert np.max(np.abs(actual-target))<.02,(actual,target)
    after=metrics()
    assert after['above_ground_vertices']>before['above_ground_vertices']+100,(before,after)
    future=save.call_async(Trigger.Request());wait_for(future.done)
    assert future.result().success,future.result().message
    report={'result':'PASS','initial_joints':initial.tolist(),'target_joints':target.tolist(),
            'before':before,'after_scan':after,'saved_mesh':future.result().message}
finally:
    current=np.array([state['joints'][j] for j in names])
    ramp(current,initial);spin(5)
restored=np.array([state['joints'][j] for j in names])
assert np.max(np.abs(restored-initial))<.02
report['restored_joint_max_error_rad']=float(np.max(np.abs(restored-initial)))
report['after_restore']=metrics()
Path('/tmp/ur7e_reconstruction_validation/scan_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
