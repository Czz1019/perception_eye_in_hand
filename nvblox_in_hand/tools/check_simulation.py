#!/usr/bin/env python3
"""Read-only simulation state audit; creates no joint-command publisher."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
LINKS=['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','tool0','camera_depth_optical_frame','camera_color_optical_frame']


def valid_positions(msg):
    values=dict(zip(msg.name,msg.position))
    return values if all(j in values and math.isfinite(values[j]) for j in JOINTS) else None


def pose_residual(expected, translation, quaternion):
    actual=np.eye(4);actual[:3,3]=translation
    actual[:3,:3]=Rotation.from_quat(quaternion).as_matrix()
    return dict(translation_error_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
                rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).as_rotvec())))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=float,default=20.)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'artifacts/robot_state_check.json')
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or not 1<=args.timeout<=300:parser.error('timeout must be 1..300')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0,str(Path(get_package_prefix('ur7e_d435_gazebo'))/'lib/ur7e_d435_gazebo'))
    from mount_geometry import link_transforms
    rclpy.init(args=[]);node=rclpy.create_node('in_hand_robot_state_audit')
    buffer=Buffer();listener=TransformListener(buffer,node)
    state=dict(joints=None,q=None,q_wall=0.,description='',clock=-1,ticks=0,clock_wall=0.)
    def joints(msg):
        q=valid_positions(msg)
        if q is not None:state.update(joints=msg,q=q,q_wall=time.monotonic())
    def clock(msg):
        stamp=msg.clock.sec*10**9+msg.clock.nanosec
        if stamp>state['clock']:
            state.update(ticks=state['ticks']+1,clock_wall=time.monotonic())
        state['clock']=stamp
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    node.create_subscription(String,'/robot_description',lambda m:state.update(description=m.data),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    report=dict(result='failed',motion_commands_sent=0,environment={k:os.environ.get(k,'') for k in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')},issues=[])
    def spin(seconds):
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    def missing():
        return [name for name,ok in {
            '/joint_states (six fresh joints)':state['q'] is not None and time.monotonic()-state['q_wall']<1.,
            '/clock (advancing)':state['ticks']>=2 and time.monotonic()-state['clock_wall']<1.,
            '/robot_description':bool(state['description'])}.items() if not ok]
    try:
        deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            spin(.1)
            if not missing():break
        if missing():raise RuntimeError('Missing or stale: '+', '.join(missing()))
        frozen=state['joints'];q=valid_positions(frozen)
        if frozen.header.stamp.sec==0 and frozen.header.stamp.nanosec==0:raise RuntimeError('Zero JointState timestamp')
        # Let RSP publish transforms bracketing the frozen joint stamp, instead of falling back to latest TF.
        spin(.6)
        report['joint_positions_rad']={j:q[j] for j in JOINTS}
        report['joint_stamp_ns']=Time.from_msg(frozen.header.stamp).nanoseconds
        report['publishers']={}
        for topic in ('/joint_states','/robot_description','/tf','/tf_static'):
            pubs=node.get_publishers_info_by_topic(topic)
            report['publishers'][topic]=[dict(node=p.node_namespace.rstrip('/')+'/'+p.node_name,gid=list(p.endpoint_gid)) for p in pubs]
            if topic in ('/joint_states','/robot_description') and len(pubs)!=1:
                report['issues'].append(f'{topic}: expected one publisher, found {len(pubs)}')
        poses=link_transforms(ET.fromstring(state['description']),q)
        root_to_base_inv=np.linalg.inv(poses['base_link'])
        report['tf_vs_joint_fk']={}
        for link in LINKS:
            try:
                tf=buffer.lookup_transform('base_link',link,Time.from_msg(frozen.header.stamp))
                t=tf.transform.translation;r=tf.transform.rotation
                residual=pose_residual(root_to_base_inv@poses[link],[t.x,t.y,t.z],[r.x,r.y,r.z,r.w])
                report['tf_vs_joint_fk'][link]=residual
                if residual['translation_error_m']>.002 or residual['rotation_error_rad']>.01:
                    report['issues'].append(f'{link}: TF differs from same-time joint FK')
            except (TransformException,KeyError) as error:
                report['issues'].append(link+': '+str(error))
        report['issues']+=missing()
        report['result']='passed' if not report['issues'] else 'failed'
        report['scope']='Joint/TF consistency and duplicate state sources; not Gazebo link truth or RViz pixel validation'
    except Exception as error:report['issues'].append(str(error))
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        node.destroy_node();rclpy.try_shutdown()
    return 0 if report['result']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
