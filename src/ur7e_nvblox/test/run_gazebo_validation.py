"""Measure actual Gazebo depth + joint-state TF, with optional simulated motion.

Start the scene in an isolated ROS domain first. This harness runs one mapper;
it never connects to a hardware driver. --exercise-simulation requires domain 127.
"""
import argparse
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException
from std_srvs.srv import Trigger

from ur7e_nvblox.depth import transform_matrix
from ur7e_nvblox.node import TfMapper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['dense', 'native', 'sparse'], default='native')
    parser.add_argument('--voxel-size', type=float, default=.05)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--esdf-hz', type=float, default=30.)
    parser.add_argument('--publish-hz', type=float, default=30.)
    parser.add_argument('--exercise-simulation', action='store_true')
    parser.add_argument('--external', action='store_true', help='Observe an already running mapper in its own process')
    parser.add_argument('--cloud-topic', help='Defaults to native v3 distance band for --external, legacy observed topic otherwise')
    parser.add_argument('--common-input', action='store_true', help='Use calibrated common 640x360 depth, no second decimation')
    parser.add_argument('--lifecycle', action='store_true', help='Verify no-consumer query suppression and reset')
    args = parser.parse_args()
    if args.exercise_simulation and os.environ.get('ROS_DOMAIN_ID') != '127':
        raise RuntimeError('Simulated motion is restricted to the dedicated test ROS_DOMAIN_ID=127')
    ros_args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'voxel_size_m:='+str(args.voxel_size),
                     '-p', 'esdf_output_mode:='+args.mode, '-p', 'esdf_hz:='+str(args.esdf_hz),
                     '-p', 'publish_hz:='+str(args.publish_hz), '-p', 'max_visualized_distance_m:=0.15']
    if args.common_input:
        ros_args += ['-p','depth_topic:=/esdf_input/depth','-p','camera_info_topic:=/esdf_input/camera_info','-p','depth_stride:=1']
    rclpy.init(args=ros_args)
    mapper, observer = (None if args.external else TfMapper()), Node('esdf_gazebo_validation')
    best = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    buffer = Buffer(node=observer)
    listener = TransformListener(buffer, observer)
    state = dict(stats=[], clouds=[], depth=[], poses=[], joints=None, tf_errors=[], invalid_clouds=[], xyz=None, distances=None, input_seen={}, arrival_latency=[], frames=[])
    def on_cloud(msg):
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        fields = {f.name: f.offset for f in msg.fields}
        if msg.width == 0:
            state['xyz']=np.empty((0,3),np.float32);state['distances']=np.empty(0,np.float32)
            state['clouds'].append((time.monotonic(),ns,0));return
        xyz = np.column_stack([np.ndarray((msg.width,), '<f4', buffer=msg.data,
                              offset=fields[k], strides=(msg.point_step,)) for k in ('x','y','z')])
        d = np.ndarray((msg.width,), '<f4', buffer=msg.data, offset=fields['distance'], strides=(msg.point_step,))
        if msg.header.frame_id != 'base_link' or not np.isfinite(xyz).all() or not np.isfinite(d).all() or (abs(d)>.15001).any():
            state['invalid_clouds'].append(ns)
        if args.mode == 'native' and len(xyz):
            error = np.max(abs(xyz / args.voxel_size - .5 - np.round(xyz / args.voxel_size - .5)))
            if error > 1e-4:
                state['invalid_clouds'].append(ns)
        state['xyz'] = xyz
        state['distances'] = d.copy()
        if not state['frames'] or time.monotonic()-state['frames'][-1][0]>.5:
            state['frames'].append((time.monotonic(),ns,xyz.copy(),d.copy()))
            del state['frames'][:-24]
        arrival=time.monotonic()
        state['clouds'].append((arrival, ns, msg.width))
        if ns in state['input_seen']:
            state['arrival_latency'].append((arrival,(arrival-state['input_seen'][ns])*1000))
    cloud_topic = args.cloud_topic or ('/ur7e_nvblox/esdf_distance_band' if args.external else '/ur7e_nvblox/esdf_observed')
    sub = observer.create_subscription(PointCloud2, cloud_topic, on_cloud, best)
    observer.create_subscription(String, '/ur7e_nvblox/stats', lambda m: state['stats'].append((time.monotonic(),json.loads(m.data))), 2)
    def on_depth(msg):
        arrival=time.monotonic();ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        state['depth'].append((arrival,ns));state['input_seen'][ns]=arrival
        if len(state['input_seen'])>512:state['input_seen'].pop(next(iter(state['input_seen'])))
    observer.create_subscription(Image, '/esdf_input/depth' if args.common_input else '/camera/depth/image_rect_raw', on_depth, best)
    observer.create_subscription(PoseStamped, '/ur7e_nvblox/camera_pose', lambda m: state['poses'].append(m), best)
    observer.create_subscription(JointState, '/joint_states', lambda m: state.update(joints=dict(zip(m.name,m.position))), best)
    command = observer.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(observer)
    if mapper is not None: executor.add_node(mapper)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if mapper is not None and mapper._error:
                raise RuntimeError(mapper._error)
            time.sleep(.05)
    reset_client = observer.create_client(Trigger, '/ur7e_nvblox/reset_map')
    def reset_map():
        if mapper is not None:
            mapper._request_reset();return
        if not reset_client.wait_for_service(timeout_sec=5):raise RuntimeError('Native reset service missing')
        future=reset_client.call_async(Trigger.Request())
        deadline=time.monotonic()+10
        while not future.done() and time.monotonic()<deadline:wait(.05)
        if not future.done() or not future.result().success:raise RuntimeError('Native reset failed')
    def poses_check():
        for msg in state['poses']:
            try:
                tf = buffer.lookup_transform('base_link','camera_depth_optical_frame',Time.from_msg(msg.header.stamp))
            except TransformException:
                continue
            expected = transform_matrix(tf.transform)
            p,q = msg.pose.position,msg.pose.orientation
            from scipy.spatial.transform import Rotation
            actual = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            state['tf_errors'].append(max(float(np.max(abs(expected[:3,3]-[p.x,p.y,p.z]))),float(np.max(abs(expected[:3,:3]-actual)))))
        state['poses'].clear()
    names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    report=dict(backend='official_ros_cpp' if args.external else 'python_torch',common_input=args.common_input,mode=args.mode,voxel_size_m=args.voxel_size,targets_hz=dict(esdf=args.esdf_hz,publish=args.publish_hz),ros_domain=os.environ.get('ROS_DOMAIN_ID'))
    try:
        deadline=time.monotonic()+60
        while (not state['stats'] or state['joints'] is None) and time.monotonic()<deadline:
            wait(.1)
        assert state['stats'] and state['joints'] is not None,'No Gazebo/mapper input'
        report['backend']=state['stats'][-1][1].get('backend','python_torch')
        if args.exercise_simulation:
            original=np.array([state['joints'][n] for n in names])
            view=np.array([0.,-1.57,1.1,-.90879335,-3.21580312,-.69520875])
            for u in np.linspace(0,1,90):
                command.publish(Float64MultiArray(data=((1-u)*original+u*view).tolist()));wait(1/30)
            wait(2)
            reset_map()
        deadline=time.monotonic()+45
        while (len(state['clouds'])<5 or not state['clouds'][-1][2]) and time.monotonic()<deadline:
            wait(.1)
        assert len(state['clouds'])>=5 and state['clouds'][-1][2]>0,'No nonempty observed ESDF'
        wait(2)
        for phase in (['stationary','moving'] if args.exercise_simulation else ['stationary']):
            start=time.monotonic(); before=state['stats'][-1]
            initial=np.array([state['joints'][n] for n in names])
            while time.monotonic()-start < args.seconds:
                if phase=='moving':
                    target=initial.copy();target[3]+=.06*np.sin((time.monotonic()-start)*1.5)
                    command.publish(Float64MultiArray(data=target.tolist()))
                poses_check();wait(.05)
            after=state['stats'][-1];dt=after[0]-before[0]
            samples=[s for s in state['clouds'] if s[0]>=start]
            depth=[s for s in state['depth'] if s[0]>=start]
            assert len(samples)>=3,'Insufficient ESDF output'
            stamps=[s[1] for s in samples]
            assert all(b>a for a,b in zip(stamps,stamps[1:])), 'Repeated or stale map timestamps'
            latency=[v for at,v in state['arrival_latency'] if at>=start]
            assert len(latency)>len(samples)*.8, 'Insufficient input/output timestamp pairs'
            assert abs(after[1].get('map_latest', after[1].get('latest', {}))['voxel_size_m']-args.voxel_size)<1e-6, 'Voxel size differs'
            rates={k:(after[1]['totals'].get(k,0)-before[1]['totals'].get(k,0))/dt
                   for k in ('depth_received','input_queue_seen','depth_integrated','esdf_updated','esdf_exported','esdf_published','observed_cloud_published')}
            report[phase]=dict(wall_seconds=dt,rates_hz=rates,
                received_cloud_hz=(len(samples)-1)/(samples[-1][0]-samples[0][0]),
                unique_stamp_count=len(set(stamps)),cloud_count=len(samples),
                simulated_cloud_hz=(len(samples)-1)*1e9/(stamps[-1]-stamps[0]),
                depth_received_hz=(len(depth)-1)/(depth[-1][0]-depth[0][0]) if len(depth)>1 else 0,
                points_min=min(s[2] for s in samples),points_max=max(s[2] for s in samples),
                observed_input_to_cloud_ms={f'p{p}':float(np.percentile(latency,p)) for p in (50,95,99)},
                latency_pairs=len(latency),
                latency=after[1]['latency'],latest=after[1].get('map_latest', after[1].get('latest', {})))
            np.savez_compressed(args.output.with_name(args.output.stem+'_'+phase+'.npz'),xyz=state['xyz'],distance=state['distances'])
            if phase=='moving':
                command.publish(Float64MultiArray(data=initial.tolist()))
        poses_check()
        assert state['tf_errors'] and max(state['tf_errors'])<1e-5, state['tf_errors'][:5]
        assert not state['invalid_clouds'],state['invalid_clouds']
        report['tf_pose_checks']=len(state['tf_errors']);report['max_tf_matrix_error']=max(state['tf_errors'])
        if args.lifecycle:
            observer.destroy_subscription(sub);sub=None;wait(1.5)
            before=state['stats'][-1][1]['totals']
            # A slow/dropping sensor can leave a 3 s window with no input at all.
            deadline = time.monotonic()+20
            while time.monotonic()<deadline:
                wait(.2)
                current=state['stats'][-1][1]['totals']
                if current.get('esdf_updated',0)-before.get('esdf_updated',0)>=3:
                    break
            after=state['stats'][-1][1]['totals']
            report['no_subscriber']=dict(esdf_updates=after.get('esdf_updated',0)-before.get('esdf_updated',0),
                queries=after.get('esdf_exported',0)-before.get('esdf_exported',0))
            assert report['no_subscriber']['queries']==0 and report['no_subscriber']['esdf_updates']>0
            sub=observer.create_subscription(PointCloud2,cloud_topic,on_cloud,best)
            reset_map();wait(4)
            report['reset_epoch']=state['stats'][-1][1]['map_latest']['map_epoch']
            assert report['reset_epoch']>0 and state['clouds'][-1][0]>time.monotonic()-2
        if state['frames']:
            arrays={}
            for i,(at,ns,xyz,d) in enumerate(state['frames']):
                arrays[f'xyz_{i}']=xyz;arrays[f'distance_{i}']=d;arrays[f'stamp_{i}']=np.array(ns)
            np.savez_compressed(args.output.with_name(args.output.stem+'_frames.npz'),**arrays)
        report['result']='PASS'
        if state['xyz'] is not None:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(args.output.with_suffix('.npz'),xyz=state['xyz'],distance=state['distances'])
    except Exception as exc:
        report['result']='FAIL';report['error']=repr(exc)
        raise
    finally:
        report['last_stats']=state['stats'][-1][1] if state['stats'] else {}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        if mapper is not None: mapper.close()
        executor.shutdown(timeout_sec=5);thread.join(timeout=3)
        if mapper is not None: mapper.destroy_node()
        observer.destroy_node();rclpy.shutdown()
        print(json.dumps({k:v for k,v in report.items() if k not in ('last_stats','stationary','moving')},indent=2))


if __name__=='__main__':
    main()
