"""Isolated-domain ROS + real CUDA fusion of synthetic RGB-D; never commands a robot."""
import json
import os
from pathlib import Path
import threading
import time
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState,PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from realtime_nvblox.ros_publishers import transform_stamped
from realtime_nvblox.nvblox_backend import NvbloxBackend
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_real_reconstruction.inputs import load_config
from ur7e_real_reconstruction.node import Reconstruction

assert os.environ.get('ROS_DOMAIN_ID')=='83','Run on isolated ROS_DOMAIN_ID=83'
root=Path(__file__).resolve().parents[3]
out=root/'log/real_reconstruction_20260928/mock';out.mkdir(parents=True,exist_ok=True)
c=load_config(root/'src/ur7e_real_reconstruction/config/real.yaml')
c.update(output_directory=str(out),settle_seconds=.15,voxel_size_m=.025)
for key in ['depth_topic','color_topic','camera_info_topic','color_info_topic','joint_topic']:c[key]='/mock/'+key
q0=np.array(json.loads((root/'src/ur7e_real_static/report/observations.json').read_text())[0]['q_rad'])


class Hardware(Node):
    def __init__(self,g):
        super().__init__('mock_rgbd');self.g=g;self.q=q0.copy();self.velocity=0.;self.bad_tf=False;self.empty=False
        self.color_offset=0.;self.tf=TransformBroadcaster(self);self.send=True;self.send_joints=True
        self.joints=self.create_publisher(JointState,c['joint_topic'],5)
        self.pubs={k:self.create_publisher(CameraInfo if 'info' in k else Image,c[k],5) for k in ['depth_topic','color_topic','camera_info_topic','color_info_topic']}
        self.create_timer(.02,self.state);self.create_timer(.067,self.images)
    def state(self):
        if not self.send or not self.send_joints:return
        now=self.get_clock().now();m=JointState();m.header.stamp=now.to_msg();m.name=list(JOINTS)
        m.position=self.q.tolist();m.velocity=[self.velocity]*6;self.joints.publish(m)
        poses=self.g.fk(self.q)
        if self.bad_tf:poses[c['depth_frame']][:3,3]+=[.04,0,0]
        self.tf.sendTransform([transform_stamped(poses[link],'base_link',link,now.nanoseconds) for link in [*self.g.links,c['depth_frame'],c['color_frame']]])
    def images(self):
        if not self.send:return
        now=self.get_clock().now();w,h=160,120;fx=fy=100.;cx=80.;cy=60.
        pose=self.g.fk(self.q)[c['depth_frame']];v,u=np.indices((h,w))
        rays=np.stack([(u-cx)/fx,(v-cy)/fy,np.ones_like(u)],-1)@pose[:3,:3].T
        d=np.divide(.15-pose[2,3],rays[...,2],out=np.zeros((h,w)),where=abs(rays[...,2])>1e-6)
        d[(d<.2)|(d>3)]=0
        if self.empty:d[:]=0
        for sensor in ['depth','color']:
            ci=CameraInfo();ci.header.stamp=now.to_msg();ci.header.frame_id=c[sensor+'_frame']
            if sensor=='color':ci.header.stamp.sec+=int(self.color_offset)
            ci.width=w;ci.height=h;ci.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];ci.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
            ci.r=np.eye(3).reshape(-1).tolist();ci.distortion_model='plumb_bob';ci.d=[0.]*5
            self.pubs['camera_info_topic' if sensor=='depth' else 'color_info_topic'].publish(ci)
            data=np.rint(d*1000).astype('<u2') if sensor=='depth' else np.full((h,w,3),[200,40,20],np.uint8)
            im=Image();im.header=ci.header;im.width=w;im.height=h;im.encoding='16UC1' if sensor=='depth' else 'rgb8'
            im.step=w*(2 if sensor=='depth' else 3);im.data=data.tobytes();self.pubs[sensor+'_topic'].publish(im)


def wait(predicate,timeout=30):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.03)
    raise AssertionError(f'Timeout: phase={app.phase} reason={app.wait_reason} error={app.error} frames={app.frames}')


def service(name):
    client=hw.create_client(Trigger,'/ur7e_real_reconstruction/'+name)
    assert client.wait_for_service(timeout_sec=5)
    f=client.call_async(Trigger.Request());wait(f.done)
    result=f.result();hw.destroy_client(client);assert result.success,result.message


rclpy.init();app=Reconstruction(c);hw=Hardware(app.geometry)
executor=SingleThreadedExecutor();executor.add_node(app);executor.add_node(hw)
t=threading.Thread(target=executor.spin,daemon=True);t.start();results={}
try:
    with patch.object(NvbloxBackend,'update_esdf',side_effect=AssertionError('ESDF must never run')):
        time.sleep(.5);assert app.frames==0;results['idle_no_fusion']=True
        service('start_scan');wait(lambda:app.frames>=5,90)
        assert app.surface is not None and len(app.surface.triangles)>100
        results['cuda_rgbd_surface']={'vertices':len(app.surface.vertices),'triangles':len(app.surface.triangles)}
        hw.velocity=.3;time.sleep(.8);n=app.frames;time.sleep(.5);assert app.frames==n
        hw.velocity=0.;wait(lambda:app.frames>n);results['movement_rejected']=True
        hw.bad_tf=True;time.sleep(.8);n=app.frames;time.sleep(.5);assert app.frames==n
        hw.bad_tf=False;wait(lambda:app.frames>n);results['wrong_tf_rejected']=True
        hw.empty=True;time.sleep(.8);n=app.frames;time.sleep(.5);assert app.frames==n
        hw.empty=False;wait(lambda:app.frames>n);results['empty_depth_rejected']=True
        hw.send_joints=False;time.sleep(.8);n=app.frames;time.sleep(.5);assert app.frames==n
        assert app.wait_reason=='WAIT_JOINT_FEEDBACK'
        hw.send_joints=True;wait(lambda:app.frames>n);results['joint_loss_blocks_and_recovers']=True
        hw.q=q0+np.array([.18,0,0,0,0,0]);wait(lambda:len(app.views)>=2)
        results['multi_view_images_saved']=len(app.views)
        service('freeze_scan');wait(lambda:app.phase=='FROZEN')
        n=app.frames;time.sleep(.5);assert app.frames==n
        results['freeze_stops_fusion']=True
        saved=app.session/'surface.npz'
        with np.load(saved,allow_pickle=False) as data:
            vertices=data['vertices'];colors=data['colors']
            error=np.abs(vertices[:,2]-.15)
            assert np.percentile(error,95)<.03
            red=(colors[:,0]>120)&(colors[:,0]>colors[:,1]*1.5)
            assert np.mean(red)>.6
            results['plane_error_p95_m']=float(np.percentile(error,95));results['red_vertices_fraction']=float(np.mean(red))
        clouds=[];qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub=hw.create_subscription(PointCloud2,'/ur7e_real_reconstruction/surface_points',clouds.append,qos)
        wait(lambda:bool(clouds));assert clouds[-1].width>0;hw.destroy_subscription(sub)
        results['late_subscriber_frozen_surface']=True
        first_session=app.session
        service('start_scan');wait(lambda:app.frames>=2 and app.session!=first_session)
        assert (first_session/'surface.npz').exists();results['restart_preserves_previous_export']=True
        service('freeze_scan');wait(lambda:app.phase=='FROZEN')
        executor.remove_node(app);app.close();app.destroy_node()
        app=Reconstruction(c,str(saved));executor.add_node(app)
        assert app.phase=='FROZEN' and len(app.surface.vertices)==len(vertices)
        results['offline_reload']=True
        results['esdf_never_called']=True;results['passed']=True
finally:
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2),flush=True)
    executor.shutdown();t.join();app.close();app.destroy_node();hw.destroy_node();rclpy.shutdown()
