"""Synthetic scene on domain 84 only. No device/RTDE access."""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState,Image,CameraInfo
from tf2_ros import TransformBroadcaster
from realtime_nvblox.ros_publishers import transform_stamped
from ur7e_auto_esdf.trajectory import load_inputs
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='84'
root=Path(__file__).resolve().parents[1]
c,_,_,g=load_inputs(*(root/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))

class Scene(Node):
    def __init__(self):
        super().__init__('mock_camera');self.q=None
        self.tf=TransformBroadcaster(self)
        self.create_subscription(JointState,'/ur7e_auto_esdf/joint_states',self.state,10)
        self.pubs={k:self.create_publisher(CameraInfo if 'info' in k else Image,c[k],5)
                   for k in ['depth_topic','camera_info_topic']}
        self.create_timer(.067,self.images)
    def state(self,msg):
        self.q=np.asarray(msg.position)
        p=g.fk(self.q);ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.tf.sendTransform([transform_stamped(p[link],'base_link',link,ns)
                               for link in [*g.links,c['camera_frame']]])
    def images(self):
        if self.q is None:return
        now=self.get_clock().now();w,h=160,120;fx=fy=100.;cx=80.;cy=60.
        pose=g.fk(self.q)[c['camera_frame']];v,u=np.indices((h,w))
        rays=np.stack([(u-cx)/fx,(v-cy)/fy,np.ones_like(u)],-1)@pose[:3,:3].T
        d=np.divide(.15-pose[2,3],rays[...,2],out=np.zeros((h,w)),where=abs(rays[...,2])>1e-6)
        d[(d<.2)|(d>3)]=0
        for sensor in ['depth']:
            ci=CameraInfo();ci.header.stamp=now.to_msg();ci.header.frame_id=c['camera_frame']
            ci.width=w;ci.height=h;ci.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];ci.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
            ci.r=np.eye(3).reshape(-1).tolist();ci.distortion_model='plumb_bob';ci.d=[0.]*5
            self.pubs['camera_info_topic' if sensor=='depth' else 'color_info_topic'].publish(ci)
            data=np.rint(d*1000).astype('<u2') if sensor=='depth' else np.full((h,w,3),[200,40,20],np.uint8)
            im=Image();im.header=ci.header;im.width=w;im.height=h;im.encoding='16UC1' if sensor=='depth' else 'rgb8'
            im.step=w*(2 if sensor=='depth' else 3);im.data=data.tobytes();self.pubs[sensor+'_topic'].publish(im)

rclpy.init();node=Scene()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
