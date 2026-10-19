"""Read-only RGB-D reconstruction. No action clients, goals or command publishers."""
import argparse
from collections import Counter, deque
from datetime import datetime
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

import message_filters
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.ros_publishers import pose_stamped
from ur7e_nvblox.depth import depth_meters, intrinsics_from_info, transform_matrix
from ur7e_reconstruction.color import rgb_image
from ur7e_reconstruction.surface import Surface, SurfaceMapper, surface_cloud, surface_markers, save_ply
from ur7e_real_static.model import Geometry
from ur7e_real_static.trajectory import model_matches
from ur7e_perception_planning_core.self_filter import robot_depth_mask
from .inputs import load_config, stamp, camera_record, signature, ColorRectifier, JointHistory


def write_json(path,value):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');temp.replace(path)


class Reconstruction(Node):
    def __init__(self,c,load=''):
        super().__init__('ur7e_real_reconstruction')
        self.c=c
        self.geometry=Geometry(c['robot_urdf'],c['collision_spheres'],camera=c['depth_frame'])
        self.lock=threading.RLock();self.stop=threading.Event();self.history=JointHistory()
        self.pending=deque(maxlen=8);self.commands=deque();self.info=None;self.color_info=None
        self.phase='IDLE';self.wait_reason='START_SCAN_REQUIRED';self.error=None;self.frames=0
        self.views=[];self.stats=Counter();self.session=None;self.surface=None;self.last_stamp=0.
        self.started=0.;self.last_input=0.;self.last_pose=np.eye(4);self.last_ns=0;self.latencies=deque(maxlen=512)
        self.results=LatestSlot();self.tf=Buffer(cache_time=Duration(seconds=10),node=self)
        self.listener=TransformListener(self.tf,self)
        qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mesh_pub=self.create_publisher(MarkerArray,'~/mesh',durable)
        self.cloud_pub=self.create_publisher(PointCloud2,'~/surface_points',durable)
        self.pose_pub=self.create_publisher(PoseStamped,'~/camera_pose',durable)
        self.status_pub=self.create_publisher(String,'~/status',durable)
        self.create_subscription(CameraInfo,c['camera_info_topic'],self.on_info,qos)
        self.create_subscription(CameraInfo,c['color_info_topic'],self.on_color_info,qos)
        self.create_subscription(JointState,c['joint_topic'],self.on_state,qos)
        self.depth_sub=message_filters.Subscriber(self,Image,c['depth_topic'],qos_profile=qos)
        self.color_sub=message_filters.Subscriber(self,Image,c['color_topic'],qos_profile=qos)
        self.sync=message_filters.ApproximateTimeSynchronizer([self.depth_sub,self.color_sub],8,c['sync_slop_sec'])
        self.sync.registerCallback(self.on_pair)
        self.create_service(Trigger,'~/start_scan',self.start_scan)
        self.create_service(Trigger,'~/freeze_scan',self.freeze_scan)
        self.create_timer(1.,self.report)
        if load:
            path=Path(load).expanduser().resolve()
            with np.load(path,allow_pickle=False) as data:
                self.surface=Surface(data['vertices'].copy(),data['triangles'].copy(),data['colors'].copy())
                record=json.loads(str(data['metadata']))
            if record['world_frame']!='base_link':raise ValueError('Snapshot must use base_link')
            self.frames=record['frames'];self.views=record['views'];self.session=path.parent
            self.phase='FROZEN';self.wait_reason='LOADED_PREVIEW'
            self.results.put((self.surface,self.get_clock().now().nanoseconds,np.eye(4)))
        self.worker=threading.Thread(target=self.mapping_loop,daemon=True,name='rgbd-fusion')
        self.publisher=threading.Thread(target=self.publish_loop,daemon=True,name='surface-publisher')
        self.worker.start();self.publisher.start()
        self.get_logger().info('Read-only RGB-D TSDF preview in base_link. Start with ~/start_scan. No ESDF or motion interfaces.')

    def on_info(self,msg):self.info=msg
    def on_color_info(self,msg):self.color_info=msg
    def on_state(self,msg):
        with self.lock:self.history.add(msg)

    def on_pair(self,depth,color):
        with self.lock:
            self.stats['rgbd_pairs_received']+=1
            if self.phase!='SCANNING':return
            td,tc=stamp(depth),stamp(color);now=self.get_clock().now().nanoseconds*1e-9
            if min(td,tc)<=self.started or td<=self.last_input or max(abs(now-td),abs(now-tc))>self.c['frame_timeout']:
                self.stats['stale_or_duplicate_input']+=1;return
            if self.info is None or self.color_info is None:self.wait_reason='WAIT_CAMERA_INFO';return
            self.last_input=td
            self.pending.append((depth,color,self.info,self.color_info,time.monotonic()))

    def start_scan(self,request,response):
        with self.lock:
            if self.phase not in ('IDLE','FROZEN','ERROR'):
                response.success=False;response.message=f'Cannot start from {self.phase}';return response
            self.phase='STARTING';self.commands.append('start')
        response.success=True;response.message='New scan requested; wait for SCANNING status. Earlier sessions stay on disk.'
        return response

    def freeze_scan(self,request,response):
        with self.lock:
            if self.phase!='SCANNING' or self.frames==0:
                response.success=False;response.message='Need a scanning session with fused frames';return response
            self.phase='FREEZING';self.pending.clear();self.commands.append('freeze')
        response.success=True;response.message='Freeze requested; final extraction/export completes when status is FROZEN.'
        return response

    def new_session(self):
        directory=Path(self.c['output_directory']).expanduser()/('scan_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        (directory/'observations').mkdir(parents=True,exist_ok=False)
        write_json(directory/'config.json',self.c)
        with self.lock:
            self.session=directory;self.frames=0;self.views=[];self.surface=None;self.pending.clear()
            self.started=self.get_clock().now().nanoseconds*1e-9;self.last_input=self.started
            self.error=None;self.phase='SCANNING';self.wait_reason='WAIT_RGBD';self.stats.clear();self.latencies.clear()
        empty=Surface(np.empty((0,3),np.float32),np.empty((0,3),np.int32),np.empty((0,3),np.uint8))
        self.results.put((empty,self.get_clock().now().nanoseconds,np.eye(4)))

    def metadata(self):
        return dict(world_frame='base_link',frames=self.frames,views=self.views,esdf_enabled=False,
                    motion_interfaces=False,config=self.c,
                    model_sha256={k:hashlib.sha256(Path(self.c[k]).read_bytes()).hexdigest() for k in ['robot_urdf','collision_spheres']})

    def export(self,surface):
        record=self.metadata()
        tmp=self.session/'surface.tmp.npz'
        np.savez_compressed(tmp,vertices=surface.vertices,triangles=surface.triangles,colors=surface.colors,
                            metadata=json.dumps(record,allow_nan=False))
        tmp.replace(self.session/'surface.npz')
        save_ply(surface,self.session/'surface.ply')
        write_json(self.session/'metadata.json',record)

    def record_view(self,item,q,depth_pose,color_pose):
        if self.views and min(np.max(abs(q-np.asarray(v['q_rad']))) for v in self.views)<self.c['view_separation_rad']:return
        depth,color,info,cinfo,_=item
        i=len(self.views)+1;folder=self.session/'observations'/f'{i:03d}';folder.mkdir()
        PILImage.fromarray(rgb_image(color)).save(folder/'rgb.png')
        # Preserve the complete ROS depth layout/encoding, and a full-size decoded metric image.
        np.savez_compressed(folder/'depth.npz',raw=np.frombuffer(bytes(depth.data),np.uint8),
                            meters=depth_meters(depth,1,self.c['min_depth_m'],self.c['max_depth_m']),
                            encoding=depth.encoding,step=depth.step,width=depth.width,height=depth.height,is_bigendian=depth.is_bigendian)
        view=dict(id=i,depth_stamp=stamp(depth),color_stamp=stamp(color),q_rad=q.tolist(),
                  T_base_depth=depth_pose.tolist(),T_base_color=color_pose.tolist(),
                  depth_info=camera_record(info),color_info=camera_record(cinfo),path=f'observations/{i:03d}')
        write_json(folder/'pose.json',view)
        with self.lock:self.views.append(view)
        write_json(self.session/'observations.json',self.views)

    def pick_ready(self):
        now=self.get_clock().now().nanoseconds*1e-9
        with self.lock:
            self.pending=deque([p for p in self.pending if max(abs(now-stamp(p[0])),abs(now-stamp(p[1])))<=self.c['frame_timeout']
                                and time.monotonic()-p[4]<=self.c['frame_timeout']],maxlen=8)
            items=list(self.pending)
        self.wait_reason='WAIT_RGBD'
        with self.lock:
            if not self.history.samples or now-self.history.samples[-1][0]>self.c['frame_timeout']:
                self.wait_reason='WAIT_JOINT_FEEDBACK'
                return None
        for item in reversed(items):
            depth,color,info,cinfo,_=item;td,tc=stamp(depth),stamp(color)
            if max(td,tc)>now:self.wait_reason='WAIT_IMAGE_CLOCK';continue
            with self.lock:
                q=self.history.stationary_at(td,self.c);cq=self.history.stationary_at(tc,self.c)
            if q is None or cq is None:self.wait_reason='WAIT_STATIONARY_JOINTS_AT_IMAGE_TIME';continue
            try:
                t=Time.from_msg(depth.header.stamp)
                poses={link:transform_matrix(self.tf.lookup_transform('base_link',link,t).transform)
                       for link in [*self.geometry.links,self.c['depth_frame']]}
                cp=transform_matrix(self.tf.lookup_transform('base_link',self.c['color_frame'],Time.from_msg(color.header.stamp)).transform)
            except TransformException:self.wait_reason='WAIT_TF_AT_IMAGE_TIME';continue
            if not model_matches(self.geometry,q,poses) or not model_matches(self.geometry,cq,{self.c['color_frame']:cp}):
                self.wait_reason='TF_MODEL_MISMATCH';self.stats['tf_model_mismatch']+=1;continue
            with self.lock:self.pending=deque([p for p in self.pending if stamp(p[0])>td],maxlen=8)
            return item,q,poses,cp
        return None

    def mapping_loop(self):
        mapper=None;calibration=None;rectifier=None;next_capture=next_mesh=0.
        while not self.stop.is_set():
            try:
                with self.lock:command=self.commands.popleft() if self.commands else None;phase=self.phase
                if command=='start':
                    mapper=None;calibration=None;rectifier=None;next_capture=next_mesh=0.
                    self.new_session();continue
                if command=='freeze':
                    surface=mapper.extract()  # Extract after the last in-flight integration, not an old display snapshot.
                    if not len(surface.triangles):raise ValueError('No surface triangles; cannot export a valid reconstruction')
                    self.export(surface)
                    with self.lock:self.surface=surface;self.phase='FROZEN';self.wait_reason='FROZEN_SAVED'
                    self.results.put((surface,self.last_ns,self.last_pose));continue
                if phase!='SCANNING' or time.monotonic()<next_capture:
                    self.stop.wait(.015);continue
                ready=self.pick_ready()
                if ready is None:self.stop.wait(.01);continue
                item,q,poses,color_pose=ready;depth,color,info,cinfo,received=item
                next_capture=time.monotonic()+1/self.c['capture_hz']
                for msg,ci,frame in [(depth,info,self.c['depth_frame']),(color,cinfo,self.c['color_frame'])]:
                    if msg.header.frame_id!=frame or ci.header.frame_id!=frame or (msg.width,msg.height)!=(ci.width,ci.height):
                        raise ValueError('Image/CameraInfo/frame mismatch')
                sig=(signature(info),signature(cinfo),depth.encoding,color.encoding)
                if calibration is not None and sig!=calibration:raise ValueError('Calibration/encoding changed; start a new scan')
                intr=intrinsics_from_info(info,self.c['depth_stride'])
                if mapper is None:
                    rectifier=ColorRectifier(cinfo,self.c['depth_stride'])
                    mapper=SurfaceMapper(intr,self.c,rectifier.intrinsics);calibration=sig
                d=depth_meters(depth,self.c['depth_stride'],self.c['min_depth_m'],self.c['max_depth_m'])
                pose=poses[self.c['depth_frame']];rgb=rectifier.decode(color)
                centers=np.empty_like(self.geometry.local)
                for i,link in enumerate(self.geometry.links):
                    ids=self.geometry.link_ids==i;p=poses[link]
                    centers[ids]=self.geometry.local[ids]@p[:3,:3].T+p[:3,3]
                mask=robot_depth_mask(d,pose,intr,centers,self.geometry.radii,self.geometry.link_ids)
                self.stats['self_filtered_pixels']+=int(np.count_nonzero(mask & (d>0)));d[mask]=0.
                if not np.any(d):self.stats['empty_depth']+=1;continue
                now=self.get_clock().now().nanoseconds*1e-9
                if max(now-stamp(depth),now-stamp(color),time.monotonic()-received)>self.c['frame_timeout']:
                    self.stats['stale_before_fusion']+=1;continue
                with self.lock:
                    if self.phase!='SCANNING':continue
                mapper.integrate(d,pose,rgb,color_pose)
                with self.lock:
                    self.frames+=1;self.last_stamp=stamp(depth);self.last_ns=depth.header.stamp.sec*10**9+depth.header.stamp.nanosec
                    self.last_pose=pose;self.wait_reason='FUSING';self.latencies.append((time.monotonic()-received)*1000)
                # Late completed fusion is retained (static scene), with honest latency counters.
                if time.monotonic()-received>self.c['frame_timeout']:self.stats['slow_completed_fusion']+=1
                self.record_view(item,q,pose,color_pose)
                if time.monotonic()>=next_mesh:
                    surface=mapper.extract()
                    with self.lock:self.surface=surface
                    self.results.put((surface,self.last_ns,pose));next_mesh=time.monotonic()+1/self.c['mesh_hz']
            except Exception as exc:
                with self.lock:self.error=repr(exc);self.phase='ERROR';self.wait_reason='ERROR';self.pending.clear()
                self.get_logger().error(self.error)
                self.stop.wait(.1)

    def publish_loop(self):
        seq=0
        while not self.stop.is_set():
            seq,result=self.results.wait_newer(seq,.2)
            if result is None:continue
            surface,ns,pose=result
            try:
                self.mesh_pub.publish(surface_markers(surface,'base_link',ns))
                self.cloud_pub.publish(surface_cloud(surface,'base_link',ns))
                self.pose_pub.publish(pose_stamped(pose,'base_link',ns))
            except Exception as exc:
                if not self.stop.is_set():self.get_logger().error(f'Visualization: {exc}')

    def report(self):
        with self.lock:
            s=dict(phase=self.phase,wait_reason=self.wait_reason,error=self.error,frames=self.frames,views=len(self.views),
                   vertices=0 if self.surface is None else len(self.surface.vertices),
                   triangles=0 if self.surface is None else len(self.surface.triangles),
                   world_frame='base_link',esdf_enabled=False,motion_interfaces=False,
                   session=None if self.session is None else str(self.session),counts=dict(self.stats),
                   joint_samples=len(self.history.samples),
                   joint_age_sec=None if not self.history.samples else self.get_clock().now().nanoseconds*1e-9-self.history.samples[-1][0],
                   input_to_fusion_ms=None if not self.latencies else dict(zip(['p50','p95'],np.percentile(self.latencies,[50,95]).tolist())))
        self.status_pub.publish(String(data=json.dumps(s,allow_nan=False)))
        if self.session:write_json(self.session/'status.json',s)

    def close(self):
        self.stop.set();self.results.close();self.worker.join();self.publisher.join()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);parser.add_argument('--load',default='')
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=None
    try:
        node=Reconstruction(load_config(args.config),args.load);rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
