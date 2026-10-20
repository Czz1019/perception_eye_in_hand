# Adapted from ur7e_auto_esdf: simulation clock and depth-derived tabletop map.
"""Scan-only ESDF mapper: no target subscription, Action client or robot control."""
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import threading
import time
import uuid
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import Image,CameraInfo,JointState,PointCloud2
from std_msgs.msg import Empty,String
from std_srvs.srv import Trigger,SetBool
from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.stats import RuntimeStats
from .config import load_mapping, identity
from ur7e_real_static.model import Geometry
from ur7e_real_static.node import RealStatic
from ur7e_real_static.field import FrozenMap
from .field import FieldBuilder, PLANNING_POLICY
from ur7e_real_reconstruction.node import write_json
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,depth_calibration,known_distances
from ur7e_nvblox.esdf import EfficientBackend


class EsdfMapper(Node):
    # Reuse input interpolation and latest-only visualization, not planning/execution.
    def on_state(self,msg):
        # Gazebo SetPosition reports intra-physics-step gravity velocity even when
        # successive measured positions are identical. Use timestamped measured
        # position differences for the simulated servo/settling velocity.
        try:
            ids=[msg.name.index(j) for j in JOINTS]
            q=np.array([msg.position[i] for i in ids])
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp<=0 or not np.isfinite(q).all():return
            with self.lock:
                if self.states and stamp<=self.states[-1][2]:return
                dq=(q-self.states[-1][0])/(stamp-self.states[-1][2]) if self.states else np.zeros(6)
                self.states.append((q,dq,stamp,time.monotonic()))
        except (ValueError,IndexError):return
    state_at=RealStatic.state_at
    settled_at=RealStatic.settled_at
    info_record=staticmethod(RealStatic.info_record)
    clear_visuals=RealStatic.clear_visuals

    def __init__(self,c,load=''):
        super().__init__('sim_esdf_mapper', parameter_overrides=[Parameter('use_sim_time', value=True)])
        if c['execution_enabled'] or not c['use_sim_time']:
            raise ValueError('Simulation mapper requires sim time and no hardware execution')
        self.c=c;self.model_id=identity(c)
        self.extra_mask=yaml.safe_load(Path(c['scan_config']).read_text())['depth_self_mask']
        self.geometry=Geometry(c['robot_urdf'],c['collision_spheres'],c['tool_frame'],c['camera_frame'])
        self.lock=threading.RLock();self.stop_event=threading.Event()
        self.frames=LatestSlot();self.visuals=LatestSlot();self.states=deque(maxlen=2000)
        self.capture_stats=RuntimeStats();self.capture_metrics={};self.capture_wait_reason='IDLE';self.capture_debug={}
        self.last_depth_stamp=0.;self.scan_started=0.;self.last_input=0.;self.input_depth_age=None
        self.phase='IDLE';self.capture_error=None;self.scanning=False;self.epoch=0
        self.views=[];self.observations=[];self.capture_count=0;self.latest_field=None
        self.online=None;self.camera_info=None;self.camera_signature=None;self.frozen=None;self.session=None
        self.gate=False;self.capture_epoch=0;self.gate_frames=0;self.gate_cutoff=0.;self.gate_q=None;self.lease=0.
        self.warmup_state='COLD';self.warmup_error=None;self.warmup_thread=None;self.warmup_signature=None
        self.raw_surface=None;self.last_report_write=0.;self.tabletop_evidence=None
        self.tf_buffer=Buffer(cache_time=Duration(seconds=10),node=self);self.tf_listener=TransformListener(self.tf_buffer,self)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub=self.create_publisher(PointCloud2,'~/esdf',latched)
        self.surface_pub=self.create_publisher(PointCloud2,'~/surface_esdf',latched)
        self.status_pub=self.create_publisher(String,'~/capture_status',10)
        self.state_group=MutuallyExclusiveCallbackGroup();self.report_group=MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState,c['joint_topic'],self.on_state,qos,callback_group=self.state_group)
        self.create_subscription(CameraInfo,c['camera_info_topic'],self.on_info,qos)
        # A depth image supersedes older images. Ten queued images at 10 Hz
        # can consume the entire freshness budget before fusion even begins.
        image_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image,c['depth_topic'],self.on_depth,image_qos)
        self.create_subscription(Empty,'/sim_esdf_sage/capture_lease',self.on_lease,5)
        # Only the closed-loop coordinator owns scan/capture/freeze transitions.
        self.create_timer(.1,self.report,callback_group=self.report_group)
        if load:self.load(load)
        from .capture import worker
        from ur7e_real_static.visualization import worker as visual_worker
        self.capture_thread=threading.Thread(target=worker,args=(self,),daemon=True)
        self.visual_thread=threading.Thread(target=visual_worker,args=(self,),daemon=True)
        self.capture_thread.start();self.visual_thread.start()

    def service(self,fn):
        def callback(req,res):
            try:
                with self.lock:res.message=fn()
                res.success=True
            except Exception as e:res.success=False;res.message=str(e)
            return res
        return callback

    def on_info(self,msg):
        with self.lock:self.camera_info=msg

    def on_depth(self,msg):
        with self.lock:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            self.input_depth_age=now-stamp
            if stamp>0 and abs(now-stamp)<self.c['frame_timeout']:self.last_input=time.monotonic()
            if self.online is not None:self.online.submit(msg,self.camera_info,stamp)
            if self.capture_allowed(stamp):RealStatic.on_depth(self,msg)

    def on_lease(self,msg):self.lease=time.monotonic()

    def capture_allowed(self,stamp=None,q=None):
        return (self.gate and time.monotonic()-self.lease<.6
                and (stamp is None or stamp>self.gate_cutoff)
                and (q is None or self.gate_q is not None and np.max(abs(q-self.gate_q))<.008))

    def start_scan(self):
        if self.scanning:raise ValueError('Already scanning')
        self.session=Path(self.c['output_directory']).expanduser()/('scan_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.session.mkdir(parents=True)
        (self.session/'mapping.yaml').write_text(yaml.safe_dump(self.c,sort_keys=False))
        self.epoch+=1;self.gate=False;self.gate_frames=0
        self.capture_count=0;self.views=[];self.observations=[];self.latest_field=None;self.raw_surface=None
        self.frozen=None;self.camera_signature=None;self.last_depth_stamp=0.;self.tabletop_evidence=None
        self.scan_started=self.get_clock().now().nanoseconds*1e-9
        self.capture_error=None;self.scanning=True;self.phase='SCANNING';self.clear_visuals()
        return str(self.session)

    def capture(self,req,res):
        with self.lock:
            try:
                if req.data:
                    if (not self.scanning or self.gate or not self.states or time.monotonic()-self.lease>=.6
                            or time.monotonic()-self.states[-1][3]>self.c['joint_timeout']):
                        raise ValueError('Need active scan, closed gate, fresh feedback and lease')
                    if np.max(abs(self.states[-1][1]))>self.c['settle_speed']:
                        raise ValueError('Robot not stationary')
                    self.capture_epoch+=1;self.gate_frames=0
                    self.gate_cutoff=self.get_clock().now().nanoseconds*1e-9
                    self.gate_q=self.states[-1][0].copy()
                self.gate=req.data
                res.success=True;res.message=json.dumps(dict(epoch=self.capture_epoch,frames=self.gate_frames))
            except Exception as e:res.success=False;res.message=str(e)
        return res

    def record_capture(self,msg,info,q,poses):
        # Called under the worker lock only after a fresh complete ESDF is accepted.
        if self.gate_frames==0:
            folder=self.session/'observations'/f'{len(self.observations)+1:03d}';folder.mkdir(parents=True)
            np.savez_compressed(folder/'depth.npz',meters=depth_meters(msg,1,self.c['min_depth'],self.c['max_depth']))
            record=dict(epoch=self.capture_epoch,q_rad=q.tolist(),stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        T_base_depth=poses[self.c['camera_frame']].tolist(),camera_info=self.info_record(info))
            write_json(folder/'pose.json',record);self.observations.append(record)
            write_json(self.session/'observations.json',self.observations)
        self.gate_frames+=1

    def publish_cloud(self,points,distance,stamp,force=False,raw=None,unknown=100.):
        self.visuals.put((self.epoch,points,distance,stamp,raw,unknown))
        if raw is not None:self.raw_surface=(points.copy(),raw.copy(),stamp,float(unknown))

    def freeze_scan(self):
        if not self.scanning or self.gate or self.latest_field is None:
            raise ValueError('Need active scan with closed gate and completed ESDF')
        if self.capture_count<self.c['min_frames'] or len(self.views)<self.c['min_views']:
            raise ValueError('Insufficient completed frames/views; partial data retained')
        d,s,o,dims,stamp,seq,received,info=self.latest_field
        if self.camera_info is None or self.info_record(self.camera_info)!=info:
            raise ValueError('Camera calibration changed before freeze')
        if self.tabletop_evidence is None:raise ValueError('Missing depth-derived column evidence')
        candidate=FrozenMap.create(d,s,o,dims,self.c['grid_resolution'],dict(identity=self.model_id,stamp=stamp,
                    planning_policy=PLANNING_POLICY,tabletop=self.c['tabletop'],source_labels={'0':'unknown_or_occupied','1':'depth_ray','2':'observed_top_column'},
                    epoch=self.epoch,sequence=seq,views=[v.tolist() for v in self.views],camera_info=info,
                    observations=self.observations,source='ur7e_gazebo_esdf_sage',planning_ready=False))
        candidate.save(self.session/'frozen_map.npz')
        np.savez_compressed(self.session/'tabletop_evidence.npz',**self.tabletop_evidence)
        if self.raw_surface is not None:
            points,raw,ts,unknown=self.raw_surface
            np.savez_compressed(self.session/'native_esdf.npz',points=points,distance=raw,stamp=ts,unknown=unknown)
        summary=dict(frames=self.capture_count,views=len(self.views),observations=len(self.observations),map_uid=candidate.uid,
                     observed_cells=int(np.count_nonzero(s==1)),tabletop_cells=int(np.count_nonzero(s==2)),scan_prior_cells=0,planning_policy=PLANNING_POLICY,planning_ready=False)
        if self.states:
            centers=self.geometry.spheres(self.states[-1][0]);clearance=candidate.snapshot.query(centers)-self.geometry.radii
            summary.update(current_arm_min_clearance_m=float(np.min(clearance)),
                           current_self_clearance_m=float(self.geometry.self_clearance(centers)),
                           current_arm_observed_safe=bool(np.all(clearance>=self.c['collision_margin'])),
                           blocked_spheres=[dict(index=int(i),link=self.geometry.links[self.geometry.link_ids[i]],center=centers[i].tolist(),clearance_m=float(clearance[i])) for i in np.flatnonzero(clearance<self.c['collision_margin'])])
        write_json(self.session/'summary.json',summary)
        self.frozen=candidate;self.scanning=False;self.phase='FROZEN'
        return str(self.session/'frozen_map.npz')

    def load(self,path):
        self.frozen=FrozenMap.load(path,self.model_id);self.session=Path(path).parent;self.phase='FROZEN'
        if self.frozen.metadata.get('planning_policy')!=PLANNING_POLICY:
            raise ValueError('Map uses an obsolete planning free-space policy')
        self.capture_count=int(self.frozen.metadata['sequence'])
        snap=self.frozen.snapshot;z,y,x=np.indices(snap.distance.shape)
        points=snap.origin+snap.resolution*np.stack([x,y,z],-1).reshape(-1,3)
        raw=None;unknown=100.
        native=self.session/'native_esdf.npz'
        if native.exists():
            with np.load(native,allow_pickle=False) as a:
                if a['points'].shape==points.shape and np.allclose(a['points'],points):raw=a['distance'].copy();unknown=float(a['unknown'])
        self.publish_cloud(points,snap.distance.reshape(-1),snap.stamp,raw=raw,unknown=unknown)

    def warmup(self,req,res):
        with self.lock:
            if self.gate or self.camera_info is None:
                res.success=False;res.message='Close capture gate and wait for CameraInfo';return res
            info=self.camera_info;sig=self.info_record(info)
            if self.warmup_state=='WARMING' or self.warmup_state=='READY' and sig==self.warmup_signature:
                res.success=True;res.message=self.warmup_state;return res
            self.warmup_state='WARMING';self.warmup_error=None
            self.warmup_thread=threading.Thread(target=self._warmup,args=(info,sig),daemon=True);self.warmup_thread.start()
        res.success=True;res.message='WARMING';return res

    def _warmup(self,info,sig):
        if self.stop_event.wait(.3):return
        try:
            intr=intrinsics_from_info(info,self.c['depth_stride']);field=FieldBuilder(self.c)
            backend=EfficientBackend(depth_calibration(intr),{'voxel_size_m':self.c['voxel_size'],'max_integration_distance_m':self.c['max_depth']})
            backend.prepare_query(field.points);depth=np.full((intr.height,intr.width),.5,np.float32)
            backend.integrate_depth(depth,np.eye(4));backend.update_esdf();raw=backend.query_prepared()
            field.update(raw,known_distances(raw,backend.esdf_unknown_distance),depth,np.eye(4),intr)
            del backend,field
            with self.lock:self.warmup_signature=sig;self.warmup_state='READY'
        except Exception as e:
            with self.lock:self.warmup_error=str(e);self.warmup_state='ERROR'

    def report(self):
        with self.lock:
            value=dict(phase=self.phase,error=self.capture_error,epoch=self.capture_epoch,gate=self.gate,
                       gate_frames=self.gate_frames,frames=self.capture_count,views=len(self.observations),
                       session=None if self.session is None else str(self.session),warmup_state=self.warmup_state,
                       warmup_error=self.warmup_error,sensor_ready=time.monotonic()-self.last_input<self.c['frame_timeout'] and self.camera_info is not None,
                       wait_reason=self.capture_wait_reason,metrics=self.capture_metrics,last_wait=self.capture_debug,
                       lease_age=time.monotonic()-self.lease,last_depth_stamp=self.last_depth_stamp,
                       input_depth_age_s=self.input_depth_age,
                       latest_joint_stamp=None if not self.states else self.states[-1][2],
                       sim_time=self.get_clock().now().nanoseconds*1e-9,stats=self.capture_stats.snapshot())
        if self.session is not None and time.monotonic()-self.last_report_write>1.:
            write_json(self.session/'capture_status.json',value);self.last_report_write=time.monotonic()
        self.status_pub.publish(String(data=json.dumps(value)))

    def close(self):
        self.stop_event.set();self.gate=False
        self.capture_thread.join();self.visual_thread.join()
        if self.warmup_thread:self.warmup_thread.join()


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--load',default='')
    args,ros=p.parse_known_args();rclpy.init(args=ros)
    node=EsdfMapper(load_mapping(args.config),args.load)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
