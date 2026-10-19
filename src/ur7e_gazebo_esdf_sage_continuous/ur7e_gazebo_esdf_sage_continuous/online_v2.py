"""Sensor-only online added-obstacle layer over an immutable static policy.

No timeout clears occupancy. Only subsequent valid free rays clear old cells.
"""
from dataclasses import replace
from pathlib import Path
import json,threading,time
import numpy as np
from scipy.ndimage import distance_transform_edt
from rclpy.time import Time
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,transform_matrix
from ur7e_perception_planning_core.grid import observed_free
from .self_filter import mask

from .online import DynamicBlocked


def update_occupancy(previous,free,hit,shape):
    # Occupancy wins over contradictory clearing within the same frame.
    return ((previous & ~free) | hit).reshape(shape)



_worker_context=None

def init_geometry_worker(config,extra,base,static_obstacle):
    global _worker_context
    import signal
    signal.signal(signal.SIGINT,signal.SIG_IGN) # Parent owns orderly worker shutdown.
    from ur7e_real_static.model import Geometry
    g=Geometry(config['robot_urdf'],config['collision_spheres'],config['tool_frame'],config['camera_frame'])
    z,y,x=np.indices(base.distance.shape)
    points=base.origin+base.resolution*np.stack([x,y,z],-1).reshape(-1,3)
    _worker_context=(g,extra,base,static_obstacle,points)

def process_geometry(depth,pose,intr,q,occupied,old_distance,ray_depth=None,ray_intr=None):
    g,extra,base,static_obstacle,grid_points=_worker_context
    timing={};t0=time.monotonic()
    if ray_depth is None:ray_depth=depth.copy()
    if ray_intr is None:ray_intr=intr
    depth[mask(depth,pose,intr,g.spheres(q),g.radii,g.link_ids,extra)]=0.
    timing["mask"]=time.monotonic()-t0;t0=time.monotonic()
    v,u=np.indices(depth.shape);valid=depth>0
    camera=np.stack([(u-intr.cx)*depth/intr.fx,(v-intr.cy)*depth/intr.fy,depth],-1)[valid]
    points=camera@pose[:3,:3].T+pose[:3,3]
    ix=np.rint((points-base.origin)/base.resolution).astype(int)
    inside=((ix>=1)&(ix<base.dimensions-1)).all(axis=1);points=points[inside];ix=ix[inside]
    # A visible return inconsistent with the static measured surface
    # is an added obstacle, including additions in old unknown space.
    novelty=static_obstacle.query(points)>.04
    hit=np.zeros(occupied.shape,bool)
    ids=ix[novelty];hit[ids[:,2],ids[:,1],ids[:,0]]=True
    for offset in ([1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]):
        adjacent=ids+offset;hit[adjacent[:,2],adjacent[:,1],adjacent[:,0]]=True
    # Clearing only matters at previously occupied cells; evaluating
    # all 724k cells needlessly competed with planning and callbacks.
    timing["hits"]=time.monotonic()-t0;t0=time.monotonic()
    occupied_ids=np.flatnonzero(occupied)
    free=np.zeros(occupied.size,bool)
    if occupied_ids.size:free[occupied_ids]=observed_free(grid_points[occupied_ids],ray_depth,pose,ray_intr,.025+np.sqrt(3)*base.resolution/2)
    updated=update_occupancy(occupied.reshape(-1),free,hit.reshape(-1),occupied.shape)
    timing["clearing"]=time.monotonic()-t0;t0=time.monotonic()
    changed=not np.array_equal(updated,occupied)
    distance=old_distance
    if changed:
        dynamic=(distance_transform_edt(~updated)*base.resolution-np.sqrt(3)*base.resolution/2) if updated.any() else np.full(updated.shape,np.inf)
        distance=np.minimum(base.distance,dynamic-.02).astype(np.float32);distance.flags.writeable=False
    timing["distance"]=time.monotonic()-t0
    return depth,updated,hit,distance,changed,timing

class _LayerWorker:
    max_age=.5
    def __init__(self,mapper,base,output,sink):
        self.mapper=mapper;self.base=base;self.output=Path(output)/'online';self.output.mkdir(exist_ok=True)
        self.lock=threading.RLock();self.stop=threading.Event();self.pending=None
        self.snapshot=base;self.version=0;self.frames=0;self.stamp=0.;self.received=0.;self.error=None
        self.occupied=np.zeros(base.distance.shape,bool);self.last_saved=-1
        z,y,x=np.indices(base.distance.shape);self.points=base.origin+base.resolution*np.stack([x,y,z],-1).reshape(-1,3)
        with np.load(mapper.session/'partial_policy.npz') as d:self.static_obstacle=replace(base,distance=d['obstacle'].copy())
        from concurrent.futures import ProcessPoolExecutor
        from multiprocessing import get_context
        self.pool=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'),initializer=init_geometry_worker,initargs=(mapper.c,mapper.extra_mask,base,self.static_obstacle))
        self.sink=sink
        self.diagnostics=(self.output/'diagnostics.jsonl').open('w');self.counts={}
        self.log=(self.output/'updates.jsonl').open('w');self.thread=threading.Thread(target=self.worker,daemon=True);self.thread.start()
    def submit(self,msg,info,stamp):
        if info is None:return
        with self.lock:self.pending=(msg,info,stamp,time.monotonic())
    def read(self,now):
        with self.lock:
            if not self.frames or now-self.stamp>self.max_age or now<self.stamp-.02 or time.monotonic()-self.received>self.max_age:
                raise DynamicBlocked('ONLINE_MAP_STALE')
            return self.snapshot,self.version
    def status(self):
        with self.lock:return dict(version=self.version,frames=self.frames,stamp=self.stamp,occupied_cells=int(self.occupied.sum()),error=self.error)
    def diagnostic(self,reason,**values):
        self.counts[reason]=self.counts.get(reason,0)+1
        self.diagnostics.write(json.dumps(dict(reason=reason,wall=time.monotonic(),**values))+'\n');self.diagnostics.flush()
    def worker(self):
        while not self.stop.wait(.01):
            with self.lock:
                item=self.pending;self.pending=None
            if item is None:continue
            msg,info,stamp,received=item;now=self.mapper.get_clock().now().nanoseconds*1e-9
            if now-stamp>.25 or time.monotonic()-received>.25:
                with self.lock:
                    if self.pending is item:self.pending=None
                self.diagnostic('input_expired',stamp=stamp);continue
            try:
                # Retain this image while its joint/TF samples catch up. New arrivals
                # replace the next slot, never the frame currently synchronizing.
                until=received+.25;state=None;pose=None
                while time.monotonic()<until and not self.stop.is_set():
                    state=self.mapper.state_at(stamp)
                    if state is not None:
                        try:
                            pose=transform_matrix(self.mapper.tf_buffer.lookup_transform(self.mapper.c['base_frame'],self.mapper.c['camera_frame'],Time.from_msg(msg.header.stamp)).transform)
                            break
                        except Exception:pass
                    self.stop.wait(.003)
                if state is None or pose is None:
                    self.diagnostic('sync_timeout',stamp=stamp);continue
                q,_=state
                expected=self.mapper.geometry.fk(q)[self.mapper.geometry.camera]
                if np.max(abs(expected-pose))>.003:raise ValueError('ONLINE_TF_MODEL_MISMATCH')
                if msg.header.frame_id!=self.mapper.c['camera_frame'] or info.header.frame_id!=msg.header.frame_id:raise ValueError('ONLINE_FRAME_MISMATCH')
                with self.lock:
                    if self.pending is item:self.pending=None
                signature=(info.width,info.height,info.header.frame_id,tuple(info.k),tuple(info.p),msg.encoding)
                if signature!=self.mapper.camera_signature:raise ValueError('ONLINE_CALIBRATION_CHANGED')
                started=time.monotonic();intr=intrinsics_from_info(info,4);ray_intr=intrinsics_from_info(info,2)
                depth=depth_meters(msg,4,self.mapper.c['min_depth'],self.mapper.c['max_depth']);ray_depth=depth_meters(msg,2,self.mapper.c['min_depth'],self.mapper.c['max_depth']);g=self.mapper.geometry
                if np.count_nonzero(ray_depth)<100:raise ValueError('ONLINE_DEPTH_INVALID')
                depth,updated,hit,distance,changed,timing=self.pool.submit(process_geometry,depth,pose,intr,q,self.occupied,self.snapshot.distance,ray_depth,ray_intr).result()
                distance.flags.writeable=False
                completed=time.monotonic();now=self.mapper.get_clock().now().nanoseconds*1e-9
                if now-stamp>self.max_age or completed-received>self.max_age:
                    self.diagnostic('processing_expired',timing=timing,stamp=stamp,processing=completed-started,age=now-stamp,wall_age=completed-received);continue
                with self.lock:
                    self.frames+=1;self.version+=int(changed);self.occupied=updated;self.stamp=stamp;self.received=received;self.error=None
                    self.snapshot=replace(self.base,distance=distance,stamp=stamp,sequence=self.version,received=received)
                    version=self.version;frames=self.frames
                self.sink(distance,version,frames,stamp,received,int(updated.sum()))
                self.log.write(json.dumps(dict(frame=frames,version=version,image_stamp=stamp,commit_sim_time=now,wall_time=completed,latency=completed-received,processing=completed-started,timing=timing,occupied_cells=int(updated.sum()),new_endpoint_cells=int(hit.sum()),q=q.tolist()))+'\n');self.log.flush()
                # Persist every changed occupancy for an independent execution audit.
                if changed:
                    np.savez_compressed(self.output/f'version_{version:06d}.npz',occupied=updated,image_stamp=stamp,commit_sim_time=now)
                if changed or frames%20==0:
                    np.savez_compressed(self.output/f'observation_{frames:06d}.npz',depth=depth,ray_depth=ray_depth,ray_intrinsics=np.array([ray_intr.fx,ray_intr.fy,ray_intr.cx,ray_intr.cy,ray_intr.width,ray_intr.height]),pose=pose,q=q,stamp=stamp,hit=hit,intrinsics=np.array([intr.fx,intr.fy,intr.cx,intr.cy,intr.width,intr.height]))
            except Exception as exc:
                self.diagnostic('exception',error=str(exc),stamp=stamp)
                with self.lock:self.error=str(exc)
                self.stop.wait(.01)
    def close(self):
        self.stop.set();self.thread.join(5);self.log.close();self.diagnostics.close();self.pool.shutdown(wait=True,cancel_futures=True)


def _sensor_process(config,extra,signature,session,base,output,shared,header,mutex,stop):
    """Own ROS executor and geometry worker; never shares a GIL with SAGE/servo."""
    import sys
    sys.setswitchinterval(.00005)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.executors import SingleThreadedExecutor
    from tf2_ros import Buffer,TransformListener
    from sensor_msgs.msg import JointState,Image,CameraInfo
    from collections import deque
    from .mapper import EsdfMapper
    from ur7e_real_static.model import Geometry
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    class Sensor(Node):
        on_state=EsdfMapper.on_state
        state_at=EsdfMapper.state_at
        def __init__(self):
            super().__init__('online_sensor_v2',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.c=config;self.extra_mask=extra;self.camera_signature=signature;self.session=Path(session)
            self.lock=threading.RLock();self.states=deque(maxlen=2000);self.camera_info=None
            self.geometry=Geometry(config['robot_urdf'],config['collision_spheres'],config['tool_frame'],config['camera_frame'])
            self.tf_buffer=Buffer(cache_time=Duration(seconds=10),node=self);self.tf_listener=TransformListener(self.tf_buffer,self)
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(JointState,config['joint_topic'],self.on_state,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(CameraInfo,config['camera_info_topic'],self.info,qos)
            self.create_subscription(Image,config['depth_topic'],self.depth,qos)
        def info(self,msg):self.camera_info=msg
        def depth(self,msg):
            if self.camera_info is not None:self.layer.submit(msg,self.camera_info,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    array=np.frombuffer(shared,dtype=np.float32).reshape(base.distance.shape)
    def sink(distance,version,frames,stamp,received,count):
        with mutex:
            if version!=int(header[0]):np.copyto(array,distance)
            header[:]=[version,frames,stamp,received,count]
    node=Sensor();node.layer=_LayerWorker(node,base,output,sink);executor=SingleThreadedExecutor();executor.add_node(node)
    try:
        while not stop.is_set():executor.spin_once(timeout_sec=.05)
    finally:
        node.layer.close();executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


class OnlineLayer:
    max_age=.5
    def __init__(self,mapper,base,output):
        from multiprocessing import get_context
        ctx=get_context('spawn');self.base=base;self.snapshot=base;self.local_version=0
        self.shared=ctx.RawArray('f',base.distance.size);np.copyto(np.frombuffer(self.shared,dtype=np.float32),base.distance.ravel())
        self.header=ctx.RawArray('d',5);self.mutex=ctx.RLock();self.stop_event=ctx.Event()
        self.process=ctx.Process(target=_sensor_process,args=(mapper.c,mapper.extra_mask,mapper.camera_signature,str(mapper.session),base,output,self.shared,self.header,self.mutex,self.stop_event),name='online_sensor_v2')
        self.process.start()
    @property
    def frames(self):
        with self.mutex:return int(self.header[1])
    @property
    def version(self):
        with self.mutex:return int(self.header[0])
    def submit(self,*args):pass # The isolated ROS subscriber owns image ingestion.
    def read(self,now):
        with self.mutex:
            version,frames,stamp,received,count=self.header[:]
            if not frames or now-stamp>self.max_age or now<stamp-.02 or time.monotonic()-received>self.max_age or not self.process.is_alive():raise DynamicBlocked('ONLINE_MAP_STALE')
            if int(version)!=self.local_version:
                distance=np.frombuffer(self.shared,dtype=np.float32).reshape(self.base.distance.shape).copy();distance.flags.writeable=False
                self.snapshot=replace(self.base,distance=distance);self.local_version=int(version)
            return replace(self.snapshot,stamp=stamp,received=received,sequence=int(version)),int(version)
    def status(self):
        with self.mutex:
            version,frames,stamp,received,count=self.header[:]
        return dict(version=int(version),frames=int(frames),stamp=stamp,occupied_cells=int(count),error=None if self.process.is_alive() else 'ONLINE_WORKER_EXITED')
    def close(self):
        self.stop_event.set();self.process.join(10)
        if self.process.is_alive():self.process.terminate();self.process.join(5)
