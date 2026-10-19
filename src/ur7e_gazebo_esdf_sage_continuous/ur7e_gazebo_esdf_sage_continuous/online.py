"""Sensor-only online added-obstacle layer over an immutable static policy.

No timeout clears occupancy. Only subsequent valid free rays clear old cells.
"""
from dataclasses import replace
from pathlib import Path
import json,threading,time
import numpy as np
from scipy.ndimage import distance_transform_edt,binary_dilation
from rclpy.time import Time
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,transform_matrix
from ur7e_perception_planning_core.grid import observed_free
from .self_filter import mask

class DynamicBlocked(RuntimeError):pass


def update_occupancy(previous,free,hit,shape):
    # Occupancy wins over contradictory clearing within the same frame.
    return ((previous & ~free) | hit).reshape(shape)


class OnlineLayer:
    max_age=.5
    def __init__(self,mapper,base,output):
        self.mapper=mapper;self.base=base;self.output=Path(output)/'online';self.output.mkdir(exist_ok=True)
        self.lock=threading.RLock();self.stop=threading.Event();self.pending=None
        self.snapshot=base;self.version=0;self.frames=0;self.stamp=0.;self.received=0.;self.error=None
        self.occupied=np.zeros(base.distance.shape,bool);self.last_saved=-1
        z,y,x=np.indices(base.distance.shape);self.points=base.origin+base.resolution*np.stack([x,y,z],-1).reshape(-1,3)
        with np.load(mapper.session/'partial_policy.npz') as d:self.static_obstacle=replace(base,distance=d['obstacle'].copy())
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
    def worker(self):
        while not self.stop.wait(.01):
            with self.lock:item=self.pending
            if item is None:continue
            msg,info,stamp,received=item;now=self.mapper.get_clock().now().nanoseconds*1e-9
            if now-stamp>.25 or time.monotonic()-received>.25:
                with self.lock:
                    if self.pending is item:self.pending=None
                continue
            try:
                state=self.mapper.state_at(stamp)
                if state is None:continue
                q,_=state;t=Time.from_msg(msg.header.stamp)
                pose=transform_matrix(self.mapper.tf_buffer.lookup_transform(self.mapper.c['base_frame'],self.mapper.c['camera_frame'],t).transform)
                expected=self.mapper.geometry.fk(q)[self.mapper.geometry.camera]
                if np.max(abs(expected-pose))>.003:raise ValueError('ONLINE_TF_MODEL_MISMATCH')
                if msg.header.frame_id!=self.mapper.c['camera_frame'] or info.header.frame_id!=msg.header.frame_id:raise ValueError('ONLINE_FRAME_MISMATCH')
                with self.lock:
                    if self.pending is item:self.pending=None
                signature=(info.width,info.height,info.header.frame_id,tuple(info.k),tuple(info.p),msg.encoding)
                if signature!=self.mapper.camera_signature:raise ValueError('ONLINE_CALIBRATION_CHANGED')
                started=time.monotonic();intr=intrinsics_from_info(info,2)
                depth=depth_meters(msg,2,self.mapper.c['min_depth'],self.mapper.c['max_depth']);ray_depth=depth.copy();g=self.mapper.geometry
                if np.count_nonzero(ray_depth)<100:raise ValueError('ONLINE_DEPTH_INVALID')
                depth[mask(depth,pose,intr,g.spheres(q),g.radii,g.link_ids,self.mapper.extra_mask)]=0.
                v,u=np.indices(depth.shape);valid=depth>0
                camera=np.stack([(u-intr.cx)*depth/intr.fx,(v-intr.cy)*depth/intr.fy,depth],-1)[valid]
                points=camera@pose[:3,:3].T+pose[:3,3]
                ix=np.rint((points-self.base.origin)/self.base.resolution).astype(int)
                inside=((ix>=1)&(ix<self.base.dimensions-1)).all(axis=1);points=points[inside];ix=ix[inside]
                # A visible return inconsistent with the static measured surface
                # is an added obstacle, including additions in old unknown space.
                novelty=self.static_obstacle.query(points)>.04
                hit=np.zeros(self.occupied.shape,bool)
                ids=ix[novelty];hit[ids[:,2],ids[:,1],ids[:,0]]=True
                hit=binary_dilation(hit,iterations=1)
                free=observed_free(self.points,ray_depth,pose,intr,.025+np.sqrt(3)*self.base.resolution/2)
                updated=update_occupancy(self.occupied.reshape(-1),free,hit.reshape(-1),self.occupied.shape)
                changed=not np.array_equal(updated,self.occupied)
                distance=self.snapshot.distance
                if changed:
                    dynamic=(distance_transform_edt(~updated)*self.base.resolution-np.sqrt(3)*self.base.resolution/2) if updated.any() else np.full(updated.shape,np.inf)
                    distance=np.minimum(self.base.distance,dynamic).astype(np.float32);distance.flags.writeable=False
                completed=time.monotonic();now=self.mapper.get_clock().now().nanoseconds*1e-9
                if now-stamp>self.max_age or completed-received>self.max_age:continue
                with self.lock:
                    self.frames+=1;self.version+=int(changed);self.occupied=updated;self.stamp=stamp;self.received=received;self.error=None
                    self.snapshot=replace(self.base,distance=distance,stamp=stamp,sequence=self.version,received=received)
                    version=self.version;frames=self.frames
                self.log.write(json.dumps(dict(frame=frames,version=version,image_stamp=stamp,commit_sim_time=now,wall_time=completed,latency=completed-received,processing=completed-started,occupied_cells=int(updated.sum()),new_endpoint_cells=int(hit.sum()),q=q.tolist()))+'\n');self.log.flush()
                # Persist every changed occupancy for an independent execution audit.
                if changed:
                    np.savez_compressed(self.output/f'version_{version:06d}.npz',occupied=updated,image_stamp=stamp,commit_sim_time=now)
                if changed or frames%20==0:
                    np.savez_compressed(self.output/f'observation_{frames:06d}.npz',depth=depth,ray_depth=ray_depth,pose=pose,q=q,stamp=stamp,hit=hit,intrinsics=np.array([intr.fx,intr.fy,intr.cx,intr.cy,intr.width,intr.height]))
            except Exception as exc:
                with self.lock:self.error=str(exc)
                self.stop.wait(.01)
    def close(self):
        self.stop.set();self.thread.join(5);self.log.close()
