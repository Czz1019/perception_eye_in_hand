"""Stationary-view nvBlox capture. No Gazebo priors and no robot commands."""
import time
from collections import deque
import numpy as np
from rclpy.time import Time
from tf2_ros import TransformException
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,depth_calibration,transform_matrix,known_distances
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.ros_publishers import esdf_cloud
from ur7e_perception_planning_core.self_filter import robot_depth_mask
from .field import FieldBuilder
from ur7e_perception_planning_core.runtime import GPU_LOCK
from .trajectory import model_matches


def worker(node):
    epoch=-1;backend=None;intr_previous=None;field=None;sequence=0;pending=deque(maxlen=8)
    while not node.stop_event.is_set():
        with node.lock: active=node.scanning;current_epoch=node.epoch
        if not active:
            pending.clear();node.stop_event.wait(.1);continue
        new_sequence,entry=node.frames.wait_newer(sequence,.1)
        if entry is not None:sequence=new_sequence;pending.append(entry)
        if epoch!=current_epoch:
            epoch=current_epoch;backend=None;intr_previous=None;field=FieldBuilder(node.c)
        now=node.get_clock().now().nanoseconds*1e-9
        pending=deque([p for p in pending if p[0]==epoch and 0<=now-p[3]<=node.c['frame_timeout']],maxlen=8)
        ready=None
        for item in reversed(pending):
            _,msg,info,stamp,received=item
            if time.monotonic()-received>node.c['frame_timeout']:continue
            state=node.state_at(stamp)
            if state is None:continue
            q,dq=state
            if np.max(abs(dq))>node.c['settle_speed'] or not node.settled_at(stamp,q):continue
            try:
                t=Time.from_msg(msg.header.stamp)
                poses={link:transform_matrix(node.tf_buffer.lookup_transform('base_link',link,t).transform)
                       for link in [*node.geometry.links,node.c['camera_frame']]}
            except TransformException:continue
            if not model_matches(node.geometry,q,poses):
                node.capture_error='TF_MODEL_MISMATCH';continue
            ready=(item,q,poses);break
        if ready is None:continue
        item,q,poses=ready;_,msg,info,stamp,received=item
        pending=deque([p for p in pending if p[3]>stamp],maxlen=8)
        try:
            if (msg.header.frame_id!=node.c['camera_frame'] or info.header.frame_id!=msg.header.frame_id
                or (msg.width,msg.height)!=(info.width,info.height)):
                raise ValueError('Depth frame/calibration dimensions mismatch')
            intr=intrinsics_from_info(info,node.c['depth_stride'])
            signature=(info.width,info.height,info.header.frame_id,tuple(info.k),tuple(info.p),msg.encoding)
            with node.lock:
                if node.camera_signature is not None and signature!=node.camera_signature:
                    raise ValueError('Camera calibration changed; restart the scan')
                node.camera_signature=signature
            if backend is None:
                with GPU_LOCK.mapping():
                    backend=NvbloxBackend(depth_calibration(intr),{'voxel_size_m':node.c['voxel_size'],'max_integration_distance_m':node.c['max_depth']})
                intr_previous=intr
            if intr!=intr_previous:raise ValueError('Depth intrinsics changed; restart scan')
            depth=depth_meters(msg,node.c['depth_stride'],node.c['min_depth'],node.c['max_depth'])
            pose=poses[node.c['camera_frame']]
            centers=np.empty_like(node.geometry.local)
            for i,link in enumerate(node.geometry.links):
                ids=node.geometry.link_ids==i;p=poses[link]
                centers[ids]=node.geometry.local[ids]@p[:3,:3].T+p[:3,3]
            depth[robot_depth_mask(depth,pose,intr,centers,node.geometry.radii,node.geometry.link_ids)]=0.
            if not np.any(depth):node.capture_error='EMPTY_FILTERED_DEPTH';continue
            # Startup and GPU initialization can take time. Never fuse a stale image afterwards.
            if node.get_clock().now().nanoseconds*1e-9-stamp>node.c['frame_timeout']:continue
            with GPU_LOCK.mapping():
                backend.integrate_depth(depth,pose);backend.update_esdf()
                raw=backend.query_esdf(field.points).reshape(-1)
            distance,source=field.update(raw,known_distances(raw,backend.esdf_unknown_distance),depth,pose,intr)
            with node.lock:
                if not node.scanning or epoch!=node.epoch:continue
                node.capture_count+=1
                if not node.views or min(np.max(abs(q-v)) for v in node.views)>=node.c['view_separation_rad']:
                    node.views.append(q.copy())
                node.latest_field=(distance,source,field.origin.copy(),field.dims.copy(),stamp,node.capture_count,time.monotonic(),node.info_record(info))
                node.capture_error=None
            if time.monotonic()-node.last_cloud>=1.:
                node.surface_pub.publish(esdf_cloud(field.points,raw,'base_link',int(stamp*1e9),.05,unknown_distance_m=backend.esdf_unknown_distance))
            node.publish_cloud(field.points,distance,stamp)
        except Exception as exc:
            with node.lock:
                node.capture_error=str(exc);node.scanning=False;node.phase='CAPTURE_ERROR'
