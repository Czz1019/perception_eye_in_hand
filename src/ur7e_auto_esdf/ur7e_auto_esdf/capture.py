# Adapted real_static worker: lease/stop gate checked before fusion and commit.
"""Stationary-view nvBlox capture. No Gazebo priors and no robot commands."""
import time
from collections import deque
import numpy as np
from rclpy.time import Time
from tf2_ros import TransformException
from ur7e_nvblox.depth import depth_meters,intrinsics_from_info,depth_calibration,transform_matrix,known_distances
from ur7e_nvblox.esdf import EfficientBackend as NvbloxBackend
from ur7e_perception_planning_core.self_filter import robot_depth_mask
from ur7e_real_static.field import FieldBuilder
from ur7e_perception_planning_core.runtime import GPU_LOCK
from ur7e_real_static.trajectory import model_matches


def worker(node):
    while not node.stop_event.is_set():
        try:
            _worker(node)
        except Exception as exc:
            with node.lock:
                node.capture_error=str(exc);node.scanning=False;node.phase='CAPTURE_ERROR'
            # Remain available for an explicit new scan after initialization errors.


def _worker(node):
    epoch=-1;backend=None;intr_previous=None;field=None;sequence=0;pending=deque(maxlen=8);next_capture=0.
    while not node.stop_event.is_set():
        with node.lock: active=node.scanning and node.capture_allowed();current_epoch=node.epoch
        if not active:
            pending.clear();node.stop_event.wait(.1);continue
        new_sequence,entry=node.frames.wait_newer(sequence,.02)
        if entry is not None:
            node.capture_stats.inc('frames_overwritten',max(0,new_sequence-sequence-1))
            sequence=new_sequence;pending.append(entry)
        if epoch!=current_epoch:
            epoch=current_epoch;backend=None;intr_previous=None;field=FieldBuilder(node.c);next_capture=0.
        if time.monotonic()<next_capture:continue
        node.capture_wait_reason='WAIT_DEPTH'
        now=node.get_clock().now().nanoseconds*1e-9
        pending=deque([p for p in pending if p[0]==epoch and abs(now-p[3])<=node.c['frame_timeout']],maxlen=8)
        ready=None;picked=time.perf_counter()
        for item in reversed(pending):
            _,msg,info,stamp,received=item
            if stamp>now:node.capture_wait_reason='WAIT_IMAGE_CLOCK';continue
            if time.monotonic()-received>node.c['frame_timeout']:continue
            state=node.state_at(stamp)
            if state is None:node.capture_wait_reason='WAIT_JOINTS_AT_IMAGE_TIME';continue
            q,dq=state
            if np.max(abs(dq))>node.c['settle_speed'] or not node.settled_at(stamp,q):
                node.capture_wait_reason='WAIT_STATIONARY';continue
            try:
                t=Time.from_msg(msg.header.stamp)
                poses={link:transform_matrix(node.tf_buffer.lookup_transform('base_link',link,t).transform)
                       for link in [*node.geometry.links,node.c['camera_frame']]}
            except TransformException:node.capture_wait_reason='WAIT_TF_AT_IMAGE_TIME';continue
            if not model_matches(node.geometry,q,poses):
                node.capture_error='TF_MODEL_MISMATCH';node.capture_wait_reason='TF_MODEL_MISMATCH';continue
            ready=(item,q,poses);break
        if ready is None:
            if pending:node.capture_stats.inc('input_wait_attempts')
            continue
        transformed=time.perf_counter()
        next_capture=picked+1/node.c['capture_hz']
        item,q,poses=ready;_,msg,info,stamp,received=item
        pending=deque([p for p in pending if p[3]>stamp],maxlen=8)
        try:
            if (msg.header.frame_id!=node.c['camera_frame'] or info.header.frame_id!=msg.header.frame_id
                or (msg.width,msg.height)!=(info.width,info.height)):
                raise ValueError('Depth frame/calibration dimensions mismatch')
            intr=intrinsics_from_info(info,node.c['depth_stride'])
            signature=(info.width,info.height,info.header.frame_id,tuple(info.k),tuple(info.p),msg.encoding)
            with node.lock:
                if epoch!=node.epoch or not node.scanning or not node.capture_allowed(stamp,q):continue
                if node.camera_signature is not None and signature!=node.camera_signature:
                    raise ValueError('Camera calibration changed; restart the scan')
                node.camera_signature=signature
            if backend is None:
                with GPU_LOCK.mapping():
                    backend=NvbloxBackend(depth_calibration(intr),{'voxel_size_m':node.c['voxel_size'],'max_integration_distance_m':node.c['max_depth']})
                    backend.prepare_query(field.points)
                intr_previous=intr
            if intr!=intr_previous:raise ValueError('Depth intrinsics changed; restart scan')
            depth=depth_meters(msg,node.c['depth_stride'],node.c['min_depth'],node.c['max_depth'])
            pose=poses[node.c['camera_frame']]
            centers=np.empty_like(node.geometry.local)
            for i,link in enumerate(node.geometry.links):
                ids=node.geometry.link_ids==i;p=poses[link]
                centers[ids]=node.geometry.local[ids]@p[:3,:3].T+p[:3,3]
            depth[robot_depth_mask(depth,pose,intr,centers,node.geometry.radii,node.geometry.link_ids)]=0.
            if not np.any(depth):
                node.capture_error='EMPTY_FILTERED_DEPTH';node.capture_stats.inc('empty_filtered_depth');continue
            # Startup and GPU initialization can take time. Never fuse a stale image afterwards.
            if node.get_clock().now().nanoseconds*1e-9-stamp>node.c['frame_timeout']:
                node.capture_stats.inc('stale_before_fusion');continue
            started=time.perf_counter()
            with GPU_LOCK.mapping():
                acquired=time.perf_counter()
                with node.lock:
                    if not node.scanning or epoch!=node.epoch or not node.capture_allowed(stamp,q):continue
                if time.monotonic()-received>node.c['frame_timeout']:
                    node.capture_stats.inc('stale_after_gpu_wait');continue
                backend.integrate_depth(depth,pose);backend.update_esdf()
                raw=backend.query_prepared()
            queried=time.perf_counter()
            distance,source=field.update(raw,known_distances(raw,backend.esdf_unknown_distance),depth,pose,intr)
            with node.lock:
                if not node.scanning or epoch!=node.epoch or not node.capture_allowed(stamp,q):continue
                if node.get_clock().now().nanoseconds*1e-9-stamp>node.c['frame_timeout'] or time.monotonic()-received>node.c['frame_timeout']:
                    node.capture_stats.inc('stale_completed_field');continue
                node.capture_count+=1
                node.record_capture(msg,info,q,poses)
                if not node.views or min(np.max(abs(q-v)) for v in node.views)>=node.c['view_separation_rad']:
                    node.views.append(q.copy())
                node.latest_field=(distance,source,field.origin.copy(),field.dims.copy(),stamp,node.capture_count,time.monotonic(),node.info_record(info))
                node.capture_error=None
                node.capture_wait_reason='CAPTURING'
                node.capture_metrics=dict(input_stamp=stamp,queue_ms=(picked-received)*1000,
                    tf_ms=(transformed-picked)*1000,filter_ms=(started-transformed)*1000,
                    gpu_wait_ms=(acquired-started)*1000,native_esdf_ms=(queried-acquired)*1000,
                    free_field_ms=(time.perf_counter()-queried)*1000,
                    receive_to_result_ms=(time.monotonic()-received)*1000,
                    edt_recomputed=field.recomputed,edt_backend=getattr(field.edt,'last_backend','cpu'),
                    observed_cells=int(np.count_nonzero(source==1)),prior_cells=int(np.count_nonzero(source==2)))
                node.capture_stats.inc('fields_completed')
                node.capture_stats.add_latency('capture',node.capture_metrics['receive_to_result_ms'])
                node.publish_cloud(field.points,distance,stamp,raw=raw,unknown=backend.esdf_unknown_distance)
        except Exception as exc:
            with node.lock:
                node.capture_error=str(exc);node.scanning=False;node.phase='CAPTURE_ERROR'
