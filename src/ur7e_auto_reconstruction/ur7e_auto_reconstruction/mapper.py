"""Add an explicit per-stop capture gate without altering the manual mapper."""
import argparse
import json
import time
import threading
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger
from ur7e_real_reconstruction.node import Reconstruction, write_json
from ur7e_real_reconstruction.inputs import load_config, stamp, camera_record, signature, ColorRectifier
from ur7e_nvblox.depth import depth_meters, intrinsics_from_info
from ur7e_reconstruction.color import rgb_image


class GatedReconstruction(Reconstruction):
    def __init__(self, config, load=''):
        self.gate = False
        self.epoch = 0
        self.gate_frames = 0
        self.gate_cutoff = 0.
        self.gate_q = None
        self.saved_epoch = -1
        self.lease = 0.
        self.pair_seen = 0.
        self.warmup_state = 'COLD'
        self.warmup_error = None
        self.warmup_thread = None
        self.warmup_signature = None
        super().__init__(config,load)
        self.create_service(SetBool,'~/capture',self.capture)
        self.create_service(Trigger,'~/warmup',self.warmup)
        self.capture_pub = self.create_publisher(String,'~/capture_status',10)
        self.create_subscription(Empty,'/ur7e_auto_scan/capture_lease',self.on_lease,5)
        self.create_timer(.2,self.capture_report)

    def warmup(self, req, res):
        with self.lock:
            if self.gate or self.phase not in ('IDLE','FROZEN','SCANNING') or self.info is None or self.color_info is None:
                res.success=False;res.message='Warmup requires a closed capture gate and camera calibration';return res
            if self.warmup_state=='WARMING':
                res.success=True;res.message='WARMING';return res
            info,cinfo=self.info,self.color_info
            sig=(signature(info),signature(cinfo))
            if self.warmup_state=='READY' and self.warmup_signature==sig:
                res.success=True;res.message='READY';return res
            self.warmup_state='WARMING';self.warmup_error=None
            self.warmup_thread=threading.Thread(target=self._warmup,args=(info,cinfo,sig),daemon=True,name='cuda-warmup')
            self.warmup_thread.start()
        res.success=True;res.message='WARMING';return res

    def _warmup(self,info,cinfo,sig):
        # Let the service response leave before a native import can hold the GIL.
        if self.stop.wait(.3):return
        try:
            from ur7e_reconstruction.surface import SurfaceMapper
            intr=intrinsics_from_info(info,self.c['depth_stride'])
            color_intr=ColorRectifier(cinfo,self.c['depth_stride']).intrinsics
            temporary=SurfaceMapper(intr,self.c,color_intr)
            depth=np.full((intr.height,intr.width),.5,np.float32)
            color=np.full((color_intr.height,color_intr.width,3),128,np.uint8)
            temporary.integrate(depth,np.eye(4),color,np.eye(4))
            temporary.extract()
            # Disposable GPU warmup map: never published, saved or counted as observations.
            del temporary
            with self.lock:self.warmup_state='READY';self.warmup_signature=sig
        except Exception as exc:
            with self.lock:self.warmup_state='ERROR';self.warmup_error=repr(exc)

    def close(self):
        super().close()
        if self.warmup_thread:self.warmup_thread.join()

    def on_lease(self, _):
        self.lease = time.monotonic()

    def capture(self, req, res):
        with self.lock:
            if req.data:
                if self.phase != 'SCANNING' or not self.history.samples or time.monotonic()-self.lease > .6:
                    res.success = False
                    res.message = 'Need SCANNING, joint feedback and a live coordinator lease'
                    return res
                if self.gate:
                    res.success = False
                    res.message = 'Capture already open; close before creating another epoch'
                    return res
                self.epoch += 1
                self.gate_frames = 0
                self.gate_cutoff = self.get_clock().now().nanoseconds*1e-9
                self.gate_q = self.history.samples[-1][1].copy()
            self.gate = bool(req.data)
            self.pending.clear()
            res.success = True
            res.message = json.dumps(dict(epoch=self.epoch, frames=self.gate_frames))
        return res

    def on_pair(self, depth, color):
        with self.lock:
            self.stats['all_pairs']+=1
            if max(abs(self.get_clock().now().nanoseconds*1e-9-stamp(m)) for m in (depth,color)) <= self.c['frame_timeout']:
                self.pair_seen = time.monotonic()
            if not self.gate or time.monotonic()-self.lease > .6:
                return
            if min(stamp(depth),stamp(color)) <= self.gate_cutoff:
                return
            super().on_pair(depth,color)

    def pick_ready(self):
        with self.lock:
            if not self.gate or time.monotonic()-self.lease > .6:
                self.pending.clear()
                return None
            return super().pick_ready()

    def new_session(self):
        with self.lock:
            self.gate = False
            self.gate_frames = 0
            self.saved_epoch = -1
        super().new_session()

    def record_view(self, item, q, depth_pose, color_pose):
        depth,color,info,cinfo,_ = item
        with self.lock:
            # An old in-flight image can finish fusion, but can never credit a new stop.
            if (not self.gate or min(stamp(depth),stamp(color)) <= self.gate_cutoff
                    or np.max(abs(q-self.gate_q)) > .008):
                return
            if self.saved_epoch != self.epoch:
                i = len(self.views)+1
                folder = self.session/'observations'/f'{i:03d}'
                folder.mkdir()
                PILImage.fromarray(rgb_image(color)).save(folder/'rgb.png')
                np.savez_compressed(folder/'depth.npz',raw=np.frombuffer(bytes(depth.data),np.uint8),
                    meters=depth_meters(depth,1,self.c['min_depth_m'],self.c['max_depth_m']),
                    encoding=depth.encoding,step=depth.step,width=depth.width,height=depth.height,
                    is_bigendian=depth.is_bigendian)
                view=dict(id=i,capture_epoch=self.epoch,depth_stamp=stamp(depth),color_stamp=stamp(color),
                          q_rad=q.tolist(),T_base_depth=depth_pose.tolist(),T_base_color=color_pose.tolist(),
                          depth_info=camera_record(info),color_info=camera_record(cinfo),path=f'observations/{i:03d}')
                write_json(folder/'pose.json',view)
                self.views.append(view)
                write_json(self.session/'observations.json',self.views)
                self.saved_epoch = self.epoch
            self.gate_frames += 1

    def capture_report(self):
        with self.lock:
            value=dict(warmup_state=self.warmup_state,warmup_error=self.warmup_error,phase=self.phase,error=self.error,wait_reason=self.wait_reason,epoch=self.epoch,
                       gate=self.gate,gate_frames=self.gate_frames,frames=self.frames,views=len(self.views),
                       session=None if self.session is None else str(self.session),
                       lease_fresh=time.monotonic()-self.lease <= .6,
                       joint_samples=len(self.history.samples),
                       image_pairs=self.stats['all_pairs'],
                       sensor_ready=time.monotonic()-self.pair_seen < self.c['frame_timeout'] and self.info is not None and self.color_info is not None,
                       last_input_age_sec=None if self.last_input<=self.started else
                       self.get_clock().now().nanoseconds*1e-9-self.last_input)
        self.capture_pub.publish(String(data=json.dumps(value,allow_nan=False)))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',required=True)
    p.add_argument('--load',default='')
    args,ros=p.parse_known_args()
    rclpy.init(args=ros)
    node=GatedReconstruction(load_config(args.config),args.load)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
