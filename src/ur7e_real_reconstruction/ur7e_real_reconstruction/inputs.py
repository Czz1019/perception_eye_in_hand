"""Real sensor validation, rectification and stationary image-time joint matching."""
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_reconstruction.color import rgb_image
from ur7e_nvblox.depth import intrinsics_from_info


def load_config(path):
    defaults = yaml.safe_load((Path(get_package_share_directory('ur7e_real_reconstruction'))/'config/real.yaml').read_text())
    supplied = yaml.safe_load(Path(path).expanduser().read_text()) or {}
    if set(supplied)-set(defaults): raise ValueError(f'Unknown keys: {set(supplied)-set(defaults)}')
    c = dict(defaults, **supplied)
    for key in ['robot_urdf', 'collision_spheres']:
        p = Path(c[key]).expanduser()
        if not p.is_absolute(): p = Path(path).resolve().parent/p
        if not p.is_file(): raise ValueError(f'Missing {key}: {p}')
        c[key] = str(p.resolve())
    if c['world_frame'] != 'base_link': raise ValueError('world_frame must be base_link')
    for key in ['capture_hz','mesh_hz','voxel_size_m','min_depth_m','max_depth_m','frame_timeout',
                'sync_slop_sec','settle_seconds','settle_speed','view_separation_rad']:
        if not np.isfinite(c[key]) or c[key] <= 0: raise ValueError(f'Invalid {key}')
    if type(c['depth_stride']) is not int or c['depth_stride']<1: raise ValueError('Invalid depth_stride')
    if c['min_depth_m']>=c['max_depth_m']: raise ValueError('Invalid depth range')
    lo,hi=np.asarray(c['roi_min_m']),np.asarray(c['roi_max_m'])
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(lo>=hi): raise ValueError('Invalid ROI')
    for key in ['depth_frame','color_frame','depth_topic','color_topic','joint_topic','camera_info_topic','color_info_topic']:
        if not isinstance(c[key],str) or not c[key]: raise ValueError(f'Invalid {key}')
    return c


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def camera_record(info):
    return dict(frame_id=info.header.frame_id, width=info.width, height=info.height,
                k=list(info.k), p=list(info.p), r=list(info.r), d=list(info.d), distortion_model=info.distortion_model)


def signature(info):
    return (info.header.frame_id,info.width,info.height,tuple(info.k),tuple(info.p),tuple(info.r),tuple(info.d),info.distortion_model)


class ColorRectifier:
    """Raw RealSense color is not assumed rectified or depth-aligned."""
    def __init__(self, info, stride):
        self.intrinsics = intrinsics_from_info(info, stride)
        self.stride = stride
        k = np.asarray(info.k).reshape(3,3)
        # Rectify distortion in the physical optical frame, without virtual rotation.
        r = np.asarray(info.r).reshape(3,3)
        if np.any(r) and not np.allclose(r,np.eye(3),atol=1e-6):
            raise ValueError('Color CameraInfo R must be identity for optical-frame TF')
        intr = intrinsics_from_info(info)
        new_k=np.array([[intr.fx,0,intr.cx],[0,intr.fy,intr.cy],[0,0,1.]])
        d=np.asarray(info.d)
        if not np.isfinite(k).all() or not np.isfinite(d).all(): raise ValueError('Invalid color calibration')
        if d.size and np.any(d):
            if info.distortion_model not in ('plumb_bob','rational_polynomial'):
                raise ValueError('Unsupported nonzero color distortion model')
            self.maps=cv2.initUndistortRectifyMap(k,d,np.eye(3),new_k,(info.width,info.height),cv2.CV_32FC1)
        else:
            if not np.allclose(k,new_k,atol=1e-5):
                self.maps=cv2.initUndistortRectifyMap(k,np.zeros(5),np.eye(3),new_k,(info.width,info.height),cv2.CV_32FC1)
            else:self.maps=None

    def decode(self,msg):
        rgb=rgb_image(msg)
        if self.maps is not None:rgb=cv2.remap(rgb,*self.maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        return np.ascontiguousarray(rgb[::self.stride,::self.stride])


class JointHistory:
    def __init__(self):self.samples=deque(maxlen=500)

    def add(self,msg):
        try:
            t=stamp(msg);ids=[msg.name.index(j) for j in JOINTS];q=np.array([msg.position[i] for i in ids])
            if t<=0 or not np.isfinite(q).all():return
            if self.samples and t<=self.samples[-1][0]:return
            # Bound processing/storage to ~50 Hz; keep original timestamps.
            if self.samples and t-self.samples[-1][0]<.018:return
            dq=np.array([msg.velocity[i] for i in ids]) if len(msg.velocity)==len(msg.name) else None
            if dq is None:
                # The initial sample seeds differencing; a full stationary history is still required.
                dq=(q-self.samples[-1][1])/(t-self.samples[-1][0]) if self.samples else np.zeros(6)
            if not np.isfinite(dq).all():return
            self.samples.append((t,q,dq))
        except (ValueError,IndexError):return

    def stationary_at(self,t,c):
        samples=list(self.samples)
        for a,b in zip(samples,samples[1:]):
            if a[0]<=t<=b[0] and b[0]-a[0]<=.1:
                q=a[1]+(t-a[0])/(b[0]-a[0])*(b[1]-a[1])
                history=[s for s in samples if t-c['settle_seconds']-.1<=s[0]<=b[0]]
                if not history or t-history[0][0]<c['settle_seconds']:return None
                if any(y[0]-x[0]>.1 for x,y in zip(history,history[1:])):return None
                if any(np.max(abs(s[2]))>c['settle_speed'] or np.max(abs(s[1]-q))>.005 for s in history):return None
                return q
        return None
