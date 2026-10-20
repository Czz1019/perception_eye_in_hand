"""ROS depth decoding and exact-time geometry, independent of camera SDKs."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from realtime_nvblox.types import Calibration, CameraIntrinsics


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def intrinsics_from_info(info, stride: int = 1) -> CameraIntrinsics:
    if stride < 1 or int(stride) != stride or info.width <= 0 or info.height <= 0:
        raise ValueError('Depth dimensions and integer stride must be positive')
    p = np.asarray(info.p, dtype=float).reshape(3, 4)
    k = p[:, :3] if np.isfinite(p).all() and p[0, 0] > 0 and p[1, 1] > 0 and p[2, 2] == 1 else np.asarray(info.k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('CameraInfo has no valid P or K')
    return CameraIntrinsics((info.width + stride - 1)//stride, (info.height + stride - 1)//stride,
                            k[0, 0]/stride, k[1, 1]/stride, k[0, 2]/stride, k[1, 2]/stride)


def depth_meters(msg, stride: int = 1, min_depth_m: float = .2, max_depth_m: float = 4.) -> np.ndarray:
    if msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Unsupported depth encoding: ' + msg.encoding)
    if stride < 1 or msg.height <= 0 or msg.width <= 0:
        raise ValueError('Invalid depth dimensions or stride')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('u2' if msg.encoding == '16UC1' else 'f4'))
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.step*msg.height:
        raise ValueError('Depth buffer/step shorter than image dimensions')
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.step, dtype.itemsize))[::stride, ::stride].astype(np.float32, copy=True)
    if msg.encoding == '16UC1':
        depth *= .001
    valid = np.isfinite(depth) & (depth > 0) & (depth >= min_depth_m) & (depth <= max_depth_m)
    depth[~valid] = 0.
    return np.ascontiguousarray(depth)


def calibration_from_info(info, stride: int = 1) -> Calibration:
    intr = intrinsics_from_info(info, stride)
    return Calibration(intr, intr, intr, intr, np.eye(4), np.eye(4), np.eye(4), np.eye(4), 1., 0.)


def transform_matrix(transform) -> np.ndarray:
    t = transform.transform
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    result[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite TF')
    return result


def stamp_rejection(stamp: int, now: int, floor: int, last: int, max_age_ns: int) -> str | None:
    if stamp <= 0 or now <= 0:
        return 'invalid_stamp'
    if stamp <= floor or stamp <= last:
        return 'out_of_order'
    if stamp > now:
        return 'future_depth'
    if now - stamp > max_age_ns:
        return 'stale_depth'
    return None
