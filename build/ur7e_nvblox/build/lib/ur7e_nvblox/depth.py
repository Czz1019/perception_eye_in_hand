"""Depth decoding and calibrated pinhole geometry, independent of ROS runtime."""
import numpy as np
from scipy.spatial.transform import Rotation

from realtime_nvblox.types import Calibration, CameraIntrinsics


def depth_meters(msg, stride=1, minimum=0.2, maximum=4.0):
    """Decode ROS depth respecting row padding and byte order; decimate in pixels."""
    formats = {'16UC1': ('u2', 0.001), '32FC1': ('f4', 1.0)}
    if msg.encoding not in formats:
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}; use 16UC1 or 32FC1')
    fmt, scale = formats[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + fmt)
    if stride < 1 or msg.width <= 0 or msg.height <= 0:
        raise ValueError('Invalid image dimensions or stride')
    if msg.step < msg.width * dtype.itemsize or len(msg.data) < msg.step * msg.height:
        raise ValueError('Truncated depth image or invalid row step')
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.step, dtype.itemsize))
    depth = np.array(raw[::stride, ::stride], dtype=np.float32, order='C') * scale
    valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum) & (depth > 0)
    depth[~valid] = 0.0
    return depth


def intrinsics_from_info(info, stride=1):
    # Input is a rectified depth image. P describes the rectified pinhole camera.
    p, k = info.p, info.k
    fx, fy, cx, cy = (p[0], p[5], p[2], p[6]) if p[0] > 0 and p[5] > 0 else (k[0], k[4], k[2], k[5])
    if not np.all(np.isfinite([fx, fy, cx, cy])) or fx <= 0 or fy <= 0:
        raise ValueError('CameraInfo has invalid pinhole intrinsics')
    if info.width <= 0 or info.height <= 0 or stride < 1:
        raise ValueError('Invalid CameraInfo dimensions or stride')
    return CameraIntrinsics(
        (info.width + stride - 1) // stride, (info.height + stride - 1) // stride,
        fx / stride, fy / stride, cx / stride, cy / stride,
    )


def depth_calibration(intrinsics):
    # The reusable backend requires the full Calibration record, but only the
    # depth sensor is used. Images are already converted to meters (scale=1).
    return Calibration(intrinsics, intrinsics, intrinsics, intrinsics,
                       np.eye(4), np.eye(4), np.eye(4), np.eye(4), 1.0, 0.0)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def known_distances(distances, unknown_value):
    d = np.asarray(distances)
    return np.isfinite(d) & ~np.isclose(np.abs(d), abs(unknown_value), rtol=1e-5, atol=1e-3)
