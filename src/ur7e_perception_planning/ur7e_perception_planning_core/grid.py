"""Explicit grid contract and conservative CPU/GPU distance lookup."""
from dataclasses import dataclass
import time
import zlib
from array import array
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates, minimum_filter


def pack_map(msg, native, planning, sources):
    payload = np.asarray(native, dtype='<f4').tobytes()+np.asarray(planning, dtype='<f4').tobytes()+np.asarray(sources, dtype='u1').tobytes()
    msg.compressed_data = array('B', zlib.compress(payload, level=1))


def map_arrays(msg):
    count = int(np.prod(msg.dimensions))
    if count < 8 or count > 2_000_000:
        raise ValueError('Invalid map dimensions')
    if msg.compressed_data:
        decoder = zlib.decompressobj()
        try:
            payload = decoder.decompress(bytes(msg.compressed_data), count*9+1)
        except zlib.error as exc:
            raise ValueError('Corrupt compressed map') from exc
        if len(payload) != count*9 or not decoder.eof or decoder.unused_data:
            raise ValueError('Compressed map size does not match dimensions')
        return (np.frombuffer(payload, dtype='<f4', count=count),
                np.frombuffer(payload, dtype='<f4', count=count, offset=count*4),
                np.frombuffer(payload, dtype='u1', count=count, offset=count*8))
    return np.asarray(msg.nvblox_distance), np.asarray(msg.planning_distance), np.asarray(msg.free_source)


def grid_points(center, size, resolution):
    origin = np.asarray(center) - np.asarray(size) / 2
    dims = np.ceil(np.asarray(size) / resolution).astype(int) + 1
    z, y, x = np.meshgrid(np.arange(dims[2]), np.arange(dims[1]), np.arange(dims[0]), indexing='ij')
    return origin, dims, origin + resolution * np.stack((x, y, z), -1).reshape(-1, 3)


def observed_free(points, depth, pose, intr, padding):
    """Project cell centres into a min-filtered image; never clear behind a return."""
    camera = (points - pose[:3, 3]) @ pose[:3, :3]
    z = camera[:, 2]
    u = np.rint(intr.fx * camera[:, 0] / np.maximum(z, 1e-6) + intr.cx).astype(int)
    v = np.rint(intr.fy * camera[:, 1] / np.maximum(z, 1e-6) + intr.cy).astype(int)
    inside = (z > .2) & (u >= 2) & (v >= 2) & (u < intr.width-2) & (v < intr.height-2)
    filtered = minimum_filter(depth, size=5)
    free = np.zeros(len(points), dtype=bool)
    ids = np.flatnonzero(inside)
    free[ids] = z[ids] + padding < filtered[v[ids], u[ids]]
    return free


def integration_depth(ray_depth, maximum):
    """Keep only surfaces inside the fusion range; never invent a capped surface.

    A valid farther return can still prove that a nearby ray segment is free.
    Its original range must therefore remain available to observed_free().
    """
    depth = ray_depth.copy()
    depth[depth > maximum] = 0.
    return depth


def conservative_field(raw, known, free_source, points, dims, resolution, cache=None, min_interval=0.):
    free = free_source > 0
    # A measured obstacle always overrides an initial clearance prior or old rays.
    free[known & (raw <= .02)] = False
    free[points[:, 2] <= 0.] = False  # fixed support plane, not scene obstacle geometry
    shape = tuple(dims[::-1])
    geometry_key = (*map(int, dims), float(resolution))
    cached = cache is not None and cache.get('geometry') == geometry_key
    unchanged = cached and np.array_equal(free, cache['free'])
    # Reuse an older, smaller free set only: its EDT is a LOWER bound. A newly
    # blocked cached-free cell forces immediate recomputation, regardless of rate.
    deferred = (cached and not unchanged and min_interval>0
                and time.monotonic()-cache.get('updated_at',0.)<min_interval
                and not np.any(cache['free'] & ~free))
    if unchanged or deferred:
        distance = cache['distance'].copy()
        cache.update(recomputed=False, deferred=bool(deferred))
    else:
        # Outside the free bounding box the EDT is zero. Cropping with a zero
        # border gives exactly the same Euclidean result as the full grid.
        volume = free.reshape(shape)
        distance = np.zeros(shape, dtype=np.float64)
        if np.any(free):
            bounds = []
            for axis in range(3):
                ids = np.flatnonzero(np.any(volume, axis=tuple(i for i in range(3) if i != axis)))
                bounds.append(slice(ids[0], ids[-1]+1))
            region = tuple(bounds)
            padded = np.pad(volume[region], 1, constant_values=False)
            transform = cache.get('edt_function', distance_transform_edt) if cache is not None else distance_transform_edt
            distance[region] = transform(padded)[1:-1, 1:-1, 1:-1]*resolution
        distance = distance.reshape(-1)
        if cache is not None:
            cache.update(free=free.copy(), distance=distance.copy(), geometry=geometry_key, recomputed=True,
                         deferred=False, updated_at=time.monotonic())
    distance -= np.sqrt(3.) * resolution / 2
    distance = np.minimum(distance, np.where(known, raw, np.inf))
    distance[~free] = np.minimum(distance[~free], 0.)
    sources = free_source.copy(); sources[~free] = 0
    return distance.astype(np.float32), sources


@dataclass(frozen=True)
class Snapshot:
    origin: np.ndarray
    resolution: float
    dimensions: np.ndarray
    distance: np.ndarray
    stamp: float
    epoch: int
    sequence: int
    received: float

    @classmethod
    def from_message(cls, msg):
        dims = np.asarray(msg.dimensions, dtype=int)
        _, values, _ = map_arrays(msg)
        origin = np.array([msg.origin.x, msg.origin.y, msg.origin.z])
        if (msg.header.frame_id != 'base_link' or np.any(dims < 2) or np.prod(dims) > 2_000_000
                or values.size != np.prod(dims) or not np.isfinite(values).all()
                or not np.isfinite(origin).all() or not np.isfinite(msg.resolution) or msg.resolution <= 0):
            raise ValueError('Invalid planning map geometry/data/frame')
        return cls(origin, float(msg.resolution), dims, values.reshape(tuple(dims[::-1])).copy(),
                   msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   msg.epoch, msg.sequence, time.monotonic())

    def query(self, points):
        xyz = (np.asarray(points) - self.origin) / self.resolution
        inside = ((xyz >= 0) & (xyz <= self.dimensions - 1)).all(axis=-1)
        result = map_coordinates(self.distance, xyz[..., ::-1].reshape(-1, 3).T,
                                 order=1, mode='constant', cval=-1., prefilter=False).reshape(xyz.shape[:-1])
        # Lipschitz allowance for trilinear interpolation and voxel discretization.
        return np.where(inside, result - np.sqrt(3.) * self.resolution / 2, -1.)
