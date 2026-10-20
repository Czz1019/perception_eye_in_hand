"""Scan-only geometric prior, separate from the depth-derived planning field."""
import numpy as np
import yaml
from scipy.spatial import cKDTree
from ur7e_real_static.field import FieldBuilder as StaticField
from ur7e_real_static.model import Geometry


def scan_samples(views, step=.015):
    samples = [views[0]]
    for a, b in zip(views, views[1:]):
        samples.extend(np.linspace(a, b, max(2, int(np.ceil(np.max(abs(b-a))/step))+1))[1:])
    return np.asarray(samples)


class ScanPriorField(StaticField):
    def __init__(self, c):
        super().__init__(c)
        config = yaml.safe_load(open(c['scan_config']))
        g = Geometry(c['robot_urdf'], c['collision_spheres'])
        centers = g.spheres_batch(scan_samples(np.array(config['views'])))
        # Offline fixture validation must check the full inflated corridor.
        for radius in np.unique(g.radii):
            group = centers[:, g.radii == radius].reshape(-1, 3)
            distance = cKDTree(group).query(self.points, workers=1)[0]
            self.source[distance < radius+config['prior_clearance']] = 2


PLANNING_POLICY = 'depth_tabletop_columns_v1'


class FieldBuilder(StaticField):
    """Only depth rays and declared tabletop extrapolation enter planning.

    A column needs top-facing, continuous depth patches covering all four XY
    quadrants, in at least two distinct camera views. No XY hole filling occurs.
    All measured surfaces (including sides) raise the column's height bound.
    A one-cell neighborhood erodes support and dilates heights at boundaries.
    """
    def __init__(self, c):
        if c['commissioned_free_boxes']:
            raise ValueError('Planning forbids commissioned free-space boxes')
        super().__init__(c)
        self.tabletop = dict(c['tabletop'])
        self.rays = np.zeros(len(self.points), bool)
        self.height = np.full((self.dims[1], self.dims[0]), -np.inf)
        self.views = []
        self.coverage = []
        self.reliable = np.zeros(self.height.shape, bool)
        self.upper_height = self.height.copy()

    def _observe_tops(self, depth, pose, intr):
        from scipy.ndimage import minimum_filter, maximum_filter
        cfg = self.tabletop
        # A scene-facing camera must actually see surfaces from above.
        if -pose[2, 2] < cfg['min_upward_normal_z']:
            return
        yy, xx = np.indices(depth.shape)
        valid = np.isfinite(depth) & (depth >= self.c['min_depth']) & (depth <= self.c['max_depth'])
        safe = np.where(valid, depth, 0.)
        camera = np.stack(((xx-intr.cx)*safe/intr.fx, (yy-intr.cy)*safe/intr.fy, safe), -1)
        world = camera @ pose[:3, :3].T + pose[:3, 3]
        ix = np.floor((world[..., 0]-self.origin[0])/self.c['grid_resolution']+.5).astype(int)
        iy = np.floor((world[..., 1]-self.origin[1])/self.c['grid_resolution']+.5).astype(int)
        valid &= (ix >= 0) & (ix < self.dims[0]) & (iy >= 0) & (iy < self.dims[1])
        valid &= (world[..., :2] >= cfg['xy_min']).all(-1) & (world[..., :2] <= cfg['xy_max']).all(-1)
        valid &= (world[..., 2] >= cfg['z_min']) & (world[..., 2] <= cfg['z_max'])
        np.maximum.at(self.height, (iy[valid], ix[valid]), world[..., 2][valid])
        # Reject invalid pixels, silhouettes/depth jumps and vertical faces.
        dx = world[1:-1, 2:] - world[1:-1, :-2]
        dy = world[2:, 1:-1] - world[:-2, 1:-1]
        normal = np.cross(dx, dy)
        length = np.linalg.norm(normal, axis=-1)
        # Orient normals toward the observing camera, not arbitrarily upward.
        facing = pose[:3, 3] - world[1:-1, 1:-1]
        normal *= np.where((normal*facing).sum(-1) >= 0, 1., -1.)[..., None]
        top = np.zeros(depth.shape, bool)
        top[1:-1, 1:-1] = normal[..., 2] >= cfg['min_upward_normal_z']*np.maximum(length, 1e-12)
        continuous = maximum_filter(safe, size=3)-minimum_filter(safe, size=3) <= cfg['max_patch_depth_span_m']
        top &= minimum_filter(valid.astype(np.uint8), size=3, mode='constant', cval=0).astype(bool) & continuous
        ids = np.flatnonzero(top)
        if not len(ids):return
        selected = None
        for k, previous in enumerate(self.views):
            angle = np.arccos(np.clip((np.trace(previous[:3,:3].T @ pose[:3,:3])-1)/2, -1., 1.))
            if np.linalg.norm(previous[:3,3]-pose[:3,3]) < cfg['view_translation_m'] and angle < cfg['view_rotation_rad']:
                selected = k;break
        if selected is None:
            if len(self.views) >= 64:raise ValueError('Too many stationary observation views')
            selected = len(self.views);self.views.append(pose.copy());self.coverage.append(np.zeros(self.height.shape, np.uint8))
        px = ix.flat[ids];py = iy.flat[ids];p = world.reshape(-1,3)[ids]
        quadrant = (p[:,0] >= self.origin[0]+px*self.c['grid_resolution']).astype(int)
        quadrant += 2*(p[:,1] >= self.origin[1]+py*self.c['grid_resolution']).astype(int)
        np.bitwise_or.at(self.coverage[selected], (py, px), (1 << quadrant).astype(np.uint8))

    def update(self, raw, known, depth, pose, intr, ray_depth=None):
        from scipy.ndimage import minimum_filter, maximum_filter
        from ur7e_perception_planning_core.grid import observed_free
        res = self.c['grid_resolution'];cfg = self.tabletop
        # Robot endpoint filtering must not discard measured empty space before
        # a return. Unfiltered depth contributes rays only, never top inference
        # or TSDF surfaces; the same truncation/noise allowance still applies.
        ray_depth = depth if ray_depth is None else ray_depth
        self.rays |= observed_free(self.points, ray_depth, pose, intr, .05+np.sqrt(3)*res/2)
        self._observe_tops(depth, pose, intr)
        counts = np.sum(np.asarray(self.coverage) == 15, axis=0) if self.coverage else np.zeros(self.height.shape)
        self.reliable = minimum_filter((counts >= cfg['min_distinct_views']).astype(np.uint8), size=3, mode='constant', cval=0).astype(bool)
        self.upper_height = maximum_filter(self.height, size=3, mode='constant', cval=np.inf)+cfg['height_margin_m']
        heights = self.upper_height.reshape(-1)
        z = self.points[:,2].reshape(-1, self.dims[0]*self.dims[1])
        inferred = ((z > heights) & self.reliable.reshape(-1) & (z <= cfg['z_max']) & (z >= cfg['z_min'])).reshape(-1)
        self.source[:] = 0;self.source[inferred] = 2;self.source[self.rays] = 1
        # Recompute distances from the allowed free set; old corridor distances
        # and their influence on neighboring observed cells are never reused.
        free = self.source > 0
        free[known & (raw <= .02)] = False
        self.recomputed = self.cached_free is None or not np.array_equal(free, self.cached_free)
        if self.recomputed:
            volume = free.reshape(tuple(self.dims[::-1]));distance = np.zeros(volume.shape)
            if np.any(volume):
                bounds = []
                for axis in range(3):
                    ids = np.flatnonzero(np.any(volume, axis=tuple(i for i in range(3) if i != axis)))
                    bounds.append(slice(ids[0], ids[-1]+1))
                region = tuple(bounds)
                distance[region] = self.edt(np.pad(volume[region], 1))[1:-1,1:-1,1:-1]*res
            self.cached_distance = distance.reshape(-1);self.cached_free = free.copy()
        d = np.minimum(self.cached_distance-np.sqrt(3)*res/2, np.where(known, raw, np.inf))
        d[~free] = np.minimum(d[~free], 0.)
        source = self.source.copy();source[~free] = 0
        return d.astype(np.float32), source

    def evidence(self):
        return dict(height=self.height.copy(), upper_height=self.upper_height.copy(),
                    reliable=self.reliable.copy(),
                    view_count=np.sum(np.asarray(self.coverage)==15,axis=0) if self.coverage else np.zeros(self.height.shape,dtype=int))
