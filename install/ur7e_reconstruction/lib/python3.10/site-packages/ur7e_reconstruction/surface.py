"""TSDF surface extraction, visualization and portable mesh export."""
from dataclasses import dataclass, replace
import os
from pathlib import Path
import tempfile

import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.ros_publishers import header
from ur7e_nvblox.depth import depth_calibration


@dataclass
class Surface:
    vertices: np.ndarray
    triangles: np.ndarray
    colors: np.ndarray


def crop_surface(vertices, triangles, minimum, maximum, colors=None):
    """Keep complete triangles in the requested base-frame ROI and reindex vertices."""
    v = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)
    t = np.asarray(triangles, dtype=np.int64).reshape(-1, 3)
    if len(t) and (t.min() < 0 or t.max() >= len(v)):
        raise ValueError('Invalid mesh triangle indices')
    valid = np.isfinite(v).all(axis=1) & (v >= minimum).all(axis=1) & (v <= maximum).all(axis=1)
    t = t[valid[t].all(axis=1)]
    used, inverse = np.unique(t.reshape(-1), return_inverse=True)
    v, t = v[used], inverse.reshape(-1, 3).astype(np.int32)
    if colors is None:
        colors = np.full((len(vertices), 3), 180, dtype=np.uint8)
    colors = np.asarray(colors).reshape(-1, 3)
    if len(colors) != len(vertices):
        raise ValueError('Mesh colors must match vertices')
    if np.issubdtype(colors.dtype, np.floating):
        colors = np.rint(np.clip(colors, 0, 1) * 255)
    return Surface(v, t, colors[used].astype(np.uint8))


class ColorBackend(NvbloxBackend):
    """Configure a fresh native mapper, retaining the original backend's public API."""
    def _initialize(self):
        super()._initialize()
        from nvblox_torch.mapper import Mapper
        from nvblox_torch.mapper_params import MapperParams
        from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
        params = MapperParams()
        projective = params.get_projective_integrator_params()
        projective.projective_integrator_truncation_distance_vox = 2.0
        projective.projective_integrator_max_integration_distance_m = self.cfg['max_integration_distance_m']
        projective.projective_integrator_max_weight = 20.0
        params.set_projective_integrator_params(projective)
        mesh = params.get_mesh_integrator_params()
        mesh.mesh_integrator_min_weight = 1.0
        params.set_mesh_integrator_params(mesh)
        self.mapper = Mapper(voxel_sizes_m=self.cfg['voxel_size_m'],
                             integrator_types=ProjectiveIntegratorType.TSDF, mapper_parameters=params)


class SurfaceMapper:
    """Fuse RGB-D and extract the measured colored surface without any ESDF updates."""
    def __init__(self, intrinsics, config, color_intrinsics=None):
        calibration = depth_calibration(intrinsics)
        if color_intrinsics is not None:
            calibration = replace(calibration, color_intrinsics=color_intrinsics)
        self.backend = ColorBackend(calibration, {
            'voxel_size_m': config['voxel_size_m'],
            'max_integration_distance_m': config['max_depth_m'],
        })
        self.minimum = np.asarray(config['roi_min_m'], dtype=np.float32)
        self.maximum = np.asarray(config['roi_max_m'], dtype=np.float32)

    def integrate(self, depth, camera_to_base, color=None, color_to_base=None):
        self.backend.integrate_depth(depth, camera_to_base)
        if color is not None:
            self.backend.integrate_color(color, color_to_base)

    def extract(self):
        self.backend.update_mesh()
        vertices, triangles, colors = self.backend.mesh_numpy()
        return crop_surface(vertices, triangles, self.minimum, self.maximum, colors)


def surface_cloud(surface, frame_id, stamp_ns):
    """Fused mesh vertices, not an instantaneous depth-camera cloud."""
    colors = surface.colors.astype(np.uint32)
    rgb = ((colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]).view(np.float32)
    data = np.column_stack((surface.vertices, rgb)).astype('<f4')
    msg = PointCloud2()
    msg.header = header(frame_id, stamp_ns)
    msg.height, msg.width = 1, len(data)
    msg.is_bigendian, msg.is_dense = False, True
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.point_step, msg.row_step = 16, 16*len(data)
    msg.data = data.tobytes()
    return msg


def surface_markers(surface, frame_id, stamp_ns, chunk_triangles=4000):
    """A complete snapshot; DELETEALL removes old chunks on reset or shrinking meshes."""
    delete = Marker()
    delete.action = Marker.DELETEALL
    delete.header = header(frame_id, stamp_ns)
    markers = [delete]
    vertices, triangles, colors = surface.vertices, surface.triangles, surface.colors
    # Vertex/color ROS objects are shared while constructing chunks, avoiding per-face copies.
    points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in vertices]
    rgba = [ColorRGBA(r=float(c[0])/255, g=float(c[1])/255, b=float(c[2])/255, a=1.0) for c in colors]
    for start in range(0, len(triangles), chunk_triangles):
        marker = Marker()
        marker.header = header(frame_id, stamp_ns)
        marker.ns, marker.id = 'tsdf_surface', start // chunk_triangles
        marker.type, marker.action = Marker.TRIANGLE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.a = 1.0
        indices = triangles[start:start+chunk_triangles].reshape(-1)
        marker.points = [points[int(i)] for i in indices]
        marker.colors = [rgba[int(i)] for i in indices]
        markers.append(marker)
    return MarkerArray(markers=markers)


def save_ply(surface, path):
    """Atomic binary PLY export, preserving all displayed vertices and triangles."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices = np.empty(len(surface.vertices), dtype=[('p', '<f4', (3,)), ('c', 'u1', (3,))])
    vertices['p'], vertices['c'] = surface.vertices, surface.colors
    faces = np.empty(len(surface.triangles), dtype=[('n', 'u1'), ('i', '<i4', (3,))])
    faces['n'], faces['i'] = 3, surface.triangles
    text = (f'ply\nformat binary_little_endian 1.0\nelement vertex {len(vertices)}\n'
            'property float x\nproperty float y\nproperty float z\n'
            'property uchar red\nproperty uchar green\nproperty uchar blue\n'
            f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n')
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
            temporary = stream.name
            stream.write(text.encode('ascii'))
            stream.write(vertices.tobytes())
            stream.write(faces.tobytes())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return path
