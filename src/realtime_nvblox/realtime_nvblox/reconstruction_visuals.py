"""Additional visualizations; existing ESDF topics keep their original semantics."""
import numpy as np
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from realtime_nvblox.ros_publishers import esdf_cloud, header, mesh_markers

# Inside, near surface, close, intermediate, far (within displayed band).
COLORS = (0xD020FF, 0xFF2020, 0xFF8000, 0xFFFF00, 0x2080FF)


def clearance_cloud(points, distances, frame, stamp, band, unknown, thresholds):
    cloud = esdf_cloud(points, distances, frame, stamp, band, False, unknown)
    packed = np.frombuffer(cloud.data, dtype=np.float32).reshape(-1, 5).copy()
    d = packed[:, 4]
    rgb = np.full(len(d), COLORS[4], dtype=np.uint32)
    for limit, color in reversed(list(zip(thresholds, COLORS[1:4]))):
        rgb[d <= limit] = color
    rgb[d < 0] = COLORS[0]
    packed[:, 3] = rgb.view(np.float32)
    cloud.data = packed.tobytes()
    return cloud


def clear_mesh(frame, stamp, first_id=0, previous_count=0):
    result = MarkerArray()
    for index in range(first_id, previous_count):
        marker = Marker()
        marker.header = header(frame, stamp)
        marker.ns, marker.id = 'realtime_nvblox_mesh', index
        marker.action = Marker.DELETE
        marker.pose.orientation.w = 1.
        result.markers.append(marker)
    return result


def reconstruction_mesh(vertices, triangles, frame, stamp, previous_count=0):
    vertices = np.asarray(vertices)
    triangles = np.asarray(triangles)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or triangles.ndim != 2 or triangles.shape[1] != 3:
        raise ValueError('Mesh requires Nx3 vertices and Mx3 triangle indices')
    if not np.issubdtype(triangles.dtype, np.integer):
        raise ValueError('Mesh triangle indices must be integers')
    valid = np.all((triangles >= 0) & (triangles < len(vertices)), axis=1)
    triangles = triangles[valid]
    xyz = vertices[triangles]
    finite = np.isfinite(xyz).all(axis=(1, 2))
    triangles, xyz = triangles[finite], xyz[finite]
    area2 = np.linalg.norm(np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0]), axis=1)
    triangles = triangles[np.isfinite(area2) & (area2 > 1e-12)]
    # Preserve all valid faces, but use smaller renderer objects and stable IDs.
    result = mesh_markers(vertices, triangles, np.empty((0, 3)), frame, stamp,
                          max_triangles=0, chunk_triangles=2000)
    count = len(result.markers)
    for marker in result.markers:
        marker.color = ColorRGBA(r=.72, g=.82, b=.88, a=1.)
    result.markers.extend(clear_mesh(frame, stamp, count, previous_count).markers)
    return result, count, len(triangles)


def distance_legend(frame, stamp, thresholds, band):
    a, b, c = thresholds
    labels = ['d < 0: reconstructed interior', f'0 .. {a:g} m: near surface',
              f'{a:g} .. {b:g} m: close', f'{b:g} .. {c:g} m: intermediate',
              f'{c:g} .. {band:g} m: farther (observed)', 'Unknown: not displayed; not confirmed free']
    result = MarkerArray()
    for index, (label, rgb) in enumerate(zip(labels, (*COLORS, 0xFFFFFF))):
        marker = Marker()
        marker.header = header(frame, stamp)
        marker.ns, marker.id = 'esdf_distance_legend', index
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y = -.65, -.65
        marker.pose.position.z = 1.35-index*.07
        marker.scale.z = .035
        marker.color = ColorRGBA(r=((rgb >> 16)&255)/255., g=((rgb >> 8)&255)/255., b=(rgb&255)/255., a=1.)
        marker.text = label
        result.markers.append(marker)
    return result
