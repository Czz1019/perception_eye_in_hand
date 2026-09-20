import os
from unittest.mock import patch

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.types import CameraIntrinsics
from ur7e_reconstruction.surface import SurfaceMapper, crop_surface, save_ply, surface_cloud, surface_markers


def sample():
    return crop_surface([[0, 0, 1], [1, 0, 1], [0, 1, 1], [9, 9, 9]],
                        [[0, 1, 2], [0, 2, 3]], [-2, -2, 0], [2, 2, 2])


def test_roi_preserves_connectivity_and_removes_outside_faces():
    surface = sample()
    assert surface.vertices.shape == (3, 3)
    np.testing.assert_array_equal(surface.triangles, [[0, 1, 2]])
    assert surface.colors.shape == (3, 3)


def test_empty_snapshot_clears_old_rviz_triangles():
    surface = sample()
    markers = surface_markers(surface, 'base_link', 123)
    assert markers.markers[0].action == Marker.DELETEALL
    assert len(markers.markers[1].points) == 3
    assert all(c.a == 1 for c in markers.markers[1].colors)
    empty = crop_surface(surface.vertices, surface.triangles, [8, 8, 8], [9, 9, 9])
    markers = surface_markers(empty, 'base_link', 124)
    assert len(markers.markers) == 1 and markers.markers[0].action == Marker.DELETEALL
    assert surface_cloud(empty, 'base_link', 124).width == 0


def test_ply_and_cloud_preserve_metric_vertices(tmp_path):
    surface = sample()
    cloud = surface_cloud(surface, 'base_link', 123)
    np.testing.assert_array_equal(np.frombuffer(cloud.data, '<f4').reshape(-1, 4)[:, :3], surface.vertices)
    path = save_ply(surface, tmp_path / 'scene.ply')
    prefix, payload = path.read_bytes().split(b'end_header\n', 1)
    assert b'element vertex 3' in prefix and b'element face 1' in prefix
    assert len(payload) == 3*15 + 13
    vertices = np.frombuffer(payload[:45], dtype=[('p', '<f4', (3,)), ('c', 'u1', (3,))])
    np.testing.assert_array_equal(vertices['p'], surface.vertices)
    np.testing.assert_array_equal(np.frombuffer(payload[46:], '<i4'), [0, 1, 2])


@pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS') != '1', reason='Explicit CUDA test')
def test_real_gpu_plane_reconstruction_without_esdf():
    def forbidden(*args, **kwargs):
        raise AssertionError('Surface reconstruction must never update or query ESDF')
    with patch.object(NvbloxBackend, 'update_esdf', forbidden), \
         patch.object(NvbloxBackend, 'query_esdf', forbidden), \
         patch.object(NvbloxBackend, 'query_esdf_grid', forbidden):
        mapper = SurfaceMapper(CameraIntrinsics(160, 120, 100, 100, 79.5, 59.5), {
            'voxel_size_m': 0.025, 'max_depth_m': 3.0,
            'roi_min_m': [-0.2, -0.2, 0.8], 'roi_max_m': [0.5, 0.4, 1.5],
        })
        pose = np.eye(4, dtype=np.float32)
        pose[:3, 3] = [0.2, 0.1, 0.3]
        for _ in range(5):
            mapper.integrate(np.ones((120, 160), dtype=np.float32), pose)
        surface = mapper.extract()
        assert len(surface.triangles) > 100
        assert np.isfinite(surface.vertices).all()
        # Measured plane at depth=1 m is at base z=1.3 m, not camera z=1 m.
        np.testing.assert_allclose(surface.vertices[:, 2], 1.3, atol=0.04)


def test_crop_keeps_actual_vertex_colors():
    rgb = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [1, 2, 3]], np.uint8)
    surface = crop_surface([[0, 0, 1], [1, 0, 1], [0, 1, 1], [9, 9, 9]],
                           [[0, 1, 2], [0, 2, 3]], [-2, -2, 0], [2, 2, 2], rgb)
    np.testing.assert_array_equal(surface.colors, rgb[:3])


def test_rgb_padded_rows_bgr_conversion():
    from sensor_msgs.msg import Image
    from ur7e_reconstruction.color import rgb_image
    msg = Image(width=2, height=2, encoding='bgr8', step=8,
                data=bytes([0, 0, 255, 0, 255, 0, 99, 99, 255, 0, 0, 255, 255, 255, 99, 99]))
    np.testing.assert_array_equal(rgb_image(msg), [[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 255]]])


@pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS') != '1', reason='Explicit CUDA test')
def test_real_gpu_rgb_integration_with_separate_camera_origin():
    intr = CameraIntrinsics(160, 120, 100, 100, 79.5, 59.5)
    mapper = SurfaceMapper(intr, {'voxel_size_m': .02, 'max_depth_m': 2.,
                           'roi_min_m': [-.3, -.3, .8], 'roi_max_m': [.3, .3, 1.2]}, intr)
    depth_pose = np.eye(4, dtype=np.float32)
    color_pose = depth_pose.copy()
    color_pose[0, 3] = -.1
    rgb = np.zeros((120, 160, 3), np.uint8)
    rgb[:, :80] = [230, 20, 35]
    rgb[:, 80:] = [20, 35, 230]
    with patch.object(NvbloxBackend, 'update_esdf', side_effect=AssertionError('ESDF forbidden')):
        for _ in range(8):
            mapper.integrate(np.ones((120, 160), np.float32), depth_pose, rgb, color_pose)
        surface = mapper.extract()
    assert len(surface.vertices) > 100
    # The RGB boundary is at base x=-0.1, not at the depth-camera center x=0.
    red = surface.colors[surface.vertices[:, 0] < -.18]
    blue = surface.colors[surface.vertices[:, 0] > -.02]
    np.testing.assert_allclose(np.median(red, axis=0), [230, 20, 35], atol=3)
    np.testing.assert_allclose(np.median(blue, axis=0), [20, 35, 230], atol=3)
