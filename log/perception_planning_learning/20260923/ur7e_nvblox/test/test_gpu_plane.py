"""A real CUDA/backend geometry test, enabled explicitly with RUN_GPU_TESTS=1."""
import os

import numpy as np
import pytest

from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.types import CameraIntrinsics
from ur7e_nvblox.depth import depth_calibration, known_distances


@pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS') != '1', reason='Explicit GPU test')
def test_plane_esdf_in_translated_base_frame():
    intr = CameraIntrinsics(160, 120, 100, 100, 79.5, 59.5)
    backend = NvbloxBackend(depth_calibration(intr), {'voxel_size_m': 0.025})
    pose = np.eye(4, dtype=np.float32)
    pose[:3, 3] = [0.2, 0.1, 0.3]
    for _ in range(5):
        backend.integrate_depth(np.ones((120, 160), dtype=np.float32), pose)
    backend.update_esdf()
    # A plane one meter ahead of the camera is z=1.3 in base coordinates.
    points = np.array([[0.2, 0.1, 1.1], [0.2, 0.1, 1.2], [0.2, 0.1, 1.3], [9, 9, 9]], dtype=np.float32)
    d = backend.query_esdf(points)
    assert np.all(known_distances(d[:3], backend.esdf_unknown_distance)), d
    np.testing.assert_allclose(d[:3], [0.2, 0.1, 0.0], atol=0.06)
    assert not known_distances(d[3:], backend.esdf_unknown_distance)[0], d
