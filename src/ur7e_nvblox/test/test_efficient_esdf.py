import os
import numpy as np
import pytest
from realtime_nvblox.types import CameraIntrinsics
from ur7e_nvblox.depth import depth_calibration,known_distances
from ur7e_nvblox.esdf import EfficientBackend


@pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS')!='1',reason='Explicit CUDA test')
def test_prepared_and_sparse_are_native_esdf_with_owned_outputs():
    b=EfficientBackend(depth_calibration(CameraIntrinsics(80,60,70,70,40,30)),{'voxel_size_m':.05})
    depth=np.ones((60,80),np.float32);pose=np.eye(4)
    for _ in range(3):b.integrate_depth(depth,pose)
    b.update_esdf()
    p=np.array([[0,0,.8],[0,0,1.],[9,9,9]],np.float32)
    b.prepare_query(p);first=b.query_prepared();original=first.copy()
    np.testing.assert_allclose(first,b.query_esdf(p),atol=1e-6)
    assert not known_distances(first[-1:],b.esdf_unknown_distance)[0]
    b.query_prepared();np.testing.assert_array_equal(first,original)
    points,dist=b.observed_esdf_numpy()
    assert len(points)>100 and known_distances(dist,b.esdf_unknown_distance).all()
    np.testing.assert_allclose(dist,b.query_esdf(points),atol=1e-6)
    # Native voxel-centre positions, never TSDF truncation values masquerading as ESDF.
    np.testing.assert_allclose(points/.05-.5,np.round(points/.05-.5),atol=1e-5)
    assert np.any(dist>.15) and np.any(abs(dist)<.051)
    # Reusing a query with a new, empty map cannot expose the previous results.
    empty=EfficientBackend(depth_calibration(CameraIntrinsics(80,60,70,70,40,30)),{'voxel_size_m':.05})
    empty.prepare_query(p);empty.output_gpu.fill_(.3)
    assert not known_distances(empty.query_prepared(),empty.esdf_unknown_distance).any()


@pytest.mark.skipif(os.environ.get('RUN_GPU_TESTS')!='1',reason='Explicit CUDA test')
def test_gpu_filtered_export_matches_native_queries_and_never_leaks_unknowns():
    from ur7e_nvblox.sampling import native_grid_points
    b = EfficientBackend(depth_calibration(CameraIntrinsics(80,60,70,70,40,30)), {'voxel_size_m': .05})
    for _ in range(3):
        b.integrate_depth(np.ones((60,80),np.float32), np.eye(4))
    b.update_esdf()
    grid = native_grid_points([0, 0, .75], [2, 2, 1.5], .05)
    b.prepare_query(grid)
    full = b.query_prepared()
    keep = known_distances(full, b.esdf_unknown_distance) & (np.abs(full) <= .15)
    points, distances = b.query_prepared_observed(.15)
    assert len(points) > 100
    np.testing.assert_array_equal(points, grid[keep])
    np.testing.assert_allclose(distances, full[keep], atol=1e-6)
    saved = distances.copy()
    b.query_prepared_observed(.05)
    np.testing.assert_array_equal(distances, saved)
    # Unknown values must not appear even when the requested distance band is huge.
    empty = EfficientBackend(depth_calibration(CameraIntrinsics(80,60,70,70,40,30)), {'voxel_size_m': .05})
    empty.prepare_query(grid)
    points, distances = empty.query_prepared_observed(2e6)
    assert points.shape == (0, 3) and distances.shape == (0,)
