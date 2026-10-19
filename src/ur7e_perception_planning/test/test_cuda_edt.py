import numpy as np
import pytest
import torch
from scipy.ndimage import distance_transform_edt
from ur7e_perception_planning_core.edt import CudaEDT


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA needed')
def test_exact_cuda_edt_matches_scipy_obstacles_and_boundaries():
    edt=CudaEDT();rng=np.random.default_rng(19)
    for shape in [(3,5,7),(20,23,17),(66,71,76)]:
        for fraction in [.0,.1,.9,1.]:
            volume=np.pad(rng.random(shape)<fraction,1)
            np.testing.assert_array_equal(edt(volume),distance_transform_edt(volume))
    volume=np.zeros((260,3,4),bool);volume[10:200,1,2]=True
    np.testing.assert_array_equal(edt(volume),distance_transform_edt(volume))
    assert edt.last_backend=='scipy_fallback'
