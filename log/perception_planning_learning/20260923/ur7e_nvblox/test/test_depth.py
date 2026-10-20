from types import SimpleNamespace as NS

import numpy as np
import pytest

from ur7e_nvblox.depth import depth_meters, intrinsics_from_info, transform_matrix


def test_millimeters_endianness_and_padding():
    pixels = np.array([[1000, 2000, 65000], [0, 3000, 65000]], dtype='>u2')
    msg = NS(width=2, height=2, step=6, data=pixels.tobytes(), encoding='16UC1', is_bigendian=True)
    np.testing.assert_allclose(depth_meters(msg), [[1, 2], [0, 3]])


def test_float_invalid_depth_and_decimation():
    pixels = np.array([[1, 9, np.nan], [2, 3, 4], [np.inf, 4, -1]], dtype='<f4')
    msg = NS(width=3, height=3, step=12, data=pixels.tobytes(), encoding='32FC1', is_bigendian=False)
    np.testing.assert_allclose(depth_meters(msg, stride=2), [[1, 0], [0, 0]])
    msg.step = 8
    with pytest.raises(ValueError, match='row step'):
        depth_meters(msg)


def test_decimated_camera_rays_are_unchanged():
    info = NS(width=5, height=3, p=[10, 0, 2, 0, 0, 12, 1, 0, 0, 0, 1, 0], k=[0]*9)
    intr = intrinsics_from_info(info, stride=2)
    assert (intr.width, intr.height) == (3, 2)
    # Decimation selects original pixel (4,2) as new pixel (2,1).
    assert (2-intr.cx)/intr.fx == (4-2)/10
    assert (1-intr.cy)/intr.fy == (2-1)/12


def test_camera_to_base_transform_direction():
    tf = NS(translation=NS(x=1, y=2, z=3), rotation=NS(x=0, y=0, z=np.sqrt(0.5), w=np.sqrt(0.5)))
    np.testing.assert_allclose(transform_matrix(tf) @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-6)
