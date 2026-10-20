from types import SimpleNamespace
import importlib
import subprocess
import sys
import threading
from unittest.mock import Mock

import numpy as np
import pytest

from realtime_nvblox.gazebo_depth import depth_meters, intrinsics_from_info, calibration_from_info, stamp_rejection


def info():
    return SimpleNamespace(width=5, height=3, p=[100., 0, 2., 0, 0, 90., 1., 0, 0, 0, 1., 0],
                           k=[80., 0, 2., 0, 70., 1., 0, 0, 1.])


@pytest.mark.parametrize('encoding,kind,scale', [('16UC1', 'u2', .001), ('32FC1', 'f4', 1.)])
@pytest.mark.parametrize('big', [False, True])
def test_padding_endianness_units_stride(encoding, kind, scale, big):
    values = np.array([[0, 1, 2, 3, 5], [.1, .2, .3, .4, .5], [1, 2, 3, 4, 5]])
    dtype = np.dtype(('>' if big else '<')+kind)
    rows = (values/scale).astype(dtype)
    payload = b''.join(row.tobytes()+b'padding!' for row in rows)
    msg = SimpleNamespace(width=5, height=3, encoding=encoding, step=5*dtype.itemsize+8, is_bigendian=big, data=payload)
    got = depth_meters(msg, 2, .2, 4.)
    np.testing.assert_allclose(got, [[0, 2, 0], [1, 3, 0]])
    assert got.flags.c_contiguous and got.dtype == np.float32


def test_invalid_depth_and_buffer():
    values = np.array([[np.nan, np.inf, -1, 0, 1]], dtype='<f4')
    msg = SimpleNamespace(width=5, height=1, encoding='32FC1', step=20, is_bigendian=False, data=values.tobytes())
    np.testing.assert_equal(depth_meters(msg), [[0, 0, 0, 0, 1]])
    msg.step = 19
    with pytest.raises(ValueError):
        depth_meters(msg)


def test_rectified_p_fallback_k_and_scaled_pixel_geometry():
    camera = info()
    intr = intrinsics_from_info(camera, 2)
    assert (intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy) == (3, 2, 50, 45, 1, .5)
    assert (2-intr.cx)/intr.fx == (4-camera.p[2])/camera.p[0]
    camera.p = [0.]*12
    assert intrinsics_from_info(camera).fx == 80
    assert calibration_from_info(camera).depth_scale_m == 1.


def test_sim_time_epoch_bounds():
    assert stamp_rejection(0, 10, 0, 0, 5) == 'invalid_stamp'
    assert stamp_rejection(20, 10, 0, 0, 5) == 'future_depth'
    assert stamp_rejection(2, 10, 0, 0, 5) == 'stale_depth'
    assert stamp_rejection(8, 10, 8, 0, 5) == 'out_of_order'
    assert stamp_rejection(8, 10, 0, 8, 5) == 'out_of_order'
    assert stamp_rejection(9, 10, 0, 8, 5) is None


def test_gazebo_import_does_not_import_hardware_or_slam():
    result = subprocess.run([sys.executable, '-c',
        "import sys; import realtime_nvblox.gazebo_ros_node; "
        "assert not any(n in sys.modules for n in ('pyrealsense2','cuvslam','realtime_nvblox.realsense_source','realtime_nvblox.runtime','realtime_nvblox.robot_pose'))"],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_lookup_interpolates_only_requested_stamp_and_rejects_latest():
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformException
    from realtime_nvblox.gazebo_ros_node import GazeboNvbloxNode
    buffer = Buffer()
    for sec, x in [(1, 0.), (2, 1.)]:
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_depth_optical_frame'
        t.header.stamp.sec = sec
        t.transform.translation.x = x
        t.transform.rotation.w = 1.
        buffer.set_transform(t, 'test')
    adapter = SimpleNamespace(tf_buffer=buffer, cfg={'world_frame': 'base_link', 'tf_timeout_sec': 0.})
    header = SimpleNamespace(frame_id='camera_depth_optical_frame', stamp=SimpleNamespace(sec=1, nanosec=500000000))
    # Time.from_msg requires the ROS message type.
    from builtin_interfaces.msg import Time
    header.stamp = Time(sec=1, nanosec=500000000)
    item = SimpleNamespace(msg=SimpleNamespace(header=header))
    assert GazeboNvbloxNode._lookup(adapter, item).transform.translation.x == .5
    header.stamp = Time(sec=3)
    with pytest.raises(TransformException):
        GazeboNvbloxNode._lookup(adapter, item)


def test_reset_clears_queues_and_rejects_inflight_epoch():
    from realtime_nvblox.gazebo_ros_node import GazeboNvbloxNode
    from realtime_nvblox.latest_slot import LatestSlot
    from realtime_nvblox.stats import RuntimeStats
    adapter = SimpleNamespace(sim_time=True, epoch=2, now_ns=100, floor_ns=0, last_input_ns=90,
                              depth_slot=LatestSlot(), result_slot=LatestSlot(), extra={}, stats=RuntimeStats(),
                              get_logger=lambda: Mock(), state=threading.RLock())
    adapter.depth_slot.put('old depth'); adapter.result_slot.put('old result')
    GazeboNvbloxNode._reset_locked(adapter, 'test')
    assert adapter.epoch == 3 and adapter.floor_ns == 100
    assert adapter.depth_slot.latest()[1] is None and adapter.result_slot.latest()[1] is None
    assert not GazeboNvbloxNode._input_valid(adapter, SimpleNamespace(epoch=2))
