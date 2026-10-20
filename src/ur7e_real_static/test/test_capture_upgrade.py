import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from scipy.ndimage import distance_transform_edt
from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.stats import RuntimeStats
from ur7e_real_static.field import FieldBuilder
from ur7e_real_static.node import RealStatic
from ur7e_real_static.config import identity
from fixtures import config


def test_cpu_cuda_fields_match_full_reference_without_simulation_floor(tmp_path):
    c,_=config(tmp_path);c.update(grid_center=[0,0,-.1],grid_size=[.4,.4,.4],grid_resolution=.02)
    cpu=FieldBuilder(dict(c,free_space_backend='cpu'));gpu=FieldBuilder(dict(c,free_space_backend='cuda'))
    rng=np.random.default_rng(22);n=len(cpu.points)
    source=(rng.random(tuple(cpu.dims[::-1]))>.01).astype('u1');source[:2]=0;source=source.reshape(-1)
    cpu.source[:]=source;gpu.source[:]=source
    raw=np.full(n,100.);known=np.zeros(n,bool);raw[n//2]=-.05;known[n//2]=True
    intr=NS(width=10,height=10,fx=8.,fy=8.,cx=5.,cy=5.)
    depth=np.zeros((10,10))
    a,sa=cpu.update(raw,known,depth,np.eye(4),intr);b,sb=gpu.update(raw,known,depth,np.eye(4),intr)
    np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(sa,sb)
    free=source>0;free[known&(raw<=.02)]=False
    expected=distance_transform_edt(np.pad(free.reshape(tuple(cpu.dims[::-1])),1))[1:-1,1:-1,1:-1].reshape(-1)*.02-np.sqrt(3)*.02/2
    expected=np.minimum(expected,np.where(known,raw,np.inf));expected[~free]=np.minimum(expected[~free],0)
    np.testing.assert_array_equal(a,expected.astype('f4'))
    assert np.any((cpu.points[:,2]<0)&(a>0))  # no injected Gazebo z=0 plane
    old=b.copy();gpu.update(raw,known,depth,np.eye(4),intr)
    assert not gpu.recomputed;np.testing.assert_array_equal(old,b)
    raw[n//3]=-.02;known[n//3]=True
    changed,_=gpu.update(raw,known,depth,np.eye(4),intr)
    assert changed[n//3]<=-.02+1e-7


def test_capture_rejects_duplicate_and_pre_scan_stamps():
    node=NS(lock=threading.RLock(),scanning=True,camera_info=object(),capture_stats=RuntimeStats(),
            c={'frame_timeout':.5},epoch=4,last_depth_stamp=0.,scan_started=9.8,frames=LatestSlot(),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    def msg(sec,nanosec=0):return NS(header=NS(stamp=NS(sec=sec,nanosec=nanosec)))
    RealStatic.on_depth(node,msg(9,700_000_000));assert node.frames.latest()[1] is None
    RealStatic.on_depth(node,msg(10));sequence,_=node.frames.latest()
    RealStatic.on_depth(node,msg(10));RealStatic.on_depth(node,msg(9,900_000_000))
    assert node.frames.latest()[0]==sequence
    RealStatic.on_depth(node,msg(20));assert node.last_depth_stamp==10.


def test_performance_options_do_not_change_map_identity(tmp_path):
    c,_=config(tmp_path);before=identity(c)
    c.update(free_space_backend='cpu',capture_hz=5.,visualization_hz=.5)
    assert identity(c)==before


def test_visualization_enqueue_never_calls_dds():
    node=NS(epoch=2,visuals=LatestSlot(),cloud_pub=Mock(),surface_pub=Mock())
    p=np.zeros((3,3));d=np.ones(3)
    RealStatic.publish_cloud(node,p,d,1.,raw=d)
    node.cloud_pub.publish.assert_not_called();node.surface_pub.publish.assert_not_called()
    assert node.visuals.latest()[1][1] is p


def test_failed_map_save_does_not_freeze_scan(tmp_path):
    import time
    import pytest
    from unittest.mock import patch
    node=NS(idle_required=lambda:None,scanning=True,latest_field=(np.zeros(8),np.zeros(8,'u1'),np.zeros(3),np.array([2,2,2]),1.,1,time.monotonic(),{}),
            capture_count=1,views=[np.zeros(6)],c={'min_frames':1,'min_views':1,'frame_timeout':.5,'grid_resolution':.1},
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000)),camera_info=object(),info_record=lambda _: {},
            model_id='model',epoch=1,directory=tmp_path,frozen=None)
    with patch('ur7e_real_static.node.FrozenMap.save',side_effect=OSError('full disk')):
        with pytest.raises(OSError):RealStatic.freeze_map(node)
    assert node.scanning and node.frozen is None


def test_capture_initialization_failure_is_reported_and_worker_survives():
    from unittest.mock import patch
    from ur7e_real_static.capture import worker
    node=NS(lock=threading.RLock(),stop_event=threading.Event(),scanning=True,phase='SCANNING')
    calls=[]
    def run(n):
        calls.append(n)
        if len(calls)==1:raise RuntimeError('CUDA initialization failed')
        assert n.phase=='CAPTURE_ERROR' and not n.scanning
        assert n.capture_error=='CUDA initialization failed'
        n.stop_event.set()
    with patch('ur7e_real_static.capture._worker',side_effect=run):worker(node)
    assert len(calls)==2
