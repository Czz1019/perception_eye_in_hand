import threading
import time
from types import SimpleNamespace as NS

import numpy as np

from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.stats import RuntimeStats
from ur7e_nvblox.node import TfMapper


def test_same_map_not_republished_and_reset_discards_old_epoch():
    messages = []
    node = NS(
        results=LatestSlot(), _last_output_sequence=0, _epoch=0,
        _state_lock=threading.Lock(), _summary={}, _published_versions={},
        cfg={'max_frame_age_sec': .5, 'max_visualized_distance_m': .15},
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_010_000_000)),
        observed_pub=NS(get_subscription_count=lambda: 1, publish=messages.append),
        cloud_pub=NS(get_subscription_count=lambda: 0), stats=RuntimeStats(), world='base_link',
    )
    summary = dict(full_grid_available=False, map_version=1, unknown_distance_m=1e6)
    result = (0, 2_000_000_000, np.eye(4), np.zeros((3, 3), np.float32),
              np.array([.1, 1e6, .3], np.float32), summary, time.perf_counter())
    node.results.put(result)
    TfMapper._publish_latest(node)
    assert len(messages) == 1 and messages[0].width == 1
    assert messages[0].header.stamp.sec == 2 and messages[0].header.stamp.nanosec == 0
    node.results.put(result)  # A consumer changes demand; the old map is still the same version.
    TfMapper._publish_latest(node)
    assert len(messages) == 1
    node._epoch = 1
    node.results.put(result)
    TfMapper._publish_latest(node)
    assert len(messages) == 1
    assert node.stats.snapshot()['totals']['observed_cloud_published'] == 1


def test_legacy_esdf_3d_does_not_publish_unknown_or_free_padding():
    """Sparse-v2 compatibility contract; added here but not run in this revision."""
    messages = []
    node = NS(
        results=LatestSlot(), _last_output_sequence=0, _epoch=0,
        _state_lock=threading.Lock(), _summary={}, _published_versions={},
        cfg={'max_frame_age_sec': .5, 'max_visualized_distance_m': .15},
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_010_000_000)),
        observed_pub=NS(get_subscription_count=lambda: 0),
        cloud_pub=NS(get_subscription_count=lambda: 1, publish=messages.append),
        stats=RuntimeStats(), world='base_link',
    )
    summary = dict(full_grid_available=True, map_version=1, unknown_distance_m=1e6)
    node.results.put((0, 2_000_000_000, np.eye(4),
                      np.zeros((4, 3), np.float32),
                      np.array([-.02, 1e6, .1, np.nan], np.float32),
                      summary, time.perf_counter()))
    TfMapper._publish_latest(node)
    assert len(messages) == 1 and messages[0].width == 1
    field = next(f for f in messages[0].fields if f.name == 'distance')
    distance = np.ndarray((1,), '<f4', buffer=messages[0].data,
                          offset=field.offset, strides=(messages[0].point_step,))
    np.testing.assert_allclose(distance, [-.02], atol=1e-6)
