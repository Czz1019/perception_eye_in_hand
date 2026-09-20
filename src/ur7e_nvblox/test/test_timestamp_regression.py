"""Regressions for stale DDS frames following a simulation clock reset."""
import threading
from types import SimpleNamespace as NS

from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.stats import RuntimeStats
from ur7e_nvblox.node import TfMapper


def test_pre_reset_image_does_not_poison_new_epoch():
    node = NS(
        stats=RuntimeStats(), info=object(), cfg={'max_frame_age_sec': 0.5},
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
        _state_lock=threading.Lock(), _epoch=2, _last_input_ns=-1, frames=LatestSlot(),
    )
    def image(sec, ns=0):
        return NS(header=NS(stamp=NS(sec=sec, nanosec=ns)))
    # A queued frame from t=600 s arrives after /clock reset to t=1 s.
    TfMapper._on_depth(node, image(600))
    assert node._last_input_ns == -1
    assert node.frames.latest()[1] is None
    # Current-time frames must still be accepted immediately.
    TfMapper._on_depth(node, image(1))
    assert node.frames.latest()[1][3] == 1_000_000_000
    # No zero timestamp (TF would interpret it as latest), duplicate, or older image.
    TfMapper._on_depth(node, image(0))
    TfMapper._on_depth(node, image(1))
    TfMapper._on_depth(node, image(0, 900_000_000))
    assert node.frames.latest()[0] == 1
