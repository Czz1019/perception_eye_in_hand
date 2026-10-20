import numpy as np
from ur7e_gazebo_esdf_sage.stationary import result_rejection


def samples():
    return [(np.zeros(6),np.zeros(6),float(t),float(100+t)) for t in np.arange(.95,3.011,.01)]


def check(states,now=3.):
    c=dict(stationary_processing_timeout=5.,joint_timeout=.6,settle_speed=.015)
    return result_rejection(c,states,np.zeros(6),1.,101.,now,100+now)


def test_slow_static_result_needs_continuous_fresh_stationary_feedback():
    assert check(samples()) is None
    assert check(samples(),now=7.)=='PROCESSING_TIMEOUT'
    assert check(samples()[:-100])=='STALE_FEEDBACK'
    s=samples();del s[40:70]
    assert check(s)=='INCOMPLETE_STATIONARY_HISTORY'
    assert check(samples()[20:])=='INCOMPLETE_STATIONARY_HISTORY'
    s=samples();s[80]=(np.full(6,.02),np.zeros(6),s[80][2],s[80][3])
    assert check(s)=='MOVED_DURING_PROCESSING'
    s=samples();s[80]=(np.zeros(6),np.full(6,.1),s[80][2],s[80][3])
    assert check(s)=='MOVED_DURING_PROCESSING'
