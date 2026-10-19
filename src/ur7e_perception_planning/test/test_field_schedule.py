import numpy as np
from ur7e_perception_planning_core.grid import grid_points,conservative_field


def test_deferred_growth_is_never_more_optimistic_and_blocking_is_immediate():
    _,dims,p=grid_points([0,0,.5],[.5,.5,.5],.025)
    source=np.zeros(tuple(dims[::-1]),np.uint8);source[5:12,5:12,5:12]=1;source=source.reshape(-1)
    raw=np.full(len(p),100.);known=np.zeros(len(p),bool);cache={}
    first,_=conservative_field(raw,known,source,p,dims,.025,cache,min_interval=100.)
    source.reshape(tuple(dims[::-1]))[3:17,3:17,3:17]=1
    reused,_=conservative_field(raw,known,source,p,dims,.025,cache,min_interval=100.)
    exact,_=conservative_field(raw,known,source,p,dims,.025)
    assert cache['deferred'] and not cache['recomputed']
    assert np.all(reused<=exact+1e-7)
    # Fresh measured obstruction inside previously free space MUST bypass the interval.
    index=np.ravel_multi_index((8,8,8),tuple(dims[::-1]));raw[index]=-.03;known[index]=True
    updated,_=conservative_field(raw,known,source,p,dims,.025,cache,min_interval=100.)
    exact,_=conservative_field(raw,known,source,p,dims,.025)
    assert cache['recomputed'] and not cache['deferred']
    np.testing.assert_allclose(updated,exact,atol=1e-7)
    assert updated[index]<0
    # Source loss/reset is handled just as promptly, even with no native observation.
    source[:]=0
    blocked,_=conservative_field(raw,known,source,p,dims,.025,cache,min_interval=100.)
    assert cache['recomputed'] and np.all(blocked<=0)


def test_deferred_free_growth_refreshes_after_deadline():
    _,dims,p=grid_points([0,0,.5],[.4,.4,.4],.02)
    raw=np.full(len(p),100.);known=np.zeros(len(p),bool);source=np.zeros(len(p),np.uint8);cache={}
    conservative_field(raw,known,source,p,dims,.02,cache,min_interval=.2)
    source[:]=1;cache['updated_at']-=1.
    updated,_=conservative_field(raw,known,source,p,dims,.02,cache,min_interval=.2)
    exact,_=conservative_field(raw,known,source,p,dims,.02)
    assert cache['recomputed'];np.testing.assert_array_equal(updated,exact)
