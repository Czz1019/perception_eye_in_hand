import numpy as np
from ur7e_gazebo_esdf_sage_continuous.online import update_occupancy

def test_hidden_obstacle_is_retained_until_a_free_ray_clears_it():
    previous=np.array([True,True,False,False]);free=np.array([False,True,True,False]);hit=np.array([False,False,True,False])
    result=update_occupancy(previous,free,hit,(2,2))
    assert np.array_equal(result.ravel(),[True,False,True,False])
    assert np.array_equal(previous,[True,True,False,False])
    # Loss of visibility and repeated empty frames never erase an obstacle.
    for _ in range(100):result=update_occupancy(result.ravel(),np.zeros(4,bool),np.zeros(4,bool),(2,2))
    assert result[0,0] and result[1,0]


def test_stale_online_map_cancels_segment_and_holds_command():
    from types import SimpleNamespace
    from test_offline import servo_fixture
    from ur7e_gazebo_esdf_sage_continuous.online import DynamicBlocked
    fake,sent,cls=servo_fixture()
    def stale(now):raise DynamicBlocked('ONLINE_MAP_STALE')
    fake.online=SimpleNamespace(read=stale)
    cls.tick(fake)
    assert fake.segment is None and fake.dynamic_hold and not fake.stopped
    assert fake.phase=='WAIT_DYNAMIC'
    np.testing.assert_array_equal(sent[-1],np.zeros(6))
