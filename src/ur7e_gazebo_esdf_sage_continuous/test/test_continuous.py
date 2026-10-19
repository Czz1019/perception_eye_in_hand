import numpy as np
from ur7e_gazebo_esdf_sage_continuous.continuous import shift_count,checked_prefix

def test_shift_accumulates_fractional_elapsed_time():
 count,origin=shift_count(.049,0.,.05,40);assert count==0 and origin==0.
 count,origin=shift_count(.11,origin,.05,40);assert count==2 and abs(origin-.1)<1e-9
 count,origin=shift_count(.149,origin,.05,40);assert count==0
 assert shift_count(5.,0.,.05,40)==(40,5.)

def test_only_collision_free_prefix_is_executable(monkeypatch):
 import ur7e_gazebo_esdf_sage_continuous.continuous as module
 monkeypatch.setattr(module,'edge_clear',lambda g,s,a,b,m,sm: b[0]<=.11)
 points=checked_prefix(None,None,np.zeros(6),np.ones((4,6)),.05,.03,.015)
 assert len(points)==3
 np.testing.assert_allclose(points[-1],.1)


def test_expired_short_horizon_cancels_all_queued_motion():
 from test_offline import servo_fixture
 fake,sent,cls=servo_fixture()
 fake.continuous_active=True;fake.plan_valid_until=0.
 fake.pending_segments=[(np.zeros(6),np.ones(6),0.,1.,False)]
 cls.tick(fake)
 assert fake.segment is None and fake.pending_segments==[] and not fake.stopped
 np.testing.assert_array_equal(sent[-1],np.zeros(6))

def test_stale_map_discards_current_and_future_segments():
 from types import SimpleNamespace
 from test_offline import servo_fixture
 from ur7e_gazebo_esdf_sage_continuous.online import DynamicBlocked
 fake,sent,cls=servo_fixture()
 fake.pending_segments=[(np.zeros(6),np.ones(6),0.,1.,False)]
 def stale():raise DynamicBlocked('ONLINE_MAP_STALE')
 fake.online=SimpleNamespace();fake.read_online=stale
 cls.tick(fake)
 assert fake.segment is None and fake.pending_segments==[] and not fake.stopped
 np.testing.assert_array_equal(sent[-1],np.zeros(6))

def test_replacement_between_validation_and_execution_waits_for_recheck(monkeypatch):
 from types import SimpleNamespace
 from test_offline import servo_fixture
 import ur7e_gazebo_esdf_sage_continuous.node as module
 fake,sent,cls=servo_fixture();fake.online=object();fake.dynamic_version=-1;fake.last_geometry_check=0.;fake.last_log=100.
 fake.g=object();fake.c={'collision_margin':.03,'self_margin':.015};fake.read_online=lambda:(object(),2)
 calls=[]
 def checked(*args):
  calls.append(True)
  if len(calls)==1:
   fake.plan_generation+=1
   fake.segment=(np.zeros(6),np.ones(6),0.,1.,False)
  return True
 monkeypatch.setattr(module,'edge_clear',checked)
 cls.tick(fake)
 np.testing.assert_array_equal(sent[-1],np.zeros(6))
 assert not fake.stopped and fake.validated_generation!=fake.plan_generation
 fake.now=lambda:11.
 cls.tick(fake)
 assert len(calls)==2 and fake.validated_generation==fake.plan_generation
 np.testing.assert_allclose(sent[-1],.02)
