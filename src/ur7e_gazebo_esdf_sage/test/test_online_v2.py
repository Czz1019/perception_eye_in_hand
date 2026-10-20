"""Sparse clearing must exactly preserve the original dense occupancy rule."""
import numpy as np
from types import SimpleNamespace
from ur7e_perception_planning_core.grid import observed_free
from ur7e_gazebo_esdf_sage.online import update_occupancy

def test_sparse_clearing_equals_dense_at_occlusion_and_invalid_pixels():
 rng=np.random.default_rng(4);points=rng.uniform([-.5,-.5,.1],[.5,.5,1.5],(5000,3));old=rng.random(5000)<.15;hit=rng.random(5000)<.02
 depth=np.full((80,100),1.);depth[10:20,30:40]=0.;depth[40:50,60:70]=.4
 intr=SimpleNamespace(width=100,height=80,fx=70,fy=70,cx=50,cy=40)
 dense=observed_free(points,depth,np.eye(4),intr,.05)
 sparse=np.zeros(len(points),bool);ids=np.flatnonzero(old);sparse[ids]=observed_free(points[ids],depth,np.eye(4),intr,.05)
 np.testing.assert_array_equal(update_occupancy(old,sparse,hit,(50,100)),update_occupancy(old,dense,hit,(50,100)))


def test_online_recovery_holds_before_reusing_a_fresh_snapshot():
 import threading
 from ur7e_gazebo_esdf_sage.node import ClosedLoop
 from ur7e_gazebo_esdf_sage.online import DynamicBlocked
 calls=[];events=[];snapshot=object()
 def read(now):
  calls.append(now)
  if len(calls)==1:raise DynamicBlocked('ONLINE_MAP_STALE')
  return snapshot,7
 fake=SimpleNamespace(stopped=False,exit=threading.Event(),scan={'online_engine':'v2'},lock=threading.RLock(),segment=('active',),online=SimpleNamespace(read=read),now=lambda:1.,event=lambda name:events.append(name))
 assert ClosedLoop.online_snapshot(fake)==(snapshot,7)
 assert fake.segment is None and events==['online_wait','online_recovered']
