"""Mapper gates and persistence only; no RTDE or physical camera connections."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='85'
import threading,time
from pathlib import Path
from types import SimpleNamespace
from collections import deque
import numpy as np
import pytest
from std_srvs.srv import SetBool
from sensor_msgs.msg import CameraInfo
from ur7e_auto_esdf.mapper import EsdfMapper
from ur7e_auto_esdf.trajectory import load_inputs
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.config import identity
ROOT=Path(__file__).resolve().parents[1]


def dummy():
    now=time.monotonic()
    n=SimpleNamespace(gate=False,scanning=True,lease=now,states=deque([(np.zeros(6),np.zeros(6),1.,now)]),
        c={'joint_timeout':.3,'settle_speed':.015},capture_epoch=0,gate_frames=0,gate_cutoff=0.,gate_q=None,
        lock=threading.RLock(),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10**9)))
    return n

@pytest.mark.parametrize('fault',['lease','feedback','moving','not_scanning','already_open'])
def test_bad_capture_request_rejected(fault):
    n=dummy()
    if fault=='lease':n.lease-=1
    if fault=='feedback':n.states[-1]=(np.zeros(6),np.zeros(6),1.,time.monotonic()-1)
    if fault=='moving':n.states[-1][1][0]=.1
    if fault=='not_scanning':n.scanning=False
    if fault=='already_open':n.gate=True
    r=EsdfMapper.capture(n,SetBool.Request(data=True),SetBool.Response())
    assert not r.success and n.capture_epoch==0


def test_epoch_cutoff_and_drift_do_not_credit_another_view():
    n=dummy();r=EsdfMapper.capture(n,SetBool.Request(data=True),SetBool.Response());assert r.success
    assert EsdfMapper.capture_allowed(n,1.1,np.zeros(6))
    assert not EsdfMapper.capture_allowed(n,.9,np.zeros(6))
    assert not EsdfMapper.capture_allowed(n,1.1,np.ones(6)*.02)
    assert not EsdfMapper.capture(n,SetBool.Request(data=True),SetBool.Response()).success
    assert EsdfMapper.capture(n,SetBool.Request(data=False),SetBool.Response()).success
    assert not EsdfMapper.capture_allowed(n,1.1,np.zeros(6))
    n.lease=time.monotonic()-1
    assert not EsdfMapper.capture_allowed(n)


def test_route_and_mapping_use_same_calibrated_model():
    c,control,route,g=load_inputs(*(ROOT/'config'/x for x in ['mapping.yaml','control.yaml','waypoints.yaml']))
    assert len(route['waypoints'])==8 and c['base_frame']=='base_link'
    assert not c['execution_enabled'] and not c['commissioned_free_boxes']
    assert c['joint_topic']=='/ur7e_auto_esdf/joint_states'


def test_frozen_map_saved_for_static_planner_but_not_declared_planning_ready(tmp_path):
    c,_,_,_=load_inputs(*(ROOT/'config'/x for x in ['mapping.yaml','control.yaml','waypoints.yaml']))
    ci=CameraInfo();ci.width=2;ci.height=2;ci.header.frame_id=c['camera_frame']
    record=EsdfMapper.info_record(ci)
    n=SimpleNamespace(scanning=True,gate=False,c=c,capture_count=64,views=[np.zeros(6)]*8,camera_info=ci,
        info_record=EsdfMapper.info_record,latest_field=(np.zeros(8),np.zeros(8,dtype=np.uint8),np.zeros(3),np.array([2,2,2]),1.,64,time.monotonic(),record),
        model_id=identity(c),epoch=1,observations=[],session=tmp_path,raw_surface=None,states=[])
    EsdfMapper.freeze_scan(n)
    a=FrozenMap.load(tmp_path/'frozen_map.npz',identity(c))
    assert not a.metadata['planning_ready'] and np.all(a.snapshot.distance<=0)
    assert not n.scanning and n.phase=='FROZEN'


def test_freeze_rejects_open_gate():
    n=SimpleNamespace(scanning=True,gate=True,latest_field=None)
    with pytest.raises(ValueError):EsdfMapper.freeze_scan(n)
