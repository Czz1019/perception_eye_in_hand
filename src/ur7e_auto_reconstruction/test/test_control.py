import copy
import json
import os
from pathlib import Path
import time
import numpy as np
import pytest
from ur7e_auto_reconstruction.trajectory import Segment,load_inputs,make_plan,verify_plan
from ur7e_auto_reconstruction.transport import Servo,Receiver

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def inputs():
    return load_inputs(*(ROOT/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))


def test_all_selected_segments_have_bounded_derivatives(inputs):
    m,c,r,g=inputs
    q=np.array(r['waypoints'][0]['q_rad'])
    for w in r['waypoints']:
        s=Segment(q,w['q_rad'],c['max_joint_speed'],c['max_joint_acceleration'])
        t=np.linspace(0,s.duration,10001)
        path=s.sample(t);velocity=np.gradient(path,t,axis=0);acc=np.gradient(velocity,t,axis=0)
        assert np.max(abs(velocity)) <= c['max_joint_speed']+1e-6
        assert np.max(abs(acc)) <= c['max_joint_acceleration']*.5+1e-6
        assert np.max(abs(velocity[[0,-1]])) < 1e-6
        assert np.allclose(path[0],q) and np.allclose(path[-1],w['q_rad'])
        assert np.all(path>g.lower) and np.all(path<g.upper)
        q=np.asarray(w['q_rad'])


def test_plan_change_stale_start_and_offline_rejected(inputs):
    m,c,r,g=inputs;q=r['waypoints'][0]['q_rad'];p=make_plan(q,c,r,g)
    verify_plan(p,q,c,r,g)
    for bad in [dict(p,source='offline_reference_NOT_EXECUTABLE'),dict(p,q_start=[0.]*6)]:
        with pytest.raises(ValueError):verify_plan(bad,q,c,r,g)
    with pytest.raises(ValueError):verify_plan(p,np.asarray(q)+.02,c,r,g)
    with pytest.raises(ValueError):verify_plan(p,q,dict(c,max_joint_speed=.13),r,g)


@pytest.mark.parametrize('q',[[0]*5,[0]*7,[np.nan]*6,[np.inf]*6])
def test_invalid_joint_targets(q):
    with pytest.raises(ValueError):Segment(q,[0]*6,.1,.1)


class Feedback:
    def __init__(self,q):
        self.q=np.asarray(q).copy();self.dq=np.zeros(6);self.fail=False;self.mode=1;self.scale=1.
    def snapshot(self,timeout=.1):
        if self.fail:raise RuntimeError('stale feedback')
        return dict(q=self.q.copy(),dq=self.dq.copy(),timestamp=time.monotonic(),received=time.monotonic(),
                    robot_mode=7,safety_mode=self.mode,scaling=self.scale)


class Control:
    def __init__(self,feedback):
        self.f=feedback;self.calls=[];self.stopped=False;self.delay=0.
    def period(self):return time.monotonic()
    def send(self,q,dt):
        self.calls.append(q.copy());self.f.dq=(q-self.f.q)/dt;self.f.q=q.copy()
    def wait(self,t):time.sleep(max(0,.002-(time.monotonic()-t))+self.delay)
    def close(self,deceleration):self.stopped=True;return []


def wait_for(p,timeout=2):
    end=time.monotonic()+timeout
    while not p():
        if time.monotonic()>end:raise AssertionError('timeout')
        time.sleep(.005)


@pytest.mark.parametrize('fault',['feedback','safety','tracking','deadline','heartbeat','scaling','velocity'])
def test_servo_stops_on_fault(inputs,fault):
    c=inputs[1];f=Feedback(np.zeros(6));b=Control(f);s=Servo(f,b,c,np.zeros(6))
    wait_for(lambda:len(b.calls)>=3)
    if fault=='feedback':f.fail=True
    if fault=='safety':f.mode=3
    if fault=='tracking':f.q[:]=.2
    if fault=='deadline':b.delay=.06
    if fault=='heartbeat':s.heartbeat-=1
    if fault=='scaling':f.scale=0
    if fault=='velocity':f.dq[:]=.4
    wait_for(s.finished.is_set)
    assert s.error and b.stopped
    if fault=='deadline':
        assert 'interval=' in s.error and 'limit=0.040000s' in s.error
        assert s.diagnostics['fault']['interval_sec']>.04
        assert s.diagnostics['fault']['last_cycle_sec']['record_and_wait']>.04
    count=len(b.calls);time.sleep(.01);assert len(b.calls)==count
    s.close()


def test_cancel_and_no_catchup(inputs):
    c=inputs[1];f=Feedback(np.zeros(6));b=Control(f);s=Servo(f,b,c,np.zeros(6))
    try:
        segment=Segment(np.zeros(6),[.05]*6,c['max_joint_speed'],c['max_joint_acceleration'])
        s.move(segment)
        for _ in range(30):s.ping();time.sleep(.003)
        s.stop.set();s.close()
        cmd=np.asarray(b.calls)
        assert np.max(abs(np.diff(cmd,axis=0))) <= c['max_joint_speed']/c['frequency_hz']+1e-9
        assert b.stopped and s.error is None
    finally:s.close()


def test_speed_scaling_preserves_command_acceleration(inputs):
    c=inputs[1];f=Feedback(np.zeros(6));b=Control(f);s=Servo(f,b,c,np.zeros(6))
    try:
        s.move(Segment([0]*6,[.05]*6,c['max_joint_speed'],c['max_joint_acceleration']))
        for i in range(400):
            f.scale=.2 if 100<i<250 else 1.
            s.ping();time.sleep(.002)
        s.close()
        cmd=np.asarray(b.calls);v=np.diff(cmd,axis=0)*c['frequency_hz'];a=np.diff(v,axis=0)*c['frequency_hz']
        assert np.max(abs(v))<=c['max_joint_speed']+1e-6
        assert np.max(abs(a))<=c['max_joint_acceleration']+1e-3
    finally:s.close()


class FrozenRTDE:
    def __init__(self,*a):self.ts=1.
    def getTimestamp(self):return self.ts
    def getActualQ(self):return [0]*6
    def getActualQd(self):return [0]*6
    def getRobotMode(self):return 7
    def getSafetyMode(self):return 1
    def getSpeedScaling(self):return 1.
    def getTargetSpeedFraction(self):return 1.
    def disconnect(self):pass


def test_repeated_rtde_timestamp_is_not_fresh():
    r=Receiver('TEST-NO-NETWORK',factory=FrozenRTDE)
    try:
        wait_for(lambda:r.latest is not None);r.snapshot(.05)
        time.sleep(.07)
        with pytest.raises(RuntimeError,match='stale'):r.snapshot(.05)
    finally:r.close()


def test_model_hash_and_waypoint_fk_checked(inputs,tmp_path):
    import yaml
    m,c,r,g=inputs
    route=copy.deepcopy(r);route['waypoints'][0]['q_rad'][0]+=.02
    path=tmp_path/'waypoints.yaml';path.write_text(yaml.safe_dump(route))
    with pytest.raises(ValueError,match='inconsistency'):
        load_inputs(ROOT/'config/mapping.yaml',ROOT/'config/control.yaml',path)


def test_servoj_does_not_overwrite_command_with_watchdog(inputs):
    from ur7e_auto_reconstruction.transport import RTDEControl
    class API:
        def __init__(self):self.calls=[]
        def servoJ(self,*args):self.calls.append('servoJ');return True
        def kickWatchdog(self):self.calls.append('NO_CMD');return True
    transport=RTDEControl.__new__(RTDEControl)
    transport.control=API();transport.c=inputs[1]
    transport.send(np.zeros(6),.002)
    assert transport.control.calls==['servoJ']
    transport.keepalive()
    assert transport.control.calls==['servoJ','NO_CMD']


def test_failed_servo_stop_is_reported_and_disconnect_still_attempted(inputs):
    from ur7e_auto_reconstruction.transport import RTDEControl
    class API:
        def __init__(self):self.calls=[]
        def servoStop(self,*args):self.calls.append('servoStop');return False
        def stopScript(self):self.calls.append('stopScript')
        def disconnect(self):self.calls.append('disconnect')
    transport=RTDEControl.__new__(RTDEControl);api=API()
    transport.control=api;transport.file=None
    assert transport.close(.5)==['servoStop: returned false']
    assert api.calls==['servoStop','stopScript','disconnect']
