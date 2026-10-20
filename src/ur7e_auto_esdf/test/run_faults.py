"""Fast coordinator fault-injection on domain 86, with no RTDE/network hardware."""
import json
import os
from pathlib import Path
import threading
import time
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from ur7e_auto_esdf.trajectory import load_inputs,make_plan
from ur7e_auto_esdf.node import Scanner
assert os.environ.get('ROS_DOMAIN_ID')=='86'
root=Path(__file__).resolve().parents[1];out=Path(os.environ.get('AUTO_TEST_OUTPUT',str(root/'validation/faults')));out.mkdir(exist_ok=True,parents=True)
inputs=load_inputs(*(root/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))
inputs[1].update(output_directory=str(out),capture_timeout_sec=.8,mapper_timeout_sec=.3,settle_sec=.1)
all_waypoints=inputs[2]['waypoints'];inputs[2]['waypoints']=all_waypoints[:1]
inputs[1]['robot_ip']='OFFLINE_ONLY'

class Feedback:
    def __init__(self):self.q=np.array(inputs[2]['waypoints'][0]['q_rad']);self.fail=False;self.safety=1;self.dq=np.zeros(6);self.scaling=0.
    def snapshot(self,timeout=.1):
        if self.fail:raise RuntimeError('Injected feedback loss')
        return dict(q=self.q.copy(),dq=self.dq.copy(),timestamp=time.monotonic(),received=time.monotonic(),robot_mode=7,safety_mode=self.safety,scaling=self.scaling)
    def close(self):pass

controls=[];connect_delay=0.;current_fault=None;run_first_control=0
class Control:
    def __init__(self,c):
        assert c['robot_ip']=='OFFLINE_ONLY' and not mapper.gate
        assert all(c.stopped for c in controls)
        self.stopped=False;self.calls=0;controls.append(self);feedback.scaling=1.;time.sleep(connect_delay)
        if current_fault=='reconnect_drift' and len(controls)-run_first_control==2:feedback.q[0]+=.02
    def keepalive(self):pass
    def period(self):return time.monotonic()
    def send(self,q,dt):self.calls+=1;feedback.q=q.copy()
    def wait(self,t):time.sleep(max(0,.002-(time.monotonic()-t)))
    def close(self,dec):
        self.stopped=True;feedback.scaling=0.
        return ['Injected stop acknowledgement failure'] if current_fault=='stop_ack_failure' else []

class Mapper(Node):
    def __init__(self):
        super().__init__('ur7e_auto_esdf_mapper');self.publish=True;self.phase='IDLE';self.gate=False;self.session='0';self.epoch=0
        self.pub=self.create_publisher(String,'~/capture_status',10)
        self.create_timer(.05,self.report)
        self.create_service(Trigger,'~/start_scan',self.start)
        self.create_service(Trigger,'~/freeze_scan',self.freeze)
        self.create_service(Trigger,'~/warmup',self.warmup)
        self.warmup_state='COLD';self.frames=0;self.freeze_at=None
        self.create_service(SetBool,'~/capture',self.capture)
    def start(self,req,res):
        self.phase='SCANNING';self.session=str(out/('map_'+str(time.monotonic())));res.success=True;return res
    def warmup(self,req,res):self.warmup_state='READY';res.success=True;return res
    def freeze(self,req,res):
        # Respond first; keep reporting SCANNING until the export completes.
        self.freeze_at=time.monotonic()+.25;res.success=True;return res
    def capture(self,req,res):
        self.gate=req.data
        if req.data:
            assert app.servo is None and all(c.stopped for c in controls)
            self.epoch+=1
        res.success=True;res.message=json.dumps({'epoch':self.epoch});return res
    def report(self):
        if self.freeze_at and time.monotonic()>=self.freeze_at:
            folder=Path(self.session);folder.mkdir(exist_ok=True)
            np.savez(folder/'frozen_map.npz',vertices=np.zeros((1,3)))
            self.phase='FROZEN';self.freeze_at=None
        if self.publish:self.pub.publish(String(data=json.dumps(dict(warmup_state=self.warmup_state,phase=self.phase,error=None,epoch=self.epoch,
            gate=self.gate,gate_frames=8 if current_fault=='reconnect_drift' and self.gate else 0,frames=self.frames,session=self.session,sensor_ready=True))))


def wait(p,timeout=8):
    end=time.monotonic()+timeout
    while not p():
        if time.monotonic()>end:raise AssertionError((app.phase,app.error))
        time.sleep(.01)

rclpy.init();feedback=Feedback();mapper=Mapper();app=Scanner(inputs,feedback,allow_motion=True,control_factory=Control)
executor=SingleThreadedExecutor();executor.add_node(mapper);executor.add_node(app)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start();results={}
try:
    with patch('ur7e_auto_esdf.node.local_control_conflicts',return_value=[]), \
         patch('ur7e_auto_reconstruction.transport.RTDEControl.__init__',side_effect=AssertionError('Hardware control forbidden')), \
         patch('ur7e_auto_reconstruction.transport.Receiver.__init__',side_effect=AssertionError('Hardware receive forbidden')):
        for fault in ['no_frames','mapper_loss','feedback_loss','cancel_connect','delayed_partial_export',
                      'capture_drift','capture_feedback_loss','capture_safety','capture_velocity','cancel_capture',
                      'stop_ack_failure','reconnect_drift']:
            current_fault=fault;feedback.q=np.array(all_waypoints[0]['q_rad']);feedback.safety=1;feedback.dq[:]=0.;feedback.scaling=0.
            inputs[2]['waypoints']=all_waypoints[:2] if fault=='reconnect_drift' else all_waypoints[:1]
            mapper.publish=True;mapper.phase='IDLE';mapper.gate=False;feedback.fail=False
            mapper.frames=1 if fault=='delayed_partial_export' else 0
            wait(lambda:app.mapper and app.mapper['phase']=='IDLE')
            plan=make_plan(feedback.q,inputs[1],inputs[2],inputs[3]);path=out/(fault+'_plan.json')
            path.write_text(json.dumps(plan));app.approved_plan=str(path)
            connect_delay=.4 if fault=='cancel_connect' else 0.
            count=len(controls);run_first_control=count;epoch_before=mapper.epoch
            res=app.start(Trigger.Request(),Trigger.Response());assert res.success,res.message
            if fault=='cancel_connect':
                wait(lambda:len(controls)>count);app.abort(Trigger.Request(),Trigger.Response())
            else:
                wait(lambda:app.phase=='MOVING')
                if fault=='mapper_loss':mapper.publish=False
                if fault=='feedback_loss':feedback.fail=True
                if fault.startswith('capture_') or fault=='cancel_capture':
                    wait(lambda:app.phase=='CAPTURING' and mapper.gate)
                    assert app.servo is None and controls[-1].stopped
                    if fault=='capture_drift':feedback.q[0]+=.02
                    if fault=='capture_feedback_loss':feedback.fail=True
                    if fault=='capture_safety':feedback.safety=3
                    if fault=='capture_velocity':feedback.dq[0]=.02
                    if fault=='cancel_capture':app.abort(Trigger.Request(),Trigger.Response())
            wait(lambda:app.phase in ('ERROR','CANCELLED'),10)
            wait(lambda:not app.worker.is_alive())
            assert controls[-1].stopped
            if fault in ('cancel_connect','cancel_capture'):
                assert app.phase=='CANCELLED'
                if fault=='cancel_connect':assert controls[-1].calls==0
            else:assert app.phase=='ERROR'
            expected={'no_frames':'Timeout during CAPTURING','mapper_loss':'Reconstruction heartbeat missing',
                      'feedback_loss':'Injected feedback loss','cancel_connect':'CANCELLED_BY_USER',
                      'delayed_partial_export':'Timeout during CAPTURING','capture_drift':'position drifted',
                      'capture_feedback_loss':'Injected feedback loss','capture_safety':'Robot state changed',
                      'capture_velocity':'Robot is moving','cancel_capture':'CANCELLED_BY_USER',
                      'stop_ack_failure':'Stop acknowledgement failed','reconnect_drift':'position drifted'}
            assert expected[fault] in app.error,(fault,app.error)
            n=controls[-1].calls;time.sleep(.05);assert controls[-1].calls==n
            assert len(controls)-count==(2 if fault=='reconnect_drift' else 1)
            if fault=='reconnect_drift':assert controls[-1].calls==0
            if fault=='stop_ack_failure':assert mapper.epoch==epoch_before
            results[fault]=dict(phase=app.phase,error=app.error,stopped=True,commands=n,control_connections=len(controls)-count)
            if fault=='delayed_partial_export':
                saved=json.loads((app.run_directory/'result.json').read_text())
                assert saved['partial_map'] and Path(saved['partial_map']).is_file(),saved
                results[fault]['partial_map']=saved['partial_map']
        results['passed']=True
finally:
    app.close();executor.shutdown();thread.join();app.destroy_node();mapper.destroy_node();rclpy.shutdown()
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
