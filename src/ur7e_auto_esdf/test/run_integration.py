"""ROS services + mock RTDE + real CUDA reconstruction; isolated domain 84."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger,SetBool
from ur7e_auto_esdf.trajectory import load_inputs
from ur7e_auto_esdf.node import Scanner

assert os.environ.get('ROS_DOMAIN_ID')=='84','Must isolate from the real robot ROS graph'
root=Path(__file__).resolve().parents[1]
out=Path(os.environ.get('AUTO_TEST_OUTPUT',str(root/('validation/warmup_integration' if os.environ.get('AUTO_TEST_ONE_POINT')=='1' else 'validation/integration'))));out.mkdir(exist_ok=True,parents=True)
inputs=load_inputs(*(root/'config'/f for f in ['mapping.yaml','control.yaml','waypoints.yaml']))
inputs[1]['output_directory']=str(out/'plans')
inputs[1]['robot_ip']='OFFLINE_ONLY'
if os.environ.get('AUTO_TEST_ONE_POINT')=='1':inputs[2]['waypoints']=inputs[2]['waypoints'][:1]
expected_views=len(inputs[2]['waypoints'])
# Production speed/acceleration, 500 Hz and all eight selected poses.

class FakeReceiver:
    def __init__(self):
        self.q=np.array(inputs[2]['waypoints'][0]['q_rad']);self.dq=np.zeros(6);self.lock=threading.Lock()
        self.fail=False;self.scaling=0.
    def snapshot(self,timeout=.1):
        with self.lock:
            if self.fail:raise RuntimeError('Injected feedback loss')
            return dict(q=self.q.copy(),dq=self.dq.copy(),timestamp=time.monotonic(),received=time.monotonic(),robot_mode=7,safety_mode=1,scaling=self.scaling)
    def close(self):pass


controls=[]
class FakeControl:
    def __init__(self,c):
        assert c['robot_ip']=='OFFLINE_ONLY'
        assert app.mapper['warmup_state']=='READY' and not app.mapper['gate']
        assert all(c.stopped for c in controls)
        receiver.scaling=1.
        self.calls=0;self.stopped=False;controls.append(self)
    def keepalive(self):pass
    def period(self):return time.monotonic()
    def send(self,q,dt):
        assert not self.stopped and not app.mapper['gate']
        with receiver.lock:
            receiver.dq=(q-receiver.q)/dt;receiver.q=q.copy()
        self.calls+=1
    def wait(self,start):time.sleep(max(0,.002-(time.monotonic()-start)))
    def close(self,dec):
        self.stopped=True
        with receiver.lock:receiver.dq[:]=0.;receiver.scaling=0.
        return []


def wait(pred,timeout=30):
    end=time.monotonic()+timeout
    while not pred():
        if time.monotonic()>end:raise AssertionError(f'Timeout {app.phase}, {app.error}, mapper={app.mapper}')
        time.sleep(.02)


def call(name,kind=Trigger,data=None):
    client=observer.create_client(kind,name);assert client.wait_for_service(timeout_sec=10),name
    f=client.call_async(kind.Request() if data is None else kind.Request(data=data));wait(f.done,30)
    res=f.result();observer.destroy_client(client);return res


mapping=inputs[0].copy();mapping['output_directory']=str(out/'maps')
import yaml
(out/'mapping.yaml').write_text(yaml.safe_dump(mapping))
processes=[];streams=[]
def launch(args,name):
    stream=open(out/(name+'.log'),'w');streams.append(stream)
    p=subprocess.Popen([sys.executable,'-B',*args],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p);return p
mapper=launch([str(root/'test/mock_mapper.py'),'--config',str(out/'mapping.yaml'),
               '--ros-args','-r','__node:=ur7e_auto_esdf_mapper'],'mapper')
scene=launch([str(root/'test/mock_camera.py')],'scene')
rclpy.init();receiver=FakeReceiver();app=Scanner(inputs,receiver,control_factory=FakeControl)
observer=Node('integration_observer');executor=SingleThreadedExecutor();executor.add_node(app);executor.add_node(observer)
t=threading.Thread(target=executor.spin,daemon=True);t.start();results={}
started=time.monotonic()
try:
    with patch('ur7e_auto_esdf.node.local_control_conflicts',return_value=[]), \
         patch('ur7e_auto_reconstruction.transport.RTDEControl.__init__',side_effect=AssertionError('Hardware control forbidden in offline test')), \
         patch('ur7e_auto_reconstruction.transport.Receiver.__init__',side_effect=AssertionError('Hardware receive forbidden in offline test')):
        wait(lambda:app.mapper is not None and app.mapper['sensor_ready'],30)
        res=call('/ur7e_auto_esdf_scan/start');assert not res.success and not controls
        results['default_start_rejected_no_control_connection']=True
        res=call('/ur7e_auto_esdf_mapper/capture',SetBool,True);assert not res.success
        results['unleased_capture_rejected']=True
        res=call('/ur7e_auto_esdf_scan/prepare');assert res.success,res.message
        app.approved_plan=res.message;app.allow_motion=True
        print('PREVIEW',res.message,flush=True)
        with receiver.lock:receiver.q[0]+=.02
        res=call('/ur7e_auto_esdf_scan/start');assert not res.success and not controls
        with receiver.lock:receiver.q[0]-=.02
        results['changed_entry_rejected']=True
        res=call('/ur7e_auto_esdf_scan/start');assert res.success,res.message
        last=None;stall_injected=False;capture_points=set()
        end=time.monotonic()+360
        while app.phase not in ('DONE','ERROR','CANCELLED') and time.monotonic()<end:
            value=(app.phase,app.waypoint)
            if value!=last:print('STATE',value,flush=True);last=value
            if app.phase=='MOVING' and app.mapper:
                assert not app.mapper['gate'],'Motion with an open capture gate'
            if app.phase=='CAPTURING':
                assert app.servo is None and all(c.stopped for c in controls)
                capture_points.add(app.waypoint)
                if (not stall_injected and app.waypoint==inputs[2]['waypoints'][-1]['name']
                        and app.mapper.get('gate_frames',0)>=1 and app.mapper['gate']):
                    # Deliberately hold this process's GIL for >40 ms after the first
                    # S08 frame. Real RTDE is forbidden; all transport is simulated.
                    import ctypes
                    libc=ctypes.PyDLL(None);libc.usleep.argtypes=[ctypes.c_uint]
                    before=sum(c.calls for c in controls);started_stall=time.monotonic()
                    libc.usleep(80000)
                    results['capture_host_stall_sec']=time.monotonic()-started_stall
                    assert sum(c.calls for c in controls)==before
                    stall_injected=True
            time.sleep(.05)
        assert app.phase=='DONE',(app.phase,app.error)
        assert stall_injected and len(capture_points)==expected_views
        assert len(controls)==expected_views
        results['completed_waypoints']=expected_views
        results['no_live_control_during_capture']=True
        results['hardware_constructors_forbidden']=True
        result=json.loads((app.run_directory/'result.json').read_text())
        saved=Path(result['map'])
        from ur7e_real_static.field import FrozenMap
        from ur7e_real_static.config import identity
        frozen=FrozenMap.load(saved,identity(mapping))
        assert len(frozen.metadata['views'])==expected_views
        assert np.count_nonzero(frozen.source==1)>100 and not np.any(frozen.source==2)
        assert np.all(frozen.snapshot.distance.reshape(-1)[frozen.source==0]<=0)
        with np.load(saved.parent/'native_esdf.npz',allow_pickle=False) as data:
            from ur7e_nvblox.depth import known_distances
            p,d=data['points'],data['distance'];known=known_distances(d,float(data['unknown']))
            mask=(known & (p[:,0]>.25)&(p[:,0]<.45)&(p[:,1]>-.3)&(p[:,1]<-.1)&(p[:,2]>.175)&(p[:,2]<.3))
            assert mask.sum()>10
            plane_error=float(np.percentile(abs(d[mask]-abs(p[mask,2]-.15)),95))
            assert plane_error<.05,plane_error
        summary=json.loads((saved.parent/'summary.json').read_text())
        results.update(views=len(frozen.metadata['views']),frames=summary['frames'],
                       observed_cells=int(np.count_nonzero(frozen.source==1)),plane_distance_error_p95_m=plane_error,
                       unknown_blocked=True,static_planner_loader_compatible=True,final_map=str(saved))
        assert controls[-1].stopped
        results['motion_finished_before_export']=True
        assert all(o['frames']>=inputs[1]['capture_frames'] for o in result['observations'])
        trace_data=np.load(app.run_directory/'servo_trace.npz',allow_pickle=False)
        trace=trace_data['samples'];same_segment=np.diff(trace_data['segment_index'])==0
        assert len(result['motion_segments'])==expected_views
        maxstep=float(abs(np.diff(trace[:,3:9],axis=0)[same_segment]).max())
        assert maxstep<=inputs[1]['max_joint_speed']/inputs[1]['frequency_hz']+1e-8
        results['max_command_step_rad']=maxstep
        results['servo_cycle_p95_ms']=float(np.percentile(np.diff(trace[:,0])[same_segment],95)*1000)
        wait(lambda:not app.worker.is_alive())
        # A second run: explicit cancellation during the entry segment.
        res=call('/ur7e_auto_esdf_scan/prepare');assert res.success;app.approved_plan=res.message
        res=call('/ur7e_auto_esdf_scan/start');assert res.success,res.message
        wait(lambda:app.phase=='MOVING',30);time.sleep(.3)
        res=call('/ur7e_auto_esdf_scan/cancel');assert res.success
        wait(lambda:app.phase in ('CANCELLED','ERROR'),20)
        assert app.phase=='CANCELLED',app.error
        assert controls[-1].stopped
        n=controls[-1].calls;time.sleep(.2);assert controls[-1].calls==n
        results['cancel_stops_and_does_not_resume']=True
        results['passed']=True
finally:
    results['elapsed_sec']=time.monotonic()-started
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2),flush=True)
    app.close();executor.shutdown();t.join();app.destroy_node();observer.destroy_node();rclpy.shutdown()
    for p in processes:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in processes:
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    for f in streams:f.close()
