"""Explicit preview/start/cancel coordinator. No control connection at startup."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import threading
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_real_reconstruction.node import write_json
from .trajectory import load_inputs, make_plan, verify_plan, Segment
from .transport import Receiver, RTDEControl, Servo, local_control_conflicts


class Scanner(Node):
    def __init__(self, inputs, receiver, *, allow_motion=False, approved_plan='', control_factory=RTDEControl):
        super().__init__('ur7e_auto_scan')
        self.mapping,self.c,self.route,self.geometry = inputs
        self.receiver,self.control_factory = receiver,control_factory
        self.allow_motion,self.approved_plan = allow_motion,approved_plan
        self.phase,self.error,self.waypoint = 'PREVIEW_ONLY' if not allow_motion else 'WAIT_START',None,None
        self.cancel = threading.Event()
        self.worker = None
        self.servo = None
        self.capture_q = None
        self.mapper,self.mapper_received = None,0.
        self.pulse = 0.
        self.last_stamp = None
        self.run_directory = None
        self.prepare_path = None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.states=self.create_publisher(JointState,'/ur7e_auto/joint_states',10)
        self.status=self.create_publisher(String,'~/status',qos)
        self.route_pub=self.create_publisher(MarkerArray,'~/route',qos)
        self.lease=self.create_publisher(Empty,'~/capture_lease',5)
        self.create_subscription(String,'/ur7e_auto_mapper/capture_status',self.on_mapper,10)
        self.mapper_clients={name:self.create_client(kind,'/ur7e_auto_mapper/'+name)
                      for name,kind in [('start_scan',Trigger),('freeze_scan',Trigger),('capture',SetBool),('warmup',Trigger)]}
        self.create_service(Trigger,'~/prepare',self.prepare)
        self.create_service(Trigger,'~/start',self.start)
        self.create_service(Trigger,'~/cancel',self.abort)
        self.create_timer(.02,self.publish_state)
        self.create_timer(.1,self.report)
        self.get_logger().info('RTDE receive-only. ~/prepare writes a preview; ~/start requires allow_motion and an approved live plan.')

    def on_mapper(self,msg):
        try:
            self.mapper=json.loads(msg.data)
            self.mapper_received=time.monotonic()
        except ValueError:
            pass

    def publish_state(self):
        try:
            s=self.receiver.snapshot(self.c['feedback_timeout_sec'])
        except Exception:
            return
        if s['timestamp']==self.last_stamp:
            return
        # Preserve age of cached RTDE data instead of stamping stale values as new.
        stamp=self.get_clock().now()-Duration(seconds=max(0.,time.monotonic()-s['received']))
        msg=JointState();msg.header.stamp=stamp.to_msg();msg.name=JOINTS
        msg.position=s['q'].tolist();msg.velocity=s['dq'].tolist()
        self.states.publish(msg);self.last_stamp=s['timestamp']

    def report(self):
        if time.monotonic()-self.pulse < .3 and not self.cancel.is_set():
            self.lease.publish(Empty())
        state=dict(phase=self.phase,error=self.error,waypoint=self.waypoint,allow_motion=self.allow_motion,
                   preview=self.prepare_path,run_directory=None if self.run_directory is None else str(self.run_directory),
                   servo_active=self.servo is not None,
                   capture_stationary_guard=self.capture_q is not None,
                   collision_checked=False,self_collision_checked=False)
        self.status.publish(String(data=json.dumps(state)))

    def prepare(self,req,res):
        if self.worker and self.worker.is_alive():
            res.success=False;res.message='Scan is running';return res
        try:
            state=self.receiver.snapshot(self.c['feedback_timeout_sec'])
            if np.max(abs(state['dq'])) > self.c['settle_speed_rad_s']:
                raise RuntimeError('Stop manual motion before preparing a trajectory')
            plan=make_plan(state['q'],self.c,self.route,self.geometry)
            directory=Path(self.c['output_directory']).expanduser()/('preview_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
            directory.mkdir(parents=True)
            write_json(directory/'plan.json',plan)
            from .preview import render
            render(plan,directory,self.geometry)
            self.prepare_path=str(directory/'plan.json')
            self.publish_route(plan)
            res.success=True;res.message=self.prepare_path
        except Exception as exc:
            res.success=False;res.message=str(exc)
        return res

    def publish_route(self,plan):
        markers=[]
        for i,segment in enumerate(plan['segments']):
            m=Marker();m.header.frame_id='base_link';m.ns='camera_route';m.id=i;m.type=Marker.LINE_STRIP
            m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.006
            m.color.r,m.color.g,m.color.b,m.color.a=(1.,.45,.1,1.) if i==0 else (.15,.8,1.,1.)
            m.points=[Point(x=x,y=y,z=z) for x,y,z in segment['camera_path_m']]
            markers.append(m)
        self.route_pub.publish(MarkerArray(markers=markers))

    def start(self,req,res):
        if not self.allow_motion or not self.approved_plan:
            res.success=False;res.message='Need allow_motion:=true and approved_plan:=<reviewed live plan.json>';return res
        if self.worker and self.worker.is_alive():
            res.success=False;res.message='Already running';return res
        try:
            plan=json.loads(Path(self.approved_plan).expanduser().read_text())
            state=self.receiver.snapshot(self.c['feedback_timeout_sec'])
            verify_plan(plan,state['q'],self.c,self.route,self.geometry)
            if state['robot_mode'] != 7 or state['safety_mode'] not in (1,2) or np.max(abs(state['dq'])) > self.c['settle_speed_rad_s']:
                raise RuntimeError('Robot must be running, stationary, and free of safety stops')
            # Even inactive ROS joint controllers do not establish exclusive script ownership.
            if local_control_conflicts() or any('controller_manager' in n for n,_ in self.get_node_names_and_namespaces()):
                raise RuntimeError('Stop UR ROS control driver before standalone RTDE control; camera can remain running')
            self.require_mapper()
            if self.mapper['phase'] not in ('IDLE','FROZEN','ERROR') or not self.mapper['sensor_ready']:
                raise RuntimeError('Need idle/frozen mapper and fresh real RGB-D streams')
        except Exception as exc:
            res.success=False;res.message=str(exc);return res
        self.cancel.clear();self.error=None;self.phase='STARTING'
        self.worker=threading.Thread(target=self.run,args=(plan,),daemon=True,name='scan-coordinator')
        self.worker.start()
        res.success=True;res.message='Accepted. Follow ~/status; use ~/cancel to stop.';return res

    def abort(self,req,res):
        self.cancel.set()
        if self.servo:self.servo.stop.set()
        res.success=True;res.message='Stop requested; wait for CANCELLED/ERROR status';return res

    def require_mapper(self):
        if self.mapper is None or time.monotonic()-self.mapper_received > self.c['mapper_timeout_sec']:
            raise RuntimeError('Reconstruction heartbeat missing')

    def tick(self, check_mapper=True):
        if self.cancel.is_set():raise RuntimeError('CANCELLED_BY_USER')
        if self.capture_q is not None:
            self.stationary_state(self.capture_q,self.c['arrival_tolerance_rad'])
        self.pulse=time.monotonic()
        if self.servo:self.servo.ping()
        if check_mapper:
            self.require_mapper()
            if self.mapper.get('error'):raise RuntimeError('Mapper: '+self.mapper['error'])
        time.sleep(.01)

    def wait(self,predicate,timeout,check_mapper=True):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:
                raise RuntimeError(f'Timeout during {self.phase}; mapper={self.mapper}')
            self.tick(check_mapper)
        self.tick(check_mapper)

    def call(self,name,data=None,cleanup=False):
        client=self.mapper_clients[name]
        if cleanup:
            if not client.service_is_ready():raise RuntimeError(f'Missing mapper service {name}')
        else:
            self.wait(client.service_is_ready,self.c['service_timeout_sec'],check_mapper=False)
        req=SetBool.Request(data=data) if name=='capture' else Trigger.Request()
        future=client.call_async(req)
        if cleanup:
            end=time.monotonic()+3
            while not future.done() and time.monotonic()<end:time.sleep(.02)
            if not future.done():raise RuntimeError('Cleanup service timeout')
        else:
            self.wait(future.done,self.c['service_timeout_sec'],check_mapper=False)
        result=future.result()
        if not result.success:raise RuntimeError(result.message)
        return result.message

    def settle(self,q):
        started=None
        end=time.monotonic()+20
        while time.monotonic()<end:
            self.tick()
            state=self.receiver.snapshot(self.c['feedback_timeout_sec'])
            if state['robot_mode'] != 7 or state['safety_mode'] not in (1,2):
                raise RuntimeError('Robot state changed while settling')
            ready=np.max(abs(state['q']-q)) < self.c['arrival_tolerance_rad'] and np.max(abs(state['dq'])) < self.c['settle_speed_rad_s']
            started=(started or time.monotonic()) if ready else None
            if started and time.monotonic()-started >= self.c['settle_sec']:return
        raise RuntimeError('Arrival/settling timeout')

    def stationary_state(self, q, tolerance):
        state=self.receiver.snapshot(self.c['feedback_timeout_sec'])
        if state['robot_mode'] != 7 or state['safety_mode'] not in (1,2):
            raise RuntimeError('Robot state changed while stationary; no automatic recovery')
        if np.max(abs(state['q']-q)) > tolerance:
            raise RuntimeError('Robot position drifted while stationary; prepare again')
        if np.max(abs(state['dq'])) > self.c['settle_speed_rad_s']:
            raise RuntimeError('Robot is moving while stationary capture/control acquisition is required')
        # A stopped RTDE script reports zero speed scaling. No motion is requested
        # here; do not treat zero scaling during a disconnected capture as a fault.
        return state

    def acquire_servo(self, expected_q, tolerance):
        self.tick()
        if self.mapper['gate'] or self.mapper['phase']!='SCANNING':
            raise RuntimeError('Control acquisition requires a closed capture gate and SCANNING mapper')
        self.stationary_state(expected_q,tolerance)
        control=None
        try:
            control=self.control_factory(self.c)
            until=time.monotonic()+5.
            while True:
                self.tick()
                control.keepalive()
                state=self.stationary_state(expected_q,tolerance)
                if state['scaling'] >= .01:break
                if time.monotonic()>until:
                    raise RuntimeError('RTDE script speed scaling stayed zero')
            # Each movement starts from fresh actual q, never the last sent sample.
            if self.cancel.is_set():raise RuntimeError('CANCELLED_BY_USER')
            self.servo=Servo(self.receiver,control,self.c,state['q'])
            control=None
            self.capture_q=None
            return state['q'].copy()
        except Exception as exc:
            if control:
                errors=control.close(self.c['stop_deceleration'])
                if errors:raise RuntimeError(f'{exc}; stop acknowledgement failed: {errors}') from exc
            raise

    def release_servo(self):
        servo=self.servo
        if servo is None:return
        try:
            servo.close()
            if servo.error:raise RuntimeError(servo.error)
        finally:
            self.servo=None

    def run(self,plan):
        segments=[];outcome={'observations':[]};terminal='ERROR'
        self.capture_q=None
        self.run_directory=Path(self.c['output_directory']).expanduser()/('run_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        try:
            self.run_directory.mkdir(parents=True)
            write_json(self.run_directory/'plan.json',plan)
            self.pulse=time.monotonic()
            self.call('capture',False)
            previous_session=self.mapper['session']
            self.call('start_scan')
            self.wait(lambda:self.mapper['phase']=='SCANNING' and self.mapper['session']!=previous_session,30)
            self.phase='WARMING_UP'
            self.call('warmup')
            # Native CUDA initialization may block mapper callbacks. This allowance is
            # only before RTDEControl exists; motion retains the strict heartbeat guard.
            self.wait(lambda:self.mapper.get('warmup_state') in ('READY','ERROR'),120,check_mapper=False)
            if self.mapper.get('warmup_state')!='READY':
                raise RuntimeError('CUDA warmup failed: '+str(self.mapper.get('warmup_error')))
            self.require_mapper()
            # Revalidate after service/setup waits, before any control object is constructed.
            state=self.receiver.snapshot(self.c['feedback_timeout_sec'])
            verify_plan(plan,state['q'],self.c,self.route,self.geometry)
            q=np.asarray(plan['q_start'])
            for index,w in enumerate(self.route['waypoints']):
                self.waypoint=w['name'];self.phase='ACQUIRING_CONTROL'
                tolerance=self.c['start_tolerance_rad'] if index==0 else self.c['arrival_tolerance_rad']
                q=self.acquire_servo(q,tolerance)
                segments.append((w['name'],self.servo))
                self.phase='MOVING'
                segment=Segment(q,w['q_rad'],self.c['max_joint_speed'],self.c['max_joint_acceleration'])
                self.servo.move(segment)
                self.wait(lambda:self.servo.segment_done,segment.duration*20+20)
                self.phase='SETTLING';self.settle(segment.b)
                # No 500 Hz host servo loop or live RTDEControl while fusing RGB-D.
                # Stop at zero trajectory velocity, then verify actual rest again.
                self.phase='STOPPING_AT_VIEW';self.release_servo()
                self.phase='VERIFYING_STATIONARY';self.settle(segment.b)
                self.capture_q=segment.b.copy()
                self.phase='CAPTURING'
                epoch=json.loads(self.call('capture',True))['epoch']
                self.wait(lambda:self.mapper['epoch']==epoch and self.mapper['gate_frames']>=self.c['capture_frames'],self.c['capture_timeout_sec'])
                outcome['observations'].append(dict(name=w['name'],epoch=epoch,frames=self.mapper['gate_frames'],session=self.mapper['session']))
                self.call('capture',False)
                self.wait(lambda:not self.mapper['gate'],3)
                q=segment.b
            self.phase='EXPORTING';self.call('freeze_scan')
            self.wait(lambda:self.mapper['phase']=='FROZEN',90)
            surface=Path(self.mapper['session'])/'surface.npz'
            if not surface.is_file():raise RuntimeError('Mapper did not export final surface')
            outcome.update(surface=str(surface),completed=True)
            terminal='DONE'
        except Exception as exc:
            self.error=str(exc)
            if self.servo:
                servo=self.servo;self.servo=None
                try:servo.close()
                except Exception as stop_exc:self.error+='; '+str(stop_exc)
            try:
                self.call('capture',False,cleanup=True)
                if self.mapper and self.mapper['phase']=='SCANNING' and self.mapper['frames']>0:
                    self.call('freeze_scan',cleanup=True)
                    end=time.monotonic()+30
                    # The service may reply before the SCANNING -> FREEZING status
                    # arrives. Wait through both states before checking the file.
                    while self.mapper['phase'] in ('SCANNING','FREEZING') and time.monotonic()<end:time.sleep(.05)
                    surface=Path(self.mapper['session'])/'surface.npz'
                    outcome['partial_surface']=str(surface) if self.mapper['phase']=='FROZEN' and surface.is_file() else None
                    if outcome['partial_surface'] is None:
                        outcome['partial_export_error']='Freeze did not produce a verified surface within cleanup timeout'
            except Exception as export_exc:
                outcome['partial_export_error']=str(export_exc)
            terminal='CANCELLED' if self.error=='CANCELLED_BY_USER' else 'ERROR'
        finally:
            self.pulse=0.;self.capture_q=None;self.phase='SAVING_RESULT'
            try:
                if segments:
                    arrays=[np.asarray(servo.trace,dtype=float).reshape(-1,21) for _,servo in segments]
                    offset=0;records=[]
                    for (name,servo),samples in zip(segments,arrays):
                        records.append(dict(waypoint=name,sample_start=offset,sample_end=offset+len(samples),
                                            error=servo.error,cleanup_errors=servo.cleanup_errors,
                                            diagnostics=servo.diagnostics))
                        offset+=len(samples)
                    np.savez_compressed(self.run_directory/'servo_trace.npz',samples=np.concatenate(arrays),
                        segment_index=np.concatenate([np.full(len(a),i,dtype=np.int32) for i,a in enumerate(arrays)]),
                        columns=['host_monotonic','robot_timestamp','time_scale']+
                        ['command_'+j for j in JOINTS]+['actual_'+j for j in JOINTS]+['velocity_'+j for j in JOINTS])
                    outcome['motion_segments']=records
                outcome.update(phase=terminal,error=self.error,waypoint=self.waypoint)
                write_json(self.run_directory/'result.json',outcome)
            except Exception as exc:
                self.error=f'Result save failed: {exc}; previous error={self.error}'
                terminal='ERROR'
            self.phase=terminal

    def close(self):
        self.cancel.set()
        if self.servo:self.servo.stop.set()
        if self.worker:self.worker.join(10)
        if self.worker and self.worker.is_alive():
            self.get_logger().error('Coordinator still exiting; controller watchdog handles lost servo updates')
        self.receiver.close()


def main():
    p=argparse.ArgumentParser()
    for flag in ['mapping','control','waypoints']:p.add_argument('--'+flag,required=True)
    p.add_argument('--approved-plan',default='')
    p.add_argument('--allow-motion',choices=['true','false'],default='false')
    args,ros=p.parse_known_args();rclpy.init(args=ros)
    inputs=load_inputs(args.mapping,args.control,args.waypoints)
    receiver=Receiver(inputs[1]['robot_ip'])
    node=Scanner(inputs,receiver,allow_motion=args.allow_motion=='true',approved_plan=args.approved_plan)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
