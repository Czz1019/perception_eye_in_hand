"""Simulation-only prescan and mode B: every SAGE step starts at measured q,dq."""
import argparse
import csv
import json
import sys
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from .markers import publish as publish_markers
from ur7e_nvblox.depth import transform_matrix
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_real_static.trajectory import edge_clear, model_matches
from .planning import route
from ur7e_perception_planning_core.demo import velocity_proposal
from .mapper import EsdfMapper
from .config import load_mapping
from .field import ScanPriorField
from .online import OnlineLayer,DynamicBlocked


class ClosedLoop(Node):
    def __init__(self, mapper, output, auto_start):
        super().__init__('sim_esdf_sage', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.mapper=mapper; self.c=mapper.c; self.g=mapper.geometry
        self.scan=yaml.safe_load(Path(self.c['scan_config']).read_text())
        self.output=Path(output).expanduser().resolve(); self.output.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock();self.exit=threading.Event();self.stopped=False
        self.phase='WAITING';self.error=None;self.waypoint=None;self.iterations=0
        self.goal=None;self.revision=0;self.handled=-1;self.frozen=None;self.collision_snapshot=None;self.prior=None;self.map_count=None;self.last_geometry_check=-1.
        self.map_fault_until=0.;self.online=None;self.dynamic_hold=False;self.dynamic_version=-1;self.qcmd=None;self.qnext=None;self.segment=None;self.last_log=-1.;self.last_tick=None
        self.fixture_ready=False;self.started=auto_start;self.feedback_used=[];self.fault_inject=False
        self.command=self.create_publisher(Float64MultiArray,'/forward_position_controller/commands',10)
        self.status_pub=self.create_publisher(String,'~/status',10)
        self.tcp_trace=[];self.markers=self.create_publisher(MarkerArray,'~/markers',10)
        self.create_subscription(PoseStamped,'/target_pose',self.on_goal,10)
        self.create_subscription(String,'/sim_esdf_sage/fixture_ready',self.on_fixture,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger,'~/start',self.start)
        self.create_service(Trigger,'~/stop',self.stop)
        self.create_service(Trigger,'~/invalidate',self.invalidate)
        self.create_service(Trigger,'~/test_feedback_loss',self.inject_feedback_loss)
        self.create_service(Trigger,'~/test_map_stale',self.inject_map_stale)
        self.servo_group=MutuallyExclusiveCallbackGroup();self.status_group=MutuallyExclusiveCallbackGroup()
        self.create_timer(.005,self.tick,callback_group=self.servo_group)
        self.create_timer(.25,self.status,callback_group=self.status_group)
        self.log=(self.output/'execution.csv').open('w');self.writer=csv.writer(self.log)
        self.writer.writerow(['sim_time','wall_time','phase','iteration']+
                             [prefix+j for prefix in ['actual_','command_','next_','velocity_'] for j in JOINTS])
        self.events=(self.output/'events.jsonl').open('w')
        self.feedback_log=(self.output/'sage_feedback.jsonl').open('w')
        self.pending_segments=[];self.plan_valid_until=0.;self.continuous_active=False;self.plan_generation=0;self.validated_generation=-1
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def on_fixture(self,msg):self.fixture_ready=msg.data=='esdf_sage_fixture_v1'

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def state(self):
        with self.mapper.lock: state=self.mapper.states[-1] if self.mapper.states else None
        if (self.fault_inject or state is None or self.now()-state[2]>self.c['joint_timeout']):
            raise RuntimeError('STALE_JOINT_STATE' if state is None else f'STALE_JOINT_STATE wall={time.monotonic()-state[3]:.3f}s sim={self.now()-state[2]:.3f}s injected={self.fault_inject}')
        return state

    def feedback_ready(self):
        try:return time.monotonic()-self.state()[3]<=self.scan['wall_feedback_timeout']
        except RuntimeError:return False

    def event(self,kind,**data):
        with self.lock:
            self.events.write(json.dumps(dict(kind=kind,sim_time=self.now(),wall_time=time.monotonic(),**data))+'\n')
            self.events.flush()

    def start(self,req,res):
        if self.stopped:res.success=False;res.message='Restart after a latched stop'
        else:self.started=True;res.success=True;res.message='Simulation scan requested'
        return res

    def fail(self,message):
        with self.lock:
            first=not self.stopped;previous=self.phase
            self.stopped=True;self.error=str(message);self.phase='STOPPED';self.segment=None;self.pending_segments=[]
        with self.mapper.lock:self.mapper.gate=False
        if first:self.event('stopped',reason=str(message),previous_phase=previous)

    def stop(self,req,res):
        self.fail('USER_STOP');self.revision+=1
        res.success=True;res.message='Stopped; no automatic resume';return res

    def invalidate(self,req,res):
        self.fail('MAP_INVALIDATED');self.revision+=1;self.frozen=None
        res.success=True;res.message='Map invalidated; restart and scan again';return res

    def inject_feedback_loss(self,req,res):
        # Simulation-only fault hook; no sensor or controller outside this task is changed.
        self.fault_inject=True;res.success=True;res.message='Local feedback guard fault injected';return res

    def inject_map_stale(self,req,res):
        self.map_fault_until=time.monotonic()+2.
        self.event('test_map_stale',duration=2.)
        res.success=True;res.message='Simulation-only local map outage for 2 wall seconds';return res

    def read_online(self):
        if time.monotonic()<self.map_fault_until:raise DynamicBlocked('ONLINE_MAP_STALE_INJECTED')
        return self.online.read(self.now())

    def on_goal(self,msg):
        try:
            if not msg.header.frame_id:raise ValueError('Goal frame is required')
            xyz=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if not np.isfinite(xyz).all():raise ValueError('Non-finite goal')
            if msg.header.frame_id!='base_link':
                pose=transform_matrix(self.mapper.tf_buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform)
                xyz=pose[:3,:3]@xyz+pose[:3,3]
            with self.lock:
                if self.stopped:raise ValueError('Stop latched; restart required')
                self.goal=xyz;self.revision+=1
                if self.frozen is not None:self.segment=None;self.pending_segments=[]
            self.event('target',xyz=xyz.tolist(),revision=self.revision)
        except Exception as exc:self.event('target_rejected',error=str(exc))

    def wait(self,predicate,timeout,guard=True):
        deadline=time.monotonic()+timeout
        while True:
            if self.exit.wait(.01):raise InterruptedError('SHUTDOWN')
            if self.stopped:raise InterruptedError(self.error)
            try:
                if guard:self.state()
                if predicate():return
            except RuntimeError as exc:
                if not str(exc).startswith('STALE_JOINT_STATE'):raise
                # The servo holds immediately. Its watchdog latches persistent loss.
            if time.monotonic()>deadline:raise TimeoutError(f'Timeout in {self.phase}: {self.mapper.capture_wait_reason}')
            self.mapper.lease=time.monotonic()

    def set_segment(self,a,b,duration,quintic,token=None):
        with self.lock:
            if self.stopped or token is not None and token!=self.revision:raise InterruptedError('Goal changed')
            self.plan_generation+=1
            self.qnext=np.asarray(b).copy()
            self.last_tick=self.now()
            self.segment=(np.asarray(a).copy(),self.qnext.copy(),0.,float(duration),quintic)

    def segment_done(self):
        with self.lock:return self.segment is None or self.segment[2]>=self.segment[3]

    def tick(self):
        try:
            state=self.state();q,dq=state[:2]
            if self.frozen is not None and (self.mapper.scanning or self.mapper.frozen is not self.frozen or self.mapper.capture_count!=self.map_count):
                raise RuntimeError('FROZEN_MAP_CHANGED')
            checked_map=self.frozen or getattr(self,'prior',None)
            checked_snapshot=self.collision_snapshot if self.frozen is not None else (None if checked_map is None else checked_map.snapshot)
            if self.online is not None and not self.stopped:
                try:
                    checked_snapshot,version=self.read_online()
                    with self.lock:segment=self.segment;command=self.qcmd;generation=self.plan_generation
                    if segment is not None and command is not None and (version!=self.dynamic_version or generation!=self.validated_generation or self.now()-self.last_geometry_check>=.05):
                        if not all(edge_clear(self.g,checked_snapshot,a,b,self.c['collision_margin'],self.c['self_margin']) for a,b in [(command,segment[1])]+[(v[0],v[1]) for v in self.pending_segments]):
                            raise DynamicBlocked('ACTIVE_SEGMENT_BLOCKED')
                    self.validated_generation=generation
                    self.dynamic_version=version;self.dynamic_hold=False
                except DynamicBlocked as exc:
                    with self.lock:self.segment=None;self.pending_segments=[];self.dynamic_hold=True
                    self.phase='WAIT_DYNAMIC';self.error=str(exc)
                    checked_snapshot=None
            if checked_snapshot is not None and checked_map is not None and not self.stopped and self.now()-self.last_geometry_check>=.05:
                self.last_geometry_check=self.now()
                centers=self.g.spheres(q)
                if (np.min(checked_snapshot.query(centers)-self.g.radii)<self.c['collision_margin'] or
                    self.g.self_clearance(centers)<self.c['self_margin']):
                    raise RuntimeError('MEASURED_BODY_CLEARANCE')
            with self.lock:
                if self.qcmd is None:self.qcmd=q.copy();self.qnext=q.copy()
                now=self.now()
                delta=0. if self.last_tick is None else float(np.clip(now-self.last_tick,0.,.02))
                self.last_tick=now
                command=self.qcmd.copy()
                if self.continuous_active and time.monotonic()>self.plan_valid_until:
                    self.segment=None;self.pending_segments=[]
                if self.segment is not None and self.segment[2]>=self.segment[3] and self.pending_segments:
                    self.segment=self.pending_segments.pop(0)
                if time.monotonic()-state[3]>self.scan['wall_feedback_timeout']:
                    # A paused/slow simulation holds, and only fresh feedback
                    # advances the segment. Never catch up by jumping in q.
                    delta=0.
                if self.online is not None and self.segment is not None and self.plan_generation!=self.validated_generation:
                    delta=0. # A replacement racing this tick must be rechecked first.
                if self.segment is not None and not self.stopped:
                    a,b,elapsed,duration,quintic=self.segment
                    elapsed=min(duration,elapsed+delta);u=elapsed/duration
                    h=10*u**3-15*u**4+6*u**5 if quintic else u
                    command=a+(b-a)*h
                    if np.max(abs(q-command))>.06:raise RuntimeError('TRACKING_ERROR')
                    self.segment=(a,b,elapsed,duration,quintic)
                elif np.max(abs(q-command))>.06:raise RuntimeError('TRACKING_ERROR')
                self.qcmd=command.copy()
            self.command.publish(Float64MultiArray(data=command.tolist()))
            if now-self.last_log>=.01:
                self.writer.writerow([now,time.monotonic(),self.phase,self.iterations,*q,*command,*self.qnext,*dq])
                self.last_log=now
        except Exception as exc:
            if self.qcmd is not None and not self.stopped:
                stale=str(exc).startswith('STALE_JOINT_STATE')
                with self.mapper.lock:
                    latest=self.mapper.states[-1] if self.mapper.states else None
                transient=(stale and not self.fault_inject and latest is not None
                           and time.monotonic()-latest[3]<=self.scan['wall_feedback_timeout'])
                self.last_tick=self.now()
                if not transient:self.fail(str(exc))
            # Hold last checked command on stale feedback; never extrapolate velocity.
            if self.qcmd is not None:self.command.publish(Float64MultiArray(data=self.qcmd.tolist()))

    def settled(self,target):
        state=self.state()
        return np.max(abs(state[0]-target))<.006 and self.mapper.settled_at(state[2],state[0])

    def scan_scene(self):
        self.phase='WARMING_UP'
        self.wait(lambda:self.fixture_ready and self.mapper.camera_info is not None,90,guard=False)
        response=self.mapper.warmup(None,SimpleNamespace())
        if not response.success:raise RuntimeError(response.message)
        self.wait(lambda:self.mapper.warmup_state in ('READY','ERROR'),120,guard=False)
        if self.mapper.warmup_state!='READY':raise RuntimeError(self.mapper.warmup_error)
        # Build the declared initial corridor, retaining unknown elsewhere.
        field=ScanPriorField(self.c)
        intr=SimpleNamespace(width=4,height=4,fx=1.,fy=1.,cx=2.,cy=2.)
        d,s=field.update(np.full(len(field.points),100.),np.zeros(len(field.points),bool),np.zeros((4,4)),np.eye(4),intr)
        from ur7e_real_static.field import FrozenMap
        prior=FrozenMap.create(d,s,field.origin,field.dims,self.c['grid_resolution'],dict(stamp=self.now(),epoch=0,sequence=0))
        self.prior=prior
        self.wait(self.feedback_ready,30,guard=False)
        with self.mapper.lock:self.mapper.start_scan()
        for i,target in enumerate(np.asarray(self.scan['views'])):
            self.waypoint=i;self.phase='SCANNING_MOVE';q=self.state()[0].copy()
            if not edge_clear(self.g,prior.snapshot,q,target,self.c['collision_margin'],self.c['self_margin']):
                raise RuntimeError(f'Scan {i} is outside the commissioned corridor or self clearance')
            delta=float(np.max(abs(target-q)))
            duration=max(.2,1.875*delta/self.scan['max_scan_speed'],np.sqrt(5.774*delta/self.scan['scan_acceleration']))
            self.set_segment(q,target,duration,True)
            self.wait(self.segment_done,duration*8+20)
            self.phase='SCAN_SETTLING';self.wait(lambda:self.settled(target),30)
            self.phase='SCAN_CAPTURE';self.mapper.lease=time.monotonic()
            response=self.mapper.capture(SimpleNamespace(data=True),SimpleNamespace())
            if not response.success:raise RuntimeError(response.message)
            self.wait(lambda:self.mapper.gate_frames>=self.scan['capture_frames'],60)
            self.mapper.capture(SimpleNamespace(data=False),SimpleNamespace())
            self.event('view_complete',view=i,frames=self.mapper.gate_frames,q=self.state()[0].tolist())
        self.phase='FREEZING'
        path=self.mapper.freeze_scan()
        from .ownership import prepare
        snapshot,ownership=prepare(self.mapper.frozen,self.g,self.state()[0].copy(),self.mapper.session)
        if self.c['planning_policy']=='bounded_unknown':
            from .partial_map import prepare as prepare_partial
            snapshot,policy=prepare_partial(self.mapper.frozen,self.mapper.session)
            self.event('partial_policy',**policy)
        with self.lock, self.mapper.lock:
            self.collision_snapshot=snapshot
            self.map_count=self.mapper.capture_count;self.frozen=self.mapper.frozen;self.prior=None
        self.event('robot_ownership',**ownership)
        self.event('frozen',path=path,uid=self.frozen.uid,frames=self.map_count,
                   observed_cells=int(np.count_nonzero(self.frozen.source==1)),tabletop_cells=int(np.count_nonzero(self.frozen.source==2)),scan_prior_cells=0)
        if not edge_clear(self.g,self.collision_snapshot,self.state()[0],self.state()[0],self.c['collision_margin'],self.c['self_margin']):
            raise RuntimeError('FROZEN_START_NOT_OBSERVED_OR_BLOCKED')
        self.phase='MAP_READY'

    def online_snapshot(self):
        waiting=False
        while True:
            if self.stopped or self.exit.is_set():raise InterruptedError('Stopped')
            try:
                result=self.read_online()
                if waiting:self.event('online_recovered')
                return result
            except DynamicBlocked:
                if self.scan.get('online_engine','v1')!='v2':raise
                with self.lock:self.segment=None
                self.phase='WAIT_DYNAMIC';self.error='ONLINE_MAP_STALE'
                if not waiting:self.event('online_wait');waiting=True
                self.exit.wait(.02)

    def reach(self,task,goal,token):
        from .continuous import reach
        return reach(self,task,goal,token)

    def run(self):
        task=None
        try:
            self.wait(lambda:self.started and len(self.mapper.states)>5 and self.now()>0,120,guard=False)
            if self.mapper.frozen is None:self.scan_scene()
            else:
                # Explicit debug replay in the same unchanged isolated fixture.
                self.wait(lambda:self.fixture_ready and self.feedback_ready(),60,guard=False)
                from .ownership import prepare
                if self.c['planning_policy']=='bounded_unknown':
                    from .partial_map import load_snapshot
                    snapshot=load_snapshot(self.mapper.frozen,self.mapper.session/'partial_policy.npz')
                    ownership=json.loads((self.mapper.session/'robot_ownership.json').read_text())
                    reference=np.array(ownership['freeze_q']);current=self.state()[0].copy()
                    if not edge_clear(self.g,snapshot,current,reference,self.c['collision_margin'],self.c['self_margin']):raise ValueError('Replay alignment blocked')
                    self.prior=SimpleNamespace(snapshot=snapshot)
                    self.set_segment(current,reference,1.,True);self.wait(self.segment_done,10);self.wait(lambda:self.settled(reference),10)
                    if np.max(abs(self.state()[0]-reference))>.0005:raise ValueError('Replay alignment failed')
                    info=self.mapper.frozen.metadata['camera_info']
                    self.mapper.camera_signature=(info['width'],info['height'],info['frame'],tuple(info['k']),tuple(info['p']),'16UC1')
                else:snapshot,ownership=prepare(self.mapper.frozen,self.g,self.state()[0].copy(),self.mapper.session)
                with self.lock:
                    self.collision_snapshot=snapshot
                    self.map_count=self.mapper.capture_count;self.frozen=self.mapper.frozen
                self.event('robot_ownership',**ownership)
                if not edge_clear(self.g,self.collision_snapshot,self.state()[0],self.state()[0],self.c['collision_margin'],self.c['self_margin']):
                    raise RuntimeError('REPLAY_START_BLOCKED')
                self.event('map_replay',uid=self.frozen.uid)

            if self.scan.get('online_obstacles',False):
                if self.c['planning_policy']!='bounded_unknown':raise ValueError('Online layer requires explicit partial policy')
                layer_class=OnlineLayer
                if self.scan.get('online_engine','v1')=='v2':
                    from .online_v2 import OnlineLayer as layer_class
                self.online=layer_class(self.mapper,self.collision_snapshot,self.output);self.mapper.online=self.online
                self.phase='ONLINE_WARMUP';self.wait(lambda:self.online.frames>2,30)
                self.event('online_ready',static_uid=self.frozen.uid)
            self.phase='INITIALIZING_SAGE'
            from .optimizer import Planner
            task=Planner(self.c,self.output/'sage',self.mapper.session)
            self.wait(self.feedback_ready,30,guard=False)
            q,dq=self.state()[:2]
            task.warm(q,dq,self.now(),self.collision_snapshot,self.g.fk(q)['tool0'][:3,3])
            if self.scan.get('hold_after_scan',False):
                self.phase='ONLINE_READY' if self.online else 'MAP_READY'
                self.error=None
                self.wait(lambda:self.goal is not None,1800)
            if self.goal is None:self.goal=np.array(self.scan['default_goal']);self.revision+=1
            while not self.exit.wait(.03) and not self.stopped:
                if self.handled==self.revision:continue
                with self.lock:token=self.revision;goal=self.goal.copy();self.handled=token
                try:self.reach(task,goal,token)
                except InterruptedError:continue
                except Exception as exc:
                    if self.online is not None and not self.stopped and (isinstance(exc,DynamicBlocked) or isinstance(exc,ValueError) and str(exc).startswith(('No observed','No checked'))):
                        with self.lock:self.segment=None;self.error=str(exc);self.phase='WAIT_DYNAMIC';self.handled=-1
                        self.event('dynamic_wait',reason=str(exc),revision=token,map_version=self.online.version)
                        self.exit.wait(1.)
                        continue
                    with self.lock:self.segment=None
                    if not self.stopped:
                        self.error=str(exc);self.phase='GOAL_REJECTED';self.event('goal_rejected',error=str(exc),revision=token)
        except Exception as exc:
            if not self.exit.is_set():self.fail(str(exc));self.event('failure',error=str(exc))
        finally:
            if task is not None:task.close()

    def status(self):
        value=dict(run_id=str(self.output.resolve()),online=None if self.online is None else self.online.status(),planning_policy=self.c['planning_policy'],phase=self.phase,error=self.error,view=self.waypoint,sage_iterations=self.iterations,
                   map_uid=None if self.frozen is None else self.frozen.uid,frames=self.mapper.capture_count,
                   map_frozen=self.frozen is not None and not self.mapper.scanning,
                   capture_wait=self.mapper.capture_wait_reason,capture_error=self.mapper.capture_error,
                   model='measured real camera extrinsics',mode='continuous final-goal SAGE with asynchronous checked execution',stopped=self.stopped)
        self.status_pub.publish(String(data=json.dumps(value)))
        (self.output/'status.json').write_text(json.dumps(value,indent=2))
        self.log.flush()
        if self.mapper.states:publish_markers(self,self.mapper.states[-1][0])

    def close(self):
        self.exit.set();self.fail('SHUTDOWN');self.thread.join(15)
        if self.online is not None:self.online.close()
        self.log.flush();self.events.flush();self.feedback_log.flush()
        (self.output/'sage_feedback.json').write_text(json.dumps(self.feedback_used))


def main():
    # The default 5 ms Python quantum equals an entire 200 Hz servo period.
    # NumPy releases the GIL frequently during depth filtering; a shorter
    # process-local quantum prevents ROS callbacks from delaying every ufunc.
    sys.setswitchinterval(.00005)
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--auto-start',default='true');parser.add_argument('--test-load-map',default='');args,ros=parser.parse_known_args()
    rclpy.init(args=ros);mapper=EsdfMapper(load_mapping(args.config),load=args.test_load_map);node=ClosedLoop(mapper,args.output,args.auto_start=='true')
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(mapper);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        # A process-group interrupt followed by launch SIGINT must not abort cleanup.
        import signal
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.close();mapper.close();executor.shutdown();node.destroy_node();mapper.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
