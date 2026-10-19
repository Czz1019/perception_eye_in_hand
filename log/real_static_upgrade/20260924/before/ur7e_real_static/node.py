import argparse
from collections import deque
import json
from pathlib import Path
import threading
import signal
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import Image,CameraInfo,JointState,PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker,MarkerArray
from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.ros_publishers import esdf_cloud
from ur7e_nvblox.depth import transform_matrix
from ur7e_perception_planning_core.geometry import JOINTS
from .config import load_config,identity
from .model import Geometry
from .field import FrozenMap
from .trajectory import model_matches,validate,parameterize
from .state import execution_gate


class RealStatic(Node):
    def __init__(self,c):
        super().__init__('ur7e_real_static',parameter_overrides=[Parameter('use_sim_time',value=c['use_sim_time'])])
        self.c=c;self.model_id=identity(c);self.geometry=Geometry(c['robot_urdf'],c['collision_spheres'],c['tool_frame'],c['camera_frame'])
        if c['max_joint_speed']>self.geometry.velocity_limits.min():raise ValueError('Configured speed exceeds URDF limit')
        self.directory=Path(c['output_directory']).expanduser();self.directory.mkdir(parents=True,exist_ok=True)
        (self.directory/'resolved_config.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
        self.last_event=None;self.preview_marker=None
        self.lock=threading.RLock();self.stop_event=threading.Event();self.frames=LatestSlot();self.states=deque(maxlen=2000)
        self.phase='IDLE';self.error=None;self.capture_error=None;self.scanning=False;self.epoch=0;self.generation=0
        self.views=[];self.capture_count=0;self.latest_field=None;self.camera_info=None;self.camera_signature=None
        self.frozen=None;self.trajectory=None;self.scene_confirmed=False;self.goal_handle=None;self.sending=False
        self.cancel_requested=False;self.action_uncertain=False;self.motion_owned=False;self.stop_wait_deadline=0.;self.plan_thread=None;self.last_clock=None;self.last_cloud=0.;self.last_feedback=0.
        self.desired_feedback=None;self.action_deadline=0.;self.tf_ok=False;self.tf_checked=0.;self.cancel_deadline=0.
        self.tf_buffer=Buffer(cache_time=Duration(seconds=10),node=self);self.tf_listener=TransformListener(self.tf_buffer,self)
        sensor=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState,c['joint_topic'],self.on_state,sensor)
        self.create_subscription(CameraInfo,c['camera_info_topic'],self.on_info,sensor)
        self.create_subscription(Image,c['depth_topic'],self.on_depth,sensor)
        self.create_subscription(PoseStamped,c['target_topic'],self.on_goal,1)
        self.status_pub=self.create_publisher(String,'~/status',1)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub=self.create_publisher(PointCloud2,'~/esdf',latched)
        self.surface_pub=self.create_publisher(PointCloud2,'~/surface_esdf',latched)
        self.preview_pub=self.create_publisher(JointTrajectory,'~/preview_trajectory',latched)
        self.marker_pub=self.create_publisher(MarkerArray,'~/markers',latched)
        self.client=ActionClient(self,FollowJointTrajectory,c['trajectory_action'])
        for name,callback in [('start_scan',self.start_scan),('freeze_map',self.freeze_map),('load_map',self.load_map),
                              ('execute',self.execute),('stop',self.stop_motion),('invalidate',self.invalidate)]:
            self.create_service(Trigger,'~/'+name,self.service(callback))
        self.create_service(SetBool,'~/confirm_scene',self.confirm_scene)
        self.create_timer(.05,self.watchdog);self.create_timer(.5,self.status)
        from .capture import worker
        self.capture_thread=threading.Thread(target=worker,args=(self,),daemon=True);self.capture_thread.start()
        self.get_logger().info('Ready for stationary-view scanning; no automatic startup motion. execution_enabled='+str(c['execution_enabled']))

    def service(self,callback):
        def run(request,response):
            try:
                with self.lock:message=callback()
                response.success=True;response.message=str(message)
            except Exception as exc:
                response.success=False;response.message=str(exc)
            return response
        return run

    def idle_required(self):
        if self.goal_handle is not None or self.sending or self.motion_owned or self.phase in ('STOPPING','STOP_UNCONFIRMED'):
            raise ValueError('Motion/cancellation outstanding; wait for terminal action result')
        if self.plan_thread is not None and self.plan_thread.is_alive():raise ValueError('Planning is active; stop first and wait')

    def on_info(self,msg):
        with self.lock:
            self.camera_info=msg
            if self.frozen is not None:
                saved=self.frozen.metadata.get('camera_info')
                actual=self.info_record(msg)
                if saved is not None and actual!=saved:
                    self.scene_confirmed=False;self.trajectory=None;self.generation+=1
                    self.cancel('CAMERA_CALIBRATION_CHANGED');self.error='CameraInfo differs from frozen map'

    @staticmethod
    def info_record(msg):
        return dict(width=msg.width,height=msg.height,frame=msg.header.frame_id,k=list(msg.k),p=list(msg.p))

    def on_depth(self,msg):
        with self.lock:
            if not self.scanning or self.camera_info is None:return
            ns=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            if ns<=0 or abs(now-ns)>self.c['frame_timeout']:return
            self.frames.put((self.epoch,msg,self.camera_info,ns,time.monotonic()))

    def on_state(self,msg):
        try:
            ids=[msg.name.index(j) for j in JOINTS];q=np.array([msg.position[i] for i in ids])
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp<=0 or not np.isfinite(q).all():return
            with self.lock:
                if self.states and stamp<=self.states[-1][2]:return
                if len(msg.velocity)==len(msg.name):dq=np.array([msg.velocity[i] for i in ids])
                elif self.states:dq=(q-self.states[-1][0])/(stamp-self.states[-1][2])
                else:dq=np.zeros(6)
                if not np.isfinite(dq).all():return
                self.states.append((q,dq,stamp,time.monotonic()))
        except (ValueError,IndexError):return

    def state_at(self,t):
        with self.lock:samples=list(self.states)
        for a,b in zip(samples,samples[1:]):
            if a[2]<=t<=b[2] and b[2]-a[2]<=.1:
                alpha=(t-a[2])/(b[2]-a[2]);return a[0]+alpha*(b[0]-a[0]),a[1]+alpha*(b[1]-a[1])
        return None

    def settled_at(self,t,q):
        with self.lock:history=[s for s in self.states if t-self.c['settle_seconds']-.1<=s[2]<=t]
        return (bool(history) and t-history[0][2]>=self.c['settle_seconds']
                and all(np.max(abs(s[1]))<=self.c['settle_speed'] and np.max(abs(s[0]-q))<.01 for s in history))

    def start_scan(self):
        self.idle_required();self.generation+=1;self.epoch+=1;self.frozen=None;self.trajectory=None
        self.scene_confirmed=False;self.views=[];self.capture_count=0;self.latest_field=None;self.camera_signature=None
        self.scanning=True;self.phase='SCANNING';self.error=None;self.capture_error=None
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        return 'Move using independently commissioned teach-pendant paths; stationary views are fused automatically'

    def freeze_map(self):
        self.idle_required()
        if not self.scanning or self.latest_field is None:raise ValueError('No active scan data')
        if self.capture_count<self.c['min_frames'] or len(self.views)<self.c['min_views']:raise ValueError('Insufficient frames/views')
        d,s,o,dims,stamp,seq,received,captured_info=self.latest_field
        if abs(self.get_clock().now().nanoseconds*1e-9-stamp)>self.c['frame_timeout'] or time.monotonic()-received>self.c['frame_timeout']:
            raise ValueError('Wait stationary for a fresh completed depth frame before freeze')
        self.frozen=FrozenMap.create(d,s,o,dims,self.c['grid_resolution'],dict(identity=self.model_id,stamp=stamp,epoch=self.epoch,
                      sequence=seq,views=[v.tolist() for v in self.views],camera_info=captured_info))
        self.frozen.save(self.directory/'frozen_map.npz');self.scanning=False;self.phase='MAP_READY';self.scene_confirmed=False
        return 'Map frozen and saved; confirm unchanged static scene before execution. Unknown space remains blocked.'

    def load_map(self):
        self.idle_required()
        if self.scanning:raise ValueError('Finish/invalidate active scan first')
        self.frozen=FrozenMap.load(self.directory/'frozen_map.npz',self.model_id);self.trajectory=None
        self.scene_confirmed=False;self.generation+=1;self.phase='MAP_READY'
        snap=self.frozen.snapshot
        z,y,x=np.indices(snap.distance.shape);points=snap.origin+snap.resolution*np.stack([x,y,z],-1).reshape(-1,3)
        self.publish_cloud(points,snap.distance.reshape(-1),snap.stamp,force=True)
        return 'Map loaded; current scene must be explicitly confirmed and live TF checked'

    def confirm_scene(self,request,response):
        with self.lock:
            if request.data and self.frozen is None:
                response.success=False;response.message='No frozen map';return response
            self.scene_confirmed=request.data
            if not request.data:self.generation+=1;self.trajectory=None;self.cancel('SCENE_INVALID')
        response.success=True;response.message='Static scene confirmation='+str(request.data);return response

    def on_goal(self,msg):
        try:
            with self.lock:
                # New target never silently overwrites an executing action.
                if self.goal_handle is not None or self.sending:
                    self.generation+=1;self.trajectory=None;self.scene_confirmed=False;self.cancel('NEW_GOAL_STOP');raise ValueError('Active motion canceled; resend goal after stop acknowledgement')
                self.generation+=1;token=self.generation;self.trajectory=None
                if self.plan_thread is not None and self.plan_thread.is_alive():raise ValueError('Previous planning canceled; resend when idle')
                self.idle_required()
                if self.frozen is None or self.scanning:raise ValueError('Freeze/load a map before setting a target')
                state=self.current_state()
                if np.max(abs(state[1]))>self.c['settle_speed']:raise ValueError('Robot moving; stop before planning')
                frame=msg.header.frame_id
                if not frame:raise ValueError('Goal frame must be explicit')
                xyz=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
                if not np.isfinite(xyz).all():raise ValueError('Non-finite target')
                if frame!='base_link':
                    transform=self.tf_buffer.lookup_transform('base_link',frame,Time.from_msg(msg.header.stamp))
                    pose=transform_matrix(transform.transform);xyz=pose[:3,:3]@xyz+pose[:3,3]
                frozen=self.frozen;start=state[0].copy();self.phase='PLANNING';self.error=None
                def work():
                    try:
                        from .planning import plan
                        candidate=plan(self.geometry,frozen,start,xyz,self.c,self.directory/'planning',lambda:self.stop_event.is_set() or token!=self.generation)
                        with self.lock:
                            if token!=self.generation:return
                            self.trajectory=candidate;self.phase='PLAN_READY'
                            np.savez_compressed(self.directory/'trajectory.npz',positions=candidate.positions,times=candidate.times,
                                                goal=xyz,map_uid=frozen.uid,identity=self.model_id,sage_iterations=candidate.sage_iterations)
                            self.preview_pub.publish(self.trajectory_message(candidate));self.publish_preview(candidate)
                    except Exception as exc:
                        with self.lock:
                            if token==self.generation:self.error=str(exc);self.phase='PLAN_FAILED'
                    finally:
                        with self.lock:
                            if token!=self.generation and self.phase=='PLANNING':self.phase='MAP_READY' if self.frozen else 'IDLE'
                self.plan_thread=threading.Thread(target=work,daemon=True);self.plan_thread.start()
        except Exception as exc:
            self.error=str(exc);self.get_logger().warning(str(exc))

    def current_state(self):
        state=self.states[-1] if self.states else None;now=self.get_clock().now().nanoseconds*1e-9
        if state is None or time.monotonic()-state[3]>self.c['joint_timeout'] or abs(now-state[2])>self.c['joint_timeout']:
            raise ValueError('Stale/missing joint feedback')
        return state

    def live_tf_matches(self,state):
        try:
            stamp=Time(seconds=state[2]);poses={link:transform_matrix(self.tf_buffer.lookup_transform('base_link',link,stamp).transform)
                                               for link in [*self.geometry.links,self.c['camera_frame']]}
            return model_matches(self.geometry,state[0],poses)
        except TransformException:return False

    def recent_tf_matches(self):
        now=self.get_clock().now().nanoseconds*1e-9
        samples=[s for s in self.states if .03<=now-s[2]<=.15]
        return any(self.live_tf_matches(s) for s in reversed(samples))

    @staticmethod
    def trajectory_message(trajectory):
        msg=JointTrajectory();msg.joint_names=list(JOINTS)
        # Providing p/v/a=0 at knots specifies the checked rest-to-rest quintic spline.
        for q,t in zip(trajectory.positions,trajectory.times):
            p=JointTrajectoryPoint();p.positions=q.tolist();p.velocities=[0.]*6;p.accelerations=[0.]*6
            p.time_from_start=Duration(seconds=float(t)).to_msg();msg.points.append(p)
        return msg

    def execute(self):
        self.idle_required()
        state=self.current_state();tf_ok=self.recent_tf_matches()
        if not self.settled_at(state[2],state[0]):raise ValueError('Wait for stationary joint history before execution')
        calibration_ok=(self.camera_info is not None and self.frozen is not None and self.info_record(self.camera_info)==self.frozen.metadata.get('camera_info'))
        execution_gate(self.c,self.frozen,self.trajectory,state,self.get_clock().now().nanoseconds*1e-9,time.monotonic(),self.scene_confirmed,tf_ok,calibration_ok)
        if identity(self.c)!=self.model_id:raise ValueError('Model/config files changed; restart and rebuild map')
        validate(self.geometry,self.frozen,self.trajectory,self.c)
        if not self.client.server_is_ready():raise ValueError('Trajectory action server is not available')
        # Revalidate the small change from planned to actual start, then time-parameterize again.
        points=self.trajectory.positions.copy();points[0]=state[0]
        trajectory=parameterize(points,self.c['max_joint_speed'],self.c['max_joint_acceleration'],self.frozen.uid,self.model_id,self.trajectory.goal,self.trajectory.sage_iterations)
        validate(self.geometry,self.frozen,trajectory,self.c)
        goal=FollowJointTrajectory.Goal();goal.trajectory=self.trajectory_message(trajectory)
        goal.trajectory.header.stamp=(self.get_clock().now()+Duration(seconds=.2)).to_msg()
        goal.path_tolerance=[JointTolerance(name=j,position=self.c['tracking_tolerance']) for j in JOINTS]
        goal.goal_tolerance=[JointTolerance(name=j,position=.01,velocity=self.c['settle_speed']) for j in JOINTS]
        goal.goal_time_tolerance=Duration(seconds=3.).to_msg()
        self.current_state()  # Recheck time budget after collision validation; stale feedback cannot execute.
        self.cancel_deadline=0.
        self.trajectory=trajectory;self.motion_owned=True;self.sending=True;self.cancel_requested=False;self.phase='SENDING'
        self.last_feedback=time.monotonic();self.desired_feedback=None;self.action_deadline=time.monotonic()+max(30.,trajectory.times[-1]*5+10)
        token=self.generation
        self.client.send_goal_async(goal,feedback_callback=self.feedback).add_done_callback(lambda f:self.accepted(f,token))
        return 'Trajectory sent for acceptance; monitor status and action result'

    def accepted(self,future,token):
        with self.lock:
            self.sending=False
            try:handle=future.result()
            except Exception as exc:
                self.action_uncertain=True;self.phase='STOP_UNCONFIRMED';self.error=f'Action acceptance unknown: {exc}';return
            if not handle.accepted:self.motion_owned=False;self.phase='EXECUTION_REJECTED';self.error='Controller rejected trajectory';return
            self.goal_handle=handle;handle.get_result_async().add_done_callback(self.finished)
            if token!=self.generation or self.cancel_requested:self.cancel('STOPPING')
            else:self.phase='EXECUTING'

    def feedback(self,message):
        with self.lock:
            f=message.feedback
            try:
                order=[f.joint_names.index(j) for j in JOINTS]
                error=np.array([f.error.positions[i] for i in order])
                if not np.isfinite(error).all() or np.max(abs(error))>self.c['tracking_tolerance']:
                    self.cancel('TRACKING_ERROR')
                self.last_feedback=time.monotonic()
            except (ValueError,IndexError):self.cancel('INVALID_ACTION_FEEDBACK')

    def finished(self,future):
        with self.lock:
            self.goal_handle=None;self.cancel_deadline=0.
            try:
                result=future.result()
            except Exception as exc:
                self.action_uncertain=True;self.phase='STOP_UNCONFIRMED';self.error=f'Action terminal result unknown: {exc}';return
            try:
                if self.cancel_requested:
                    self.phase='STOPPING';self.stop_wait_deadline=time.monotonic()+3.;self.trajectory=None
                elif result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0:
                    state=self.current_state();xyz=self.geometry.fk(state[0])[self.c['tool_frame']][:3,3]
                    if np.linalg.norm(xyz-self.trajectory.goal)>.02:raise ValueError('Controller success but measured goal error exceeds 2 cm')
                    self.phase='GOAL_REACHED';self.motion_owned=False
                else:self.cancel('CONTROLLER_FAILED: '+result.result.error_string)
            except Exception as exc:self.cancel('EXECUTION_RESULT_CHECK_FAILED: '+str(exc))

    def cancel(self,reason):
        if self.action_uncertain:
            self.phase='STOP_UNCONFIRMED';return
        self.error=reason;self.phase='STOPPING';self.cancel_requested=True
        if self.goal_handle is not None:
            if not self.cancel_deadline:
                self.cancel_deadline=time.monotonic()+3.
                self.goal_handle.cancel_goal_async()
        elif not self.sending:
            if self.motion_owned:
                self.phase='STOPPING'
                if not self.stop_wait_deadline:self.stop_wait_deadline=time.monotonic()+3.
            else:self.phase='STOPPED'
            self.cancel_deadline=0.

    def stop_motion(self):
        self.generation+=1;self.scanning=False;self.trajectory=None;self.scene_confirmed=False
        self.cancel('USER_STOP');return 'Stop requested; terminal action acknowledgement is required'

    def invalidate(self):
        self.stop_motion();self.frozen=None;self.epoch+=1;return 'Map invalidated and stop requested'

    def watchdog(self):
        with self.lock:
            if self.action_uncertain:return
            now=self.get_clock().now().nanoseconds*1e-9
            if self.last_clock is not None and now<self.last_clock:
                self.invalidate();self.states.clear();self.error='CLOCK_RESET'
            self.last_clock=now
            if self.goal_handle is None and not self.sending:
                if self.motion_owned and self.cancel_requested:
                    try:
                        state=self.current_state()
                        if self.settled_at(state[2],state[0]):
                            self.motion_owned=False;self.phase='STOPPED';self.stop_wait_deadline=0.
                    except ValueError:pass
                    if self.motion_owned and self.stop_wait_deadline and time.monotonic()>self.stop_wait_deadline:
                        self.phase='STOP_UNCONFIRMED';self.error='Cancellation acknowledged but stationary feedback not confirmed'
                return
            if time.monotonic()-self.last_feedback>1. or time.monotonic()>self.action_deadline:
                self.cancel('ACTION_FEEDBACK_TIMEOUT')
            try:
                state=self.current_state()
                if self.frozen is not None:
                    from .trajectory import edge_clear
                    if not edge_clear(self.geometry,self.frozen.snapshot,state[0],state[0],self.c['collision_margin'],self.c['self_margin']):
                        self.cancel('ACTUAL_STATE_OUTSIDE_CHECKED_FREE_SPACE')
                # Give TF its normal publication lag; validate a recent buffered feedback sample.
                if time.monotonic()-self.tf_checked>.1:
                    self.tf_ok=self.recent_tf_matches();self.tf_checked=time.monotonic()
                    if not self.tf_ok:self.cancel('TF_MODEL_MISMATCH_OR_MISSING')
            except ValueError:self.cancel('STALE_JOINT_STATE')
            if self.cancel_deadline and time.monotonic()>self.cancel_deadline:
                self.phase='STOP_UNCONFIRMED';self.error='Controller cancellation not confirmed; operator must stop using robot controls'

    def publish_cloud(self,points,distance,stamp,force=False):
        if not force and time.monotonic()-self.last_cloud<1.:return
        self.last_cloud=time.monotonic()
        ns=int(stamp*1e9)
        visible=np.isfinite(distance)&(distance>0)
        self.cloud_pub.publish(esdf_cloud(points[visible],distance[visible],'base_link',ns,.15,unknown_distance_m=100.))

    def publish_preview(self,trajectory):
        from geometry_msgs.msg import Point
        marker=Marker();marker.header.frame_id='base_link';marker.ns='validated_tcp';marker.id=0
        marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.scale.x=.006;marker.color.g=1.;marker.color.a=1.
        for t in np.linspace(0,trajectory.times[-1],200):
            xyz=self.geometry.fk(trajectory.sample(t))[self.c['tool_frame']][:3,3]
            marker.points.append(Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2])))
        self.preview_marker=marker
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def status(self):
        with self.lock:
            data=dict(phase=self.phase,error=self.error,capture_error=self.capture_error,frames=self.capture_count,views=len(self.views),
                      map_uid=None if self.frozen is None else self.frozen.uid,scene_confirmed=self.scene_confirmed,
                      execution_enabled=self.c['execution_enabled'],planning_active=self.plan_thread is not None and self.plan_thread.is_alive(),
                      trajectory_seconds=None if self.trajectory is None else float(self.trajectory.times[-1]),
                      sage_iterations=None if self.trajectory is None else self.trajectory.sage_iterations)
        self.status_pub.publish(String(data=json.dumps(data)))
        markers=[]
        if self.states:
            for i,(point,radius) in enumerate(zip(self.geometry.spheres(self.states[-1][0]),self.geometry.radii)):
                m=Marker();m.header.frame_id='base_link';m.ns='collision_model';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,point);m.pose.orientation.w=1.
                m.scale.x=m.scale.y=m.scale.z=float(radius*2);m.color.b=1.;m.color.g=.5;m.color.a=.3;markers.append(m)
        if self.trajectory is not None and self.preview_marker is not None:markers.append(self.preview_marker)
        else:
            m=Marker();m.header.frame_id='base_link';m.ns='validated_tcp';m.id=0;m.action=Marker.DELETE;markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))
        (self.directory/'status.json').write_text(json.dumps(data,indent=2))
        event=(data['phase'],data['error'],data['capture_error'],data['map_uid'],data['scene_confirmed'])
        if event!=self.last_event:
            self.last_event=event
            with (self.directory/'events.jsonl').open('a') as stream:stream.write(json.dumps(dict(wall_time=time.time(),**data))+'\n')

    def close(self):
        self.stop_event.set();self.frames.close();self.generation+=1
        if self.goal_handle is not None and rclpy.ok():self.goal_handle.cancel_goal_async()
        self.capture_thread.join(timeout=3)
        if self.plan_thread:self.plan_thread.join(timeout=3)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);args,ros_args=parser.parse_known_args()
    from rclpy.signals import SignalHandlerOptions
    c=load_config(args.config);rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    node=None;executor=SingleThreadedExecutor()
    try:
        node=RealStatic(c);executor.add_node(node);executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:
            if rclpy.ok() and (node.goal_handle is not None or node.sending or node.motion_owned):
                node.stop_motion();deadline=time.monotonic()+3.
                while time.monotonic()<deadline and (node.goal_handle is not None or node.sending or node.motion_owned):
                    executor.spin_once(timeout_sec=.05)
                if node.goal_handle is not None or node.motion_owned:
                    node.get_logger().error('Stop unconfirmed on shutdown; use robot controls to stop motion')
            node.close()
        executor.shutdown()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
