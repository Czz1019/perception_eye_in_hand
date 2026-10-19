"""Actual ROS + nvBlox + SAGE + simulated FollowJointTrajectory; no robot connection."""
import argparse,json,os,time,threading
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from sensor_msgs.msg import JointState,Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger,SetBool
from control_msgs.action import FollowJointTrajectory
from tf2_ros import TransformBroadcaster
from realtime_nvblox.ros_publishers import transform_stamped
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_real_static.node import RealStatic
from ur7e_real_static.field import FrozenMap
from fixtures import config,INITIAL


class MockHardware(Node):
    def __init__(self,c,g):
        super().__init__('mock_hardware_only');self.c=c;self.g=g;self.q=INITIAL.copy();self.dq=np.zeros(6)
        self.send_states=True;self.active=False;self.accept_count=0;self.cancel_count=0;self.lock=threading.RLock()
        self.state_pub=self.create_publisher(JointState,c['joint_topic'],10)
        self.depth_pub=self.create_publisher(Image,c['depth_topic'],10);self.info_pub=self.create_publisher(CameraInfo,c['camera_info_topic'],10)
        self.tf=TransformBroadcaster(self);group=ReentrantCallbackGroup()
        self.create_timer(.01,self.state,callback_group=group);self.create_timer(.1,self.depth,callback_group=group)
        self.server=ActionServer(self,FollowJointTrajectory,c['trajectory_action'],execute_callback=self.execute,
                                goal_callback=self.goal,cancel_callback=self.cancel,callback_group=group)
    def goal(self,request):
        if self.active:return GoalResponse.REJECT
        self.active=True;self.accept_count+=1;return GoalResponse.ACCEPT
    def cancel(self,handle):self.cancel_count+=1;return CancelResponse.ACCEPT
    def execute(self,handle):
        request=handle.request;points=request.trajectory.points
        positions=np.array([p.positions for p in points]);times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points])
        start=request.trajectory.header.stamp.sec+request.trajectory.header.stamp.nanosec*1e-9
        from ur7e_real_static.trajectory import Trajectory
        trajectory=Trajectory(positions,times,'','',np.zeros(3),0)
        try:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    with self.lock:self.dq=np.zeros(6)
                    handle.canceled();return FollowJointTrajectory.Result(error_code=0)
                elapsed=self.get_clock().now().nanoseconds*1e-9-start
                q=trajectory.sample(max(0,elapsed));dq=(trajectory.sample(max(0,elapsed)+.001)-q)/.001
                with self.lock:self.q=q;self.dq=dq
                feedback=FollowJointTrajectory.Feedback();feedback.joint_names=list(JOINTS)
                feedback.actual.positions=q.tolist();feedback.desired.positions=q.tolist();feedback.error.positions=[0.]*6
                handle.publish_feedback(feedback)
                if elapsed>=times[-1]:
                    with self.lock:self.dq=np.zeros(6)
                    handle.succeed();return FollowJointTrajectory.Result(error_code=0)
                time.sleep(.015)
        finally:self.active=False
        return FollowJointTrajectory.Result(error_code=-1,error_string='shutdown')
    def state(self):
        if not self.send_states:return
        now=self.get_clock().now();stamp=now.to_msg()
        with self.lock:q=self.q.copy();dq=self.dq.copy()
        msg=JointState();msg.header.stamp=stamp;msg.name=list(JOINTS);msg.position=q.tolist();msg.velocity=dq.tolist();self.state_pub.publish(msg)
        poses=self.g.fk(q)
        self.tf.sendTransform([transform_stamped(poses[link],'base_link',link,now.nanoseconds) for link in [*self.g.links,self.c['camera_frame']]])
    def depth(self):
        now=self.get_clock().now();info=CameraInfo();info.header.stamp=now.to_msg();info.header.frame_id=self.c['camera_frame']
        info.width=160;info.height=90;info.k=[100.,0.,80.,0.,100.,45.,0.,0.,1.];info.p=[100.,0.,80.,0.,0.,100.,45.,0.,0.,0.,1.,0.]
        self.info_pub.publish(info)
        with self.lock:pose=self.g.fk(self.q)[self.c['camera_frame']]
        v,u=np.indices((90,160));rays=np.stack([(u-80)/100,(v-45)/100,np.ones_like(u)],-1)@pose[:3,:3].T
        z=np.divide(-.1-pose[2,3],rays[...,2],out=np.zeros((90,160)),where=abs(rays[...,2])>1e-6)
        z[(z<.2)|(z>3)]=0;z=z.astype('<f4')
        image=Image();image.header=info.header;image.width=160;image.height=90;image.encoding='32FC1';image.step=640;image.data=z.tobytes()
        self.depth_pub.publish(image)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    c,g=config(output)
    # The synthetic scene is only a floor at z=-0.1; this surveyed box is TEST-ONLY.
    c.update(execution_enabled=True,commissioned=True,commissioned_free_boxes=[dict(min=[-.95,-.95,-.04],max=[.95,.95,1.32])])
    rclpy.init();app=RealStatic(c);hardware=MockHardware(c,g);client=Node('integration_test')
    executor=MultiThreadedExecutor(num_threads=4)
    for n in [app,hardware,client]:executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start();results={}
    def wait_for(predicate,timeout=30):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.03)
        raise TimeoutError(f'phase={app.phase}, error={app.error}, capture={app.capture_error}, frames={app.capture_count}, views={len(app.views)}')
    def service(name,data=None,expect=True):
        cls=Trigger if data is None else SetBool;srv=client.create_client(cls,'/ur7e_real_static/'+name)
        assert srv.wait_for_service(timeout_sec=5)
        req=cls.Request()
        if data is not None:req.data=data
        future=srv.call_async(req);wait_for(future.done,10);response=future.result();client.destroy_client(srv)
        if expect:assert response.success,response.message
        else:assert not response.success,response.message
        return response.message
    publisher=client.create_publisher(PoseStamped,c['target_topic'],1)
    def send_goal(xyz):
        msg=PoseStamped();msg.header.frame_id='base_link';msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,xyz);msg.pose.orientation.w=1.
        publisher.publish(msg)
    try:
        wait_for(lambda:len(app.states)>40)
        assert hardware.accept_count==0;results['no_startup_motion']=True
        service('execute',expect=False);results['no_map_rejected']=True
        service('start_scan');wait_for(lambda:app.capture_count>=3,60)
        with hardware.lock:hardware.q=INITIAL+np.array([.25,0,0,0,0,0])
        wait_for(lambda:len(app.views)>=2,30)
        with hardware.lock:hardware.q=INITIAL.copy()
        time.sleep(.4)
        # Freeze requires a fresh completed frame, retry during GPU processing windows.
        deadline=time.monotonic()+15
        while True:
            srv=client.create_client(Trigger,'/ur7e_real_static/freeze_map');srv.wait_for_service(timeout_sec=2)
            future=srv.call_async(Trigger.Request());wait_for(future.done,5);reply=future.result();client.destroy_client(srv)
            if reply.success:break
            if time.monotonic()>deadline:raise RuntimeError(reply.message)
            time.sleep(.1)
        results['actual_nvblox_scan_frozen']=dict(frames=app.capture_count,views=len(app.views),map_uid=app.frozen.uid,
                                                observed_cells=int(np.count_nonzero(app.frozen.source==1)),surveyed_cells=int(np.count_nonzero(app.frozen.source==2)))
        saved=FrozenMap.load(Path(c['output_directory'])/'frozen_map.npz',app.model_id);assert saved.uid==app.frozen.uid
        goal=g.fk(INITIAL+np.array([.07,0,0,0,0,0]))[g.tool][:3,3]
        send_goal(goal);wait_for(lambda:app.phase in ['PLAN_READY','PLAN_FAILED'],130)
        assert app.phase=='PLAN_READY',app.error
        assert app.trajectory.sage_iterations>0
        service('execute',expect=False);results['scene_confirmation_required']=True
        service('confirm_scene',True)
        time.sleep(2.)  # Frozen map intentionally older than the old online 1.5 s limit.
        service('execute');wait_for(lambda:app.phase in ['GOAL_REACHED','EXECUTION_FAILED','EXECUTION_ERROR','STOPPED','STOP_UNCONFIRMED'],40)
        assert app.phase=='GOAL_REACHED',(app.phase,app.error)
        actual=g.fk(hardware.q)[g.tool][:3,3];results['static_map_goal_reached']=dict(error_m=float(np.linalg.norm(actual-goal)),sage_iterations=app.trajectory.sage_iterations)
        # Live joint feedback loss must cancel even though the static map remains valid.
        send_goal(g.fk(INITIAL)[g.tool][:3,3]);wait_for(lambda:app.phase in ['PLAN_READY','PLAN_FAILED'],130)
        assert app.phase=='PLAN_READY',app.error
        time.sleep(.3);service('execute');wait_for(lambda:app.phase=='EXECUTING',5)
        hardware.send_states=False
        wait_for(lambda:hardware.cancel_count>0,5)
        wait_for(lambda:app.goal_handle is None,5);hardware.send_states=True
        wait_for(lambda:app.phase=='STOPPED',5)
        results['joint_stream_loss_cancels']=True
        service('invalidate');assert app.frozen is None and not app.scene_confirmed;results['invalidation_discards_map']=True
        service('execute',expect=False)
        service('load_map');assert app.frozen is not None and not app.scene_confirmed;results['reload_requires_reconfirmation']=True
        service('execute',expect=False)
        results['passed']=True
    except Exception as exc:
        results.update(passed=False,error=str(exc),phase=app.phase,application_error=app.error)
        raise
    finally:
        (output/'result.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2),flush=True)
        app.close();executor.shutdown(timeout_sec=5);thread.join(timeout=3)
        for n in [app,hardware,client]:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        thread.join(timeout=3)


if __name__=='__main__':main()
