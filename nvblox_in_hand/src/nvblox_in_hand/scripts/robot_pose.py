#!/usr/bin/env python3
"""Observe robot TF at camera-info timestamps; never creates or changes TF."""
from collections import deque
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


class RobotPose(Node):
    def __init__(self):
        super().__init__('robot_camera_pose')
        self.frame=self.declare_parameter('global_frame','base_link').value
        self.limit=self.declare_parameter('path_max_poses',3000).value
        self.enabled=self.declare_parameter('publish_path',True).value
        self.tf=Buffer(cache_time=Duration(seconds=15.));self.listener=TransformListener(self.tf,self)
        self.queue=deque(maxlen=10);self.poses=deque(maxlen=max(1,self.limit))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub=self.create_publisher(PoseStamped,'/nvblox_in_hand/camera_pose',1)
        self.path_pub=self.create_publisher(Path,'/nvblox_in_hand/camera_path',qos)
        self.health=self.create_publisher(String,'/nvblox_in_hand/pose_health',1)
        self.create_subscription(CameraInfo,'camera_info',self.info,qos_profile_sensor_data)
        self.create_timer(.02,self.tick);self.create_timer(1.,self.report)
        self.seen=0;self.resolved=0;self.expired=0;self.last_input=0.;self.last_stamp=-1;self.last_error='waiting_for_camera_info'
        self.last_path_wall=0.

    def info(self,msg):
        self.seen+=1;self.last_input=time.monotonic()
        if len(self.queue)==self.queue.maxlen:self.expired+=1
        self.queue.append((msg,time.monotonic()))

    def tick(self):
        while self.queue:
            msg,received=self.queue[0]
            if msg.header.stamp.sec==0 and msg.header.stamp.nanosec==0:
                self.queue.popleft();self.expired+=1;self.last_error='zero_image_stamp';continue
            try:
                tf=self.tf.lookup_transform(self.frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
            except TransformException as error:
                self.last_error=str(error)
                if time.monotonic()-received<.5:return
                self.queue.popleft();self.expired+=1;continue
            self.queue.popleft()
            stamp=Time.from_msg(msg.header.stamp).nanoseconds
            if stamp<=self.last_stamp:continue
            self.last_stamp=stamp;self.resolved+=1;self.last_error=''
            pose=PoseStamped();pose.header=msg.header;pose.header.frame_id=self.frame
            pose.pose.position.x=tf.transform.translation.x
            pose.pose.position.y=tf.transform.translation.y
            pose.pose.position.z=tf.transform.translation.z
            pose.pose.orientation=tf.transform.rotation;self.pose_pub.publish(pose)
            # Limit visualization traffic, preserving image-time robot pose without affecting mapping.
            if self.enabled and time.monotonic()-self.last_path_wall>=.2:
                self.poses.append(pose);path=Path();path.header=pose.header;path.poses=list(self.poses)
                self.path_pub.publish(path);self.last_path_wall=time.monotonic()

    def report(self):
        msg=String();msg.data=json.dumps(dict(camera_info_seen=self.seen,image_time_tf_resolved=self.resolved,
            expired=self.expired,input_age_wall_sec=None if not self.last_input else time.monotonic()-self.last_input,
            last_error=self.last_error,pose_source='robot_tf_at_camera_info_stamp',mapping_frames_counted=False))
        self.health.publish(msg)


def main():
    rclpy.init();node=RobotPose()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
