#!/usr/bin/env python3
"""Isolated synthetic ROS test: exact-time TF failure must never fall back to latest.
Requires a separate ROS_DOMAIN_ID with no Gazebo/mapper running.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_prefix
from realtime_nvblox.ros_publishers import ns_to_stamp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='artifacts/gazebo_perception/rejection.json')
    args=parser.parse_args()
    rclpy.init()
    node=Node('synthetic_depth_verification')
    if node.get_publishers_info_by_topic('/clock'):
        raise RuntimeError('Use a separate ROS_DOMAIN_ID without Gazebo')
    best=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    clock=node.create_publisher(Clock,'/clock',best)
    camera=node.create_publisher(CameraInfo,'/camera/depth/camera_info',best)
    depth=node.create_publisher(Image,'/camera/depth/image_rect_raw',best)
    tf=TransformBroadcaster(node)
    reports=[]
    node.create_subscription(String,'/realtime_nvblox/stats',lambda m:reports.append(json.loads(m.data)),10)
    executable=Path(get_package_prefix('realtime_nvblox'))/'lib/realtime_nvblox/gazebo_ros_mapper'
    log_path=Path(args.output).with_suffix('.log');log_path.parent.mkdir(parents=True,exist_ok=True)
    with log_path.open('w') as log:
        child=subprocess.Popen([str(executable)],stdout=log,stderr=subprocess.STDOUT)
        try:
            start=time.monotonic()
            while time.monotonic()-start < 12:
                if child.poll() is not None:
                    raise RuntimeError('Mapper exited; see '+str(log_path))
                clock.publish(Clock(clock=ns_to_stamp(10_000_000_000)))
                info=CameraInfo(width=8,height=6)
                info.header.frame_id='camera_depth_optical_frame'
                info.p=[8.,0.,3.5,0.,0.,8.,2.5,0.,0.,0.,1.,0.]
                camera.publish(info)
                # Latest exists at 9 seconds, but input requires exactly 9.9 seconds.
                transform=TransformStamped()
                transform.header.frame_id='base_link'
                transform.child_frame_id=info.header.frame_id
                transform.header.stamp=ns_to_stamp(9_000_000_000)
                transform.transform.rotation.w=1.
                tf.sendTransform(transform)
                elapsed=time.monotonic()-start
                stamp=9_900_000_000 if elapsed < 6 else (0 if elapsed < 8 else (8_000_000_000 if elapsed < 10 else 20_000_000_000))
                msg=Image(width=8,height=6,encoding='32FC1',step=32,data=np.ones((6,8),dtype='<f4').tobytes())
                msg.header.frame_id=info.header.frame_id
                msg.header.stamp=ns_to_stamp(stamp)
                depth.publish(msg)
                until=time.monotonic()+.25
                while time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.02)
            result=reports[-1]
            assert result['tf_missing'] > 0
            assert result['invalid_stamp'] > 0
            assert result['stale_depth'] > 0
            assert result['future_depth'] > 0
            assert result['depth_integrated'] == result['esdf_published'] == 0
            result = dict(rejection=result)
            # Supply valid exposure-time TF after two backwards clock jumps.
            # This isolates mapper epoch recovery from Gazebo's controller reset bug.
            previous_integrated = 0
            for phase, base in enumerate((1_000_000_000, 100_000_000), start=1):
                start = time.monotonic()
                step = 0
                while time.monotonic()-start < 10:
                    stamp = base+step*100_000_000
                    clock.publish(Clock(clock=ns_to_stamp(stamp)))
                    camera.publish(info)
                    transform.header.stamp=ns_to_stamp(stamp)
                    tf.sendTransform(transform)
                    msg.header.stamp=ns_to_stamp(stamp)
                    depth.publish(msg)
                    until=time.monotonic()+.1
                    while time.monotonic()<until:
                        rclpy.spin_once(node,timeout_sec=.01)
                    step+=1
                latest=reports[-1]
                assert latest['map_epoch'] == phase, latest
                assert latest['depth_integrated'] > previous_integrated
                assert latest['total_cells'] == 52111 and latest['known_cells'] > 0
                assert latest['input_stamp_ns'] >= base and latest['input_stamp_ns'] <= stamp
                previous_integrated=latest['depth_integrated']
                result['recovered_epoch_'+str(phase)] = latest
            result['passed']=True
            Path(args.output).write_text(json.dumps(result,indent=2))
            print(json.dumps(result,indent=2))
        finally:
            child.terminate()
            try: child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill();child.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
