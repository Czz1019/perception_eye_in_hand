"""Common calibrated depth input for fair Python/native ROS comparison."""
import copy
import argparse
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from ur7e_nvblox.depth import depth_meters, intrinsics_from_info


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stride',type=int,default=2)
    args,ros_args=parser.parse_known_args()
    if args.stride<1:raise ValueError('stride must be positive')
    rclpy.init(args=ros_args)
    node=rclpy.create_node('ur7e_common_depth')
    raw=node.declare_parameter('depth_topic','/camera/depth/image_rect_raw').value
    info_topic=node.declare_parameter('camera_info_topic','/camera/depth/camera_info').value
    minimum=float(node.declare_parameter('min_depth_m',.2).value)
    maximum=float(node.declare_parameter('max_depth_m',4.).value)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    images=node.create_publisher(Image,'/esdf_input/depth',qos)
    infos=node.create_publisher(CameraInfo,'/esdf_input/camera_info',qos)
    state={'info':None}
    node.create_subscription(CameraInfo,info_topic,lambda m:state.update(info=m),qos)
    def receive(msg):
        info=state['info']
        if info is None:return
        if (msg.width,msg.height,msg.header.frame_id)!=(info.width,info.height,info.header.frame_id):
            node.get_logger().warning('Raw depth/calibration mismatch',throttle_duration_sec=5.)
            return
        try:
            d=depth_meters(msg,args.stride,minimum,maximum)
            intr=intrinsics_from_info(info,args.stride)
        except ValueError as exc:
            node.get_logger().warning(str(exc),throttle_duration_sec=5.);return
        cal=copy.deepcopy(info);cal.header=msg.header;cal.width=intr.width;cal.height=intr.height
        cal.k=[intr.fx,0.,intr.cx,0.,intr.fy,intr.cy,0.,0.,1.]
        cal.p=[intr.fx,0.,intr.cx,0.,0.,intr.fy,intr.cy,0.,0.,0.,1.,0.]
        out=Image();out.header=msg.header;out.width=intr.width;out.height=intr.height
        out.encoding='32FC1';out.is_bigendian=False;out.step=intr.width*4;out.data=d.astype('<f4',copy=False).tobytes()
        infos.publish(cal);images.publish(out)
    node.create_subscription(Image,raw,receive,qos)
    node.get_logger().info(f'Common depth stride={args.stride}, metres [{minimum},{maximum}], image-time stamp/optical frame unchanged')
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
