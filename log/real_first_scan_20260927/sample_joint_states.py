"""Read-only latest state relay for preview; preserves hardware timestamps verbatim."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
class SampleStates(Node):
    def __init__(self):
        super().__init__('preview_joint_state_sampler');self.latest=None;self.sent=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub=self.create_publisher(JointState,'/ur7e_real_static/joint_states_sampled',qos)
        self.create_subscription(JointState,'/joint_states',self.receive,qos)
        self.create_timer(.02,self.publish)
    def receive(self,msg):self.latest=msg
    def publish(self):
        msg=self.latest
        if msg is None:return
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==self.sent:return
        self.pub.publish(msg);self.sent=stamp
rclpy.init();node=SampleStates()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
