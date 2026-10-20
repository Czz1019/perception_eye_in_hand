"""Save the live robot_description String verbatim, without ROS CLI YAML quoting."""
import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--topic',default='/robot_description');a=p.parse_args()
    path=Path(a.output).expanduser()
    if path.exists():raise FileExistsError(path)
    rclpy.init();node=rclpy.create_node('export_calibrated_description');messages=[]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,a.topic,lambda msg:messages.append(msg.data),qos)
    deadline=time.monotonic()+15
    try:
        while not messages and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
        if not messages:raise TimeoutError('No robot_description; check topic and driver')
        if ET.fromstring(messages[0]).tag!='robot':raise ValueError('Not a URDF robot')
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(messages[0]);print(path)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
