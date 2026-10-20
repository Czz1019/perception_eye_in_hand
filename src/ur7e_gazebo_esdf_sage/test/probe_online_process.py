"""Read-only live component check of isolated mapping, no robot commands."""
import argparse,time,json
from pathlib import Path
from types import SimpleNamespace
import yaml,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo,Image
from ur7e_gazebo_esdf_sage.config import load_mapping,identity
from ur7e_gazebo_esdf_sage.partial_map import load_snapshot
from ur7e_real_static.field import FrozenMap
from ur7e_gazebo_esdf_sage.online_v2 import OnlineLayer

def main():
 p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');a=p.parse_args();src=Path(a.source);out=Path(a.output);out.mkdir(parents=True,exist_ok=True);c=load_mapping(src/'mapping.yaml');path=next((src/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path)
 rclpy.init();node=Node('probe_online_process',parameter_overrides=[Parameter('use_sim_time',value=True)]);received={}
 node.create_subscription(CameraInfo,c['camera_info_topic'],lambda m:received.update(info=m),qos_profile_sensor_data);node.create_subscription(Image,c['depth_topic'],lambda m:received.update(image=m),qos_profile_sensor_data)
 deadline=time.monotonic()+10
 while len(received)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 info=received['info'];signature=(info.width,info.height,info.header.frame_id,tuple(info.k),tuple(info.p),received['image'].encoding)
 mapper=SimpleNamespace(c=c,session=path.parent,camera_signature=signature,extra_mask=yaml.safe_load((src/'scan.yaml').read_text())['depth_self_mask'])
 layer=OnlineLayer(mapper,base,out)
 try:
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
  print(layer.status());assert layer.frames>40;layer.read(node.get_clock().now().nanoseconds*1e-9)
 finally:layer.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
