"""Read-only command arrival timing; ROS receipt timing is not publisher timing."""
import argparse,csv,time,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray,String
p=argparse.ArgumentParser();p.add_argument('output');out=Path(p.parse_args().output).resolve()
rclpy.init();node=Node('continuous_command_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
phase='waiting';bound=False
log=(out/'command_arrivals.csv').open('w');writer=csv.writer(log);writer.writerow(['wall_time','sim_time','phase']+['q'+str(i) for i in range(6)])
def status(msg):
 global phase,bound
 value=json.loads(msg.data);bound=value.get('run_id')==str(out);phase=value.get('phase','waiting')
def command(msg):
 if bound:writer.writerow([time.monotonic(),node.get_clock().now().nanoseconds*1e-9,phase,*msg.data])
node.create_subscription(String,'/sim_esdf_sage/status',status,10)
node.create_subscription(Float64MultiArray,'/forward_position_controller/commands',command,qos_profile_sensor_data)
node.create_timer(1.,log.flush)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
