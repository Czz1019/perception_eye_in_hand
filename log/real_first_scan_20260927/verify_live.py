import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import JointState,Image
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS
setup=Path('/home/wqj/ur7e_real_setup');g=Geometry(str(setup/'calibrated_with_camera.urdf'),str(setup/'collision_spheres_preview.yaml'))
rclpy.init();n=rclpy.create_node('verify_real_sensor_frames');b=Buffer();lis=TransformListener(b,n);states=[];depth=[]
n.create_subscription(JointState,'/joint_states',states.append,qos_profile_sensor_data)
def image(m):
    d=np.frombuffer(m.data,dtype='<u2').reshape(m.height,m.step//2)[:,:m.width];valid=d[(d>=200)&(d<=3000)]
    depth.append(dict(frame=m.header.frame_id,encoding=m.encoding,width=m.width,height=m.height,age_ms=(n.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e6,valid_fraction=float(len(valid)/d.size),median_m=float(np.median(valid)*.001) if len(valid) else None))
n.create_subscription(Image,'/camera/camera/depth/image_rect_raw',image,qos_profile_sensor_data)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert len(states)>100 and depth
# Check an exact buffered feedback timestamp, allowing normal TF publication lag.
msg=states[-100];t=Time.from_msg(msg.header.stamp);q=np.array([msg.position[msg.name.index(j)] for j in JOINTS]);poses=g.fk(q);errors={}
for link in [*g.links,g.camera]:
    live=b.lookup_transform('base_link',link,t).transform
    actual=np.eye(4);actual[:3,3]=[live.translation.x,live.translation.y,live.translation.z];actual[:3,:3]=Rotation.from_quat([live.rotation.x,live.rotation.y,live.rotation.z,live.rotation.w]).as_matrix()
    expected=poses[link]
    errors[link]=dict(translation_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),rotation_rad=float(Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude()))
result=dict(joint_samples=len(states),depth_samples=len(depth),depth=depth[-1],tf_model_errors=errors,depth_pose_in_base=poses[g.camera].tolist())
print(json.dumps(result,indent=2));Path('log/real_first_scan_20260927/sensor_checks.json').write_text(json.dumps(result,indent=2))
assert max(v['translation_m'] for v in errors.values())<.003 and max(v['rotation_rad'] for v in errors.values())<.01
n.destroy_node();rclpy.shutdown()
