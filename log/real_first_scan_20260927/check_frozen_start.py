from pathlib import Path
import json,time,numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.config import load_config,identity
from ur7e_real_static.model import Geometry
from ur7e_real_static.trajectory import edge_clear
from ur7e_perception_planning_core.geometry import JOINTS
c=load_config('/home/wqj/ur7e_real_setup/preview.yaml');m=FrozenMap.load(Path(c['output_directory'])/'frozen_map.npz',identity(c));g=Geometry(c['robot_urdf'],c['collision_spheres'])
rclpy.init();node=rclpy.create_node('read_only_frozen_start_check');states=[];node.create_subscription(JointState,c['joint_topic'],states.append,qos_profile_sensor_data);deadline=time.monotonic()+5
while not states and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
assert states
msg=states[-1];q=np.array([msg.position[msg.name.index(j)] for j in JOINTS]);centers=g.spheres(q);dist=m.snapshot.query(centers);clear=dist-g.radii
rows=[]
for i,link in enumerate(g.links):
    ids=g.link_ids==i
    rows.append(dict(link=link,spheres=int(ids.sum()),nonpositive_or_unknown_center_count=int(np.count_nonzero(dist[ids]<=0)),margin_failed_count=int(np.count_nonzero(clear[ids]<c['collision_margin'])),min_clearance_m=float(clear[ids].min())))
result=dict(start_passes=bool(edge_clear(g,m.snapshot,q,q,c['collision_margin'],c['self_margin'])),self_clearance_m=float(g.self_clearance(centers)),required_environment_margin_m=c['collision_margin'],tool0_xyz=g.fk(q)[g.tool][:3,3].tolist(),q=q.tolist(),per_link=rows)
Path('log/real_first_scan_20260927/frozen_start_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
node.destroy_node();rclpy.shutdown()
