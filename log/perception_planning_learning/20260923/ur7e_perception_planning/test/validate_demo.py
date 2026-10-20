"""Observe an A-to-B run; scene truth is an independent test oracle only."""
import argparse,json,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur7e_perception_planning.msg import PlanningMap
from ur7e_perception_planning_core.grid import map_arrays
from ur7e_perception_planning_core.geometry import RobotGeometry,JOINTS
from ament_index_python.packages import get_package_share_directory

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=600);p.add_argument('--goal',type=float,nargs=3,default=[.5,0,.45]);p.add_argument('--save-maps',action='store_true');args=p.parse_args()
rclpy.init();node=rclpy.create_node('validate_ab_demo',parameter_overrides=[Parameter('use_sim_time',value=True)])
r=RobotGeometry();oracle=yaml.safe_load((Path(get_package_share_directory('ur7e_sage_planner'))/'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
statuses=[];maps=[];states=[];clearances=[];self_clearances=[];latest={};last_state=0.;last_map=0.;mapper_stats=[]
out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
def state_cb(msg):
 global last_state
 now=time.monotonic()
 if now-last_state < .1:return
 last_state=now
 q=np.array([msg.position[msg.name.index(j)] for j in JOINTS]);centers=r.spheres(q);d=[]
 for box in oracle['cube'].values():
  x=np.abs(centers-box['pose'][:3])-np.array(box['dims'])/2
  d.extend(np.linalg.norm(np.maximum(x,0),axis=1)+np.minimum(x.max(axis=1),0)-r.radii)
 for ball in oracle['sphere'].values():d.extend(np.linalg.norm(centers-ball['position'],axis=1)-r.radii-ball['radius'])
 clearances.append(float(min(d)));self_clearances.append(r.self_clearance(centers));states.append(q.tolist())
def status_cb(msg):
 latest.clear();latest.update(json.loads(msg.data));statuses.append(dict(wall=time.monotonic(),**latest))
 if not args.save_maps and latest.get('map_sequence') is not None:
  sample=[time.monotonic(),latest['map_sequence'],latest['map_epoch']]
  if not maps or sample[1:]!=maps[-1][1:]:maps.append(sample)
def map_cb(msg):
 global last_map
 now=time.monotonic();maps.append([now,msg.sequence,msg.epoch])
 if now-last_map>8:
  raw,d,source=map_arrays(msg);np.savez_compressed(out.with_suffix('.map.npz'),raw=raw,distance=d,source=source,origin=[msg.origin.x,msg.origin.y,msg.origin.z],dims=msg.dimensions,res=msg.resolution)
  last_map=now
node.create_subscription(JointState,'/joint_states',state_cb,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_subscription(String,'/perception_planning/status',status_cb,1)
def stats_cb(msg):
 data=json.loads(msg.data)
 # Read-only host frequency evidence; do not change any power/thermal settings.
 frequencies=[float(line.split(':')[1]) for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('cpu MHz')]
 mapper_stats.append(dict(wall=time.monotonic(), cpu_mhz_median=float(np.median(frequencies)) if frequencies else None, **data))
 out.with_suffix('.stats.json').write_text(json.dumps(mapper_stats[-1], indent=2))
node.create_subscription(String,'/ur7e_nvblox/stats',stats_cb,1)
if args.save_maps:node.create_subscription(PlanningMap,'/perception_planning/map',map_cb,1)
t=time.monotonic();streak=0;count=0;report=0.
while time.monotonic()-t<args.duration:
 rclpy.spin_once(node,timeout_sec=.1)
 if latest.get('error'):break
 if len(statuses)!=count:
  count=len(statuses)
  good=latest.get('status')=='GOAL_REACHED' and np.allclose(latest.get('goal',[99]*3),args.goal) and latest.get('goal_error',1)<.02
  streak=streak+1 if good else 0
  if streak>=3:break
 if time.monotonic()-report>10:
  print(json.dumps(dict(elapsed=round(time.monotonic()-t,1),status=latest.get('status'),phase=latest.get('demo_phase'),error=latest.get('goal_error'))),flush=True);report=time.monotonic()
checks=dict(goal_reached=streak>=3,no_runtime_error=bool(statuses) and not any(s.get('error') for s in statuses),online_map_updates=len(maps)>5,
            collision_model_clear=bool(clearances) and min(clearances)>.015,
            self_collision_clear=bool(self_clearances) and min(self_clearances)>.008,
            sage_reaching_used=any(s.get('command_source')=='sage' for s in statuses))
result=dict(passed=all(checks.values()),checks=checks,duration=time.monotonic()-t,latest=latest,min_geometry_clearance=min(clearances) if clearances else None,
            min_self_clearance=min(self_clearances) if self_clearances else None,
            map_rate=(maps[-1][1]-maps[0][1])/(maps[-1][0]-maps[0][0]) if len(maps)>1 else 0,statuses=statuses,map_samples=maps,joint_states=states,mapper_stats=mapper_stats)
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('statuses','map_samples','joint_states','mapper_stats')},indent=2),flush=True)
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
