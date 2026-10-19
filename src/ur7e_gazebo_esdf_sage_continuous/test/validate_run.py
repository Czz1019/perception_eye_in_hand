"""Independent audit of measured motion; never supplies geometry to control."""
import argparse,json
from pathlib import Path
import numpy as np
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from design_scan import Fixture

p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres']);truth=Fixture()
frozen=FrozenMap.load(next((out/'maps').rglob('frozen_map.npz')),identity(c))
from ur7e_gazebo_esdf_sage_continuous.ownership import load_snapshot
ownership=next((out/'maps').rglob('robot_ownership.npz'),None)
snapshot=load_snapshot(frozen,ownership) if ownership is not None else frozen.snapshot
partial=next((out/'maps').rglob('partial_policy.npz'),None)
if partial is not None:
    from ur7e_gazebo_esdf_sage_continuous.partial_map import load_snapshot as load_partial
    snapshot=load_partial(frozen,partial)
rows=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
names=rows.dtype.names;q=np.stack([rows[n] for n in names[4:10]],axis=-1);commands=np.stack([rows[n] for n in names[10:16]],axis=-1)
self_gaps=[];fixture_gaps=[];map_gaps=[]
for i in range(0,len(q),512):
    centers=g.spheres_batch(q[i:i+512]);self_gaps.extend(g.self_clearance_batch(centers));fixture_gaps.extend((truth.query(centers)-g.radii).min(-1));map_gaps.extend((snapshot.query(centers)-g.radii).min(-1))
self_gaps=np.array(self_gaps);fixture_gaps=np.array(fixture_gaps);map_gaps=np.array(map_gaps)
planning=np.isin(rows['phase'],['REACHING','GOAL_REACHED','SOLVING_GUIDE'])
time=rows['sim_time'];dt=np.diff(time);speed=abs(np.diff(commands,axis=0))/np.maximum(dt[:,None],1e-9)
events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
goals=[e for e in events if e['kind']=='goal_reached']
feedback=json.loads((out/'sage_feedback.json').read_text()) if (out/'sage_feedback.json').exists() else []
blocked=[]
for event in [e for e in events if e['kind']=='guide']:
    guide=np.array(event['joints']);centers=g.spheres_batch(np.linspace(guide[0],guide[-1],1000))
    blocked.append(dict(goal=event['goal'],waypoints=len(guide),direct_joint_segment_fixture_gap_m=float((truth.query(centers)-g.radii).min()),direct_joint_segment_map_gap_m=float((snapshot.query(centers)-g.radii).min())))
result=dict(distance_semantics='partial-policy feasibility, not observed-free ESDF' if partial is not None else 'strict observed-free ESDF',samples=len(q),capture_frames=frozen.metadata['sequence'],observations=len(frozen.metadata['observations']),map_uid=frozen.uid,
    min_measured_self_gap_m=float(min(self_gaps)),min_measured_fixture_gap_m=float(min(fixture_gaps)),
    min_planning_map_gap_m=float(min(map_gaps[planning])) if np.any(planning) else None,
    max_tracking_error_rad=float(abs(q-commands).max()),max_command_speed_rad_s=float(speed.max()),
    max_reaching_command_speed_rad_s=float(speed[planning[1:] & planning[:-1]].max()) if planning[1:].any() else None,
    goals_reached=goals,guide_checks=blocked,sage_feedback_steps=len(feedback),
    unique_feedback_timestamps=len(set(s['stamp'] for s in feedback)),observed_cells=int(np.count_nonzero(frozen.source==1)),tabletop_cells=int(np.count_nonzero(frozen.source==2)),planning_policy=frozen.metadata.get("planning_policy"))
(out/'motion_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert goals and feedback,'No completed SAGE reaching'
assert min(self_gaps)>.015 and min(fixture_gaps)>.03
assert result['min_planning_map_gap_m']>.029
assert result['max_command_speed_rad_s']<.27
assert result['max_reaching_command_speed_rad_s']<.16
assert result['max_tracking_error_rad']<.06
assert result['unique_feedback_timestamps']==len(feedback)
assert all(e['error_m']<.02 for e in goals)
