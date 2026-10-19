"""Post-run stage-one audit; fixture truth is used only here, never by control."""
import argparse,json
from pathlib import Path
from dataclasses import replace
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_gazebo_esdf_sage_continuous.partial_map import load_snapshot
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.trajectory import edge_clear
from design_scan import Fixture
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output);c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
scenario=json.loads((out/'online_scenario.json').read_text());assert scenario[-1]['kind']=='complete' and scenario[-1]['passed']
by={e['kind']:e for e in scenario};start=by['spawn_requested']['sim_time'];end=by['delete_completed']['sim_time'];center=np.array(by['spawn_requested']['center']);half=np.array(by['spawn_requested']['size'])/2
rows=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8');q=np.stack([rows[n] for n in rows.dtype.names[4:10]],-1)
minimum_static=np.inf;minimum_self=np.inf;minimum_added=np.inf;active_count=0
for i in range(0,len(q),256):
 cs=g.spheres_batch(q[i:i+256]);minimum_static=min(minimum_static,float((Fixture().query(cs)-g.radii).min()));minimum_self=min(minimum_self,float(g.self_clearance_batch(cs).min()))
 active=(rows['sim_time'][i:i+256]>=start)&(rows['sim_time'][i:i+256]<=end)
 if active.any():
  delta=abs(cs[active]-center)-half;distance=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
  minimum_added=min(minimum_added,float((distance-g.radii).min()));active_count+=int(active.sum())
path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));base=load_snapshot(f,path)
feedback=json.loads((out/'sage_feedback.json').read_text());cache={0:base};checked=0
for record in feedback:
 version=record['execute_map_version'];assert version is not None
 if version not in cache:
  with np.load(out/'online'/f'version_{version:06d}.npz') as d:occupied=d['occupied']
  dynamic=distance_transform_edt(~occupied)*base.resolution-np.sqrt(3)*base.resolution/2 if occupied.any() else np.full(occupied.shape,np.inf)
  cache[version]=replace(base,distance=np.minimum(base.distance,dynamic).astype(np.float32))
 assert edge_clear(g,cache[version],record['q'],record['next'],c['collision_margin'],c['self_margin']),record
 checked+=1
updates=[json.loads(line) for line in (out/'online/updates.jsonl').read_text().splitlines()]
assert checked>0 and updates and active_count>0
assert minimum_static>.03 and minimum_added>.03 and minimum_self>.015
assert by['resumed_reached']['status']['map_uid']==f.uid
latencies=np.array([u['latency'] for u in updates]);stampages=np.array([u['commit_sim_time']-u['image_stamp'] for u in updates]);assert max(latencies)<=.5 and max(stampages)<=.5
r=dict(static_map_uid=f.uid,static_frames=f.metadata['sequence'],actual_samples=len(q),object_present_samples=active_count,
 min_static_fixture_gap_m=minimum_static,min_added_obstacle_gap_m=minimum_added,min_self_gap_m=minimum_self,
 executed_steps_checked_against_recorded_online_version=checked,online_frames=len(updates),changed_versions=max(u['version'] for u in updates),
 online_latency_p50_s=float(np.quantile(latencies,.5)),online_latency_p95_s=float(np.quantile(latencies,.95)),online_latency_max_s=float(max(latencies)),online_commit_image_age_max_s=float(max(stampages)),
 detection_after_spawn_complete_s=by['detected']['sim_time']-by['spawn_completed']['sim_time'],clearing_after_delete_complete_s=by['ray_cleared']['sim_time']-by['delete_completed']['sim_time'],
 blocked_command_range_rad=by['blocked_hold_passed']['command_range_rad'],goal_error_m=by['resumed_reached']['goal_event']['error_m'])
(out/'online_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
