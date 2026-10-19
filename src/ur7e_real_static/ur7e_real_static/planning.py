"""Offline SAGE refinement on one immutable map; no commands are sent here."""
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
from ur7e_perception_planning_core.runtime import GPU_LOCK
from .trajectory import edge_clear, simplify, parameterize, validate


def route(g,snapshot,start,xyz,anchors,c,cancel=lambda:False):
    deadline=time.monotonic()+min(15.,c['max_plan_seconds'])
    def clear(a,b): return edge_clear(g,snapshot,a,b,c['collision_margin']+.005,c['self_margin'])
    def residual(q):
        centers=g.spheres(q)
        return np.r_[25*(g.fk(q)[g.tool][:3,3]-xyz),.01*(q-start),
                     5*np.maximum(0,c['collision_margin']+.015-(snapshot.query(centers)-g.radii)),
                     5*max(0,c['self_margin']+.005-g.self_clearance(centers))]
    candidates=[]
    seeds=[start,*[np.asarray(a) for a in anchors]]
    for sign in [-1,1]: seeds.append(start+np.array([0,0,sign*.7,-sign*.4,0,0]))
    for seed in seeds[:14]:
        if cancel(): raise InterruptedError('Planning canceled')
        if time.monotonic()>deadline: break
        q=least_squares(residual,np.clip(seed,g.lower+.001,g.upper-.001),bounds=(g.lower,g.upper),max_nfev=80,diff_step=1e-3).x
        if np.linalg.norm(g.fk(q)[g.tool][:3,3]-xyz)<.006 and clear(q,q):
            candidates.append(q)
            if clear(start,q): return [start,q]
    if not candidates: raise ValueError('No observed collision-free goal IK; collect more views or change target')
    vertices=[start,*[np.asarray(a) for a in anchors if clear(a,a)],*candidates]
    goal_indices=set(range(len(vertices)-len(candidates),len(vertices)))
    queue=[0];parent={0:None}
    while queue and time.monotonic()<deadline:
        if cancel(): raise InterruptedError('Planning canceled')
        i=queue.pop(0)
        for j in sorted(set(range(len(vertices)))-set(parent),key=lambda k:np.linalg.norm(vertices[k]-vertices[i])):
            if clear(vertices[i],vertices[j]):
                parent[j]=i
                if j in goal_indices:
                    path=[]
                    while j is not None: path.append(vertices[j]);j=parent[j]
                    return path[::-1]
                queue.append(j)
    # Sampling never declares unknown cells free. All tree edges are checked.
    rng=np.random.default_rng(41);points=[start];parents=[-1]
    for _ in range(1800):
        if cancel(): raise InterruptedError('Planning canceled')
        if time.monotonic()>deadline: break
        goal=candidates[int(rng.integers(len(candidates)))]
        low=np.maximum(g.lower,np.minimum(start,goal)-.8);high=np.minimum(g.upper,np.maximum(start,goal)+.8)
        sample=goal if rng.random()<.25 else rng.uniform(low,high)
        i=int(np.argmin(np.linalg.norm(np.asarray(points)-sample,axis=1)))
        delta=sample-points[i];q=points[i]+delta*min(1.,.2/max(np.linalg.norm(delta),1e-9))
        if not clear(points[i],q): continue
        points.append(q);parents.append(i)
        if clear(q,goal):
            path=[goal];j=len(points)-1
            while j>=0:path.append(points[j]);j=parents[j]
            return path[::-1]
    raise ValueError('No checked route in observed space within search budget')


def build_task(g,c,directory):
    import torch
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from storm_kit.mpc.rollout.sage_arm_reacher import SageArmReacher
    from ur7e_perception_planning_core.sage import EsdfRollout, EsdfTask
    class RealRollout(EsdfRollout):
        def __init__(self,**kwargs):
            SageArmReacher.__init__(self,**kwargs)
            self.local=torch.tensor(g.local,**self.tensor_args);self.radii=torch.tensor(g.radii,**self.tensor_args)
            self.link_ids=torch.tensor(g.link_ids,device=self.local.device)
            self.pair_a=torch.tensor(g.pair_a,device=self.local.device);self.pair_b=torch.tensor(g.pair_b,device=self.local.device)
            self.map=None;self.margin=c['collision_margin']+.02
            self.joint_guide=None;self.joint_guide_weight=0.;self.joint_segment=None
        def set_map(self,snapshot):
            from dataclasses import replace
            super().set_map(replace(snapshot,origin=snapshot.origin.copy()))
    class RealTask(EsdfTask):
        def get_rollout_fn(self,**kwargs): return RealRollout(**kwargs)
    def resolve(value):
        if isinstance(value,dict): return {k:resolve(v) for k,v in value.items()}
        if isinstance(value,list): return [resolve(v) for v in value]
        if isinstance(value,str) and value.startswith('package://'):
            pkg,p=value[10:].split('/',1);return str(Path(get_package_share_directory(pkg))/p)
        return value
    d=Path(directory);d.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('ur7e_sage_planner'))
    cfg=resolve(yaml.safe_load((share/'config/ur7e_reacher_gazebo_tall_sage_clean.yml').read_text()))
    urdf=g.write_urdf(d/'calibrated_planning.urdf')
    cfg['control_space']='vel';cfg['control_dt']=.1
    cfg['model'].update(urdf_path=urdf,link_names=g.links,ee_link_name=g.tool,collision_spheres=c['collision_spheres'],
                        dt=.1,vel_scale=.3/np.pi,max_acc=.5,dt_traj_params={'base_dt':.1,'base_ratio':.5,'max_dt':.2})
    cfg['model']['robot_collision_params'].update(urdf=urdf,collision_spheres=c['collision_spheres'],link_objs=g.links)
    for name in ['primitive_collision','voxel_collision','robot_self_collision']:cfg['cost'][name]['weight']=0.
    cfg['sage_deployment_refinement']['enabled']=False
    cfg['mppi'].update(num_particles=c['particles'],horizon=32,n_iters=1,execute_best=True,init_cov=.025)
    cfg['sage_controller_core']['sigma_0']=.025
    cfg['cost']['stop_cost']['weight']=1.;cfg['cost']['state_bound']['bound_thresh']=.01
    (d/'task.yaml').write_text(yaml.safe_dump(cfg));(d/'world.yaml').write_text('world_model:\n  coll_objs: {}\n')
    (d/'robot.yaml').write_text(yaml.safe_dump({'sim_params':{'sim_urdf':urdf,'robot_pose':[0,0,0,0,0,0,1]}}))
    torch.set_num_threads(1)
    task=RealTask(str(d/'task.yaml'),str(d/'robot.yaml'),str(d/'world.yaml'),tensor_args={'device':torch.device('cuda:0'),'dtype':torch.float32})
    # Verify the actual Storm FK uses the same calibrated model and joint order.
    model=task.controller.rollout_fn.dynamics_model.robot_model
    for q in [np.zeros(6),np.array([.2,-.8,.6,-.4,.3,.1])]:
        tq=torch.tensor(q,**task.tensor_args)[None]
        xyz,rot=model.compute_forward_kinematics(tq,torch.zeros_like(tq),link_name=g.tool)
        expected=g.fk(q)[g.tool]
        if not np.allclose(xyz.detach().cpu().numpy()[0],expected[:3,3],atol=2e-5) or not np.allclose(rot.detach().cpu().numpy()[0],expected[:3,:3],atol=2e-5):
            raise ValueError('SAGE/CPU calibrated FK mismatch')
    return task


def plan(g,frozen,start,xyz,c,directory,cancel=lambda:False):
    import torch
    from ur7e_perception_planning_core.demo import velocity_proposal
    if not edge_clear(g,frozen.snapshot,start,start,c['collision_margin'],c['self_margin']):
        raise ValueError('Start body is unknown or too close to obstacles; more observation/commissioning is required')
    begun=time.monotonic()
    anchors=frozen.metadata.get('views',[])
    guide=route(g,frozen.snapshot,np.asarray(start),np.asarray(xyz),anchors,c,cancel)
    with GPU_LOCK:
        task=build_task(g,c,directory);rollout=task.controller.rollout_fn;rollout.set_map(frozen.snapshot)
    durations=rollout.dynamics_model.traj_dt.detach().cpu().numpy()
    q=np.array(start);points=[q.copy()];iterations=0
    try:
        for target in guide[1:]:
            segment_start=q.copy();stalled=0
            for _ in range(1200):
                if cancel(): raise InterruptedError('Planning canceled')
                if time.monotonic()-begun>c['max_plan_seconds']:raise TimeoutError('Offline SAGE planning budget exceeded')
                if np.max(abs(target-q))<.008:break
                with GPU_LOCK, torch.no_grad():
                    pose=g.fk(target)[g.tool]
                    task.update_params(goal_ee_pos=pose[:3,3],goal_ee_rot=pose[:3,:3])
                    rollout.joint_guide=torch.tensor(target,**task.tensor_args);rollout.joint_guide_weight=200.
                    rollout.joint_segment=torch.tensor(np.stack([segment_start,target]),**task.tensor_args)
                    task.controller.mean_action=torch.tensor(velocity_proposal(q,target,durations,.3),**task.tensor_args)
                    state=torch.tensor(np.r_[q,np.zeros(12),iterations*.1],**task.tensor_args)[None]
                    velocity=task.controller.optimize(state,shift_steps=0)[0][0].detach().cpu().numpy()
                candidate=q+np.clip(velocity,-.3,.3)*.1;iterations+=1
                if not edge_clear(g,frozen.snapshot,q,candidate,c['collision_margin'],c['self_margin']):
                    raise ValueError('SAGE proposed an unsafe segment; no trajectory will execute')
                stalled=stalled+1 if np.linalg.norm(target-candidate)>=np.linalg.norm(target-q)-1e-6 else 0
                if stalled>30:raise ValueError('SAGE stalled; no execution')
                points.append(candidate.copy());q=candidate
            else:raise TimeoutError('SAGE waypoint iteration budget exceeded')
        # A checked final connector enforces exact endpoint; SAGE performs reaching above.
        if not edge_clear(g,frozen.snapshot,q,guide[-1],c['collision_margin'],c['self_margin']):raise ValueError('Final connector blocked')
        points.append(guide[-1]);points=simplify(g,frozen.snapshot,points,c['collision_margin'],c['self_margin'])
        trajectory=parameterize(points,c['max_joint_speed'],c['max_joint_acceleration'],frozen.uid,frozen.metadata['identity'],xyz,iterations)
        validate(g,frozen,trajectory,c)
        return trajectory
    finally:task.close()
