from pathlib import Path
import numpy as np

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
    cfg['control_space']='vel';cfg['control_dt']=.05
    cfg['model'].update(urdf_path=urdf,link_names=g.links,ee_link_name=g.tool,collision_spheres=c['collision_spheres'],
                        dt=.05,vel_scale=.15/np.pi,max_acc=.5,dt_traj_params={'base_dt':.05,'base_ratio':1.,'max_dt':.05})
    cfg['model']['robot_collision_params'].update(urdf=urdf,collision_spheres=c['collision_spheres'],link_objs=g.links)
    for name in ['primitive_collision','voxel_collision','robot_self_collision']:cfg['cost'][name]['weight']=0.
    cfg['sage_deployment_refinement']['enabled']=False
    cfg['mppi'].update(num_particles=c['particles'],horizon=40,n_iters=1,execute_best=True,init_cov=.04)
    cfg['sage_controller_core']['sigma_0']=.12
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
