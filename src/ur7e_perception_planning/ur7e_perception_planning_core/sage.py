"""SAGE extension using a frozen GPU ESDF snapshot; no STORM source edits."""
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
import yaml
from ament_index_python.packages import get_package_share_directory
from storm_kit.mpc.rollout.sage_arm_reacher import SageArmReacher
from storm_kit.mpc.task.sage_reacher_task_v3 import SageReacherTaskV3
from .geometry import RobotGeometry


class EsdfRollout(SageArmReacher):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        geometry = RobotGeometry()
        self.local = torch.tensor(geometry.local, **self.tensor_args)
        self.radii = torch.tensor(geometry.radii, **self.tensor_args)
        self.link_ids = torch.tensor(geometry.link_ids, device=self.local.device)
        self.pair_a = torch.tensor([a for a, b in geometry.pairs], device=self.local.device)
        self.pair_b = torch.tensor([b for a, b in geometry.pairs], device=self.local.device)
        self.map = None
        self.margin = .035

    def set_map(self, snapshot):
        self.map = torch.as_tensor(snapshot.distance.copy(), **self.tensor_args)[None, None]
        self.origin = torch.as_tensor(snapshot.origin, **self.tensor_args)
        self.extent = torch.as_tensor((snapshot.dimensions-1)*snapshot.resolution, **self.tensor_args)
        self.interpolation_margin = np.sqrt(3)*snapshot.resolution/2

    def query(self, centers):
        coordinates = 2*(centers-self.origin)/self.extent-1
        shape = centers.shape[:-1]
        distance = F.grid_sample(self.map, coordinates.reshape(1, 1, 1, -1, 3),
                                 mode='bilinear', padding_mode='zeros', align_corners=True).reshape(shape)
        return torch.where((coordinates.abs() <= 1).all(dim=-1),
                           distance-self.interpolation_margin, -torch.ones_like(distance))

    def cost_fn(self, state_dict, action_batch, no_coll=False, horizon_cost=True, return_dist=False):
        base = super().cost_fn(state_dict, action_batch, no_coll, horizon_cost, return_dist)
        if no_coll:
            return base
        if self.map is None:
            raise RuntimeError('SAGE must not optimize before a valid ESDF snapshot')
        rotation = state_dict['link_rot_seq'][..., self.link_ids, :, :]
        position = state_dict['link_pos_seq'][..., self.link_ids, :]
        centers = (rotation @ self.local[..., None]).squeeze(-1) + position
        clearance = self.query(centers) - self.radii
        penalty = torch.relu(self.margin-clearance)
        # A soft buffer for ranking and a strong term for penetrating the hard
        # guard margin. Execution has an independent hard collision check.
        collision = 6000*penalty.square().sum(-1) + 2000*torch.relu(.025-clearance).sum(-1)
        self_distance = torch.linalg.vector_norm(centers[..., self.pair_a, :] - centers[..., self.pair_b, :], dim=-1)
        self_distance -= self.radii[self.pair_a] + self.radii[self.pair_b]
        collision += 10000*torch.relu(.015-self_distance).square().sum(-1)
        if return_dist:
            return (base[0]+collision, *base[1:])
        return base+collision


class EsdfTask(SageReacherTaskV3):
    def get_rollout_fn(self, **kwargs):
        return EsdfRollout(**kwargs)

    def init_aux(self):
        # The application calls controller.optimize synchronously and integrates
        # only its first acceleration using fresh measured q,dq. No unused child
        # process, state prediction or unguarded Cartesian refinement is needed.
        self.n_dofs = self.controller.rollout_fn.dynamics_model.n_dofs

    def update_params(self, **kwargs):
        return self.controller.rollout_fn.update_params(**kwargs)

    def close(self):
        pass


def build_task(run_dir, dt=.1, particles=384):
    share = Path(get_package_share_directory('ur7e_sage_planner'))
    params = yaml.safe_load((share/'config/ur7e_reacher_gazebo_tall_sage_clean.yml').read_text())

    def resolve(value):
        if isinstance(value, dict):
            return {key: resolve(item) for key, item in value.items()}
        if isinstance(value, list):
            return [resolve(item) for item in value]
        if isinstance(value, str) and value.startswith('package://'):
            package, path = value[10:].split('/', 1)
            return str(Path(get_package_share_directory(package))/path)
        return value

    params = resolve(params)
    params['control_dt'] = dt
    params['model']['link_names'] = RobotGeometry().links
    params['model']['max_acc'] = .5
    params['model']['vel_scale'] = .06
    params['cost']['primitive_collision']['weight'] = 0.
    params['cost']['voxel_collision']['weight'] = 0.
    params['cost']['robot_self_collision']['weight'] = 0.
    params['sage_deployment_refinement']['enabled'] = False
    params['mppi'].update(num_particles=particles, horizon=24, n_iters=2, execute_best=True,
                          init_cov=.08)
    params['sage_controller_core']['enable_runtime_stats'] = True
    params['sage_controller_core']['sigma_0'] = .08
    Path(run_dir).mkdir(parents=True, exist_ok=True)
    config = Path(run_dir)/'resolved_task.yaml'
    config.write_text(yaml.safe_dump(params))
    # No wall/sphere truth is loaded by the planning task.
    world = Path(run_dir)/'empty_world.yaml'
    world.write_text('world_model:\n  coll_objs: {}\n')
    return EsdfTask(str(config), str(share/'config/ur7e_robot_gazebo.yml'), str(world),
                    tensor_args={'device': torch.device('cuda:0'), 'dtype': torch.float32})
