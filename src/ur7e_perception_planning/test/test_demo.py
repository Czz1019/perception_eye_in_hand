"""Demo-specific commissioning and SAGE velocity action contracts."""
from pathlib import Path
import time
import threading
from types import SimpleNamespace
import numpy as np
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning_core.demo import scan_configuration, scan_samples, goal_ik, velocity_proposal, guide_is_clear
from ur7e_perception_planning_core.geometry import RobotGeometry
from ur7e_perception_planning_core.grid import Snapshot, grid_points, observed_free, integration_depth
from ur7e_perception_planning_core.sage import build_task
from ur7e_perception_planning_core.planner import Planner


def test_scan_corridor_commissioned_against_fixture():
    # Truth is confined to the test: runtime receives only depth/TF and the
    # bounded, explicitly commissioned observation corridor.
    geometry = RobotGeometry()
    views, cfg = scan_configuration()
    share = Path(get_package_share_directory('ur7e_sage_planner'))
    obstacles = yaml.safe_load((share/'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
    np.testing.assert_array_equal(views[0], views[-1])
    for q in scan_samples(views):
        centers = geometry.spheres(q)
        assert geometry.self_clearance(centers) > .015
        assert np.all(q >= geometry.lower) and np.all(q <= geometry.upper)
        for name, box in obstacles['cube'].items():
            if name == 'ground':
                continue  # The prior is independently clipped at z=0.
            delta = abs(centers-box['pose'][:3])-np.array(box['dims'])/2
            clearance = np.linalg.norm(np.maximum(delta, 0), axis=1)+np.minimum(delta.max(axis=1), 0)-geometry.radii
            assert clearance.min() > cfg['clearance_m']
        for ball in obstacles['sphere'].values():
            clearance = np.linalg.norm(centers-ball['position'], axis=1)-geometry.radii-ball['radius']
            assert clearance.min() > cfg['clearance_m']


def test_goal_ik_rejects_unobserved_endpoint():
    geometry = RobotGeometry()
    initial = scan_configuration()[0][0]
    class Unknown:
        def query(self, centers):
            return np.full(len(centers), -1.)
    assert goal_ik(geometry, Unknown(), initial, np.array([.5, 0, .45])) is None


def test_guide_rejects_obstruction_between_valid_endpoints():
    class Geometry:
        radii = np.array([.005])
        def spheres(self, q): return np.array([[q[0], 0., .5]])
        def self_clearance(self, centers): return 1.
    class Obstacle:
        def query(self, centers): return abs(centers[:, 0]-.2)-.02
    a, b = np.zeros(6), np.array([.4, 0., 0., 0., 0., 0.])
    assert not guide_is_clear(Geometry(), Obstacle(), a, b)


def test_goal_ik_changes_redundant_posture_to_clear_intermediate_path():
    class Geometry:
        radii = np.array([.005])
        lower, upper = np.full(6, -2.), np.full(6, 2.)
        def fk(self, q):
            pose = np.eye(4); pose[:3, 3] = [q[0], 0., .5]
            return {'tool0': pose}
        def spheres(self, q):
            return np.array([[q[0], q[1]*np.sin(np.pi*q[0]), .5]])
        def self_clearance(self, centers): return 1.
    class Obstacle:
        def query(self, centers):
            return np.linalg.norm(centers-np.array([.5, 0., .5]), axis=-1)-.1
    geometry, snapshot = Geometry(), Obstacle()
    start = np.array([0., .05, 0., 0., 0., 0.])
    naive = start.copy(); naive[0] = 1.
    assert not guide_is_clear(geometry, snapshot, start, naive)
    goal = goal_ik(geometry, snapshot, start, np.array([1., 0., .5]))
    assert goal is not None
    assert guide_is_clear(geometry, snapshot, start, goal)
    assert np.linalg.norm(geometry.fk(goal)['tool0'][:3, 3]-[1., 0., .5]) < .008


def test_far_measured_return_proves_free_ray_without_inventing_near_surface():
    intr = SimpleNamespace(fx=10., fy=10., cx=5., cy=5., width=11, height=11)
    ray = np.full((11, 11), 5., dtype='f4')
    near = integration_depth(ray, 3.)
    assert not near.any() and np.all(ray == 5.)
    points = np.array([[0., 0., 1.], [0., 0., 5.1]])
    assert observed_free(points, ray, np.eye(4), intr, .05).tolist() == [True, False]
    ray[5, 5] = 0.  # An invalid or self-filtered pixel cannot clear its ray.
    assert not observed_free(points, ray, np.eye(4), intr, .05).any()


def test_braking_proposal_does_not_overshoot_nearby_observation_pose():
    q = np.zeros(6)
    goal = np.array([.01, -.01, .5, -.5, 0., .001])
    dt = np.linspace(.1, .2, 32)
    actions = velocity_proposal(q, goal, dt, .08)
    positions = np.cumsum(actions*dt[:, None], axis=0)
    assert abs(actions).max() <= .08+1e-12
    assert np.all(positions >= np.minimum(q, goal)-1e-12)
    assert np.all(positions <= np.maximum(q, goal)+1e-12)
    assert np.linalg.norm(goal-positions[-1]) < np.linalg.norm(goal-q)
    nonzero = np.flatnonzero(goal)
    ratios = positions[:, nonzero]/goal[nonzero]
    np.testing.assert_allclose(ratios, np.repeat(ratios[:, :1], len(nonzero), axis=1), atol=1e-12)


def test_sage_velocity_actions_and_joint_guide(tmp_path):
    task = build_task(tmp_path, particles=64, velocity_control=True)
    geometry = RobotGeometry()
    q = scan_configuration()[0][0]
    rollout = task.controller.rollout_fn
    pose = geometry.fk(q)['tool0']
    task.update_params(goal_ee_pos=pose[:3, 3], goal_ee_rot=pose[:3, :3])
    origin, dims, _ = grid_points([0, 0, .6], [1.8, 1.8, 1.5], .03)
    snapshot = Snapshot(origin, .03, dims, np.ones(tuple(dims[::-1]), 'f4'), 1., 1, 1, time.monotonic())
    rollout.set_map(snapshot)
    state = torch.tensor(np.r_[q, np.zeros(12), 1.], **task.tensor_args)[None]
    actions = torch.full((64, 32, 6), .02, **task.tensor_args)
    predicted = rollout.dynamics_model.rollout_open_loop(state, actions)
    positions = predicted['state_seq'][0, :, :6].detach().cpu().numpy()
    assert np.all(positions[0] > q)
    assert np.max(positions[0]-q) <= .02*.2+1e-6
    baseline = rollout.cost_fn(predicted, actions).detach().clone()
    rollout.joint_guide = torch.tensor(q+1, **task.tensor_args)
    rollout.joint_guide_weight = 150.
    guided = rollout.cost_fn(predicted, actions)
    assert torch.isfinite(guided).all() and torch.all(guided > baseline)
    rollout.joint_segment = torch.tensor(np.stack([q, q+np.eye(6)[0]]), **task.tensor_args)
    corridor_cost = rollout.cost_fn(predicted, actions)
    assert torch.all(corridor_cost > guided)
    rollout.joint_segment = None
    result = task.controller.optimize(state, shift_steps=1)[0]
    assert torch.isfinite(result).all()
    assert result.abs().max().item() <= .080001


def test_direct_map_handoff_keeps_freshness_and_epoch_guards():
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_000_000_000)),
        cfg={'map_timeout_sim': 1.5}, lock=threading.RLock(), snapshot=None,
        last_map_epoch=None, target='old command', generation=0)
    def snapshot(stamp=5., epoch=1, sequence=1):
        return Snapshot(np.zeros(3), .02, np.array([2, 2, 2]), np.ones((2, 2, 2)),
                        stamp, epoch, sequence, time.monotonic())
    first = snapshot()
    Planner.on_snapshot(node, first)
    assert node.snapshot is first and node.target is None and node.generation == 1
    node.target = 'new command'
    Planner.on_snapshot(node, snapshot(stamp=1., epoch=2))
    Planner.on_snapshot(node, snapshot(sequence=0))
    assert node.snapshot is first and node.target == 'new command'
    Planner.on_snapshot(node, snapshot(epoch=2))
    assert node.snapshot.epoch == 2 and node.target is None and node.generation == 2
