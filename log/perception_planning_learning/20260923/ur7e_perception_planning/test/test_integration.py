"""Contracts that can otherwise silently invert or bypass collision checking.

Run with ROS/workspace sourced and whole_control Python -m pytest this directory.
"""
import time
from array import array
from pathlib import Path
import numpy as np
import pytest
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning_core.geometry import RobotGeometry
from ur7e_perception_planning_core.grid import Snapshot, grid_points, conservative_field, observed_free, pack_map, map_arrays
from ur7e_perception_planning_core.planner import check_segment
from ur7e_perception_planning_core.sage import build_task
from ur7e_perception_planning.msg import PlanningMap


@pytest.fixture(scope='module')
def geometry():
    return RobotGeometry()


@pytest.fixture(scope='module')
def initial():
    share = Path(get_package_share_directory('ur7e_reconstruction'))
    return np.array(list(yaml.safe_load((share/'config/high_initial_positions.yaml').read_text()).values()))


@pytest.fixture(scope='module')
def task(tmp_path_factory):
    return build_task(tmp_path_factory.mktemp('sage'), particles=64)


def test_cpu_gpu_fk_and_calibrated_payload(task, geometry, initial):
    for q in (initial, initial+np.array([.1, -.1, .15, -.2, .1, .2])):
        poses = geometry.fk(q)
        qt = torch.tensor(q, **task.tensor_args)[None]
        for link in geometry.links:
            pos, rot = task.controller.rollout_fn.dynamics_model.robot_model.compute_forward_kinematics(qt, qt*0, link_name=link)
            np.testing.assert_allclose(pos.detach().cpu()[0], poses[link][:3, 3], atol=2e-6)
            np.testing.assert_allclose(rot.detach().cpu()[0], poses[link][:3, :3], atol=2e-6)
    assert len(geometry.radii) == 20
    assert geometry.self_clearance(geometry.spheres(initial)) > .015


def test_grid_order_sign_outside_and_gpu_agreement(task):
    origin, dims, pts = grid_points([0, 0, .5], [1.2, 1.2, 1.2], .03)
    field = (pts[:, 0]+2*pts[:, 1]+3*pts[:, 2]).astype('f4')
    snap = Snapshot(origin, .03, dims, field.reshape(tuple(dims[::-1])), 1., 0, 1, time.monotonic())
    points = np.array([[.11, .07, .42], [-.31, .18, .56], [5, 0, .5]])
    task.controller.rollout_fn.set_map(snap)
    gpu = task.controller.rollout_fn.query(torch.tensor(points, **task.tensor_args)).cpu().numpy()
    np.testing.assert_allclose(gpu, snap.query(points), atol=2e-6)
    assert gpu[-1] == -1
    assert gpu[0] > 0


def test_sage_rollout_really_consumes_esdf(task, geometry, initial):
    origin, dims, pts = grid_points([0, 0, .6], [1.8, 1.8, 1.5], .03)
    rollout = task.controller.rollout_fn
    pose = geometry.fk(initial)['tool0']
    task.update_params(goal_ee_pos=pose[:3, 3], goal_ee_rot=pose[:3, :3])
    state = torch.tensor(np.r_[initial, np.zeros(12), 1.], **task.tensor_args)[None]
    actions = torch.zeros((64, 24, 6), **task.tensor_args)
    costs = []
    for distance in (1., -.1):
        snap = Snapshot(origin, .03, dims, np.full(tuple(dims[::-1]), distance, 'f4'), 1., 1, 1, time.monotonic())
        rollout.set_map(snap)
        costs.append(rollout.rollout_fn(state, actions)['costs'].detach().clone())
    assert torch.isfinite(costs[0]).all() and torch.isfinite(costs[1]).all()
    assert torch.all(costs[1] > costs[0]+1000)


def test_unknown_is_not_free_and_obstacle_overrides_prior():
    origin, dims, pts = grid_points([0, 0, .6], [.3, .3, .3], .03)
    raw = np.full(len(pts), 1e6, 'f4'); known = np.zeros(len(pts), bool)
    sources = np.zeros(len(pts), 'u1')
    field, _ = conservative_field(raw, known, sources, pts, dims, .03)
    assert np.max(field) <= 0
    sources[:] = 2
    middle = len(pts)//2
    raw[middle] = -.01; known[middle] = True
    field, source = conservative_field(raw, known, sources, pts, dims, .03)
    assert field[middle] < 0 and source[middle] == 0
    assert field.max() > .03


def test_depth_rays_do_not_clear_occlusion_or_invalid_pixels():
    from types import SimpleNamespace
    intr = SimpleNamespace(fx=10., fy=10., cx=5., cy=5., width=11, height=11)
    points = np.array([[0, 0, .5], [0, 0, 1.1], [0, 0, .1]])
    depth = np.ones((11, 11), dtype='f4')
    assert observed_free(points, depth, np.eye(4), intr, .05).tolist() == [True, False, False]
    depth[5, 5] = 0
    assert not observed_free(points, depth, np.eye(4), intr, .05).any()


def test_atomic_map_rejects_invalid_dimensions_frame_and_nan():
    msg = PlanningMap(); msg.header.frame_id = 'base_link'; msg.resolution = .03
    msg.dimensions = [2, 2, 2]; msg.planning_distance = [1.]*8
    Snapshot.from_message(msg)
    msg.header.frame_id = 'world'
    with pytest.raises(ValueError): Snapshot.from_message(msg)
    msg.header.frame_id = 'base_link'; msg.planning_distance = array('f', [float('nan')]*8)
    with pytest.raises(ValueError): Snapshot.from_message(msg)


def test_lossless_map_transport_and_corrupt_payload():
    msg = PlanningMap(); msg.dimensions = [3, 4, 5]
    raw = np.arange(60, dtype='f4')*.013-1.
    planning = raw*.1; sources = np.arange(60, dtype='u1')%3
    pack_map(msg, raw, planning, sources)
    for expected, decoded in zip((raw, planning, sources), map_arrays(msg)):
        np.testing.assert_array_equal(expected, decoded)
    msg.compressed_data = msg.compressed_data[:5]
    with pytest.raises(ValueError): map_arrays(msg)


def test_guard_checks_middle_of_segment():
    # The obstacle is between two clear endpoints, not at either target pose.
    class Geometry:
        radii = np.array([.005])
        def spheres(self, q): return np.array([[q[0], 0., .5]])
        def self_clearance(self, c): return 1.
    class Map:
        def query(self, c): return np.abs(c[:, 0]-.05)-.01
    start = np.zeros(6); end = start.copy(); end[0] = .1
    assert Map().query(Geometry().spheres(start))[0] > .03
    assert Map().query(Geometry().spheres(end))[0] > .03
    assert not check_segment(Geometry(), Map(), start, end, margin=.002)[0]


def test_commissioned_initial_prior_against_static_scene(geometry, initial):
    # Ground-truth primitives are a TEST ORACLE ONLY, never planner inputs.
    share = Path(get_package_share_directory('ur7e_sage_planner'))
    world = yaml.safe_load((share/'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
    centers = geometry.spheres(initial)
    for name, box in world['cube'].items():
        if name == 'ground': continue  # prior is clipped against z=0 separately
        delta = np.abs(centers-np.array(box['pose'][:3]))-np.array(box['dims'])/2
        distance = np.linalg.norm(np.maximum(delta, 0), axis=1)+np.minimum(delta.max(axis=1), 0)-geometry.radii
        assert distance.min() > .12
    ball = world['sphere']['sphere1']
    assert np.min(np.linalg.norm(centers-ball['position'], axis=1)-geometry.radii-ball['radius']) > .12
