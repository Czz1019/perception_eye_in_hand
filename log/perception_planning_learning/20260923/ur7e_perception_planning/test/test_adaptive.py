import threading
import time
import numpy as np
from ur7e_perception_planning_core.geometry import RobotGeometry
from ur7e_perception_planning_core.demo import scan_configuration, guide_is_clear
from ur7e_perception_planning_core.routing import joint_route
from ur7e_perception_planning_core.speed import latency_speed
from ur7e_perception_planning_core.runtime import MappingPriorityLock
from ur7e_perception_planning_core.self_filter import robot_depth_mask
from types import SimpleNamespace
from scipy.ndimage import distance_transform_edt
from ur7e_perception_planning_core.grid import conservative_field, grid_points


def test_batched_swept_geometry_matches_scalar_fk():
    robot = RobotGeometry()
    initial = scan_configuration()[0][0]
    qs = initial+np.random.default_rng(8).uniform(-1., 1., (31, 6))
    reference = np.array([robot.spheres(q) for q in qs])
    actual = robot.spheres_batch(qs)
    np.testing.assert_allclose(actual, reference, atol=1e-12)
    np.testing.assert_allclose(robot.self_clearance_batch(actual),
                               [robot.self_clearance(c) for c in reference], atol=1e-12)


def test_guide_refines_sampling_without_reducing_physical_margin():
    robot = RobotGeometry(); start = scan_configuration()[0][0]
    end = start+np.array([.2, 0, 0, 0, 0, 0])
    class Uniform:
        value = float(robot.radii.max())+.021
        def query(self, points): return np.full(points.shape[:-1], self.value)
    world = Uniform()
    assert guide_is_clear(robot, world, start, end)
    world.value = float(robot.radii.max())+.019
    assert not guide_is_clear(robot, world, start, end)


def test_cached_cropped_edt_matches_full_grid_and_reacts_to_obstacles():
    _, dims, points = grid_points([0, 0, .5], [.4, .4, .4], .02)
    source = np.zeros(tuple(dims[::-1]), dtype='u1')
    source[3:16, 2:18, 4:17] = 1
    raw = np.full(source.size, 10.)
    known = np.zeros(source.size, bool)
    cache = {}
    def reference():
        free = source.reshape(-1)>0
        free[known & (raw <= .02)] = False
        d = distance_transform_edt(np.pad(free.reshape(source.shape), 1))[1:-1, 1:-1, 1:-1].reshape(-1)*.02
        d -= np.sqrt(3)*.02/2
        d = np.minimum(d, np.where(known, raw, np.inf))
        d[~free] = np.minimum(d[~free], 0.)
        return d
    for edit in (False, False, True):
        if edit:
            i = np.ravel_multi_index((10, 10, 10), source.shape)
            known[i] = True; raw[i] = -.01
        d, _ = conservative_field(raw, known, source.reshape(-1), points, dims, .02, cache)
        np.testing.assert_allclose(d, reference(), atol=1e-7)
    assert cache['recomputed']
    conservative_field(raw, known, source.reshape(-1), points, dims, .02, cache)
    assert not cache['recomputed']


def test_route_goes_around_wall_when_direct_edge_is_blocked():
    class Geometry:
        radii = np.array([.01])
        lower, upper = np.full(6, -2.), np.full(6, 2.)
        def spheres(self, q): return np.array([[q[0], q[1], .5]])
        def self_clearance(self, centers): return 1.
    class Wall:
        def query(self, centers):
            d = abs(centers-np.array([.5, 0, .5]))-[.1, .15, .2]
            return np.linalg.norm(np.maximum(d, 0), axis=-1)+np.minimum(d.max(axis=-1), 0)
    start, end = np.zeros(6), np.array([1., 0, 0, 0, 0, 0])
    anchors = [start+[0, .4, 0, 0, 0, 0], end+[0, .4, 0, 0, 0, 0]]
    robot, world = Geometry(), Wall()
    assert not guide_is_clear(robot, world, start, end)
    path = joint_route(robot, world, start, [end], anchors, budget=1.)
    assert path is not None and len(path) >= 3
    assert all(guide_is_clear(robot, world, a, b) for a, b in zip(path, path[1:]))
    np.testing.assert_allclose(path[-1], end)


def test_speed_requires_stable_maps_and_drops_on_latency():
    assert latency_speed(.3, .1, [], 10) == .08
    assert latency_speed(.3, .1, [.25]*8, 10) == .3
    assert latency_speed(.3, .4, [.25]*8, 10) == .24
    assert latency_speed(.3, .7, [.25]*8, 10) == .16
    assert latency_speed(.3, 1., [.25]*8, 10) == .08
    assert latency_speed(.3, 1.6, [.25]*8, 10) == 0.
    assert latency_speed(.3, .1, [.25]*8, 400) == .08
    assert latency_speed(.08, .1, [.25]*8, 10) == .08


def test_ray_self_filter_masks_noisy_robot_returns_but_preserves_foreground():
    intr = SimpleNamespace(cx=2., cy=2., fx=20., fy=20.)
    centers, radii, ids = np.array([[0., 0., .6]]), np.array([.1]), np.array([0])
    depth = np.ones((5, 5))  # Noisy/occluded return beyond the robot surface.
    assert robot_depth_mask(depth, np.eye(4), intr, centers, radii, ids)[2, 2]
    depth[:] = .3  # Visible foreground object, well in front of the robot.
    assert not robot_depth_mask(depth, np.eye(4), intr, centers, radii, ids)[2, 2]
    assert not robot_depth_mask(depth, np.eye(4), intr, np.zeros((1, 3)), np.array([.03]), ids).any()


def test_mapping_priority_and_reentrant_gpu_lock():
    lock, order = MappingPriorityLock(), []
    def planner():
        with lock: order.append('planner')
    def mapper():
        with lock.mapping(): order.append('mapper')
    with lock:
        with lock: pass
        a, b = threading.Thread(target=planner), threading.Thread(target=mapper)
        a.start(); b.start()
        deadline = time.monotonic()+1.
        while lock.mapping_waiters == 0 and time.monotonic() < deadline:
            time.sleep(.001)
        assert lock.mapping_waiters == 1
    a.join(1); b.join(1)
    assert not a.is_alive() and not b.is_alive()
    assert order == ['mapper', 'planner']


def test_exact_time_queue_uses_ready_older_image_without_latest_tf_fallback():
    from ur7e_perception_planning_core.frames import newest_ready
    from tf2_ros import TransformException
    calls = []
    def resolve(stamp):
        calls.append(stamp)
        if stamp > 20:
            raise TransformException('Future extrapolation')
        return f'exact transform at {stamp}'
    assert newest_ready([10, 20, 30], resolve) == (20, 'exact transform at 20')
    assert calls == [30, 20]
    assert newest_ready([30, 40], resolve) == (None, None)


def test_demo_rejects_sky_facing_path_even_in_free_space():
    robot = RobotGeometry()
    sky = np.array([-.319, -1.623, 1.722, -.220, 1.288, -.092])
    assert robot.camera_axis_z(sky)[0] > 0
    class Free:
        def query(self, points): return np.ones(points.shape[:-1])
    assert not guide_is_clear(robot, Free(), sky, sky)


def test_recovery_does_not_escape_below_execution_clearance():
    from ur7e_perception_planning_core.routing import recovery_step
    robot = RobotGeometry(); start = scan_configuration()[0][0]
    class Collision:
        def query(self, points): return np.full(points.shape[:-1], robot.radii.max()+.014)
    assert recovery_step(robot, Collision(), start) is None
