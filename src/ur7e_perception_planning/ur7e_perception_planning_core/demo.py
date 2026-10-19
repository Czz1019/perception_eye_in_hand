"""Static-fixture observation preparation and SAGE-only point reaching.

The bounded observation corridor is commissioned offline against the fixture.
Unknown cells outside it are never made free merely to reach a user goal.
"""
from pathlib import Path
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from ament_index_python.packages import get_package_share_directory
from .geometry import JOINTS


def scan_configuration():
    share = Path(get_package_share_directory('ur7e_perception_planning'))
    config = yaml.safe_load((share/'config/demo_scan.yaml').read_text())
    root = Path(get_package_share_directory('ur7e_reconstruction'))
    initial = yaml.safe_load((root/'config/high_initial_positions.yaml').read_text())
    q = np.array([initial[j] for j in JOINTS])
    return np.asarray(config['joint_offsets'])+q, config


def scan_samples(views):
    samples = [views[0]]
    for a, b in zip(views, views[1:]):
        count = max(2, int(np.ceil(np.max(np.abs(b-a))/.02))+1)
        samples.extend(np.linspace(a, b, count)[1:])
    return np.asarray(samples)


def scan_prior(points, geometry):
    views, config = scan_configuration()
    centers = np.array([geometry.spheres(q) for q in scan_samples(views)])
    free = np.zeros(len(points), bool)
    for radius in np.unique(geometry.radii):
        group = np.unique(centers[:, geometry.radii == radius].reshape(-1, 3), axis=0)
        padding = radius+config['clearance_m']
        inside = ((points >= group.min(0)-padding) & (points <= group.max(0)+padding)).all(1)
        ids = np.flatnonzero(inside & ~free)
        distances = cKDTree(group).query(points[ids], workers=1)[0]
        free[ids[distances < padding]] = True
    return free


def camera_axis_z(geometry, qs):
    # Synthetic geometry without an attached sensor has no view constraint.
    if not hasattr(geometry, 'camera_axis_z'):
        return np.full(len(np.atleast_2d(qs)), -1.)
    return geometry.camera_axis_z(qs)


def goal_candidates(geometry, snapshot, start, xyz):
    """Map-checked endpoint IK branches; paths still require separate checks."""
    def residual(q):
        centers = geometry.spheres(q)
        collision = np.maximum(0., .03-(snapshot.query(centers)-geometry.radii))
        return np.r_[15*(geometry.fk(q)['tool0'][:3, 3]-xyz),
                     .015*(q-start), 5*collision,
                     5*max(0., .012-geometry.self_clearance(centers)),
                     5*max(0., camera_axis_z(geometry, q)[0]+.30)]
    seeds = [start.copy()]
    for elbow in (.6, -.6, 1.1):
        q = start.copy(); q[2] += elbow; q[3] -= elbow*.5
        seeds.append(q)
    candidates = []
    for seed in seeds:
        result = least_squares(residual, np.clip(seed, geometry.lower+.001, geometry.upper-.001),
                               bounds=(geometry.lower, geometry.upper), max_nfev=90,
                               diff_step=1e-3)
        q = result.x
        error = np.linalg.norm(geometry.fk(q)['tool0'][:3, 3]-xyz)
        clearance = np.min(snapshot.query(geometry.spheres(q))-geometry.radii)
        if (error < .008 and clearance >= .022 and geometry.self_clearance(geometry.spheres(q)) > .012
                and camera_axis_z(geometry, q)[0] <= -.25):
            candidates.append(q)
    return sorted(candidates, key=lambda q: np.linalg.norm(q-start))


def goal_ik(geometry, snapshot, start, xyz):
    """Find a goal branch with a checked joint interpolation."""
    candidates = goal_candidates(geometry, snapshot, start, xyz)
    for q in candidates:
        if guide_is_clear(geometry, snapshot, start, q):
            return q
    # Position-only goals have redundant wrist/elbow choices. An endpoint-only
    # IK can pick a valid final pose whose interpolation crosses unknown space.
    # Refine the final posture against intermediate whole-arm configurations;
    # the coarse objective is only a proposal, the dense guard is authoritative.
    def path_residual(q):
        qs = np.linspace(start, q, 9)[1:]
        centers = (geometry.spheres_batch(qs) if hasattr(geometry, 'spheres_batch')
                   else np.asarray([geometry.spheres(p) for p in qs]))
        clearance = snapshot.query(centers)-geometry.radii
        self_clearance = np.asarray([geometry.self_clearance(c) for c in centers])
        return np.r_[30*(geometry.fk(q)['tool0'][:3, 3]-xyz), .005*(q-start),
                     10*np.maximum(0., .032-clearance).ravel(),
                     10*np.maximum(0., .012-self_clearance),
                     5*np.maximum(0., camera_axis_z(geometry, qs)+.30)]
    for seed in sorted(candidates, key=lambda q: np.linalg.norm(q-start))[:2]:
        result = least_squares(path_residual, seed, bounds=(geometry.lower, geometry.upper),
                               max_nfev=100, diff_step=1e-3)
        q = result.x
        if (np.linalg.norm(geometry.fk(q)['tool0'][:3, 3]-xyz) < .008
                and guide_is_clear(geometry, snapshot, start, q)):
            return q
    return None


def guide_is_clear(geometry, snapshot, start, end):
    # Refine sampling near the 2 cm planning margin instead of rejecting a
    # safe current pose just because a coarse interval adds 3 mm of padding.
    # The physical margin and the 1.4 m/rad motion bound are unchanged.
    if not np.isfinite(end).all() or not np.isfinite(start).all():
        return False
    if hasattr(geometry, 'lower') and (np.any(end < geometry.lower) or np.any(end > geometry.upper)):
        return False
    if hasattr(geometry, 'camera_axis_z'):
        angular_travel = float(np.sum(abs(end-start)))
        view_count = max(2, int(np.ceil(angular_travel/.04))+1)
        # Axis rotation is 1-Lipschitz in total joint travel. Include the
        # half-sample allowance so the entire edge keeps observing downward.
        if np.max(geometry.camera_axis_z(np.linspace(start, end, view_count))) > -.25-angular_travel/(2*(view_count-1)):
            return False
    endpoints = np.asarray([geometry.spheres(start), geometry.spheres(end)])
    slack = min(float(np.min(snapshot.query(endpoints.reshape(-1, 3)).reshape(2, -1)-geometry.radii))-.020,
                float(np.min(endpoints[..., 2]-geometry.radii))-.020,
                min(geometry.self_clearance(c) for c in endpoints)/2-.004)
    if slack <= 0:
        return False
    interval = min(.006, 1.8*slack)
    count = max(2, int(np.ceil(1.4*np.sum(abs(end-start))/interval))+1)
    if count > 20000:
        return False
    padding = 1.4*np.sum(abs(end-start))/(2*(count-1))
    if hasattr(geometry, 'spheres_batch'):
        centers = geometry.spheres_batch(np.linspace(start, end, count))
        return bool(np.min(snapshot.query(centers)-geometry.radii) >= .020+padding
                    and np.min(geometry.self_clearance_batch(centers)) >= .008+2*padding
                    and np.min(centers[..., 2]-geometry.radii) >= .020+padding)
    for q in np.linspace(start, end, count):
        centers = geometry.spheres(q)
        if (np.min(snapshot.query(centers)-geometry.radii) < .020+padding
                or geometry.self_clearance(centers) < .008+2*padding
                or np.min(centers[:, 2]-geometry.radii) < .020+padding):
            return False
    return True


def velocity_proposal(start, goal, durations, maximum):
    """A braking proposal over SAGE's actual (nonuniform) prediction horizon.

    Repeating the first velocity throughout the horizon overshoots nearby
    observation poses, so SAGE incorrectly anticipates leaving the known tube.
    This is an optimization seed, never an executable bypass of SAGE/guard.
    """
    predicted = np.asarray(start).copy()
    actions = []
    for dt in durations:
        delta = goal-predicted
        velocity = 1.2*delta
        # Preserve joint ratios so a multi-joint scan follows the commissioned
        # segment instead of reaching short-travel joints much earlier.
        velocity *= min(1., maximum/max(float(np.max(abs(velocity))), 1e-12))
        velocity = np.sign(velocity)*np.minimum(abs(velocity), abs(delta)/dt)
        actions.append(velocity)
        predicted += velocity*dt
    return np.asarray(actions)


def run_demo(node, task):
    from .adaptive_demo import run_demo as run_adaptive
    return run_adaptive(node, task)
