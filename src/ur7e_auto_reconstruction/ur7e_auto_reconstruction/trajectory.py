"""Offline joint-space path generation; no hardware imports or connections."""
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from ur7e_real_static.model import Geometry
from ur7e_perception_planning_core.geometry import JOINTS


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def vector(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('Expected six finite joint angles, without angle wrapping')
    return q


class Segment:
    def __init__(self, a, b, speed, acceleration):
        self.a, self.b = vector(a).copy(), vector(b).copy()
        if not np.isfinite([speed, acceleration]).all() or min(speed, acceleration) <= 0:
            raise ValueError('Invalid velocity/acceleration limit')
        distance = float(np.max(abs(self.b-self.a)))
        # Quintic maxima: s'=1.875, |s''|=10/sqrt(3).
        # Half the acceleration budget is reserved for time-scaling changes.
        self.duration = max(.5, 1.875*distance/speed,
                            np.sqrt((10/np.sqrt(3))*distance/(acceleration*.5)))

    def sample(self, t):
        u = np.clip(np.asarray(t)/self.duration, 0., 1.)
        s = 10*u**3-15*u**4+6*u**5
        return self.a + np.expand_dims(s, -1)*(self.b-self.a)


def load_inputs(mapping_path, control_path, waypoints_path):
    from ur7e_real_reconstruction.inputs import load_config
    mapping = load_config(mapping_path)
    control = yaml.safe_load(Path(control_path).read_text())
    expected = {'robot_ip','frequency_hz','max_joint_speed','max_joint_acceleration','lookahead_time','gain',
                'feedback_timeout_sec','cycle_timeout_sec','tracking_error_rad','start_tolerance_rad',
                'arrival_tolerance_rad','settle_speed_rad_s','settle_sec','capture_frames','capture_timeout_sec',
                'service_timeout_sec','mapper_timeout_sec','stop_deceleration','output_directory'}
    if set(control) != expected:
        raise ValueError(f'Unexpected/missing controller keys: {set(control)^expected}')
    for key in expected-{'robot_ip','output_directory'}:
        if not np.isfinite(control[key]) or control[key] <= 0:
            raise ValueError(f'Invalid control setting {key}')
    if not (125 <= control['frequency_hz'] <= 500 and .03 <= control['lookahead_time'] <= .2
            and 100 <= control['gain'] <= 2000 and control['max_joint_speed'] <= .3
            and control['max_joint_acceleration'] <= .5):
        raise ValueError('Controller settings outside the commissioning envelope')
    if type(control['capture_frames']) is not int:
        raise ValueError('capture_frames must be an integer')
    route = yaml.safe_load(Path(waypoints_path).read_text())
    if route['frame_id'] != 'base_link' or route['joint_names'] != JOINTS or not route['waypoints']:
        raise ValueError('Invalid route frame, joints or empty route')
    if route['collision_policy'] != 'operator_confirmed_clear_overhead':
        raise ValueError('This scanner only supports the explicitly confirmed clear-overhead policy')
    for key, sha in route['model_sha256'].items():
        if hashlib.sha256(Path(mapping[key]).read_bytes()).hexdigest() != sha:
            raise ValueError(f'Commissioned model changed: {key}; reselect/check observation poses')
    geometry = Geometry(mapping['robot_urdf'], mapping['collision_spheres'])
    if control['max_joint_speed'] > min(geometry.velocity_limits):
        raise ValueError('Speed exceeds URDF limits')
    names = [w['name'] for w in route['waypoints']]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate waypoint names')
    for w in route['waypoints']:
        q = vector(w['q_rad'])
        if np.any(q < geometry.lower+.01) or np.any(q > geometry.upper-.01):
            raise ValueError('Waypoint outside joint limits')
        for frame in ['tool0','camera_depth_optical_frame','camera_color_optical_frame']:
            if not np.allclose(geometry.fk(q)[frame], w[frame]['T_base_frame'], atol=1e-6):
                raise ValueError(f'Waypoint/calibration inconsistency: {w["name"]} {frame}')
    return mapping, control, route, geometry


def make_plan(q0, control, route, geometry, source='live_rtde'):
    q0 = vector(q0)
    if np.any(q0 < geometry.lower+.01) or np.any(q0 > geometry.upper-.01):
        raise ValueError('Initial state outside joint limits')
    segments, previous = [], q0
    for w in route['waypoints']:
        s = Segment(previous, w['q_rad'], control['max_joint_speed'], control['max_joint_acceleration'])
        qs = s.sample(np.linspace(0, s.duration, max(101, int(s.duration*20))))
        poses = geometry.poses_batch(qs)
        camera = poses['camera_depth_optical_frame'][:, :3, 3]
        segments.append(dict(to=w['name'], q_start=previous.tolist(), q_end=s.b.tolist(),
                             duration_sec=s.duration, max_delta_deg=float(np.rad2deg(abs(s.b-previous)).max()),
                             camera_min_z_m=float(camera[:, 2].min()),
                             camera_path_m=camera.tolist()))
        previous = s.b
    plan = dict(schema_version=1, source=source, frame_id='base_link', q_start=q0.tolist(),
                collision_checked=False, self_collision_checked=False,
                policy=route['collision_policy'], model_sha256=route['model_sha256'],
                control=control, route_digest=digest(route), segments=segments,
                nominal_motion_sec=sum(s['duration_sec'] for s in segments),
                capture_frames_per_view=control['capture_frames'],
                limitations=route['unresolved'])
    plan['sha256'] = digest(plan)
    return plan


def verify_plan(plan, q, control, route, geometry):
    unsigned = {k:v for k,v in plan.items() if k != 'sha256'}
    if digest(unsigned) != plan.get('sha256') or plan.get('source') != 'live_rtde':
        raise ValueError('Plan must be an intact live RTDE preview')
    rebuilt = make_plan(plan['q_start'], control, route, geometry)
    if rebuilt['sha256'] != plan['sha256']:
        raise ValueError('Plan inputs changed; prepare and review again')
    if np.max(abs(vector(q)-vector(plan['q_start']))) > control['start_tolerance_rad']:
        raise ValueError('Robot moved since preview; prepare and review again')
    return rebuilt
