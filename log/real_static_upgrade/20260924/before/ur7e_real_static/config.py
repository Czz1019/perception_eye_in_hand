from pathlib import Path
import hashlib
import json
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def load_config(path):
    defaults = yaml.safe_load((Path(get_package_share_directory('ur7e_real_static'))/'config/real.yaml').read_text())
    supplied = yaml.safe_load(Path(path).expanduser().read_text()) or {}
    if set(supplied)-set(defaults):
        raise ValueError(f'Unknown configuration fields: {set(supplied)-set(defaults)}')
    c = dict(defaults, **supplied)
    for key in ['robot_urdf', 'collision_spheres']:
        if not c[key]:
            raise ValueError(f'{key} is required; supply measured robot/camera model, not Gazebo defaults')
        p = Path(c[key]).expanduser()
        if not p.is_absolute(): p = Path(path).expanduser().resolve().parent/p
        if not p.is_file(): raise ValueError(f'Missing {key}: {p}')
        c[key] = str(p.resolve())
    for key in ['execution_enabled', 'commissioned', 'use_sim_time']:
        if type(c[key]) is not bool: raise ValueError(f'{key} must be boolean')
    if c['tool_frame'] != 'tool0': raise ValueError('First version requires tool_frame=tool0 (position target at tool0)')
    if c['base_frame'] != 'base_link': raise ValueError('First version requires base_frame=base_link')
    for key in ['max_joint_speed', 'max_joint_acceleration', 'collision_margin', 'self_margin',
                'joint_timeout', 'frame_timeout', 'start_tolerance', 'tracking_tolerance',
                'settle_speed', 'settle_seconds', 'view_separation_rad', 'max_plan_seconds',
                'voxel_size', 'grid_resolution', 'min_depth', 'max_depth']:
        if not np.isfinite(c[key]) or c[key] <= 0: raise ValueError(f'Invalid {key}')
    if c['max_joint_speed'] > .1 or c['max_joint_acceleration'] > .3:
        raise ValueError('First hardware version caps speed at .1 rad/s and acceleration at .3 rad/s^2')
    for key in ['min_frames', 'min_views', 'depth_stride', 'particles']:
        if type(c[key]) is not int or c[key] < 1: raise ValueError(f'Invalid {key}')
    for key in ['grid_center', 'grid_size']:
        a = np.asarray(c[key]);
        if a.shape != (3,) or not np.isfinite(a).all(): raise ValueError(f'Invalid {key}')
    if min(c['grid_size']) <= 0 or c['max_depth'] <= c['min_depth']: raise ValueError('Invalid range')
    if np.prod(np.ceil(np.array(c['grid_size'])/c['grid_resolution'])+1) > 2_000_000:
        raise ValueError('Grid exceeds 2 million cells')
    if c['commissioned_free_boxes'] and not c['commissioned']:
        raise ValueError('Free boxes require physical commissioning, never automatically cleared')
    for box in c['commissioned_free_boxes']:
        low, high = np.asarray(box['min']), np.asarray(box['max'])
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low,high]).all() or np.any(low >= high):
            raise ValueError('Invalid commissioned free box')
    if c['execution_enabled'] and c['use_sim_time']:
        raise ValueError('Real execution requires use_sim_time=false')
    if c['execution_enabled'] and not c['commissioned']:
        raise ValueError('Execution requires commissioned=true')
    return c


def identity(c):
    # Exact files and mapping parameters bind every frozen map and trajectory.
    keys = ['base_frame','tool_frame','camera_frame','voxel_size','grid_resolution','grid_center',
            'grid_size','min_depth','max_depth','depth_stride','commissioned_free_boxes']
    h = hashlib.sha256(json.dumps({k:c[k] for k in keys}, sort_keys=True).encode())
    for k in ['robot_urdf','collision_spheres']: h.update(Path(c[k]).read_bytes())
    return h.hexdigest()
