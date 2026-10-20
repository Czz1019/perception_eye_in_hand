from pathlib import Path
import hashlib
import json
import yaml
import numpy as np
from ur7e_real_static.config import load_config, identity as real_identity


def load_mapping(path):
    c = load_config(path)
    c.setdefault('stationary_processing_timeout',5.)
    if not c['frame_timeout']<=c['stationary_processing_timeout']<=10.:
        raise ValueError('Invalid static stationary processing budget')
    c['scan_config'] = str(Path(path).resolve().parent/'scan.yaml')
    scan=yaml.safe_load(Path(c['scan_config']).read_text())
    c['planning_policy']=scan.get('planning_policy','strict')
    if c['planning_policy'] not in ('strict','bounded_unknown'):raise ValueError('Unknown planning policy')
    views=np.asarray(scan['views'],dtype=float)
    if (c['execution_enabled'] or not c['use_sim_time'] or scan.get('fixture_only') is not True
            or views.ndim!=2 or views.shape[1]!=6 or not 2<=len(views)<=64 or not np.isfinite(views).all()):
        raise ValueError('Requires simulation-only configuration and finite six-joint scan views')
    for key in ['prior_clearance','max_scan_speed','scan_acceleration','rolling_speed','control_dt','wall_feedback_timeout']:
        if not np.isfinite(scan[key]) or scan[key]<=0:raise ValueError('Invalid scan parameter '+key)
    if scan['rolling_speed']>.6 or scan['max_scan_speed']>.6 or not .02<=scan['control_dt']<=.2:
        raise ValueError('Simulation speed/control interval exceeds supported limits')
    if type(scan['capture_frames']) is not int or scan['capture_frames']<1:
        raise ValueError('capture_frames must be a positive integer')
    cert=scan['commissioning']
    if cert['min_environment_gap']-cert['sampling_padding']<scan['prior_clearance']:
        raise ValueError('Scan prior exceeds its offline certified clearance')
    if cert['min_self_gap']-2*cert['sampling_padding']<c['self_margin']:
        raise ValueError('Scan self-clearance certification is insufficient')
    if c['commissioned_free_boxes']:
        raise ValueError('Planning does not accept commissioned free-space boxes')
    c['tabletop'] = yaml.safe_load((Path(path).resolve().parent/'tabletop.yaml').read_text())
    t=c['tabletop']
    if t.get('ray_evidence','filtered_depth') not in ('filtered_depth','raw_depth_before_returns'):
        raise ValueError('Unknown depth ray evidence policy')
    if t['policy']!='depth_tabletop_columns_v1' or not t.get('assumption'):
        raise ValueError('Explicit tabletop assumptions and current policy are required')
    for key in ['height_margin_m','max_patch_depth_span_m','view_translation_m','view_rotation_rad']:
        if not np.isfinite(t[key]) or t[key]<=0:raise ValueError('Invalid tabletop parameter '+key)
    if type(t['min_distinct_views']) is not int or not 2<=t['min_distinct_views']<=64:
        raise ValueError('At least two distinct views must support each column')
    if not .5<=t['min_upward_normal_z']<=1.:raise ValueError('Invalid upward normal threshold')
    if (np.shape(t['xy_min'])!=(2,) or np.shape(t['xy_max'])!=(2,)
        or not np.isfinite([*t['xy_min'],*t['xy_max'],t['z_min'],t['z_max']]).all()
        or np.any(np.array(t['xy_min'])>=t['xy_max']) or t['z_min']>=t['z_max']):
        raise ValueError('Invalid declared tabletop workspace')
    return c


def identity(c):
    # Versioned planning semantics reject maps made with the former scan prior.
    scan=yaml.safe_load(Path(c['scan_config']).read_text())
    geometry=dict(tabletop=c['tabletop'],depth_self_mask=scan['depth_self_mask'])
    return hashlib.sha256((real_identity(c)+json.dumps(geometry,sort_keys=True)).encode()).hexdigest()
