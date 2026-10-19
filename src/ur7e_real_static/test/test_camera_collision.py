import itertools
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from fixtures import config, free_map
from ur7e_real_static.camera_collision import cover_box, fixed_transform
from ur7e_real_static.config import identity
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry


def test_cover_includes_volume_slab_boundaries_and_padding():
    low = np.array([-.020755, -.0625, -.012501])
    high = np.array([.012525, .027501, .012501])
    centers, radii = cover_box(low, high)
    # All slab corners are extrema of distance to their sphere. Also sample the
    # interior and sphere seams independently of the generation formula.
    corners = np.array(list(itertools.product([low[0], high[0]],
                         np.linspace(low[1], high[1], 7), [low[2], high[2]])))
    points = np.r_[corners, np.random.default_rng(8).uniform(low, high, (20000, 3))]
    distance = np.min(np.linalg.norm(points[:, None]-centers, axis=-1)-radii, axis=1)
    assert distance.max() <= -.005+1e-12


def camera_profile(tmp_path):
    c, _ = config(tmp_path)
    root = ET.parse(c['robot_urdf']).getroot()
    ET.SubElement(root, 'link', name='camera_link')
    joint = ET.SubElement(root, 'joint', name='body_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='tool0')
    ET.SubElement(joint, 'child', link='camera_link')
    ET.SubElement(joint, 'origin', xyz='-.04 -.06 -.012', rpy='.1 .2 .3')
    ET.ElementTree(root).write(c['robot_urdf'])
    transform = fixed_transform(root, 'tool0', 'camera_link')
    low, high = [-.020755, -.0625, -.012501], [.012525, .027501, .012501]
    centers, radii = cover_box(low, high)
    centers = centers @ transform[:3, :3].T + transform[:3, 3]
    data = yaml.safe_load(open(c['collision_spheres']))
    data['camera_collision'] = dict(schema_version=1, parent_frame='tool0', body_frame='camera_link',
        T_tool0_body=transform.tolist(), bounds_min=low, bounds_max=high, sphere_count=6, padding_m=.005)
    data['collision_spheres']['tool0'] = [dict(center=p.tolist(), radius=float(r)) for p, r in zip(centers, radii)]
    with open(c['collision_spheres'], 'w') as f:
        yaml.safe_dump(data, f)
    return c, data


def test_calibrated_cover_and_existing_pairs(tmp_path):
    c, data = camera_profile(tmp_path)
    g = Geometry(c['robot_urdf'], c['collision_spheres'])
    assert len(g.radii) == 22
    payload = np.flatnonzero(g.link_ids == g.links.index('tool0'))
    wrist2 = np.flatnonzero(g.link_ids == g.links.index('wrist_2_link'))
    assert all((int(a), int(b)) in g.pairs for a in payload for b in wrist2)
    q = np.array([.2, -.8, .6, -.4, .3, .1])
    np.testing.assert_allclose(g.spheres(q), g.spheres_batch([q])[0], atol=1e-12)


def test_different_mount_is_rejected(tmp_path):
    c, _ = camera_profile(tmp_path)
    root = ET.parse(c['robot_urdf']).getroot()
    root.find("joint[@name='body_mount']/origin").set('xyz', '0 0 .1')
    ET.ElementTree(root).write(c['robot_urdf'])
    with pytest.raises(ValueError, match='mounting differs'):
        Geometry(c['robot_urdf'], c['collision_spheres'])


@pytest.mark.parametrize('change', ['radius', 'center', 'missing'])
def test_cover_cannot_be_silently_shrunk(tmp_path, change):
    c, data = camera_profile(tmp_path)
    spheres = data['collision_spheres']['tool0']
    if change == 'radius': spheres[0]['radius'] -= .003
    elif change == 'center': spheres[0]['center'][0] += .02
    else: spheres.pop()
    with open(c['collision_spheres'], 'w') as f:
        yaml.safe_dump(data, f)
    with pytest.raises(ValueError, match='cover'):
        Geometry(c['robot_urdf'], c['collision_spheres'])


def test_movable_mount_and_old_map_rejected(tmp_path):
    c, _ = camera_profile(tmp_path)
    previous = free_map(c)
    path = tmp_path/'old_map.npz'
    previous.save(path)
    root = ET.parse(c['robot_urdf']).getroot()
    root.find("joint[@name='body_mount']").set('type', 'revolute')
    with pytest.raises(ValueError, match='moving joint'):
        fixed_transform(root, 'tool0', 'camera_link')
    # Even a different valid sphere-model file invalidates the old frozen map.
    with open(c['collision_spheres'], 'a') as f: f.write('\n# changed profile\n')
    with pytest.raises(ValueError, match='identity mismatch'):
        FrozenMap.load(path, identity(c))
