"""Offline camera-body sphere generation and runtime mounting consistency checks.

No ROS nodes or hardware connections. Mesh/Xacro dependencies are generation-only.
The union of the vendor collision box and visual mesh is conservatively covered;
this does not measure an installed bracket, cable or gripper.
"""
from pathlib import Path
import argparse
import hashlib
import itertools
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return transform


def fixed_transform(root, parent, child):
    """Require a wholly fixed descendant chain; never bake in a movable joint."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    transform = np.eye(4)
    visited = set()
    while child != parent:
        if child in visited or child not in joints:
            raise ValueError('Camera body must be a fixed descendant of tool0')
        visited.add(child)
        joint = joints[child]
        if joint.get('type') != 'fixed':
            raise ValueError('Camera mounting chain contains a moving joint')
        transform = origin(joint.find('origin')) @ transform
        child = joint.find('parent').get('link')
    return transform


def cover_box(low, high, count=6, padding=.005):
    """Partition the longest axis; each sphere encloses an entire closed slab.

    The half-diagonal radius proves coverage of the full volume, not just sampled
    vertices. Adding padding preserves that minimum clearance around the volume.
    """
    low, high = np.asarray(low, float), np.asarray(high, float)
    if (low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all()
            or np.any(high <= low) or type(count) is not int or count < 1 or count > 128
            or not np.isfinite(padding) or padding < 0):
        raise ValueError('Invalid camera box/cover parameters')
    half = (high-low)/2
    axis = int(np.argmax(half))
    centers = np.tile((low+high)/2, (count, 1))
    centers[:, axis] = low[axis]+(np.arange(count)+.5)*(high[axis]-low[axis])/count
    half[axis] /= count
    return centers, np.full(count, np.linalg.norm(half)+padding)


def validate_camera_cover(root, spheres, metadata):
    if metadata is None:  # Existing generic/manual payload profiles remain supported.
        return
    if metadata.get('schema_version') != 1 or metadata.get('parent_frame') != 'tool0':
        raise ValueError('Unsupported camera collision metadata')
    expected = np.asarray(metadata['T_tool0_body'], float)
    actual = fixed_transform(root, 'tool0', metadata['body_frame'])
    if expected.shape != (4, 4) or not np.allclose(actual, expected, atol=1e-9, rtol=0):
        raise ValueError('Camera collision mounting differs from URDF; regenerate spheres')
    centers, radii = cover_box(metadata['bounds_min'], metadata['bounds_max'],
                               metadata['sphere_count'], metadata['padding_m'])
    centers = centers @ actual[:3, :3].T + actual[:3, 3]
    # Camera spheres occupy the first slots; additional measured payload is allowed.
    if len(spheres) < len(centers):
        raise ValueError('Camera collision cover is incomplete')
    supplied = np.array([s['center'] for s in spheres[:len(centers)]])
    supplied_radii = np.array([s['radius'] for s in spheres[:len(centers)]])
    if (supplied.shape != centers.shape or not np.allclose(supplied, centers, atol=1e-9, rtol=0)
            or not np.allclose(supplied_radii, radii, atol=1e-9, rtol=0)):
        raise ValueError('Camera spheres do not match the conservative body cover')


def vendor_body(description):
    """Expand the installed D435i description, retaining mesh scene transforms."""
    import xacro
    import trimesh
    wrapper = ET.Element('robot', name='camera_collision_source',
                         attrib={'xmlns:xacro': 'http://ros.org/wiki/xacro'})
    ET.SubElement(wrapper, 'xacro:include', filename=str(Path(description).resolve()))
    ET.SubElement(wrapper, 'link', name='tool0')
    macro = ET.SubElement(wrapper, 'xacro:sensor_d435i', parent='tool0', name='camera',
                          use_nominal_extrinsics='true')
    ET.SubElement(macro, 'origin', xyz='0 0 0', rpy='0 0 0')
    document = xacro.parse(ET.tostring(wrapper, encoding='unicode'))
    xacro.process_doc(document)
    link = ET.fromstring(document.toxml()).find("link[@name='camera_link']")
    points, sources = [], []
    for kind in ['collision', 'visual']:
        for element in link.findall(kind):
            pose = origin(element.find('origin'))
            box, mesh = element.find('geometry/box'), element.find('geometry/mesh')
            if box is not None:
                half = np.fromstring(box.get('size'), sep=' ')/2
                vertices = np.array(list(itertools.product([-1, 1], repeat=3)))*half
            elif mesh is not None:
                filename = mesh.get('filename')
                if not filename.startswith('file://'):
                    raise ValueError('Expected an expanded file:// vendor mesh')
                path = Path(filename[7:])
                scene = trimesh.load(path, force='scene', process=False)
                vertices = np.asarray(scene.to_geometry().vertices)
                vertices = vertices*np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                sources.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            else:
                raise ValueError('Unsupported vendor camera geometry')
            points.append(vertices @ pose[:3, :3].T + pose[:3, 3])
    if not points:
        raise ValueError('No vendor body geometry')
    return np.concatenate(points), sources


def generate(urdf, arm_spheres, description):
    points, sources = vendor_body(description)
    transform = fixed_transform(ET.parse(urdf).getroot(), 'tool0', 'camera_link')
    # Round outwards, avoiding mesh-loader floating point differences at the boundary.
    low = np.floor(points.min(0)*1e6)/1e6
    high = np.ceil(points.max(0)*1e6)/1e6
    centers, radii = cover_box(low, high)
    centers = centers @ transform[:3, :3].T + transform[:3, 3]
    data = yaml.safe_load(Path(arm_spheres).read_text())
    data.pop('preview_notes', None)
    data['collision_spheres']['tool0'] = [dict(center=c.tolist(), radius=float(r))
                                          for c, r in zip(centers, radii)]
    data['link_names'] = list(data['collision_spheres'])
    data['camera_collision'] = dict(
        schema_version=1, parent_frame='tool0', body_frame='camera_link',
        T_tool0_body=transform.tolist(), bounds_min=low.tolist(), bounds_max=high.tolist(),
        sphere_count=6, padding_m=.005,
        method='equal closed slabs covering the union of vendor collision box and visual mesh',
        source_description=str(Path(description).resolve()), source_meshes=sources,
        source_urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
        physically_verified=False, unmodelled=['mount/bracket', 'gripper/tool', 'cable'])
    data['model_notes'] = ('Arm template unchanged. Camera body has a conservative 5 mm padded cover. '
                           'Self-collision margins and checked pairs are unchanged. '
                           'Installed bracket/tool/cable geometry still requires measurement.')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--arm-spheres', required=True)
    parser.add_argument('--description', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    data = generate(args.urdf, args.arm_spheres, args.description)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, sort_keys=False))
    print(output.resolve())


if __name__ == '__main__':
    main()
