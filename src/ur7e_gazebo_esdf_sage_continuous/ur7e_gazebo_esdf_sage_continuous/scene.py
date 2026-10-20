"""Build one calibrated model for Gazebo, TF, depth pose and SAGE FK."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
import xacro


def build_robot(share):
    share = Path(share).resolve()
    scene = Path(get_package_share_directory('ur7e_d435_gazebo'))
    document = xacro.process_file(str(scene/'urdf/ur7e_d435.urdf.xacro'), mappings={
        'initial_positions_file': str(share/'config/initial_positions.yaml')})
    root = ET.fromstring(document.toxml())
    calibrated = ET.parse(share/'config/real_calibrated.urdf').getroot()
    camera_body = copy.deepcopy(root.find("link[@name='camera_link']"))
    for item in list(root):
        if item.tag == 'link' and (item.get('name', '').startswith('camera_') or item.get('name') == 'd435_mount_link'):
            root.remove(item)
        elif item.tag == 'joint' and (item.find('child').get('link', '').startswith('camera_') or item.find('child').get('link') == 'd435_mount_link'):
            root.remove(item)
        elif item.tag == 'gazebo' and item.get('reference') == 'd435_mount_link':
            root.remove(item)
    # Use the real factory kinematics on all corresponding arm joints.
    for joint in root.findall('joint'):
        actual = calibrated.find(f"joint[@name='{joint.get('name')}']")
        if actual is not None and actual.find('origin') is not None:
            if joint.find('origin') is not None: joint.remove(joint.find('origin'))
            joint.append(copy.deepcopy(actual.find('origin')))
    for link in calibrated.findall('link'):
        if link.get('name', '').startswith('camera_'):
            root.append(camera_body if link.get('name') == 'camera_link' else copy.deepcopy(link))
    for joint in calibrated.findall('joint'):
        if joint.find('child').get('link', '').startswith('camera_'):
            root.append(copy.deepcopy(joint))
    # IR streams are unused; their declared frames still match the sensor poses.
    for number, y in [(1, 0.), (2, -.05)]:
        link = f'camera_infra{number}_optical_frame'
        ET.SubElement(root, 'link', name=link)
        j = ET.SubElement(root, 'joint', name=f'infra{number}_fixed', type='fixed')
        ET.SubElement(j, 'parent', link='camera_link'); ET.SubElement(j, 'child', link=link)
        ET.SubElement(j, 'origin', xyz=f'0 {y} 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    # Gazebo renders along sensor +X; frame_id names alone do not rotate an image.
    # Copy the measured body->non-optical stream pose into each sensor pose.
    from ur7e_real_static.camera_collision import fixed_transform
    for sensor in root.findall("gazebo[@reference='camera_link']/sensor"):
        name = sensor.get('name')
        if name in ('cameradepth', 'cameracolor'):
            frame = 'camera_depth_frame' if name == 'cameradepth' else 'camera_color_frame'
            pose = fixed_transform(calibrated, 'camera_link', frame)
            text = ' '.join(map(str, [*pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_euler('xyz')]))
            if sensor.find('pose') is None: ET.SubElement(sensor, 'pose')
            sensor.find('pose').text = text
        sensor.find('update_rate').text = '10' if name == 'cameradepth' else '1'
        sensor.find('camera/image/width').text = '320'
        sensor.find('camera/image/height').text = '240'
        noise = sensor.find('camera/noise/stddev')
        if noise is not None: noise.text = '0.002'
    plugin = root.find("gazebo/plugin[@name='camera']")
    for tag, value in [('depthUpdateRate','10'),('colorUpdateRate','1'),('pointCloud','false')]:
        plugin.find(tag).text = value
    # A tiny conservative simulated shell, matching the derived union body box.
    collision = camera_body.find('collision')
    collision.find('origin').set('xyz', '-0.004115 -0.0174995 0')
    collision.find('geometry/box').set('size', '0.03328 0.090001 0.025002')
    return ET.tostring(root, encoding='unicode')
