from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np,yaml
from scipy.spatial.transform import Rotation
root=Path('/home/wqj/realtime_nvblox_ws');run=root/'log/real_first_scan_20260927';out=Path('/home/wqj/ur7e_real_setup')
record=json.loads((run/'camera_calibration.json').read_text())['transforms']
handeye=yaml.safe_load((root/'src/ur7e_real_static/config/handeye_d435i.yaml').read_text());tc=np.array(handeye['matrix']);tc[:3,:3]=Rotation.from_matrix(tc[:3,:3]).as_matrix()
def matrix(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v['xyzw']).as_matrix();t[:3,3]=v['xyz'];return t
body_color=matrix(record['camera_link <- camera_color_optical_frame']);mount=tc@np.linalg.inv(body_color)
xml=ET.parse(run/'robot_driver.urdf');robot=xml.getroot()
names={l.attrib['name'] for l in robot.findall('link')}
links=['camera_link','camera_depth_frame','camera_depth_optical_frame','camera_color_frame','camera_color_optical_frame']
assert not names.intersection(links)
for name in links:ET.SubElement(robot,'link',name=name)
def joint(parent,child,t):
    j=ET.SubElement(robot,'joint',name=child+'_measured_fixed_joint',type='fixed')
    ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
    ET.SubElement(j,'origin',xyz=' '.join(format(v,'.17g') for v in t[:3,3]),rpy=' '.join(format(v,'.17g') for v in Rotation.from_matrix(t[:3,:3]).as_euler('xyz')))
joint('tool0','camera_link',mount)
for key in ['camera_link <- camera_depth_frame','camera_depth_frame <- camera_depth_optical_frame','camera_link <- camera_color_frame','camera_color_frame <- camera_color_optical_frame']:
    parent,child=key.split(' <- ');joint(parent,child,matrix(record[key]))
tool_depth=mount@matrix(record['camera_link <- camera_depth_frame'])@matrix(record['camera_depth_frame <- camera_depth_optical_frame'])
np.testing.assert_allclose(tool_depth,tc@matrix(record['camera_color_optical_frame <- camera_depth_optical_frame']),atol=1e-12)
path=out/'calibrated_with_camera.urdf';assert not path.exists();ET.indent(xml);xml.write(path,encoding='unicode')
xyz=mount[:3,3];q=Rotation.from_matrix(mount[:3,:3]).as_quat()
command=['ros2','run','tf2_ros','static_transform_publisher','--frame-id','tool0','--child-frame-id','camera_link']
for name,value in zip(['x','y','z','qx','qy','qz','qw'],[*xyz,*q]):command.extend(['--'+name,str(value)])
import shlex
(run/'start_camera_mount.sh').write_text('#!/usr/bin/env bash\nset -eo pipefail\nsource /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=0\nexec '+shlex.join(command)+'\n')
summary=dict(camera_serial='349622072343',factory_calibration_hash='calib_16570280516586036686',tool0_to_camera_link=mount.tolist(),tool0_to_depth_optical=tool_depth.tolist(),handeye_color_chain_max_error=float(np.max(abs(mount@body_color-tc))))
(out/'camera_mount.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2));print(path)
