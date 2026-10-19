"""Simulation-only test data; never installed as a real commissioning profile."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_real_static.config import load_config,identity
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap
from ur7e_perception_planning_core.grid import grid_points

INITIAL=np.array([.15747077968146944,-1.6722293577794138,.5533633396871515,-.5892178762164055,-1.7765728296143388,.1586121657343889])


def config(directory):
    d=Path(directory);d.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('ur7e_sage_planner'))
    root=ET.parse(share/'assets/urdf/ur7e/ur7e.urdf').getroot()
    ET.SubElement(root,'link',name='camera_depth_optical_frame')
    j=ET.SubElement(root,'joint',name='test_camera_fixed',type='fixed')
    ET.SubElement(j,'parent',link='tool0');ET.SubElement(j,'child',link='camera_depth_optical_frame')
    ET.SubElement(j,'origin',xyz='-0.0325 -0.075 0.0357',rpy='0 0 0')
    (d/'robot.urdf').write_text(ET.tostring(root,encoding='unicode'))
    spheres=yaml.safe_load((share/'config/ur7e_collision_spheres.yml').read_text())
    spheres['collision_spheres']['tool0']=[dict(center=[x,-.075,.027],radius=.030) for x in [-.03,0.,.03]]
    spheres['collision_spheres']['tool0'].append(dict(center=[0.,-.040,.007],radius=.024))
    (d/'spheres.yaml').write_text(yaml.safe_dump(spheres))
    supplied=dict(robot_urdf=str(d/'robot.urdf'),collision_spheres=str(d/'spheres.yaml'),output_directory=str(d/'output'),
                  grid_center=[0,0,.6],grid_size=[2.,2.,1.5],min_frames=3,min_views=2,settle_seconds=.15,
                  max_plan_seconds=120.,trajectory_action='/mock/follow_joint_trajectory',joint_topic='/mock/joint_states',
                  depth_topic='/mock/depth',camera_info_topic='/mock/info')
    (d/'config.yaml').write_text(yaml.safe_dump(supplied))
    c=load_config(d/'config.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
    return c,g


def free_map(c):
    origin,dims,_=grid_points(c['grid_center'],c['grid_size'],c['grid_resolution'])
    return FrozenMap.create(np.full(np.prod(dims),.5),np.ones(np.prod(dims),'u1'),origin,dims,c['grid_resolution'],
                            dict(identity=identity(c),stamp=1.,epoch=1,sequence=10,views=[INITIAL.tolist()]))
