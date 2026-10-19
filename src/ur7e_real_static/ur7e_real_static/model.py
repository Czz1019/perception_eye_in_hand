"""Use one commissioned URDF and sphere model for TF checks, CPU and SAGE."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ur7e_perception_planning_core.geometry import RobotGeometry, JOINTS


class Geometry(RobotGeometry):
    def __init__(self, urdf, spheres, tool='tool0', camera='camera_depth_optical_frame'):
        root = ET.parse(urdf).getroot()
        descendants = {'base_link'}
        joints = root.findall('joint')
        for _ in joints:
            for j in joints:
                if j.find('parent').get('link') in descendants:
                    descendants.add(j.find('child').get('link'))
        if not {tool,camera}.issubset(descendants):
            raise ValueError('URDF must include tool and measured camera fixed transform below base_link')
        normalized = ET.Element('robot', name='ur7e_commissioned')
        ET.SubElement(normalized, 'link', name='world')
        anchor = ET.SubElement(normalized,'joint',name='static_planning_origin',type='fixed')
        ET.SubElement(anchor,'parent',link='world'); ET.SubElement(anchor,'child',link='base_link')
        ET.SubElement(anchor,'origin',xyz='0 0 0',rpy='0 0 0')
        links={link.get('name'):link for link in root.findall('link')}
        children={}
        for joint in joints:
            children.setdefault(joint.find('parent').get('link'),[]).append(joint)
        visited=set()
        def append_branch(name):
            if name in visited:raise ValueError('URDF cycle or multiple parents')
            visited.add(name)
            link=copy.deepcopy(links[name])
            # Canonical topological link order is needed by Storm's joint indexing.
            # Collision checks use the commissioned spheres, not visual meshes.
            for tag in ['visual','collision']:
                for element in link.findall(tag):link.remove(element)
            normalized.append(link)
            for joint in children.get(name,[]):
                normalized.append(copy.deepcopy(joint));append_branch(joint.find('child').get('link'))
        append_branch('base_link')
        for joint in normalized.findall('joint'):
            if joint.find('origin') is None: ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0')
            if joint.get('type')!='fixed' and joint.find('axis') is None:ET.SubElement(joint,'axis',xyz='1 0 0')
        self.xml = ET.tostring(normalized, encoding='unicode')
        self.tool, self.camera = tool, camera
        self.joints, limits, velocities = [], {}, {}
        for j in normalized.findall('joint'):
            kind, name = j.get('type'), j.get('name')
            if kind not in ('fixed','revolute') or j.find('mimic') is not None:
                raise ValueError(f'Unsupported joint {name}; only fixed payload and six UR joints are supported')
            if kind != 'fixed' and name not in JOINTS: raise ValueError(f'Unexpected moving joint {name}')
            origin=j.find('origin'); pose=np.eye(4)
            if origin is not None:
                pose[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                pose[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            axis=j.find('axis'); axis=np.fromstring(axis.get('xyz','1 0 0'),sep=' ') if axis is not None else np.array([1.,0,0])
            self.joints.append((name,j.find('parent').get('link'),j.find('child').get('link'),pose,axis,kind))
            if kind != 'fixed':
                lim=j.find('limit')
                if lim is None: raise ValueError(f'Missing limits: {name}')
                limits[name]=(float(lim.get('lower','nan')),float(lim.get('upper','nan')))
                velocities[name]=float(lim.get('velocity','nan'))
                if not np.isclose(np.linalg.norm(axis),1.,atol=1e-6):raise ValueError('Moving joint axis must be normalized')
            if not np.isfinite(pose).all() or not np.isfinite(axis).all():raise ValueError('Non-finite URDF transform')
        if set(limits) != set(JOINTS): raise ValueError('Expected exactly the six canonical UR joint names (no prefix)')
        self.lower=np.array([limits[j][0] for j in JOINTS]);self.upper=np.array([limits[j][1] for j in JOINTS])
        self.velocity_limits=np.array([velocities[j] for j in JOINTS])
        if not np.isfinite([self.lower,self.upper,self.velocity_limits]).all() or np.any(self.lower>=self.upper) or np.any(self.velocity_limits<=0):
            raise ValueError('Invalid URDF joint limits')
        data=yaml.safe_load(Path(spheres).read_text()); groups=data['collision_spheres']
        from .camera_collision import validate_camera_cover
        validate_camera_cover(root, groups.get(tool, []), data.get('camera_collision'))
        arm=['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
        if not set(arm).issubset(groups) or tool not in groups:
            raise ValueError('Sphere file must cover all six arm links and payload on tool frame')
        if set(groups)-set(arm+[tool]): raise ValueError('First version expects six arm links plus tool payload group')
        self.links=arm+[tool]; self.local=[];self.radii=[];self.link_ids=[]
        for i,link in enumerate(self.links):
            if not groups[link]: raise ValueError(f'No collision spheres: {link}')
            for item in groups[link]:
                self.local.append(item['center']);self.radii.append(item['radius']);self.link_ids.append(i)
        self.local=np.asarray(self.local);self.radii=np.asarray(self.radii);self.link_ids=np.asarray(self.link_ids)
        if self.local.shape != (len(self.radii),3) or not np.isfinite(self.local).all() or not np.isfinite(self.radii).all() or np.any(self.radii<=0):
            raise ValueError('Invalid sphere geometry')
        self.pairs=[(a,b) for a in range(len(self.radii)) for b in range(a) if abs(self.link_ids[a]-self.link_ids[b])>1]
        self.pair_a=np.array([a for a,b in self.pairs]);self.pair_b=np.array([b for a,b in self.pairs])
        # Triangle bound on link point radius about any joint, computed from supplied geometry.
        self.motion_bound=sum(np.linalg.norm(j[3][:3,3]) for j in self.joints)+float(np.linalg.norm(self.local,axis=1).max())

    def camera_axis_z(self, configurations):
        # A frozen-map trajectory has no simulated camera-down constraint.
        return np.full(len(np.atleast_2d(configurations)), -1.)

    def write_urdf(self, path):
        Path(path).write_text(self.xml)
        return str(Path(path).resolve())
