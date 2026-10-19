"""Small CPU FK for the independent execution guard and depth self filtering."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from ament_index_python.packages import get_package_share_directory

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class RobotGeometry:
    def __init__(self):
        share = Path(get_package_share_directory('ur7e_sage_planner'))
        root = ET.parse(share / 'assets/urdf/ur7e/ur7e.urdf').getroot()
        self.joints = []
        limits = {}
        for joint in root.findall('joint'):
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            matrix[:3, 3] = xyz
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.array([1., 0., 0.])
            self.joints.append((joint.get('name'), joint.find('parent').get('link'),
                                joint.find('child').get('link'), matrix, axis, joint.get('type')))
            if joint.get('name') in JOINTS:
                limit = joint.find('limit')
                limits[joint.get('name')] = (float(limit.get('lower', '-6.283185')), float(limit.get('upper', '6.283185')))
        self.lower = np.array([limits[j][0] for j in JOINTS])
        self.upper = np.array([limits[j][1] for j in JOINTS])
        data = yaml.safe_load((share / 'config/ur7e_collision_spheres.yml').read_text())
        spheres = data['collision_spheres']
        # Conservative overlapping spheres covering the D435 body and side bracket,
        # expressed in tool0 (the calibrated mounting frame in this scene).
        spheres['tool0'] = [dict(center=[x, -.075, .027], radius=.030) for x in [-.03, 0., .03]]
        spheres['tool0'] += [dict(center=[0., -.040, .007], radius=.024)]
        self.links, self.local, self.radii, self.link_ids = list(spheres), [], [], []
        for i, (link, items) in enumerate(spheres.items()):
            for item in items:
                self.local.append(item['center']); self.radii.append(item['radius']); self.link_ids.append(i)
        self.local, self.radii = np.asarray(self.local), np.asarray(self.radii)
        self.link_ids = np.asarray(self.link_ids)
        # Only the same link and immediately adjacent links are excluded.
        # In particular shoulder/forearm and upper-arm/wrist pairs are checked.
        self.pairs = [(a, b) for a in range(len(self.radii)) for b in range(a)
                      if abs(self.link_ids[a] - self.link_ids[b]) > 1]
        self.pair_a = np.array([a for a, b in self.pairs])
        self.pair_b = np.array([b for a, b in self.pairs])

    def fk(self, q):
        values = dict(zip(JOINTS, q))
        poses, pending = {'world': np.eye(4)}, list(self.joints)
        while pending:
            remaining = []
            for name, parent, child, origin, axis, kind in pending:
                if parent not in poses:
                    remaining.append((name, parent, child, origin, axis, kind)); continue
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = Rotation.from_rotvec(axis * values[name]).as_matrix()
                poses[child] = poses[parent] @ origin @ motion
            if len(remaining) == len(pending):
                raise ValueError('Disconnected robot tree')
            pending = remaining
        return poses

    def spheres(self, q):
        poses = self.fk(q)
        centers = np.empty_like(self.local)
        for i, link in enumerate(self.links):
            mask = self.link_ids == i
            centers[mask] = self.local[mask] @ poses[link][:3, :3].T + poses[link][:3, 3]
        return centers

    def self_clearance(self, centers):
        return float(np.min(np.linalg.norm(centers[self.pair_a]-centers[self.pair_b], axis=1)
                            -self.radii[self.pair_a]-self.radii[self.pair_b]))

    def poses_batch(self, configurations):
        """Vectorized CPU FK using the same URDF."""
        q = np.atleast_2d(configurations)
        poses = {'world': np.broadcast_to(np.eye(4), (len(q), 4, 4))}
        pending = list(self.joints)
        while pending:
            remaining = []
            for name, parent, child, origin, axis, kind in pending:
                if parent not in poses:
                    remaining.append((name, parent, child, origin, axis, kind)); continue
                if kind in ('revolute', 'continuous'):
                    motion = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
                    motion[:, :3, :3] = Rotation.from_rotvec(q[:, JOINTS.index(name), None]*axis).as_matrix()
                    poses[child] = poses[parent] @ origin @ motion
                else:
                    poses[child] = poses[parent] @ origin
            if len(remaining) == len(pending):
                raise ValueError('Disconnected robot tree')
            pending = remaining
        return poses

    def camera_axis_z(self, configurations):
        # This demo fixture has optical +Z aligned with tool0 +Z.
        return self.poses_batch(configurations)['tool0'][:, 2, 2]

    def spheres_batch(self, configurations):
        q = np.atleast_2d(configurations)
        poses = self.poses_batch(q)
        centers = np.empty((len(q), len(self.radii), 3))
        for i, link in enumerate(self.links):
            ids = self.link_ids == i
            centers[:, ids] = self.local[ids] @ poses[link][:, :3, :3].transpose(0, 2, 1) + poses[link][:, None, :3, 3]
        return centers

    def self_clearance_batch(self, centers):
        return np.min(np.linalg.norm(centers[..., self.pair_a, :]-centers[..., self.pair_b, :], axis=-1)
                      -self.radii[self.pair_a]-self.radii[self.pair_b], axis=-1)
