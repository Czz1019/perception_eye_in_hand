"""Frozen robot-owned solids, distinct from sensed environmental free space.

Only cells wholly inside an uninflated, convex URDF collision solid at the
freeze pose can be owned. No union over historical or future robot poses.
"""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

POLICY = 'frozen_urdf_interior_cells_v1'


def contained_cells(points, resolution, vertices, normals, face_points):
    """Convex half-space proof for the entire axis-aligned voxel, not its centre."""
    result = np.zeros(len(points), bool)
    ids = np.flatnonzero(np.all((points >= vertices.min(0)) & (points <= vertices.max(0)), axis=1))
    offset = np.sum(normals*face_points, axis=1)
    support = resolution/2*np.abs(normals).sum(axis=1)
    for start in range(0, len(ids), 256):
        batch = ids[start:start+256]
        result[batch] = np.all(points[batch]@normals.T + support <= offset-1e-8, axis=1)
    return result


class RobotOwnership:
    def __init__(self, urdf, geometry):
        import trimesh
        self.geometry = geometry
        self.solids = []
        path = Path(urdf)
        self.provenance = dict(policy=POLICY, urdf=str(path.resolve()),
                               urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), meshes=[])
        for link in ET.parse(path).getroot().findall('link'):
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    # No approximation of unsupported primitives or visual envelopes.
                    continue
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    package, relative = filename[len('package://'):].split('/', 1)
                    file = Path(get_package_share_directory(package))/relative
                else:
                    file = path.parent/filename
                surface = trimesh.load(file, force='mesh', process=True)
                if not surface.is_watertight or not surface.is_convex or not surface.is_winding_consistent or surface.volume <= 0:
                    raise ValueError('Ownership requires a closed convex mesh with consistent winding: '+str(file))
                scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                    raise ValueError('Invalid ownership mesh scale')
                surface.apply_scale(scale)
                origin = collision.find('origin');transform = np.eye(4)
                if origin is not None:
                    transform[:3,3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                    transform[:3,:3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                vertices = np.asarray(surface.vertices)@transform[:3,:3].T+transform[:3,3]
                normals = np.asarray(surface.face_normals)@transform[:3,:3].T
                faces = np.asarray(surface.triangles_center)@transform[:3,:3].T+transform[:3,3]
                self.solids.append((link.get('name'), vertices, normals, faces))
                self.provenance['meshes'].append(dict(link=link.get('name'), path=str(file), sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
        if not self.solids:raise ValueError('No verified URDF ownership solids')

    def mask(self, points, resolution, q):
        poses = self.geometry.fk(q)
        owned = np.zeros(len(points), bool);counts = {}
        for link, vertices, normals, faces in self.solids:
            pose = poses[link];r=pose[:3,:3];t=pose[:3,3]
            inside = contained_cells(points, resolution, vertices@r.T+t, normals@r.T, faces@r.T+t)
            owned |= inside;counts[link] = counts.get(link,0)+int(inside.sum())
        return owned, counts


def collision_snapshot(frozen, owned, native, known):
    """Keep environmental evidence unchanged; derive one conservative check field."""
    snap = frozen.snapshot
    owned = np.asarray(owned, bool).reshape(-1).copy()
    native = np.asarray(native).reshape(-1);known=np.asarray(known,bool).reshape(-1)
    if owned.shape != frozen.source.shape or native.shape != owned.shape or known.shape != owned.shape:
        raise ValueError('Ownership/native map shape mismatch')
    occupied = known & (native <= .02)
    owned[occupied] = False  # even an inconsistent measured obstacle is never erased
    allowed = ((frozen.source > 0) | owned) & ~occupied
    volume = allowed.reshape(snap.distance.shape)
    distances = distance_transform_edt(np.pad(volume, 1))[1:-1,1:-1,1:-1]*snap.resolution
    distances = distances.reshape(-1)-np.sqrt(3)*snap.resolution/2
    distances = np.minimum(distances, np.where(known, native, np.inf))
    distances[~allowed] = np.minimum(distances[~allowed], 0.)
    distances = distances.astype(np.float32).reshape(volume.shape);distances.flags.writeable=False
    owned.flags.writeable=False
    return replace(snap, distance=distances), owned


def prepare(frozen, geometry, q, session):
    from ur7e_nvblox.depth import known_distances
    # The calibrated real URDF excludes the simulator's added camera shell.
    urdf = Path(get_package_share_directory('ur7e_gazebo_esdf_sage_continuous'))/'config/real_calibrated.urdf'
    model = RobotOwnership(urdf, geometry)
    snap = frozen.snapshot
    z,y,x = np.indices(snap.distance.shape)
    points = snap.origin+snap.resolution*np.stack((x,y,z),-1).reshape(-1,3)
    owned, counts = model.mask(points, snap.resolution, q)
    with np.load(Path(session)/'native_esdf.npz', allow_pickle=False) as data:
        if not np.allclose(points, data['points'], atol=1e-9, rtol=0):raise ValueError('Native grid differs')
        raw=data['distance'].copy();known=known_distances(raw,float(data['unknown']))
    derived, retained = collision_snapshot(frozen, owned, raw, known)
    metadata = dict(model.provenance, environment_map_uid=frozen.uid, freeze_q=np.asarray(q).tolist(),
                    owned_cells=int(retained.sum()), per_link_interior_cells=counts,
                    measured_obstacle_conflicts=int(np.count_nonzero(owned & ~retained)),
                    environment_source_changed=False, historical_sweep_used=False)
    h=hashlib.sha256(json.dumps(metadata, sort_keys=True).encode())
    h.update(retained.tobytes());h.update(derived.distance.tobytes());metadata['collision_uid']=h.hexdigest()
    np.savez_compressed(Path(session)/'robot_ownership.npz', owned=retained, distance=derived.distance,
                        metadata=json.dumps(metadata,sort_keys=True))
    gaps=derived.query(geometry.spheres(q))-geometry.radii
    metadata.update(start_min_clearance_m=float(gaps.min()), blocked_spheres=np.flatnonzero(gaps < .03).tolist())
    (Path(session)/'robot_ownership.json').write_text(json.dumps(metadata,indent=2))
    return derived, metadata


def load_snapshot(frozen, path):
    """Read an immutable derived field bound to the exact sensing map and meshes."""
    with np.load(path, allow_pickle=False) as data:
        metadata=json.loads(str(data['metadata']));owned=data['owned'].copy();distance=data['distance'].copy()
    uid=metadata.pop('collision_uid')
    if metadata['policy']!=POLICY or metadata['environment_map_uid']!=frozen.uid:
        raise ValueError('Ownership environment/policy mismatch')
    if owned.dtype!=np.bool_ or owned.shape!=frozen.source.shape or distance.shape!=frozen.snapshot.distance.shape or not np.isfinite(distance).all():
        raise ValueError('Invalid ownership arrays')
    h=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode());h.update(owned.tobytes());h.update(distance.tobytes())
    if h.hexdigest()!=uid:raise ValueError('Ownership checksum mismatch')
    for path,expected in [(metadata['urdf'],metadata['urdf_sha256']),*[(m['path'],m['sha256']) for m in metadata['meshes']]]:
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:raise ValueError('Ownership model changed')
    if np.any(distance.reshape(-1)[(frozen.source==0)&~owned]>0):
        raise ValueError('Unknown environmental cells cleared outside body')
    distance.flags.writeable=False
    return replace(frozen.snapshot,distance=distance)
