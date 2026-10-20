from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from ur7e_gazebo_esdf_sage.ownership import contained_cells, collision_snapshot
from ur7e_real_static.field import FrozenMap


def test_entire_voxel_is_inside_rotated_solid_without_expansion():
    normals=np.r_[np.eye(3),-np.eye(3)]
    vertices=np.array(np.meshgrid(*[[-.1,.1]]*3)).reshape(3,-1).T
    rotation=Rotation.from_euler('z',.45).as_matrix()
    points=np.array(np.meshgrid(*[np.linspace(-.2,.2,21)]*3)).reshape(3,-1).T
    mask=contained_cells(points,.02,vertices@rotation.T,normals@rotation.T,.1*normals@rotation.T)
    corners=np.array(np.meshgrid(*[[-.01,.01]]*3)).reshape(3,-1).T
    assert mask.any()
    # Independent box-coordinate containment proof for every voxel corner.
    local=(points[:,None]+corners)@rotation
    expected=np.all(abs(local)<.1-1e-8,axis=(1,2))
    assert np.array_equal(mask,expected)
    assert not mask[np.linalg.norm(points,axis=1)>.18].any()


def test_ownership_does_not_clear_outside_unknown_or_measured_obstacles():
    dims=np.array([9,9,9]);count=int(np.prod(dims))
    source=np.zeros(count,np.uint8);d=np.zeros(count)
    f=FrozenMap.create(d,source,[-.08]*3,dims,.02,dict(stamp=0.,epoch=1,sequence=1))
    owned=np.zeros((9,9,9),bool);owned[2:7,2:7,2:7]=True
    raw=np.full(count,100.);known=np.zeros(count,bool)
    obstacle=np.ravel_multi_index((4,4,4),(9,9,9));known[obstacle]=True;raw[obstacle]=-.01
    snap,retained=collision_snapshot(f,owned,raw,known)
    assert not retained[obstacle]
    assert np.all(snap.distance.reshape(-1)[~owned.reshape(-1)]<=0)
    assert snap.distance.reshape(-1)[obstacle]<0
    assert np.count_nonzero(f.source)==0 and np.all(f.snapshot.distance==0)
    assert not snap.distance.flags.writeable and not retained.flags.writeable
    # The second pose gets its own snapshot, never the union with the first pose.
    moved=np.roll(owned,3,axis=0)
    _,second=collision_snapshot(f,moved,raw,known)
    assert not second[owned.reshape(-1)&~moved.reshape(-1)].any()


def test_saved_snapshot_rejects_tampering_and_wrong_environment(tmp_path):
    import hashlib,json,pytest
    from ur7e_gazebo_esdf_sage.ownership import load_snapshot,POLICY
    f=FrozenMap.create(np.zeros(27),np.zeros(27,np.uint8),[0,0,0],[3,3,3],.02,dict(stamp=0.,epoch=1,sequence=1))
    model=tmp_path/'model.urdf';model.write_text('<robot/>')
    metadata=dict(policy=POLICY,environment_map_uid=f.uid,urdf=str(model),urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),meshes=[])
    owned=np.zeros(27,bool);distance=np.zeros((3,3,3),np.float32)
    h=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode());h.update(owned.tobytes());h.update(distance.tobytes());metadata['collision_uid']=h.hexdigest()
    path=tmp_path/'owned.npz'
    def save():np.savez(path,owned=owned,distance=distance,metadata=json.dumps(metadata))
    save();assert not load_snapshot(f,path).distance.flags.writeable
    distance[1,1,1]=1.;save()
    with pytest.raises(ValueError,match='checksum'):load_snapshot(f,path)
    distance[:]=0.;metadata['environment_map_uid']='different';save()
    with pytest.raises(ValueError,match='environment'):load_snapshot(f,path)
    metadata['environment_map_uid']=f.uid;save();model.write_text('<robot changed="yes"/>')
    with pytest.raises(ValueError,match='model changed'):load_snapshot(f,path)
