"""Perception-only column inference, boundaries, counterevidence and isolation."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.ndimage import distance_transform_edt
from ur7e_gazebo_esdf_sage.config import load_mapping, identity
from ur7e_gazebo_esdf_sage.field import FieldBuilder

ROOT=Path(__file__).resolve().parents[1]

def setup():
    c=load_mapping(ROOT/'config/mapping.yaml');c.update(free_space_backend='cpu',grid_size=[1.,1.,1.4],grid_center=[0.,0.,.6],grid_resolution=.02)
    # Planning must not even open scan geometry or robot models to add free cells.
    c['scan_config']='/nonexistent/scan.yaml';c['robot_urdf']='/nonexistent/robot.urdf'
    f=FieldBuilder(c);intr=SimpleNamespace(width=240,height=240,fx=220.,fy=220.,cx=120.,cy=120.)
    return f,intr

def frame(intr,tx=0.,height=0.,hole=False):
    pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[tx,0.,1.]
    depth=np.full((intr.height,intr.width),1.-height)
    yy,xx=np.indices(depth.shape);x=(xx-intr.cx)*depth/intr.fx+tx;y=-(yy-intr.cy)*depth/intr.fy
    if hole:depth[(abs(x-.16)<.10)&(abs(y)<.10)]=0.
    return depth,pose

def update(f,intr,tx=0.,height=0.,hole=False,occupied=None):
    depth,pose=frame(intr,tx,height,hole);raw=np.full(len(f.points),100.);known=np.zeros(len(raw),bool)
    if occupied is not None:known[occupied]=True;raw[occupied]=-.02
    return f.update(raw,known,depth,pose,intr)

def index(f,xyz):return int(np.argmin(np.linalg.norm(f.points-np.array(xyz),axis=1)))

def test_two_distinct_views_required_and_ray_vs_inference():
    f,intr=setup();d,s=update(f,intr)
    assert not np.any(s==2)
    for _ in range(3):d,s=update(f,intr)
    assert not np.any(s==2),'Repeated frames are not distinct viewpoints'
    d,s=update(f,intr,tx=.08)
    assert s[index(f,[0,0,1.2])]==2,'Above-camera freedom must come from tabletop evidence'
    assert s[index(f,[0,0,.5])]==1
    assert s[index(f,[-.48,.48,1.2])]==0,'Uncovered footprint cannot be extrapolated'
    assert s[index(f,[0,0,.02])]==0,'Do not free below measured top plus margin'
    expected=distance_transform_edt(np.pad((s>0).reshape(tuple(f.dims[::-1])),1))[1:-1,1:-1,1:-1].reshape(-1)*f.c['grid_resolution']-np.sqrt(3)*f.c['grid_resolution']/2
    np.testing.assert_allclose(d,expected,atol=1e-7)

def test_invalid_depth_holes_not_filled_and_obstacles_override_columns():
    f,intr=setup();update(f,intr,hole=True);d,s=update(f,intr,tx=.08,hole=True)
    assert s[index(f,[.16,0,1.2])]==0
    occupied=index(f,[-.12,0,1.1]);assert s[occupied]==2
    d,s=update(f,intr,tx=.08,hole=True,occupied=occupied)
    assert s[occupied]==0 and d[occupied]<0

def test_higher_top_retracts_previous_inference():
    f,intr=setup();update(f,intr);update(f,intr,tx=.08)
    f.rays[:]=False  # isolate structural inference from independent historical rays
    d,s=update(f,intr,tx=.08,height=.30)
    assert f.upper_height[25,25]>=.339
    assert s[index(f,[0,0,.32])]!=2

def test_side_faces_cannot_support_top_columns():
    f,intr=setup()
    # Tilt camera 25 degrees; a vertical plane remains in a downward-facing view.
    from scipy.spatial.transform import Rotation
    for tx in [0.,.08]:
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('y',25,degrees=True).as_matrix()@np.diag([1.,-1.,-1.]);pose[:3,3]=[tx,0.,1.]
        yy,xx=np.indices((intr.height,intr.width));rays=np.stack([(xx-intr.cx)/intr.fx,(yy-intr.cy)/intr.fy,np.ones_like(xx)],-1)@pose[:3,:3].T
        depth=np.divide(.3-tx,rays[...,0],out=np.zeros(rays.shape[:2]),where=rays[...,0]>0)
        depth[(depth<.2)|(depth>3.)]=0.
        f.update(np.full(len(f.points),100.),np.zeros(len(f.points),bool),depth,pose,intr)
    assert not np.any(f.reliable) and not np.any(f.source==2)

def test_old_prior_identity_is_rejected():
    import hashlib,json,yaml
    from ur7e_real_static.config import identity as old_base_identity
    from ur7e_real_static.field import FrozenMap
    import pytest
    c=load_mapping(ROOT/'config/mapping.yaml');scan=yaml.safe_load(Path(c['scan_config']).read_text())
    old=hashlib.sha256((old_base_identity(c)+json.dumps({k:scan[k] for k in ['views','prior_clearance','depth_self_mask']},sort_keys=True)).encode()).hexdigest()
    assert old!=identity(c)
    # The planning contract is independent of the scan prior's trajectory/radius.
    from tempfile import TemporaryDirectory
    with TemporaryDirectory() as td:
        path=Path(td)/'scan.yaml';scan['views']=[[0]*6,[1]*6];scan['prior_clearance']=999.;path.write_text(yaml.safe_dump(scan))
        changed=dict(c,scan_config=str(path));assert identity(changed)==identity(c)
        frozen=FrozenMap.create(np.zeros(8),np.zeros(8),[0,0,0],[2,2,2],.02,dict(stamp=0,epoch=1,sequence=1,identity=old))
        frozen.save(Path(td)/'old.npz')
        with pytest.raises(ValueError,match='identity mismatch'):FrozenMap.load(Path(td)/'old.npz',identity(c))


def test_robot_hit_retains_only_measured_free_ray_before_return():
    f,intr=setup();raw=np.full(len(f.points),100.);known=np.zeros(len(raw),bool)
    measured,pose=frame(intr,height=.4)
    # Every endpoint was self-filtered: it must not support a tabletop column.
    filtered=np.zeros_like(measured)
    d,s=f.update(raw,known,filtered,pose,intr,ray_depth=measured)
    assert s[index(f,[0,0,.7])]==1
    assert s[index(f,[0,0,.4])]==0
    assert s[index(f,[0,0,.2])]==0
    assert s[index(f,[0,0,1.2])]==0
    assert not np.any(s==2)
    occupied=index(f,[0,0,.7]);known[occupied]=True;raw[occupied]=-.02
    d,s=f.update(raw,known,filtered,pose,intr,ray_depth=measured)
    assert s[occupied]==0 and d[occupied]<=0
