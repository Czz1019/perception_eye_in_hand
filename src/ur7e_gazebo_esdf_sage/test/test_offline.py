from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ur7e_gazebo_esdf_sage.scene import build_robot
from ur7e_gazebo_esdf_sage.config import load_mapping
from ur7e_gazebo_esdf_sage.field import ScanPriorField,scan_samples
from ur7e_real_static.model import Geometry
from ur7e_real_static.camera_collision import fixed_transform
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.trajectory import edge_clear
from types import SimpleNamespace
from design_scan import Fixture

ROOT=Path(__file__).resolve().parents[1]

def test_real_calibration_and_render_pose():
    real=ET.parse(ROOT/'config/real_calibrated.urdf').getroot()
    sim=ET.fromstring(build_robot(ROOT))
    for child in ['camera_link','camera_color_optical_frame','camera_depth_optical_frame']:
        np.testing.assert_allclose(fixed_transform(real,'tool0',child),fixed_transform(sim,'tool0',child),atol=1e-12)
    optical=np.eye(4);optical[:3,:3]=Rotation.from_euler('xyz',[-np.pi/2,0,-np.pi/2]).as_matrix()
    for sensor,frame in [('cameradepth','camera_depth_optical_frame'),('cameracolor','camera_color_optical_frame')]:
        xyzrpy=np.fromstring(sim.find(f"gazebo[@reference='camera_link']/sensor[@name='{sensor}']/pose").text,sep=' ')
        rendered=np.eye(4);rendered[:3,3]=xyzrpy[:3];rendered[:3,:3]=Rotation.from_euler('xyz',xyzrpy[3:]).as_matrix()
        np.testing.assert_allclose(rendered@optical,fixed_transform(real,'camera_link',frame),atol=2e-5)
    assert 'rtde' not in ET.tostring(sim,encoding='unicode').lower()

def test_scan_corridor_and_unknown():
    c=load_mapping(ROOT/'config/mapping.yaml');c['free_space_backend']='cpu'
    config=yaml.safe_load((ROOT/'config/scan.yaml').read_text());views=np.array(config['views'])
    g=Geometry(c['robot_urdf'],c['collision_spheres']);field=ScanPriorField(c)
    qs=scan_samples(views,.002);centers=g.spheres_batch(qs)
    padding=g.motion_bound*np.max(np.sum(abs(np.diff(qs,axis=0)),axis=1))/2
    assert (Fixture().query(centers)-g.radii).min()-padding>config['prior_clearance']
    assert g.self_clearance_batch(centers).min()-2*padding>c['self_margin']
    d,s=field.update(np.full(len(field.points),100.),np.zeros(len(field.points),bool),np.zeros((4,4)),np.eye(4),SimpleNamespace(width=4,height=4,fx=1,fy=1,cx=2,cy=2))
    frozen=FrozenMap.create(d,s,field.origin,field.dims,c['grid_resolution'],dict(stamp=0.,epoch=0,sequence=0))
    assert all(edge_clear(g,frozen.snapshot,a,b,.03,.015) for a,b in zip(views,views[1:]))
    assert np.all(d[s==0]<=0)
    assert frozen.snapshot.query(np.array([[5.,5.,5.]]))[0]<=0
    # A measured occupied voxel must override the fixed initial free-space prior.
    index=int(np.flatnonzero(s==2)[0]);raw=np.full(len(d),100.);known=np.zeros(len(d),bool);known[index]=True;raw[index]=-.01
    d,s=field.update(raw,known,np.zeros((4,4)),np.eye(4),SimpleNamespace(width=4,height=4,fx=1,fy=1,cx=2,cy=2))
    assert d[index]<0 and s[index]==0

def servo_fixture(elapsed=0.,age=0.):
    import time,threading
    from types import SimpleNamespace
    from ur7e_gazebo_esdf_sage.node import ClosedLoop
    sent=[]
    fake=SimpleNamespace(online=None,frozen=None,scan={'wall_feedback_timeout':2.},lock=threading.RLock(),
        qcmd=np.zeros(6),qnext=np.ones(6),segment=(np.zeros(6),np.ones(6),elapsed,1.,False),
        mapper=SimpleNamespace(lock=threading.RLock(),states=[]),fault_inject=False,stopped=False,phase='REACHING',last_tick=0.,last_log=10.,iterations=0,
        state=lambda:(np.zeros(6),np.zeros(6),10.,time.monotonic()-age),now=lambda:10.,
        command=SimpleNamespace(publish=lambda msg:sent.append(np.array(msg.data))))
    def fail(reason):fake.stopped=True;fake.segment=None
    fake.fail=fail
    return fake,sent,ClosedLoop

def test_servo_clock_jump_is_bounded():
    fake,sent,cls=servo_fixture()
    cls.tick(fake)
    np.testing.assert_allclose(sent[-1],.02)
    assert not fake.stopped

def test_tracking_failure_never_publishes_rejected_command():
    fake,sent,cls=servo_fixture(elapsed=.5)
    cls.tick(fake)
    assert fake.stopped
    np.testing.assert_array_equal(sent[-1],np.zeros(6))

def test_paused_simulation_holds_segment_until_fresh_feedback():
    fake,sent,cls=servo_fixture(age=3.)
    cls.tick(fake)
    assert not fake.stopped and fake.segment[2]==0.
    np.testing.assert_array_equal(sent[-1],np.zeros(6))

def test_fresh_joint_stamp_ahead_of_clock_is_not_expired():
    import threading,time
    from ur7e_gazebo_esdf_sage.node import ClosedLoop
    state=(np.zeros(6),np.zeros(6),10.7,time.monotonic())
    fake=SimpleNamespace(mapper=SimpleNamespace(lock=threading.RLock(),states=[state]),
        fault_inject=False,now=lambda:10.,c={'joint_timeout':.6})
    assert ClosedLoop.state(fake) is state
    fake.now=lambda:12.
    try:ClosedLoop.state(fake)
    except RuntimeError as error:assert 'STALE_JOINT_STATE' in str(error)
    else:raise AssertionError('Expired simulation feedback accepted')


def test_transient_stale_feedback_holds_then_latches():
    import time
    fake,sent,cls=servo_fixture()
    fake.mapper.states=[(np.zeros(6),np.zeros(6),9.,time.monotonic())]
    def stale():raise RuntimeError('STALE_JOINT_STATE')
    fake.state=stale
    cls.tick(fake)
    assert not fake.stopped and fake.segment[2]==0.
    np.testing.assert_array_equal(sent[-1],np.zeros(6))
    fake.mapper.states[-1]=(*fake.mapper.states[-1][:3],time.monotonic()-3.)
    cls.tick(fake)
    assert fake.stopped
    np.testing.assert_array_equal(sent[-1],np.zeros(6))
