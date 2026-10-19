import numpy as np
from sensor_msgs.msg import JointState, CameraInfo, Image
from ur7e_perception_planning_core.geometry import JOINTS
from ur7e_real_reconstruction.inputs import JointHistory, ColorRectifier


def sample(t,q=0.,v=0.):
    m=JointState();m.header.stamp.sec=int(t);m.header.stamp.nanosec=int(round((t-int(t))*1e9))
    m.name=list(JOINTS);m.position=[q]*6;m.velocity=[v]*6;return m


def test_stationarity_requires_bracketing_and_uninterrupted_history():
    h=JointHistory();c=dict(settle_seconds=.5,settle_speed=.015)
    for t in np.arange(1.,1.8,.02):h.add(sample(float(t)))
    assert h.stationary_at(1.7,c) is not None
    assert h.stationary_at(1.1,c) is None
    assert h.stationary_at(1.9,c) is None
    h.add(sample(1.81,v=.2))
    assert h.stationary_at(1.8,c) is None
    g=JointHistory()
    for t in [1.,1.02,1.04,1.6,1.62,1.64]:g.add(sample(t))
    assert g.stationary_at(1.63,c) is None


def test_color_rectification_handles_raw_distortion_and_bgr_padding():
    info=CameraInfo();info.width=80;info.height=60
    info.k=[60.,0.,40.,0.,60.,30.,0.,0.,1.];info.p=[60.,0.,40.,0.,0.,60.,30.,0.,0.,0.,1.,0.]
    info.r=np.eye(3).reshape(-1).tolist();info.d=[.1,0.,0.,0.,0.];info.distortion_model='plumb_bob'
    rect=ColorRectifier(info,2)
    data=np.zeros((60,244),np.uint8);data[:,:240].reshape(60,80,3)[:]=[20,40,200]
    image=Image();image.width=80;image.height=60;image.step=244;image.encoding='bgr8';image.data=data.tobytes()
    rgb=rect.decode(image)
    assert rgb.shape==(30,40,3)
    np.testing.assert_array_equal(rgb[15,20],[200,40,20])
    assert rect.maps is not None


def test_joint_stream_without_velocity_can_seed_and_detect_motion():
    h=JointHistory();c=dict(settle_seconds=.5,settle_speed=.015)
    for t in np.arange(1.,1.8,.02):
        m=sample(float(t));m.velocity=[];h.add(m)
    assert h.stationary_at(1.7,c) is not None
    m=sample(1.81,q=.1);m.velocity=[];h.add(m)
    assert h.stationary_at(1.8,c) is None
