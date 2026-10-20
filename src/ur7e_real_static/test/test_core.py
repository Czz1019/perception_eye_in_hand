import json
import time
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt
from types import SimpleNamespace
from fixtures import config,free_map,INITIAL
from ur7e_real_static.config import load_config,identity
from ur7e_real_static.model import Geometry
from ur7e_real_static.field import FrozenMap,FieldBuilder
from ur7e_real_static.trajectory import edge_clear,parameterize,validate,model_matches
from ur7e_real_static.state import execution_gate


def test_requires_explicit_real_configuration(tmp_path):
    p=tmp_path/'empty.yaml';p.write_text('{}')
    with pytest.raises(ValueError,match='robot_urdf'):load_config(p)
    c,g=config(tmp_path)
    p=tmp_path/'config.yaml'
    import yaml
    data=yaml.safe_load(p.read_text());data['execution_enabled']=True;p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='commissioned'):load_config(p)


def test_model_single_calibrated_chain_and_payload_required(tmp_path):
    c,g=config(tmp_path)
    q=INITIAL;poses=g.fk(q)
    np.testing.assert_allclose(poses['camera_depth_optical_frame'][:3,3],poses['tool0'][:3,3]+poses['tool0'][:3,:3]@[-.0325,-.075,.0357],atol=1e-10)
    np.testing.assert_allclose(g.spheres_batch([q])[0],g.spheres(q),atol=1e-10)
    assert model_matches(g,q,{g.camera:poses[g.camera]})
    wrong=poses[g.camera].copy();wrong[0,3]+=.02
    assert not model_matches(g,q,{g.camera:wrong})
    import yaml
    data=yaml.safe_load(open(c['collision_spheres']));del data['collision_spheres']['tool0']
    p=tmp_path/'missing.yaml';p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='payload'):Geometry(c['robot_urdf'],p)


def test_snapshot_roundtrip_identity_checksum_and_unknown(tmp_path):
    c,g=config(tmp_path);f=free_map(c);p=tmp_path/'map.npz';f.save(p)
    loaded=FrozenMap.load(p,identity(c));assert loaded.uid==f.uid
    assert not loaded.snapshot.distance.flags.writeable
    with pytest.raises(ValueError,match='identity'):FrozenMap.load(p,'different')
    with np.load(p,allow_pickle=False) as data:copy={k:data[k] for k in data.files}
    copy['distance']=copy['distance'].copy();copy['distance'].flat[0]=.2;np.savez(p,**copy)
    with pytest.raises(ValueError,match='checksum'):FrozenMap.load(p,identity(c))
    with pytest.raises(ValueError,match='Unknown'):FrozenMap.create([1]*8,[0]*8,[0,0,0],[2,2,2],.1,dict(stamp=1,epoch=1,sequence=1))


def test_field_empty_depth_never_clears_unknown_and_obstacle_overrides_prior(tmp_path):
    c,g=config(tmp_path);c.update(grid_size=[.2,.2,.2],grid_center=[0,0,.6],grid_resolution=.02)
    f=FieldBuilder(c);n=len(f.points);intr=SimpleNamespace(width=20,height=20,fx=20.,fy=20.,cx=10.,cy=10.)
    d,source=f.update(np.full(n,100.),np.zeros(n,bool),np.zeros((20,20)),np.eye(4),intr)
    assert np.all(d<=0) and not source.any()
    c['commissioned_free_boxes']=[dict(min=[-.2,-.2,.3],max=[.2,.2,.9])]
    f=FieldBuilder(c);raw=np.full(n,100.);known=np.zeros(n,bool);raw[n//2]=-.03;known[n//2]=True
    d,s=f.update(raw,known,np.zeros((20,20)),np.eye(4),intr)
    assert s[n//2]==0 and d[n//2]<=-.03+1e-7 and np.any(s==2)


def test_quintic_limits_and_collision_checks(tmp_path):
    c,g=config(tmp_path);f=free_map(c);goal=INITIAL+[.08,0,0,0,0,0]
    assert edge_clear(g,f.snapshot,INITIAL,goal,c['collision_margin'],c['self_margin'])
    xyz=g.fk(goal)[g.tool][:3,3]
    t=parameterize([INITIAL,goal],.05,.1,f.uid,identity(c),xyz)
    validate(g,f,t,c)
    times=np.linspace(0,t.times[-1],2001);positions=np.array([t.sample(x) for x in times]);dt=times[1]-times[0]
    velocities=np.diff(positions,axis=0)/dt;acc=np.diff(velocities,axis=0)/dt
    assert np.max(abs(velocities))<=.050001 and np.max(abs(acc))<=.10001
    blocked=FrozenMap.create(np.zeros(f.source.size),np.zeros(f.source.size,'u1'),f.snapshot.origin,f.snapshot.dimensions,f.snapshot.resolution,f.metadata)
    assert not edge_clear(g,blocked.snapshot,INITIAL,goal)
    with pytest.raises(ValueError,match='version|different'):validate(g,blocked,t,c)


def test_frozen_map_age_is_not_control_freshness(tmp_path):
    c,g=config(tmp_path);f=free_map(c);q=INITIAL.copy();xyz=g.fk(q)[g.tool][:3,3]
    t=parameterize([q,q],.05,.1,f.uid,identity(c),xyz)
    state=(q,np.zeros(6),1000.,time.monotonic())
    with pytest.raises(ValueError,match='preview'):execution_gate(c,f,t,state,1000.,state[3],True,True,True)
    c.update(execution_enabled=True,commissioned=True)
    assert execution_gate(c,f,t,state,1000.,state[3],True,True,True)  # frozen stamp is 1 s, current time 1000 s
    with pytest.raises(ValueError,match='Stale'):execution_gate(c,f,t,state,1001.,state[3]+1,True,True,True)
    with pytest.raises(ValueError,match='confirmation'):execution_gate(c,f,t,state,1000.,state[3],False,True,True)
    with pytest.raises(ValueError,match='TF'):execution_gate(c,f,t,state,1000.,state[3],True,False,True)
    moved=(q+.03,np.zeros(6),1000.,state[3])
    with pytest.raises(ValueError,match='moved'):execution_gate(c,f,t,moved,1000.,state[3],True,True,True)


def test_unknown_corridor_blocks_even_with_free_goal(tmp_path):
    c,g=config(tmp_path);f=free_map(c);goal=INITIAL+[.2,0,0,0,0,0]
    centers=g.spheres(INITIAL+.5*(goal-INITIAL));point=centers[-1]
    class Obstacle:
        def query(self,points):return np.linalg.norm(points-point,axis=-1)-.02
    assert not edge_clear(g,Obstacle(),INITIAL,goal)
