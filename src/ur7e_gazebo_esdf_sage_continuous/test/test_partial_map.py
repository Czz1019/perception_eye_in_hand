import numpy as np
from ur7e_real_static.field import FrozenMap
from ur7e_gazebo_esdf_sage_continuous.partial_map import fields


def test_partial_policy_preserves_unknown_and_obstacles_and_bounds_exploration():
    shape=(21,21,21);source=np.zeros(shape,np.uint8);source[:,:,8:13]=1
    raw=np.full(shape,100.);known=np.zeros(shape,bool);known[10,10,10]=True;raw[10,10,10]=-.02
    owned=np.zeros(shape,bool);owned[10,10,2]=True
    frozen=FrozenMap.create(np.zeros(np.prod(shape)),source.ravel(),[-.2]*3,[21]*3,.02,dict(stamp=0,epoch=1,sequence=1))
    original=frozen.source.copy()
    distance,obstacle,uncertainty,admitted=fields(frozen,owned.ravel(),raw.ravel(),known.ravel(),band=.04)
    admitted=admitted.reshape(shape)
    assert np.array_equal(frozen.source,original)
    assert admitted[10,10,7] and source[10,10,7]==0 and uncertainty[10,10,7]>0
    assert not admitted[10,10,5]  # Too far from environmental evidence.
    assert admitted[10,10,2] and not admitted[10,10,3]  # Body is not dilated.
    assert not admitted[10,10,10] and distance[10,10,10]<0 and obstacle[10,10,10]<0
    assert np.all(distance<=obstacle)


def test_partial_policy_refuses_empty_evidence():
    import pytest
    f=FrozenMap.create(np.zeros(125),np.zeros(125,np.uint8),[0]*3,[5]*3,.02,dict(stamp=0,epoch=1,sequence=1))
    with pytest.raises(ValueError,match='evidence'):fields(f,np.zeros(125,bool),np.zeros(125),np.ones(125,bool))


def test_policy_selection_uses_local_scan_config(tmp_path):
    from pathlib import Path
    import shutil,yaml,pytest
    from ur7e_gazebo_esdf_sage_continuous.config import load_mapping
    directory=tmp_path/'config';shutil.copytree(Path(__file__).resolve().parents[1]/'config',directory)
    assert load_mapping(directory/'mapping.yaml')['planning_policy']=='strict'
    scan=yaml.safe_load((directory/'scan.yaml').read_text());scan['planning_policy']='bounded_unknown'
    (directory/'scan.yaml').write_text(yaml.safe_dump(scan))
    assert load_mapping(directory/'mapping.yaml')['planning_policy']=='bounded_unknown'
    scan['planning_policy']='anything';(directory/'scan.yaml').write_text(yaml.safe_dump(scan))
    with pytest.raises(ValueError,match='Unknown planning policy'):load_mapping(directory/'mapping.yaml')
