"""Runs the actual installed whole_control SAGE/CUDA backend, not a fake optimizer."""
import numpy as np
from fixtures import config,free_map,INITIAL
from ur7e_real_static.planning import plan
from ur7e_real_static.trajectory import validate


def test_actual_sage_plan_uses_calibrated_model_and_static_esdf(tmp_path):
    c,g=config(tmp_path);f=free_map(c)
    goal=g.fk(INITIAL+np.array([.07,0,0,0,0,0]))[g.tool][:3,3]
    result=plan(g,f,INITIAL,goal,c,tmp_path/'sage')
    assert result.sage_iterations>0
    assert np.linalg.norm(g.fk(result.positions[-1])[g.tool][:3,3]-goal)<.012
    validate(g,f,result,c)


def test_shuffled_urdf_order_keeps_actual_sage_fk_consistent(tmp_path):
    import xml.etree.ElementTree as ET
    from pathlib import Path
    from ur7e_real_static.model import Geometry
    from ur7e_real_static.planning import build_task
    c,g=config(tmp_path)
    root=ET.parse(c['robot_urdf']).getroot()
    root[:]=list(reversed(list(root)))
    Path(c['robot_urdf']).write_text(ET.tostring(root,encoding='unicode'))
    calibrated=Geometry(c['robot_urdf'],c['collision_spheres'])
    task=build_task(calibrated,c,tmp_path/'shuffled')  # includes actual CPU/CUDA FK comparison
    task.close()


def test_sage_detours_around_synthetic_obstacle(tmp_path):
    from ur7e_real_static.field import FrozenMap
    from ur7e_real_static.config import identity
    from ur7e_perception_planning_core.grid import grid_points
    from ur7e_real_static.trajectory import edge_clear
    c,g=config(tmp_path)
    end=INITIAL+np.array([1.2,0,0,0,0,0])
    center=g.spheres((INITIAL+end)/2)[16]
    origin,dims,points=grid_points(c['grid_center'],c['grid_size'],c['grid_resolution'])
    raw=np.linalg.norm(points-center,axis=1)-.025
    source=np.where(raw>0,1,0).astype('u1')
    # Synthetic analytic signed-distance map, explicitly separate from sensor validation.
    f=FrozenMap.create(raw,source,origin,dims,c['grid_resolution'],dict(identity=identity(c),stamp=1.,epoch=1,sequence=1,
                      views=[(INITIAL+[0,-.35,0,0,0,0]).tolist(),(end+[0,-.35,0,0,0,0]).tolist()]))
    assert edge_clear(g,f.snapshot,INITIAL,INITIAL,c['collision_margin'],c['self_margin'])
    assert edge_clear(g,f.snapshot,end,end,c['collision_margin'],c['self_margin'])
    assert not edge_clear(g,f.snapshot,INITIAL,end,c['collision_margin'],c['self_margin'])
    goal=g.fk(end)[g.tool][:3,3]
    result=plan(g,f,INITIAL,goal,c,tmp_path/'detour')
    validate(g,f,result,c)
    assert result.sage_iterations>0
