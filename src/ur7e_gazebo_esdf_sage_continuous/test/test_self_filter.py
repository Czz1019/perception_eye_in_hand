"""The batched filter must preserve every pixel of the reference mask."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
from ur7e_gazebo_esdf_sage_continuous.self_filter import robot_depth_mask
from ur7e_perception_planning_core.self_filter import robot_depth_mask as reference
from ur7e_real_static.model import Geometry


def test_batched_mask_matches_reference_for_camera_and_robot_views():
    root=Path(__file__).resolve().parents[1]
    g=Geometry(root/'config/real_calibrated.urdf',root/'config/collision_spheres.yaml')
    views=yaml.safe_load((root/'config/scan_coverage.yaml').read_text())['views']
    intr=SimpleNamespace(fx=115.,fy=115.,cx=80.,cy=60.)
    rng=np.random.default_rng(423)
    depth=rng.uniform(.05,3.,(120,160));depth[::7,::9]=0.
    for q in views:
        pose=g.fk(q)[g.camera]
        centers=np.r_[g.spheres(q),[[0.,0.,.037706]]]
        radii=np.r_[g.radii,.085];links=np.r_[g.link_ids,7]
        np.testing.assert_array_equal(robot_depth_mask(depth,pose,intr,centers,radii,links),
                                      reference(depth,pose,intr,centers,radii,links))
    # Optical centre inside a sphere, blind-zone and invalid depth cases.
    centers=np.array([[0.,0.,0.],[.04,0.,.15],[0.,0.,.7]])
    radii=np.array([.05,.08,.1]);links=np.arange(3)
    np.testing.assert_array_equal(robot_depth_mask(depth,np.eye(4),intr,centers,radii,links),
                                  reference(depth,np.eye(4),intr,centers,radii,links))
