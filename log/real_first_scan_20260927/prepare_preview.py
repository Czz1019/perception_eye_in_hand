from pathlib import Path
import json,yaml,numpy as np
from ur7e_real_static.config import load_config
from ur7e_real_static.model import Geometry
root=Path('/home/wqj/realtime_nvblox_ws');setup=Path('/home/wqj/ur7e_real_setup')
mount=np.array(json.loads((setup/'camera_mount.json').read_text())['tool0_to_camera_link'])
spheres=yaml.safe_load((root/'src/ur7e_real_static/config/collision_spheres.template.yaml').read_text())
# D435i uses the D435 body description, _d435.urdf.xacro, 90 x 25 x 25.05 mm.
# Preview-only geometric approximation, not a measured tool/mount envelope.
center=mount[:3,:3]@np.array([0.,-.0175,0.])+mount[:3,3]
radius=float(np.linalg.norm([.02505,.09,.025])/2+.005)
spheres['collision_spheres']['tool0']=[dict(center=center.tolist(),radius=radius)]
spheres['preview_notes']='Unverified arm template; camera body sphere from installed D435i description. Gripper/mount omitted at user request. Preview only.'
(setup/'collision_spheres_preview.yaml').write_text(yaml.safe_dump(spheres,sort_keys=False))
c=yaml.safe_load((root/'src/ur7e_real_static/config/real.yaml').read_text())
c.update(robot_urdf=str(setup/'calibrated_with_camera.urdf'),collision_spheres=str(setup/'collision_spheres_preview.yaml'),output_directory=str(root/'log/real_first_scan_20260927/output'),execution_enabled=False,commissioned=False,commissioned_free_boxes=[])
(setup/'preview.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
checked=load_config(str(setup/'preview.yaml'));g=Geometry(checked['robot_urdf'],checked['collision_spheres'])
print('Preview configuration valid; execution disabled; no commissioned free volumes.');print('camera body sphere:',spheres['collision_spheres']['tool0']);print('links',g.links)
