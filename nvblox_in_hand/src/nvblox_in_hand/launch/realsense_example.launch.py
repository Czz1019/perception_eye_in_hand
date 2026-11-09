"""D435 + robot TF example; excludes VSLAM/recognition/other sensor workflows."""
from pathlib import Path
import importlib.util
_spec = importlib.util.spec_from_file_location('in_hand_bringup', Path(__file__).with_name('bringup.launch.py'))
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
generate_launch_description = _module.generate_launch_description
