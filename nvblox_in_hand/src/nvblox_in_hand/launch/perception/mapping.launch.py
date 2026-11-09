"""Compatibility entry; implementation lives in the example-shaped nvblox launch."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location('in_hand_nvblox_launch',
    Path(__file__).with_name('nvblox.launch.py'))
_impl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_impl)
setup = _impl.setup
generate_launch_description = _impl.generate_launch_description
