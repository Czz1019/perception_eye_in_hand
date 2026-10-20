"""Resolve installed resources without reading STORM's examples/content/weights."""
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

SHARE = Path(get_package_share_directory("ur7e_sage_planner"))
RUN_DIR = Path(os.environ.get("SAGE_RUN_DIR", str(Path.home() / ".ros/ur7e_sage_planner"))).expanduser().absolute()
RUN_DIR.mkdir(parents=True, exist_ok=True)


def _resolve(value):
    if isinstance(value, dict):
        return {key: _resolve(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_resolve(item) for item in value]
    if isinstance(value, str) and value.startswith("package://"):
        package, relative = value[len("package://"):].split("/", 1)
        path = Path(get_package_share_directory(package)) / relative
        if not path.is_file():
            raise FileNotFoundError(path)
        return str(path)
    return value


def _resolved_yaml(source, name):
    params = _resolve(yaml.safe_load(Path(source).read_text()))
    # Unique files also make concurrent CLI diagnostics safe.
    with tempfile.NamedTemporaryFile(mode="w", dir=RUN_DIR, prefix=name + "_", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return stream.name


TASK_FILE = _resolved_yaml(os.environ.get("SAGE_TASK_FILE", str(SHARE / "config/ur7e_reacher_gazebo_tall_sage_clean.yml")), "task")
ROBOT_FILE = _resolved_yaml(SHARE / "config/ur7e_robot_gazebo.yml", "robot")
WORLD_FILE = str(SHARE / "config/collision_world_gazebo_tall.yml")
