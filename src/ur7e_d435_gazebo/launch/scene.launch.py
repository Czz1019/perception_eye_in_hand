"""UR7e + outer-side D435 + tall walls, without external workspace paths."""
import os
import importlib.util
import socket
from pathlib import Path
from urllib.parse import urlparse
from xml.dom import minidom

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def _remove_comments(node):
    # gazebo_ros2_control also parses the description; avoid YAML-like XML comments.
    for child in list(node.childNodes):
        if child.nodeType == minidom.Node.COMMENT_NODE:
            node.removeChild(child)
        else:
            _remove_comments(child)


def _check_gazebo_master_available(uri):
    endpoint = urlparse(uri)
    host, port = endpoint.hostname or "127.0.0.1", endpoint.port or 11345
    try:
        connection = socket.create_connection((host, port), timeout=1.0)
    except ConnectionRefusedError:
        return
    except OSError as error:
        raise RuntimeError(f"Cannot check Gazebo master {uri}: {error}") from error
    connection.close()
    raise RuntimeError(
        f"Gazebo master {uri} is already in use. Close the previous Gazebo server before starting this scene. "
        f"Check its PID with: ss -ltnp 'sport = :{port}'. "
        "Closing only the Gazebo window may leave gzserver running.")


def _setup(context):
    _check_gazebo_master_available(os.environ.get("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"))
    share = Path(get_package_share_directory("ur7e_d435_gazebo"))
    lib = Path(get_package_prefix("ur7e_d435_gazebo")) / "lib"
    if not (lib / "librealsense_gazebo_plugin.so").is_file():
        raise RuntimeError("Build ur7e_d435_gazebo first: RealSense plugin is missing")
    calibration_file = Path(LaunchConfiguration("calibration_file").perform(context)).expanduser().resolve()
    calibration = yaml.safe_load(calibration_file.read_text())
    expected_frames = {
        "calibration_type": "eye_in_hand",
        "robot_effector_frame": "tool0",
        "tracking_base_frame": "camera_color_optical_frame",
    }
    for key, expected in expected_frames.items():
        actual = calibration["parameters"][key]
        if actual != expected:
            raise ValueError(f"{calibration_file}: {key} must be {expected!r}, got {actual!r}; convert the calibration transform before using this mount")
    document = xacro.process_file(str(share / "urdf/ur7e_d435.urdf.xacro"),
                                  mappings={"eih_calibration": str(calibration_file)})
    _remove_comments(document)
    xml = document.toxml()
    spec = importlib.util.spec_from_file_location("mount_geometry", lib / "ur7e_d435_gazebo/mount_geometry.py")
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    initial = yaml.safe_load((share / "config/initial_positions.yaml").read_text())
    report = geometry.check_mount_geometry(xml, initial, lib / "ur7e_d435_gazebo/check_mount_meshes")
    colliding = sorted({pair["arm"] for pair in report["pairs"] if pair["collision"]})
    if colliding:
        diagnostic = ("WARNING: measured camera mount intersects " + ", ".join(colliding) +
                      ". Original calibration is preserved. Successful startup does not validate physical mounting. "
                      "Check marker scale/intrinsics, robot TF and calibration samples; do not move only the camera mesh.")
    else:
        diagnostic = f"Camera/arm geometry clearance at initial pose: {report['minimum_distance_m']*1000:.2f} mm."
    description = ParameterValue(xml, value_type=str)
    return [
        LogInfo(msg=f"Camera transform file: {calibration_file}"),
        LogInfo(msg=diagnostic),
        # This scene uses local assets; do not fetch the online Insert catalog.
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(filter(None, [str(lib), os.environ.get("GAZEBO_PLUGIN_PATH", "")]))),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.pathsep.join(filter(None, ["/usr/share/gazebo-11", os.environ.get("GAZEBO_RESOURCE_PATH", "")]))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "publish_frequency": 100.0,
                          "robot_description": description}], output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("gazebo_ros")) / "launch/gazebo.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gazebo_gui"),
                              "params_file": str(share / "config/gazebo_clock.yaml"),
                              "server_required": "true", "verbose": "true"}.items()),
        Node(package="gazebo_ros", executable="spawn_entity.py", name="spawn_ur",
             arguments=["-entity", "ur", "-topic", "robot_description", "-timeout", "60"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "60"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["forward_position_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"], output="screen"),
        Node(package="ur7e_d435_gazebo", executable="spawn_scene.py", output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Open the Gazebo GUI"),
        DeclareLaunchArgument("calibration_file",
                              default_value=str(Path(get_package_share_directory("ur7e_d435_gazebo")) /
                                                "config/ur7e_d435_handeye_20260907.yaml"),
                              description="Eye-in-hand YAML; default preserves the measured transform"),
        OpaqueFunction(function=_setup),
    ])
