"""Same scene and initial pose, reduced sensor rendering load for online planning."""
import os
from pathlib import Path
from xml.dom import minidom

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _remove_comments(node):
    # gazebo_ros2_control also parses the description; avoid YAML-like XML comments.
    for child in list(node.childNodes):
        if child.nodeType == minidom.Node.COMMENT_NODE:
            node.removeChild(child)
        else:
            _remove_comments(child)


def _setup(context):
    share = Path(get_package_share_directory("ur7e_d435_gazebo"))
    lib = Path(get_package_prefix("ur7e_d435_gazebo")) / "lib"
    if not (lib / "librealsense_gazebo_plugin.so").is_file():
        raise RuntimeError("Build ur7e_d435_gazebo first: RealSense plugin is missing")
    reconstruction = Path(get_package_share_directory('ur7e_reconstruction'))
    document = xacro.process_file(str(share / "urdf/ur7e_d435.urdf.xacro"), mappings={
        'initial_positions_file': str(reconstruction / 'config/high_initial_positions.yaml')})
    for sensor in document.getElementsByTagName('sensor'):
        rate = 10 if sensor.getAttribute('name') == 'cameradepth' else (5 if sensor.getAttribute('name') == 'cameracolor' else 1)
        for element in sensor.getElementsByTagName('update_rate'):
            element.firstChild.nodeValue = str(rate)
        for element in sensor.getElementsByTagName('width'):
            element.firstChild.nodeValue = '320'
        for element in sensor.getElementsByTagName('height'):
            element.firstChild.nodeValue = '180'
    for tag, value in [('depthUpdateRate', '10'), ('colorUpdateRate', '5'), ('pointCloud', 'false')]:
        for element in document.getElementsByTagName(tag):
            element.firstChild.nodeValue = value
    _remove_comments(document)
    description = ParameterValue(document.toxml(), value_type=str)
    return [
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(filter(None, [str(lib), os.environ.get("GAZEBO_PLUGIN_PATH", "")]))),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.pathsep.join(filter(None, ["/usr/share/gazebo-11", os.environ.get("GAZEBO_RESOURCE_PATH", "")]))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": True, "robot_description": description}], output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("gazebo_ros")) / "launch/gazebo.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gazebo_gui")}.items()),
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
        DeclareLaunchArgument("gazebo_gui", default_value="false", description="Open the Gazebo GUI"),
        OpaqueFunction(function=_setup),
    ])
