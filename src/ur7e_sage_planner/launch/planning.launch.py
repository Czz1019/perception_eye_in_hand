"""Start the packaged planner, optionally with the packaged tall D435 scene."""
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _planner(context):
    executable = str(Path(get_package_prefix("ur7e_sage_planner")) / "lib/ur7e_sage_planner/sage_planner")
    return [ExecuteProcess(
        cmd=[executable, "--execution-layer", LaunchConfiguration("execution_layer").perform(context),
             "--executor-frequency", LaunchConfiguration("executor_frequency").perform(context),
             "--max-joint-speed", LaunchConfiguration("max_joint_speed").perform(context),
             "--max-joint-acceleration", LaunchConfiguration("max_joint_acceleration").perform(context),
             "--max-steps", LaunchConfiguration("max_steps").perform(context)]
            + shlex.split(LaunchConfiguration("planner_args").perform(context)),
        output="screen")]


def generate_launch_description():
    share = Path(get_package_share_directory("ur7e_sage_planner"))
    scene = Path(get_package_share_directory("ur7e_d435_gazebo"))
    return LaunchDescription([
        DeclareLaunchArgument("start_scene", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("execution_layer", default_value="servo"),
        DeclareLaunchArgument("executor_frequency", default_value="200"),
        DeclareLaunchArgument("max_joint_speed", default_value="0.6"),
        DeclareLaunchArgument("max_joint_acceleration", default_value="0.0"),
        DeclareLaunchArgument("max_steps", default_value="0"),
        DeclareLaunchArgument("planner_args", default_value=""),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene / "launch/scene.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_scene")),
            launch_arguments={"gazebo_gui": LaunchConfiguration("gazebo_gui")}.items()),
        OpaqueFunction(function=_planner),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/reach_static_clean.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
