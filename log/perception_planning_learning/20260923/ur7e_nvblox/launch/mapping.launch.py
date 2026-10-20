from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ur7e_nvblox'))
    python = Path.home() / 'miniconda3/envs/whole_control/bin/python'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/gazebo.yaml')),
        DeclareLaunchArgument('python_executable', default_value=str(python)),
        DeclareLaunchArgument('run_rviz', default_value='false'),
        Node(package='ur7e_nvblox', executable='tf_mapper', name='ur7e_nvblox',
             prefix=[LaunchConfiguration('python_executable')], output='screen',
             parameters=[LaunchConfiguration('config')]),
        Node(package='rviz2', executable='rviz2', name='ur7e_esdf_rviz',
             arguments=['-d', str(share / 'rviz/esdf.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ])
