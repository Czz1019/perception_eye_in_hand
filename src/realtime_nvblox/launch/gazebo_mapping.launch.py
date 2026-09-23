"""Independent Gazebo mapper; no RealSenseSource or cuVSLAM runtime."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('realtime_nvblox'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/gazebo_robot.yaml')),
        DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('run_rviz', default_value='false'),
        Node(package='realtime_nvblox', executable='gazebo_ros_mapper', name='realtime_nvblox',
             prefix=[LaunchConfiguration('python_executable')], output='screen',
             parameters=[{'config': LaunchConfiguration('config'), 'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(share/'rviz/gazebo_esdf.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ])
