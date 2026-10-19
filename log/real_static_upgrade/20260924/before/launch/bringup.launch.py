"""Start only the application. Robot/camera drivers are deliberately external."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',description='Absolute path to completed real.yaml'),
        DeclareLaunchArgument('run_rviz',default_value='false'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(get_package_share_directory('ur7e_real_static'))/'config/view.rviz')],condition=IfCondition(LaunchConfiguration('run_rviz'))),
        Node(package='ur7e_real_static',executable='real_static',arguments=['--config',LaunchConfiguration('config')],output='screen'),
    ])
