from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ur7e_real_reconstruction'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/real.yaml')),
        DeclareLaunchArgument('run_rviz', default_value='true'),
        DeclareLaunchArgument('load', default_value='', description='Saved surface.npz; preview without scanning'),
        Node(package='ur7e_real_reconstruction', executable='real_reconstruction',
             arguments=['--config', LaunchConfiguration('config'), '--load', LaunchConfiguration('load')], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/view.rviz')],
             condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ])
