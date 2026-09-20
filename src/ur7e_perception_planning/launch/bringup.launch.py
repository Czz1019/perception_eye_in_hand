from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ur7e_perception_planning'))
    scene = share/'launch/scene.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('start_scene', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false',
                             description='Open gzclient; headless by default to avoid remote model-list requests'),
        DeclareLaunchArgument('run_rviz', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share/'config/online.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene)),
                                 launch_arguments={'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items(),
                                 condition=IfCondition(LaunchConfiguration('start_scene'))),
        Node(package='ur7e_perception_planning', executable='perception_planning',
             arguments=['combined'], parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'rviz/online.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ])
