"""Select a view that matches the actual mapper output; never launch another mapper."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    if value('run_rviz').lower() not in ('true', 'false'):
        raise ValueError('run_rviz must be true or false')
    if value('run_rviz').lower() == 'false':
        return []
    mode = value('rviz_mode')
    if mode == 'auto':
        mode = '2d' if value('esdf_mode') == '2d' else (
            'diagnostic' if value('mapper_backend') != 'official' else 'mesh')
    configs = {'mesh': 'mesh.rviz', '2d': 'official_2d.rviz', 'diagnostic': 'diagnostic.rviz'}
    if mode not in configs:
        raise ValueError('rviz_mode must be auto, mesh, 2d or diagnostic')
    share = Path(get_package_share_directory('nvblox_in_hand'))
    return [Node(package='rviz2', executable='rviz2',
        arguments=['-d', str(share/'rviz'/configs[mode])],
        parameters=[{'use_sim_time': value('use_sim_time').lower() == 'true'}], output='screen')]


def generate_launch_description():
    defaults = dict(run_rviz='true', use_sim_time='true', rviz_mode='auto',
                    esdf_mode='3d', mapper_backend='official')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
