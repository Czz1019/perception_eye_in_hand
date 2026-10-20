from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def setup(context):
    share=Path(get_package_share_directory('ur7e_auto_esdf'))
    mapping=LaunchConfiguration('mapping').perform(context)
    c=yaml.safe_load(Path(mapping).read_text())
    tf=[('/tf','/ur7e_auto_esdf/tf'),('/tf_static','/ur7e_auto_esdf/tf_static')]
    return [
        Node(package='ur7e_auto_esdf',executable='auto_mapper',name='ur7e_auto_esdf_mapper',
             arguments=['--config',mapping,'--load',LaunchConfiguration('load')],remappings=tf,output='screen'),
        Node(package='ur7e_auto_esdf',executable='auto_scan',
             arguments=['--mapping',mapping,'--control',LaunchConfiguration('control'),
                        '--waypoints',LaunchConfiguration('waypoints'),'--allow-motion',LaunchConfiguration('allow_motion'),
                        '--approved-plan',LaunchConfiguration('approved_plan')],output='screen',
             condition=IfCondition(LaunchConfiguration('run_controller'))),
        Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='ur7e_auto_esdf',
             parameters=[{'robot_description':ParameterValue(Path(c['robot_urdf']).expanduser().read_text(),value_type=str),
                          'publish_frequency':50.}],remappings=tf+[('joint_states','/ur7e_auto_esdf/joint_states')],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/view.rviz')],remappings=tf,
             condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ]


def generate_launch_description():
    share=Path(get_package_share_directory('ur7e_auto_esdf'))
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in dict(
        mapping=str(share/'config/mapping.yaml'),control=str(share/'config/control.yaml'),
        waypoints=str(share/'config/waypoints.yaml'),allow_motion='false',approved_plan='',
        run_rviz='true',run_controller='true',load='').items()]
    return LaunchDescription(args+[OpaqueFunction(function=setup)])
