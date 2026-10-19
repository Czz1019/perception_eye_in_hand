from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    from ur7e_gazebo_esdf_sage.scene import build_robot
    share = Path(get_package_share_directory('ur7e_gazebo_esdf_sage'))
    output = Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    description = build_robot(share)
    (output/'robot.urdf').write_text(description)
    c = yaml.safe_load((share/'config/mapping.yaml').read_text())
    c.update(robot_urdf=str(output/'robot.urdf'), collision_spheres=str(share/'config/collision_spheres.yaml'),
             output_directory=str(output/'maps'))
    (output/'mapping.yaml').write_text(yaml.safe_dump(c))
    scan_override = LaunchConfiguration('scan_config').perform(context)
    scan_path = Path(scan_override).expanduser() if scan_override else share/'config/scan.yaml'
    scan=yaml.safe_load(scan_path.read_text())
    scan['planning_policy']=LaunchConfiguration('planning_policy').perform(context)
    scan['online_engine']=LaunchConfiguration('online_engine').perform(context)
    scan['online_obstacles']=LaunchConfiguration('online_obstacles').perform(context)=='true'
    scan['hold_after_scan']=LaunchConfiguration('hold_after_scan').perform(context)=='true'
    (output/'scan.yaml').write_text(yaml.safe_dump(scan,sort_keys=False))
    (output/'tabletop.yaml').write_text((share/'config/tabletop.yaml').read_text())
    replay=LaunchConfiguration('test_load_map').perform(context)
    if replay:
        import shutil
        destination=output/'maps'/'debug_replay'
        shutil.copytree(Path(replay).parent,destination)
        replay=str(destination/'frozen_map.npz')
    lib = str(Path(get_package_prefix('ur7e_d435_gazebo'))/'lib')
    return [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', lib+os.pathsep+os.environ.get('GAZEBO_PLUGIN_PATH','')),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH','/usr/share/gazebo-11'+os.pathsep+os.environ.get('GAZEBO_RESOURCE_PATH','')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time':True,'robot_description':ParameterValue(description,value_type=str)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
                                 launch_arguments={'gui':LaunchConfiguration('gazebo_gui'),'params_file':str(share/'config/gazebo.yaml')}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity','ur','-topic','robot_description','-timeout','90']),
        Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager-timeout','90']),
        Node(package='controller_manager',executable='spawner',arguments=['forward_position_controller','--controller-manager-timeout','90']),
        Node(package='ur7e_gazebo_esdf_sage',executable='spawn_fixture'),
        Node(package='ur7e_gazebo_esdf_sage',executable='closed_loop',output='screen',sigterm_timeout='35',
             arguments=['--config',str(output/'mapping.yaml'),'--output',str(output),'--auto-start',LaunchConfiguration('auto_start'),'--test-load-map',replay]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/view.rviz')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('run_rviz'))),
    ]


def generate_launch_description():
    from datetime import datetime
    return LaunchDescription([
        DeclareLaunchArgument('output',default_value=str(Path.home()/'nvblox_output/gazebo_esdf_sage'/datetime.now().strftime('%Y%m%d_%H%M%S'))),
        DeclareLaunchArgument('gazebo_gui',default_value='false'),
        DeclareLaunchArgument('run_rviz',default_value='false'),
        DeclareLaunchArgument('auto_start',default_value='true'),
        DeclareLaunchArgument('scan_config',default_value=''),
        DeclareLaunchArgument('planning_policy',default_value='strict'),
        DeclareLaunchArgument('test_load_map',default_value=''),
        DeclareLaunchArgument('online_engine',default_value='v1'),
        DeclareLaunchArgument('online_obstacles',default_value='false'),
        DeclareLaunchArgument('hold_after_scan',default_value='false'),
        OpaqueFunction(function=setup)])
