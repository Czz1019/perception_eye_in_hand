"""Project-owned orchestration, keeping sensors/perception/robot/visualization separate."""
import importlib.util
import shlex
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def v(k):return LaunchConfiguration(k).perform(context)
    def flag(k):
        if v(k) not in ('true','false'):raise ValueError(k+' must be true or false')
        return v(k)=='true'
    share=Path(get_package_share_directory('nvblox_in_hand'));actions=[]
    if v('rosbag'):
        bag=Path(v('rosbag')).expanduser().resolve()
        if not bag.exists():raise FileNotFoundError(bag)
        if flag('start_scene') or flag('launch_camera') or not flag('use_sim_time'):
            raise ValueError('Bag replay requires start_scene=false launch_camera=false use_sim_time=true')
        # Replay a recorded /clock by default. For bags without /clock, the user
        # may pass rosbag_args:="--clock"; never create two clock publishers.
        actions.append(ExecuteProcess(cmd=['ros2','bag','play',str(bag)]+shlex.split(v('rosbag_args')),output='screen'))
    if flag('start_scene') and not flag('use_sim_time'):raise ValueError('Gazebo requires use_sim_time=true')
    if flag('launch_camera') and (flag('start_scene') or flag('use_sim_time')):raise ValueError('Physical D435 requires start_scene=false use_sim_time=false')
    if flag('start_scene'):
        scene=Path(get_package_share_directory('ur7e_d435_gazebo'))/'launch/scene.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scene)),launch_arguments={k:v(k) for k in ('gazebo_gui','camera_width','camera_height')}.items()))
    if flag('launch_camera'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors/d435.launch.py')),launch_arguments={k:v(k) for k in ('camera_width','camera_height')}.items()))
    mapping=share/'launch/perception/nvblox.launch.py'
    spec=importlib.util.spec_from_file_location('in_hand_mapping',mapping);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # Reuse the mapping launch's argument declarations, without manually duplicating its interface.
    forwarded={a.name:v(a.name) for a in module.generate_launch_description().entities if isinstance(a,DeclareLaunchArgument)}
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mapping)),launch_arguments=forwarded.items()))
    if flag('publish_pose'):
        actions.append(Node(package='nvblox_in_hand',executable='robot_pose.py',parameters=[{'use_sim_time':flag('use_sim_time'),'global_frame':v('global_frame'),'publish_path':flag('publish_path')}],remappings=[('camera_info',v('depth_info_topic'))],output='screen'))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/visualization/visualization.launch.py')),
        launch_arguments={k:v(k) for k in ('run_rviz','use_sim_time','rviz_mode','esdf_mode','mapper_backend')}.items()))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('nvblox_in_hand'))
    spec=importlib.util.spec_from_file_location('in_hand_mapping_declarations',share/'launch/perception/nvblox.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    args=[a for a in module.generate_launch_description().entities if isinstance(a,DeclareLaunchArgument)]
    flags=dict(start_scene='true',gazebo_gui='true',run_rviz='true',launch_camera='false',publish_pose='true',publish_path='true',rviz_mode='auto',rosbag='',rosbag_args='',camera_width='640',camera_height='480')
    return LaunchDescription(args+[DeclareLaunchArgument(k,default_value=val) for k,val in flags.items()]+[OpaqueFunction(function=setup)])
