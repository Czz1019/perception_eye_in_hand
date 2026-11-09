"""Project-owned depth + TF + native ESDF + visualization bringup."""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        flag = value(name).lower()
        if flag not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return flag == 'true'

    root = Path(os.environ['UR7E_PERCEPTION_ROOT'])
    stride = int(value('depth_stride'))
    if stride < 1:
        raise ValueError('depth_stride must be positive')
    config = Path(value('mapper_config')).expanduser().resolve()
    if not config.is_file():
        raise FileNotFoundError(config)
    sim_time = 'true' if enabled('use_sim_time') else 'false'
    actions = []
    if enabled('start_scene'):
        if sim_time != 'true':
            raise ValueError('start_scene=true requires use_sim_time=true')
        scene = Path(get_package_share_directory('ur7e_d435_gazebo')) / 'launch/scene.launch.py'
        # Reuse the scene's check before starting any mapper/input processes.
        spec = importlib.util.spec_from_file_location('ur7e_scene_preflight', scene)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module._check_gazebo_master_available(
            os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene)),
            launch_arguments={'gazebo_gui': value('gazebo_gui')}.items()))
    else:
        actions.append(LogInfo(msg='Using external depth/TF; no Gazebo scene will be created.'))

    depth = ExecuteProcess(
        cmd=[str(root / 'run_ur7e_common_depth.sh'), '--stride', str(stride),
             '--ros-args', '-p', 'use_sim_time:=' + sim_time,
             '-p', 'depth_topic:=' + value('depth_topic'),
             '-p', 'camera_info_topic:=' + value('camera_info_topic')],
        output='screen')
    mapper = ExecuteProcess(
        cmd=[str(root / 'run_ur7e_native.sh'), '-p', 'use_sim_time:=' + sim_time],
        additional_env={'NVBLOX_NATIVE_CONFIG': str(config),
                        'NVBLOX_COLOR_TOPIC': value('color_topic'),
                        'NVBLOX_COLOR_INFO_TOPIC': value('color_camera_info_topic')}, output='screen')
    actions.extend([
        LogInfo(msg=f'Project perception: {root}; config: {config}; depth stride: {stride}'),
        depth, mapper,
    ])
    # A failed input or mapper must not leave a partially running perception app.
    for process in (depth, mapper):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Required perception process exited'))])))
    if enabled('run_rviz'):
        actions.append(ExecuteProcess(
            cmd=[str(root / 'run_ur7e_native_rviz.sh')],
            additional_env={'ESDF_USE_SIM_TIME': sim_time}, output='screen'))
    return actions


def generate_launch_description():
    root = Path(os.environ['UR7E_PERCEPTION_ROOT'])
    return LaunchDescription([
        DeclareLaunchArgument('start_scene', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('run_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('depth_stride', default_value='2'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('color_camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('mapper_config', default_value=str(
            root / 'src/ur7e_nvblox_native/config/gazebo.yaml')),
        OpaqueFunction(function=_setup),
    ])
