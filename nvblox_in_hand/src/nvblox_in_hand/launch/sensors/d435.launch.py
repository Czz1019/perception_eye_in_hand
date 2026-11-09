"""Optional D435 driver; calibrated optical TF is owned by the robot URDF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    dims = [LaunchConfiguration(k).perform(context) for k in ('camera_width', 'camera_height')]
    if any(not text.isdigit() or int(text) <= 0 for text in dims):
        raise ValueError('camera_width and camera_height must be positive integers')
    share = Path(get_package_share_directory('nvblox_in_hand'))
    profile = 'x'.join(dims+['30'])
    return [Node(package='realsense2_camera', executable='realsense2_camera_node',
        name='camera', namespace='', parameters=[str(share/'config/sensors/d435.yaml'),
            {'depth_module.profile': profile, 'rgb_camera.profile': profile}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'), OpaqueFunction(function=setup)])
