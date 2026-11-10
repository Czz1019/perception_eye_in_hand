"""Official workspaces NvbloxNode with robot-TF Gazebo RGB-D input.

No VSLAM, identity TF aliases, camera-info substitution or project mapper plugin.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = Path(get_package_share_directory('nvblox_examples_bringup'))
    mapper = ComposableNode(
        name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[str(share/'config/nvblox/nvblox_base.yaml'),
                    str(share/'config/nvblox/specializations/nvblox_realsense.yaml'),
                    str(share/'config/nvblox/ur7e_lab_tf.yaml')],
        remappings=[('camera_0/depth/image', '/camera/depth/image_rect_raw'),
                    ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
                    ('camera_0/color/image', '/camera/color/image_raw'),
                    ('camera_0/color/camera_info', '/camera/color/camera_info')])
    return LaunchDescription([ComposableNodeContainer(
        name='ur7e_lab_nvblox_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[mapper],
        output='screen')])
