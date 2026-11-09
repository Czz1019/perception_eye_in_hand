"""Single-D435 specialization of nvblox_examples/perception/nvblox.launch.py.

The production backend loads the unmodified upstream NvbloxNode component.
Project diagnostics are an explicit alternative, never a second mapper.
"""
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.actions import OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def setup(context):
    def value(name):
        return context.launch_configurations[name]

    def flag(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        return text == 'true'

    def rate(name):
        number = float(value(name))
        if not math.isfinite(number) or number < 0:
            raise ValueError(name + ' must be finite and nonnegative')
        return number

    share = Path(get_package_share_directory('nvblox_in_hand'))
    backend = value('mapper_backend')
    if backend not in ('official', 'diagnostic', 'observed_view'):
        raise ValueError('mapper_backend must be official or diagnostic (observed_view is a legacy alias)')
    if value('esdf_mode') not in ('2d', '3d'):
        raise ValueError('esdf_mode must be 2d or 3d')
    if value('esdf_view_mode') not in ('observed', 'band', 'inside', 'surface'):
        raise ValueError('esdf_view_mode must be observed, band or inside (surface is a legacy alias)')
    if value('profile') not in ('realtime', 'reference'):
        raise ValueError('profile must be realtime or reference')
    if not value('container_name') or '/' in value('container_name'):
        raise ValueError('container_name must be a nonempty ROS node basename')

    # Same layering as the upstream example: base -> camera -> application -> CLI.
    configs = [str(share/'config/nvblox/nvblox_base.yaml'),
               str(share/'config/nvblox/specializations/nvblox_realsense.yaml'),
               str(share/'config/robot/ur7e.yaml'),
               str(share/'config/profiles'/(value('profile')+'.yaml')),
               str(share/'config/nvblox'/('simulation.yaml' if flag('use_sim_time') else 'realsense.yaml'))]
    if value('mapper_config'):
        custom = Path(value('mapper_config')).expanduser().resolve()
        if not custom.is_file():
            raise FileNotFoundError(custom)
        configs.append(str(custom))
    overrides = dict(use_sim_time=flag('use_sim_time'), global_frame=value('global_frame'),
        esdf_mode=value('esdf_mode'), use_depth=flag('enable_depth'), use_color=flag('enable_color'),
        use_lidar=False, num_cameras=1, mapping_type='static_tsdf',
        use_tf_transforms=True, use_topic_transforms=False,
        publish_esdf_distance_slice=flag('publish_esdf_distance_slice'))
    rate_names = {'depth_hz': 'integrate_depth_rate_hz', 'color_hz': 'integrate_color_rate_hz',
                  'esdf_hz': 'update_esdf_rate_hz', 'mesh_hz': 'update_mesh_rate_hz',
                  'layer_hz': 'publish_layer_rate_hz', 'decay_hz': 'decay_tsdf_rate_hz',
                  'clearing_hz': 'clear_map_outside_radius_rate_hz'}
    for arg, param in rate_names.items():
        if value(arg):
            overrides[param] = rate(arg)
    # In this local ROS port, update_mesh_rate_hz is declared but not consumed
    # by tick(). LayerPublisher computes subscribed mesh during publishLayers.
    # Keep the old CLI spelling as an honest alias for that shared schedule.
    if value('mesh_hz'):
        if value('layer_hz') and rate('mesh_hz') != rate('layer_hz'):
            raise ValueError('This ROS port shares mesh/layer publication: mesh_hz and layer_hz must match; use enable_mesh to disable mesh only')
        overrides['publish_layer_rate_hz'] = rate('mesh_hz')
    for arg, param in [('voxel_size', 'voxel_size'),
                       ('max_range_m', 'static_mapper.projective_integrator_max_integration_distance_m')]:
        if value(arg):
            number = float(value(arg))
            if not math.isfinite(number) or number <= 0:
                raise ValueError(arg + ' must be finite and positive')
            overrides[param] = number
    if value('clearing_radius_m'):
        radius = float(value('clearing_radius_m'))
        if not math.isfinite(radius):
            raise ValueError('clearing_radius_m must be finite')
        overrides['map_clearing_radius_m'] = radius
    for arg, param in [('slice_height', 'esdf_slice_height'),
                       ('slice_min_height', 'esdf_slice_min_height'),
                       ('slice_max_height', 'esdf_slice_max_height')]:
        if value(arg):
            height = float(value(arg))
            if not math.isfinite(height):
                raise ValueError(arg + ' must be finite')
            overrides['static_mapper.'+param] = height
    if value('slice_min_height') and value('slice_max_height') and float(value('slice_min_height')) > float(value('slice_max_height')):
        raise ValueError('slice_min_height must not exceed slice_max_height')
    if not flag('enable_esdf'):
        overrides['update_esdf_rate_hz'] = 0.0
    if not flag('enable_mesh'):
        overrides['update_mesh_rate_hz'] = 0.0
    # Upstream uses one publication task for both mesh and voxel layers. Turning
    # voxel layers off must not silently turn mesh publication off.
    if not flag('enable_layers') and not flag('enable_mesh'):
        overrides['publish_layer_rate_hz'] = 0.0
    remaps = [('camera_0/depth/image', value('depth_topic')),
              ('camera_0/depth/camera_info', value('depth_info_topic')),
              ('camera_0/color/image', value('color_topic')),
              ('camera_0/color/camera_info', value('color_info_topic'))]
    if not flag('enable_mesh'):
        remaps.append(('~/mesh', '/nvblox_in_hand/disabled/mesh'))
    if not flag('enable_layers'):
        remaps += [('~/'+name, '/nvblox_in_hand/disabled/'+name) for name in
                   ('tsdf_layer', 'tsdf_layer_marker', 'color_layer', 'color_layer_marker')]
    if backend == 'official':
        component = ComposableNode(package='nvblox_ros', plugin='nvblox::NvbloxNode',
            name='nvblox_node', parameters=configs+[overrides], remappings=remaps)
        if flag('attach_to_container'):
            return [LoadComposableNodes(target_container='/'+value('container_name'),
                                        composable_node_descriptions=[component])]
        container = ComposableNodeContainer(name=value('container_name'), namespace='',
            package='rclcpp_components', executable='component_container_mt',
            composable_node_descriptions=[component], output='screen',
            arguments=['--ros-args', '--log-level', value('log_level')])
        return [container, RegisterEventHandler(OnProcessExit(target_action=container,
            on_exit=[EmitEvent(event=Shutdown(reason='Required nvblox container exited'))]))]
    if flag('attach_to_container'):
        raise ValueError('diagnostic backend is standalone; attach_to_container requires official')
    band = float(value('esdf_view_band_m'))
    if not math.isfinite(band) or band <= 0:
        raise ValueError('esdf_view_band_m must be finite and positive')
    overrides.update(enable_esdf_view=flag('enable_esdf_view'), esdf_view_hz=rate('esdf_view_hz'),
                     esdf_view_band_m=band, esdf_view_mode=value('esdf_view_mode'))
    node = Node(package='nvblox_in_hand', executable='mapper_node', name='nvblox_node',
                parameters=configs+[overrides], remappings=remaps, output='screen')
    return [node, RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required diagnostic mapper exited'))]))]


def generate_launch_description():
    defaults = dict(use_sim_time='true', global_frame='base_link', mapper_backend='official',
        mapper_config='', profile='realtime', container_name='nvblox_container',
        attach_to_container='false', log_level='info', esdf_mode='3d',
        enable_depth='true', enable_color='true', enable_esdf='true', enable_mesh='true',
        enable_layers='true', publish_esdf_distance_slice='true',
        voxel_size='', max_range_m='', depth_hz='', color_hz='', esdf_hz='', mesh_hz='', layer_hz='',
        decay_hz='', clearing_hz='', clearing_radius_m='',
        slice_height='', slice_min_height='', slice_max_height='',
        enable_esdf_view='true', esdf_view_hz='33.4', esdf_view_band_m='0.12', esdf_view_mode='observed',
        depth_topic='/camera/depth/image_rect_raw', depth_info_topic='/camera/depth/camera_info',
        color_topic='/camera/color/image_raw', color_info_topic='/camera/color/camera_info')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
