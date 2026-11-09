"""Static D435 mapping. Upstream NvbloxNode owns all core operations."""
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def v(k):return LaunchConfiguration(k).perform(context)
    def flag(k):
        s=v(k).lower()
        if s not in ('true','false'):raise ValueError(k+' must be true or false')
        return s=='true'
    share=Path(get_package_share_directory('nvblox_in_hand'))
    mode=v('esdf_mode');backend=v('mapper_backend')
    if mode not in ('2d','3d'):raise ValueError('esdf_mode must be 2d or 3d')
    if backend not in ('official','observed_view'):raise ValueError('mapper_backend must be official or observed_view')
    cfg=[str(share/'config/nvblox/base.yaml'),str(share/'config/nvblox'/('simulation.yaml' if flag('use_sim_time') else 'realsense.yaml'))]
    if v('mapper_config'):
        custom=Path(v('mapper_config')).expanduser().resolve()
        if not custom.is_file():raise FileNotFoundError(custom)
        cfg.append(str(custom))
    overrides=dict(use_sim_time=flag('use_sim_time'),global_frame=v('global_frame'),esdf_mode=mode,
        use_color=flag('enable_color'),use_depth=flag('enable_depth'),use_lidar=False,num_cameras=1,
        mapping_type='static_tsdf',use_tf_transforms=True,use_topic_transforms=False,
        update_esdf_rate_hz=float(v('esdf_hz')) if flag('enable_esdf') else 0.,
        update_mesh_rate_hz=float(v('mesh_hz')) if flag('enable_mesh') else 0.,
        publish_layer_rate_hz=max(float(v('layer_hz')) if flag('enable_layers') else 0., float(v('mesh_hz')) if flag('enable_mesh') else 0.),
        integrate_depth_rate_hz=float(v('depth_hz')),integrate_color_rate_hz=float(v('color_hz')),
        decay_tsdf_rate_hz=float(v('decay_hz')),map_clearing_radius_m=float(v('clearing_radius_m')),
        clear_map_outside_radius_rate_hz=float(v('clearing_hz')))
    for name in ('voxel_size','max_range_m'):
        if v(name) and (not math.isfinite(float(v(name))) or float(v(name))<=0):raise ValueError(name+' must be finite and positive')
    if v('voxel_size'):overrides['voxel_size']=float(v('voxel_size'))
    if v('max_range_m'):overrides['static_mapper.projective_integrator_max_integration_distance_m']=float(v('max_range_m'))
    for name in ('depth_hz','color_hz','esdf_hz','mesh_hz','layer_hz','decay_hz','clearing_hz','esdf_view_hz'):
        if not math.isfinite(float(v(name))) or float(v(name))<0:raise ValueError(name+' must be finite and nonnegative')
    if not math.isfinite(float(v('clearing_radius_m'))):raise ValueError('clearing_radius_m must be finite')
    if backend=='observed_view':
        overrides.update(enable_esdf_view=flag('enable_esdf_view'),esdf_view_hz=float(v('esdf_view_hz')),esdf_view_band_m=float(v('esdf_view_band_m')))
    remaps=[]
    if not flag('enable_mesh'):remaps.append(('/nvblox_node/mesh','/nvblox_in_hand/disabled/mesh'))
    if not flag('enable_layers'):
        remaps += [('/nvblox_node/'+k,'/nvblox_in_hand/disabled/'+k) for k in ('tsdf_layer','tsdf_layer_marker','color_layer','color_layer_marker')]
    mapper=Node(package='nvblox_ros' if backend=='official' else 'nvblox_in_hand',
        executable='nvblox_node' if backend=='official' else 'mapper_node',name='nvblox_node',
        parameters=cfg+[overrides],remappings=[
            ('camera_0/depth/image',v('depth_topic')),('camera_0/depth/camera_info',v('depth_info_topic')),
            ('camera_0/color/image',v('color_topic')),('camera_0/color/camera_info',v('color_info_topic'))]+remaps,
        output='screen')
    return [mapper,RegisterEventHandler(OnProcessExit(target_action=mapper,on_exit=[EmitEvent(event=Shutdown(reason='Required mapper exited'))]))]


def generate_launch_description():
    defaults=dict(use_sim_time='true',global_frame='base_link',mapper_backend='observed_view',mapper_config='',
        esdf_mode='3d',enable_depth='true',enable_color='true',enable_esdf='true',enable_mesh='true',
        enable_layers='true',enable_esdf_view='true',voxel_size='',max_range_m='',
        depth_hz='30.0',color_hz='30.0',esdf_hz='30.0',mesh_hz='5.0',layer_hz='5.0',
        esdf_view_hz='30.0',esdf_view_band_m='0.12',decay_hz='0.0',clearing_hz='0.0',clearing_radius_m='-1.0',
        depth_topic='/camera/depth/image_rect_raw',depth_info_topic='/camera/depth/camera_info',
        color_topic='/camera/color/image_raw',color_info_topic='/camera/color/camera_info')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=val) for k,val in defaults.items()]+[OpaqueFunction(function=setup)])
