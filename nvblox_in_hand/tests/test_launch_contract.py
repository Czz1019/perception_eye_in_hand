"""Offline example/robot specialization contract; does not initialize ROS or CUDA."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'src/nvblox_in_hand'
spec = importlib.util.spec_from_file_location('mapping', PACKAGE/'launch/perception/nvblox.launch.py')
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


class LaunchContract(unittest.TestCase):
    def invoke(self, **changes):
        context = LaunchContext()
        for arg in mapping.generate_launch_description().entities:
            if isinstance(arg, DeclareLaunchArgument):
                context.launch_configurations[arg.name] = ''.join(x.perform(context) for x in arg.default_value)
        context.launch_configurations.update(changes)
        with patch.object(mapping, 'get_package_share_directory', return_value=str(PACKAGE)), \
             patch.object(mapping, 'Node', side_effect=lambda **kw: dict(kind='node', **kw)), \
             patch.object(mapping, 'ComposableNode', side_effect=lambda **kw: dict(kind='component', **kw)), \
             patch.object(mapping, 'ComposableNodeContainer', side_effect=lambda **kw: dict(kind='container', **kw)), \
             patch.object(mapping, 'LoadComposableNodes', side_effect=lambda **kw: dict(kind='load', **kw)), \
             patch.object(mapping, 'OnProcessExit', side_effect=lambda **kw: None), \
             patch.object(mapping, 'RegisterEventHandler', side_effect=lambda *a: None):
            return mapping.setup(context)[0]

    def mapper(self, **changes):
        action = self.invoke(**changes)
        return action['composable_node_descriptions'][0] if action['kind'] in ('container', 'load') else action

    def effective(self, node):
        result = {}
        def merge(data, prefix=''):
            for key, val in data.items():
                if isinstance(val, dict):
                    merge(val, prefix+key+'.')
                else:
                    result[prefix+key] = val
        for config in node['parameters']:
            merge(config if isinstance(config, dict) else yaml.safe_load(Path(config).read_text())['/**']['ros__parameters'])
        return result

    def test_default_runs_official_component_without_custom_export(self):
        action = self.invoke()
        self.assertEqual(action['executable'], 'component_container_mt')
        self.assertEqual(len(action['composable_node_descriptions']), 1)
        node = self.mapper()
        self.assertEqual(node['package'], 'nvblox_ros')
        self.assertEqual(node['plugin'], 'nvblox::NvbloxNode')
        self.assertNotIn('enable_esdf_view', node['parameters'][-1])

    def test_attach_loads_one_official_mapper_into_existing_container(self):
        action = self.invoke(attach_to_container='true', container_name='external_nvblox')
        self.assertEqual(action['kind'], 'load')
        self.assertEqual(action['target_container'], '/external_nvblox')
        self.assertEqual(len(action['composable_node_descriptions']), 1)

    def test_robot_tf_and_independent_depth_color_calibration(self):
        node = self.mapper(); params = self.effective(node)
        self.assertTrue(params['use_tf_transforms']); self.assertFalse(params['use_topic_transforms'])
        self.assertFalse(params['use_lidar']); self.assertEqual(params['num_cameras'], 1)
        self.assertEqual(params['mapping_type'], 'static_tsdf')
        self.assertEqual(params['global_frame'], 'base_link')
        self.assertIn(('camera_0/depth/image', '/camera/depth/image_rect_raw'), node['remappings'])
        self.assertIn(('camera_0/color/camera_info', '/camera/color/camera_info'), node['remappings'])
        self.assertIn(('camera_0/depth/camera_info', '/camera/depth/camera_info'), node['remappings'])

    def test_requested_realtime_profile_retains_resolution_and_rates(self):
        params = self.effective(self.mapper())
        self.assertEqual(params['voxel_size'], .03)
        self.assertEqual(params['integrate_depth_rate_hz'], 40.)
        self.assertEqual(params['integrate_color_rate_hz'], 40.)
        self.assertEqual(params['update_esdf_rate_hz'], 33.4)
        self.assertEqual(params['publish_layer_rate_hz'], 5.)
        self.assertEqual(params['static_mapper.projective_integrator_max_integration_distance_m'], 3.)

    def test_reference_profile_inherits_base_and_cli_overrides_are_explicit(self):
        params = self.effective(self.mapper(profile='reference'))
        self.assertEqual(params['voxel_size'], .02)
        self.assertEqual(params['update_esdf_rate_hz'], 5.)
        self.assertEqual(params['decay_tsdf_rate_hz'], 2.)
        params = self.effective(self.mapper(profile='reference', voxel_size='.03', esdf_hz='30'))
        self.assertEqual(params['voxel_size'], .03); self.assertEqual(params['update_esdf_rate_hz'], 30.)

    def test_custom_config_is_not_silently_overridden_by_rate_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            custom = Path(folder)/'custom.yaml'
            custom.write_text('/**:\n  ros__parameters:\n    voxel_size: 0.015\n    update_esdf_rate_hz: 9.0\n')
            params = self.effective(self.mapper(mapper_config=str(custom)))
            self.assertEqual(params['voxel_size'], .015); self.assertEqual(params['update_esdf_rate_hz'], 9.)

    def test_layer_disable_retains_mesh_and_core_outputs(self):
        node = self.mapper(enable_layers='false'); params = self.effective(node)
        self.assertGreater(params['publish_layer_rate_hz'], 0)
        self.assertIn(('~/color_layer', '/nvblox_in_hand/disabled/color_layer'), node['remappings'])
        params = self.effective(self.mapper(enable_esdf='false', enable_mesh='false', enable_layers='false'))
        self.assertEqual(params['update_esdf_rate_hz'], 0)
        self.assertEqual(params['update_mesh_rate_hz'], 0)
        self.assertEqual(params['publish_layer_rate_hz'], 0)

    def test_diagnostics_are_opt_in_and_include_outside_by_default(self):
        node = self.mapper(mapper_backend='diagnostic')
        self.assertEqual(node['kind'], 'node'); self.assertEqual(node['executable'], 'mapper_node')
        self.assertEqual(node['parameters'][-1]['esdf_view_mode'], 'observed')
        self.assertEqual(node['parameters'][-1]['esdf_view_hz'], 33.4)
        self.assertEqual(self.mapper(mapper_backend='observed_view')['package'], 'nvblox_in_hand')
        node = self.mapper(mapper_backend='diagnostic', esdf_view_mode='inside')
        self.assertEqual(node['parameters'][-1]['esdf_view_mode'], 'inside')

    def test_2d_is_native_mode_not_renamed_3d_points(self):
        node = self.mapper(esdf_mode='2d')
        self.assertEqual(node['plugin'], 'nvblox::NvbloxNode')
        self.assertEqual(node['parameters'][-1]['esdf_mode'], '2d')
        self.assertTrue(node['parameters'][-1]['publish_esdf_distance_slice'])

    def test_mesh_rate_controls_the_real_shared_publication_task(self):
        params = self.effective(self.mapper(mesh_hz='2'))
        self.assertEqual(params['publish_layer_rate_hz'], 2.)
        with self.assertRaises(ValueError):
            self.invoke(mesh_hz='2', layer_hz='5')

    def test_invalid_inputs_rejected(self):
        for args in [dict(esdf_mode='humans'), dict(esdf_view_mode='cubes'),
                     dict(mapper_backend='legacy'), dict(enable_color='yes'),
                     dict(voxel_size='-1'), dict(voxel_size='nan'), dict(depth_hz='nan'),
                     dict(profile='unknown'), dict(container_name=''),
                     dict(mapper_backend='diagnostic', attach_to_container='true'),
                     dict(slice_min_height='1', slice_max_height='0')]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.invoke(**args)

if __name__ == '__main__':
    unittest.main()
