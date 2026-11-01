"""Driver-schema tests; additionally use the actual ROS parameter parser on Foxy."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

try:
    import rclpy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    modules = {
        'ament_index_python': {}, 'ament_index_python.packages': {'get_package_share_directory': lambda _: ''},
        'launch': {'LaunchDescription': object},
        'launch.actions': {key: object for key in ('DeclareLaunchArgument', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler')},
        'launch.event_handlers': {'OnShutdown': object},
        'launch.substitutions': {'LaunchConfiguration': object},
        'launch_ros': {}, 'launch_ros.actions': {'Node': object},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        sys.modules[name] = module

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('d435_launch', PACKAGE / 'launch/d435.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class ConfigurationTests(unittest.TestCase):
    def config(self):
        params = launch._read_config(PACKAGE / 'config/d435.yaml')
        for key in ('camera_namespace', 'log_level', 'output', 'driver_api'):
            params.pop(key)
        return params

    def test_main_settings_preserved_for_split_driver(self):
        params = self.config()
        converted, _ = launch._translate(params, 'split')
        self.assertEqual(converted, params)
        self.assertTrue(converted['enable_color'])
        self.assertTrue(converted['pointcloud.enable'])
        self.assertEqual(converted['depth_module.depth_profile'], '848x480x90')

    def test_legacy_conversion(self):
        params, _ = launch._translate(self.config(), 'legacy')
        self.assertEqual(params['infra_width'], 848)
        self.assertEqual(params['infra_fps'], 90.0)
        self.assertIs(type(params['infra_fps']), float)
        self.assertEqual(params['color_fps'], 60.0)
        self.assertTrue(params['enable_pointcloud'])
        self.assertTrue(params['align_depth'])
        self.assertEqual(params['pointcloud_texture_stream'], 'RS2_STREAM_DEPTH')
        self.assertEqual(params['stereo_module.emitter_enabled'], 1)
        self.assertEqual(params['infra_qos'], 'SYSTEM_DEFAULT')
        self.assertEqual(params['filters'], '')
        self.assertNotIn('depth_module.depth_profile', params)

    def test_shared_profile_conversion_and_conflict(self):
        params, _ = launch._translate(self.config(), 'profile')
        self.assertEqual(params['depth_module.profile'], '848,480,90')
        self.assertEqual(params['rgb_camera.profile'], '848,480,60')
        self.assertTrue(params['allow_no_texture_points'])
        bad = self.config()
        bad['depth_module.infra_profile'] = '640x480x30'
        with self.assertRaisesRegex(ValueError, 'must match'):
            launch._translate(bad, 'profile')

    def test_driver_detection_uses_parameter_declarations(self):
        for api, name in [('legacy', 'depth_width'), ('profile', 'depth_module.profile'),
                          ('split', 'depth_module.depth_profile')]:
            with self.subTest(api=api), tempfile.TemporaryDirectory() as directory:
                share = Path(directory)
                (share / 'launch').mkdir()
                (share / 'launch/rs_launch.py').write_text(
                    'configurable_parameters = [{"name": ' + repr(name) + ', "default": "0"}]')
                self.assertEqual(launch._detect_api(share), api)

    def test_real_installed_driver_detection(self):
        if not HAS_ROS:
            self.skipTest('ROS is not installed locally')
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('realsense2_camera'))
        self.assertIn(launch._detect_api(share), ('legacy', 'profile', 'split'))

    def test_nested_parameter_dump(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'dump.yaml'
            path.write_text('/cam/front:\n  ros__parameters:\n    serial_no: "001234"\n'
                            '    depth_module:\n      depth_profile: "640x480x30"\n')
            params = launch._read_config(path)
            self.assertEqual(params['camera_name'], 'front')
            self.assertEqual(params['camera_namespace'], 'cam')
            self.assertEqual(params['serial_no'], '001234')
            self.assertEqual(params['depth_module.depth_profile'], '640x480x30')

    def test_parameter_file_retains_string_and_numeric_types(self):
        params = {'serial_no': '001234', 'device_type': '', 'depth_fps': 90.0,
                  'enable_depth': True, 'depth_width': 848}
        path = launch._write_parameters(params)
        try:
            self.assertEqual(yaml.safe_load(path.read_text())['/**']['ros__parameters'], params)
            if HAS_ROS:
                rclpy.init()
                try:
                    node = rclpy.create_node('camera', namespace='/camera',
                        cli_args=['--ros-args', '--params-file', str(path)],
                        automatically_declare_parameters_from_overrides=True)
                    try:
                        for key, value in params.items():
                            actual = node.get_parameter(key).value
                            self.assertEqual(actual, value)
                            self.assertIs(type(actual), type(value))
                    finally:
                        node.destroy_node()
                finally:
                    rclpy.shutdown()
        finally:
            path.unlink()

    def test_foxy_launch_action_and_shutdown_cleanup(self):
        if not HAS_ROS:
            self.skipTest('ROS is not installed locally')
        from launch import LaunchContext, LaunchDescription, LaunchService
        from launch.actions import EmitEvent
        from launch.events import Shutdown
        context = LaunchContext()
        context.launch_configurations.update({
            'config_file': str(PACKAGE / 'config/d435.yaml'), 'driver_api': 'auto'})
        paths = []
        writer = launch._write_parameters

        def record(params):
            path = writer(params)
            paths.append(path)
            return path

        with patch.object(launch, '_write_parameters', side_effect=record):
            actions = launch._launch_camera(context)
        self.assertEqual(len(paths), 1)
        self.assertTrue(paths[0].is_file())
        # Exercise real Foxy launch cleanup, without opening a physical camera.
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            *actions[:-1], EmitEvent(event=Shutdown(reason='configuration test'))]))
        self.assertEqual(service.run(), 0)
        self.assertFalse(paths[0].exists())

    def test_bad_profiles_and_unsupported_settings(self):
        for value in ('640x480', '640x480x0', 640):
            with self.subTest(value=value), self.assertRaises(ValueError):
                launch._profile(value)
        bad = self.config()
        bad['enable_rgbd'] = True
        with self.assertRaises(ValueError):
            launch._translate(bad, 'legacy')


if __name__ == '__main__':
    unittest.main()
