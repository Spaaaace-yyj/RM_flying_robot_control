"""Exercise parameter selection/conversion without a ROS installation.

Launch/Node classes are stubbed only to inspect the generated configuration;
this is not a hardware or DDS integration test.
Run: python3 -m unittest discover -s tests -p 'test_sensor_launch_parameters.py' -v
"""

import ast
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]


class FakeLaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.get(self.name, "")


class FakeAction:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


def load_launch(package, filename):
    modules = {}
    for name in (
        "ament_index_python", "ament_index_python.packages", "launch",
        "launch.actions", "launch.substitutions", "launch_ros", "launch_ros.actions",
    ):
        modules[name] = types.ModuleType(name)
    modules["ament_index_python.packages"].get_package_share_directory = (
        lambda name: str(ROOT / "src" / name))
    modules["launch"].LaunchDescription = FakeAction
    modules["launch.actions"].DeclareLaunchArgument = FakeAction
    modules["launch.actions"].OpaqueFunction = FakeAction
    modules["launch.substitutions"].LaunchConfiguration = FakeLaunchConfiguration
    modules["launch_ros.actions"].Node = FakeAction
    spec = importlib.util.spec_from_file_location(
        package, ROOT / "src" / package / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class SensorLaunchParametersTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.camera = load_launch("mindvision_camera", "mv_launch.py")
        cls.interface = load_launch("driver_interface", "driver_interface.launch.py")

    def params(self, module, config, **overrides):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.yaml"
            path.write_text(yaml.safe_dump(config), encoding="utf-8")
            context = {"config_file": str(path), **overrides}
            return module._launch_setup(context)[0].kwargs["parameters"][0]

    def test_camera_yaml_is_preserved(self):
        expected = dict(full_speed=False, target_fps=25, output_encoding="mono8")
        actual = self.params(self.camera, {"/**": {"ros__parameters": expected}})
        self.assertEqual(actual, expected)

    def test_camera_legacy_key_and_typed_overrides(self):
        actual = self.params(
            self.camera, {"/mv_camera": {"ros__parameters": {"full_speed": True}}},
            full_speed="false", target_fps="30", output_encoding="mono8",
            image_topic="/image_raw", use_sensor_data_qos="true", qos_depth="2")
        self.assertIs(actual["full_speed"], False)
        self.assertIs(actual["use_sensor_data_qos"], True)
        self.assertEqual(actual["target_fps"], 30)
        self.assertEqual(actual["qos_depth"], 2)
        self.assertEqual(actual["image_topic"], "/image_raw")

    def test_interface_yaml_is_preserved(self):
        expected = dict(enable_image_bridge=False, timestamp_mode="px4", timestamp_offset_sec=0.1)
        actual = self.params(self.interface, {"driver_interface": {"ros__parameters": expected}})
        self.assertEqual(actual, expected)

    def test_interface_typed_overrides(self):
        actual = self.params(
            self.interface,
            {"driver_interface": {"ros__parameters": {"enable_image_bridge": True}}},
            enable_image_bridge="false", timestamp_mode="px4",
            timestamp_offset_sec="-0.005", convert_frd_to_flu="false")
        self.assertIs(actual["enable_image_bridge"], False)
        self.assertIs(actual["convert_frd_to_flu"], False)
        self.assertEqual(actual["timestamp_offset_sec"], -0.005)

    def test_combined_yaml_does_not_mix_nodes(self):
        config = {
            "/camera_node": {"ros__parameters": {"output_encoding": "mono8"}},
            "/driver_interface": {"ros__parameters": {"enable_image_bridge": False}},
        }
        self.assertEqual(self.params(self.camera, config), {"output_encoding": "mono8"})
        self.assertEqual(self.params(self.interface, config), {"enable_image_bridge": False})

    def test_invalid_boolean_rejected(self):
        for module in (self.camera, self.interface):
            with self.assertRaises(ValueError):
                module._parse_bool("not-a-bool")

    def test_missing_node_configuration_rejected(self):
        for module in (self.camera, self.interface):
            with self.assertRaises(ValueError):
                self.params(module, {"unrelated": {"ros__parameters": {"foo": 1}}})

    def test_shipped_foxy_presets_preserve_time_modes(self):
        paths = (
            ("src/driver_interface/config/driver_interface.yaml", "auto"),
            ("src/bringup/config/driver_interface.yaml", "px4"),
            ("src/bringup/config/camera_interface.yaml", "px4"),
        )
        for filename, expected_mode in paths:
            config = yaml.safe_load((ROOT / filename).read_text(encoding="utf-8"))
            params = self.params(self.interface, config)
            self.assertEqual(params["timestamp_mode"], expected_mode)
        combined = yaml.safe_load((ROOT / paths[-1][0]).read_text(encoding="utf-8"))
        camera_params = self.params(self.camera, combined)
        self.assertEqual(camera_params["image_topic"], "/image_raw")
        self.assertEqual(camera_params["output_encoding"], "mono8")
        self.assertIs(camera_params["full_speed"], False)

    def test_launch_python_38_syntax(self):
        for package in ("mindvision_camera", "driver_interface", "bringup"):
            for filename in (ROOT / "src" / package / "launch").glob("*.py"):
                ast.parse(filename.read_text(encoding="utf-8"),
                          filename=str(filename), feature_version=8)


if __name__ == "__main__":
    unittest.main()
