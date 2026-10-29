"""Start the installed RealSense driver using one YAML configuration file."""

import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_config(path):
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not config:
        raise ValueError(f"Expected a non-empty YAML mapping: {path}")

    # Also accept a single-node ROS parameter dump, or a /** wildcard file.
    blocks = [
        (key, value["ros__parameters"])
        for key, value in config.items()
        if isinstance(value, dict) and "ros__parameters" in value
    ]
    if blocks:
        if len(config) != 1 or len(blocks) != 1:
            raise ValueError("Use a YAML file containing exactly one camera node.")
        node_path, config = blocks[0]
        if not isinstance(config, dict):
            raise ValueError("ros__parameters must contain a mapping.")
        config = dict(config)
        if node_path != "/**":
            parts = node_path.strip("/").split("/")
            config.setdefault("camera_name", parts[-1])
            config.setdefault("camera_namespace", "/".join(parts[:-1]))
    return config


def _preserve_strings(value):
    # Prevent numeric-looking serial numbers and empty strings being inferred
    # as another parameter type by launch_ros. Also handle nested ROS dumps.
    if isinstance(value, dict):
        return {key: _preserve_strings(item) for key, item in value.items()}
    if isinstance(value, str):
        return ParameterValue(value, value_type=str)
    return value


def _launch_camera(context):
    filename = LaunchConfiguration("config_file").perform(context)
    path = Path(os.path.expandvars(filename)).expanduser().resolve()
    config = _read_config(path)

    # These options control the launch action, rather than sensor parameters.
    namespace = config.pop("camera_namespace", "camera")
    log_level = config.pop("log_level", "info")
    output = config.pop("output", "screen")
    name = config.setdefault("camera_name", "camera")
    if not all(isinstance(item, str) for item in (namespace, log_level, output, name)):
        raise ValueError("camera_name, camera_namespace, log_level and output must be strings.")

    return [
        LogInfo(msg=f"D435 configuration: {path}"),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name=name,
            namespace=namespace,
            output=output,
            emulate_tty=True,
            parameters=[_preserve_strings(config)],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description():
    # The same relative layout works from the source and installed package.
    default_config = Path(__file__).resolve().parent.parent / "config" / "d435.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(default_config),
            description="Path to the camera YAML configuration file",
        ),
        OpaqueFunction(function=_launch_camera),
    ])
