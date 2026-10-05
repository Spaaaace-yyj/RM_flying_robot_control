import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value):
    if value.lower() not in ("true", "false"):
        raise ValueError("Boolean launch arguments must be true or false")
    return value.lower() == "true"


def _launch_setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    with open(config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    # Accept the wildcard config and the original /mv_camera config. A plain
    # dict allows explicit CLI overrides to win over node-specific YAML keys.
    params = {}
    for key in ("/**", "mv_camera", "/mv_camera", "camera_node", "/camera_node"):
        if key in config:
            params.update(config[key]["ros__parameters"])
    if not params:
        raise ValueError("No camera ros__parameters found in " + config_file)

    converters = {
        "full_speed": _parse_bool,
        "target_fps": int,
        "output_encoding": str,
        "image_topic": str,
        "use_sensor_data_qos": _parse_bool,
        "qos_depth": int,
    }
    for name, convert in converters.items():
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            params[name] = convert(value)

    return [Node(
        package="mindvision_camera",
        executable="mindvision_camera_node",
        name="camera_node",
        output="screen",
        parameters=[params],
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mindvision_camera"), "config", "camera_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        # Empty means use YAML (do not overwrite it with launch defaults).
        DeclareLaunchArgument("full_speed", default_value=""),
        DeclareLaunchArgument("target_fps", default_value=""),
        DeclareLaunchArgument("output_encoding", default_value=""),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument("use_sensor_data_qos", default_value=""),
        DeclareLaunchArgument("qos_depth", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
