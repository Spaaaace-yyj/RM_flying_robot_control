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

    camera_node = Node(
        package="mindvision_camera",
        executable="mindvision_camera_node",
        name="camera_node",
        output="screen",
        parameters=[params],
    )
    return [camera_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mindvision_camera"), "config", "camera_params.yaml"
    )

    declare_config_file_cmd = DeclareLaunchArgument("config_file", default_value=default_config)
    declare_full_speed_cmd = DeclareLaunchArgument("full_speed", default_value="")
    declare_target_fps_cmd = DeclareLaunchArgument("target_fps", default_value="")
    declare_output_encoding_cmd = DeclareLaunchArgument("output_encoding", default_value="")
    declare_image_topic_cmd = DeclareLaunchArgument("image_topic", default_value="")
    declare_use_sensor_data_qos_cmd = DeclareLaunchArgument("use_sensor_data_qos", default_value="")
    declare_qos_depth_cmd = DeclareLaunchArgument("qos_depth", default_value="")

    launch_setup = OpaqueFunction(function=_launch_setup)

    return LaunchDescription(
        [
            declare_config_file_cmd,
            declare_full_speed_cmd,
            declare_target_fps_cmd,
            declare_output_encoding_cmd,
            declare_image_topic_cmd,
            declare_use_sensor_data_qos_cmd,
            declare_qos_depth_cmd,
            launch_setup,
        ]
    )
