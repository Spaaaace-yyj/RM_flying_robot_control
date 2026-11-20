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

    params = {}
    for key in ("/**", "driver_interface", "/driver_interface"):
        if key in config:
            params.update(config[key]["ros__parameters"])
    if not params:
        raise ValueError("No driver_interface ros__parameters found in " + config_file)

    converters = {
        "enable_image_bridge": _parse_bool,
        "camera_input_topic": str,
        "gray_output_topic": str,
        "px4_imu_input_topic": str,
        "imu_output_topic": str,
        "timestamp_mode": str,
        "timestamp_offset_sec": float,
        "convert_frd_to_flu": _parse_bool,
    }
    for name, convert in converters.items():
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            params[name] = convert(value)

    driver_interface_node = Node(
        package="driver_interface",
        executable="driver_interface_node",
        name="driver_interface",
        output="screen",
        parameters=[params],
    )
    return [driver_interface_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("driver_interface"), "config", "driver_interface.yaml"
    )

    declare_config_file_cmd = DeclareLaunchArgument("config_file", default_value=default_config)
    declare_enable_image_bridge_cmd = DeclareLaunchArgument("enable_image_bridge", default_value="")
    declare_camera_input_topic_cmd = DeclareLaunchArgument("camera_input_topic", default_value="")
    declare_gray_output_topic_cmd = DeclareLaunchArgument("gray_output_topic", default_value="")
    declare_px4_imu_input_topic_cmd = DeclareLaunchArgument("px4_imu_input_topic", default_value="")
    declare_imu_output_topic_cmd = DeclareLaunchArgument("imu_output_topic", default_value="")
    declare_timestamp_mode_cmd = DeclareLaunchArgument("timestamp_mode", default_value="")
    declare_timestamp_offset_sec_cmd = DeclareLaunchArgument(
        "timestamp_offset_sec", default_value=""
    )
    declare_convert_frd_to_flu_cmd = DeclareLaunchArgument("convert_frd_to_flu", default_value="")

    launch_setup = OpaqueFunction(function=_launch_setup)

    return LaunchDescription(
        [
            declare_config_file_cmd,
            declare_enable_image_bridge_cmd,
            declare_camera_input_topic_cmd,
            declare_gray_output_topic_cmd,
            declare_px4_imu_input_topic_cmd,
            declare_imu_output_topic_cmd,
            declare_timestamp_mode_cmd,
            declare_timestamp_offset_sec_cmd,
            declare_convert_frd_to_flu_cmd,
            launch_setup,
        ]
    )
