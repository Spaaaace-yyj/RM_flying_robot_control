"""Start PX4 serial, D435, the IMU interface and VINS-Fusion from YAML."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _expand_path(value):
    return Path(os.path.expandvars(value)).expanduser()


def _config_path(section, config_dir):
    """Resolve an absolute, package-relative or localization-YAML-relative file."""
    path = _expand_path(section["config_file"])
    if not path.is_absolute():
        package = section.get("config_package")
        base = Path(get_package_share_directory(package)) if package else config_dir
        path = base / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file does not exist: {path}")
    return str(path)


def _include(package, filename, arguments):
    path = Path(get_package_share_directory(package)) / "launch" / filename
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    config_path = _expand_path(LaunchConfiguration("config_file").perform(context)).resolve()
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping: {config_path}")
    for section in ("px4_agent", "driver_interface", "d435", "vins_fusion"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing YAML mapping '{section}': {config_path}")

    agent = config["px4_agent"]
    device, baud = agent["device"], agent["baud"]
    if not isinstance(device, str) or not device.strip():
        raise ValueError("px4_agent.device must be a non-empty string")
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError("px4_agent.baud must be a positive integer")

    driver_config = _config_path(config["driver_interface"], config_path.parent)
    camera_config = _config_path(config["d435"], config_path.parent)
    vins_config = _config_path(config["vins_fusion"], config_path.parent)
    use_sim_time = config["vins_fusion"]["use_sim_time"]
    if not isinstance(use_sim_time, bool):
        raise ValueError("vins_fusion.use_sim_time must be a YAML boolean")

    actions = [
        LogInfo(msg=f"Localization configuration: {config_path}"),
        _include("bringup", "px4_xrce_agent.launch.py", {
            "device": device,
            "baud": str(baud),
        }),
        # Pass file paths only, preserving all settings in the existing YAMLs.
        _include("driver_interface", "driver_interface.launch.py", {
            "config_file": driver_config,
        }),
        _include("d435_bringup", "d435.launch.py", {
            "config_file": camera_config,
        }),
        Node(
            package="vins_fusion_ros2",
            executable="vins_fusion_ros2_node",
            name="vins_fusion_ros2_node",
            output="screen",
            emulate_tty=True,
            parameters=[{"config_file": vins_config, "use_sim_time": use_sim_time}],
        ),
    ]

    # Optional for existing custom localization YAMLs; enabled in the shipped profile.
    visualization = config.get("visualization", {})
    if not isinstance(visualization, dict):
        raise ValueError("visualization must be a YAML mapping")
    enabled = visualization.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("visualization.enabled must be a YAML boolean")
    if enabled:
        marker_config = _config_path(visualization, config_path.parent)
        # The child launch also declares config_file: keep it in its own scope.
        actions.append(GroupAction(scoped=True, actions=[
            _include("drone_visualization", "drone_visualization.launch.py", {
                "config_file": marker_config,
                "use_sim_time": str(use_sim_time).lower(),
            }),
        ]))
    return actions


def generate_launch_description():
    default_config = Path(get_package_share_directory("bringup")) / "config" / "localization.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(default_config),
            description="Localization YAML with serial settings and component configuration paths",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
