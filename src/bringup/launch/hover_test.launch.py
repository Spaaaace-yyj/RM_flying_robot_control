"""Start D435/PX4 VINS-Fusion localization and the explicit hover manager."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _expand_path(value):
    return Path(os.path.expandvars(value)).expanduser()


def _config_path(section, config_dir):
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
    # Both child launches use config_file: keep their launch configurations scoped.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )])


def _launch_setup(context):
    path = _expand_path(LaunchConfiguration("config_file").perform(context)).resolve()
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    for section in ("localization", "hover"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing YAML mapping '{section}': {path}")
    localization_config = _config_path(config["localization"], path.parent)
    hover_config = _config_path(config["hover"], path.parent)
    use_sim_time = config["hover"]["use_sim_time"]
    if not isinstance(use_sim_time, bool):
        raise ValueError("hover.use_sim_time must be a YAML boolean")
    return [
        LogInfo(msg=f"Hover test configuration: {path}"),
        _include("bringup", "localization.launch.py", {
            "config_file": localization_config,
        }),
        _include("px4_hover", "px4_hover.launch.py", {
            "config_file": hover_config,
            "use_sim_time": str(use_sim_time).lower(),
        }),
    ]


def generate_launch_description():
    default_config = Path(get_package_share_directory("bringup")) / "config" / "hover_test.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(default_config),
            description="YAML selecting localization and VINS-Fusion hover configurations",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
