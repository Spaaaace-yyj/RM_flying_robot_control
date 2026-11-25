"""Start localization, explicit takeoff and gated fixed-goal EGO/PX4 execution."""

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
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(path)),
                launch_arguments=arguments.items(),
            )
        ],
    )


def _launch_setup(context):
    path = _expand_path(LaunchConfiguration("config_file").perform(context)).resolve()
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    for section in ("localization", "control"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing YAML mapping '{section}': {path}")
    localization_config = _config_path(config["localization"], path.parent)
    control_config = _config_path(config["control"], path.parent)
    use_sim_time = config["control"]["use_sim_time"]
    if not isinstance(use_sim_time, bool):
        raise ValueError("control.use_sim_time must be a YAML boolean")
    log_config = LogInfo(msg=f"Onboard control configuration: {path}")
    localization_launch = _include(
        "bringup",
        "localization.launch.py",
        {
            "config_file": localization_config,
        },
    )
    control_launch = _include(
        "px4_control",
        "px4_control.launch.py",
        {
            "config_file": control_config,
            "use_sim_time": str(use_sim_time).lower(),
        },
    )
    # Read the matching D435 profile so image pixels and the calibrated optical
    # frame use the same rotation. Nested mappings also occur in ros2 param dumps.
    with open(localization_config, encoding="utf-8") as stream:
        localization = yaml.safe_load(stream)
    camera_path = _config_path(localization["d435"], Path(localization_config).parent)
    with open(camera_path, encoding="utf-8") as stream:
        camera = yaml.safe_load(stream)
    for entry in camera.values():
        if isinstance(entry, dict) and "ros__parameters" in entry:
            camera = entry["ros__parameters"]
            break
    rotation = camera.get("rotation_filter", {})
    enabled = camera.get("rotation_filter.enable", rotation.get("enable", False))
    angle = camera.get("rotation_filter.rotation", rotation.get("rotation", 0)) if enabled else 0
    if angle not in (0, 180, -180):
        raise ValueError("Depth preview supports unrotated or 180-degree D435 calibration")
    preview_launch = _include(
        "bringup", "ego_preview.launch.py", {
            "use_sim_time": str(use_sim_time).lower(),
            "images_rotated_180": str(abs(angle) == 180).lower(),
            "goal_topic": "/ego_execution/goal",
            "planner_config_file": str(Path(get_package_share_directory("bringup")) / "config" / "ego_execution.yaml"),
        },
    )
    return [log_config, localization_launch, control_launch, preview_launch]


def generate_launch_description():
    default_config = Path(get_package_share_directory("bringup")) / "config" / "onboard_control.yaml"

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=str(default_config),
        description="YAML selecting localization and PX4 control configurations",
    )

    launch_setup = OpaqueFunction(function=_launch_setup)

    return LaunchDescription(
        [
            declare_config_file_cmd,
            launch_setup,
        ]
    )

