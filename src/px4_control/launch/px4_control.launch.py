from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("px4_control"), "config", "px4_control.yaml"]
        ),
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument("use_sim_time", default_value="false")

    px4_control_node = Node(
        package="px4_control",
        executable="px4_control_node",
        name="px4_control",
        output="screen",
        parameters=[config_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(
        [
            declare_config_file_cmd,
            declare_use_sim_time_cmd,
            px4_control_node,
        ]
    )
