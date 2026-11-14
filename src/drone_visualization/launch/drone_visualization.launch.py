"""Display a simple quadrotor at a nav_msgs/Odometry pose."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory("drone_visualization")) / "config" / "drone_visualization.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(config)),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="drone_visualization",
            executable="odometry_drone_marker",
            name="drone_visualization",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            ],
        ),
    ])
