"""Run the standalone C++ depth adapter with a configurable parameter file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            FindPackageShare("ego_depth_inputs"), "config", "ego_depth_inputs.yaml"])),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="ego_depth_inputs", executable="ego_depth_inputs_node",
             name="ego_depth_inputs", output="screen", parameters=[
                 LaunchConfiguration("config_file"),
                 {"use_sim_time": ParameterValue(
                     LaunchConfiguration("use_sim_time"), value_type=bool)}]),
    ])
