from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([FindPackageShare('px4_hover'), 'config', 'px4_hover.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='px4_hover',
            executable='px4_hover_node',
            name='px4_hover',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type = bool)}
            ]),
    ])
