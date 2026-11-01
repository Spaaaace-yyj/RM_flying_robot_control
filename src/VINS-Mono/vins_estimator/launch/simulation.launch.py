import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vins_estimator')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_dir, 'config', 'simulation', 'simulation_config.yaml'),
            description='Absolute path to the simulation VINS config'),
        Node(
            package='data_generator',
            executable='data_generator',
            name='data_generator',
            output='screen'),
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            parameters=[{'config_file': config_file}]),
    ])
