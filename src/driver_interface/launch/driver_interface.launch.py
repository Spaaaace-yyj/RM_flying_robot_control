from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    package_share = get_package_share_directory('driver_interface')
    default_config = os.path.join(
        package_share, 'config', 'driver_interface.yaml')

    config_file = LaunchConfiguration('config_file')
    camera_input_topic = LaunchConfiguration('camera_input_topic')
    gray_output_topic = LaunchConfiguration('gray_output_topic')
    px4_imu_input_topic = LaunchConfiguration('px4_imu_input_topic')
    imu_output_topic = LaunchConfiguration('imu_output_topic')
    timestamp_mode = LaunchConfiguration('timestamp_mode')
    timestamp_offset_sec = LaunchConfiguration('timestamp_offset_sec')
    convert_frd_to_flu = LaunchConfiguration('convert_frd_to_flu')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to driver_interface parameter YAML'),
        DeclareLaunchArgument(
            'camera_input_topic', default_value='/image_raw'),
        DeclareLaunchArgument(
            'gray_output_topic', default_value='/image_gray'),
        DeclareLaunchArgument(
            'px4_imu_input_topic', default_value='/fmu/out/sensor_combined'),
        DeclareLaunchArgument(
            'imu_output_topic', default_value='/px4/imu'),
        DeclareLaunchArgument(
            'timestamp_mode', default_value='auto'),
        DeclareLaunchArgument(
            'timestamp_offset_sec', default_value='0.0'),
        DeclareLaunchArgument(
            'convert_frd_to_flu', default_value='true'),

        Node(
            package='driver_interface',
            executable='driver_interface_node',
            name='driver_interface',
            output='screen',
            parameters=[
                config_file,
                {
                    'camera_input_topic': camera_input_topic,
                    'gray_output_topic': gray_output_topic,
                    'px4_imu_input_topic': px4_imu_input_topic,
                    'imu_output_topic': imu_output_topic,
                    'timestamp_mode': timestamp_mode,
                    'timestamp_offset_sec': ParameterValue(
                        timestamp_offset_sec, value_type=float),
                    'convert_frd_to_flu': ParameterValue(
                        convert_frd_to_flu, value_type=bool),
                },
            ],
        ),
    ])
