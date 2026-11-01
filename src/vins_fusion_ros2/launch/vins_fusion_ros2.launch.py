"""Start VINS-Fusion on Foxy with an external calibration or the EuRoC example."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_node(context):
    config_file = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('config_file').perform(context)))
    if not os.path.isfile(config_file):
        raise RuntimeError('VINS config_file does not exist: ' + config_file)
    # Foxy versions differ in ParameterValue support. Pass native Python types.
    clock = LaunchConfiguration('use_sim_time').perform(context).lower()
    if clock not in ('true', 'false', '1', '0'):
        raise RuntimeError('use_sim_time must be true or false')
    return [Node(
        package='vins_fusion_ros2', executable='vins_fusion_ros2_node',
        name='vins_fusion_ros2_node', output='screen', emulate_tty=True,
        parameters=[{
            'config_file': config_file,
            'use_sim_time': clock in ('true', '1'),
            **{name: LaunchConfiguration(name).perform(context) for name in (
                'world_frame_id', 'body_frame_id', 'camera_frame_id')},
        }],
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('vins_fusion_ros2'), 'config',
        'euroc', 'euroc_stereo_imu_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='OpenCV VINS YAML, not a ROS parameter YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world_frame_id', default_value='world'),
        DeclareLaunchArgument('body_frame_id', default_value='body'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        OpaqueFunction(function=launch_node),
    ])
