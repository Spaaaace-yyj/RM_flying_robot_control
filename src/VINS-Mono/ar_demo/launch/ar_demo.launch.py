import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vins_pkg_dir = get_package_share_directory('vins_estimator')

    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(vins_pkg_dir, 'config', 'euroc', 'euroc_config.yaml'),
            description='Absolute path to the VINS config YAML (used as camera calibration file)'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/cam0/image_raw',
            description='Raw image topic to draw AR objects on'),

        Node(
            package='ar_demo',
            executable='ar_demo_node',
            name='ar_demo',
            output='screen',
            parameters=[{
                'use_undistored_img': False,
                'calib_file': config_file,
            }],
            remappings=[
                ('camera_pose', '/vins_estimator/camera_pose'),
                ('pointcloud', '/vins_estimator/point_cloud'),
                ('image_raw', image_topic),
            ],
        ),
    ])
