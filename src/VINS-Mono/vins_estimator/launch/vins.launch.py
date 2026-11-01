import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vins_estimator')

    config_file = LaunchConfiguration('config_file')
    vins_folder = LaunchConfiguration('vins_folder')
    use_rviz = LaunchConfiguration('use_rviz')
    use_pose_graph = LaunchConfiguration('use_pose_graph')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_dir, 'config', 'euroc', 'euroc_config.yaml'),
            description='Absolute path to the VINS config YAML file'),
        DeclareLaunchArgument(
            'vins_folder',
            default_value=pkg_dir + '/',
            description='VINS root folder used to locate support files (e.g. fisheye mask)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),
        DeclareLaunchArgument(
            'use_pose_graph',
            default_value='true',
            description='Launch the loop-closure pose graph node'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock (enable this when playing a bag with --clock)'),

        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'vins_folder': vins_folder,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='pose_graph',
            executable='pose_graph',
            name='pose_graph',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'visualization_shift_x': 0,
                'visualization_shift_y': 0,
                'skip_cnt': 0,
                'skip_dis': 0.0,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(use_pose_graph),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'vins_rviz2_config.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
