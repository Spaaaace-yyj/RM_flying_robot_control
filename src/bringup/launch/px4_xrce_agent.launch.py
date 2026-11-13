from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyAS4'),
        DeclareLaunchArgument('baud', default_value='1500000'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'serial', '--dev', LaunchConfiguration('device'),
                            '-b', LaunchConfiguration('baud')], output='screen'),
    ])
