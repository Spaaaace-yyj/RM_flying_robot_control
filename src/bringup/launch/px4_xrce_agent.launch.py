from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='921600'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'serial', '--dev', LaunchConfiguration('device'),
                            '-b', LaunchConfiguration('baud')], output='screen'),
    ])
