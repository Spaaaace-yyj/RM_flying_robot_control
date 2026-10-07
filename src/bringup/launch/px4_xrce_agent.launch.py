from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    micro_xrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '--dev', '/dev/ttyTHS0',
            '-b', '921600'
        ],
        output='screen'
    )

    return LaunchDescription([
        micro_xrce_agent
    ])
