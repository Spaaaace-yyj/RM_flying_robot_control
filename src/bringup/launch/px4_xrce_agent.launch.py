from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    device = LaunchConfiguration("device")
    baud = LaunchConfiguration("baud")

    declare_device_cmd = DeclareLaunchArgument("device", default_value="/dev/ttyAS4")
    declare_baud_cmd = DeclareLaunchArgument("baud", default_value="1500000")

    px4_agent_process = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", device, "-b", baud], output="screen"
    )

    return LaunchDescription(
        [
            declare_device_cmd,
            declare_baud_cmd,
            px4_agent_process,
        ]
    )
