from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vins_config_file = LaunchConfiguration("vins_config_file")
    hover_config_file = LaunchConfiguration("hover_config_file")
    sensor_config_file = LaunchConfiguration("sensor_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    start_agent = LaunchConfiguration("start_agent")
    agent_device = LaunchConfiguration("agent_device")
    agent_baud = LaunchConfiguration("agent_baud")

    declare_vins_config_file_cmd = DeclareLaunchArgument(
        "vins_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("vins_estimator"),
                "config",
                "euroc",
                "euroc_config_px4.yaml",
            ]
        ),
        description="实机标定的 VINS OpenCV YAML 绝对路径（必填）",
    )
    declare_hover_config_file_cmd = DeclareLaunchArgument(
        "hover_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("px4_hover"),
                "config",
                "px4_hover.yaml",
            ]
        ),
        description="px4_hover 管理器参数文件",
    )
    declare_sensor_config_file_cmd = DeclareLaunchArgument(
        "sensor_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("bringup"),
                "config",
                "camera_interface.yaml",
            ]
        ),
        description="相机与 driver_interface 共用的参数文件",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="是否启动 RViz2",
    )
    declare_start_agent_cmd = DeclareLaunchArgument(
        "start_agent",
        default_value="true",
        description="是否启动 MicroXRCEAgent",
    )
    declare_agent_device_cmd = DeclareLaunchArgument(
        "agent_device",
        default_value="/dev/ttyUSB0",
        description="MicroXRCEAgent 串口设备",
    )
    declare_agent_baud_cmd = DeclareLaunchArgument(
        "agent_baud",
        default_value="921600",
        description="MicroXRCEAgent 串口波特率",
    )

    vins_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bringup"),
                    "launch",
                    "vins_system.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vins_config_file": vins_config_file,
            "camera_config_file": sensor_config_file,
            "driver_config_file": sensor_config_file,
            "use_sim_time": "false",
            "use_pose_graph": "false",
            "use_rviz": use_rviz,
            "start_agent": start_agent,
            "agent_device": agent_device,
            "agent_baud": agent_baud,
        }.items(),
    )

    px4_hover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("px4_hover"),
                    "launch",
                    "px4_hover.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": hover_config_file,
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_vins_config_file_cmd,
            declare_hover_config_file_cmd,
            declare_sensor_config_file_cmd,
            declare_use_rviz_cmd,
            declare_start_agent_cmd,
            declare_agent_device_cmd,
            declare_agent_baud_cmd,
            vins_system_launch,
            px4_hover_launch,
        ]
    )
