from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vins_config_file = LaunchConfiguration("vins_config_file")
    use_pose_graph = LaunchConfiguration("use_pose_graph")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    driver_config_file = LaunchConfiguration("driver_config_file")
    camera_config_file = LaunchConfiguration("camera_config_file")

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
        description="VINS 配置文件绝对路径",
    )
    declare_use_pose_graph_cmd = DeclareLaunchArgument(
        "use_pose_graph",
        default_value="true",
        description="是否启动 VINS 回环检测",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz2",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="实时传感器设为 false，播放 rosbag 设为 true",
    )
    declare_driver_config_file_cmd = DeclareLaunchArgument(
        "driver_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("bringup"),
                "config",
                "driver_interface.yaml",
            ]
        ),
        description="driver_interface 参数文件（bringup 包内副本）",
    )
    declare_camera_config_file_cmd = DeclareLaunchArgument(
        "camera_config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("mindvision_camera"),
                "config",
                "camera_params.yaml",
            ]
        ),
        description="相机参数文件；也可与 driver_config_file 指向同一多节点 YAML",
    )
    declare_camera_full_speed_cmd = DeclareLaunchArgument("camera_full_speed", default_value="")
    declare_camera_target_fps_cmd = DeclareLaunchArgument("camera_target_fps", default_value="")
    declare_camera_output_encoding_cmd = DeclareLaunchArgument(
        "camera_output_encoding", default_value=""
    )
    declare_camera_image_topic_cmd = DeclareLaunchArgument("camera_image_topic", default_value="")
    declare_camera_use_sensor_data_qos_cmd = DeclareLaunchArgument(
        "camera_use_sensor_data_qos", default_value=""
    )
    declare_camera_qos_depth_cmd = DeclareLaunchArgument("camera_qos_depth", default_value="")
    declare_enable_image_bridge_cmd = DeclareLaunchArgument("enable_image_bridge", default_value="")
    declare_start_agent_cmd = DeclareLaunchArgument("start_agent", default_value="true")
    declare_agent_device_cmd = DeclareLaunchArgument("agent_device", default_value="/dev/ttyUSB0")
    declare_agent_baud_cmd = DeclareLaunchArgument("agent_baud", default_value="921600")

    mindvision_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mindvision_camera"),
                    "launch",
                    "mv_launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": camera_config_file,
            "full_speed": LaunchConfiguration("camera_full_speed"),
            "target_fps": LaunchConfiguration("camera_target_fps"),
            "output_encoding": LaunchConfiguration("camera_output_encoding"),
            "image_topic": LaunchConfiguration("camera_image_topic"),
            "use_sensor_data_qos": LaunchConfiguration("camera_use_sensor_data_qos"),
            "qos_depth": LaunchConfiguration("camera_qos_depth"),
        }.items(),
    )

    px4_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bringup"),
                    "launch",
                    "px4_xrce_agent.launch.py",
                ]
            )
        ),
        launch_arguments={
            "device": LaunchConfiguration("agent_device"),
            "baud": LaunchConfiguration("agent_baud"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_agent")),
    )

    driver_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("driver_interface"),
                    "launch",
                    "driver_interface.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": driver_config_file,
            "enable_image_bridge": LaunchConfiguration("enable_image_bridge"),
        }.items(),
    )

    vins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vins_estimator"),
                    "launch",
                    "vins.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": vins_config_file,
            "use_pose_graph": use_pose_graph,
            "use_rviz": use_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_vins_config_file_cmd,
            declare_use_pose_graph_cmd,
            declare_use_rviz_cmd,
            declare_use_sim_time_cmd,
            declare_driver_config_file_cmd,
            declare_camera_config_file_cmd,
            declare_camera_full_speed_cmd,
            declare_camera_target_fps_cmd,
            declare_camera_output_encoding_cmd,
            declare_camera_image_topic_cmd,
            declare_camera_use_sensor_data_qos_cmd,
            declare_camera_qos_depth_cmd,
            declare_enable_image_bridge_cmd,
            declare_start_agent_cmd,
            declare_agent_device_cmd,
            declare_agent_baud_cmd,
            mindvision_camera_launch,
            px4_system_launch,
            driver_interface_launch,
            vins_launch,
        ]
    )
